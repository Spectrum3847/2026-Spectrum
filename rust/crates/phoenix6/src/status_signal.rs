//! Status signals (`StatusSignal` and the static helpers of `BaseStatusSignal`).
//!
//! A signal holds the last value read from one device and when it was sampled. One value kind
//! serves every signal: the raw `f64` the C layer reports, in the units each device getter
//! documents. Read many signals with one call per bus through [`refresh_all`] and
//! [`wait_for_all`].

use crate::device::{DeviceIdentifier, FirmwareChecker};
use crate::error_report::report_status_code;
use crate::native::{NetworkSignal, phoenix};
use crate::status_code::StatusCode;
use crate::utils;
use std::ffi::CStr;
use std::sync::Arc;

/// `Context` enum of the C layer: the public API caller.
const CONTEXT_API: i32 = 0;

/// Java's default `setUpdateFrequency` timeout.
const DEFAULT_TIMEOUT_SECONDS: f64 = 0.100;

/// Java's default `getLatencyCompensatedValue` cap.
pub const DEFAULT_MAX_LATENCY_SECONDS: f64 = 0.300;

/// Which clock produced a timestamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimestampSource {
    /// The roboRIO system clock, when the message arrived.
    System = 0,
    /// The CANivore clock.
    CanIvore = 1,
    /// The device's own clock.
    Device = 2,
}

/// When a value was sampled, on Phoenix's clock (see [`utils::current_time_seconds`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timestamp {
    time: f64,
    source: TimestampSource,
    valid: bool,
}

impl Default for Timestamp {
    fn default() -> Self {
        Self {
            time: 0.0,
            source: TimestampSource::System,
            valid: false,
        }
    }
}

impl Timestamp {
    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn source(&self) -> TimestampSource {
        self.source
    }

    pub fn is_valid(&self) -> bool {
        self.valid
    }

    /// Seconds since the sample, on Phoenix's clock.
    pub fn latency(&self) -> f64 {
        utils::current_time_seconds() - self.time
    }
}

/// The three timestamps of one sample.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AllTimestamps {
    system: Timestamp,
    canivore: Timestamp,
    device: Timestamp,
}

impl AllTimestamps {
    /// The device timestamp when valid, else the CANivore one, else the system one.
    pub fn best(&self) -> Timestamp {
        if self.device.valid {
            self.device
        } else if self.canivore.valid {
            self.canivore
        } else {
            self.system
        }
    }

    pub fn system(&self) -> Timestamp {
        self.system
    }

    pub fn canivore(&self) -> Timestamp {
        self.canivore
    }

    pub fn device(&self) -> Timestamp {
        self.device
    }
}

/// One signal of one device.
#[derive(Debug, Clone)]
pub struct StatusSignal {
    id: Arc<DeviceIdentifier>,
    firmware: Option<Arc<FirmwareChecker>>,
    spn: u32,
    name: &'static str,
    value: f64,
    timestamps: AllTimestamps,
    status: StatusCode,
    last_timestamp: f64,
}

impl AsMut<StatusSignal> for StatusSignal {
    fn as_mut(&mut self) -> &mut StatusSignal {
        self
    }
}

impl StatusSignal {
    pub(crate) fn new(
        id: Arc<DeviceIdentifier>,
        firmware: Option<Arc<FirmwareChecker>>,
        spn: i32,
        name: &'static str,
    ) -> Self {
        Self {
            id,
            firmware,
            spn: spn as u32,
            name,
            value: 0.0,
            timestamps: AllTimestamps::default(),
            status: StatusCode::StatusCodeNotInitialized,
            last_timestamp: 0.0,
        }
    }

    pub fn name(&self) -> &'static str {
        self.name
    }

    /// The last value, in the units of the getter that made this signal.
    pub fn value(&self) -> f64 {
        self.value
    }

    /// The value truncated to an integer, for enum and counter signals.
    pub fn int_value(&self) -> i32 {
        self.value as i32
    }

    /// True when the value is not zero, for boolean signals.
    pub fn bool_value(&self) -> bool {
        self.value != 0.0
    }

    pub fn status(&self) -> StatusCode {
        self.status
    }

    pub fn all_timestamps(&self) -> &AllTimestamps {
        &self.timestamps
    }

    /// The best timestamp of the last sample.
    pub fn timestamp(&self) -> Timestamp {
        self.timestamps.best()
    }

    /// True once per new sample, judged by the system timestamp.
    pub fn has_updated(&mut self) -> bool {
        let stamp = self.timestamps.system;
        if stamp.valid && self.last_timestamp != stamp.time {
            self.last_timestamp = stamp.time;
            return true;
        }
        false
    }

    pub fn is_near(&self, target: f64, tolerance: f64) -> bool {
        (self.value - target).abs() <= tolerance
    }

    /// Reads the newest value without blocking and reports a failure to the driver station.
    pub fn refresh(&mut self) -> &mut Self {
        self.refresh_with(true)
    }

    pub fn refresh_with(&mut self, report_error: bool) -> &mut Self {
        self.refresh_value(false, 0.0, report_error);
        self
    }

    /// Blocks until a new value arrives or the timeout passes.
    pub fn wait_for_update(&mut self, timeout_seconds: f64) -> &mut Self {
        self.wait_for_update_with(timeout_seconds, true)
    }

    pub fn wait_for_update_with(&mut self, timeout_seconds: f64, report_error: bool) -> &mut Self {
        self.refresh_value(true, timeout_seconds, report_error);
        self
    }

    /// Asks the device to send this signal at `frequency_hz`. Zero stops it.
    pub fn set_update_frequency(&self, frequency_hz: f64) -> StatusCode {
        self.set_update_frequency_with_timeout(frequency_hz, DEFAULT_TIMEOUT_SECONDS)
    }

    pub fn set_update_frequency_with_timeout(
        &self,
        frequency_hz: f64,
        timeout_seconds: f64,
    ) -> StatusCode {
        // SAFETY: the network is NUL terminated and outlives the call.
        let status = StatusCode(unsafe {
            (phoenix().c_ctre_phoenix6_SetUpdateFrequency)(
                CONTEXT_API,
                self.id.network_c().as_ptr(),
                self.id.device_hash(),
                self.spn as u16,
                frequency_hz,
                timeout_seconds,
            )
        });
        if !status.is_ok() {
            let location = format!("{} setUpdateFrequency {}", self.id.description(), self.name);
            report_status_code(status, &location);
        }
        status
    }

    /// The rate the device currently sends this signal at.
    pub fn applied_update_frequency(&self) -> f64 {
        // SAFETY: the network is NUL terminated and outlives the call.
        unsafe {
            (phoenix().c_ctre_phoenix6_GetUpdateFrequency)(
                self.id.network_c().as_ptr(),
                self.id.device_hash(),
                self.spn as u16,
            )
        }
    }

    fn refresh_value(&mut self, wait: bool, timeout_seconds: f64, report_error: bool) {
        if let Some(firmware) = &self.firmware {
            firmware.report_if_too_old();
        }
        let reading = read_signals(
            self.id.network_c(),
            wait,
            timeout_seconds,
            &[self.id.device_hash()],
            &[self.spn],
        );
        self.apply_reading(reading.status, &reading, 0);
        if report_error && !self.status.is_ok() {
            let location = format!("{} Status Signal {}", self.id.description(), self.name);
            report_status_code(self.status, &location);
        }
    }

    fn apply_reading(&mut self, status: StatusCode, reading: &Readings, index: usize) {
        self.status = status;
        self.value = reading.values[index];
        self.timestamps = AllTimestamps {
            system: Timestamp {
                time: reading.system[index],
                source: TimestampSource::System,
                valid: true,
            },
            canivore: Timestamp {
                time: reading.canivore[index],
                source: TimestampSource::CanIvore,
                valid: true,
            },
            device: Timestamp {
                time: reading.device[index],
                source: TimestampSource::Device,
                valid: reading.device[index] != 0.0,
            },
        };
    }

    /// A signal with fixed contents, for tests that must not touch the natives.
    #[cfg(test)]
    pub(crate) fn from_parts(
        value: f64,
        status: StatusCode,
        system: f64,
        canivore: f64,
        device: f64,
    ) -> Self {
        let id = Arc::new(DeviceIdentifier::for_test("talon fx", 1, "rio"));
        let mut signal = Self::new(id, None, 0, "Test");
        let reading = Readings {
            status,
            values: vec![value],
            canivore: vec![canivore],
            system: vec![system],
            device: vec![device],
        };
        signal.apply_reading(status, &reading, 0);
        signal
    }
}

/// The output arrays of one `c_ctre_phoenix6_get_signal_simplified` call.
struct Readings {
    status: StatusCode,
    values: Vec<f64>,
    canivore: Vec<f64>,
    system: Vec<f64>,
    device: Vec<f64>,
}

fn read_signals(
    network: &CStr,
    wait: bool,
    timeout_seconds: f64,
    device_hashes: &[u32],
    spns: &[u32],
) -> Readings {
    let count = device_hashes.len();
    let mut readings = Readings {
        status: StatusCode::OK,
        values: vec![0.0; count],
        canivore: vec![0.0; count],
        system: vec![0.0; count],
        device: vec![0.0; count],
    };
    // SAFETY: every array holds `count` elements and the network is NUL terminated.
    let status = unsafe {
        (phoenix().c_ctre_phoenix6_get_signal_simplified)(
            network.as_ptr(),
            i32::from(wait),
            timeout_seconds,
            count as u32,
            device_hashes.as_ptr(),
            spns.as_ptr(),
            readings.values.as_mut_ptr(),
            readings.canivore.as_mut_ptr(),
            readings.system.as_mut_ptr(),
            readings.device.as_mut_ptr(),
        )
    };
    readings.status = StatusCode(status);
    readings
}

/// Refreshes every signal with one call to the C layer, without blocking. All signals must be
/// on the same bus, else every signal gets `InvalidNetwork`. The C layer returns one status for
/// the call, so every signal takes that status.
pub fn refresh_all<S: AsMut<StatusSignal>>(signals: &mut [S]) -> StatusCode {
    refresh_all_with(true, signals)
}

pub fn refresh_all_with<S: AsMut<StatusSignal>>(
    report_error: bool,
    signals: &mut [S],
) -> StatusCode {
    wait_for_all_impl(
        "ctre.phoenix6.BaseStatusSignal.refreshAll",
        false,
        0.0,
        report_error,
        signals,
    )
}

/// Blocks until every signal has a new value or the timeout passes. Same rules as [`refresh_all`].
pub fn wait_for_all<S: AsMut<StatusSignal>>(timeout_seconds: f64, signals: &mut [S]) -> StatusCode {
    wait_for_all_with(timeout_seconds, true, signals)
}

pub fn wait_for_all_with<S: AsMut<StatusSignal>>(
    timeout_seconds: f64,
    report_error: bool,
    signals: &mut [S],
) -> StatusCode {
    wait_for_all_impl(
        "ctre.phoenix6.BaseStatusSignal.waitForAll",
        true,
        timeout_seconds,
        report_error,
        signals,
    )
}

fn wait_for_all_impl<S: AsMut<StatusSignal>>(
    location: &str,
    wait: bool,
    timeout_seconds: f64,
    report_error: bool,
    signals: &mut [S],
) -> StatusCode {
    let Some(first) = signals.first_mut() else {
        return StatusCode::OK;
    };
    let network = first.as_mut().id.network().clone();
    if signals
        .iter_mut()
        .any(|signal| *signal.as_mut().id.network() != network)
    {
        for signal in signals.iter_mut() {
            signal.as_mut().status = StatusCode::InvalidNetwork;
        }
        report_status_code(StatusCode::InvalidNetwork, location);
        return StatusCode::InvalidNetwork;
    }
    let mut hashes = Vec::with_capacity(signals.len());
    let mut spns = Vec::with_capacity(signals.len());
    for signal in signals.iter_mut() {
        let signal = signal.as_mut();
        if let Some(firmware) = &signal.firmware {
            firmware.report_if_too_old();
        }
        hashes.push(signal.id.device_hash());
        spns.push(signal.spn);
    }
    let readings = read_signals(
        signals[0].as_mut().id.network_c(),
        wait,
        timeout_seconds,
        &hashes,
        &spns,
    );
    let overall = readings.status;
    for (index, signal) in signals.iter_mut().enumerate() {
        signal.as_mut().apply_reading(overall, &readings, index);
    }
    if report_error && !overall.is_ok() {
        for signal in signals.iter_mut() {
            let signal = signal.as_mut();
            if signal.timestamps.system.time == 0.0 {
                let place = format!("{} Status Signal {}", signal.id.description(), signal.name);
                report_status_code(overall, &place);
            }
        }
        report_status_code(overall, location);
    }
    overall
}

/// Asks devices to send every signal at `frequency_hz`, in one call. The signals may be on
/// different buses.
pub fn set_update_frequency_for_all<S: AsRef<StatusSignal>>(
    frequency_hz: f64,
    signals: &[S],
) -> StatusCode {
    if signals.is_empty() {
        return StatusCode::OK;
    }
    let list: Vec<NetworkSignal> = signals
        .iter()
        .map(|signal| {
            let signal = signal.as_ref();
            NetworkSignal {
                network: signal.id.network_c().as_ptr(),
                device_hash: signal.id.device_hash(),
                spn: signal.spn,
            }
        })
        .collect();
    // SAFETY: `list` holds `len` entries whose network pointers stay valid while `signals`
    // is borrowed.
    let status = StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_SetUpdateFrequencyForAll)(
            CONTEXT_API,
            list.as_ptr(),
            list.len(),
            frequency_hz,
            DEFAULT_TIMEOUT_SECONDS,
        )
    });
    if !status.is_ok() {
        report_status_code(
            status,
            "ctre.phoenix6.BaseStatusSignal.setUpdateFrequencyForAll",
        );
    }
    status
}

impl AsRef<StatusSignal> for StatusSignal {
    fn as_ref(&self) -> &StatusSignal {
        self
    }
}

/// True when every signal's last read succeeded.
pub fn is_all_good<S: AsRef<StatusSignal>>(signals: &[S]) -> bool {
    signals.iter().all(|signal| signal.as_ref().status.is_ok())
}

/// `signal` plus `slope` times the time since the sample, with the time capped at 0.300 s.
/// Use a position signal with its velocity signal.
pub fn latency_compensated_value(signal: &StatusSignal, slope: &StatusSignal) -> f64 {
    latency_compensated_value_with_max(signal, slope, DEFAULT_MAX_LATENCY_SECONDS)
}

/// As [`latency_compensated_value`] with your own cap. A cap of zero or less turns it off.
pub fn latency_compensated_value_with_max(
    signal: &StatusSignal,
    slope: &StatusSignal,
    max_latency_seconds: f64,
) -> f64 {
    compensate(
        signal.value,
        slope.value,
        signal.timestamp().latency(),
        max_latency_seconds,
    )
}

fn compensate(value: f64, slope: f64, latency_seconds: f64, max_latency_seconds: f64) -> f64 {
    let latency = if max_latency_seconds > 0.0 && latency_seconds > max_latency_seconds {
        max_latency_seconds
    } else {
        latency_seconds
    };
    value + slope * latency
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compensation_adds_slope_times_latency() {
        assert_eq!(compensate(2.0, 10.0, 0.01, 0.3), 2.1);
        assert_eq!(compensate(-1.0, -4.0, 0.05, 0.3), -1.2);
    }

    #[test]
    fn compensation_caps_latency() {
        assert_eq!(compensate(1.0, 2.0, 5.0, 0.3), 1.6);
        assert_eq!(compensate(1.0, 2.0, 0.3, 0.3), 1.6);
    }

    #[test]
    fn compensation_cap_of_zero_or_less_is_off() {
        assert_eq!(compensate(1.0, 2.0, 5.0, 0.0), 11.0);
        assert_eq!(compensate(1.0, 2.0, 5.0, -1.0), 11.0);
    }

    #[test]
    fn best_timestamp_prefers_device_then_canivore_then_system() {
        let signal = StatusSignal::from_parts(0.0, StatusCode::OK, 3.0, 2.0, 1.0);
        let best = signal.timestamp();
        assert_eq!(best.time(), 1.0);
        assert_eq!(best.source(), TimestampSource::Device);

        let no_device = StatusSignal::from_parts(0.0, StatusCode::OK, 3.0, 2.0, 0.0);
        assert_eq!(no_device.timestamp().time(), 2.0);
        assert_eq!(no_device.timestamp().source(), TimestampSource::CanIvore);
        assert!(!no_device.all_timestamps().device().is_valid());
    }

    #[test]
    fn has_updated_reports_each_new_system_timestamp_once() {
        let mut signal = StatusSignal::from_parts(0.0, StatusCode::OK, 5.0, 5.0, 5.0);
        assert!(signal.has_updated());
        assert!(!signal.has_updated());
        let reading = Readings {
            status: StatusCode::OK,
            values: vec![0.0],
            canivore: vec![6.0],
            system: vec![6.0],
            device: vec![6.0],
        };
        signal.apply_reading(StatusCode::OK, &reading, 0);
        assert!(signal.has_updated());
    }

    #[test]
    fn value_views() {
        let signal = StatusSignal::from_parts(2.9, StatusCode::OK, 1.0, 1.0, 1.0);
        assert_eq!(signal.value(), 2.9);
        assert_eq!(signal.int_value(), 2);
        assert!(signal.bool_value());
        assert!(signal.is_near(3.0, 0.2));
        assert!(!signal.is_near(3.0, 0.05));
        let zero = StatusSignal::from_parts(0.0, StatusCode::OK, 1.0, 1.0, 1.0);
        assert!(!zero.bool_value());
    }

    #[test]
    fn all_good_needs_every_status_ok() {
        let good = StatusSignal::from_parts(0.0, StatusCode::OK, 1.0, 1.0, 1.0);
        let bad = StatusSignal::from_parts(0.0, StatusCode::RxTimeout, 0.0, 0.0, 0.0);
        assert!(is_all_good(&[good.clone(), good.clone()]));
        assert!(!is_all_good(&[good, bad]));
        assert!(is_all_good::<StatusSignal>(&[]));
    }
}
