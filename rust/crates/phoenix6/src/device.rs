//! The part every device shares (`ParentDevice`): identity, firmware check, control requests,
//! connection test and bus optimization.

use crate::auto_feed;
use crate::can_bus::CanBus;
use crate::configurator::Configurator;
use crate::controls::{ControlRequest, EmptyControl};
use crate::error_report::{cstring, report_status_code};
use crate::native::phoenix;
use crate::sim::{self, DeviceType};
use crate::spn;
use crate::status_code::StatusCode;
use crate::status_signal::StatusSignal;
use crate::unmanaged;
use crate::utils;
use std::ffi::{CStr, CString};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

const CONTEXT_API: i32 = 0;

/// Java's default `optimizeBusUtilization` rate and timeout.
const OPTIMIZED_FREQUENCY_HZ: f64 = 4.0;
const DEFAULT_TIMEOUT_SECONDS: f64 = 0.100;

/// Java's default `isConnected` limit on signal age.
const CONNECTED_MAX_LATENCY_SECONDS: f64 = 0.500;

pub(crate) fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Which device and where it is (`DeviceIdentifier`).
#[derive(Debug)]
pub struct DeviceIdentifier {
    network: CanBus,
    network_c: CString,
    model: &'static str,
    device_id: i32,
    device_hash: u32,
    description: String,
}

impl DeviceIdentifier {
    /// Asks the C layer for the device hash. Fails with `InvalidDeviceId` for an ID outside
    /// 0 to 62.
    pub fn new(device_id: i32, model: &'static str, network: CanBus) -> Result<Self, StatusCode> {
        let model_c = cstring(model);
        let network_c = cstring(network.name());
        let mut hash = 0_u32;
        // SAFETY: both strings are NUL terminated and `hash` is valid for one write.
        let status = unsafe {
            (phoenix().c_ctre_phoenix6_encode_device)(
                device_id,
                model_c.as_ptr(),
                network_c.as_ptr(),
                &mut hash,
            )
        };
        if status != 0 {
            hash = 0;
        }
        if hash == 0 {
            return Err(StatusCode::InvalidDeviceId);
        }
        Ok(Self::build(device_id, model, network, network_c, hash))
    }

    fn build(
        device_id: i32,
        model: &'static str,
        network: CanBus,
        network_c: CString,
        device_hash: u32,
    ) -> Self {
        let description = format!("{model} {device_id} (\"{}\")", network.name());
        Self {
            network,
            network_c,
            model,
            device_id,
            device_hash,
            description,
        }
    }

    /// An identifier that never touches the natives.
    #[cfg(test)]
    pub(crate) fn for_test(model: &'static str, device_id: i32, network: &str) -> Self {
        Self::build(
            device_id,
            model,
            CanBus::new(network),
            cstring(network),
            device_id as u32 + 1,
        )
    }

    pub fn network(&self) -> &CanBus {
        &self.network
    }

    pub(crate) fn network_c(&self) -> &CStr {
        &self.network_c
    }

    pub fn model(&self) -> &'static str {
        self.model
    }

    pub fn device_id(&self) -> i32 {
        self.device_id
    }

    pub fn device_hash(&self) -> u32 {
        self.device_hash
    }

    /// Java's `toString`: `talon fx 1 ("rio")`.
    pub fn description(&self) -> &str {
        &self.description
    }
}

/// Compares the API's compliancy number with the firmware's (`FirmwareVersChecker`).
#[derive(Debug)]
pub(crate) struct FirmwareChecker {
    id: Arc<DeviceIdentifier>,
    state: Mutex<FirmwareState>,
}

#[derive(Debug)]
struct FirmwareState {
    compliancy: StatusSignal,
    creation_time: f64,
    time_to_refresh: f64,
    version_status: StatusCode,
}

impl FirmwareChecker {
    fn new(id: Arc<DeviceIdentifier>) -> Self {
        let mut compliancy =
            StatusSignal::new(Arc::clone(&id), None, spn::Compliancy_Version, "Compliancy");
        compliancy.refresh_with(false);
        let now = utils::current_time_seconds();
        let version_status = if id.device_hash == 0 {
            StatusCode::OK
        } else {
            StatusCode::CouldNotRetrieveV6Firmware
        };
        Self {
            id,
            state: Mutex::new(FirmwareState {
                compliancy,
                creation_time: now,
                time_to_refresh: now,
                version_status,
            }),
        }
    }

    /// Re-reads the firmware version every 250 ms until it matches, and reports a mismatch.
    pub(crate) fn report_if_too_old(&self) {
        let mut state = lock(&self.state);
        if state.version_status.is_ok() {
            return;
        }
        let now = utils::current_time_seconds();
        if now < state.time_to_refresh {
            return;
        }
        state.time_to_refresh = now + 0.25;
        state.compliancy.refresh_with(false);
        let firmware = state
            .compliancy
            .status()
            .is_ok()
            .then(|| state.compliancy.int_value());
        let code = classify_versions(firmware, unmanaged::api_compliancy());
        let since_creation = now - state.creation_time;
        if should_report(code, since_creation) {
            report_status_code(code, self.id.description());
        }
        state.version_status = code;
    }

    /// The version status and whether the compliancy signal itself was received.
    fn status(&self) -> (StatusCode, bool) {
        let state = lock(&self.state);
        (state.version_status, state.compliancy.status().is_ok())
    }

    /// Age of the newest compliancy frame, in seconds.
    fn compliancy_latency(&self) -> f64 {
        let mut state = lock(&self.state);
        state.compliancy.refresh_with(false).timestamp().latency()
    }
}

/// What the version check concludes. `None` means the compliancy frame did not arrive.
fn classify_versions(firmware: Option<i32>, api: i32) -> StatusCode {
    match firmware {
        None => StatusCode::CouldNotRetrieveV6Firmware,
        Some(firmware) if api > firmware => StatusCode::FirmwareTooOld,
        Some(firmware) if api < firmware => StatusCode::ApiTooOld,
        Some(_) => StatusCode::OK,
    }
}

/// A mismatch is reported at once. A missing frame is reported only after three seconds,
/// because a device built a moment ago may not have sent its version yet.
fn should_report(code: StatusCode, seconds_since_creation: f64) -> bool {
    code == StatusCode::FirmwareTooOld
        || code == StatusCode::ApiTooOld
        || (!code.is_ok() && seconds_since_creation >= 3.0)
}

/// State and behavior shared by TalonFX, CANcoder, Pigeon 2 and CANdle. The concrete devices
/// dereference to this.
#[derive(Debug)]
pub struct Device {
    id: Arc<DeviceIdentifier>,
    firmware: Arc<FirmwareChecker>,
    control_lock: Mutex<()>,
    configurator: Configurator,
    feeds_enable: bool,
}

impl Device {
    /// Creates the device handle, registers it with the simulator and starts the simulation
    /// enable feeder. Fails with `InvalidDeviceId` for an ID outside 0 to 62.
    pub(crate) fn new(
        device_id: i32,
        model: &'static str,
        network: CanBus,
        sim_type: DeviceType,
    ) -> Result<Self, StatusCode> {
        let id = Arc::new(DeviceIdentifier::new(device_id, model, network)?);
        let firmware = Arc::new(FirmwareChecker::new(Arc::clone(&id)));
        let configurator = Configurator::new(Arc::clone(&id));
        sim::create(sim_type, device_id);
        let feeds_enable = wpilib::is_simulation();
        if feeds_enable {
            auto_feed::start();
        }
        Ok(Self {
            id,
            firmware,
            control_lock: Mutex::new(()),
            configurator,
            feeds_enable,
        })
    }

    pub fn device_id(&self) -> i32 {
        self.id.device_id
    }

    pub fn network(&self) -> &CanBus {
        &self.id.network
    }

    pub fn device_hash(&self) -> u32 {
        self.id.device_hash
    }

    pub fn identifier(&self) -> &DeviceIdentifier {
        &self.id
    }

    pub fn configurator(&self) -> &Configurator {
        &self.configurator
    }

    /// True when a frame of the compliancy signal arrived in the last 0.5 s.
    pub fn is_connected(&self) -> bool {
        self.is_connected_within(CONNECTED_MAX_LATENCY_SECONDS)
    }

    pub fn is_connected_within(&self, max_latency_seconds: f64) -> bool {
        self.firmware.compliancy_latency() <= max_latency_seconds
    }

    /// Stops sending every signal that has no explicit update frequency, and sends the rest at
    /// 4 Hz. Set the signals you need first.
    pub fn optimize_bus_utilization(&self) -> StatusCode {
        self.optimize_bus_utilization_with(OPTIMIZED_FREQUENCY_HZ, DEFAULT_TIMEOUT_SECONDS)
    }

    pub fn optimize_bus_utilization_with(
        &self,
        optimized_frequency_hz: f64,
        timeout_seconds: f64,
    ) -> StatusCode {
        // SAFETY: the network is NUL terminated and outlives the call.
        StatusCode(unsafe {
            (phoenix().c_ctre_phoenix6_OptimizeUpdateFrequencies)(
                CONTEXT_API,
                self.id.network_c().as_ptr(),
                self.id.device_hash,
                optimized_frequency_hz,
                timeout_seconds,
            )
        })
    }

    /// Sends a request. When firmware and API disagree it sends an empty request instead and
    /// returns the mismatch. Failures also go to the driver station.
    pub(crate) fn set_control_request(
        &self,
        request: &(impl ControlRequest + ?Sized),
    ) -> StatusCode {
        let status = {
            let _guard = lock(&self.control_lock);
            self.firmware.report_if_too_old();
            let (version_status, compliancy_ok) = self.firmware.status();
            if !version_status.is_ok() && compliancy_ok {
                EmptyControl.send(self.id.network_c(), self.id.device_hash);
                version_status
            } else {
                request.send(self.id.network_c(), self.id.device_hash)
            }
        };
        if !status.is_ok() {
            let location = format!("{} SetControl {}", self.id.description(), request.name());
            report_status_code(status, &location);
        }
        status
    }

    /// A signal of this device. `report_on_construction` is Java's flag for whether the first
    /// read reports a failure; `refresh` reads it right away.
    pub(crate) fn signal(
        &self,
        spn: i32,
        name: &'static str,
        report_on_construction: bool,
        refresh: bool,
    ) -> StatusSignal {
        let mut signal = StatusSignal::new(
            Arc::clone(&self.id),
            Some(Arc::clone(&self.firmware)),
            spn,
            name,
        );
        if refresh {
            signal.refresh_with(report_on_construction);
        }
        signal
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        if self.feeds_enable {
            auto_feed::stop();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_classification_matches_java() {
        assert_eq!(classify_versions(Some(19), 19), StatusCode::OK);
        assert_eq!(classify_versions(Some(18), 19), StatusCode::FirmwareTooOld);
        assert_eq!(classify_versions(Some(20), 19), StatusCode::ApiTooOld);
        assert_eq!(
            classify_versions(None, 19),
            StatusCode::CouldNotRetrieveV6Firmware
        );
    }

    #[test]
    fn missing_firmware_is_reported_after_three_seconds_only() {
        let missing = StatusCode::CouldNotRetrieveV6Firmware;
        assert!(!should_report(missing, 2.9));
        assert!(should_report(missing, 3.0));
        assert!(should_report(StatusCode::FirmwareTooOld, 0.0));
        assert!(should_report(StatusCode::ApiTooOld, 0.0));
        assert!(!should_report(StatusCode::OK, 10.0));
    }

    #[test]
    fn identifier_description_matches_java_to_string() {
        let id = DeviceIdentifier::for_test("talon fx", 7, "*");
        assert_eq!(id.description(), "talon fx 7 (\"*\")");
        assert_eq!(id.device_id(), 7);
        assert_eq!(id.model(), "talon fx");
        assert_eq!(id.network().name(), "*");
    }
}
