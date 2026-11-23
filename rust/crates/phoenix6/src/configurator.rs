//! Reading and writing device configuration (`ParentConfigurator` and the device configurators).

use crate::configs::{ConfigItem, ConfigValue, ParentConfiguration};
use crate::device::{DeviceIdentifier, lock};
use crate::error_report::{cstring, report_status_code};
use crate::native::phoenix;
use crate::status_code::StatusCode;
use crate::utils;
use std::ffi::c_char;
use std::ptr;
use std::sync::{Arc, Mutex};

const CONTEXT_API: i32 = 0;

/// Java's `DefaultTimeoutSeconds`: how long a config call waits for the device.
pub const DEFAULT_TIMEOUT_SECONDS: f64 = 0.100;

/// Applies and reads configuration for one device. Calls block until the device answers or the
/// timeout passes, so run them at startup, not in the robot loop.
#[derive(Debug)]
pub struct Configurator {
    id: Arc<DeviceIdentifier>,
    state: Mutex<FrequencyWatch>,
}

/// Warns when config calls arrive constantly, as Java's `reportIfFrequent` does.
#[derive(Debug)]
struct FrequencyWatch {
    creation_time: f64,
    last_config_time: f64,
    frequent_since: f64,
}

impl FrequencyWatch {
    /// True when calls have come less than a second apart for over three seconds.
    fn note_call(&mut self, now: f64) -> bool {
        let last = self.last_config_time;
        self.last_config_time = now;
        if now - self.creation_time < 5.0 {
            return false;
        }
        if now - last < 1.0 {
            if self.frequent_since == 0.0 {
                self.frequent_since = last;
            }
        } else {
            self.frequent_since = 0.0;
        }
        self.frequent_since > 0.0 && now - self.frequent_since > 3.0
    }
}

impl Configurator {
    pub(crate) fn new(id: Arc<DeviceIdentifier>) -> Self {
        let now = utils::current_time_seconds();
        Self {
            id,
            state: Mutex::new(FrequencyWatch {
                creation_time: now,
                last_config_time: now,
                frequent_since: 0.0,
            }),
        }
    }

    /// Writes a configuration or a group with the default timeout.
    pub fn apply(&self, config: &impl ParentConfiguration) -> StatusCode {
        self.apply_with_timeout(config, DEFAULT_TIMEOUT_SECONDS)
    }

    pub fn apply_with_timeout(
        &self,
        config: &impl ParentConfiguration,
        timeout_seconds: f64,
    ) -> StatusCode {
        self.set_configs(
            &config.serialize(),
            timeout_seconds,
            config.future_proof(),
            false,
        )
    }

    /// Reads a configuration or a group from the device with the default timeout.
    pub fn refresh(&self, config: &mut impl ParentConfiguration) -> StatusCode {
        self.refresh_with_timeout(config, DEFAULT_TIMEOUT_SECONDS)
    }

    pub fn refresh_with_timeout(
        &self,
        config: &mut impl ParentConfiguration,
        timeout_seconds: f64,
    ) -> StatusCode {
        let (status, text) = self.get_configs(timeout_seconds);
        if status.is_ok() {
            config.deserialize(&text);
        }
        status
    }

    /// Writes one number to a command SPN such as set position or clear sticky faults. These
    /// overwrite a pending duplicate and are never future-proofed.
    pub(crate) fn set_command_value(
        &self,
        spn: i32,
        value: f64,
        timeout_seconds: f64,
    ) -> StatusCode {
        let text = crate::configs::serialize_items(&[ConfigItem {
            spn,
            value: ConfigValue::Double(value),
        }]);
        self.set_configs(&text, timeout_seconds, false, true)
    }

    fn set_configs(
        &self,
        text: &str,
        timeout_seconds: f64,
        future_proof: bool,
        override_if_duplicate: bool,
    ) -> StatusCode {
        let values = cstring(text);
        let status = {
            let mut watch = lock(&self.state);
            // SAFETY: the network and the values are NUL terminated and outlive the call;
            // the length excludes the terminator.
            let status = StatusCode(unsafe {
                (phoenix().c_ctre_phoenix6_set_configs)(
                    CONTEXT_API,
                    self.id.network_c().as_ptr(),
                    self.id.device_hash() as i32,
                    timeout_seconds,
                    values.as_ptr(),
                    u32::try_from(values.as_bytes().len()).unwrap_or(u32::MAX),
                    future_proof,
                    override_if_duplicate,
                    false,
                )
            });
            self.report_if_frequent(&mut watch);
            status
        };
        if !status.is_ok() && status != StatusCode::TimeoutCannotBeZero {
            report_status_code(status, &format!("{} Apply Config", self.id.description()));
        }
        status
    }

    fn get_configs(&self, timeout_seconds: f64) -> (StatusCode, String) {
        let mut out: *mut c_char = ptr::null_mut();
        let status = {
            let mut watch = lock(&self.state);
            // SAFETY: the network is NUL terminated; `out` is a valid out pointer.
            let status = StatusCode(unsafe {
                (phoenix().c_ctre_phoenix6_get_configs)(
                    CONTEXT_API,
                    self.id.network_c().as_ptr(),
                    self.id.device_hash() as i32,
                    timeout_seconds,
                    &mut out,
                    false,
                )
            });
            self.report_if_frequent(&mut watch);
            status
        };
        let text = crate::configs::take_string(&mut out);
        if !status.is_ok() {
            report_status_code(status, &format!("{} Refresh Config", self.id.description()));
        }
        (status, text)
    }

    fn report_if_frequent(&self, watch: &mut FrequencyWatch) {
        if watch.note_call(utils::current_time_seconds()) {
            let location = format!("{} Config", self.id.description());
            report_status_code(StatusCode::FrequentConfigCalls, &location);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::FrequencyWatch;

    fn watch(creation: f64) -> FrequencyWatch {
        FrequencyWatch {
            creation_time: creation,
            last_config_time: creation,
            frequent_since: 0.0,
        }
    }

    #[test]
    fn no_warning_in_the_first_five_seconds() {
        let mut w = watch(0.0);
        for i in 1..40 {
            assert!(!w.note_call(f64::from(i) * 0.1));
        }
    }

    #[test]
    fn warns_after_three_seconds_of_calls_under_a_second_apart() {
        let mut w = watch(0.0);
        assert!(!w.note_call(6.0));
        let mut warned_at = None;
        let mut t = 6.0;
        while warned_at.is_none() && t < 20.0 {
            t += 0.5;
            if w.note_call(t) {
                warned_at = Some(t);
            }
        }
        // Frequent calls start counting from the call before the first frequent one: 6.0.
        assert_eq!(warned_at, Some(9.5));
    }

    #[test]
    fn a_pause_resets_the_count() {
        let mut w = watch(0.0);
        w.note_call(6.0);
        w.note_call(6.5);
        w.note_call(7.0);
        assert!(!w.note_call(10.0));
        assert!(!w.note_call(10.5));
        assert_eq!(w.frequent_since, 10.0);
        assert!(!w.note_call(12.0));
        assert_eq!(w.frequent_since, 0.0);
    }
}
