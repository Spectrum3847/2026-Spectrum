//! Hoot signal logging (`com.ctre.phoenix6.SignalLogger`). Writes go into a `.hoot` file that
//! Tuner X reads. Writes before `start` return `LoggerNotRunning`.

use crate::error_report::{cstring, report_status_code};
use crate::native::phoenix;
use crate::status_code::StatusCode;

/// Where hoot files go. Failures also go to the driver station.
pub fn set_path(path: &str) -> StatusCode {
    let path = cstring(path);
    // SAFETY: the path is NUL terminated and outlives the call.
    let status =
        StatusCode(unsafe { (phoenix().c_ctre_phoenix6_platform_set_logger_path)(path.as_ptr()) });
    if !status.is_ok() {
        report_status_code(status, "ctre.phoenix6.SignalLogger.setPath");
    }
    status
}

pub fn start() -> StatusCode {
    // SAFETY: no arguments, no preconditions.
    StatusCode(unsafe { (phoenix().c_ctre_phoenix6_platform_start_logger)() })
}

pub fn stop() -> StatusCode {
    // SAFETY: no arguments, no preconditions.
    StatusCode(unsafe { (phoenix().c_ctre_phoenix6_platform_stop_logger)() })
}

/// Logs every device signal on its own when on. Java enables it by default.
pub fn enable_auto_logging(enable: bool) -> StatusCode {
    // SAFETY: plain boolean argument.
    StatusCode(unsafe { (phoenix().c_ctre_phoenix6_platform_enable_auto_logging)(enable) })
}

pub fn write_string(name: &str, value: &str) -> StatusCode {
    write_string_with_latency(name, value, 0.0)
}

/// `latency_seconds` backdates the sample.
pub fn write_string_with_latency(name: &str, value: &str, latency_seconds: f64) -> StatusCode {
    let (name, value) = (cstring(name), cstring(value));
    // SAFETY: both strings are NUL terminated and outlive the call.
    StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_platform_write_string)(
            name.as_ptr(),
            value.as_ptr(),
            latency_seconds,
        )
    })
}

pub fn write_boolean(name: &str, value: bool) -> StatusCode {
    let name = cstring(name);
    // SAFETY: the name is NUL terminated and outlives the call.
    StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_platform_write_boolean)(name.as_ptr(), value, 0.0)
    })
}

pub fn write_double(name: &str, value: f64, units: &str) -> StatusCode {
    let (name, units) = (cstring(name), cstring(units));
    // SAFETY: both strings are NUL terminated and outlive the call.
    StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_platform_write_double)(name.as_ptr(), value, units.as_ptr(), 0.0)
    })
}

pub fn write_integer(name: &str, value: i64, units: &str) -> StatusCode {
    let (name, units) = (cstring(name), cstring(units));
    // SAFETY: both strings are NUL terminated and outlive the call.
    StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_platform_write_integer)(
            name.as_ptr(),
            value,
            units.as_ptr(),
            0.0,
        )
    })
}
