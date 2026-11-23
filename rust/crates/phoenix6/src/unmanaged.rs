//! Library-wide switches (`com.ctre.phoenix6.unmanaged.Unmanaged`).

use crate::native::phoenix;

/// Tells the devices the robot stays enabled for `timeout_ms` more milliseconds. Zero disables.
/// Simulation needs this every 10 ms or so; see [`crate::auto_feed`].
pub fn feed_enable(timeout_ms: i32) {
    // SAFETY: plain integer argument.
    unsafe { (phoenix().c_ctre_phoenix6_unmanaged_feed_enable)(timeout_ms) }
}

pub fn enable_state() -> bool {
    // SAFETY: no arguments, no preconditions.
    unsafe { (phoenix().c_ctre_phoenix6_unmanaged_get_enable_state)() }
}

pub fn phoenix_version() -> i32 {
    // SAFETY: no arguments, no preconditions.
    unsafe { (phoenix().c_ctre_phoenix6_unmanaged_get_phoenix_version)() }
}

/// The compliancy number the API expects from device firmware.
pub fn api_compliancy() -> i32 {
    // SAFETY: no arguments, no preconditions.
    unsafe { (phoenix().c_ctre_phoenix6_unmanaged_get_api_compliancy)() }
}

/// Delays the Phoenix diagnostics server start. A negative value disables it.
pub fn set_phoenix_diagnostics_start_time(start_time_seconds: f64) {
    // SAFETY: plain float argument.
    unsafe { (phoenix().c_Phoenix_Diagnostics_SetSecondsToStart)(start_time_seconds) }
}
