//! Java `RobotController`: clock, power rails, identity.

use crate::hal::{self, HalError, or_report};
use std::sync::{PoisonError, RwLock};

pub use crate::hal::{is_real, is_simulation};

/// Microsecond clock behind [`time`]. Simulation can swap it for a stepped clock.
type TimeSource = fn() -> u64;

static TIME_SOURCE: RwLock<TimeSource> = RwLock::new(fpga_time);

/// FPGA time in microseconds since the program started.
#[track_caller]
pub fn fpga_time() -> u64 {
    or_report(hal::fpga_time())
}

/// Time from the installed source, microseconds. FPGA time unless replaced.
pub fn time() -> u64 {
    let source = *TIME_SOURCE.read().unwrap_or_else(PoisonError::into_inner);
    source()
}

pub fn set_time_source(source: TimeSource) {
    *TIME_SOURCE.write().unwrap_or_else(PoisonError::into_inner) = source;
}

/// [`time`] in seconds, Java's `Timer.getTimestamp`.
pub fn time_seconds() -> f64 {
    time() as f64 / 1e6
}

#[track_caller]
pub fn serial_number() -> String {
    hal::serial_number()
}

#[track_caller]
pub fn battery_voltage() -> f64 {
    or_report(hal::vin_voltage())
}

#[track_caller]
pub fn input_current() -> f64 {
    or_report(hal::vin_current())
}

#[track_caller]
pub fn is_browned_out() -> bool {
    or_report(hal::browned_out())
}

#[track_caller]
pub fn brownout_voltage() -> f64 {
    or_report(hal::brownout_voltage())
}

pub fn set_brownout_voltage(volts: f64) -> Result<(), HalError> {
    hal::set_brownout_voltage(volts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stepped() -> u64 {
        7_000_000
    }

    #[test]
    fn time_source_can_be_replaced() {
        // Other tests in this binary never read the global source.
        set_time_source(stepped);
        assert_eq!(time(), 7_000_000);
        assert_eq!(time_seconds(), 7.0);
        set_time_source(fpga_time);
    }
}
