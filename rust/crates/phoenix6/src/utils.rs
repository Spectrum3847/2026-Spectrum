//! Phoenix time and runtime helpers (`com.ctre.phoenix6.Utils`).

use crate::native::phoenix;

/// Phoenix's clock in seconds. Status signal timestamps are on this clock.
pub fn current_time_seconds() -> f64 {
    // SAFETY: no arguments, no preconditions.
    unsafe { (phoenix().c_ctre_phoenix6_get_current_time_seconds)() }
}

pub fn system_time_seconds() -> f64 {
    // SAFETY: no arguments, no preconditions.
    unsafe { (phoenix().c_ctre_phoenix6_get_system_time_seconds)() }
}

pub fn is_simulation() -> bool {
    // SAFETY: no arguments, no preconditions.
    unsafe { (phoenix().c_ctre_phoenix6_is_simulation)() != 0 }
}

pub fn is_replay() -> bool {
    // SAFETY: no arguments, no preconditions.
    unsafe { (phoenix().c_ctre_phoenix6_is_replay)() != 0 }
}

/// Converts an FPGA timestamp to Phoenix's clock, for `samplePoseAt` style lookups.
pub fn fpga_to_current_time(fpga_time_seconds: f64) -> f64 {
    let reference = if is_replay() {
        wpilib::timer::timestamp()
    } else {
        wpilib::timer::fpga_timestamp()
    };
    shift_clock(current_time_seconds(), reference, fpga_time_seconds)
}

/// Converts a Phoenix timestamp to the FPGA clock.
pub fn current_time_to_fpga_time(current_time_seconds_value: f64) -> f64 {
    let reference = if is_replay() {
        wpilib::timer::timestamp()
    } else {
        wpilib::timer::fpga_timestamp()
    };
    shift_clock(
        reference,
        current_time_seconds(),
        current_time_seconds_value,
    )
}

/// `time` moved from the clock reading `from_now` to the clock reading `to_now`.
fn shift_clock(to_now: f64, from_now: f64, time: f64) -> f64 {
    (to_now - from_now) + time
}

#[cfg(test)]
mod tests {
    use super::shift_clock;

    #[test]
    fn clock_shift_matches_java() {
        // Java: (getCurrentTimeSeconds() - FPGA now) + fpga
        assert_eq!(shift_clock(1000.0, 10.0, 7.5), 997.5);
        // currentTimeToFPGATime: (FPGA now - current now) + current
        assert_eq!(shift_clock(10.0, 1000.0, 997.5), 7.5);
    }
}
