//! Base setup for Spectrum robots. Java's `SpectrumRobot extends TimedRobot`; Rust
//! has no inheritance, so [`timed_robot`] wraps the robot in a `TimedRobot` with the same
//! settings applied.

use commands::CommandScheduler;
use wpilib::driver_station;
use wpilib::robot::{Robot, TimedRobot};

/// Loop length past which WPILib prints "Loop time of Xs overrun" and a per-section epoch
/// breakdown to the Driver Station. WPILib's own default is 0.5 s.
///
/// Every overrun below this value stays off the console. In the 2026-09-05 logs 60 to 90
/// percent of enabled loops ran over 25 ms. The `Scheduler/*` timers in the wpilog are the
/// record to read instead. Drop this to 0.04 for a diagnostic build that wants the
/// per-section print, keeping in mind the print is itself work the loop has to do.
pub const LOOP_OVERRUN_WARNING_SECONDS: f64 = 0.20;

/// Builds the timed robot loop, silences joystick connection warnings and sets the loop
/// overrun watchdog and the scheduler's overrun period to [`LOOP_OVERRUN_WARNING_SECONDS`].
///
/// Java reached into the private `m_watchdog` field by reflection because WPILib has no
/// setter. `TimedRobot::set_watchdog_timeout` is the setter here, so the Java fallback
/// warning ("Failed to disable loop overrun warnings.") has nothing left to catch.
///
/// Java ran this in the base constructor, before the subclass constructor. Here the robot
/// value already exists, so do subsystem construction in `Robot::robot_init`.
pub fn timed_robot<R: Robot>(robot: R) -> TimedRobot<R> {
    let mut timed_robot = TimedRobot::new(robot);
    driver_station::silence_joystick_connection_warning(true);
    timed_robot.set_watchdog_timeout(LOOP_OVERRUN_WARNING_SECONDS);
    CommandScheduler::get_instance().set_period(LOOP_OVERRUN_WARNING_SECONDS);
    timed_robot
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overrun_threshold_is_two_hundred_milliseconds() {
        assert_eq!(LOOP_OVERRUN_WARNING_SECONDS, 0.20);
    }
}
