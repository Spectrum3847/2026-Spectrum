//! Plain functions the `commands` crate takes at startup, so it never depends on `wpilib`.

use crate::driver_station::{self, ControlWord};

/// What the robot is doing, from the driver station control word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotMode {
    Disabled,
    Autonomous,
    Teleop,
    Test,
}

impl RobotMode {
    /// The mode the timed loop runs, in Java's `IterativeRobotBase` order: disabled when not
    /// enabled or no driver station, then autonomous, teleop, test.
    pub fn from_control_word(word: ControlWord) -> Self {
        if !word.is_enabled() {
            Self::Disabled
        } else if word.autonomous() {
            Self::Autonomous
        } else if !word.test() {
            Self::Teleop
        } else {
            Self::Test
        }
    }

    pub fn is_enabled(self) -> bool {
        self != Self::Disabled
    }
}

/// FPGA time in seconds, Java's `Timer.getFPGATimestamp`.
pub fn time_source() -> f64 {
    crate::timer::fpga_timestamp()
}

/// The current mode from the cached control word.
pub fn robot_mode() -> RobotMode {
    RobotMode::from_control_word(driver_station::control_word())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_follows_java_precedence() {
        let mode = |e, a, t, ds| RobotMode::from_control_word(ControlWord::new(e, a, t, ds));
        assert_eq!(mode(false, false, false, true), RobotMode::Disabled);
        assert_eq!(mode(true, false, false, false), RobotMode::Disabled);
        assert_eq!(mode(true, true, false, true), RobotMode::Autonomous);
        assert_eq!(mode(true, false, false, true), RobotMode::Teleop);
        assert_eq!(mode(true, false, true, true), RobotMode::Test);
        assert_eq!(mode(true, true, true, true), RobotMode::Autonomous);
    }

    #[test]
    fn only_disabled_is_not_enabled() {
        assert!(!RobotMode::Disabled.is_enabled());
        assert!(RobotMode::Teleop.is_enabled());
    }
}
