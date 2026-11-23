use crate::robot::subsystems::swerve::swerve::PilotDriveInputs;
use crate::robot::subsystems::swerve::swerve_config::SwerveConfig;
use crate::spectrum_lib::gamepads::gamepad::{Config, Gamepad};
use crate::spectrum_lib::util::curve::Curve;
use commands::Trigger;
use std::ops::Deref;
use std::rc::Rc;
use telemetry::telemetry;
use wpimath::geometry::Rotation2d;
use wpimath::units::degrees_to_radians;

/// A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation.
#[derive(Clone)]
pub struct PilotButtons {
    pub lb: Trigger,
    pub rb: Trigger,
    pub lt: Trigger,
    pub rt: Trigger,

    pub a_button: Trigger,
    pub b_button: Trigger,
    pub x_button: Trigger,
    pub y_button: Trigger,

    pub start_button: Trigger,
    pub select_button: Trigger,

    pub left_stick_press: Trigger,
    pub right_stick_press: Trigger,

    pub d_pad_up: Trigger,
    pub d_pad_down: Trigger,
    pub d_pad_left: Trigger,
    pub d_pad_right: Trigger,
}

impl PilotButtons {
    pub fn from_gamepad(gamepad: &Gamepad) -> Self {
        Self {
            lb: gamepad.left_bumper.clone(),
            rb: gamepad.right_bumper.clone(),
            lt: gamepad.left_trigger.clone(),
            rt: gamepad.right_trigger.clone(),

            a_button: gamepad.a.clone(),
            b_button: gamepad.b.clone(),
            x_button: gamepad.x.clone(),
            y_button: gamepad.y.clone(),

            start_button: gamepad.start.clone(),
            select_button: gamepad.select.clone(),

            left_stick_press: gamepad.left_stick_click.clone(),
            right_stick_press: gamepad.right_stick_click.clone(),

            d_pad_up: gamepad.up_dpad.clone(),
            d_pad_down: gamepad.down_dpad.clone(),
            d_pad_left: gamepad.left_dpad.clone(),
            d_pad_right: gamepad.right_dpad.clone(),
        }
    }

    /// Every button is `trigger`.
    pub fn filled_with(trigger: &Trigger) -> Self {
        Self {
            lb: trigger.clone(),
            rb: trigger.clone(),
            lt: trigger.clone(),
            rt: trigger.clone(),

            a_button: trigger.clone(),
            b_button: trigger.clone(),
            x_button: trigger.clone(),
            y_button: trigger.clone(),

            start_button: trigger.clone(),
            select_button: trigger.clone(),

            left_stick_press: trigger.clone(),
            right_stick_press: trigger.clone(),

            d_pad_up: trigger.clone(),
            d_pad_down: trigger.clone(),
            d_pad_left: trigger.clone(),
            d_pad_right: trigger.clone(),
        }
    }
}

pub struct Pilot {
    gamepad: Rc<Gamepad>,
    pub buttons: PilotButtons,
}

pub struct PilotConfig {
    pub base: Config,
}

impl Default for PilotConfig {
    fn default() -> Self {
        let deadzone = 0.15;

        let mut base = Config::new("Pilot", 0);

        base.left_stick_deadzone = deadzone;
        base.left_stick_exp = 3.0;

        base.right_stick_deadzone = deadzone;
        base.right_stick_exp = 3.0;

        base.triggers_deadzone = deadzone;
        base.triggers_exp = 1.0;
        base.triggers_scalar = 1.0;
        Self { base }
    }
}

impl Deref for Pilot {
    type Target = Gamepad;

    fn deref(&self) -> &Gamepad {
        &self.gamepad
    }
}

impl Pilot {
    /// Java reads `Robot.getConfig().swerve` for the stick scalars, so the swerve config
    /// comes in as a parameter.
    pub fn new(config: &mut PilotConfig, swerve: &SwerveConfig) -> Rc<Pilot> {
        let gamepad = Gamepad::new(&config.base);

        let (left_scalar, right_scalar) = stick_scalars(swerve);
        config.base.left_stick_scalar = left_scalar;
        config.base.right_stick_scalar = right_scalar;
        gamepad.left_stick_curve.set_scalar(left_scalar);
        gamepad.right_stick_curve.set_scalar(right_scalar);

        let pilot = Rc::new(Pilot {
            buttons: PilotButtons::from_gamepad(&gamepad),
            gamepad,
        });

        telemetry::print("Pilot Subsystem Initialized: ");
        pilot
    }

    pub fn set_max_velocity(&self, max_velocity: f64) {
        self.left_stick_curve.set_scalar(max_velocity);
    }

    pub fn set_max_rotational_velocity(&self, max_rotational_velocity: f64) {
        self.right_stick_curve.set_scalar(max_rotational_velocity);
    }

    // Positive is forward. Up on the left stick is positive, hence the negation.
    pub fn get_drive_fwd_positive(&self) -> f64 {
        self.left_stick_curve.calculate(-self.get_left_y())
    }

    // Positive is left. Left on the left stick is positive, hence the negation.
    pub fn get_drive_left_positive(&self) -> f64 {
        -self.left_stick_curve.calculate(self.get_left_x())
    }

    // Positive is counter-clockwise
    pub fn get_drive_ccw_positive(&self) -> f64 {
        let ccw_positive = self.right_stick_curve.calculate(self.get_right_x());
        -ccw_positive
    }

    pub fn get_pilot_stick_angle(&self) -> f64 {
        let direction: Rotation2d = self.get_left_stick_direction();
        direction.radians()
    }
}

/// Left stick m/s and right stick rad/s at full deflection.
fn stick_scalars(swerve: &SwerveConfig) -> (f64, f64) {
    (
        swerve.linear_speed_at_12_volts_mps,
        degrees_to_radians(swerve.angular_speed_at_12_volts_dps),
    )
}

impl PilotDriveInputs for Pilot {
    fn get_drive_fwd_positive(&self) -> f64 {
        Pilot::get_drive_fwd_positive(self)
    }

    fn get_drive_left_positive(&self) -> f64 {
        Pilot::get_drive_left_positive(self)
    }

    fn get_drive_ccw_positive(&self) -> f64 {
        Pilot::get_drive_ccw_positive(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_matches_java() {
        let config = PilotConfig::default().base;
        assert_eq!((config.name.as_str(), config.port), ("Pilot", 0));
        assert_eq!(config.left_stick_deadzone, 0.15);
        assert_eq!(config.left_stick_exp, 3.0);
        assert_eq!(config.right_stick_deadzone, 0.15);
        assert_eq!(config.right_stick_exp, 3.0);
        assert_eq!(config.triggers_deadzone, 0.15);
        assert_eq!(config.triggers_exp, 1.0);
        assert_eq!(config.triggers_scalar, 1.0);
    }

    #[test]
    fn stick_scalars_come_from_the_swerve_speeds() {
        let (left, right) = stick_scalars(&SwerveConfig::default());
        assert_eq!(left, 5.12);
        assert!((right - 540.0_f64.to_radians()).abs() < 1e-12);
    }
}
