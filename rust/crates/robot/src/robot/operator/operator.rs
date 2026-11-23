use crate::spectrum_lib::gamepads::gamepad::{Config, Gamepad};
use commands::Trigger;
use std::ops::Deref;
use std::rc::Rc;
use telemetry::telemetry;

/// A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation.
#[derive(Clone)]
pub struct OperatorButtons {
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

impl OperatorButtons {
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

pub struct Operator {
    gamepad: Rc<Gamepad>,
    pub buttons: OperatorButtons,
}

pub struct OperatorConfig {
    pub base: Config,
}

impl Default for OperatorConfig {
    fn default() -> Self {
        let mut base = Config::new("Operator", 1);
        base.triggers_deadzone = 0.0;
        Self { base }
    }
}

impl Deref for Operator {
    type Target = Gamepad;

    fn deref(&self) -> &Gamepad {
        &self.gamepad
    }
}

impl Operator {
    pub fn new(config: &OperatorConfig) -> Rc<Operator> {
        let gamepad = Gamepad::new(&config.base);

        let operator = Rc::new(Operator {
            buttons: OperatorButtons::from_gamepad(&gamepad),
            gamepad,
        });

        telemetry::print("Operator Subsystem Initialized: ");
        operator
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_matches_java() {
        let config = OperatorConfig::default().base;
        assert_eq!((config.name.as_str(), config.port), ("Operator", 1));
        assert_eq!(config.triggers_deadzone, 0.0);
        assert_eq!(config.left_stick_deadzone, 0.001);
    }
}
