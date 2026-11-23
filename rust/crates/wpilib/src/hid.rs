//! Java `GenericHID` and `XboxController`: reads and outputs for one joystick port.
//!
//! Edge-triggered reads (`raw_button_pressed`, `raw_button_released`) clear the shared latch in
//! [`crate::driver_station`], so two readers of the same button split the events, as in Java.

use crate::driver_station;
use std::cell::Cell;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RumbleType {
    Left,
    Right,
    Both,
}

/// `GenericHID.HIDType` values as the driver station reports them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HidType {
    Unknown,
    XInputUnknown,
    XInputGamepad,
    XInputWheel,
    XInputArcadeStick,
    XInputFlightStick,
    XInputDancePad,
    XInputGuitar,
    XInputGuitar2,
    XInputDrumKit,
    XInputGuitar3,
    XInputArcadePad,
    HidJoystick,
    HidGamepad,
    HidDriving,
    HidFlight,
    Hid1stPerson,
}

impl HidType {
    pub fn from_value(value: i32) -> Option<Self> {
        Some(match value {
            -1 => Self::Unknown,
            0 => Self::XInputUnknown,
            1 => Self::XInputGamepad,
            2 => Self::XInputWheel,
            3 => Self::XInputArcadeStick,
            4 => Self::XInputFlightStick,
            5 => Self::XInputDancePad,
            6 => Self::XInputGuitar,
            7 => Self::XInputGuitar2,
            8 => Self::XInputDrumKit,
            11 => Self::XInputGuitar3,
            19 => Self::XInputArcadePad,
            20 => Self::HidJoystick,
            21 => Self::HidGamepad,
            22 => Self::HidDriving,
            23 => Self::HidFlight,
            24 => Self::Hid1stPerson,
            _ => return None,
        })
    }
}

/// Rumble as the HAL takes it: 0 to 65535. Out-of-range input clamps to 0..=1 first.
fn rumble_value(value: f64) -> i32 {
    (value.clamp(0.0, 1.0) * 65535.0) as i32
}

/// Outputs and rumble state of one port, and the reads for it.
#[derive(Debug)]
pub struct GenericHid {
    port: i32,
    outputs: Cell<i32>,
    left_rumble: Cell<i32>,
    right_rumble: Cell<i32>,
}

impl GenericHid {
    pub fn new(port: i32) -> Self {
        Self {
            port,
            outputs: Cell::new(0),
            left_rumble: Cell::new(0),
            right_rumble: Cell::new(0),
        }
    }

    pub fn port(&self) -> i32 {
        self.port
    }

    pub fn raw_button(&self, button: i32) -> bool {
        driver_station::stick_button(self.port, button)
    }

    pub fn raw_button_pressed(&self, button: i32) -> bool {
        driver_station::stick_button_pressed(self.port, button)
    }

    pub fn raw_button_released(&self, button: i32) -> bool {
        driver_station::stick_button_released(self.port, button)
    }

    pub fn raw_axis(&self, axis: i32) -> f64 {
        driver_station::stick_axis(self.port, axis)
    }

    /// Angle of POV `pov` in degrees, -1 when centered.
    pub fn pov_at(&self, pov: i32) -> i32 {
        driver_station::stick_pov(self.port, pov)
    }

    pub fn pov(&self) -> i32 {
        self.pov_at(0)
    }

    pub fn axis_count(&self) -> usize {
        driver_station::stick_axis_count(self.port)
    }

    pub fn pov_count(&self) -> usize {
        driver_station::stick_pov_count(self.port)
    }

    pub fn button_count(&self) -> usize {
        driver_station::stick_button_count(self.port)
    }

    pub fn is_connected(&self) -> bool {
        driver_station::is_joystick_connected(self.port)
    }

    pub fn hid_type(&self) -> Option<HidType> {
        HidType::from_value(driver_station::joystick_type(self.port))
    }

    pub fn is_xbox(&self) -> bool {
        driver_station::joystick_is_xbox(self.port)
    }

    pub fn set_output(&self, output_number: i32, value: bool) {
        let bit = 1 << (output_number - 1);
        let outputs = if value {
            self.outputs.get() | bit
        } else {
            self.outputs.get() & !bit
        };
        self.outputs.set(outputs);
        self.send_outputs();
    }

    pub fn set_outputs(&self, value: i32) {
        self.outputs.set(value);
        self.send_outputs();
    }

    pub fn set_rumble(&self, kind: RumbleType, value: f64) {
        let rumble = rumble_value(value);
        match kind {
            RumbleType::Left => self.left_rumble.set(rumble),
            RumbleType::Right => self.right_rumble.set(rumble),
            RumbleType::Both => {
                self.left_rumble.set(rumble);
                self.right_rumble.set(rumble);
            }
        }
        self.send_outputs();
    }

    fn send_outputs(&self) {
        driver_station::set_joystick_outputs(
            self.port,
            i64::from(self.outputs.get()),
            self.left_rumble.get(),
            self.right_rumble.get(),
        );
    }
}

/// `XboxController.Button` numbers, from 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    A = 1,
    B = 2,
    X = 3,
    Y = 4,
    LeftBumper = 5,
    RightBumper = 6,
    Back = 7,
    Start = 8,
    LeftStick = 9,
    RightStick = 10,
}

/// `XboxController.Axis` numbers, from 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    LeftX = 0,
    LeftY = 1,
    LeftTrigger = 2,
    RightTrigger = 3,
    RightX = 4,
    RightY = 5,
}

/// An Xbox controller on a port. Y axes are positive toward the driver, as in Java.
#[derive(Debug)]
pub struct XboxController {
    hid: GenericHid,
}

impl XboxController {
    pub fn new(port: i32) -> Self {
        Self {
            hid: GenericHid::new(port),
        }
    }

    pub fn hid(&self) -> &GenericHid {
        &self.hid
    }

    pub fn axis(&self, axis: Axis) -> f64 {
        self.hid.raw_axis(axis as i32)
    }

    pub fn left_x(&self) -> f64 {
        self.axis(Axis::LeftX)
    }

    pub fn right_x(&self) -> f64 {
        self.axis(Axis::RightX)
    }

    pub fn left_y(&self) -> f64 {
        self.axis(Axis::LeftY)
    }

    pub fn right_y(&self) -> f64 {
        self.axis(Axis::RightY)
    }

    pub fn left_trigger_axis(&self) -> f64 {
        self.axis(Axis::LeftTrigger)
    }

    pub fn right_trigger_axis(&self) -> f64 {
        self.axis(Axis::RightTrigger)
    }

    pub fn button(&self, button: Button) -> bool {
        self.hid.raw_button(button as i32)
    }

    pub fn button_pressed(&self, button: Button) -> bool {
        self.hid.raw_button_pressed(button as i32)
    }

    pub fn button_released(&self, button: Button) -> bool {
        self.hid.raw_button_released(button as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rumble_scales_and_clamps() {
        assert_eq!(rumble_value(0.0), 0);
        assert_eq!(rumble_value(1.0), 65535);
        assert_eq!(rumble_value(0.5), 32767);
        assert_eq!(rumble_value(2.0), 65535);
        assert_eq!(rumble_value(-1.0), 0);
        assert_eq!(rumble_value(f64::NAN), 0);
    }

    #[test]
    fn hid_type_values_match_the_driver_station() {
        assert_eq!(HidType::from_value(-1), Some(HidType::Unknown));
        assert_eq!(HidType::from_value(1), Some(HidType::XInputGamepad));
        assert_eq!(HidType::from_value(21), Some(HidType::HidGamepad));
        assert_eq!(HidType::from_value(9), None);
    }

    #[test]
    fn xbox_numbering_matches_wpilib() {
        assert_eq!(Button::A as i32, 1);
        assert_eq!(Button::RightStick as i32, 10);
        assert_eq!(Axis::LeftX as i32, 0);
        assert_eq!(Axis::LeftTrigger as i32, 2);
        assert_eq!(Axis::RightX as i32, 4);
        assert_eq!(Axis::RightY as i32, 5);
    }
}
