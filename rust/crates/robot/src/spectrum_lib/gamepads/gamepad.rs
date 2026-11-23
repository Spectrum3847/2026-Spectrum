//! Base for the robot's Xbox-compatible gamepads. Wraps a WPILib `XboxController` and
//! exposes the buttons, bumper and trigger combinations, stick axis helpers, and rumble
//! commands that bindings need.
//!
//! Java's `Gamepad` is an abstract base class, subclassed once per operator role (pilot,
//! copilot). A Rust gamepad holds the `Rc<Gamepad>` that [`Gamepad::new`] returns, which
//! registers with the scheduler itself. Java `protected` members are `pub`, and the
//! Triggers the subclass copies out of the base (`LB = leftBumper`) are `Trigger` clones.
//!
//! When `Config::attached` is `false`, every trigger stays `false` for the life of the
//! object and axis reads return `0.0`, because no controller is constructed.

use crate::spectrum_lib::util::exp_curve::ExpCurve;
use crate::spectrum_lib::util::util;
use commands::{Command, CommandScheduler, Commands, Subsystem, SubsystemId, Trigger};
use std::cell::Cell;
use std::f64::consts::PI;
use std::rc::Rc;
use telemetry::telemetry;
use wpilib::dashboard::{Alert, AlertType};
use wpilib::driver_station::{self, Alliance};
use wpilib::hid::{Axis, Button, GenericHid, RumbleType, XboxController};
use wpimath::geometry::Rotation2d;

thread_local! {
    static K_FALSE: Trigger = Trigger::new(|| false);
}

/// A trigger that is always `false`; the default for every button before wiring. Java's
/// `Gamepad.kFalse`.
pub fn k_false() -> Trigger {
    K_FALSE.with(Trigger::clone)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Threshold {
    /// Fires when the axis value is strictly greater than the threshold.
    Greater,
    /// Fires when the axis value is strictly less than the threshold.
    Less,
    /// Fires when the absolute axis value is greater than the threshold (deadband check).
    AbsGreater,
}

impl Threshold {
    pub fn test(self, value: f64, threshold: f64) -> bool {
        match self {
            Threshold::Greater => value > threshold,
            Threshold::Less => value < threshold,
            Threshold::AbsGreater => value.abs() > threshold,
        }
    }
}

/// DriverStation USB port, axis curve parameters, and whether this robot uses the
/// controller. Lombok getters and setters on the Java class are public fields.
#[derive(Debug, Clone)]
pub struct Config {
    /// Human-readable controller name used in alerts and telemetry.
    pub name: String,

    /// USB port number as shown in the DriverStation application (0-indexed).
    pub port: i32,

    /// Whether this controller should be used on the current robot; `false` disables it.
    /// Java: starts `false`, so a gamepad stays off until its config turns it on.
    pub attached: bool,

    /// Deadzone applied to both left-stick axes before the exponential curve.
    pub left_stick_deadzone: f64,

    /// Exponent for the left-stick exponential response curve (1.0 = linear).
    pub left_stick_exp: f64,

    /// Output scalar applied after the left-stick exponential curve.
    pub left_stick_scalar: f64,

    /// Deadzone applied to both right-stick axes before the exponential curve.
    pub right_stick_deadzone: f64,

    /// Exponent for the right-stick exponential response curve.
    pub right_stick_exp: f64,

    /// Output scalar applied after the right-stick exponential curve.
    pub right_stick_scalar: f64,

    /// Deadzone applied to both analog trigger axes before the exponential curve.
    pub triggers_deadzone: f64,

    /// Exponent for the analog-trigger exponential response curve.
    pub triggers_exp: f64,

    /// Output scalar applied after the analog-trigger exponential curve.
    pub triggers_scalar: f64,
}

impl Config {
    pub fn new(name: impl Into<String>, port: i32) -> Self {
        Self {
            name: name.into(),
            port,
            attached: false,
            left_stick_deadzone: 0.001,
            left_stick_exp: 1.0,
            left_stick_scalar: 1.0,
            right_stick_deadzone: 0.001,
            right_stick_exp: 1.0,
            right_stick_scalar: 1.0,
            triggers_deadzone: 0.002,
            triggers_exp: 1.0,
            triggers_scalar: 1.0,
        }
    }
}

/// The controller and its attach flag, shared with every trigger and command closure.
struct Input {
    attached: bool,
    xbox: Option<XboxController>,
}

impl Input {
    fn is_connected(&self) -> bool {
        self.attached && self.xbox.as_ref().is_some_and(|x| x.hid().is_connected())
    }

    /// Reads an axis, or returns `0.0` when the controller is not connected.
    fn axis(&self, raw: impl Fn(&XboxController) -> f64) -> f64 {
        match &self.xbox {
            Some(xbox) if self.is_connected() => raw(xbox),
            _ => 0.0,
        }
    }

    fn raw_button(&self, button: Button) -> bool {
        self.xbox.as_ref().is_some_and(|x| x.button(button))
    }

    fn raw_trigger_axis(&self, axis: Axis) -> f64 {
        self.xbox.as_ref().map_or(0.0, |x| x.axis(axis))
    }

    fn pov_is(&self, angle: i32) -> bool {
        self.xbox
            .as_ref()
            .is_some_and(|x| x.hid().pov_at(0) == angle)
    }

    fn left_x(&self) -> f64 {
        self.axis(XboxController::left_x)
    }

    fn left_y(&self) -> f64 {
        self.axis(XboxController::left_y)
    }

    fn right_x(&self) -> f64 {
        self.axis(XboxController::right_x)
    }

    fn right_y(&self) -> f64 {
        self.axis(XboxController::right_y)
    }

    fn rumble(&self, left_intensity: f64, right_intensity: f64) {
        if !self.is_connected() {
            return;
        }
        if let Some(xbox) = &self.xbox {
            xbox.hid().set_rumble(RumbleType::Left, left_intensity);
            xbox.hid().set_rumble(RumbleType::Right, right_intensity);
        }
    }
}

fn button_trigger(input: &Rc<Input>, button: Button) -> Trigger {
    let input = input.clone();
    Trigger::new(move || input.raw_button(button))
}

fn pov_trigger(input: &Rc<Input>, angle: i32) -> Trigger {
    let input = input.clone();
    Trigger::new(move || input.pov_is(angle))
}

fn axis_greater_than(input: &Rc<Input>, axis: Axis, threshold: f64) -> Trigger {
    let input = input.clone();
    Trigger::new(move || input.raw_trigger_axis(axis) > threshold)
}

fn axis_trigger(t: Threshold, threshold: f64, v: impl Fn() -> f64 + 'static) -> Trigger {
    Trigger::new(move || t.test(v(), threshold))
}

pub struct Gamepad {
    id: SubsystemId,
    /// Java's default subsystem name is the subclass name, which is also the config name
    /// for the pilot and operator.
    name: String,
    input: Rc<Input>,
    disconnected_alert: Alert,

    /// Xbox A, the cross glyph.
    pub a: Trigger,
    /// Xbox B, the circle glyph.
    pub b: Trigger,
    /// Xbox X, the square glyph.
    pub x: Trigger,
    /// Xbox Y, the triangle glyph.
    pub y: Trigger,
    pub left_bumper: Trigger,
    pub right_bumper: Trigger,
    /// Fires once the left analog trigger passes the configured trigger deadzone.
    pub left_trigger: Trigger,
    /// Fires once the right analog trigger passes the configured trigger deadzone.
    pub right_trigger: Trigger,
    pub left_stick_click: Trigger,
    pub right_stick_click: Trigger,
    /// Xbox Menu, exposed by WPILib as the start button.
    pub start: Trigger,
    /// Xbox View, exposed by WPILib as the back button.
    pub select: Trigger,
    pub up_dpad: Trigger,
    pub down_dpad: Trigger,
    /// Fires for left, up-left, or down-left on the D-pad.
    pub left_dpad: Trigger,
    /// Fires for right, up-right, or down-right on the D-pad.
    pub right_dpad: Trigger,
    /// Fires when the left stick's Y axis leaves the configured deadzone in either direction.
    pub left_stick_y: Trigger,
    /// Fires when the left stick's X axis leaves the configured deadzone in either direction.
    pub left_stick_x: Trigger,
    /// Fires when the right stick's Y axis leaves the configured deadzone in either direction.
    pub right_stick_y: Trigger,
    /// Fires when the right stick's X axis leaves the configured deadzone in either direction.
    pub right_stick_x: Trigger,

    pub no_bumpers: Trigger,
    pub left_bumper_only: Trigger,
    pub right_bumper_only: Trigger,
    pub both_bumpers: Trigger,
    pub no_triggers: Trigger,
    pub left_trigger_only: Trigger,
    pub right_trigger_only: Trigger,
    pub both_triggers: Trigger,
    pub no_modifiers: Trigger,

    /// Last non-zero left-stick direction, kept so the value survives the stick recentring.
    stored_left_stick_direction: Cell<Rotation2d>,

    /// Last non-zero right-stick direction, kept so the value survives the stick recentring.
    stored_right_stick_direction: Cell<Rotation2d>,

    /// Set the first time the gamepad is seen connected.
    configured: Cell<bool>,

    printed: Cell<bool>,

    /// Exponential response curve applied to both left-stick axes.
    pub left_stick_curve: ExpCurve,

    /// Exponential response curve applied to both right-stick axes.
    pub right_stick_curve: ExpCurve,

    pub triggers_curve: ExpCurve,

    pub teleop: Trigger,
    pub auto_mode: Trigger,
    pub test_mode: Trigger,
    pub disabled: Trigger,
}

impl Gamepad {
    /// Builds the controller and triggers from `config`, and registers with the scheduler.
    /// Later changes to the config do not reach the gamepad, where Java kept the same
    /// object.
    pub fn new(config: &Config) -> Rc<Gamepad> {
        let disconnected_alert = Alert::new(
            &format!("{} Gamepad Disconnected", config.name),
            AlertType::Error,
        );
        let (left_stick_curve, right_stick_curve, triggers_curve) = curves(config);

        let input = Rc::new(Input {
            attached: config.attached,
            xbox: config.attached.then(|| XboxController::new(config.port)),
        });

        let off = k_false();
        let mut a = off.clone();
        let mut b = off.clone();
        let mut x = off.clone();
        let mut y = off.clone();
        let mut left_bumper = off.clone();
        let mut right_bumper = off.clone();
        let mut left_trigger = off.clone();
        let mut right_trigger = off.clone();
        let mut left_stick_click = off.clone();
        let mut right_stick_click = off.clone();
        let mut start = off.clone();
        let mut select = off.clone();
        let mut up_dpad = off.clone();
        let mut down_dpad = off.clone();
        let mut left_dpad = off.clone();
        let mut right_dpad = off.clone();
        let mut left_stick_y = off.clone();
        let mut left_stick_x = off.clone();
        let mut right_stick_y = off.clone();
        let mut right_stick_x = off.clone();
        let mut no_bumpers = off.clone();
        let mut left_bumper_only = off.clone();
        let mut right_bumper_only = off.clone();
        let mut both_bumpers = off.clone();
        let mut no_triggers = off.clone();
        let mut left_trigger_only = off.clone();
        let mut right_trigger_only = off.clone();
        let mut both_triggers = off.clone();
        let mut no_modifiers = off;

        if config.attached {
            a = button_trigger(&input, Button::A);
            b = button_trigger(&input, Button::B);
            x = button_trigger(&input, Button::X);
            y = button_trigger(&input, Button::Y);
            left_bumper = button_trigger(&input, Button::LeftBumper);
            right_bumper = button_trigger(&input, Button::RightBumper);
            left_trigger = axis_greater_than(&input, Axis::LeftTrigger, config.triggers_deadzone);
            right_trigger = axis_greater_than(&input, Axis::RightTrigger, config.triggers_deadzone);
            left_stick_click = button_trigger(&input, Button::LeftStick);
            right_stick_click = button_trigger(&input, Button::RightStick);
            start = button_trigger(&input, Button::Start);
            select = button_trigger(&input, Button::Back);
            up_dpad = pov_trigger(&input, 0);
            down_dpad = pov_trigger(&input, 180);
            left_dpad = pov_trigger(&input, 270)
                .or(pov_trigger(&input, 315))
                .or(pov_trigger(&input, 225));
            right_dpad = pov_trigger(&input, 90)
                .or(pov_trigger(&input, 135))
                .or(pov_trigger(&input, 45));
            let stick = |read: fn(&Input) -> f64| {
                let input = input.clone();
                move || read(&input)
            };
            left_stick_y = axis_trigger(
                Threshold::AbsGreater,
                config.left_stick_deadzone,
                stick(Input::left_y),
            );
            left_stick_x = axis_trigger(
                Threshold::AbsGreater,
                config.left_stick_deadzone,
                stick(Input::left_x),
            );
            right_stick_y = axis_trigger(
                Threshold::AbsGreater,
                config.right_stick_deadzone,
                stick(Input::right_y),
            );
            right_stick_x = axis_trigger(
                Threshold::AbsGreater,
                config.right_stick_deadzone,
                stick(Input::right_x),
            );

            no_bumpers = right_bumper.negate().and(left_bumper.negate());
            left_bumper_only = left_bumper.and(right_bumper.negate());
            right_bumper_only = right_bumper.and(left_bumper.negate());
            both_bumpers = right_bumper.and(&left_bumper);
            no_triggers = left_trigger.negate().and(right_trigger.negate());
            left_trigger_only = left_trigger.and(right_trigger.negate());
            right_trigger_only = right_trigger.and(left_trigger.negate());
            both_triggers = left_trigger.and(&right_trigger);
            no_modifiers = no_bumpers.and(&no_triggers);
        }

        let gamepad = Rc::new(Gamepad {
            id: SubsystemId::new(),
            name: config.name.clone(),
            input,
            disconnected_alert,
            a,
            b,
            x,
            y,
            left_bumper,
            right_bumper,
            left_trigger,
            right_trigger,
            left_stick_click,
            right_stick_click,
            start,
            select,
            up_dpad,
            down_dpad,
            left_dpad,
            right_dpad,
            left_stick_y,
            left_stick_x,
            right_stick_y,
            right_stick_x,
            no_bumpers,
            left_bumper_only,
            right_bumper_only,
            both_bumpers,
            no_triggers,
            left_trigger_only,
            right_trigger_only,
            both_triggers,
            no_modifiers,
            stored_left_stick_direction: Cell::new(Rotation2d::ZERO),
            stored_right_stick_direction: Cell::new(Rotation2d::ZERO),
            configured: Cell::new(false),
            printed: Cell::new(false),
            left_stick_curve,
            right_stick_curve,
            triggers_curve,
            teleop: util::teleop(),
            auto_mode: util::auto_mode(),
            test_mode: util::test_mode(),
            disabled: util::disabled(),
        });
        CommandScheduler::get_instance().register_subsystem(gamepad.clone());
        gamepad
    }

    /// Raises the disconnect alert, and the first time the gamepad is seen connected, prints
    /// a confirmation. Called from `periodic`.
    pub fn configure(&self) {
        if self.input.attached {
            self.disconnected_alert.set(!self.is_connected());

            if !self.configured.get() {
                if !self.is_connected() {
                    if !self.printed.get() {
                        telemetry::print(&format!("##{}: GAMEPAD NOT CONNECTED ##", self.name));
                        self.printed.set(true);
                    }
                    return;
                }

                self.configured.set(true);
                telemetry::print(&format!("## {}: gamepad is connected ##", self.name));
            }
        }
    }

    /// Clears the connection state so [`Self::configure`] runs its detection again. Pair it
    /// with clearing the scheduler's buttons.
    pub fn reset_config(&self) {
        self.configured.set(false);
        self.configure();
    }

    /// Direction of the left stick, zero pointing up toward positive Y and 90 degrees
    /// pointing left. The last non-zero direction is kept when the stick is released.
    pub fn get_left_stick_direction(&self) -> Rotation2d {
        stick_direction(
            -self.get_left_x(),
            -self.get_left_y(),
            &self.stored_left_stick_direction,
        )
    }

    /// Direction of the right stick, keeping the last non-zero value when the stick is
    /// released. Unlike [`Self::get_left_stick_direction`], the axis values are not negated.
    pub fn get_right_stick_direction(&self) -> Rotation2d {
        stick_direction(
            self.get_right_x(),
            self.get_right_y(),
            &self.stored_right_stick_direction,
        )
    }

    /// Snaps the left-stick direction to the nearest of 0, plus or minus pi/2 and pi radians.
    pub fn get_left_stick_cardinals(&self) -> f64 {
        snap_to_cardinal(self.get_left_stick_direction().radians())
    }

    /// Snaps the right-stick direction to the nearest of 0, plus or minus pi/2 and pi radians.
    pub fn get_right_stick_cardinals(&self) -> f64 {
        snap_to_cardinal(self.get_right_stick_direction().radians())
    }

    /// Euclidean magnitude of the left stick deflection: up to the square root of 2 before
    /// the curve and scalar, 1.0 after.
    pub fn get_left_stick_magnitude(&self) -> f64 {
        stick_magnitude(-self.get_left_x(), -self.get_left_y())
    }

    pub fn get_right_stick_magnitude(&self) -> f64 {
        stick_magnitude(self.get_right_x(), self.get_right_y())
    }

    /// Cardinal stick angles rotated to match the current alliance's driver viewpoint.
    pub fn choose_cardinal_directions(&self) -> f64 {
        // Java: Blue picks the red-alliance cardinals and Red picks the blue ones.
        if driver_station::alliance().unwrap_or(Alliance::Blue) == Alliance::Blue {
            return self.get_red_alliance_stick_cardinals();
        }
        self.get_blue_alliance_stick_cardinals()
    }

    /// Snaps the right stick to the nearest 45 degree increment, with forward as 0 rad.
    pub fn get_blue_alliance_stick_cardinals(&self) -> f64 {
        blue_alliance_cardinal(self.get_right_stick_direction().radians())
    }

    /// Returns the blue-alliance cardinals rotated by pi, for red-alliance driving.
    pub fn get_red_alliance_stick_cardinals(&self) -> f64 {
        red_alliance_cardinal(self.get_blue_alliance_stick_cardinals())
    }

    pub fn left_y_trigger(&self, t: Threshold, threshold: f64) -> Trigger {
        let input = self.input.clone();
        axis_trigger(t, threshold, move || input.left_y())
    }

    pub fn left_x_trigger(&self, t: Threshold, threshold: f64) -> Trigger {
        let input = self.input.clone();
        axis_trigger(t, threshold, move || input.left_x())
    }

    pub fn right_y_trigger(&self, t: Threshold, threshold: f64) -> Trigger {
        let input = self.input.clone();
        axis_trigger(t, threshold, move || input.right_y())
    }

    pub fn right_x_trigger(&self, t: Threshold, threshold: f64) -> Trigger {
        let input = self.input.clone();
        axis_trigger(t, threshold, move || input.right_x())
    }

    /// Fires when either right-stick axis is at or beyond `threshold` in absolute value.
    pub fn right_stick(&self, threshold: f64) -> Trigger {
        let input = self.input.clone();
        Trigger::new(move || {
            input.right_x().abs() >= threshold || input.right_y().abs() >= threshold
        })
    }

    /// Fires when either left-stick axis is at or beyond `threshold` in absolute value.
    pub fn left_stick(&self, threshold: f64) -> Trigger {
        let input = self.input.clone();
        Trigger::new(move || input.left_x().abs() >= threshold || input.left_y().abs() >= threshold)
    }

    /// Rumble command that holds the given intensities for a fixed time and then stops. It
    /// keeps running while the robot is disabled.
    ///
    /// Intensities are 0.0 to 1.0, `duration_seconds` is in seconds.
    pub fn rumble_command(
        &self,
        left_intensity: f64,
        right_intensity: f64,
        duration_seconds: f64,
    ) -> Command {
        let (start, stop) = (self.input.clone(), self.input.clone());
        Commands::sequence([
            Commands::run_once(
                move || start.rumble(left_intensity, right_intensity),
                &[self.id],
            ),
            Commands::wait_seconds(duration_seconds),
            Commands::run_once(move || stop.rumble(0.0, 0.0), &[self.id]),
        ])
        .ignoring_disable(true)
        .with_name("Gamepad.Rumble")
    }

    /// Rumble command with the same intensity on both motors.
    pub fn rumble_command_uniform(&self, intensity: f64, duration_seconds: f64) -> Command {
        self.rumble_command(intensity, intensity, duration_seconds)
    }

    /// Runs `command` alongside a fixed 0.5 s full-strength rumble, under the same name.
    pub fn rumble_command_with(&self, command: Command) -> Command {
        let name = command.get_name();
        command
            .along_with(self.rumble_command_uniform(1.0, 0.5))
            .with_name(name)
    }

    pub fn is_connected(&self) -> bool {
        self.input.is_connected()
    }

    /// Raw right-trigger axis, 0.0 to 1.0, or 0.0 when the gamepad is not connected.
    pub fn get_right_trigger_axis(&self) -> f64 {
        self.input.axis(XboxController::right_trigger_axis)
    }

    /// Raw left-trigger axis, 0.0 to 1.0, or 0.0 when the gamepad is not connected.
    pub fn get_left_trigger_axis(&self) -> f64 {
        self.input.axis(XboxController::left_trigger_axis)
    }

    /// Right trigger minus left trigger, usable as a single twist axis in [-1, 1].
    pub fn get_twist(&self) -> f64 {
        self.get_right_trigger_axis() - self.get_left_trigger_axis()
    }

    /// Raw left-stick X axis, -1.0 to 1.0, or 0.0 when not connected.
    pub fn get_left_x(&self) -> f64 {
        self.input.left_x()
    }

    /// Raw left-stick Y axis, -1.0 to 1.0, or 0.0 when not connected. Negative is up.
    pub fn get_left_y(&self) -> f64 {
        self.input.left_y()
    }

    /// Raw right-stick X axis, -1.0 to 1.0, or 0.0 when not connected.
    pub fn get_right_x(&self) -> f64 {
        self.input.right_x()
    }

    /// Raw right-stick Y axis, -1.0 to 1.0, or 0.0 when not connected. Negative is up.
    pub fn get_right_y(&self) -> f64 {
        self.input.right_y()
    }

    /// The raw HID device for low-level access, `None` when this gamepad is not attached.
    pub fn get_hid(&self) -> Option<&GenericHid> {
        if !self.input.attached {
            return None;
        }
        self.input.xbox.as_ref().map(XboxController::hid)
    }

    /// The raw HID device for rumble output, `None` unless the gamepad is attached and up.
    pub fn get_rumble_hid(&self) -> Option<&GenericHid> {
        if !self.is_connected() {
            return None;
        }
        self.input.xbox.as_ref().map(XboxController::hid)
    }

    /// Sets both rumble motors immediately. Use [`Self::rumble_command`] for a timed burst.
    ///
    /// Intensities are 0.0 to 1.0.
    pub fn rumble_controller(&self, left_intensity: f64, right_intensity: f64) {
        self.input.rumble(left_intensity, right_intensity);
    }
}

impl Subsystem for Gamepad {
    fn id(&self) -> SubsystemId {
        self.id
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    /// Calls [`Gamepad::configure`] once per robot loop.
    fn periodic(&self) {
        self.configure();
    }
}

/// The three response curves a config describes, with no offset.
fn curves(config: &Config) -> (ExpCurve, ExpCurve, ExpCurve) {
    (
        ExpCurve::with_params(
            config.left_stick_exp,
            0.0,
            config.left_stick_scalar,
            config.left_stick_deadzone,
        ),
        ExpCurve::with_params(
            config.right_stick_exp,
            0.0,
            config.right_stick_scalar,
            config.right_stick_deadzone,
        ),
        ExpCurve::with_params(
            config.triggers_exp,
            0.0,
            config.triggers_scalar,
            config.triggers_deadzone,
        ),
    )
}

/// Direction of a stick, or the stored one while the stick sits at zero.
fn stick_direction(x: f64, y: f64, stored: &Cell<Rotation2d>) -> Rotation2d {
    if x != 0.0 || y != 0.0 {
        // Java: new Rotation2d(y, x), so the angle is measured from the Y axis.
        stored.set(Rotation2d::from_xy(y, x));
    }
    stored.get()
}

fn stick_magnitude(x: f64, y: f64) -> f64 {
    (x * x + y * y).sqrt()
}

/// Snaps an angle in radians to the nearest of 0, plus or minus pi/2 and pi.
fn snap_to_cardinal(stick_angle: f64) -> f64 {
    if stick_angle > -PI / 4.0 && stick_angle <= PI / 4.0 {
        0.0
    } else if stick_angle > PI / 4.0 && stick_angle <= 3.0 * PI / 4.0 {
        PI / 2.0
    } else if stick_angle > 3.0 * PI / 4.0 || stick_angle <= -3.0 * PI / 4.0 {
        PI
    } else {
        -PI / 2.0
    }
}

/// Snaps an angle in radians to the nearest 45 degree increment.
fn blue_alliance_cardinal(stick_angle: f64) -> f64 {
    if stick_angle > -PI / 8.0 && stick_angle <= PI / 8.0 {
        0.0
    } else if stick_angle > PI / 8.0 && stick_angle <= 3.0 * PI / 8.0 {
        PI / 4.0
    } else if stick_angle > 3.0 * PI / 8.0 && stick_angle <= 5.0 * PI / 8.0 {
        PI / 2.0
    } else if stick_angle > 5.0 * PI / 8.0 && stick_angle <= 7.0 * PI / 8.0 {
        3.0 * PI / 4.0
    } else if (-3.0 * PI / 8.0..-PI / 8.0).contains(&stick_angle) {
        -PI / 4.0
    } else if (-5.0 * PI / 8.0..-3.0 * PI / 8.0).contains(&stick_angle) {
        -PI / 2.0
    } else if (-7.0 * PI / 8.0..-5.0 * PI / 8.0).contains(&stick_angle) {
        -3.0 * PI / 4.0
    } else {
        PI
    }
}

fn red_alliance_cardinal(blue: f64) -> f64 {
    if blue > 0.0 { blue - PI } else { blue + PI }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectrum_lib::util::curve::Curve;

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-9, "{expected} vs {actual}");
    }

    fn unattached_input() -> Rc<Input> {
        Rc::new(Input {
            attached: false,
            xbox: None,
        })
    }

    #[test]
    fn config_starts_with_the_java_defaults() {
        let config = Config::new("Pilot", 0);
        assert_eq!(config.name, "Pilot");
        assert_eq!(config.port, 0);
        assert!(!config.attached);
        assert_eq!(config.left_stick_deadzone, 0.001);
        assert_eq!(config.left_stick_exp, 1.0);
        assert_eq!(config.left_stick_scalar, 1.0);
        assert_eq!(config.right_stick_deadzone, 0.001);
        assert_eq!(config.right_stick_exp, 1.0);
        assert_eq!(config.right_stick_scalar, 1.0);
        assert_eq!(config.triggers_deadzone, 0.002);
        assert_eq!(config.triggers_exp, 1.0);
        assert_eq!(config.triggers_scalar, 1.0);
    }

    #[test]
    fn curves_take_their_parameters_from_the_config() {
        let mut config = Config::new("Pilot", 0);
        config.left_stick_deadzone = 0.15;
        config.left_stick_exp = 3.0;
        config.left_stick_scalar = 4.5;
        config.right_stick_exp = 2.0;
        config.triggers_deadzone = 0.0;
        let (left, right, triggers) = curves(&config);
        assert_close(0.15, left.get_deadzone());
        assert_close(3.0, left.get_exp_val());
        assert_close(4.5, left.get_scalar());
        assert_close(0.0, left.get_offset());
        assert_close(2.0, right.get_exp_val());
        assert_close(0.001, right.get_deadzone());
        assert_close(0.0, triggers.get_deadzone());
    }

    #[test]
    fn pilot_style_curve_blanks_the_deadzone_then_curves_the_rest() {
        let mut config = Config::new("Pilot", 0);
        config.left_stick_deadzone = 0.15;
        config.left_stick_exp = 3.0;
        config.left_stick_scalar = 4.0;
        let (left, _, _) = curves(&config);
        // The deadzone is a full width, so the dead radius is 0.075.
        assert_close(0.0, left.calculate(0.07));
        assert_close(0.0, left.calculate(-0.07));
        assert_close(4.0, left.calculate(1.0));
        assert_close(-4.0, left.calculate(-1.0));
        let deadbanded = (1.0 / (1.0 - 0.075)) * (0.5 - 0.075);
        let expected = (3.0_f64.powf(deadbanded) - 1.0) / 2.0 * 4.0;
        assert_close(expected, left.calculate(0.5));
    }

    #[test]
    fn the_pilot_can_retune_a_curve_scalar_through_a_shared_reference() {
        let (left, _, _) = curves(&Config::new("Pilot", 0));
        left.set_scalar(2.5);
        assert_close(2.5, left.calculate(1.0));
    }

    #[test]
    fn threshold_greater_and_less_are_strict() {
        assert!(Threshold::Greater.test(0.6, 0.5));
        assert!(!Threshold::Greater.test(0.5, 0.5));
        assert!(Threshold::Less.test(0.4, 0.5));
        assert!(!Threshold::Less.test(0.5, 0.5));
    }

    #[test]
    fn threshold_abs_greater_checks_the_magnitude_in_both_directions() {
        assert!(Threshold::AbsGreater.test(0.6, 0.5));
        assert!(Threshold::AbsGreater.test(-0.6, 0.5));
        assert!(!Threshold::AbsGreater.test(0.5, 0.5));
        assert!(!Threshold::AbsGreater.test(-0.5, 0.5));
        assert!(!Threshold::AbsGreater.test(0.0, 0.001));
    }

    #[test]
    fn axis_trigger_follows_its_supplier() {
        let value = Rc::new(Cell::new(0.0));
        let source = value.clone();
        let trigger = axis_trigger(Threshold::AbsGreater, 0.15, move || source.get());
        assert!(!trigger.get_as_boolean());
        value.set(-0.2);
        assert!(trigger.get_as_boolean());
        value.set(0.1);
        assert!(!trigger.get_as_boolean());
    }

    #[test]
    fn cardinals_snap_to_the_nearest_quarter_turn() {
        assert_close(0.0, snap_to_cardinal(0.0));
        assert_close(0.0, snap_to_cardinal(PI / 4.0));
        assert_close(PI / 2.0, snap_to_cardinal(PI / 4.0 + 0.01));
        assert_close(PI / 2.0, snap_to_cardinal(3.0 * PI / 4.0));
        assert_close(PI, snap_to_cardinal(3.0 * PI / 4.0 + 0.01));
        assert_close(PI, snap_to_cardinal(-3.0 * PI / 4.0));
        assert_close(-PI / 2.0, snap_to_cardinal(-3.0 * PI / 4.0 + 0.01));
        assert_close(0.0, snap_to_cardinal(-PI / 4.0 + 0.01));
        assert_close(-PI / 2.0, snap_to_cardinal(-PI / 4.0));
    }

    #[test]
    fn blue_alliance_cardinals_snap_to_eighth_turns() {
        assert_close(0.0, blue_alliance_cardinal(0.0));
        assert_close(0.0, blue_alliance_cardinal(PI / 8.0));
        assert_close(PI / 4.0, blue_alliance_cardinal(PI / 4.0));
        assert_close(PI / 2.0, blue_alliance_cardinal(PI / 2.0));
        assert_close(3.0 * PI / 4.0, blue_alliance_cardinal(3.0 * PI / 4.0));
        assert_close(PI, blue_alliance_cardinal(PI));
        assert_close(-PI / 4.0, blue_alliance_cardinal(-PI / 4.0));
        assert_close(-PI / 2.0, blue_alliance_cardinal(-PI / 2.0));
        assert_close(-3.0 * PI / 4.0, blue_alliance_cardinal(-3.0 * PI / 4.0));
        // Java: a stick pointing straight back snaps to pi from either side.
        assert_close(PI, blue_alliance_cardinal(-PI));
        assert_close(PI, blue_alliance_cardinal(-7.0 * PI / 8.0 - 0.01));
    }

    #[test]
    fn red_alliance_cardinals_are_the_blue_ones_turned_by_pi() {
        assert_close(PI, red_alliance_cardinal(0.0));
        assert_close(PI / 4.0 - PI, red_alliance_cardinal(PI / 4.0));
        assert_close(0.0, red_alliance_cardinal(PI));
        assert_close(-PI / 4.0 + PI, red_alliance_cardinal(-PI / 4.0));
        assert_close(0.0, red_alliance_cardinal(-PI));
    }

    #[test]
    fn stick_direction_keeps_the_last_non_zero_value() {
        let stored = Cell::new(Rotation2d::ZERO);
        // Java: new Rotation2d(y, x) measures from the Y axis.
        let up = stick_direction(0.0, 1.0, &stored);
        assert_close(0.0, up.radians());
        let right = stick_direction(1.0, 0.0, &stored);
        assert_close(PI / 2.0, right.radians());
        let released = stick_direction(0.0, 0.0, &stored);
        assert_close(PI / 2.0, released.radians());
    }

    #[test]
    fn stick_direction_before_any_input_is_zero() {
        let stored = Cell::new(Rotation2d::ZERO);
        assert_close(0.0, stick_direction(0.0, 0.0, &stored).radians());
    }

    #[test]
    fn stick_magnitude_is_euclidean() {
        assert_close(0.0, stick_magnitude(0.0, 0.0));
        assert_close(5.0, stick_magnitude(3.0, 4.0));
        assert_close(2.0_f64.sqrt(), stick_magnitude(1.0, -1.0));
        assert_close(1.0, stick_magnitude(-1.0, 0.0));
    }

    #[test]
    fn an_unattached_input_reads_zero_and_presses_nothing() {
        let input = unattached_input();
        assert!(!input.is_connected());
        assert_eq!(input.left_x(), 0.0);
        assert_eq!(input.left_y(), 0.0);
        assert_eq!(input.right_x(), 0.0);
        assert_eq!(input.right_y(), 0.0);
        assert!(!input.raw_button(Button::A));
        assert!(!input.pov_is(0));
        assert_eq!(input.raw_trigger_axis(Axis::LeftTrigger), 0.0);
        input.rumble(1.0, 1.0);
    }

    #[test]
    fn triggers_built_on_an_unattached_input_stay_false() {
        let input = unattached_input();
        assert!(!button_trigger(&input, Button::A).get_as_boolean());
        assert!(!pov_trigger(&input, 0).get_as_boolean());
        assert!(!axis_greater_than(&input, Axis::RightTrigger, 0.1).get_as_boolean());
    }

    #[test]
    fn k_false_is_never_true() {
        assert!(!k_false().get_as_boolean());
        assert!(!k_false().or(k_false()).get_as_boolean());
    }
}
