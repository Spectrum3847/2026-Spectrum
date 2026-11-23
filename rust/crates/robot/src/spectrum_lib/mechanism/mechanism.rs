//! Base for a CTRE TalonFX-driven mechanism: motor creation, cached sensor reads, control
//! mode helpers, threshold triggers, and periodic current reporting.
//!
//! With `Config::attached` false, the mechanism builds no hardware, commands nothing, and
//! every sensor read returns 0.
//!
//! [`Mechanism::get_target`] and [`Mechanism::get_velocity_target_rps`] hold the last
//! setpoint this class sent to the motor, which is what the at-target triggers compare
//! against.
//!
//! Everything here is in CTRE Phoenix 6 units, so rotations and rotations per second rather
//! than degrees, and the mechanism's gearing is
//! `Config::talon_config.feedback.sensor_to_mechanism_ratio`. A subsystem supplies a
//! [`Config`] holding its motor IDs, Talon settings, followers, and rotation bounds.
//!
//! Java's `Mechanism` is an abstract base class. A Rust subsystem holds the `Rc<Mechanism>`
//! that [`Mechanism::new`] returns and delegates to it:
//! - The subsystem registers itself with the scheduler. Java's `this.register()` in the
//!   base constructor registered the subclass, so `new` does not register.
//! - The subsystem implements `Subsystem` with `id()` returning [`Mechanism::id`], so the
//!   commands built here and the subsystem's own commands require the same id.
//! - Java `protected` members are `pub`.
//! - Pass the base [`Config`] by value. A subclass config keeps its extra fields beside it.
//! - Suppliers are `impl Fn() -> f64`. Pass `&supplier` to hand one on without moving it.
//!
//! Java's base class has no simulation code, so there are no sim hooks. A subsystem builds
//! its own sim from [`Mechanism::get_motor`].

use crate::context;
use crate::spectrum_lib::framework::robot_loop;
use crate::spectrum_lib::hardware::{can_config_budget, talon_fx_factory};
use crate::spectrum_lib::util::can_device_id::CanDeviceId;
use crate::spectrum_lib::util::{conversions, java_double};
use commands::{Command, CommandBody, Commands, Subsystem, SubsystemBase, SubsystemId, Trigger};
use phoenix6::controls::{
    DutyCycleOut, DynamicMotionMagicTorqueCurrentFOC, DynamicMotionMagicVoltage,
    MotionMagicTorqueCurrentFOC, MotionMagicVelocityTorqueCurrentFOC, MotionMagicVelocityVoltage,
    MotionMagicVoltage, PositionTorqueCurrentFOC, PositionVoltage, TorqueCurrentFOC,
    VelocityTorqueCurrentFOC, VelocityVoltage, VoltageOut,
};
use phoenix6::signals::{
    FeedbackSensorSourceValue, GravityTypeValue, InvertedValue, MotorAlignmentValue,
    NeutralModeValue,
};
use phoenix6::status_signal::{self, StatusSignal};
use phoenix6::{TalonFX, TalonFXConfiguration};
use std::cell::{Cell, OnceCell, RefCell};
use std::rc::{Rc, Weak};
use telemetry::telemetry;
use wpilib::dashboard::{Alert, AlertType};
use wpilib::{driver_station, timer};
use wpimath::{math_util, units};

// Every signal published here costs CANivore bandwidth on every mechanism motor AND every
// follower. Publishing all eight at 250 Hz put bus utilization at 66-81% in the 2026-09-04
// logs, which is where the stale-frame warnings and the intermittently unresponsive hood
// came from. Only the feedback used for control needs to be fast; everything else is read
// once per 20 ms robot loop for logging, so a faster frame is bandwidth spent on samples
// nobody reads.

/// Rate for a leader's position and velocity. Nothing on the roboRIO consumes them faster
/// than the 50 Hz loop, since Motion Magic closes its loop on the motor itself, so 100 Hz
/// leaves a 2x margin for latency compensation. At 250 Hz the 2026-09-05 logs showed CANivore
/// utilization at 63-77% and the roboRIO CPU at 92-95%.
const CONTROL_SIGNAL_HZ: f64 = 100.0;

/// Rate for the output signals (duty cycle, motor voltage, torque current) of a leader that
/// has followers. A follower mirrors its leader out of the leader's status frames, so CTRE
/// requires these to stay enabled on such a leader. 50 Hz is the rate the followers have run
/// at.
const FOLLOWED_LEADER_OUTPUT_HZ: f64 = 50.0;

/// Rate for the signals nothing controls on: currents and voltage, the output signals of a
/// leader with no followers, and everything on a follower. Read once per loop and logged at
/// 10 Hz, so 20 Hz is already double what gets kept.
const DIAGNOSTIC_SIGNAL_HZ: f64 = 20.0;

/// Rate for device temperature, which changes over seconds.
const TEMPERATURE_SIGNAL_HZ: f64 = 4.0;

/// Leader supply current above which a healthy follower should be pulling its share.
const FOLLOWER_CHECK_LEADER_AMPS: f64 = 8.0;

/// Follower supply current at or below which it is doing no work at all.
const FOLLOWER_DEAD_AMPS: f64 = 0.5;

/// Seconds of accumulated mismatch before it is called out.
///
/// Accumulated rather than continuous, because nothing here stays loaded for long. A launch
/// burst runs one to three seconds, and the longest unbroken stretch of the launcher tower's
/// leader above the threshold in either 2026-09-05 log was 2.78 s, so a rule wanting an
/// uninterrupted fault window would have watched that tower run all day on one motor and
/// said nothing. Replayed against those logs, 1.5 s trips the dead follower in both while the
/// two healthy ones reach 0.00 s and 0.50 s.
const FOLLOWER_DEAD_SECONDS: f64 = 1.5;

/// Ceiling on one loop's contribution, so a disabled stretch cannot bank a fault.
const FOLLOWER_MAX_LOOP_SECONDS: f64 = 0.5;

// Positions in `Mechanism::signals`: the six leader signals, then each follower's supply
// current. One refresh covers them all.
const POSITION: usize = 0;
const VELOCITY: usize = 1;
const VOLTAGE: usize = 2;
const STATOR_CURRENT: usize = 3;
const SUPPLY_CURRENT: usize = 4;
const TEMP: usize = 5;
const FOLLOWER_SUPPLY_START: usize = 6;

/// What a motor has to publish, by its job in the mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SignalRole {
    /// Runs the mechanism alone.
    Leader,
    /// Runs the mechanism with followers mirroring its output frames.
    FollowedLeader,
    /// Mirrors a leader; nothing on the rio reads its feedback.
    Follower,
}

/// How [`Mechanism::log_standard`] logs `<prefix>/RPM`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RpmLog {
    /// Not logged.
    None,
    /// Slow tier, wpilog only.
    Slow,
    /// Slow tier, also published to the dashboard.
    SlowDash,
    /// Every loop, for data that needs it (shot dips, feedforward fits), wpilog only.
    Loop,
    /// Every loop, also published to the dashboard.
    LoopDash,
}

/// Diagnostic log keys, built on the first `log_diagnostics` call for the prefix it was given.
struct DiagnosticKeys {
    prefix: String,
    voltage: String,
    stator_current: String,
    supply_current: String,
    temp: String,
    motor_connected: String,
}

/// `log_standard` keys, built the same way.
struct StandardKeys {
    prefix: String,
    current_command: String,
    rpm: String,
}

pub struct Mechanism {
    me: Weak<Mechanism>,
    base: SubsystemBase,

    /// Leader TalonFX, the motor this mechanism commands. `None` when unattached.
    motor: Option<TalonFX>,

    /// Follower TalonFXs, empty when the mechanism runs on the leader alone.
    follower_motors: Vec<TalonFX>,

    /// Motor IDs, Talon settings, and mechanism parameters. Hold a borrow only inside one
    /// method body.
    pub config: RefCell<Config>,

    /// Raised by the diagnostic commands when a current check fails. Built on first use.
    current_alert: OnceCell<Alert>,

    /// One alert per follower, raised when it stops pulling its share of the load.
    follower_alerts: Vec<Alert>,

    /// Seconds a follower has drawn nothing while its leader was loaded. Any draw at all
    /// resets it.
    follower_mismatch_seconds: RefCell<Vec<f64>>,

    /// FPGA time of the previous follower check, used to measure the mismatch interval.
    follower_check_last_seconds: Cell<f64>,

    /// The last closed-loop position setpoint (in rotations) sent to the motor.
    target: Cell<f64>,

    /// The last closed-loop velocity setpoint (in rotations per second) sent to the motor.
    velocity_target: Cell<f64>,

    /// Every status signal the getters read, refreshed together once per loop. Empty when
    /// unattached.
    signals: RefCell<Vec<StatusSignal>>,

    /// Loop on which `signals` were last refreshed, or `None` for never.
    last_signal_refresh_loop: Cell<Option<u64>>,

    /// Per-follower supply current log keys, built once.
    follower_current_keys: Vec<String>,

    diagnostic_keys: RefCell<Option<DiagnosticKeys>>,
    standard_keys: RefCell<Option<StandardKeys>>,

    /// BatteryLogger channel name for this mechanism, built once to avoid per-loop
    /// concatenation.
    battery_key: String,
}

impl Mechanism {
    /// Builds the leader TalonFX and any followers if `config.attached` is true. Sensor
    /// caches exist either way, so an unattached mechanism reads 0.
    pub fn new(config: Config) -> Rc<Mechanism> {
        let battery_key = format!("Mechanisms/{}", config.name);
        let mut motor = None;
        let mut follower_motors = Vec::new();
        let mut signals = Vec::new();
        let mut follower_current_keys = Vec::new();
        let mut follower_alerts = Vec::new();

        if config.attached {
            let leader = talon_fx_factory::create_config_talon(&config.id, &config.talon_config);
            let has_followers = !config.follower_configs.is_empty();
            configure_status_signals(
                &leader,
                if has_followers {
                    SignalRole::FollowedLeader
                } else {
                    SignalRole::Leader
                },
                config.fast_output_logging,
            );

            let mut follower_supply_signals = Vec::new();
            for follower_config in &config.follower_configs {
                let follower = talon_fx_factory::create_permanent_follower_talon(
                    &follower_config.id,
                    &leader,
                    follower_config.oppose_leader,
                );
                configure_status_signals(&follower, SignalRole::Follower, false);
                follower_supply_signals.push(follower.supply_current(false));
                follower_current_keys
                    .push(format!("Followers/{}/SupplyCurrent", follower_config.name));
                follower_motors.push(follower);
            }

            // The false argument hands back the device's signal without refreshing it. The
            // getters refresh all of them in one Phoenix call per loop.
            signals.push(leader.position(false));
            signals.push(leader.velocity(false));
            signals.push(leader.motor_voltage(false));
            signals.push(leader.stator_current(false));
            signals.push(leader.supply_current(false));
            signals.push(leader.device_temp(false));
            signals.extend(follower_supply_signals);

            for follower_config in &config.follower_configs {
                follower_alerts.push(Alert::new(
                    &format!(
                        "{} (CAN {}) is not drawing current while {} is loaded - check its power leads",
                        follower_config.name,
                        follower_config.id.get_device_number(),
                        config.name
                    ),
                    AlertType::Error,
                ));
            }
            motor = Some(leader);
        }

        let follower_count = follower_motors.len();
        Rc::new_cyclic(|me| Mechanism {
            me: me.clone(),
            base: SubsystemBase::new(config.name.clone()),
            motor,
            follower_motors,
            config: RefCell::new(config),
            current_alert: OnceCell::new(),
            follower_alerts,
            follower_mismatch_seconds: RefCell::new(vec![0.0; follower_count]),
            follower_check_last_seconds: Cell::new(0.0),
            target: Cell::new(0.0),
            velocity_target: Cell::new(0.0),
            signals: RefCell::new(signals),
            last_signal_refresh_loop: Cell::new(None),
            follower_current_keys,
            diagnostic_keys: RefCell::new(None),
            standard_keys: RefCell::new(None),
            battery_key,
        })
    }

    /// Overrides the `attached` flag in the config before the hardware is built.
    ///
    /// `attached` false runs the mechanism in software-only mode.
    pub fn with_attached(mut config: Config, attached: bool) -> Rc<Mechanism> {
        config.attached = attached;
        Self::new(config)
    }

    fn rc(&self) -> Rc<Mechanism> {
        self.me
            .upgrade()
            .expect("Mechanism is only built through Mechanism::new, which returns an Rc")
    }

    pub fn id(&self) -> SubsystemId {
        self.base.id()
    }

    /// The leader motor, `None` when unattached.
    pub fn get_motor(&self) -> Option<&TalonFX> {
        self.motor.as_ref()
    }

    pub fn get_follower_motors(&self) -> &[TalonFX] {
        &self.follower_motors
    }

    /// Where Java would throw a null pointer: the mechanism has no hardware.
    fn motor(&self) -> &TalonFX {
        self.motor
            .as_ref()
            .expect("mechanism is not attached, so it has no motor")
    }

    fn current_alert(&self) -> &Alert {
        self.current_alert
            .get_or_init(|| Alert::new("", AlertType::Warning))
    }

    /// The mechanism's name, as its [`Config`] spells it.
    pub fn get_name(&self) -> String {
        self.config.borrow().name.clone()
    }

    /// True when the mechanism has hardware and should send motor commands.
    pub fn is_attached(&self) -> bool {
        self.config.borrow().attached
    }

    /// True when the leader is attached and its status frames are arriving over CAN. A
    /// mechanism that is commanded but reports 0 V with this false has dropped off the bus,
    /// which is what the hood did intermittently on the 2026-09-04 bench.
    pub fn is_motor_connected(&self) -> bool {
        self.is_attached() && self.motor().is_connected()
    }

    /// Reports the leader's and followers' combined supply current to the battery logger, and
    /// checks each follower for life. Does nothing if the mechanism is not attached.
    pub fn log_battery_usage(&self) {
        if self.is_attached() {
            // get_supply_current refreshes every signal for this loop, followers included.
            let motor_current = self.get_supply_current();
            let mut followers_current = 0.0;
            for i in 0..self.follower_motors.len() {
                let amps = self.signals.borrow()[FOLLOWER_SUPPLY_START + i].value();
                followers_current += amps;
                self.check_follower_alive(i, amps, motor_current);
            }
            context::battery_logger()
                .report_current_usage(&self.battery_key, &[motor_current + followers_current]);
        }
    }

    /// Raises an alert when a follower stops pulling while its leader is clearly working.
    ///
    /// A dead permanent follower is invisible. It shares a gearbox with its leader, so the
    /// mechanism keeps moving on one motor at half the torque and no dashboard looks wrong. On
    /// 2026-09-05 the launcher tower ran the whole day that way with its second motor's power
    /// lead off. Reading the gap between `BatteryLogger/Current/Mechanisms/*` (leader plus
    /// followers) and `LauncherTower/SupplyCurrent` (leader only) across three logs was the
    /// only place the fault showed up, so each follower now publishes its own draw and calls
    /// out its own silence.
    ///
    /// This only catches a dead power path. A follower off the CAN bus entirely never updates
    /// its signal, so it reads a constant zero and trips the same way, which is the right
    /// outcome even though the message names the wrong wire.
    fn check_follower_alive(&self, index: usize, follower_amps: f64, leader_amps: f64) {
        let Some(alert) = self.follower_alerts.get(index) else {
            return;
        };
        // Per-follower keys, so a dead follower is visible on the dashboard at all.
        if telemetry::slow_log_this_loop() {
            telemetry::log_dash_with_unit(
                &self.follower_current_keys[index],
                follower_amps,
                "amps",
            );
        }

        let now = timer::fpga_timestamp();
        // Java: the last check time starts at 0, so the first loop charges the full cap.
        let dt = follower_loop_seconds(now, self.follower_check_last_seconds.get());
        if index == self.follower_motors.len() - 1 {
            self.follower_check_last_seconds.set(now);
        }

        let mut mismatch = self.follower_mismatch_seconds.borrow_mut();
        mismatch[index] = follower_mismatch_step(mismatch[index], dt, follower_amps, leader_amps);
        alert.set(mismatch[index] >= FOLLOWER_DEAD_SECONDS);
    }

    /// The running command's name, or `"none"` when nothing is scheduled.
    pub fn get_current_command_name(&self) -> String {
        match self.get_current_command() {
            Some(command) => command.get_name(),
            None => "none".to_owned(),
        }
    }

    pub fn running_default_command(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_running_default_command())
    }

    fn is_running_default_command(&self) -> bool {
        self.get_current_command() == self.get_default_command()
    }

    /// Last closed-loop position setpoint this class sent, in rotations.
    pub fn get_target(&self) -> f64 {
        self.target.get()
    }

    /// Last closed-loop velocity setpoint this class sent, in rotations per second.
    pub fn get_velocity_target_rps(&self) -> f64 {
        self.velocity_target.get()
    }

    /// Trigger true while the motor is within `tolerance` rotations of [`Self::get_target`].
    pub fn at_target_position(&self, tolerance: impl Fn() -> f64 + 'static) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_at_target_position(&tolerance))
    }

    /// True while the motor is within `tolerance` rotations of [`Self::get_target`].
    pub fn is_at_target_position(&self, tolerance: impl Fn() -> f64) -> bool {
        (self.get_position_rotations() - self.target.get()).abs() < tolerance()
    }

    /// Trigger true while the position is within `tolerance` rotations of `target`.
    pub fn at_rotations(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        near(move || me.get_position_rotations(), target, tolerance)
    }

    /// True while the position is within `tolerance` rotations of `target`.
    pub fn is_at_rotations(&self, target: impl Fn() -> f64, tolerance: impl Fn() -> f64) -> bool {
        (self.get_position_rotations() - target()).abs() < tolerance()
    }

    /// Trigger true while the position is below `target` + `tolerance`, in rotations.
    pub fn below_rotations(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        below(move || me.get_position_rotations(), target, tolerance)
    }

    /// Trigger true while the position is above `target` - `tolerance`, in rotations.
    pub fn above_rotations(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        above(move || me.get_position_rotations(), target, tolerance)
    }

    /// Trigger true while the position is within `tolerance` percent of `target`, both of
    /// max rotations.
    pub fn at_percentage(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        near(move || me.get_position_percentage(), target, tolerance)
    }

    /// Trigger true while the position is below `target` + `tolerance`, as a percentage of
    /// max rotations.
    pub fn below_percentage(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        below(move || me.get_position_percentage(), target, tolerance)
    }

    /// Trigger true while the position is above `target` - `tolerance`, as a percentage of
    /// max rotations.
    pub fn above_percentage(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        above(move || me.get_position_percentage(), target, tolerance)
    }

    /// Trigger true while the position is within `tolerance` degrees of `target`.
    pub fn at_degrees(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        near(move || me.get_position_degrees(), target, tolerance)
    }

    /// Trigger true while the position is below `target` + `tolerance`, in degrees.
    pub fn below_degrees(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        below(move || me.get_position_degrees(), target, tolerance)
    }

    /// Trigger true while the position is above `target` - `tolerance`, in degrees.
    pub fn above_degrees(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        above(move || me.get_position_degrees(), target, tolerance)
    }

    /// Trigger true while the velocity is within `tolerance` RPM of `target`.
    pub fn at_velocity_rpm(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        near(move || me.get_velocity_rpm(), target, tolerance)
    }

    /// Trigger true while the velocity is below `target` + `tolerance` RPM.
    pub fn below_velocity_rpm(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        below(move || me.get_velocity_rpm(), target, tolerance)
    }

    /// Trigger true while the velocity is above `target` - `tolerance` RPM.
    pub fn above_velocity_rpm(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        above(move || me.get_velocity_rpm(), target, tolerance)
    }

    /// Trigger true while the stator current is within `tolerance` amps of `target`.
    pub fn at_current(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        near(move || me.get_stator_current(), target, tolerance)
    }

    /// Trigger true while the stator current is below `target` + `tolerance` amps.
    pub fn below_current(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        below(move || me.get_stator_current(), target, tolerance)
    }

    /// Trigger true while the stator current is above `target` - `tolerance` amps.
    pub fn above_current(
        &self,
        target: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Trigger {
        let me = self.rc();
        above(move || me.get_stator_current(), target, tolerance)
    }

    /// Refreshes every status signal this mechanism reads, once per robot loop, in one
    /// Phoenix call.
    ///
    /// A Phoenix getter such as `motor.stator_current(true)` refreshes its own signal, and
    /// each refresh is a call into the native library. Eight signals per mechanism, read once
    /// a loop, was over a hundred calls a loop across the robot. One `refresh_all` for the lot
    /// is CTRE's recommendation, and every read in a loop then sees the same sample.
    fn refresh_signals_once_per_loop(&self) {
        let loop_count = robot_loop::count();
        if self.last_signal_refresh_loop.get() == Some(loop_count)
            || self.signals.borrow().is_empty()
        {
            return;
        }
        self.last_signal_refresh_loop.set(Some(loop_count));
        status_signal::refresh_all(self.signals.borrow_mut().as_mut_slice());
    }

    /// This loop's value of one of the leader's signals, or 0 when there is no hardware.
    fn signal_value(&self, index: usize) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        self.refresh_signals_once_per_loop();
        self.signals.borrow()[index].value()
    }

    /// Motor stator current, in amps, sampled once per loop.
    pub fn get_stator_current(&self) -> f64 {
        self.signal_value(STATOR_CURRENT)
    }

    /// Motor supply current, in amps, sampled once per loop.
    pub fn get_supply_current(&self) -> f64 {
        self.signal_value(SUPPLY_CURRENT)
    }

    /// Applied motor voltage, in volts, sampled once per loop.
    pub fn get_voltage(&self) -> f64 {
        self.signal_value(VOLTAGE)
    }

    /// Motor temperature in Celsius, sampled once per loop.
    pub fn get_temp(&self) -> f64 {
        self.signal_value(TEMP)
    }

    /// Logs voltage, stator and supply current, temperature and connection under
    /// `prefix/...` at 10 Hz. Subsystems call this from `periodic`, and the log keys are
    /// built on the first call.
    pub fn log_diagnostics(&self, prefix: &str) {
        self.log_diagnostics_dash(prefix, false);
    }

    /// As [`Self::log_diagnostics`], optionally publishing to the dashboard as well.
    ///
    /// `dashboard` true publishes through `telemetry::log_dash`.
    pub fn log_diagnostics_dash(&self, prefix: &str, dashboard: bool) {
        let mut keys = self.diagnostic_keys.borrow_mut();
        if keys.as_ref().is_none_or(|k| k.prefix != prefix) {
            *keys = Some(DiagnosticKeys {
                prefix: prefix.to_owned(),
                voltage: format!("{prefix}/Voltage"),
                stator_current: format!("{prefix}/StatorCurrent"),
                supply_current: format!("{prefix}/SupplyCurrent"),
                temp: format!("{prefix}/Temp"),
                motor_connected: format!("{prefix}/MotorConnected"),
            });
        }
        let Some(keys) = keys.as_ref() else {
            return;
        };
        if !telemetry::slow_log_this_loop() {
            // A feedforward fit needs the applied voltage on every velocity sample, not one
            // in five.
            if self.config.borrow().fast_output_logging {
                telemetry::log_with_unit(&keys.voltage, self.get_voltage(), "volts");
            }
            return;
        }
        if dashboard {
            telemetry::log_dash_with_unit(&keys.voltage, self.get_voltage(), "volts");
            telemetry::log_dash_with_unit(&keys.stator_current, self.get_stator_current(), "amps");
            telemetry::log_dash_with_unit(&keys.supply_current, self.get_supply_current(), "amps");
            telemetry::log_dash_with_unit(&keys.temp, self.get_temp(), "deg_C");
            telemetry::log_dash(&keys.motor_connected, self.is_motor_connected());
        } else {
            telemetry::log_with_unit(&keys.voltage, self.get_voltage(), "volts");
            telemetry::log_with_unit(&keys.stator_current, self.get_stator_current(), "amps");
            telemetry::log_with_unit(&keys.supply_current, self.get_supply_current(), "amps");
            telemetry::log_with_unit(&keys.temp, self.get_temp(), "deg_C");
            telemetry::log(&keys.motor_connected, self.is_motor_connected());
        }
    }

    /// The per-loop logging every mechanism does: battery use, the current command,
    /// [`Self::log_diagnostics_dash`] and RPM.
    ///
    /// `dashboard_diagnostics` is whether the diagnostics also go to the dashboard.
    pub fn log_standard(&self, prefix: &str, dashboard_diagnostics: bool, rpm: RpmLog) {
        self.log_battery_usage();
        {
            let mut keys = self.standard_keys.borrow_mut();
            if keys.as_ref().is_none_or(|k| k.prefix != prefix) {
                *keys = Some(StandardKeys {
                    prefix: prefix.to_owned(),
                    current_command: format!("{prefix}/CurrentCommand"),
                    rpm: format!("{prefix}/RPM"),
                });
            }
        }
        let (current_command_key, rpm_key) = match self.standard_keys.borrow().as_ref() {
            Some(keys) => (keys.current_command.clone(), keys.rpm.clone()),
            None => return,
        };
        telemetry::log(&current_command_key, self.get_current_command_name());
        self.log_diagnostics_dash(prefix, dashboard_diagnostics);
        let slow = rpm == RpmLog::Slow || rpm == RpmLog::SlowDash;
        if rpm == RpmLog::None || (slow && !telemetry::slow_log_this_loop()) {
            return;
        }
        if rpm == RpmLog::SlowDash || rpm == RpmLog::LoopDash {
            telemetry::log_dash_with_unit(&rpm_key, self.get_velocity_rpm(), "RPM");
        } else {
            telemetry::log_with_unit(&rpm_key, self.get_velocity_rpm(), "RPM");
        }
    }

    /// Percentage of `Config::max_rotations` to an absolute rotation count.
    pub fn percent_to_rotations(&self, percent: impl Fn() -> f64) -> f64 {
        (percent() / 100.0) * self.config.borrow().max_rotations
    }

    /// Absolute rotation count to a percentage of `Config::max_rotations`.
    pub fn rotations_to_percent(&self, rotations: impl Fn() -> f64) -> f64 {
        (rotations() / self.config.borrow().max_rotations) * 100.0
    }

    pub fn degrees_to_rotations(&self, degrees: impl Fn() -> f64) -> f64 {
        units::degrees_to_rotations(degrees())
    }

    pub fn rotations_to_degrees(&self, rotations: impl Fn() -> f64) -> f64 {
        units::rotations_to_degrees(rotations())
    }

    /// Motor position in rotations, sampled once per loop.
    pub fn get_position_rotations(&self) -> f64 {
        self.signal_value(POSITION)
    }

    /// Motor position in rotations, projected from the position signal's own timestamp to now
    /// along the velocity signal. A mechanism moving at speed then reads where it is rather
    /// than where it was when the CAN frame left the motor. Returns 0 when unattached.
    pub fn get_latency_compensated_position_rotations(&self) -> f64 {
        if !self.is_attached() || self.signals.borrow().is_empty() {
            return 0.0;
        }
        self.refresh_signals_once_per_loop();
        let signals = self.signals.borrow();
        status_signal::latency_compensated_value(&signals[POSITION], &signals[VELOCITY])
    }

    pub fn get_latency_compensated_position_degrees(&self) -> f64 {
        self.rotations_to_degrees(|| self.get_latency_compensated_position_rotations())
    }

    /// Motor position as a percentage of max rotations, sampled once per loop.
    pub fn get_position_percentage(&self) -> f64 {
        self.rotations_to_percent(|| self.get_position_rotations())
    }

    /// Motor position in degrees, sampled once per loop.
    pub fn get_position_degrees(&self) -> f64 {
        self.rotations_to_degrees(|| self.get_position_rotations())
    }

    /// Motor velocity in RPM, sampled once per loop.
    pub fn get_velocity_rpm(&self) -> f64 {
        conversions::rps_to_rpm(self.signal_value(VELOCITY))
    }

    /// Runs the mechanism at a constant velocity, in closed-loop voltage control.
    pub fn run_velocity(&self, velocity_rpm: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_velocity(|| conversions::rpm_to_rps_supplier(&velocity_rpm)))
            .with_name(format!("{}.runVelocity", self.get_name()))
    }

    /// As [`Self::run_velocity`], in torque current FOC, which requires Phoenix Pro.
    pub fn run_velocity_tc_foc_rpm(&self, velocity_rpm: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || {
            me.set_velocity_torque_current_foc(|| conversions::rpm_to_rps_supplier(&velocity_rpm));
        })
        .with_name(format!("{}.runVelocityTcFocRPM", self.get_name()))
    }

    /// Runs the mechanism at an open-loop percent output, in [-1, 1].
    pub fn run_percentage(&self, percent: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_percent_output(&percent))
            .with_name(format!("{}.runPercentage", self.get_name()))
    }

    /// Applies a constant voltage in volts, bypassing closed-loop control.
    pub fn run_voltage(&self, voltage: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_voltage_output(&voltage))
            .with_name(format!("{}.runVoltage", self.get_name()))
    }

    /// Applies a constant voltage in volts and ignores the software limit switches, so it can
    /// drive the mechanism past its configured travel.
    pub fn run_voltage_no_soft_limit(&self, voltage: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_voltage_output_no_soft_limit(&voltage))
            .with_name(format!("{}.runVoltageNoSoftLimit", self.get_name()))
    }

    /// Holds a torque current in amps through FOC, which requires Phoenix Pro.
    pub fn run_torque_current_foc(&self, current: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_torque_current_foc(&current))
            .with_name(format!("{}.runTorqueCurrentFoc", self.get_name()))
    }

    /// Moves the mechanism to a position in rotations with Motion Magic, in torque current
    /// FOC, which requires Phoenix Pro.
    pub fn move_to_rotations(&self, rotations: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_mm_position_foc(&rotations))
            // Java: the name differs from the method name.
            .with_name(format!("{}.runPoseRevolutions", self.get_name()))
    }

    /// As [`Self::move_to_rotations`], with the target as a percentage of max rotations.
    pub fn move_to_percentage(&self, percent: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_mm_position_foc(|| me.percent_to_rotations(&percent)))
            // Java: the name differs from the method name.
            .with_name(format!("{}.runPosePercentage", self.get_name()))
    }

    /// As [`Self::move_to_rotations`], with the target in degrees.
    pub fn move_to_degrees(&self, degrees: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_mm_position_foc(|| me.degrees_to_rotations(&degrees)))
            // Java: the name differs from the method name.
            .with_name(format!("{}.runPoseDegrees", self.get_name()))
    }

    /// Same as [`Self::move_to_rotations`], which is the clearer name. Requires Phoenix Pro.
    pub fn run_foc_rotations(&self, rotations: impl Fn() -> f64 + 'static) -> Command {
        let me = self.rc();
        self.run(move || me.set_mm_position_foc(&rotations))
            // Java: the name differs from the method name.
            .with_name(format!("{}.runFOCPosition", self.get_name()))
    }

    pub fn run_stop(&self) -> Command {
        let me = self.rc();
        self.run(move || me.stop())
            .with_name(format!("{}.runStop", self.get_name()))
    }

    /// Coasts while the command runs, then returns to brake. Safe to run while the robot is
    /// disabled.
    pub fn coast_mode(&self) -> Command {
        let (coast, brake) = (self.rc(), self.rc());
        self.start_end(
            move || coast.set_brake_mode(false),
            move || brake.set_brake_mode(true),
        )
        .ignoring_disable(true)
        .with_name(format!("{}.coastMode", self.get_name()))
    }

    /// Returns to brake if the mechanism is coasting. Safe to run while the robot is
    /// disabled.
    pub fn ensure_brake_mode(&self) -> Command {
        let (brake, coasting) = (self.rc(), self.rc());
        self.run_once(move || brake.set_brake_mode(true))
            .only_if(move || {
                let config = coasting.config.borrow();
                config.attached
                    && config.talon_config.motor_output.neutral_mode == NeutralModeValue::Coast
            })
            .ignoring_disable(true)
            .with_name(format!("{}.ensureBrakeMode", self.get_name()))
    }

    /// Applies new supply and stator current limits, in amps, to the mechanism.
    pub fn run_current_limits(
        &self,
        supply_limit: impl Fn() -> f64 + 'static,
        stator_limit: impl Fn() -> f64 + 'static,
    ) -> Command {
        let me = self.rc();
        Commands::run_once(
            move || me.set_current_limits(&supply_limit, &stator_limit),
            &[],
        )
    }

    pub fn set_current_limits(
        &self,
        supply_limit: impl Fn() -> f64,
        stator_limit: impl Fn() -> f64,
    ) {
        self.apply_current_limit(supply_limit, stator_limit);
    }

    /// Stops the motor output. Does nothing if the mechanism is not attached.
    pub fn stop(&self) {
        if self.is_attached() {
            self.motor().stop_motor();
        }
    }

    /// Sets the mechanism's reported position to zero (tares the motor encoder). Does nothing
    /// if the mechanism is not attached.
    pub fn tare_motor(&self) {
        if self.is_attached() {
            self.set_motor_position(|| 0.0);
        }
    }

    /// Writes the motor's internal position register, in rotations, without moving the motor.
    pub fn set_motor_position(&self, rotations: impl Fn() -> f64) {
        if self.is_attached() {
            self.motor().set_position(rotations());
        }
    }

    /// Velocity in rotations per second, via Motion Magic in torque current FOC (Phoenix Pro).
    pub fn set_mm_velocity_foc(&self, velocity_rps: impl Fn() -> f64) {
        if self.is_attached() {
            let velocity = velocity_rps();
            self.velocity_target.set(velocity);
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.mm_velocity_foc_request(velocity));
        }
    }

    /// Velocity in rotations per second, in torque current FOC (requires Phoenix Pro).
    pub fn set_velocity_torque_current_foc(&self, velocity_rps: impl Fn() -> f64) {
        if self.is_attached() {
            let velocity = velocity_rps();
            self.velocity_target.set(velocity);
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.velocity_torque_current_foc_request(velocity));
        }
    }

    /// As [`Self::set_velocity_torque_current_foc`], taking RPM. Requires Phoenix Pro.
    pub fn set_velocity_tcfoc_rpm(&self, velocity_rpm: impl Fn() -> f64) {
        self.set_velocity_torque_current_foc(|| conversions::rpm_to_rps(velocity_rpm()));
    }

    /// Velocity in rotations per second, in closed-loop voltage control.
    pub fn set_velocity(&self, velocity_rps: impl Fn() -> f64) {
        if self.is_attached() {
            let velocity = velocity_rps();
            self.velocity_target.set(velocity);
            let mut config = self.config.borrow_mut();
            self.motor().set_control(config.velocity_request(velocity));
        }
    }

    /// As [`Self::set_velocity`], taking RPM.
    pub fn set_velocity_rpm(&self, velocity_rpm: impl Fn() -> f64) {
        self.set_velocity(|| conversions::rpm_to_rps(velocity_rpm()));
    }

    /// Position control with voltage compensation and no velocity feedforward.
    pub fn set_position(&self, rotations: impl Fn() -> f64) {
        self.set_position_with_velocity(rotations, || 0.0);
    }

    /// Position control with an explicit velocity feedforward. This is the right primitive
    /// for a continuously moving setpoint, such as a turret tracking a target while the robot
    /// drives, where a profiled motion would leave steady-state lag.
    ///
    /// `rotations` is the target position in mechanism rotations. `velocity_rps` is the
    /// feedforward in mechanism rotations per second.
    pub fn set_position_with_velocity(
        &self,
        rotations: impl Fn() -> f64,
        velocity_rps: impl Fn() -> f64,
    ) {
        if self.is_attached() {
            let target = rotations();
            self.target.set(target);
            let velocity = velocity_rps();
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.position_request(target, velocity));
        }
    }

    /// Position in rotations, via Motion Magic in torque current FOC (requires Phoenix Pro).
    pub fn set_mm_position_foc(&self, rotations: impl Fn() -> f64) {
        if self.is_attached() {
            let target = rotations();
            self.target.set(target);
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.mm_position_foc_request(target));
        }
    }

    /// As [`Self::set_position_with_velocity`], in `PositionTorqueCurrentFOC`, which requires
    /// Phoenix Pro.
    pub fn set_position_foc_with_velocity(
        &self,
        rotations: impl Fn() -> f64,
        velocity_rps: impl Fn() -> f64,
    ) {
        if self.is_attached() {
            let target = rotations();
            self.target.set(target);
            let velocity = velocity_rps();
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.position_foc_request(target, velocity));
        }
    }

    /// Dynamic Motion Magic in torque current FOC, which requires Phoenix Pro. The trajectory
    /// parameters can change every loop cycle.
    ///
    /// `velocity` is the cruise velocity in rotations per second, `acceleration` is in
    /// rotations per second squared, `jerk` is in rotations per second cubed.
    pub fn set_dyn_mm_position_foc(
        &self,
        rotations: impl Fn() -> f64,
        velocity: impl Fn() -> f64,
        acceleration: impl Fn() -> f64,
        jerk: impl Fn() -> f64,
    ) {
        if self.is_attached() {
            let target = rotations();
            self.target.set(target);
            let (velocity, acceleration, jerk) = (velocity(), acceleration(), jerk());
            let mut config = self.config.borrow_mut();
            self.motor().set_control(config.dyn_mm_position_foc_request(
                target,
                velocity,
                acceleration,
                jerk,
            ));
        }
    }

    /// Dynamic Motion Magic with voltage compensation. The trajectory parameters can change
    /// every loop cycle.
    ///
    /// `velocity` is the cruise velocity in rotations per second, `acceleration` is in
    /// rotations per second squared, `jerk` is in rotations per second cubed.
    pub fn set_dyn_mm_position_voltage(
        &self,
        rotations: impl Fn() -> f64,
        velocity: impl Fn() -> f64,
        acceleration: impl Fn() -> f64,
        jerk: impl Fn() -> f64,
    ) {
        if self.is_attached() {
            let target = rotations();
            self.target.set(target);
            let (velocity, acceleration, jerk) = (velocity(), acceleration(), jerk());
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.dyn_mm_position_voltage_request(
                    target,
                    velocity,
                    acceleration,
                    jerk,
                ));
        }
    }

    /// Position in rotations, via Motion Magic in voltage control on slot 0.
    pub fn set_mm_position(&self, rotations: impl Fn() -> f64) {
        self.set_mm_position_slot(rotations, 0);
    }

    /// Position in rotations, via Motion Magic in voltage control on a chosen gain slot.
    ///
    /// `slot` is the gain slot, 0, 1 or 2.
    pub fn set_mm_position_slot(&self, rotations: impl Fn() -> f64, slot: i32) {
        if self.is_attached() {
            let target = rotations();
            self.target.set(target);
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.mm_position_voltage_request(target, slot));
        }
    }

    /// Open-loop percent output, in [-1, 1]. The applied voltage is percent times
    /// `Config::voltage_comp_saturation`.
    pub fn set_percent_output(&self, percent: impl Fn() -> f64) {
        self.set_voltage_output(|| self.config.borrow().voltage_comp_saturation * percent());
    }

    /// Open-loop voltage control. Applies the requested volts with no scaling.
    pub fn set_voltage_output(&self, voltage: impl Fn() -> f64) {
        self.voltage_out(voltage, false);
    }

    /// Open-loop voltage control that ignores the software limit switches, so it can drive
    /// the mechanism past its configured travel.
    pub fn set_voltage_output_no_soft_limit(&self, voltage: impl Fn() -> f64) {
        self.voltage_out(voltage, true);
    }

    /// Sends the shared voltage request. The soft-limit flag is set on every call because the
    /// request object is reused.
    fn voltage_out(&self, voltage: impl Fn() -> f64, ignore_soft_limits: bool) {
        if self.is_attached() {
            let output = voltage();
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.voltage_request(output, ignore_soft_limits));
        }
    }

    /// Torque current in amps through FOC, which requires Phoenix Pro.
    pub fn set_torque_current_foc(&self, current: impl Fn() -> f64) {
        if self.is_attached() {
            let output = current();
            let mut config = self.config.borrow_mut();
            self.motor()
                .set_control(config.torque_current_foc_request(output));
        }
    }

    /// Sets brake or coast mode and applies it to the hardware immediately.
    pub fn set_brake_mode(&self, is_in_brake: bool) {
        if self.is_attached() {
            let mut config = self.config.borrow_mut();
            config.config_neutral_brake_mode(is_in_brake);
            config.apply_talon_config(self.motor());
        }
    }

    /// Enables or disables the reverse software limit and applies the change immediately. The
    /// threshold comes from the current configuration.
    pub fn toggle_reverse_soft_limit(&self, enabled: bool) {
        if self.is_attached() {
            let mut config = self.config.borrow_mut();
            let threshold = config
                .talon_config
                .software_limit_switch
                .reverse_soft_limit_threshold;
            config.config_reverse_soft_limit(threshold, enabled);
            config.apply_talon_config(self.motor());
        }
    }

    /// Enables or disables a forward and reverse torque current limit and applies the change
    /// immediately. Disabling resets the peak to plus or minus 300 A, which is effectively
    /// unlimited.
    ///
    /// `enabled_limit` is the torque current limit in amps, applied when `enabled` is true.
    pub fn toggle_torque_current_limit(&self, enabled_limit: impl Fn() -> f64, enabled: bool) {
        if self.is_attached() {
            let limits = if enabled {
                Some((enabled_limit(), enabled_limit(), enabled_limit()))
            } else {
                None
            };
            let mut config = self.config.borrow_mut();
            match limits {
                Some((forward, reverse, stator)) => {
                    config.config_forward_torque_current_limit(forward);
                    config.config_reverse_torque_current_limit(-reverse);
                    config.config_stator_current_limit(stator, true);
                }
                None => {
                    // Java: the stator limit that the enabled branch turns on is left on.
                    config.config_forward_torque_current_limit(300.0);
                    config.config_reverse_torque_current_limit(-300.0);
                }
            }
            config.apply_talon_config(self.motor());
        }
    }

    /// Enables or disables the supply current limit and applies the change immediately.
    ///
    /// `enabled_limit` is the supply current limit in amps.
    pub fn toggle_supply_current_limit(&self, enabled_limit: impl Fn() -> f64, enabled: bool) {
        if self.is_attached() {
            let limit = enabled_limit();
            let mut config = self.config.borrow_mut();
            config.config_supply_current_limit(limit, enabled);
            config.apply_talon_config(self.motor());
        }
    }

    /// Applies new supply and stator current limits, in amps, but only when they differ from
    /// the configured ones. The apply is retried while [`can_config_budget::max_attempts`]
    /// allows it.
    pub fn apply_current_limit(
        &self,
        supply_limit: impl Fn() -> f64,
        stator_limit: impl Fn() -> f64,
    ) {
        if self.is_attached() {
            let (supply, stator) = (supply_limit(), stator_limit());
            let mut config = self.config.borrow_mut();
            if config.talon_config.current_limits.stator_current_limit != stator
                || config.talon_config.current_limits.supply_current_limit != supply
            {
                config.config_supply_current_limit(supply.abs(), true);
                config.config_stator_current_limit(stator.abs(), true);
                config.config_forward_torque_current_limit(stator.abs());
                config.config_reverse_torque_current_limit(-stator.abs());
                let mut attempt = 0;
                while attempt < can_config_budget::max_attempts() {
                    let result = can_config_budget::run(&config.name, |timeout| {
                        self.motor()
                            .configurator()
                            .apply_with_timeout(&config.talon_config, timeout)
                    });
                    if result.is_ok() {
                        break;
                    }
                    println!("Could not apply config changes to {}'s motor ", config.name);
                    attempt += 1;
                }
            }
        }
    }

    /// Averages the stator current over the command's runtime and raises a warning [`Alert`]
    /// if the average is further than `tolerance` from `expected_current`.
    ///
    /// `expected_current` is the expected average stator current in amps. `tolerance` is the
    /// largest acceptable deviation in amps.
    pub fn check_avg_current(
        &self,
        expected_current: impl Fn() -> f64 + 'static,
        tolerance: impl Fn() -> f64 + 'static,
    ) -> Command {
        let alert_text = format!("{} AvgCurrent Error", self.get_name());
        // Java: an anonymous class, so the command's name is empty.
        Command::builder(
            "",
            AvgCurrentCheck {
                mechanism: self.rc(),
                expected_current: Box::new(expected_current),
                tolerance: Box::new(tolerance),
                total_current: 0.0,
                count: 0,
                alert_text,
            },
        )
        .build()
    }

    /// Tracks the peak stator current and raises a warning [`Alert`] if it exceeds
    /// `expected_current`.
    ///
    /// `expected_current` is the highest acceptable peak stator current in amps.
    pub fn check_max_current(&self, expected_current: impl Fn() -> f64 + 'static) -> Command {
        self.check_peak_current(expected_current, true, " MaxCurrent Error Expected: ")
    }

    /// Tracks the peak stator current and raises a warning [`Alert`] if it never reaches
    /// `expected_current`, which checks that a mechanism drew at least its expected load.
    ///
    /// `expected_current` is the lowest acceptable peak stator current in amps.
    pub fn check_min_threshold_current(
        &self,
        expected_current: impl Fn() -> f64 + 'static,
    ) -> Command {
        self.check_peak_current(
            expected_current,
            false,
            " Current Error Expected at least: ",
        )
    }

    /// Tracks the peak stator current and raises the current alert if it lands on the wrong
    /// side of expected.
    fn check_peak_current(
        &self,
        expected_current: impl Fn() -> f64 + 'static,
        fail_above: bool,
        message: &'static str,
    ) -> Command {
        // Java: an anonymous class, so the command's name is empty.
        Command::builder(
            "",
            PeakCurrentCheck {
                mechanism: self.rc(),
                expected_current: Box::new(expected_current),
                fail_above,
                message,
                max_current: 0.0,
            },
        )
        .build()
    }
}

impl Subsystem for Mechanism {
    fn id(&self) -> SubsystemId {
        self.base.id()
    }

    fn name(&self) -> String {
        self.get_name()
    }
}

struct AvgCurrentCheck {
    mechanism: Rc<Mechanism>,
    expected_current: Box<dyn Fn() -> f64>,
    tolerance: Box<dyn Fn() -> f64>,
    total_current: f64,
    count: i32,
    alert_text: String,
}

impl CommandBody for AvgCurrentCheck {
    fn initialize(&mut self) {
        self.total_current = 0.0;
        self.count = 0;
    }

    fn execute(&mut self) {
        self.total_current += self.mechanism.get_stator_current();
        self.count += 1;
    }

    fn end(&mut self, _interrupted: bool) {
        let avg_current = self.total_current / f64::from(self.count);
        if (avg_current - (self.expected_current)()).abs() > (self.tolerance)() {
            let alert = self.mechanism.current_alert();
            alert.set_text(&format!(
                "{} Expected: {} Actual: {}",
                self.alert_text,
                java_double::to_string((self.expected_current)()),
                java_double::to_string(avg_current)
            ));
            alert.set(true);
        }
    }
}

struct PeakCurrentCheck {
    mechanism: Rc<Mechanism>,
    expected_current: Box<dyn Fn() -> f64>,
    fail_above: bool,
    message: &'static str,
    max_current: f64,
}

impl CommandBody for PeakCurrentCheck {
    fn initialize(&mut self) {
        self.max_current = 0.0;
    }

    fn execute(&mut self) {
        self.max_current = self.max_current.max(self.mechanism.get_stator_current());
    }

    fn end(&mut self, _interrupted: bool) {
        let expected = (self.expected_current)();
        let failed = if self.fail_above {
            self.max_current > expected
        } else {
            self.max_current < expected
        };
        if failed {
            let alert = self.mechanism.current_alert();
            alert.set_text(&format!(
                "{}{}{} Actual: {}",
                self.mechanism.get_name(),
                self.message,
                java_double::to_string(expected),
                java_double::to_string(self.max_current)
            ));
            alert.set(true);
        }
    }
}

/// Sets the status frame rates this mechanism relies on, then lets Phoenix disable
/// everything else. A follower's frames cost the same bandwidth as the leader's, so it runs
/// at the diagnostic rate throughout. A leader with followers keeps the output frames they
/// mirror. A leader whose feedforward is fit from logs (`Config::fast_output_logging`)
/// publishes its output at the control rate, so the logged voltage lines up with the logged
/// velocity.
fn configure_status_signals(talon: &TalonFX, role: SignalRole, fast_output: bool) {
    let control_hz = if role == SignalRole::Follower {
        DIAGNOSTIC_SIGNAL_HZ
    } else {
        CONTROL_SIGNAL_HZ
    };
    let output_hz = if fast_output {
        CONTROL_SIGNAL_HZ
    } else if role == SignalRole::FollowedLeader {
        FOLLOWED_LEADER_OUTPUT_HZ
    } else {
        DIAGNOSTIC_SIGNAL_HZ
    };
    // Setting a frequency needs no read, so the signals are not refreshed first.
    status_signal::set_update_frequency_for_all(
        control_hz,
        &[talon.position(false), talon.velocity(false)],
    );
    status_signal::set_update_frequency_for_all(
        output_hz,
        &[
            talon.duty_cycle(false),
            talon.motor_voltage(false),
            talon.torque_current(false),
        ],
    );
    status_signal::set_update_frequency_for_all(
        DIAGNOSTIC_SIGNAL_HZ,
        &[talon.stator_current(false), talon.supply_current(false)],
    );
    talon
        .device_temp(false)
        .set_update_frequency(TEMPERATURE_SIGNAL_HZ);
    // A long run of per-signal config calls, and only ever an optimization. On a dead bus it
    // is pure boot latency, so it is the first thing dropped once the budget is spent.
    if !can_config_budget::exhausted() {
        talon.optimize_bus_utilization();
    }
}

/// One loop's contribution to a follower's mismatch time, capped so a stretch with the
/// robot disabled cannot bank a fault.
fn follower_loop_seconds(now: f64, last_check: f64) -> f64 {
    math_util::clamp(now - last_check, 0.0, FOLLOWER_MAX_LOOP_SECONDS)
}

/// The follower's mismatch time after one loop. Any draw at all means the power path is
/// intact, so it forgets what came before. Otherwise it accumulates while the leader is
/// loaded.
fn follower_mismatch_step(seconds: f64, dt: f64, follower_amps: f64, leader_amps: f64) -> f64 {
    if follower_amps > FOLLOWER_DEAD_AMPS {
        0.0
    } else if leader_amps > FOLLOWER_CHECK_LEADER_AMPS {
        seconds + dt
    } else {
        seconds
    }
}

/// Active while `value` is within `tolerance` of `target`.
fn near(
    value: impl Fn() -> f64 + 'static,
    target: impl Fn() -> f64 + 'static,
    tolerance: impl Fn() -> f64 + 'static,
) -> Trigger {
    Trigger::new(move || (value() - target()).abs() < tolerance())
}

/// Active while `value` is below `target + tolerance`.
fn below(
    value: impl Fn() -> f64 + 'static,
    target: impl Fn() -> f64 + 'static,
    tolerance: impl Fn() -> f64 + 'static,
) -> Trigger {
    Trigger::new(move || value() < target() + tolerance())
}

/// Active while `value` is above `target - tolerance`.
fn above(
    value: impl Fn() -> f64 + 'static,
    target: impl Fn() -> f64 + 'static,
    tolerance: impl Fn() -> f64 + 'static,
) -> Trigger {
    Trigger::new(move || value() > target() - tolerance())
}

/// A TalonFX follower that mirrors the leader's output. Set `oppose_leader` to
/// [`MotorAlignmentValue::Opposed`] when the motor is mounted in the opposite direction and
/// has to spin in reverse to move the mechanism the same way.
#[derive(Debug, Clone)]
pub struct FollowerConfig {
    /// Name for this follower, used in its alerts and log keys.
    pub name: String,
    pub id: CanDeviceId,
    pub attached: bool,
    /// Alignment of the follower relative to the leader. Use `Opposed` when the follower is
    /// physically mounted in the opposite direction.
    pub oppose_leader: MotorAlignmentValue,
}

impl FollowerConfig {
    /// `canbus` is the CAN bus name, such as `"rio"` or `"canivore"`.
    pub fn new(
        name: impl Into<String>,
        id: i32,
        canbus: &str,
        oppose_leader: MotorAlignmentValue,
    ) -> Self {
        Self {
            name: name.into(),
            id: CanDeviceId::new(id, canbus),
            attached: true,
            oppose_leader,
        }
    }
}

/// TalonFX hardware configuration, the control request objects, and the mechanism-level
/// parameters (gear ratio, soft limits, PID/FF gains, Motion Magic profile, current limits).
///
/// Call the `config_*` helpers in the subsystem's config constructor, and pass the result to
/// [`Mechanism::new`]. Lombok getters and setters on the Java class are public fields.
#[derive(Debug, Clone)]
pub struct Config {
    /// Name for this mechanism, used in log keys and alerts.
    pub name: String,

    /// False builds no hardware, which is how the sim runs a real mechanism config.
    pub attached: bool,

    /// Publish the leader's output signals (duty cycle, motor voltage, torque current) at the
    /// control rate and log its applied voltage on every loop instead of at 10 Hz. Set it on
    /// the mechanisms whose feedforward is fit from logs: kS and kV survive a 10 Hz voltage,
    /// kA does not, because the acceleration it multiplies is gone between one sample and the
    /// next. Costs one status frame at 100 Hz on this motor and one double per loop in the
    /// log; the followers are untouched.
    pub fast_output_logging: bool,

    pub id: CanDeviceId,

    /// Applied to the leader motor on startup.
    pub talon_config: TalonFXConfiguration,

    /// Leader plus followers.
    pub num_motors: i32,

    /// Saturation for [`Mechanism::set_percent_output`], 12 V by default.
    pub voltage_comp_saturation: f64,

    /// Bottom of the mechanism's range, in rotations, for the percentage helpers.
    pub min_rotations: f64,

    /// Top of the mechanism's range, in rotations, for the percentage helpers.
    pub max_rotations: f64,

    /// Empty when the mechanism runs on the leader alone.
    pub follower_configs: Vec<FollowerConfig>,

    // Pre-built control requests, reused each loop to keep the loop allocation free.
    pub mm_velocity_foc: MotionMagicVelocityTorqueCurrentFOC,
    pub mm_position_foc: MotionMagicTorqueCurrentFOC,
    pub position_torque_foc: PositionTorqueCurrentFOC,
    pub dynamic_mm_position_foc: DynamicMotionMagicTorqueCurrentFOC,
    pub dynamic_motion_magic_voltage: DynamicMotionMagicVoltage,
    pub mm_velocity_voltage: MotionMagicVelocityVoltage,
    pub mm_position_voltage: MotionMagicVoltage,
    pub mm_position_voltage_slot: MotionMagicVoltage,
    pub voltage_control: VoltageOut,
    pub velocity_control: VelocityVoltage,
    pub position_control: PositionVoltage,
    pub velocity_torque_current_foc: VelocityTorqueCurrentFOC,
    pub torque_current_foc: TorqueCurrentFOC,
    /// Percent output control. Prefer `voltage_control` in most cases.
    pub percent_output: DutyCycleOut,
}

impl Config {
    /// Leaves the Talon settings at their defaults, with the hardware limit switches
    /// disabled.
    ///
    /// `canbus` is the CAN bus name, such as `"rio"` or `"canivore"`.
    pub fn new(name: impl Into<String>, id: i32, canbus: &str) -> Self {
        let mut talon_config = TalonFXConfiguration::default();
        talon_config.hardware_limit_switch.forward_limit_enable = false;
        talon_config.hardware_limit_switch.reverse_limit_enable = false;

        Self {
            name: name.into(),
            attached: true,
            fast_output_logging: false,
            id: CanDeviceId::new(id, canbus),
            talon_config,
            num_motors: 1,
            voltage_comp_saturation: 12.0,
            min_rotations: 0.0,
            max_rotations: 1.0,
            follower_configs: Vec::new(),
            mm_velocity_foc: MotionMagicVelocityTorqueCurrentFOC::new(0.0),
            mm_position_foc: MotionMagicTorqueCurrentFOC::new(0.0),
            position_torque_foc: PositionTorqueCurrentFOC::new(0.0),
            dynamic_mm_position_foc: DynamicMotionMagicTorqueCurrentFOC::new(0.0, 0.0, 0.0),
            dynamic_motion_magic_voltage: DynamicMotionMagicVoltage::new(0.0, 0.0, 0.0),
            mm_velocity_voltage: MotionMagicVelocityVoltage::new(0.0),
            mm_position_voltage: MotionMagicVoltage::new(0.0),
            mm_position_voltage_slot: MotionMagicVoltage::new(0.0).with_slot(1),
            voltage_control: VoltageOut::new(0.0),
            velocity_control: VelocityVoltage::new(0.0),
            position_control: PositionVoltage::new(0.0),
            velocity_torque_current_foc: VelocityTorqueCurrentFOC::new(0.0),
            torque_current_foc: TorqueCurrentFOC::new(0.0),
            percent_output: DutyCycleOut::new(0.0),
        }
    }

    // The request builders below set the fields a control mode uses on the shared request
    // object and hand it back to send. Java chained `withX` calls on the same objects.

    pub(crate) fn mm_velocity_foc_request(
        &mut self,
        velocity: f64,
    ) -> &MotionMagicVelocityTorqueCurrentFOC {
        self.mm_velocity_foc.velocity = velocity;
        &self.mm_velocity_foc
    }

    pub(crate) fn velocity_torque_current_foc_request(
        &mut self,
        velocity: f64,
    ) -> &VelocityTorqueCurrentFOC {
        self.velocity_torque_current_foc.velocity = velocity;
        &self.velocity_torque_current_foc
    }

    pub(crate) fn velocity_request(&mut self, velocity: f64) -> &VelocityVoltage {
        self.velocity_control.velocity = velocity;
        &self.velocity_control
    }

    pub(crate) fn position_request(&mut self, position: f64, velocity: f64) -> &PositionVoltage {
        self.position_control.position = position;
        self.position_control.velocity = velocity;
        &self.position_control
    }

    pub(crate) fn mm_position_foc_request(
        &mut self,
        position: f64,
    ) -> &MotionMagicTorqueCurrentFOC {
        self.mm_position_foc.position = position;
        &self.mm_position_foc
    }

    pub(crate) fn position_foc_request(
        &mut self,
        position: f64,
        velocity: f64,
    ) -> &PositionTorqueCurrentFOC {
        self.position_torque_foc.position = position;
        self.position_torque_foc.velocity = velocity;
        &self.position_torque_foc
    }

    pub(crate) fn dyn_mm_position_foc_request(
        &mut self,
        position: f64,
        velocity: f64,
        acceleration: f64,
        jerk: f64,
    ) -> &DynamicMotionMagicTorqueCurrentFOC {
        let request = &mut self.dynamic_mm_position_foc;
        request.position = position;
        request.velocity = velocity;
        request.acceleration = acceleration;
        request.jerk = jerk;
        &self.dynamic_mm_position_foc
    }

    pub(crate) fn dyn_mm_position_voltage_request(
        &mut self,
        position: f64,
        velocity: f64,
        acceleration: f64,
        jerk: f64,
    ) -> &DynamicMotionMagicVoltage {
        let request = &mut self.dynamic_motion_magic_voltage;
        request.position = position;
        request.velocity = velocity;
        request.acceleration = acceleration;
        request.jerk = jerk;
        &self.dynamic_motion_magic_voltage
    }

    pub(crate) fn mm_position_voltage_request(
        &mut self,
        position: f64,
        slot: i32,
    ) -> &MotionMagicVoltage {
        self.mm_position_voltage_slot.slot = slot;
        self.mm_position_voltage_slot.position = position;
        &self.mm_position_voltage_slot
    }

    pub(crate) fn voltage_request(
        &mut self,
        output: f64,
        ignore_software_limits: bool,
    ) -> &VoltageOut {
        self.voltage_control.output = output;
        self.voltage_control.ignore_software_limits = ignore_software_limits;
        &self.voltage_control
    }

    pub(crate) fn torque_current_foc_request(&mut self, output: f64) -> &TorqueCurrentFOC {
        self.torque_current_foc.output = output;
        &self.torque_current_foc
    }

    /// Applies the current `TalonFXConfiguration` to the given motor, reporting a Driver
    /// Station warning if the apply fails.
    pub fn apply_talon_config(&self, talon: &TalonFX) {
        let result = can_config_budget::run(&self.name, |timeout| {
            talon
                .configurator()
                .apply_with_timeout(&self.talon_config, timeout)
        });
        if !result.is_ok() {
            driver_station::report_warning(&format!(
                "Could not apply config changes to {}'s motor ",
                self.name
            ));
        }
    }

    pub fn set_follower_configs(&mut self, followers: Vec<FollowerConfig>) {
        self.follower_configs = followers;
    }

    pub fn config_voltage_compensation(&mut self, voltage_comp_saturation: f64) {
        self.voltage_comp_saturation = voltage_comp_saturation;
    }

    /// Counter-clockwise positive, which is the right default for most mechanisms when read
    /// from the shaft end.
    pub fn config_counter_clockwise_positive(&mut self) {
        self.talon_config.motor_output.inverted = InvertedValue::CounterClockwise_Positive;
    }

    pub fn config_clockwise_positive(&mut self) {
        self.talon_config.motor_output.inverted = InvertedValue::Clockwise_Positive;
    }

    /// Sets the peak forward output voltage, in volts.
    pub fn config_forward_voltage_limit(&mut self, voltage_limit: f64) {
        self.talon_config.voltage.peak_forward_voltage = voltage_limit;
    }

    /// Sets the peak reverse output voltage, in volts.
    pub fn config_reverse_voltage_limit(&mut self, voltage_limit: f64) {
        self.talon_config.voltage.peak_reverse_voltage = voltage_limit;
    }

    /// Sets the current limits as a group: the supply limit with its lower limit and
    /// lower-limit time, the stator limit, and both torque limits at the stator value.
    ///
    /// `stator_limit` is the stator current limit in amps, also used for both torque limits.
    pub fn config_current_limits(
        &mut self,
        supply_limit: f64,
        stator_limit: f64,
        lower_supply_limit: f64,
        lower_supply_time: f64,
    ) {
        self.config_supply_current_limit(supply_limit, true);
        self.config_stator_current_limit(stator_limit, true);
        self.config_lower_supply_current_limit(lower_supply_limit);
        self.config_lower_supply_current_time(lower_supply_time);
        self.config_forward_torque_current_limit(stator_limit);
        self.config_reverse_torque_current_limit(stator_limit);
    }

    /// Configures the supply current limit, in amps, from the absolute value of the argument,
    /// so a negative limit is corrected rather than rejected.
    pub fn config_supply_current_limit(&mut self, supply_limit: f64, enabled: bool) {
        self.talon_config.current_limits.supply_current_limit = supply_limit.abs();
        self.talon_config.current_limits.supply_current_limit_enable = enabled;
    }

    /// Configures the stator current limit, in amps, from the absolute value of the argument,
    /// so a negative limit is corrected rather than rejected.
    pub fn config_stator_current_limit(&mut self, stator_limit: f64, enabled: bool) {
        self.talon_config.current_limits.stator_current_limit = stator_limit.abs();
        self.talon_config.current_limits.stator_current_limit_enable = enabled;
    }

    /// Sets the peak forward torque current limit, in amps, from the absolute value of the
    /// argument, so a negative limit is corrected rather than rejected.
    pub fn config_forward_torque_current_limit(&mut self, current_limit: f64) {
        self.talon_config.torque_current.peak_forward_torque_current = current_limit.abs();
    }

    /// Sets the open-loop ramp period, in seconds, for duty cycle, voltage and torque.
    pub fn config_open_loop_ramps(&mut self, seconds: f64) {
        let ramps = &mut self.talon_config.open_loop_ramps;
        ramps.duty_cycle_open_loop_ramp_period = seconds;
        ramps.voltage_open_loop_ramp_period = seconds;
        ramps.torque_open_loop_ramp_period = seconds;
    }

    /// Sets the closed-loop ramp period, in seconds, for duty cycle, voltage and torque.
    pub fn config_closed_loop_ramps(&mut self, seconds: f64) {
        let ramps = &mut self.talon_config.closed_loop_ramps;
        ramps.duty_cycle_closed_loop_ramp_period = seconds;
        ramps.voltage_closed_loop_ramp_period = seconds;
        ramps.torque_closed_loop_ramp_period = seconds;
    }

    /// Sets the peak reverse torque current limit, in amps. The stored value is forced
    /// negative, so a positive argument is corrected rather than rejected.
    pub fn config_reverse_torque_current_limit(&mut self, current_limit: f64) {
        self.talon_config.torque_current.peak_reverse_torque_current = -current_limit.abs();
    }

    /// Sets the lower supply current limit, in amps, that applies once the upper one trips.
    pub fn config_lower_supply_current_limit(&mut self, current_limit: f64) {
        self.talon_config.current_limits.supply_current_lower_limit = current_limit;
    }

    /// Sets how long the upper supply limit holds, in seconds, before dropping to the lower.
    pub fn config_lower_supply_current_time(&mut self, time: f64) {
        self.talon_config.current_limits.supply_current_lower_time = time;
    }

    /// Sets the duty-cycle neutral deadband as a fraction of full output, so outputs below
    /// that magnitude are treated as zero.
    pub fn config_neutral_deadband(&mut self, deadband: f64) {
        self.talon_config.motor_output.duty_cycle_neutral_deadband = deadband;
    }

    /// Sets the peak duty-cycle output limits.
    ///
    /// `forward` is the maximum forward output, [0, 1]. `reverse` is the maximum reverse
    /// output, [-1, 0].
    pub fn config_peak_output(&mut self, forward: f64, reverse: f64) {
        self.talon_config.motor_output.peak_forward_duty_cycle = forward;
        self.talon_config.motor_output.peak_reverse_duty_cycle = reverse;
    }

    /// Configures the forward software limit, with the threshold in rotations.
    pub fn config_forward_soft_limit(&mut self, threshold: f64, enabled: bool) {
        self.talon_config
            .software_limit_switch
            .forward_soft_limit_threshold = threshold;
        self.talon_config
            .software_limit_switch
            .forward_soft_limit_enable = enabled;
    }

    /// Configures the reverse software limit, with the threshold in rotations.
    pub fn config_reverse_soft_limit(&mut self, threshold: f64, enabled: bool) {
        self.talon_config
            .software_limit_switch
            .reverse_soft_limit_threshold = threshold;
        self.talon_config
            .software_limit_switch
            .reverse_soft_limit_enable = enabled;
    }

    /// Enables or disables continuous wrap-around in closed-loop control, for a mechanism
    /// that turns indefinitely such as a swerve azimuth.
    pub fn config_continuous_wrap(&mut self, enabled: bool) {
        self.talon_config.closed_loop_general.continuous_wrap = enabled;
    }

    /// Sets the Motion Magic acceleration and feed-forward on both the FOC and the voltage
    /// velocity requests.
    ///
    /// `acceleration` is in rotations per second squared.
    pub fn config_motion_magic_velocity(&mut self, acceleration: f64, feedforward: f64) {
        self.mm_velocity_foc.acceleration = acceleration;
        self.mm_velocity_foc.feed_forward = feedforward;
        self.mm_velocity_voltage.acceleration = acceleration;
        self.mm_velocity_voltage.feed_forward = feedforward;
    }

    /// Sets the feed-forward term on every Motion Magic position request.
    pub fn config_motion_magic_position(&mut self, feedforward: f64) {
        self.mm_position_foc.feed_forward = feedforward;
        self.mm_position_voltage.feed_forward = feedforward;
        self.mm_position_voltage_slot.feed_forward = feedforward;
    }

    /// Sets the Motion Magic trajectory limits.
    ///
    /// `cruise_velocity` is in rotations per second, `acceleration` in rotations per second
    /// squared, `jerk` in rotations per second cubed.
    pub fn config_motion_magic(&mut self, cruise_velocity: f64, acceleration: f64, jerk: f64) {
        let motion_magic = &mut self.talon_config.motion_magic;
        motion_magic.motion_magic_cruise_velocity = cruise_velocity;
        motion_magic.motion_magic_acceleration = acceleration;
        motion_magic.motion_magic_jerk = jerk;
    }

    /// Sets the sensor-to-mechanism gear ratio, counted in sensor turns per mechanism output
    /// turn, so 11.25 means 11.25 sensor turns per output rotation.
    pub fn config_gear_ratio(&mut self, gear_ratio: f64) {
        self.talon_config.feedback.sensor_to_mechanism_ratio = gear_ratio;
    }

    /// The configured sensor-to-mechanism gear ratio.
    pub fn get_gear_ratio(&self) -> f64 {
        self.talon_config.feedback.sensor_to_mechanism_ratio
    }

    /// Sets brake mode when `is_in_brake` is true, coast when it is false.
    pub fn config_neutral_brake_mode(&mut self, is_in_brake: bool) {
        self.talon_config.motor_output.neutral_mode = if is_in_brake {
            NeutralModeValue::Brake
        } else {
            NeutralModeValue::Coast
        };
    }

    /// Configures the PID gains in slot 0.
    pub fn config_pid_gains(&mut self, k_p: f64, k_i: f64, k_d: f64) {
        self.config_pid_gains_slot(0, k_p, k_i, k_d);
    }

    /// Configures the PID gains in a gain slot.
    ///
    /// `slot` is the gain slot, 0, 1 or 2.
    pub fn config_pid_gains_slot(&mut self, slot: i32, k_p: f64, k_i: f64, k_d: f64) {
        match slot {
            0 => {
                let gains = &mut self.talon_config.slot0;
                (gains.k_p, gains.k_i, gains.k_d) = (k_p, k_i, k_d);
            }
            1 => {
                let gains = &mut self.talon_config.slot1;
                (gains.k_p, gains.k_i, gains.k_d) = (k_p, k_i, k_d);
            }
            2 => {
                let gains = &mut self.talon_config.slot2;
                (gains.k_p, gains.k_i, gains.k_d) = (k_p, k_i, k_d);
            }
            _ => driver_station::report_warning("MechConfig: Invalid Feedback slot"),
        }
    }

    /// Configures the feed-forward gains in slot 0.
    ///
    /// `k_s` is static friction compensation, in volts or amps depending on the control mode.
    pub fn config_feed_forward_gains(&mut self, k_s: f64, k_v: f64, k_a: f64, k_g: f64) {
        self.config_feed_forward_gains_slot(0, k_s, k_v, k_a, k_g);
    }

    /// Configures the feed-forward gains in a gain slot.
    ///
    /// `slot` is the gain slot, 0, 1 or 2. `k_s` is static friction compensation, in volts or
    /// amps depending on the control mode.
    pub fn config_feed_forward_gains_slot(
        &mut self,
        slot: i32,
        k_s: f64,
        k_v: f64,
        k_a: f64,
        k_g: f64,
    ) {
        match slot {
            0 => {
                let gains = &mut self.talon_config.slot0;
                (gains.k_s, gains.k_v, gains.k_a, gains.k_g) = (k_s, k_v, k_a, k_g);
            }
            1 => {
                let gains = &mut self.talon_config.slot1;
                (gains.k_s, gains.k_v, gains.k_a, gains.k_g) = (k_s, k_v, k_a, k_g);
            }
            2 => {
                let gains = &mut self.talon_config.slot2;
                (gains.k_s, gains.k_v, gains.k_a, gains.k_g) = (k_s, k_v, k_a, k_g);
            }
            _ => driver_station::report_warning("MechConfig: Invalid FeedForward slot"),
        }
    }

    /// Configures the feedback sensor source with a rotor offset of 0.
    pub fn config_feedback_sensor_source(&mut self, source: FeedbackSensorSourceValue) {
        self.config_feedback_sensor_source_offset(source, 0.0);
    }

    /// Configures the feedback sensor source and its rotor offset, in rotations.
    pub fn config_feedback_sensor_source_offset(
        &mut self,
        source: FeedbackSensorSourceValue,
        offset: f64,
    ) {
        self.talon_config.feedback.feedback_sensor_source = source;
        self.talon_config.feedback.feedback_rotor_offset = offset;
    }

    /// Configures the gravity compensation type in slot 0.
    ///
    /// `is_arm` is true for `Arm_Cosine` on a rotating arm, false for `Elevator_Static` on an
    /// elevator.
    pub fn config_gravity_type(&mut self, is_arm: bool) {
        self.config_gravity_type_slot(0, is_arm);
    }

    /// Configures the gravity compensation type in a gain slot.
    ///
    /// `slot` is the gain slot, 0, 1 or 2. `is_arm` is true for `Arm_Cosine` on a rotating
    /// arm, false for `Elevator_Static` on an elevator.
    pub fn config_gravity_type_slot(&mut self, slot: i32, is_arm: bool) {
        let gravity_type = if is_arm {
            GravityTypeValue::Arm_Cosine
        } else {
            GravityTypeValue::Elevator_Static
        };
        match slot {
            0 => self.talon_config.slot0.gravity_type = gravity_type,
            1 => self.talon_config.slot1.gravity_type = gravity_type,
            2 => self.talon_config.slot2.gravity_type = gravity_type,
            _ => driver_station::report_warning("MechConfig: Invalid slot"),
        }
    }

    /// Sets the rotation limits the percentage helpers such as
    /// [`Mechanism::percent_to_rotations`] convert against.
    pub fn config_min_max_rotations(&mut self, min_rotation: f64, max_rotation: f64) {
        self.min_rotations = min_rotation;
        self.max_rotations = max_rotation;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectrum_lib::framework::robot_loop;

    fn unattached(name: &str) -> Rc<Mechanism> {
        let mut config = Config::new(name, 1, "rio");
        config.attached = false;
        Mechanism::new(config)
    }

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-9, "{expected} vs {actual}");
    }

    // Config defaults

    #[test]
    fn config_starts_with_the_java_defaults() {
        let config = Config::new("Hood", 15, "canivore");
        assert_eq!(config.name, "Hood");
        assert!(config.attached);
        assert!(!config.fast_output_logging);
        assert_eq!(config.id, CanDeviceId::new(15, "canivore"));
        assert_eq!(config.num_motors, 1);
        assert_eq!(config.voltage_comp_saturation, 12.0);
        assert_eq!(config.min_rotations, 0.0);
        assert_eq!(config.max_rotations, 1.0);
        assert!(config.follower_configs.is_empty());
        assert!(
            !config
                .talon_config
                .hardware_limit_switch
                .forward_limit_enable
        );
        assert!(
            !config
                .talon_config
                .hardware_limit_switch
                .reverse_limit_enable
        );
        assert_eq!(config.mm_position_voltage_slot.slot, 1);
        assert_eq!(config.mm_position_voltage.slot, 0);
    }

    #[test]
    fn follower_config_defaults_to_attached() {
        let follower = FollowerConfig::new("Left", 7, "rio", MotorAlignmentValue::Opposed);
        assert_eq!(follower.name, "Left");
        assert_eq!(follower.id, CanDeviceId::new(7, "rio"));
        assert!(follower.attached);
        assert_eq!(follower.oppose_leader, MotorAlignmentValue::Opposed);
    }

    // Config helpers

    #[test]
    fn current_limit_helpers_correct_signs() {
        let mut config = Config::new("x", 1, "rio");
        config.config_supply_current_limit(-40.0, true);
        config.config_stator_current_limit(-60.0, false);
        config.config_forward_torque_current_limit(-80.0);
        config.config_reverse_torque_current_limit(80.0);
        let talon = &config.talon_config;
        assert_eq!(talon.current_limits.supply_current_limit, 40.0);
        assert!(talon.current_limits.supply_current_limit_enable);
        assert_eq!(talon.current_limits.stator_current_limit, 60.0);
        assert!(!talon.current_limits.stator_current_limit_enable);
        assert_eq!(talon.torque_current.peak_forward_torque_current, 80.0);
        assert_eq!(talon.torque_current.peak_reverse_torque_current, -80.0);
    }

    #[test]
    fn config_current_limits_sets_the_group() {
        let mut config = Config::new("x", 1, "rio");
        config.config_current_limits(40.0, 60.0, 30.0, 0.5);
        let talon = &config.talon_config;
        assert_eq!(talon.current_limits.supply_current_limit, 40.0);
        assert_eq!(talon.current_limits.stator_current_limit, 60.0);
        assert_eq!(talon.current_limits.supply_current_lower_limit, 30.0);
        assert_eq!(talon.current_limits.supply_current_lower_time, 0.5);
        assert_eq!(talon.torque_current.peak_forward_torque_current, 60.0);
        assert_eq!(talon.torque_current.peak_reverse_torque_current, -60.0);
    }

    #[test]
    fn pid_and_feedforward_gains_land_in_the_chosen_slot() {
        let mut config = Config::new("x", 1, "rio");
        config.config_pid_gains(1.0, 2.0, 3.0);
        config.config_pid_gains_slot(1, 4.0, 5.0, 6.0);
        config.config_pid_gains_slot(2, 7.0, 8.0, 9.0);
        config.config_feed_forward_gains(0.1, 0.2, 0.3, 0.4);
        config.config_feed_forward_gains_slot(2, 1.1, 1.2, 1.3, 1.4);
        let talon = &config.talon_config;
        assert_eq!(
            (talon.slot0.k_p, talon.slot0.k_i, talon.slot0.k_d),
            (1.0, 2.0, 3.0)
        );
        assert_eq!(
            (talon.slot1.k_p, talon.slot1.k_i, talon.slot1.k_d),
            (4.0, 5.0, 6.0)
        );
        assert_eq!(
            (talon.slot2.k_p, talon.slot2.k_i, talon.slot2.k_d),
            (7.0, 8.0, 9.0)
        );
        assert_eq!(
            (
                talon.slot0.k_s,
                talon.slot0.k_v,
                talon.slot0.k_a,
                talon.slot0.k_g
            ),
            (0.1, 0.2, 0.3, 0.4)
        );
        assert_eq!(
            (
                talon.slot2.k_s,
                talon.slot2.k_v,
                talon.slot2.k_a,
                talon.slot2.k_g
            ),
            (1.1, 1.2, 1.3, 1.4)
        );
        assert_eq!(talon.slot1.k_s, 0.0);
    }

    #[test]
    fn gravity_type_follows_the_arm_flag() {
        let mut config = Config::new("x", 1, "rio");
        config.config_gravity_type(true);
        config.config_gravity_type_slot(1, false);
        config.config_gravity_type_slot(2, true);
        assert_eq!(
            config.talon_config.slot0.gravity_type,
            GravityTypeValue::Arm_Cosine
        );
        assert_eq!(
            config.talon_config.slot1.gravity_type,
            GravityTypeValue::Elevator_Static
        );
        assert_eq!(
            config.talon_config.slot2.gravity_type,
            GravityTypeValue::Arm_Cosine
        );
    }

    #[test]
    fn motion_magic_helpers_set_the_requests_and_the_profile() {
        let mut config = Config::new("x", 1, "rio");
        config.config_motion_magic_velocity(50.0, 0.5);
        assert_eq!(config.mm_velocity_foc.acceleration, 50.0);
        assert_eq!(config.mm_velocity_foc.feed_forward, 0.5);
        assert_eq!(config.mm_velocity_voltage.acceleration, 50.0);
        assert_eq!(config.mm_velocity_voltage.feed_forward, 0.5);

        config.config_motion_magic_position(0.25);
        assert_eq!(config.mm_position_foc.feed_forward, 0.25);
        assert_eq!(config.mm_position_voltage.feed_forward, 0.25);
        assert_eq!(config.mm_position_voltage_slot.feed_forward, 0.25);

        config.config_motion_magic(100.0, 300.0, 1000.0);
        let mm = &config.talon_config.motion_magic;
        assert_eq!(mm.motion_magic_cruise_velocity, 100.0);
        assert_eq!(mm.motion_magic_acceleration, 300.0);
        assert_eq!(mm.motion_magic_jerk, 1000.0);
    }

    #[test]
    fn simple_talon_helpers_write_their_fields() {
        let mut config = Config::new("x", 1, "rio");
        config.config_gear_ratio(11.25);
        assert_eq!(config.get_gear_ratio(), 11.25);
        config.config_clockwise_positive();
        assert_eq!(
            config.talon_config.motor_output.inverted,
            InvertedValue::Clockwise_Positive
        );
        config.config_counter_clockwise_positive();
        assert_eq!(
            config.talon_config.motor_output.inverted,
            InvertedValue::CounterClockwise_Positive
        );
        config.config_neutral_brake_mode(false);
        assert_eq!(
            config.talon_config.motor_output.neutral_mode,
            NeutralModeValue::Coast
        );
        config.config_neutral_brake_mode(true);
        assert_eq!(
            config.talon_config.motor_output.neutral_mode,
            NeutralModeValue::Brake
        );
        config.config_neutral_deadband(0.05);
        config.config_peak_output(0.8, -0.7);
        config.config_forward_voltage_limit(10.0);
        config.config_reverse_voltage_limit(-9.0);
        config.config_continuous_wrap(true);
        config.config_forward_soft_limit(2.8, true);
        config.config_reverse_soft_limit(0.1, true);
        config.config_open_loop_ramps(0.2);
        config.config_closed_loop_ramps(0.3);
        config.config_voltage_compensation(11.0);
        config.config_min_max_rotations(0.024, 0.137);
        let talon = &config.talon_config;
        assert_eq!(talon.motor_output.duty_cycle_neutral_deadband, 0.05);
        assert_eq!(talon.motor_output.peak_forward_duty_cycle, 0.8);
        assert_eq!(talon.motor_output.peak_reverse_duty_cycle, -0.7);
        assert_eq!(talon.voltage.peak_forward_voltage, 10.0);
        assert_eq!(talon.voltage.peak_reverse_voltage, -9.0);
        assert!(talon.closed_loop_general.continuous_wrap);
        assert_eq!(
            talon.software_limit_switch.forward_soft_limit_threshold,
            2.8
        );
        assert!(talon.software_limit_switch.forward_soft_limit_enable);
        assert_eq!(
            talon.software_limit_switch.reverse_soft_limit_threshold,
            0.1
        );
        assert!(talon.software_limit_switch.reverse_soft_limit_enable);
        assert_eq!(talon.open_loop_ramps.torque_open_loop_ramp_period, 0.2);
        assert_eq!(
            talon.closed_loop_ramps.duty_cycle_closed_loop_ramp_period,
            0.3
        );
        assert_eq!(config.voltage_comp_saturation, 11.0);
        assert_eq!((config.min_rotations, config.max_rotations), (0.024, 0.137));
    }

    #[test]
    fn feedback_source_helper_sets_source_and_offset() {
        let mut config = Config::new("x", 1, "rio");
        config.config_feedback_sensor_source_offset(FeedbackSensorSourceValue::FusedCANcoder, 0.25);
        assert_eq!(
            config.talon_config.feedback.feedback_sensor_source,
            FeedbackSensorSourceValue::FusedCANcoder
        );
        assert_eq!(config.talon_config.feedback.feedback_rotor_offset, 0.25);
        config.config_feedback_sensor_source(FeedbackSensorSourceValue::RotorSensor);
        assert_eq!(config.talon_config.feedback.feedback_rotor_offset, 0.0);
    }

    // Control request selection

    #[test]
    fn velocity_requests_set_only_the_velocity() {
        let mut config = Config::new("x", 1, "rio");
        assert_eq!(config.velocity_request(12.5).velocity, 12.5);
        assert_eq!(
            config.velocity_torque_current_foc_request(3.0).velocity,
            3.0
        );
        assert_eq!(config.mm_velocity_foc_request(4.0).velocity, 4.0);
    }

    #[test]
    fn position_requests_carry_the_feedforward_velocity() {
        let mut config = Config::new("x", 1, "rio");
        let request = config.position_request(0.5, 2.0);
        assert_eq!((request.position, request.velocity), (0.5, 2.0));
        let request = config.position_foc_request(0.25, 1.0);
        assert_eq!((request.position, request.velocity), (0.25, 1.0));
        assert_eq!(config.mm_position_foc_request(0.75).position, 0.75);
    }

    #[test]
    fn dynamic_motion_magic_requests_carry_the_trajectory() {
        let mut config = Config::new("x", 1, "rio");
        let request = config.dyn_mm_position_foc_request(1.0, 2.0, 3.0, 4.0);
        assert_eq!(
            (
                request.position,
                request.velocity,
                request.acceleration,
                request.jerk
            ),
            (1.0, 2.0, 3.0, 4.0)
        );
        let request = config.dyn_mm_position_voltage_request(5.0, 6.0, 7.0, 8.0);
        assert_eq!(
            (
                request.position,
                request.velocity,
                request.acceleration,
                request.jerk
            ),
            (5.0, 6.0, 7.0, 8.0)
        );
    }

    #[test]
    fn motion_magic_voltage_position_uses_the_slot_request() {
        let mut config = Config::new("x", 1, "rio");
        let request = config.mm_position_voltage_request(0.3, 2);
        assert_eq!((request.position, request.slot), (0.3, 2));
        // The slot is set on every call, not left over from the last one.
        assert_eq!(config.mm_position_voltage_request(0.4, 0).slot, 0);
        assert_eq!(config.mm_position_voltage.slot, 0);
    }

    #[test]
    fn voltage_request_resets_the_soft_limit_flag_every_call() {
        let mut config = Config::new("x", 1, "rio");
        let request = config.voltage_request(6.0, true);
        assert_eq!(request.output, 6.0);
        assert!(request.ignore_software_limits);
        let request = config.voltage_request(3.0, false);
        assert_eq!(request.output, 3.0);
        assert!(!request.ignore_software_limits);
    }

    #[test]
    fn torque_current_request_sets_the_output() {
        let mut config = Config::new("x", 1, "rio");
        assert_eq!(config.torque_current_foc_request(25.0).output, 25.0);
    }

    // Mechanism without hardware

    #[test]
    fn an_unattached_mechanism_reads_zero_everywhere() {
        let mechanism = unattached("Test");
        assert!(!mechanism.is_attached());
        assert!(mechanism.get_motor().is_none());
        assert!(mechanism.get_follower_motors().is_empty());
        assert_eq!(mechanism.get_stator_current(), 0.0);
        assert_eq!(mechanism.get_supply_current(), 0.0);
        assert_eq!(mechanism.get_voltage(), 0.0);
        assert_eq!(mechanism.get_temp(), 0.0);
        assert_eq!(mechanism.get_position_rotations(), 0.0);
        assert_eq!(mechanism.get_position_degrees(), 0.0);
        assert_eq!(mechanism.get_position_percentage(), 0.0);
        assert_eq!(mechanism.get_velocity_rpm(), 0.0);
        assert_eq!(mechanism.get_latency_compensated_position_rotations(), 0.0);
        assert_eq!(mechanism.get_latency_compensated_position_degrees(), 0.0);
    }

    #[test]
    fn an_unattached_mechanism_sends_nothing_and_keeps_its_targets() {
        let mechanism = unattached("Test");
        mechanism.set_velocity(|| 5.0);
        mechanism.set_velocity_rpm(|| 300.0);
        mechanism.set_velocity_torque_current_foc(|| 5.0);
        mechanism.set_velocity_tcfoc_rpm(|| 300.0);
        mechanism.set_mm_velocity_foc(|| 5.0);
        mechanism.set_position(|| 1.0);
        mechanism.set_position_with_velocity(|| 1.0, || 2.0);
        mechanism.set_position_foc_with_velocity(|| 1.0, || 2.0);
        mechanism.set_mm_position(|| 1.0);
        mechanism.set_mm_position_slot(|| 1.0, 2);
        mechanism.set_mm_position_foc(|| 1.0);
        mechanism.set_dyn_mm_position_foc(|| 1.0, || 2.0, || 3.0, || 4.0);
        mechanism.set_dyn_mm_position_voltage(|| 1.0, || 2.0, || 3.0, || 4.0);
        mechanism.set_percent_output(|| 0.5);
        mechanism.set_voltage_output(|| 6.0);
        mechanism.set_voltage_output_no_soft_limit(|| 6.0);
        mechanism.set_torque_current_foc(|| 20.0);
        mechanism.set_motor_position(|| 1.0);
        mechanism.tare_motor();
        mechanism.stop();
        mechanism.set_brake_mode(false);
        mechanism.toggle_reverse_soft_limit(true);
        mechanism.toggle_torque_current_limit(|| 40.0, true);
        mechanism.toggle_supply_current_limit(|| 40.0, true);
        mechanism.apply_current_limit(|| 40.0, || 60.0);
        mechanism.set_current_limits(|| 40.0, || 60.0);
        assert_eq!(mechanism.get_target(), 0.0);
        assert_eq!(mechanism.get_velocity_target_rps(), 0.0);
        // No hardware was touched, so the config is exactly as built.
        let config = mechanism.config.borrow();
        assert_eq!(
            config.talon_config.current_limits.supply_current_limit,
            70.0
        );
        assert_eq!(
            config.talon_config.motor_output.neutral_mode,
            NeutralModeValue::Coast
        );
    }

    #[test]
    fn with_attached_overrides_the_config_flag() {
        let mechanism = Mechanism::with_attached(Config::new("Test", 1, "rio"), false);
        assert!(!mechanism.is_attached());
        assert!(!mechanism.config.borrow().attached);
    }

    #[test]
    fn the_name_comes_from_the_config() {
        let mechanism = unattached("Hood");
        assert_eq!(mechanism.get_name(), "Hood");
        assert_eq!(Subsystem::name(&*mechanism), "Hood");
        assert_eq!(mechanism.battery_key, "Mechanisms/Hood");
    }

    #[test]
    fn mechanisms_have_distinct_subsystem_ids() {
        assert_ne!(unattached("A").id(), unattached("B").id());
    }

    #[test]
    fn current_command_name_is_none_without_a_command() {
        assert_eq!(unattached("Test").get_current_command_name(), "none");
    }

    // Unit conversions

    #[test]
    fn percent_and_rotations_convert_against_max_rotations() {
        let mut config = Config::new("Test", 1, "rio");
        config.attached = false;
        config.config_min_max_rotations(0.0, 2.8);
        let mechanism = Mechanism::new(config);
        assert_close(1.4, mechanism.percent_to_rotations(|| 50.0));
        assert_close(0.0, mechanism.percent_to_rotations(|| 0.0));
        assert_close(2.8, mechanism.percent_to_rotations(|| 100.0));
        assert_close(25.0, mechanism.rotations_to_percent(|| 0.7));
        assert_close(100.0, mechanism.rotations_to_percent(|| 2.8));
    }

    #[test]
    fn degrees_and_rotations_convert_by_the_full_turn() {
        let mechanism = unattached("Test");
        assert_close(0.5, mechanism.degrees_to_rotations(|| 180.0));
        assert_close(0.25, mechanism.degrees_to_rotations(|| 90.0));
        assert_close(360.0, mechanism.rotations_to_degrees(|| 1.0));
        assert_close(-90.0, mechanism.rotations_to_degrees(|| -0.25));
    }

    #[test]
    fn rpm_helpers_stay_in_rpm() {
        assert_close(1.0, conversions::rpm_to_rps(60.0));
        assert_close(6000.0, conversions::rps_to_rpm(100.0));
    }

    // Triggers

    #[test]
    fn position_triggers_compare_against_the_position() {
        // An unattached mechanism sits at 0 rotations.
        let mechanism = unattached("Test");
        assert!(mechanism.at_rotations(|| 0.05, || 0.1).get_as_boolean());
        assert!(!mechanism.at_rotations(|| 0.5, || 0.1).get_as_boolean());
        assert!(mechanism.below_rotations(|| 1.0, || 0.1).get_as_boolean());
        assert!(!mechanism.below_rotations(|| -1.0, || 0.5).get_as_boolean());
        assert!(mechanism.above_rotations(|| 0.05, || 0.1).get_as_boolean());
        assert!(!mechanism.above_rotations(|| 1.0, || 0.1).get_as_boolean());
    }

    #[test]
    fn tolerance_bounds_are_exclusive() {
        let mechanism = unattached("Test");
        assert!(!mechanism.at_rotations(|| 0.1, || 0.1).get_as_boolean());
        assert!(!mechanism.below_rotations(|| -0.1, || 0.1).get_as_boolean());
        assert!(!mechanism.above_rotations(|| 0.1, || 0.1).get_as_boolean());
    }

    #[test]
    fn the_other_trigger_families_read_their_own_quantity() {
        let mechanism = unattached("Test");
        assert!(mechanism.at_percentage(|| 0.0, || 1.0).get_as_boolean());
        assert!(mechanism.below_percentage(|| 5.0, || 1.0).get_as_boolean());
        assert!(mechanism.above_percentage(|| -5.0, || 1.0).get_as_boolean());
        assert!(mechanism.at_degrees(|| 0.0, || 1.0).get_as_boolean());
        assert!(mechanism.below_degrees(|| 5.0, || 1.0).get_as_boolean());
        assert!(mechanism.above_degrees(|| -5.0, || 1.0).get_as_boolean());
        assert!(mechanism.at_velocity_rpm(|| 0.0, || 1.0).get_as_boolean());
        assert!(
            mechanism
                .below_velocity_rpm(|| 5.0, || 1.0)
                .get_as_boolean()
        );
        assert!(
            mechanism
                .above_velocity_rpm(|| -5.0, || 1.0)
                .get_as_boolean()
        );
        assert!(mechanism.at_current(|| 0.0, || 1.0).get_as_boolean());
        assert!(mechanism.below_current(|| 5.0, || 1.0).get_as_boolean());
        assert!(mechanism.above_current(|| -5.0, || 1.0).get_as_boolean());
    }

    #[test]
    fn target_position_trigger_compares_against_the_last_target() {
        let mechanism = unattached("Test");
        assert!(mechanism.is_at_target_position(|| 0.01));
        assert!(mechanism.at_target_position(|| 0.01).get_as_boolean());
        // The unattached mechanism never records a target, so a wide miss stays impossible.
        mechanism.target.set(5.0);
        assert!(!mechanism.is_at_target_position(|| 0.01));
        assert!(!mechanism.at_target_position(|| 0.01).get_as_boolean());
        assert!(mechanism.is_at_target_position(|| 6.0));
        assert!(!mechanism.is_at_rotations(|| 1.0, || 0.5));
        assert!(mechanism.is_at_rotations(|| 0.0, || 0.5));
    }

    #[test]
    fn running_default_command_is_true_with_no_commands() {
        // Java: both the current and the default command are null, which compares equal.
        let mechanism = unattached("Test");
        assert!(mechanism.running_default_command().get_as_boolean());
    }

    // Commands

    #[test]
    fn command_names_follow_java() {
        let mechanism = unattached("Hood");
        assert_eq!(
            mechanism.run_velocity(|| 1.0).get_name(),
            "Hood.runVelocity"
        );
        assert_eq!(
            mechanism.run_velocity_tc_foc_rpm(|| 1.0).get_name(),
            "Hood.runVelocityTcFocRPM"
        );
        assert_eq!(
            mechanism.run_percentage(|| 1.0).get_name(),
            "Hood.runPercentage"
        );
        assert_eq!(mechanism.run_voltage(|| 1.0).get_name(), "Hood.runVoltage");
        assert_eq!(
            mechanism.run_voltage_no_soft_limit(|| 1.0).get_name(),
            "Hood.runVoltageNoSoftLimit"
        );
        assert_eq!(
            mechanism.run_torque_current_foc(|| 1.0).get_name(),
            "Hood.runTorqueCurrentFoc"
        );
        assert_eq!(
            mechanism.move_to_rotations(|| 1.0).get_name(),
            "Hood.runPoseRevolutions"
        );
        assert_eq!(
            mechanism.move_to_percentage(|| 1.0).get_name(),
            "Hood.runPosePercentage"
        );
        assert_eq!(
            mechanism.move_to_degrees(|| 1.0).get_name(),
            "Hood.runPoseDegrees"
        );
        assert_eq!(
            mechanism.run_foc_rotations(|| 1.0).get_name(),
            "Hood.runFOCPosition"
        );
        assert_eq!(mechanism.run_stop().get_name(), "Hood.runStop");
        assert_eq!(mechanism.coast_mode().get_name(), "Hood.coastMode");
        assert_eq!(
            mechanism.ensure_brake_mode().get_name(),
            "Hood.ensureBrakeMode"
        );
    }

    #[test]
    fn mechanism_commands_require_the_mechanism() {
        let mechanism = unattached("Hood");
        let command = mechanism.run_velocity(|| 1.0);
        assert!(command.has_requirement(mechanism.id()));
        assert!(mechanism.run_stop().has_requirement(mechanism.id()));
        assert!(mechanism.coast_mode().has_requirement(mechanism.id()));
    }

    #[test]
    fn coast_mode_and_brake_commands_run_while_disabled() {
        let mechanism = unattached("Hood");
        assert!(mechanism.coast_mode().runs_when_disabled());
        assert!(mechanism.ensure_brake_mode().runs_when_disabled());
        assert!(!mechanism.run_velocity(|| 1.0).runs_when_disabled());
    }

    #[test]
    fn a_running_command_is_the_current_command() {
        let mechanism = unattached("Hood");
        let command = mechanism.run_velocity(|| 1.0);
        commands::CommandScheduler::get_instance().schedule(&command);
        assert_eq!(mechanism.get_current_command_name(), "Hood.runVelocity");
        assert!(!mechanism.running_default_command().get_as_boolean());
        command.cancel();
        assert_eq!(mechanism.get_current_command_name(), "none");
    }

    #[test]
    fn current_check_commands_have_no_requirements_and_an_empty_name() {
        let mechanism = unattached("Hood");
        for command in [
            mechanism.check_avg_current(|| 10.0, || 1.0),
            mechanism.check_max_current(|| 10.0),
            mechanism.check_min_threshold_current(|| 10.0),
        ] {
            assert_eq!(command.get_name(), "");
            assert!(command.get_requirements().is_empty());
        }
    }

    #[test]
    fn the_run_current_limits_command_needs_no_subsystem() {
        let mechanism = unattached("Hood");
        assert!(
            mechanism
                .run_current_limits(|| 40.0, || 60.0)
                .get_requirements()
                .is_empty()
        );
    }

    // Once-per-loop signal cache

    #[test]
    fn the_refresh_loop_marker_starts_unset() {
        let mechanism = unattached("Test");
        assert_eq!(mechanism.last_signal_refresh_loop.get(), None);
        robot_loop::next();
        // Unattached reads return before touching the cache.
        mechanism.get_position_rotations();
        assert_eq!(mechanism.last_signal_refresh_loop.get(), None);
    }

    // Follower check

    #[test]
    fn follower_loop_time_is_capped_and_never_negative() {
        assert_close(0.02, follower_loop_seconds(10.02, 10.0));
        assert_close(0.5, follower_loop_seconds(100.0, 10.0));
        assert_close(0.5, follower_loop_seconds(100.0, 0.0));
        assert_close(0.0, follower_loop_seconds(5.0, 10.0));
    }

    #[test]
    fn a_follower_with_any_draw_resets_its_mismatch() {
        assert_eq!(follower_mismatch_step(1.2, 0.02, 0.6, 20.0), 0.0);
        assert_eq!(follower_mismatch_step(1.2, 0.02, 5.0, 0.0), 0.0);
    }

    #[test]
    fn a_silent_follower_under_a_loaded_leader_accumulates() {
        assert_close(1.02, follower_mismatch_step(1.0, 0.02, 0.5, 8.5));
        assert_close(0.02, follower_mismatch_step(0.0, 0.02, 0.0, 20.0));
    }

    #[test]
    fn an_idle_leader_neither_accumulates_nor_clears() {
        assert_eq!(follower_mismatch_step(0.7, 0.02, 0.0, 8.0), 0.7);
        assert_eq!(follower_mismatch_step(0.7, 0.02, 0.5, 2.0), 0.7);
    }

    #[test]
    fn the_follower_alert_trips_at_one_and_a_half_seconds() {
        let mut seconds = 0.0;
        let mut loops = 0;
        while seconds < FOLLOWER_DEAD_SECONDS {
            seconds = follower_mismatch_step(seconds, 0.02, 0.0, 30.0);
            loops += 1;
        }
        assert_eq!(loops, 75);
    }
}
