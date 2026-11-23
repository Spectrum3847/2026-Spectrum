//! Rack-and-pinion fuel intake deploy, driven by two independent axes: the left, which is
//! [`IntakeExtension`], and the right ([`IntakeExtensionRight`]). Each side runs its own
//! closed-loop position control rather than one following the other, so a side that skips a
//! tooth can be driven on its own to resync.
//!
//! Resyncing drives each side into the fully extended hard stop and re-zeroes that side's
//! encoder at max rotations. A skipped tooth otherwise leaves the motor encoder reading a
//! position the rack is no longer at.

use crate::context;
use crate::spectrum_lib::hardware::rio::Rio;
use crate::spectrum_lib::mechanism::mechanism::{Config, Mechanism};
use crate::spectrum_lib::sim::linear_config::LinearConfig;
use crate::spectrum_lib::sim::linear_sim::LinearSim;
use commands::{Command, CommandScheduler, Commands, Subsystem, SubsystemId};
use phoenix6::{TalonFX, TalonFXConfiguration};
use std::cell::{Cell, RefCell};
use std::ops::Deref;
use std::rc::{Rc, Weak};
use telemetry::{EnumName, telemetry};
use wpilib::dashboard::Color8Bit;
use wpilib::timer::Timer;
use wpimath::units::inches_to_meters;

#[derive(Clone, Debug)]
pub struct IntakeExtensionConfig {
    pub base: Config,

    pub init_position: f64,
    pub trigger_tolerance: f64,

    pub zero_speed: f64,
    pub hold_max_speed_rpm: f64,

    // Java: these shadow the base config's min and max rotations, which the
    // constructor sets to the same values.
    pub max_rotations: f64,
    pub min_rotations: f64,

    pub supply_current_limit: f64,
    pub stator_current_limit: f64,
    pub lower_supply_current_limit: f64,
    pub lower_supply_current_time: f64,

    pub position_kp: f64,
    pub position_ki: f64,
    pub position_kd: f64,
    pub position_kv: f64,
    pub position_ks: f64,
    pub position_ka: f64,
    pub position_kg: f64,
    pub gear_ratio: f64,
    pub mm_cruise_velocity: f64,
    pub mm_acceleration: f64,
    pub mm_jerk: f64,
    pub slow_mm_cruise_velocity: f64,
    pub slow_mm_acceleration: f64,
    pub slow_mm_jerk: f64,

    pub sensor_to_mechanism_ratio: f64,
    pub rotor_to_sensor_ratio: f64,
    pub cancoder_rotor_to_sensor_ratio: f64,
    pub cancoder_sensor_to_mechanism_ratio: f64,
    pub cancoder_offset: f64,
    pub cancoder_attached: bool,

    // Resync settings, all driving toward the fully extended hard stop
    pub homing_voltage: f64,
    pub homing_stall_rpm: f64,
    pub homing_min_time_secs: f64,
    pub homing_stall_debounce_secs: f64,
    pub homing_timeout_secs: f64,

    pub intake_x: f64,
    pub intake_y: f64,
    pub extension_mass: f64,
    pub drum_radius_meters: f64,
    pub extension_gearing: f64,
    pub angle: f64,
    pub static_length: f64,
    pub moving_length: f64,
    pub line_width: f64,
    pub max_extension_height: f64,
}

impl Default for IntakeExtensionConfig {
    fn default() -> Self {
        let init_position = 0.0;
        let max_rotations = 2.8;
        let min_rotations = 0.0;
        let supply_current_limit = 20.0;
        let stator_current_limit = 40.0;
        let lower_supply_current_limit = 20.0;
        let lower_supply_current_time = 0.0;
        let position_kp = 13.0;
        let position_ki = 0.0;
        let position_kd = 0.0;
        let position_kv = 1.0;
        let position_ks = 2.0;
        let position_ka = 0.0;
        let position_kg = 0.0;
        let gear_ratio = 11.25;
        let mm_cruise_velocity = 100.0;
        let mm_acceleration = 300.0;
        let mm_jerk = 1000.0;

        let mut base = Config::new("IntakeExtension", 4, Rio::CANIVORE);
        base.config_min_max_rotations(min_rotations, max_rotations);
        base.config_pid_gains_slot(0, position_kp, position_ki, position_kd);
        base.config_feed_forward_gains(position_ks, position_kv, position_ka, position_kg);
        base.config_motion_magic(mm_cruise_velocity, mm_acceleration, mm_jerk);
        base.config_supply_current_limit(supply_current_limit, true);
        base.config_stator_current_limit(stator_current_limit, true);
        base.config_lower_supply_current_limit(lower_supply_current_limit);
        base.config_lower_supply_current_time(lower_supply_current_time);
        base.config_gear_ratio(gear_ratio);
        base.config_forward_torque_current_limit(stator_current_limit);
        base.config_reverse_torque_current_limit(stator_current_limit);
        base.config_forward_soft_limit(max_rotations, true);
        base.config_reverse_soft_limit(min_rotations, true);
        base.config_neutral_brake_mode(true);
        base.config_clockwise_positive();

        Self {
            base,
            init_position,
            trigger_tolerance: 5.0,
            zero_speed: -0.1,
            hold_max_speed_rpm: 18.0,
            max_rotations,
            min_rotations,
            supply_current_limit,
            stator_current_limit,
            lower_supply_current_limit,
            lower_supply_current_time,
            position_kp,
            position_ki,
            position_kd,
            position_kv,
            position_ks,
            position_ka,
            position_kg,
            gear_ratio,
            mm_cruise_velocity,
            mm_acceleration,
            mm_jerk,
            slow_mm_cruise_velocity: 4.0,
            slow_mm_acceleration: 20.0,
            slow_mm_jerk: 1000.0,
            sensor_to_mechanism_ratio: 11.25,
            rotor_to_sensor_ratio: 1.0,
            cancoder_rotor_to_sensor_ratio: 1.7,
            cancoder_sensor_to_mechanism_ratio: 1.0,
            cancoder_offset: 0.0,
            cancoder_attached: false,
            homing_voltage: 6.0,
            homing_stall_rpm: 50.0,
            homing_min_time_secs: 0.3,
            homing_stall_debounce_secs: 0.15,
            homing_timeout_secs: 3.0,
            intake_x: inches_to_meters(70.0),
            intake_y: inches_to_meters(23.0),
            extension_mass: 10.0,
            drum_radius_meters: inches_to_meters(0.955 / 2.0),
            extension_gearing: 11.25,
            angle: 180.0,
            static_length: 10.0,
            moving_length: 55.0,
            line_width: 20.0,
            max_extension_height: 40.0,
        }
    }
}

impl IntakeExtensionConfig {
    pub fn apply_motor_config(&mut self, motor: &TalonFX) -> &mut Self {
        let talon_config_mod: TalonFXConfiguration = self.base.talon_config.clone();

        motor.configurator().apply(&talon_config_mod);
        self.base.talon_config = talon_config_mod;
        self
    }
}

/// Right deploy axis, a standalone [`Mechanism`] rather than a follower so it can be driven on
/// its own during a resync. Gains, limits, and geometry come from the left config; only the
/// CAN id, name, and inversion differ, since the right gearbox is mounted mirrored. That keeps
/// both axes on "positive extends".
pub struct IntakeExtensionRight {
    mechanism: Rc<Mechanism>,
    pub right_config: Config,
}

impl Deref for IntakeExtensionRight {
    type Target = Mechanism;

    fn deref(&self) -> &Mechanism {
        &self.mechanism
    }
}

/// Java's `RightConfig(left)`.
pub fn right_config(left: &IntakeExtensionConfig) -> Config {
    let mut config = Config::new("IntakeExtensionRight", 5, Rio::CANIVORE);
    config.attached = left.base.attached;
    config.config_min_max_rotations(left.min_rotations, left.max_rotations);
    config.config_pid_gains_slot(0, left.position_kp, left.position_ki, left.position_kd);
    config.config_feed_forward_gains(
        left.position_ks,
        left.position_kv,
        left.position_ka,
        left.position_kg,
    );
    config.config_motion_magic(left.mm_cruise_velocity, left.mm_acceleration, left.mm_jerk);
    config.config_supply_current_limit(left.supply_current_limit, true);
    config.config_stator_current_limit(left.stator_current_limit, true);
    config.config_lower_supply_current_limit(left.lower_supply_current_limit);
    config.config_lower_supply_current_time(left.lower_supply_current_time);
    config.config_gear_ratio(left.gear_ratio);
    config.config_forward_torque_current_limit(left.stator_current_limit);
    config.config_reverse_torque_current_limit(left.stator_current_limit);
    config.config_forward_soft_limit(left.max_rotations, true);
    config.config_reverse_soft_limit(left.min_rotations, true);
    config.config_neutral_brake_mode(true);
    config.config_counter_clockwise_positive();
    config
}

impl IntakeExtensionRight {
    pub fn new(left_config: &IntakeExtensionConfig) -> Rc<IntakeExtensionRight> {
        let right = Self::build(left_config);
        telemetry::print(&format!("{} Subsystem Initialized", right.get_name()));
        right
    }

    fn build(left_config: &IntakeExtensionConfig) -> Rc<IntakeExtensionRight> {
        let right_config = right_config(left_config);
        let right = Rc::new(IntakeExtensionRight {
            mechanism: Mechanism::new(right_config.clone()),
            right_config,
        });
        CommandScheduler::get_instance().register_subsystem(right.clone());
        right
    }

    /// Closed-loop Motion Magic to an absolute rotation target.
    pub fn go_to_rotations(&self, rotations: f64) {
        self.set_mm_position(move || rotations);
    }

    /// Dynamic Motion Magic voltage move to a rotation target, using the slow profile.
    pub fn go_to_rotations_slow(
        &self,
        rotations: f64,
        cruise_velocity: f64,
        acceleration: f64,
        jerk: f64,
    ) {
        self.set_dyn_mm_position_voltage(
            move || rotations,
            move || cruise_velocity,
            move || acceleration,
            move || jerk,
        );
    }

    /// Open-loop voltage that ignores soft limits, so homing can reach the hard stop.
    pub fn drive_homing_voltage(&self, volts: f64) {
        self.set_voltage_output_no_soft_limit(move || volts);
    }

    pub fn set_initial_position(&self, rotations: f64) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            motor.set_position(rotations);
        }
    }

    pub fn zero_at_max(&self) {
        self.set_motor_position(|| self.right_config.max_rotations);
    }

    pub fn stop_axis(&self) {
        self.stop();
    }
}

impl Subsystem for IntakeExtensionRight {
    fn id(&self) -> SubsystemId {
        self.mechanism.id()
    }

    fn name(&self) -> String {
        self.get_name()
    }

    fn periodic(&self) {
        self.log_battery_usage();
        telemetry::log(
            "IntakeExtensionRight/CurrentCommand",
            self.get_current_command_name(),
        );
        telemetry::log_with_unit("IntakeExtensionRight/Voltage", self.get_voltage(), "volts");
        telemetry::log_with_unit(
            "IntakeExtensionRight/StatorCurrent",
            self.get_stator_current(),
            "amps",
        );
        telemetry::log_with_unit(
            "IntakeExtensionRight/SupplyCurrent",
            self.get_supply_current(),
            "amps",
        );
        telemetry::log_with_unit(
            "IntakeExtensionRight/Position",
            self.get_position_rotations(),
            "rotations",
        );
        telemetry::log_with_unit("IntakeExtensionRight/RPM", self.get_velocity_rpm(), "RPM");
        telemetry::log_with_unit("IntakeExtensionRight/Temp", self.get_temp(), "deg_C");
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WantedState {
    Stopped,
    FullExtend,
    ConditionalExtend,
    FullRetract,
    SlowClose,
    Resync,
}

impl EnumName for WantedState {
    fn name(&self) -> &'static str {
        match self {
            Self::Stopped => "STOPPED",
            Self::FullExtend => "FULL_EXTEND",
            Self::ConditionalExtend => "CONDITIONAL_EXTEND",
            Self::FullRetract => "FULL_RETRACT",
            Self::SlowClose => "SLOW_CLOSE",
            Self::Resync => "RESYNC",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Stopped,
    FullExtend,
    FullRetract,
    SlowClose,
    Homing,
}

impl EnumName for SystemState {
    fn name(&self) -> &'static str {
        match self {
            Self::Stopped => "STOPPED",
            Self::FullExtend => "FULL_EXTEND",
            Self::FullRetract => "FULL_RETRACT",
            Self::SlowClose => "SLOW_CLOSE",
            Self::Homing => "HOMING",
        }
    }
}

/// `sent_out_by_intake_state` remembers that the intake state extended the intake, so a
/// conditional extend only extends after that.
pub fn handle_state_transition(
    wanted_state: WantedState,
    sent_out_by_intake_state: &mut bool,
) -> SystemState {
    match wanted_state {
        WantedState::Stopped => SystemState::Stopped,
        WantedState::FullExtend => {
            *sent_out_by_intake_state = true;
            SystemState::FullExtend
        }
        WantedState::ConditionalExtend => {
            if *sent_out_by_intake_state {
                SystemState::FullExtend
            } else {
                SystemState::Stopped
            }
        }
        WantedState::FullRetract => {
            *sent_out_by_intake_state = false;
            SystemState::FullRetract
        }
        WantedState::SlowClose => SystemState::SlowClose,
        WantedState::Resync => SystemState::Homing,
    }
}

/// The position both axes are commanded to, as a percentage of max rotations, and whether to
/// use the slow profile. `None` for the states that do not command a position.
pub fn both_command_for(system_state: SystemState) -> Option<(f64, bool)> {
    match system_state {
        SystemState::FullExtend => Some((100.0, false)),
        SystemState::FullRetract => Some((0.0, false)),
        SystemState::SlowClose => Some((25.0, true)),
        SystemState::Homing | SystemState::Stopped => None,
    }
}

/// What one side does on a homing step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SideAction {
    /// Drive the homing voltage toward the hard stop.
    Drive,
    /// Stalled: re-zero at max rotations and stop.
    ZeroAndStop,
    /// Timed out: report and stop, without re-zeroing.
    TimedOut,
    /// Already homed: keep stopped.
    Stop,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HomingStep {
    pub left: SideAction,
    pub right: SideAction,
}

#[derive(Debug, Clone, Copy, Default)]
struct Homing {
    left_homed: bool,
    right_homed: bool,
    // homing timer seconds when each side was last seen above the stall speed
    left_last_moving: f64,
    right_last_moving: f64,
}

/// A side is stalled once it has gone the debounce window without moving above the stall
/// speed, but never before the minimum drive time elapses, so the zero velocity before motion
/// starts does not read as a stall.
fn is_stalled(config: &IntakeExtensionConfig, now: f64, last_moving: f64) -> bool {
    if now < config.homing_min_time_secs {
        return false;
    }
    (now - last_moving) >= config.homing_stall_debounce_secs
}

impl Homing {
    /// Re-arm on entry to the HOMING state.
    fn rearm(&mut self) {
        *self = Self::default();
    }

    fn detect_left_stall(
        &mut self,
        config: &IntakeExtensionConfig,
        now: f64,
        left_velocity_rpm: f64,
    ) -> bool {
        if left_velocity_rpm.abs() >= config.homing_stall_rpm {
            self.left_last_moving = now;
        }
        is_stalled(config, now, self.left_last_moving)
    }

    fn detect_right_stall(
        &mut self,
        config: &IntakeExtensionConfig,
        now: f64,
        right_velocity_rpm: f64,
    ) -> bool {
        if right_velocity_rpm.abs() >= config.homing_stall_rpm {
            self.right_last_moving = now;
        }
        is_stalled(config, now, self.right_last_moving)
    }

    /// One homing cycle. `now`, `left_velocity_rpm` and `right_velocity_rpm` are read where
    /// Java reads them.
    fn step(
        &mut self,
        config: &IntakeExtensionConfig,
        right_attached: bool,
        now: impl Fn() -> f64,
        left_velocity_rpm: impl Fn() -> f64,
        right_velocity_rpm: impl Fn() -> f64,
    ) -> HomingStep {
        let timed_out = now() >= config.homing_timeout_secs;

        let left = if !self.left_homed {
            if self.detect_left_stall(config, now(), left_velocity_rpm()) {
                self.left_homed = true;
                SideAction::ZeroAndStop
            } else if timed_out {
                self.left_homed = true;
                SideAction::TimedOut
            } else {
                SideAction::Drive
            }
        } else {
            SideAction::Stop
        };

        let right = if right_attached {
            if !self.right_homed {
                if self.detect_right_stall(config, now(), right_velocity_rpm()) {
                    self.right_homed = true;
                    SideAction::ZeroAndStop
                } else if timed_out {
                    self.right_homed = true;
                    SideAction::TimedOut
                } else {
                    SideAction::Drive
                }
            } else {
                SideAction::Stop
            }
        } else {
            self.right_homed = true; // no right axis to resync
            SideAction::Stop
        };

        HomingStep { left, right }
    }
}

pub struct IntakeExtension {
    me: Weak<IntakeExtension>,
    mechanism: Rc<Mechanism>,
    pub config: IntakeExtensionConfig,
    sim: RefCell<Option<LinearSim>>,
    right: Rc<IntakeExtensionRight>,

    wanted_state: Cell<WantedState>,
    system_state: Cell<SystemState>,
    previous_system_state: Cell<SystemState>,
    sent_out_by_intake_state: Cell<bool>,

    homing_timer: RefCell<Timer>,
    homing: Cell<Homing>,
}

impl Deref for IntakeExtension {
    type Target = Mechanism;

    fn deref(&self) -> &Mechanism {
        &self.mechanism
    }
}

impl IntakeExtension {
    pub fn new(config: IntakeExtensionConfig) -> Rc<IntakeExtension> {
        let extension = Self::build(config, wpilib::robot_controller::time);
        telemetry::print(&format!(
            "{} Subsystem Initialized",
            extension.right.get_name()
        ));
        extension.set_initial_position();

        extension.simulation_init();
        telemetry::print(&format!("{} Subsystem Initialized", extension.get_name()));
        extension
    }

    /// `clock` is the homing timer's microsecond clock.
    fn build(config: IntakeExtensionConfig, clock: fn() -> u64) -> Rc<IntakeExtension> {
        let mechanism = Mechanism::new(config.base.clone());
        let right = IntakeExtensionRight::build(&config);
        let extension = Rc::new_cyclic(|me| IntakeExtension {
            me: me.clone(),
            mechanism,
            config,
            sim: RefCell::new(None),
            right,
            wanted_state: Cell::new(WantedState::Stopped),
            system_state: Cell::new(SystemState::Stopped),
            previous_system_state: Cell::new(SystemState::Stopped),
            sent_out_by_intake_state: Cell::new(false),
            homing_timer: RefCell::new(Timer::with_clock(clock)),
            homing: Cell::new(Homing::default()),
        });
        CommandScheduler::get_instance().register_subsystem(extension.clone());
        extension
    }

    fn rc(&self) -> Rc<IntakeExtension> {
        self.me
            .upgrade()
            .expect("IntakeExtension is only built through its constructor, which returns an Rc")
    }

    pub fn get_sim(&self) -> Option<LinearSim> {
        self.sim.borrow().clone()
    }

    fn set_initial_position(&self) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            let initial_rotations = self.config.init_position;
            motor.set_position(initial_rotations);
            self.right.set_initial_position(initial_rotations);
        }
    }

    pub fn reset_current_position_to_max(&self) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            motor.set_position(self.config.max_rotations);
        }
        if self.right.is_attached() {
            self.right.zero_at_max();
        }
    }

    pub fn reset_current_position_to_max_command(&self) -> Command {
        let me = self.rc();
        Commands::run_once(move || me.reset_current_position_to_max(), &[])
    }

    pub fn reset_to_initial_pos(&self) -> Command {
        let me = self.rc();
        Commands::run_once(move || me.set_initial_position(), &[])
    }

    /// Sets brake mode on both deploy axes.
    pub fn set_brake_mode(&self, is_in_brake: bool) {
        self.mechanism.set_brake_mode(is_in_brake);
        if self.right.is_attached() {
            self.right.set_brake_mode(is_in_brake);
        }
    }

    pub fn set_wanted_state(&self, state: WantedState) {
        self.wanted_state.set(state);
    }

    pub fn get_wanted_state(&self) -> WantedState {
        self.wanted_state.get()
    }

    pub fn get_system_state(&self) -> SystemState {
        self.system_state.get()
    }

    fn handle_state_transition(&self) -> SystemState {
        let mut sent_out = self.sent_out_by_intake_state.get();
        let state = handle_state_transition(self.wanted_state.get(), &mut sent_out);
        self.sent_out_by_intake_state.set(sent_out);
        state
    }

    fn apply_states(&self) {
        let state = self.system_state.get();
        if let Some((percent, slow)) = both_command_for(state) {
            self.command_both(percent, slow);
            return;
        }
        match state {
            SystemState::Homing => self.apply_homing(),
            SystemState::Stopped => {
                self.stop();
                if self.right.is_attached() {
                    self.right.stop_axis();
                }
            }
            SystemState::FullExtend | SystemState::FullRetract | SystemState::SlowClose => {}
        }
    }

    /// Commands both axes to the same position, given as a percentage of max rotations.
    ///
    /// `percent` is 0 to 100. `slow` uses the slow dynamic Motion Magic voltage profile.
    fn command_both(&self, percent: f64, slow: bool) {
        let rotations = self.percent_to_rotations(|| percent);
        if slow {
            let (cruise, acceleration, jerk) = (
                self.config.slow_mm_cruise_velocity,
                self.config.slow_mm_acceleration,
                self.config.slow_mm_jerk,
            );
            self.set_dyn_mm_position_voltage(
                move || rotations,
                move || cruise,
                move || acceleration,
                move || jerk,
            );
            if self.right.is_attached() {
                self.right
                    .go_to_rotations_slow(rotations, cruise, acceleration, jerk);
            }
        } else {
            self.set_mm_position(move || rotations);
            if self.right.is_attached() {
                self.right.go_to_rotations(rotations);
            }
        }
    }

    /// Drives each side into the fully extended hard stop and re-zeroes that side's encoder at
    /// max rotations once it stalls. After a tooth skip the hard stop is the only position
    /// that can be trusted, so homing uses a voltage that bypasses the soft limits and reaches
    /// the physical stop even when the stale encoder thinks the soft limit is already reached.
    fn apply_homing(&self) {
        let mut homing = self.homing.get();
        if self.previous_system_state.get() != SystemState::Homing {
            self.homing_timer.borrow_mut().restart();
            homing.rearm();
        }

        let step = homing.step(
            &self.config,
            self.right.is_attached(),
            || self.homing_timer.borrow().get(),
            || self.get_velocity_rpm(),
            || self.right.get_velocity_rpm(),
        );
        self.homing.set(homing);

        match step.left {
            SideAction::ZeroAndStop => {
                self.set_motor_position(|| self.config.max_rotations);
                self.stop();
            }
            SideAction::TimedOut => {
                telemetry::print("IntakeExtension: LEFT resync timed out");
                self.stop();
            }
            SideAction::Drive => {
                let volts = self.config.homing_voltage;
                self.set_voltage_output_no_soft_limit(move || volts);
            }
            SideAction::Stop => self.stop(),
        }

        if self.right.is_attached() {
            match step.right {
                SideAction::ZeroAndStop => {
                    self.right.zero_at_max();
                    self.right.stop_axis();
                }
                SideAction::TimedOut => {
                    telemetry::print("IntakeExtension: RIGHT resync timed out");
                    self.right.stop_axis();
                }
                SideAction::Drive => self.right.drive_homing_voltage(self.config.homing_voltage),
                SideAction::Stop => self.right.stop_axis(),
            }
        }
    }

    /// Whether the most recent resync has finished homing both sides.
    pub fn is_resync_complete(&self) -> bool {
        let homing = self.homing.get();
        self.system_state.get() == SystemState::Homing && homing.left_homed && homing.right_homed
    }

    /// Drives both sides into the extended hard stop and re-zeroes each one as it stalls, then
    /// returns the subsystem to STOPPED. A homing timeout can end the command with a side that
    /// never stalled, and that side is then left where it stopped rather than re-zeroed.
    ///
    /// The command ends once both sides are re-zeroed, or once the homing timeout hits.
    pub fn resync_command(&self) -> Command {
        let (start, end, done) = (self.rc(), self.rc(), self.rc());
        self.start_end(
            move || start.set_wanted_state(WantedState::Resync),
            move || end.set_wanted_state(WantedState::Stopped),
        )
        .until(move || done.is_resync_complete())
        .with_name("IntakeExtension.resync")
    }

    pub fn simulation_init(&self) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            *self.sim.borrow_mut() = Some(self.intake_extension_sim(motor));
        }
    }

    /// Java's inner `IntakeExtensionSim`.
    fn intake_extension_sim(&self, motor: &TalonFX) -> LinearSim {
        let config = &self.config;
        LinearSim::new(
            LinearConfig::new(
                config.intake_x,
                config.intake_y,
                config.extension_gearing,
                config.drum_radius_meters,
            )
            .set_angle(config.angle)
            .set_moving_length(config.moving_length)
            .set_static_length(config.static_length)
            .set_max_height(config.max_extension_height)
            .set_line_width(config.line_width)
            .set_color(Color8Bit::LIGHT_GRAY),
            &context::left_view(),
            motor,
            &config.base.name,
        )
    }
}

impl Subsystem for IntakeExtension {
    fn id(&self) -> SubsystemId {
        self.mechanism.id()
    }

    fn name(&self) -> String {
        self.get_name()
    }

    fn periodic(&self) {
        self.system_state.set(self.handle_state_transition());
        self.apply_states();
        self.log_battery_usage();
        telemetry::log_dash(
            "IntakeExtension/WantedState",
            self.wanted_state.get().name(),
        );
        telemetry::log_dash(
            "IntakeExtension/SystemState",
            self.system_state.get().name(),
        );
        telemetry::log_dash(
            "IntakeExtension/CurrentCommand",
            self.get_current_command_name(),
        );
        telemetry::log_dash_with_unit("IntakeExtension/Voltage", self.get_voltage(), "volts");
        telemetry::log_dash_with_unit(
            "IntakeExtension/StatorCurrent",
            self.get_stator_current(),
            "amps",
        );
        telemetry::log_dash_with_unit(
            "IntakeExtension/SupplyCurrent",
            self.get_supply_current(),
            "amps",
        );
        telemetry::log_dash_with_unit(
            "IntakeExtension/Position",
            self.get_position_rotations(),
            "rotations",
        );
        telemetry::log_dash_with_unit("IntakeExtension/RPM", self.get_velocity_rpm(), "RPM");
        telemetry::log_dash_with_unit("IntakeExtension/Temp", self.get_temp(), "deg_C");
        let homing = self.homing.get();
        telemetry::log("IntakeExtension/LeftHomed", homing.left_homed);
        telemetry::log("IntakeExtension/RightHomed", homing.right_homed);

        self.previous_system_state.set(self.system_state.get());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use commands::CommandScheduler;
    use phoenix6::signals::{InvertedValue, NeutralModeValue};
    use std::sync::atomic::{AtomicU64, Ordering};

    static MICROS: AtomicU64 = AtomicU64::new(0);

    fn fake_clock() -> u64 {
        MICROS.load(Ordering::SeqCst)
    }

    fn unattached() -> Rc<IntakeExtension> {
        let mut config = IntakeExtensionConfig::default();
        config.base.attached = false;
        IntakeExtension::build(config, fake_clock)
    }

    #[test]
    fn stopped_and_resync_map_straight_across() {
        let mut sent_out = false;
        assert_eq!(
            handle_state_transition(WantedState::Stopped, &mut sent_out),
            SystemState::Stopped
        );
        assert_eq!(
            handle_state_transition(WantedState::SlowClose, &mut sent_out),
            SystemState::SlowClose
        );
        assert_eq!(
            handle_state_transition(WantedState::Resync, &mut sent_out),
            SystemState::Homing
        );
        assert!(!sent_out);
    }

    #[test]
    fn conditional_extend_only_extends_after_a_full_extend() {
        let mut sent_out = false;
        assert_eq!(
            handle_state_transition(WantedState::ConditionalExtend, &mut sent_out),
            SystemState::Stopped
        );
        assert_eq!(
            handle_state_transition(WantedState::FullExtend, &mut sent_out),
            SystemState::FullExtend
        );
        assert!(sent_out);
        assert_eq!(
            handle_state_transition(WantedState::ConditionalExtend, &mut sent_out),
            SystemState::FullExtend
        );
        assert_eq!(
            handle_state_transition(WantedState::FullRetract, &mut sent_out),
            SystemState::FullRetract
        );
        assert!(!sent_out);
        assert_eq!(
            handle_state_transition(WantedState::ConditionalExtend, &mut sent_out),
            SystemState::Stopped
        );
    }

    #[test]
    fn each_position_state_commands_both_axes_to_its_percentage() {
        assert_eq!(
            both_command_for(SystemState::FullExtend),
            Some((100.0, false))
        );
        assert_eq!(
            both_command_for(SystemState::FullRetract),
            Some((0.0, false))
        );
        assert_eq!(both_command_for(SystemState::SlowClose), Some((25.0, true)));
        assert_eq!(both_command_for(SystemState::Homing), None);
        assert_eq!(both_command_for(SystemState::Stopped), None);
    }

    #[test]
    fn the_extension_starts_stopped() {
        let extension = unattached();
        assert_eq!(extension.get_wanted_state(), WantedState::Stopped);
        assert_eq!(extension.get_system_state(), SystemState::Stopped);
        extension.set_wanted_state(WantedState::FullExtend);
        assert_eq!(extension.get_wanted_state(), WantedState::FullExtend);
    }

    #[test]
    fn full_extend_percent_is_max_rotations() {
        let extension = unattached();
        assert!((extension.percent_to_rotations(|| 100.0) - 2.8).abs() < 1e-12);
        assert!((extension.percent_to_rotations(|| 25.0) - 0.7).abs() < 1e-12);
    }

    #[test]
    fn applying_states_to_an_unattached_extension_commands_nothing() {
        let extension = unattached();
        for state in [
            SystemState::Stopped,
            SystemState::FullExtend,
            SystemState::FullRetract,
            SystemState::SlowClose,
            SystemState::Homing,
        ] {
            extension.system_state.set(state);
            extension.apply_states();
        }
        assert!(extension.get_sim().is_none());
    }

    fn config() -> IntakeExtensionConfig {
        IntakeExtensionConfig::default()
    }

    #[test]
    fn a_side_is_not_stalled_before_the_minimum_drive_time() {
        let config = config();
        assert!(!is_stalled(&config, 0.29, 0.0));
        assert!(is_stalled(&config, 0.3, 0.0));
    }

    #[test]
    fn a_side_stalls_after_the_debounce_window_without_motion() {
        let config = config();
        assert!(!is_stalled(&config, 1.0, 0.86));
        assert!(is_stalled(&config, 1.0, 0.85));
        assert!(is_stalled(&config, 1.0, 0.2));
    }

    #[test]
    fn homing_drives_until_a_side_stalls_then_zeroes_and_stops_it() {
        let config = config();
        let mut homing = Homing::default();
        let time = Cell::new(0.0);
        let now = || time.get();
        let moving = || 200.0;
        let still = || 0.0;

        // Both sides moving: both keep driving.
        time.set(0.5);
        let step = homing.step(&config, true, now, moving, moving);
        assert_eq!(
            step,
            HomingStep {
                left: SideAction::Drive,
                right: SideAction::Drive
            }
        );
        assert_eq!(homing.left_last_moving, 0.5);

        // Left stops moving and the debounce passes. Right keeps going.
        time.set(0.7);
        let step = homing.step(&config, true, now, still, moving);
        assert_eq!(
            step,
            HomingStep {
                left: SideAction::ZeroAndStop,
                right: SideAction::Drive
            }
        );
        assert!(homing.left_homed && !homing.right_homed);

        // Left is done and stays stopped. Right stalls.
        time.set(1.0);
        let step = homing.step(&config, true, now, still, still);
        assert_eq!(
            step,
            HomingStep {
                left: SideAction::Stop,
                right: SideAction::ZeroAndStop
            }
        );
        assert!(homing.right_homed);
    }

    #[test]
    fn homing_times_out_a_side_that_never_stalls() {
        let config = config();
        let mut homing = Homing::default();
        let time = Cell::new(3.0);
        let moving = || 200.0;
        let step = homing.step(&config, true, || time.get(), moving, moving);
        assert_eq!(
            step,
            HomingStep {
                left: SideAction::TimedOut,
                right: SideAction::TimedOut
            }
        );
        assert!(homing.left_homed && homing.right_homed);
    }

    #[test]
    fn homing_counts_an_unattached_right_side_as_done() {
        let config = config();
        let mut homing = Homing::default();
        let step = homing.step(&config, false, || 0.1, || 200.0, || 0.0);
        assert_eq!(step.left, SideAction::Drive);
        assert_eq!(step.right, SideAction::Stop);
        assert!(homing.right_homed);
    }

    #[test]
    fn rearming_clears_the_homing_progress() {
        let mut homing = Homing {
            left_homed: true,
            right_homed: true,
            left_last_moving: 1.0,
            right_last_moving: 2.0,
        };
        homing.rearm();
        assert!(!homing.left_homed && !homing.right_homed);
        assert_eq!(
            (homing.left_last_moving, homing.right_last_moving),
            (0.0, 0.0)
        );
    }

    #[test]
    fn the_resync_command_asks_for_a_resync_and_ends_stopped() {
        MICROS.store(0, Ordering::SeqCst);
        let extension = unattached();
        let command = extension.resync_command();
        assert_eq!(command.get_name(), "IntakeExtension.resync");
        let scheduler = CommandScheduler::get_instance();
        scheduler.schedule(&command);
        assert_eq!(extension.get_wanted_state(), WantedState::Resync);
        scheduler.cancel(&command);
        assert_eq!(extension.get_wanted_state(), WantedState::Stopped);
    }

    #[test]
    fn resync_is_complete_only_while_homing_with_both_sides_homed() {
        let extension = unattached();
        assert!(!extension.is_resync_complete());
        extension.system_state.set(SystemState::Homing);
        assert!(!extension.is_resync_complete());
        extension.homing.set(Homing {
            left_homed: true,
            right_homed: true,
            ..Homing::default()
        });
        assert!(extension.is_resync_complete());
        extension.system_state.set(SystemState::Stopped);
        assert!(!extension.is_resync_complete());
    }

    #[test]
    fn the_right_config_copies_the_left_gains_and_mirrors_the_inversion() {
        let left = IntakeExtensionConfig::default();
        let right = right_config(&left);
        assert_eq!(right.name, "IntakeExtensionRight");
        assert_eq!(right.id.get_device_number(), 5);
        assert_eq!(right.attached, left.base.attached);
        assert_eq!((right.min_rotations, right.max_rotations), (0.0, 2.8));
        assert_eq!(right.talon_config.slot0.k_p, left.position_kp);
        assert_eq!(right.talon_config.slot0.k_v, left.position_kv);
        assert_eq!(
            right.talon_config.feedback.sensor_to_mechanism_ratio,
            left.gear_ratio
        );
        assert_eq!(
            right.talon_config.motor_output.inverted,
            InvertedValue::CounterClockwise_Positive
        );
        assert_eq!(
            left.base.talon_config.motor_output.inverted,
            InvertedValue::Clockwise_Positive
        );
    }

    #[test]
    fn the_right_config_follows_an_unattached_left() {
        let mut left = IntakeExtensionConfig::default();
        left.base.attached = false;
        assert!(!right_config(&left).attached);
    }

    #[test]
    fn config_holds_the_java_motor_setup() {
        let config = IntakeExtensionConfig::default();
        let base = &config.base;
        assert_eq!(base.name, "IntakeExtension");
        assert_eq!(base.id.get_device_number(), 4);
        let talon = &base.talon_config;
        assert_eq!(talon.slot0.k_p, 13.0);
        assert_eq!(talon.slot0.k_v, 1.0);
        assert_eq!(talon.slot0.k_s, 2.0);
        assert_eq!(talon.feedback.sensor_to_mechanism_ratio, 11.25);
        assert_eq!(talon.current_limits.supply_current_limit, 20.0);
        assert_eq!(talon.current_limits.stator_current_limit, 40.0);
        assert_eq!(
            talon.software_limit_switch.forward_soft_limit_threshold,
            2.8
        );
        assert_eq!(
            talon.software_limit_switch.reverse_soft_limit_threshold,
            0.0
        );
        assert_eq!(talon.motion_magic.motion_magic_cruise_velocity, 100.0);
        assert_eq!(talon.motor_output.neutral_mode, NeutralModeValue::Brake);
        assert!((config.drum_radius_meters - inches_to_meters(0.4775)).abs() < 1e-12);
        assert_eq!(config.homing_timeout_secs, 3.0);
    }
}
