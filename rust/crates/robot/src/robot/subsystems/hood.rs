//! Sets the hood angle that aims the fuel shot.

use crate::context;
use crate::spectrum_lib::hardware::rio::Rio;
use crate::spectrum_lib::mechanism::mechanism::{Config, Mechanism};
use crate::spectrum_lib::sim::arm_config::ArmConfig;
use crate::spectrum_lib::sim::arm_sim::ArmSim;
use commands::{CommandScheduler, Subsystem, SubsystemId};
use std::cell::{Cell, RefCell};
use std::ops::Deref;
use std::rc::Rc;
use telemetry::{EnumName, telemetry};
use wpimath::units::inches_to_meters;

#[derive(Clone, Debug)]
pub struct HoodConfig {
    pub base: Config,

    pub init_position: f64,

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
    pub hold_max_speed_rpm: f64,

    pub hood_x: f64,
    pub hood_y: f64,
    pub sim_ratio: f64,
    pub length: f64,
}

impl Default for HoodConfig {
    fn default() -> Self {
        let init_position = 9.0;

        let max_rotations = 0.137;
        let min_rotations = 0.024;

        let supply_current_limit = 40.0;
        let stator_current_limit = 60.0;
        let lower_supply_current_limit = 40.0;
        let lower_supply_current_time = 1.0;
        let position_kp = 3000.0;
        let position_ki = 0.0;
        let position_kd = 220.0;
        let position_kv = 0.0;
        let position_ks = 25.0;
        let position_ka = 0.0;
        let position_kg = 0.0;

        let gear_ratio = 51.667;
        let mm_cruise_velocity = 50.0;
        let mm_acceleration = 200.0;
        let mm_jerk = 1000.0;

        let mut base = Config::new("Hood", 15, Rio::CANIVORE);
        base.config_min_max_rotations(min_rotations, max_rotations);
        base.config_pid_gains_slot(0, position_kp, position_ki, position_kd);
        base.config_feed_forward_gains(position_ks, position_kv, position_ka, position_kg);
        base.config_motion_magic(mm_cruise_velocity, mm_acceleration, mm_jerk);
        base.config_gear_ratio(gear_ratio);
        base.config_supply_current_limit(supply_current_limit, true);
        base.config_stator_current_limit(stator_current_limit, true);
        base.config_lower_supply_current_limit(lower_supply_current_limit);
        base.config_lower_supply_current_time(lower_supply_current_time);
        base.config_forward_torque_current_limit(stator_current_limit);
        base.config_reverse_torque_current_limit(stator_current_limit);
        base.config_forward_soft_limit(max_rotations, true);
        base.config_reverse_soft_limit(min_rotations, true);
        base.config_neutral_brake_mode(true);
        base.config_clockwise_positive();

        Self {
            base,
            init_position,
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
            hold_max_speed_rpm: 18.0,
            hood_x: inches_to_meters(62.5),
            hood_y: inches_to_meters(50.0),
            sim_ratio: 51.667,
            length: inches_to_meters(10.0),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WantedState {
    Home,
    Stopped,
    AimAtTarget,
}

impl EnumName for WantedState {
    fn name(&self) -> &'static str {
        match self {
            Self::Home => "HOME",
            Self::Stopped => "STOPPED",
            Self::AimAtTarget => "AIM_AT_TARGET",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Home,
    Stopped,
    AimAtTarget,
}

impl EnumName for SystemState {
    fn name(&self) -> &'static str {
        match self {
            Self::Home => "HOME",
            Self::Stopped => "STOPPED",
            Self::AimAtTarget => "AIM_AT_TARGET",
        }
    }
}

/// What a system state asks of the motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Output {
    Stop,
    MmPositionDegrees(f64),
}

pub fn handle_state_transition(wanted_state: WantedState) -> SystemState {
    match wanted_state {
        WantedState::Home => SystemState::Home,
        WantedState::Stopped => SystemState::Stopped,
        WantedState::AimAtTarget => SystemState::AimAtTarget,
    }
}

/// `aim_degrees` is only called in `AimAtTarget`, where Java asks the shot calculator.
pub fn output_for(system_state: SystemState, aim_degrees: impl FnOnce() -> f64) -> Output {
    match system_state {
        SystemState::Home => Output::MmPositionDegrees(9.0),
        SystemState::Stopped => Output::Stop,
        SystemState::AimAtTarget => Output::MmPositionDegrees(aim_degrees()),
    }
}

pub struct Hood {
    mechanism: Rc<Mechanism>,
    wanted_state: Cell<WantedState>,
    system_state: Cell<SystemState>,
    pub config: HoodConfig,
    sim: RefCell<Option<ArmSim>>,
}

impl Deref for Hood {
    type Target = Mechanism;

    fn deref(&self) -> &Mechanism {
        &self.mechanism
    }
}

impl Hood {
    pub fn new(config: HoodConfig) -> Rc<Hood> {
        let hood = Self::build(config);
        hood.set_initial_position();
        hood.simulation_init();
        telemetry::print(&format!("{} Subsystem Initialized", hood.get_name()));
        hood
    }

    fn build(config: HoodConfig) -> Rc<Hood> {
        let hood = Rc::new(Hood {
            mechanism: Mechanism::new(config.base.clone()),
            wanted_state: Cell::new(WantedState::Home),
            system_state: Cell::new(SystemState::Home),
            config,
            sim: RefCell::new(None),
        });
        CommandScheduler::get_instance().register_subsystem(hood.clone());
        hood
    }

    pub fn get_sim(&self) -> Option<ArmSim> {
        self.sim.borrow().clone()
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

    fn apply_states(&self) {
        let output = output_for(self.system_state.get(), || {
            context::shot_calculator().get_parameters().hood_angle
        });
        match output {
            Output::Stop => self.stop(),
            Output::MmPositionDegrees(wanted_degrees) => {
                let wanted_position = self.degrees_to_rotations(move || wanted_degrees);
                self.set_mm_position_foc(move || wanted_position);
            }
        }
    }

    fn set_initial_position(&self) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            motor.set_position(self.degrees_to_rotations(|| self.config.init_position));
        }
    }

    pub fn simulation_init(&self) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            *self.sim.borrow_mut() = Some(self.hood_sim(motor));
        }
    }

    /// Java's inner `HoodSim`.
    fn hood_sim(&self, motor: &phoenix6::TalonFX) -> ArmSim {
        ArmSim::new(
            ArmConfig::new(
                self.config.hood_x,
                self.config.hood_y,
                self.config.sim_ratio,
                self.config.length,
                90.0,
                180.0 - 9.0,
                180.0 - 9.0,
            )
            .set_simulated_gravity(false),
            &context::left_view(),
            motor,
            &self.config.base.name,
        )
    }
}

impl Subsystem for Hood {
    fn id(&self) -> SubsystemId {
        self.mechanism.id()
    }

    fn name(&self) -> String {
        self.get_name()
    }

    fn periodic(&self) {
        self.system_state
            .set(handle_state_transition(self.wanted_state.get()));
        self.apply_states();
        self.log_battery_usage();
        telemetry::log_dash("Hood/WantedState", self.wanted_state.get().name());
        telemetry::log_dash("Hood/SystemState", self.system_state.get().name());
        telemetry::log_dash("Hood/CurrentCommand", self.get_current_command_name());
        telemetry::log_dash_with_unit("Hood/Voltage", self.get_voltage(), "volts");
        telemetry::log_dash_with_unit("Hood/StatorCurrent", self.get_stator_current(), "amps");
        telemetry::log_dash_with_unit("Hood/SupplyCurrent", self.get_supply_current(), "amps");
        telemetry::log_dash_with_unit(
            "Hood/PositionDegrees",
            self.get_position_degrees(),
            "degrees",
        );
        telemetry::log_dash_with_unit("Hood/RPM", self.get_velocity_rpm(), "RPM");
        telemetry::log_dash_with_unit("Hood/Temp", self.get_temp(), "deg_C");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoenix6::signals::{InvertedValue, NeutralModeValue};

    fn unattached() -> Rc<Hood> {
        let mut config = HoodConfig::default();
        config.base.attached = false;
        Hood::build(config)
    }

    #[test]
    fn every_wanted_state_becomes_the_system_state_of_the_same_name() {
        let pairs = [
            (WantedState::Home, SystemState::Home),
            (WantedState::Stopped, SystemState::Stopped),
            (WantedState::AimAtTarget, SystemState::AimAtTarget),
        ];
        for (wanted, system) in pairs {
            assert_eq!(handle_state_transition(wanted), system);
            assert_eq!(wanted.name(), system.name());
        }
    }

    #[test]
    fn each_state_picks_its_hood_output() {
        let never = || panic!("only AIM_AT_TARGET asks the shot calculator");
        assert_eq!(
            output_for(SystemState::Home, never),
            Output::MmPositionDegrees(9.0)
        );
        assert_eq!(output_for(SystemState::Stopped, never), Output::Stop);
        assert_eq!(
            output_for(SystemState::AimAtTarget, || 33.5),
            Output::MmPositionDegrees(33.5)
        );
    }

    #[test]
    fn the_hood_starts_homed() {
        let hood = unattached();
        assert_eq!(hood.get_wanted_state(), WantedState::Home);
        assert_eq!(hood.get_system_state(), SystemState::Home);
        hood.set_wanted_state(WantedState::AimAtTarget);
        assert_eq!(hood.get_wanted_state(), WantedState::AimAtTarget);
    }

    #[test]
    fn applying_states_to_an_unattached_hood_commands_nothing() {
        let hood = unattached();
        for state in [SystemState::Home, SystemState::Stopped] {
            hood.system_state.set(state);
            hood.apply_states();
        }
        assert!(hood.get_sim().is_none());
        assert_eq!(hood.get_target(), 0.0);
    }

    #[test]
    fn home_is_nine_degrees_in_rotations() {
        let hood = unattached();
        let rotations = hood.degrees_to_rotations(|| 9.0);
        assert!((rotations - 0.025).abs() < 1e-12);
        // The soft limits (0.024 to 0.137 rotations) are 8.64 to 49.32 degrees.
        assert!(rotations > hood.config.min_rotations);
    }

    #[test]
    fn config_holds_the_java_motor_setup() {
        let config = HoodConfig::default();
        let base = &config.base;
        assert_eq!(base.name, "Hood");
        assert_eq!(base.id.get_device_number(), 15);
        assert_eq!((base.min_rotations, base.max_rotations), (0.024, 0.137));
        let talon = &base.talon_config;
        assert_eq!(talon.slot0.k_p, 3000.0);
        assert_eq!(talon.slot0.k_d, 220.0);
        assert_eq!(talon.slot0.k_s, 25.0);
        assert_eq!(talon.feedback.sensor_to_mechanism_ratio, 51.667);
        assert_eq!(talon.current_limits.supply_current_limit, 40.0);
        assert_eq!(talon.current_limits.stator_current_limit, 60.0);
        assert_eq!(talon.current_limits.supply_current_lower_time, 1.0);
        assert!(talon.software_limit_switch.forward_soft_limit_enable);
        assert_eq!(
            talon.software_limit_switch.forward_soft_limit_threshold,
            0.137
        );
        assert_eq!(
            talon.software_limit_switch.reverse_soft_limit_threshold,
            0.024
        );
        assert_eq!(talon.motion_magic.motion_magic_cruise_velocity, 50.0);
        assert_eq!(talon.motion_magic.motion_magic_acceleration, 200.0);
        assert_eq!(talon.motion_magic.motion_magic_jerk, 1000.0);
        assert_eq!(talon.motor_output.neutral_mode, NeutralModeValue::Brake);
        assert_eq!(
            talon.motor_output.inverted,
            InvertedValue::Clockwise_Positive
        );
    }
}
