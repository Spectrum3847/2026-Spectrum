//! Lifts fuel from the bed up to the launcher.

use crate::spectrum_lib::hardware::rio::Rio;
use crate::spectrum_lib::mechanism::mechanism::{Config, FollowerConfig, Mechanism};
use commands::{CommandScheduler, Subsystem, SubsystemId};
use phoenix6::signals::MotorAlignmentValue;
use std::cell::Cell;
use std::ops::Deref;
use std::rc::Rc;
use telemetry::{EnumName, telemetry};
use wpimath::units::inches_to_meters;

#[derive(Clone, Debug)]
pub struct IndexerTowerConfig {
    pub base: Config,
    pub supply_current_limit: f64,
    pub stator_current_limit: f64,
    pub lower_supply_current_limit: f64,
    pub lower_supply_current_time: f64,
    pub velocity_kp: f64,
    pub velocity_kv: f64,
    pub velocity_ks: f64,

    pub intake_x: f64,
    pub intake_y: f64,
    pub wheel_diameter: f64,
}

impl Default for IndexerTowerConfig {
    fn default() -> Self {
        let supply_current_limit = 40.0;
        let stator_current_limit = 180.0;
        let lower_supply_current_limit = 40.0;
        let lower_supply_current_time = 0.0;
        let velocity_kp = 50.0;
        let velocity_kv = 0.0;
        let velocity_ks = 40.0;

        let mut base = Config::new("IndexerTower", 51, Rio::CANIVORE);
        base.config_pid_gains_slot(0, velocity_kp, 0.0, 0.0);
        base.config_feed_forward_gains(velocity_ks, velocity_kv, 0.0, 0.0);
        base.config_gear_ratio(1.0);
        base.config_supply_current_limit(supply_current_limit, true);
        base.config_stator_current_limit(stator_current_limit, true);
        base.config_forward_torque_current_limit(stator_current_limit);
        base.config_reverse_torque_current_limit(stator_current_limit);
        base.config_lower_supply_current_limit(lower_supply_current_limit);
        base.config_lower_supply_current_time(lower_supply_current_time);
        base.config_neutral_brake_mode(true);
        base.config_clockwise_positive();
        base.set_follower_configs(vec![FollowerConfig::new(
            "IndexerTower Follower",
            52,
            Rio::CANIVORE,
            MotorAlignmentValue::Aligned,
        )]);

        Self {
            base,
            supply_current_limit,
            stator_current_limit,
            lower_supply_current_limit,
            lower_supply_current_time,
            velocity_kp,
            velocity_kv,
            velocity_ks,
            intake_x: inches_to_meters(60.0),
            intake_y: inches_to_meters(75.0),
            wheel_diameter: 12.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WantedState {
    Off,
    IndexMax,
    SlowIndex,
    Unjam,
}

impl EnumName for WantedState {
    fn name(&self) -> &'static str {
        match self {
            Self::Off => "OFF",
            Self::IndexMax => "INDEX_MAX",
            Self::SlowIndex => "SLOW_INDEX",
            Self::Unjam => "UNJAM",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Off,
    IndexMax,
    SlowIndex,
    Unjam,
}

impl EnumName for SystemState {
    fn name(&self) -> &'static str {
        match self {
            Self::Off => "OFF",
            Self::IndexMax => "INDEX_MAX",
            Self::SlowIndex => "SLOW_INDEX",
            Self::Unjam => "UNJAM",
        }
    }
}

/// What a system state asks of the motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Output {
    Stop,
    VelocityTcFocRpm(f64),
}

pub fn handle_state_transition(wanted_state: WantedState) -> SystemState {
    match wanted_state {
        WantedState::Off => SystemState::Off,
        WantedState::IndexMax => SystemState::IndexMax,
        WantedState::SlowIndex => SystemState::SlowIndex,
        WantedState::Unjam => SystemState::Unjam,
    }
}

pub fn output_for(system_state: SystemState) -> Output {
    match system_state {
        SystemState::Off => Output::Stop,
        SystemState::IndexMax => Output::VelocityTcFocRpm(4000.0),
        SystemState::SlowIndex => Output::VelocityTcFocRpm(1000.0),
        SystemState::Unjam => Output::VelocityTcFocRpm(-1500.0),
    }
}

pub struct IndexerTower {
    mechanism: Rc<Mechanism>,
    wanted_state: Cell<WantedState>,
    system_state: Cell<SystemState>,
    pub config: IndexerTowerConfig,
}

impl Deref for IndexerTower {
    type Target = Mechanism;

    fn deref(&self) -> &Mechanism {
        &self.mechanism
    }
}

impl IndexerTower {
    pub fn new(config: IndexerTowerConfig) -> Rc<IndexerTower> {
        let indexer = Self::build(config);
        telemetry::print(&format!("{} Subsystem Initialized", indexer.get_name()));
        indexer
    }

    fn build(config: IndexerTowerConfig) -> Rc<IndexerTower> {
        let indexer = Rc::new(IndexerTower {
            mechanism: Mechanism::new(config.base.clone()),
            wanted_state: Cell::new(WantedState::Off),
            system_state: Cell::new(SystemState::Off),
            config,
        });
        CommandScheduler::get_instance().register_subsystem(indexer.clone());
        indexer
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
        match output_for(self.system_state.get()) {
            Output::Stop => self.stop(),
            Output::VelocityTcFocRpm(wanted_rpm) => self.set_velocity_tcfoc_rpm(move || wanted_rpm),
        }
    }
}

impl Subsystem for IndexerTower {
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
        telemetry::log_dash("IndexerTower/WantedState", self.wanted_state.get().name());
        telemetry::log_dash("IndexerTower/SystemState", self.system_state.get().name());
        telemetry::log_dash(
            "IndexerTower/CurrentCommand",
            self.get_current_command_name(),
        );
        telemetry::log_dash_with_unit("IndexerTower/Voltage", self.get_voltage(), "volts");
        telemetry::log_dash_with_unit(
            "IndexerTower/StatorCurrent",
            self.get_stator_current(),
            "amps",
        );
        telemetry::log_dash_with_unit(
            "IndexerTower/SupplyCurrent",
            self.get_supply_current(),
            "amps",
        );
        telemetry::log_dash_with_unit("IndexerTower/RPM", self.get_velocity_rpm(), "RPM");
        telemetry::log_dash_with_unit("IndexerTower/Temp", self.get_temp(), "deg_C");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoenix6::signals::{InvertedValue, NeutralModeValue};

    fn unattached() -> Rc<IndexerTower> {
        let mut config = IndexerTowerConfig::default();
        config.base.attached = false;
        IndexerTower::build(config)
    }

    #[test]
    fn every_wanted_state_becomes_the_system_state_of_the_same_name() {
        let pairs = [
            (WantedState::Off, SystemState::Off),
            (WantedState::IndexMax, SystemState::IndexMax),
            (WantedState::SlowIndex, SystemState::SlowIndex),
            (WantedState::Unjam, SystemState::Unjam),
        ];
        for (wanted, system) in pairs {
            assert_eq!(handle_state_transition(wanted), system);
            assert_eq!(wanted.name(), system.name());
        }
    }

    #[test]
    fn each_state_picks_its_velocity() {
        assert_eq!(output_for(SystemState::Off), Output::Stop);
        assert_eq!(
            output_for(SystemState::IndexMax),
            Output::VelocityTcFocRpm(4000.0)
        );
        assert_eq!(
            output_for(SystemState::SlowIndex),
            Output::VelocityTcFocRpm(1000.0)
        );
        assert_eq!(
            output_for(SystemState::Unjam),
            Output::VelocityTcFocRpm(-1500.0)
        );
    }

    #[test]
    fn the_indexer_starts_off() {
        let indexer = unattached();
        assert_eq!(indexer.get_wanted_state(), WantedState::Off);
        assert_eq!(indexer.get_system_state(), SystemState::Off);
        indexer.set_wanted_state(WantedState::Unjam);
        assert_eq!(indexer.get_wanted_state(), WantedState::Unjam);
    }

    #[test]
    fn applying_states_to_an_unattached_indexer_commands_nothing() {
        let indexer = unattached();
        for state in [
            SystemState::Off,
            SystemState::IndexMax,
            SystemState::SlowIndex,
            SystemState::Unjam,
        ] {
            indexer.system_state.set(state);
            indexer.apply_states();
        }
    }

    #[test]
    fn config_holds_the_java_motor_setup() {
        let config = IndexerTowerConfig::default();
        let base = &config.base;
        assert_eq!(base.name, "IndexerTower");
        assert_eq!(base.id.get_device_number(), 51);
        assert_eq!(base.follower_configs.len(), 1);
        assert_eq!(base.follower_configs[0].id.get_device_number(), 52);
        assert_eq!(
            base.follower_configs[0].oppose_leader,
            MotorAlignmentValue::Aligned
        );
        let talon = &base.talon_config;
        assert_eq!(talon.slot0.k_p, 50.0);
        assert_eq!(talon.slot0.k_s, 40.0);
        assert_eq!(talon.current_limits.supply_current_limit, 40.0);
        assert_eq!(talon.current_limits.stator_current_limit, 180.0);
        assert_eq!(talon.motor_output.neutral_mode, NeutralModeValue::Brake);
        assert_eq!(
            talon.motor_output.inverted,
            InvertedValue::Clockwise_Positive
        );
    }
}
