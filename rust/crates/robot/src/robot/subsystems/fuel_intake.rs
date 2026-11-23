//! Pulls fuel into the robot.

use crate::context;
use crate::spectrum_lib::hardware::rio::Rio;
use crate::spectrum_lib::mechanism::mechanism::{Config, FollowerConfig, Mechanism};
use crate::spectrum_lib::sim::roller_config::RollerConfig;
use crate::spectrum_lib::sim::roller_sim::RollerSim;
use commands::{CommandScheduler, Subsystem, SubsystemId};
use phoenix6::signals::MotorAlignmentValue;
use std::cell::{Cell, RefCell};
use std::ops::Deref;
use std::rc::Rc;
use telemetry::{EnumName, telemetry};
use wpimath::units::inches_to_meters;

#[derive(Clone, Debug)]
pub struct FuelIntakeConfig {
    pub base: Config,

    pub supply_current_limit: f64,
    pub stator_current_limit: f64,
    pub lower_supply_current_limit: f64,
    pub lower_supply_current_time: f64,
    pub voltage_limit: f64,
    pub velocity_kp: f64,
    pub velocity_kv: f64,
    pub velocity_ks: f64,

    pub intake_x: f64,
    pub intake_y: f64,
    pub wheel_diameter: f64,
}

impl Default for FuelIntakeConfig {
    fn default() -> Self {
        let supply_current_limit = 45.0;
        let stator_current_limit = 90.0;
        let lower_supply_current_limit = 45.0;
        let lower_supply_current_time = 1.0;
        let voltage_limit = 12.0;
        let velocity_kp = 5.0;
        let velocity_kv = 0.0;
        let velocity_ks = 4.0;

        let mut base = Config::new("Intake", 5, Rio::RIO_CANBUS);
        base.config_pid_gains_slot(0, velocity_kp, 0.0, 0.0);
        base.config_feed_forward_gains(velocity_ks, velocity_kv, 0.0, 0.0);
        base.config_gear_ratio(1.0);
        base.config_supply_current_limit(supply_current_limit, true);
        base.config_stator_current_limit(stator_current_limit, true);
        base.config_lower_supply_current_limit(lower_supply_current_limit);
        base.config_lower_supply_current_time(lower_supply_current_time);
        base.config_forward_torque_current_limit(stator_current_limit);
        base.config_reverse_torque_current_limit(stator_current_limit);
        base.config_forward_voltage_limit(voltage_limit);
        base.config_reverse_voltage_limit(-voltage_limit);
        base.config_neutral_brake_mode(false);
        base.config_counter_clockwise_positive();
        base.set_follower_configs(vec![FollowerConfig::new(
            "Intake Right",
            6,
            Rio::RIO_CANBUS,
            MotorAlignmentValue::Opposed,
        )]);

        Self {
            base,
            supply_current_limit,
            stator_current_limit,
            lower_supply_current_limit,
            lower_supply_current_time,
            voltage_limit,
            velocity_kp,
            velocity_kv,
            velocity_ks,
            intake_x: inches_to_meters(15.0),
            intake_y: inches_to_meters(23.0),
            wheel_diameter: 6.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WantedState {
    Neutral,
    Off,
    Intake,
    Outtake,
    SlowIntake,
}

impl EnumName for WantedState {
    fn name(&self) -> &'static str {
        match self {
            Self::Neutral => "NEUTRAL",
            Self::Off => "OFF",
            Self::Intake => "INTAKE",
            Self::Outtake => "OUTTAKE",
            Self::SlowIntake => "SLOW_INTAKE",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Neutral,
    Off,
    Intake,
    Outtake,
    SlowIntake,
}

impl EnumName for SystemState {
    fn name(&self) -> &'static str {
        match self {
            Self::Neutral => "NEUTRAL",
            Self::Off => "OFF",
            Self::Intake => "INTAKE",
            Self::Outtake => "OUTTAKE",
            Self::SlowIntake => "SLOW_INTAKE",
        }
    }
}

/// What a system state asks of the motor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Output {
    Stop,
    Voltage(f64),
}

pub fn handle_state_transition(wanted_state: WantedState) -> SystemState {
    match wanted_state {
        WantedState::Neutral => SystemState::Neutral,
        WantedState::Intake => SystemState::Intake,
        WantedState::Outtake => SystemState::Outtake,
        WantedState::SlowIntake => SystemState::SlowIntake,
        WantedState::Off => SystemState::Off,
    }
}

pub fn output_for(system_state: SystemState) -> Output {
    match system_state {
        SystemState::Neutral => Output::Voltage(0.0),
        SystemState::Intake => Output::Voltage(12.0),
        SystemState::Outtake => Output::Voltage(-12.0),
        SystemState::SlowIntake => Output::Voltage(5.0),
        SystemState::Off => Output::Stop,
    }
}

pub struct FuelIntake {
    mechanism: Rc<Mechanism>,
    wanted_state: Cell<WantedState>,
    system_state: Cell<SystemState>,
    pub config: FuelIntakeConfig,
    sim: RefCell<Option<RollerSim>>,
}

impl Deref for FuelIntake {
    type Target = Mechanism;

    fn deref(&self) -> &Mechanism {
        &self.mechanism
    }
}

impl FuelIntake {
    pub fn new(config: FuelIntakeConfig) -> Rc<FuelIntake> {
        let intake = Self::build(config);
        intake.simulation_init();
        telemetry::print(&format!("{} Subsystem Initialized", intake.get_name()));
        intake
    }

    fn build(config: FuelIntakeConfig) -> Rc<FuelIntake> {
        let intake = Rc::new(FuelIntake {
            mechanism: Mechanism::new(config.base.clone()),
            wanted_state: Cell::new(WantedState::Neutral),
            system_state: Cell::new(SystemState::Neutral),
            config,
            sim: RefCell::new(None),
        });
        CommandScheduler::get_instance().register_subsystem(intake.clone());
        intake
    }

    pub fn get_sim(&self) -> Option<RollerSim> {
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
        match output_for(self.system_state.get()) {
            Output::Stop => self.stop(),
            Output::Voltage(wanted_voltage) => self.set_voltage_output(move || wanted_voltage),
        }
    }

    pub fn simulation_init(&self) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            *self.sim.borrow_mut() = Some(self.fuel_intake_sim(motor));
        }
    }

    /// Java's inner `FuelIntakeSim`.
    fn fuel_intake_sim(&self, motor: &phoenix6::TalonFX) -> RollerSim {
        let extension_sim = context::intake_extension().get_sim();
        RollerSim::new(
            RollerConfig::new(self.config.wheel_diameter)
                .set_position(self.config.intake_x, self.config.intake_y)
                .set_mount_linear(extension_sim.as_ref()),
            &context::left_view(),
            motor,
            &self.config.base.name,
        )
    }
}

impl Subsystem for FuelIntake {
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
        telemetry::log_dash("FuelIntake/WantedState", self.wanted_state.get().name());
        telemetry::log_dash("FuelIntake/SystemState", self.system_state.get().name());
        telemetry::log_dash("FuelIntake/CurrentCommand", self.get_current_command_name());
        telemetry::log_dash_with_unit("FuelIntake/Voltage", self.get_voltage(), "volts");
        telemetry::log_dash_with_unit(
            "FuelIntake/StatorCurrent",
            self.get_stator_current(),
            "amps",
        );
        telemetry::log_dash_with_unit(
            "FuelIntake/SupplyCurrent",
            self.get_supply_current(),
            "amps",
        );
        telemetry::log_dash_with_unit("FuelIntake/RPM", self.get_velocity_rpm(), "RPM");
        telemetry::log_dash_with_unit("FuelIntake/Temp", self.get_temp(), "deg_C");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoenix6::signals::{InvertedValue, NeutralModeValue};

    fn unattached() -> Rc<FuelIntake> {
        let mut config = FuelIntakeConfig::default();
        config.base.attached = false;
        FuelIntake::build(config)
    }

    #[test]
    fn every_wanted_state_becomes_the_system_state_of_the_same_name() {
        let pairs = [
            (WantedState::Neutral, SystemState::Neutral),
            (WantedState::Off, SystemState::Off),
            (WantedState::Intake, SystemState::Intake),
            (WantedState::Outtake, SystemState::Outtake),
            (WantedState::SlowIntake, SystemState::SlowIntake),
        ];
        for (wanted, system) in pairs {
            assert_eq!(handle_state_transition(wanted), system);
            assert_eq!(wanted.name(), system.name());
        }
    }

    #[test]
    fn each_state_picks_its_roller_voltage() {
        assert_eq!(output_for(SystemState::Neutral), Output::Voltage(0.0));
        assert_eq!(output_for(SystemState::Intake), Output::Voltage(12.0));
        assert_eq!(output_for(SystemState::Outtake), Output::Voltage(-12.0));
        assert_eq!(output_for(SystemState::SlowIntake), Output::Voltage(5.0));
        assert_eq!(output_for(SystemState::Off), Output::Stop);
    }

    #[test]
    fn the_intake_starts_neutral() {
        let intake = unattached();
        assert_eq!(intake.get_wanted_state(), WantedState::Neutral);
        assert_eq!(intake.get_system_state(), SystemState::Neutral);
        intake.set_wanted_state(WantedState::Intake);
        assert_eq!(intake.get_wanted_state(), WantedState::Intake);
    }

    #[test]
    fn applying_states_to_an_unattached_intake_commands_nothing() {
        let intake = unattached();
        for state in [
            SystemState::Neutral,
            SystemState::Off,
            SystemState::Intake,
            SystemState::Outtake,
            SystemState::SlowIntake,
        ] {
            intake.system_state.set(state);
            intake.apply_states();
        }
        assert!(intake.get_sim().is_none());
    }

    #[test]
    fn config_holds_the_java_motor_setup() {
        let config = FuelIntakeConfig::default();
        let base = &config.base;
        assert_eq!(base.name, "Intake");
        assert_eq!(base.id.get_device_number(), 5);
        assert_eq!(base.follower_configs.len(), 1);
        assert_eq!(base.follower_configs[0].id.get_device_number(), 6);
        assert_eq!(
            base.follower_configs[0].oppose_leader,
            MotorAlignmentValue::Opposed
        );
        let talon = &base.talon_config;
        assert_eq!(talon.slot0.k_p, 5.0);
        assert_eq!(talon.slot0.k_s, 4.0);
        assert_eq!(talon.current_limits.supply_current_limit, 45.0);
        assert_eq!(talon.current_limits.stator_current_limit, 90.0);
        assert_eq!(talon.motor_output.neutral_mode, NeutralModeValue::Coast);
        assert_eq!(
            talon.motor_output.inverted,
            InvertedValue::CounterClockwise_Positive
        );
    }
}
