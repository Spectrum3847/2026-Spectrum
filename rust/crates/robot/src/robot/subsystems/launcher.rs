//! Four-motor flywheel that launches fuel at the hub.

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
pub struct LauncherConfig {
    pub base: Config,

    pub supply_current_limit: f64,
    pub stator_current_limit: f64,
    pub lower_supply_current_limit: f64,
    pub lower_supply_current_time: f64,
    pub forward_torque_current_limit: f64,
    pub reverse_torque_current_limit: f64,
    pub voltage_limit: f64,
    pub velocity_kp: f64,
    pub velocity_kv: f64,
    pub velocity_ks: f64,

    pub on_target_tolerance_rpm: f64,

    pub launcher_x: f64,
    pub launcher_y: f64,
    pub wheel_diameter: f64,
}

impl Default for LauncherConfig {
    fn default() -> Self {
        let supply_current_limit = 80.0;
        let stator_current_limit = 180.0;
        let lower_supply_current_limit = 80.0;
        let lower_supply_current_time = 0.0;
        let forward_torque_current_limit = stator_current_limit;
        let reverse_torque_current_limit = 10.0;
        let voltage_limit = 12.0;
        let velocity_kp = 10.0;
        let velocity_kv = 0.0;
        let velocity_ks = 20.0;

        let mut base = Config::new("Launcher", 46, Rio::CANIVORE);
        base.config_pid_gains_slot(0, velocity_kp, 0.0, 0.0);
        base.config_feed_forward_gains(velocity_ks, velocity_kv, 0.0, 0.0);
        base.config_gear_ratio(1.0);
        base.config_lower_supply_current_limit(lower_supply_current_limit);
        base.config_lower_supply_current_time(lower_supply_current_time);
        base.config_supply_current_limit(supply_current_limit, true);
        base.config_stator_current_limit(stator_current_limit, true);
        base.config_forward_torque_current_limit(forward_torque_current_limit);
        base.config_reverse_torque_current_limit(reverse_torque_current_limit);
        base.config_neutral_brake_mode(false);
        base.config_forward_voltage_limit(voltage_limit);
        base.config_reverse_voltage_limit(-voltage_limit);
        base.config_clockwise_positive();
        base.set_follower_configs(vec![
            FollowerConfig::new(
                "Launcher Top Right",
                47,
                Rio::CANIVORE,
                MotorAlignmentValue::Opposed,
            ),
            FollowerConfig::new(
                "Launcher Bottom Left",
                48,
                Rio::CANIVORE,
                MotorAlignmentValue::Aligned,
            ),
            FollowerConfig::new(
                "Launcher Bottom Right",
                49,
                Rio::CANIVORE,
                MotorAlignmentValue::Opposed,
            ),
        ]);

        Self {
            base,
            supply_current_limit,
            stator_current_limit,
            lower_supply_current_limit,
            lower_supply_current_time,
            forward_torque_current_limit,
            reverse_torque_current_limit,
            voltage_limit,
            velocity_kp,
            velocity_kv,
            velocity_ks,
            on_target_tolerance_rpm: 100.0,
            launcher_x: inches_to_meters(62.5),
            launcher_y: inches_to_meters(60.0),
            wheel_diameter: 4.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WantedState {
    Off,
    IdlePrep,
    SlowLaunch,
    AimAtTarget,
}

impl EnumName for WantedState {
    fn name(&self) -> &'static str {
        match self {
            Self::Off => "OFF",
            Self::IdlePrep => "IDLE_PREP",
            Self::SlowLaunch => "SLOW_LAUNCH",
            Self::AimAtTarget => "AIM_AT_TARGET",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Off,
    IdlePrep,
    SlowLaunch,
    AimAtTarget,
}

impl EnumName for SystemState {
    fn name(&self) -> &'static str {
        match self {
            Self::Off => "OFF",
            Self::IdlePrep => "IDLE_PREP",
            Self::SlowLaunch => "SLOW_LAUNCH",
            Self::AimAtTarget => "AIM_AT_TARGET",
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
        WantedState::IdlePrep => SystemState::IdlePrep,
        WantedState::SlowLaunch => SystemState::SlowLaunch,
        WantedState::AimAtTarget => SystemState::AimAtTarget,
    }
}

/// `aim_rpm` is only called in `AimAtTarget`, where Java asks the shot calculator.
pub fn output_for(system_state: SystemState, aim_rpm: impl FnOnce() -> f64) -> Output {
    match system_state {
        SystemState::Off => Output::Stop,
        SystemState::IdlePrep => Output::VelocityTcFocRpm(700.0),
        SystemState::SlowLaunch => Output::VelocityTcFocRpm(400.0),
        SystemState::AimAtTarget => Output::VelocityTcFocRpm(aim_rpm()),
    }
}

pub struct Launcher {
    mechanism: Rc<Mechanism>,
    wanted_state: Cell<WantedState>,
    system_state: Cell<SystemState>,
    pub config: LauncherConfig,
    sim: RefCell<Option<RollerSim>>,
}

impl Deref for Launcher {
    type Target = Mechanism;

    fn deref(&self) -> &Mechanism {
        &self.mechanism
    }
}

impl Launcher {
    pub fn new(config: LauncherConfig) -> Rc<Launcher> {
        let launcher = Self::build(config);
        launcher.simulation_init();
        telemetry::print(&format!("{} Subsystem Initialized", launcher.get_name()));
        launcher
    }

    fn build(config: LauncherConfig) -> Rc<Launcher> {
        let launcher = Rc::new(Launcher {
            mechanism: Mechanism::new(config.base.clone()),
            wanted_state: Cell::new(WantedState::Off),
            system_state: Cell::new(SystemState::Off),
            config,
            sim: RefCell::new(None),
        });
        CommandScheduler::get_instance().register_subsystem(launcher.clone());
        launcher
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
        let output = output_for(self.system_state.get(), || {
            context::shot_calculator().get_parameters().flywheel_speed
        });
        match output {
            Output::Stop => self.stop(),
            Output::VelocityTcFocRpm(wanted_rpm) => self.set_velocity_tcfoc_rpm(move || wanted_rpm),
        }
    }

    pub fn simulation_init(&self) {
        if self.is_attached()
            && let Some(motor) = self.get_motor()
        {
            *self.sim.borrow_mut() = Some(self.launcher_sim(motor));
        }
    }

    /// Java's inner `LauncherSim`.
    fn launcher_sim(&self, motor: &phoenix6::TalonFX) -> RollerSim {
        let hood_sim = context::hood().get_sim();
        RollerSim::new(
            RollerConfig::new(self.config.wheel_diameter)
                .set_position(self.config.launcher_x, self.config.launcher_y)
                .set_mount_arm(hood_sim.as_ref()),
            &context::left_view(),
            motor,
            &self.config.base.name,
        )
    }
}

impl Subsystem for Launcher {
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
        telemetry::log_dash("Launcher/WantedState", self.wanted_state.get().name());
        telemetry::log_dash("Launcher/SystemState", self.system_state.get().name());
        telemetry::log_dash("Launcher/CurrentCommand", self.get_current_command_name());
        telemetry::log_dash_with_unit("Launcher/Voltage", self.get_voltage(), "volts");
        telemetry::log_dash_with_unit("Launcher/StatorCurrent", self.get_stator_current(), "amps");
        telemetry::log_dash_with_unit("Launcher/SupplyCurrent", self.get_supply_current(), "amps");
        telemetry::log_dash_with_unit("Launcher/RPM", self.get_velocity_rpm(), "RPM");
        telemetry::log_dash_with_unit("Launcher/Temp", self.get_temp(), "deg_C");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoenix6::signals::{InvertedValue, NeutralModeValue};

    fn unattached() -> Rc<Launcher> {
        let mut config = LauncherConfig::default();
        config.base.attached = false;
        Launcher::build(config)
    }

    #[test]
    fn every_wanted_state_becomes_the_system_state_of_the_same_name() {
        let pairs = [
            (WantedState::Off, SystemState::Off),
            (WantedState::IdlePrep, SystemState::IdlePrep),
            (WantedState::SlowLaunch, SystemState::SlowLaunch),
            (WantedState::AimAtTarget, SystemState::AimAtTarget),
        ];
        for (wanted, system) in pairs {
            assert_eq!(handle_state_transition(wanted), system);
            assert_eq!(wanted.name(), system.name());
        }
    }

    #[test]
    fn each_state_picks_its_flywheel_output() {
        let never = || panic!("only AIM_AT_TARGET asks the shot calculator");
        assert_eq!(output_for(SystemState::Off, never), Output::Stop);
        assert_eq!(
            output_for(SystemState::IdlePrep, never),
            Output::VelocityTcFocRpm(700.0)
        );
        assert_eq!(
            output_for(SystemState::SlowLaunch, never),
            Output::VelocityTcFocRpm(400.0)
        );
        assert_eq!(
            output_for(SystemState::AimAtTarget, || 3210.5),
            Output::VelocityTcFocRpm(3210.5)
        );
    }

    #[test]
    fn the_launcher_starts_off() {
        let launcher = unattached();
        assert_eq!(launcher.get_wanted_state(), WantedState::Off);
        assert_eq!(launcher.get_system_state(), SystemState::Off);
        launcher.set_wanted_state(WantedState::SlowLaunch);
        assert_eq!(launcher.get_wanted_state(), WantedState::SlowLaunch);
    }

    #[test]
    fn applying_states_to_an_unattached_launcher_commands_nothing() {
        let launcher = unattached();
        for state in [
            SystemState::Off,
            SystemState::IdlePrep,
            SystemState::SlowLaunch,
        ] {
            launcher.system_state.set(state);
            launcher.apply_states();
        }
        assert!(launcher.get_sim().is_none());
    }

    #[test]
    fn config_holds_the_java_motor_setup() {
        let config = LauncherConfig::default();
        let base = &config.base;
        assert_eq!(base.name, "Launcher");
        assert_eq!(base.id.get_device_number(), 46);
        assert_eq!(base.follower_configs.len(), 3);
        let follower_ids: Vec<i32> = base
            .follower_configs
            .iter()
            .map(|f| f.id.get_device_number())
            .collect();
        assert_eq!(follower_ids, vec![47, 48, 49]);
        let alignments: Vec<MotorAlignmentValue> = base
            .follower_configs
            .iter()
            .map(|f| f.oppose_leader)
            .collect();
        assert_eq!(
            alignments,
            vec![
                MotorAlignmentValue::Opposed,
                MotorAlignmentValue::Aligned,
                MotorAlignmentValue::Opposed
            ]
        );
        let talon = &base.talon_config;
        assert_eq!(talon.slot0.k_p, 10.0);
        assert_eq!(talon.slot0.k_s, 20.0);
        assert_eq!(talon.current_limits.supply_current_limit, 80.0);
        assert_eq!(talon.current_limits.stator_current_limit, 180.0);
        assert_eq!(talon.torque_current.peak_forward_torque_current, 180.0);
        assert_eq!(talon.torque_current.peak_reverse_torque_current, -10.0);
        assert_eq!(talon.voltage.peak_forward_voltage, 12.0);
        assert_eq!(talon.voltage.peak_reverse_voltage, -12.0);
        assert_eq!(
            talon.motor_output.inverted,
            InvertedValue::Clockwise_Positive
        );
        assert_eq!(talon.motor_output.neutral_mode, NeutralModeValue::Coast);
        assert_eq!(talon.feedback.sensor_to_mechanism_ratio, 1.0);
    }
}
