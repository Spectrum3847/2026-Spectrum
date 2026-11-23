//! `SwerveDrivetrainConstants`, `SwerveModuleConstants` and `SwerveModuleConstantsFactory`.
//!
//! Only the Talon FX drive and steer motors and the CANcoder that the robot uses. Lengths are
//! meters, angles rotations (the Phoenix boundary), currents amps.

use phoenix6::configs::{
    CANcoderConfiguration, Pigeon2Configuration, Slot0Configs, TalonFXConfiguration,
};

/// How a closed-loop module request drives the motor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClosedLoopOutputType {
    Voltage,
    TorqueCurrentFoc,
}

/// How the steer motor reads the CANcoder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SteerFeedbackType {
    FusedCANcoder,
    SyncCANcoder,
    RemoteCANcoder,
}

#[derive(Clone, Debug, Default)]
pub struct DrivetrainConstants {
    pub can_bus_name: String,
    pub pigeon2_id: i32,
    /// `None` leaves the Pigeon 2 configuration alone.
    pub pigeon2_configs: Option<Pigeon2Configuration>,
}

impl DrivetrainConstants {
    #[must_use]
    pub fn with_can_bus_name(mut self, name: impl Into<String>) -> Self {
        self.can_bus_name = name.into();
        self
    }

    #[must_use]
    pub fn with_pigeon2_id(mut self, id: i32) -> Self {
        self.pigeon2_id = id;
        self
    }

    #[must_use]
    pub fn with_pigeon2_configs(mut self, configs: Pigeon2Configuration) -> Self {
        self.pigeon2_configs = Some(configs);
        self
    }
}

/// Torque model of the drive motor, for wheel force feedforwards. CTRE reads these from the
/// device's `MotorKT` and `MotorStallCurrent` signals. A module uses this model only until the
/// Talon has reported them. The default is the Kraken X60 with FOC.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveMotorModel {
    /// Torque per amp, N*m/A.
    pub kt: f64,
    pub stall_current_a: f64,
}

impl Default for DriveMotorModel {
    fn default() -> Self {
        Self {
            kt: 9.37 / 483.0,
            stall_current_a: 483.0,
        }
    }
}

/// Everything one module needs. Defaults are CTRE's.
#[derive(Clone, Debug)]
pub struct ModuleConstants {
    pub steer_motor_id: i32,
    pub drive_motor_id: i32,
    pub encoder_id: i32,
    pub encoder_offset_rot: f64,
    pub location_x_m: f64,
    pub location_y_m: f64,
    pub drive_motor_inverted: bool,
    pub steer_motor_inverted: bool,
    pub encoder_inverted: bool,
    /// Motor rotations per wheel rotation.
    pub drive_motor_gear_ratio: f64,
    /// Motor rotations per azimuth rotation.
    pub steer_motor_gear_ratio: f64,
    /// Drive motor rotations caused by one azimuth rotation.
    pub coupling_gear_ratio: f64,
    pub wheel_radius_m: f64,
    pub steer_motor_gains: Slot0Configs,
    pub drive_motor_gains: Slot0Configs,
    pub steer_motor_closed_loop_output: ClosedLoopOutputType,
    pub drive_motor_closed_loop_output: ClosedLoopOutputType,
    /// Stator current at which the wheels slip.
    pub slip_current_a: f64,
    /// Free speed at 12 V. Scales open-loop drive and bounds wheel speed desaturation.
    pub speed_at_12_volts_mps: f64,
    pub feedback_source: SteerFeedbackType,
    pub drive_motor_initial_configs: Option<TalonFXConfiguration>,
    pub steer_motor_initial_configs: Option<TalonFXConfiguration>,
    pub encoder_initial_configs: Option<CANcoderConfiguration>,
    /// Simulation only: kg*m^2.
    pub steer_inertia: f64,
    pub drive_inertia: f64,
    /// Simulation only: volts needed to overcome friction.
    pub steer_friction_voltage: f64,
    pub drive_friction_voltage: f64,
    pub drive_motor_model: DriveMotorModel,
}

impl Default for ModuleConstants {
    fn default() -> Self {
        Self {
            steer_motor_id: 0,
            drive_motor_id: 0,
            encoder_id: 0,
            encoder_offset_rot: 0.0,
            location_x_m: 0.0,
            location_y_m: 0.0,
            drive_motor_inverted: false,
            steer_motor_inverted: false,
            encoder_inverted: false,
            drive_motor_gear_ratio: 0.0,
            steer_motor_gear_ratio: 0.0,
            coupling_gear_ratio: 0.0,
            wheel_radius_m: 0.0,
            steer_motor_gains: Slot0Configs::default(),
            drive_motor_gains: Slot0Configs::default(),
            steer_motor_closed_loop_output: ClosedLoopOutputType::Voltage,
            drive_motor_closed_loop_output: ClosedLoopOutputType::Voltage,
            slip_current_a: 120.0,
            speed_at_12_volts_mps: 0.0,
            feedback_source: SteerFeedbackType::FusedCANcoder,
            drive_motor_initial_configs: None,
            steer_motor_initial_configs: None,
            encoder_initial_configs: None,
            steer_inertia: 0.01,
            drive_inertia: 0.035,
            steer_friction_voltage: 0.2,
            drive_friction_voltage: 0.2,
            drive_motor_model: DriveMotorModel::default(),
        }
    }
}

impl ModuleConstants {
    /// Drive motor rotations per meter of travel.
    pub fn drive_rotations_per_meter(&self) -> f64 {
        self.drive_motor_gear_ratio / (2.0 * std::f64::consts::PI * self.wheel_radius_m)
    }
}

/// Holds the constants shared by every module (`base`) and stamps out one per module.
#[derive(Clone, Debug, Default)]
pub struct ModuleConstantsFactory {
    pub base: ModuleConstants,
}

impl ModuleConstantsFactory {
    #[allow(clippy::too_many_arguments)]
    pub fn create_module_constants(
        &self,
        steer_motor_id: i32,
        drive_motor_id: i32,
        encoder_id: i32,
        encoder_offset_rot: f64,
        location_x_m: f64,
        location_y_m: f64,
        drive_motor_inverted: bool,
        steer_motor_inverted: bool,
        encoder_inverted: bool,
    ) -> ModuleConstants {
        ModuleConstants {
            steer_motor_id,
            drive_motor_id,
            encoder_id,
            encoder_offset_rot,
            location_x_m,
            location_y_m,
            drive_motor_inverted,
            steer_motor_inverted,
            encoder_inverted,
            ..self.base.clone()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_ctre() {
        let c = ModuleConstants::default();
        assert_eq!(c.slip_current_a, 120.0);
        assert_eq!(c.steer_inertia, 0.01);
        assert_eq!(c.drive_inertia, 0.035);
        assert_eq!(c.steer_friction_voltage, 0.2);
        assert_eq!(c.feedback_source, SteerFeedbackType::FusedCANcoder);
        assert_eq!(
            c.drive_motor_closed_loop_output,
            ClosedLoopOutputType::Voltage
        );
    }

    #[test]
    fn factory_copies_shared_fields_and_sets_per_module_ones() {
        let factory = ModuleConstantsFactory {
            base: ModuleConstants {
                drive_motor_gear_ratio: 6.03,
                coupling_gear_ratio: 4.5,
                ..ModuleConstants::default()
            },
        };
        let m = factory.create_module_constants(2, 1, 3, -0.5, 0.276, -0.276, true, false, false);
        assert_eq!(
            (m.steer_motor_id, m.drive_motor_id, m.encoder_id),
            (2, 1, 3)
        );
        assert_eq!(m.encoder_offset_rot, -0.5);
        assert_eq!(m.location_y_m, -0.276);
        assert!(m.drive_motor_inverted);
        assert_eq!(m.drive_motor_gear_ratio, 6.03);
        assert_eq!(m.coupling_gear_ratio, 4.5);
    }

    #[test]
    fn rotations_per_meter() {
        let c = ModuleConstants {
            drive_motor_gear_ratio: 2.0 * std::f64::consts::PI,
            wheel_radius_m: 1.0,
            ..ModuleConstants::default()
        };
        assert!((c.drive_rotations_per_meter() - 1.0).abs() < 1e-12);
    }
}
