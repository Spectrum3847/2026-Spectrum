use crate::error::{Error, Result};
use crate::feedforwards::MAX_MODULES;
use crate::util::deploy_directory;
use serde::Deserialize;
use wpimath::geometry::Translation2d;
use wpimath::kinematics::{ChassisSpeeds, SwerveDriveKinematics, SwerveModuleState};
use wpimath::system::DcMotor;
use wpimath::units::rotations_per_minute_to_radians_per_second as rpm;

/// PID gains for the path following controllers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidConstants {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    /// Integral range.
    pub i_zone: f64,
}

impl PidConstants {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self::with_i_zone(kp, ki, kd, 1.0)
    }

    pub fn with_i_zone(kp: f64, ki: f64, kd: f64, i_zone: f64) -> Self {
        Self { kp, ki, kd, i_zone }
    }
}

/// `DCMotor.getKrakenX60`. wpimath has only the FOC variant.
pub fn kraken_x60(num_motors: i32) -> DcMotor {
    DcMotor::new(12.0, 7.09, 366.0, 2.0, rpm(6000.0), num_motors)
}

/// The motor behind a `driveMotorType` name in the GUI settings.
fn motor_from_settings_name(name: &str, num_motors: i32) -> Result<DcMotor> {
    let motor = match name {
        "krakenX60" => kraken_x60(num_motors),
        "krakenX60FOC" => DcMotor::kraken_x60_foc(num_motors),
        "falcon500" => DcMotor::new(12.0, 4.69, 257.0, 1.5, rpm(6380.0), num_motors),
        "falcon500FOC" => DcMotor::new(12.0, 5.84, 304.0, 1.5, rpm(6080.0), num_motors),
        "vortex" => DcMotor::new(12.0, 3.60, 211.0, 3.6, rpm(6784.0), num_motors),
        "NEO" => DcMotor::new(12.0, 2.6, 105.0, 1.8, rpm(5676.0), num_motors),
        "CIM" => DcMotor::new(12.0, 2.42, 133.0, 2.7, rpm(5310.0), num_motors),
        "miniCIM" => DcMotor::new(12.0, 1.41, 89.0, 3.0, rpm(5840.0), num_motors),
        other => return Err(Error::Invalid(format!("Invalid motor type: {other}"))),
    };
    Ok(motor)
}

/// One drive module: a swerve module, or one side of a differential drive.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModuleConfig {
    pub wheel_radius_meters: f64,
    /// Top speed of the drive motor while driving the robot at full output.
    pub max_drive_velocity_mps: f64,
    /// Friction coefficient between the wheel and the carpet.
    pub wheel_cof: f64,
    /// The drive gearbox, including reduction.
    pub drive_motor: DcMotor,
    pub drive_current_limit: f64,
    pub max_drive_velocity_rad_per_sec: f64,
    /// Motor torque lost while driving, from the torque at top speed under load.
    pub torque_loss: f64,
}

impl ModuleConfig {
    /// `drive_motor` already includes the gear reduction. `num_motors` is 1 for swerve.
    pub fn new(
        wheel_radius_meters: f64,
        max_drive_velocity_mps: f64,
        wheel_cof: f64,
        drive_motor: DcMotor,
        drive_current_limit: f64,
        num_motors: i32,
    ) -> Self {
        let drive_current_limit = drive_current_limit * f64::from(num_motors);
        let max_drive_velocity_rad_per_sec = max_drive_velocity_mps / wheel_radius_meters;
        let max_speed_current_draw = drive_motor.current(max_drive_velocity_rad_per_sec, 12.0);
        let torque_loss = drive_motor
            .torque(max_speed_current_draw.min(drive_current_limit))
            .max(0.0);
        Self {
            wheel_radius_meters,
            max_drive_velocity_mps,
            wheel_cof,
            drive_motor,
            drive_current_limit,
            max_drive_velocity_rad_per_sec,
            torque_loss,
        }
    }

    /// Like [`new`](Self::new) with a motor that does not include the reduction yet.
    pub fn with_gearing(
        wheel_radius_meters: f64,
        max_drive_velocity_mps: f64,
        wheel_cof: f64,
        drive_motor: DcMotor,
        drive_gearing: f64,
        drive_current_limit: f64,
        num_motors: i32,
    ) -> Self {
        Self::new(
            wheel_radius_meters,
            max_drive_velocity_mps,
            wheel_cof,
            drive_motor.with_reduction(drive_gearing),
            drive_current_limit,
            num_motors,
        )
    }
}

#[derive(Clone, Debug)]
enum Drive {
    Swerve(Box<SwerveDriveKinematics<MAX_MODULES>>),
    Differential { trackwidth_meters: f64 },
}

/// Everything trajectory generation needs to know about the robot.
#[derive(Clone, Debug)]
pub struct RobotConfig {
    pub mass_kg: f64,
    /// Moment of inertia in kg m^2.
    pub moi: f64,
    pub module_config: ModuleConfig,
    /// Robot-relative module locations in meters, FL, FR, BL, BR for swerve.
    pub module_locations: Vec<Translation2d>,
    pub is_holonomic: bool,
    pub num_modules: usize,
    /// Distance from the robot center to each module.
    pub module_pivot_distance: Vec<f64>,
    /// Static friction force between one drive wheel and the carpet in newtons.
    pub wheel_friction_force: f64,
    /// Largest torque a module can apply without slipping the wheel.
    pub max_torque_friction: f64,
    drive: Drive,
    // Rows of the (2 * modules) x 3 force kinematics matrix.
    force_kinematics: Vec<[f64; 3]>,
}

impl RobotConfig {
    /// A holonomic swerve robot. `module_offsets` are FL, FR, BL, BR from the robot center.
    pub fn new(
        mass_kg: f64,
        moi: f64,
        module_config: ModuleConfig,
        module_offsets: [Translation2d; 4],
    ) -> Self {
        Self::build(
            mass_kg,
            moi,
            module_config,
            module_offsets.to_vec(),
            Drive::Swerve(Box::new(SwerveDriveKinematics::new(module_offsets))),
        )
    }

    /// A differential drive robot. Only the warmup commands use one.
    pub fn new_differential(
        mass_kg: f64,
        moi: f64,
        module_config: ModuleConfig,
        trackwidth_meters: f64,
    ) -> Self {
        let locations = vec![
            Translation2d::new(0.0, trackwidth_meters / 2.0),
            Translation2d::new(0.0, -trackwidth_meters / 2.0),
        ];
        Self::build(
            mass_kg,
            moi,
            module_config,
            locations,
            Drive::Differential { trackwidth_meters },
        )
    }

    fn build(
        mass_kg: f64,
        moi: f64,
        module_config: ModuleConfig,
        module_locations: Vec<Translation2d>,
        drive: Drive,
    ) -> Self {
        let num_modules = module_locations.len();
        let module_pivot_distance = module_locations.iter().map(|l| l.norm()).collect();
        let wheel_friction_force = module_config.wheel_cof * ((mass_kg / num_modules as f64) * 9.8);
        let max_torque_friction = wheel_friction_force * module_config.wheel_radius_meters;

        let mut force_kinematics = Vec::with_capacity(num_modules * 2);
        for location in &module_locations {
            let reciprocal = Translation2d::from_polar(1.0 / location.norm(), location.angle());
            force_kinematics.push([1.0, 0.0, -reciprocal.y]);
            force_kinematics.push([0.0, 1.0, reciprocal.x]);
        }

        Self {
            mass_kg,
            moi,
            module_config,
            is_holonomic: matches!(drive, Drive::Swerve(_)),
            num_modules,
            module_pivot_distance,
            wheel_friction_force,
            max_torque_friction,
            drive,
            force_kinematics,
            module_locations,
        }
    }

    /// Module speeds for robot-relative chassis speeds, in module order. Only the first
    /// `num_modules` entries are used. Module angles are not needed by the callers.
    pub fn module_speeds(&self, speeds: ChassisSpeeds) -> [f64; MAX_MODULES] {
        let mut out = [0.0; MAX_MODULES];
        match &self.drive {
            Drive::Swerve(_) => {
                for (speed, m) in out.iter_mut().zip(&self.module_locations) {
                    let x = speeds.vx + -m.y * speeds.omega;
                    let y = speeds.vy + m.x * speeds.omega;
                    *speed = x.hypot(y);
                }
            }
            Drive::Differential { trackwidth_meters } => {
                out[0] = speeds.vx - trackwidth_meters / 2.0 * speeds.omega;
                out[1] = speeds.vx + trackwidth_meters / 2.0 * speeds.omega;
            }
        }
        out
    }

    /// Robot-relative chassis speeds for module states, one per module. Differential drive reads
    /// the speeds only.
    pub fn to_chassis_speeds(&self, states: &[SwerveModuleState]) -> ChassisSpeeds {
        match &self.drive {
            Drive::Swerve(kinematics) => {
                let states: &[SwerveModuleState; MAX_MODULES] = states
                    .try_into()
                    .expect("swerve config has exactly 4 modules");
                kinematics.to_chassis_speeds(states)
            }
            Drive::Differential { trackwidth_meters } => ChassisSpeeds::new(
                (states[0].speed + states[1].speed) / 2.0,
                0.0,
                (states[1].speed - states[0].speed) / trackwidth_meters,
            ),
        }
    }

    /// Splits chassis forces (passed as `ChassisSpeeds`) into one force vector per module.
    pub fn chassis_forces_to_wheel_force_vectors(
        &self,
        chassis_forces: ChassisSpeeds,
    ) -> [Translation2d; MAX_MODULES] {
        let n = self.num_modules as f64;
        let vector = [
            chassis_forces.vx / n,
            chassis_forces.vy / n,
            chassis_forces.omega / n,
        ];
        let mut out = [Translation2d::ZERO; MAX_MODULES];
        for (m, force) in out.iter_mut().enumerate().take(self.num_modules) {
            let row = |r: usize| {
                let mut total = 0.0;
                for (k, v) in vector.iter().enumerate() {
                    total += self.force_kinematics[r][k] * v;
                }
                total
            };
            *force = Translation2d::new(row(m * 2), row(m * 2 + 1));
        }
        out
    }

    /// Loads the robot config from the settings file the GUI shares with the robot,
    /// `pathplanner/settings.json` in the deploy folder.
    pub fn from_gui_settings() -> Result<Self> {
        let path = deploy_directory().join("pathplanner/settings.json");
        let text = std::fs::read_to_string(path)?;
        Self::from_gui_settings_json(&text)
    }

    /// Like [`from_gui_settings`](Self::from_gui_settings) with the settings text in hand.
    pub fn from_gui_settings_json(text: &str) -> Result<Self> {
        let s: SettingsJson = serde_json::from_str(text)?;
        let num_motors = if s.holonomic_mode { 1 } else { 2 };
        let gearbox = motor_from_settings_name(&s.drive_motor_type, num_motors)?
            .with_reduction(s.drive_gearing);
        let module_config = ModuleConfig::new(
            s.drive_wheel_radius,
            s.max_drive_speed,
            s.wheel_cof,
            gearbox,
            s.drive_current_limit,
            num_motors,
        );
        if s.holonomic_mode {
            let offsets = [
                Translation2d::new(s.fl_module_x, s.fl_module_y),
                Translation2d::new(s.fr_module_x, s.fr_module_y),
                Translation2d::new(s.bl_module_x, s.bl_module_y),
                Translation2d::new(s.br_module_x, s.br_module_y),
            ];
            Ok(Self::new(s.robot_mass, s.robot_moi, module_config, offsets))
        } else {
            Ok(Self::new_differential(
                s.robot_mass,
                s.robot_moi,
                module_config,
                s.robot_trackwidth,
            ))
        }
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct SettingsJson {
    holonomic_mode: bool,
    robot_mass: f64,
    #[serde(rename = "robotMOI")]
    robot_moi: f64,
    drive_wheel_radius: f64,
    drive_gearing: f64,
    max_drive_speed: f64,
    #[serde(rename = "wheelCOF")]
    wheel_cof: f64,
    drive_motor_type: String,
    drive_current_limit: f64,
    // Swerve and differential settings are each only required in their own mode.
    #[serde(default)]
    robot_trackwidth: f64,
    #[serde(default)]
    fl_module_x: f64,
    #[serde(default)]
    fl_module_y: f64,
    #[serde(default)]
    fr_module_x: f64,
    #[serde(default)]
    fr_module_y: f64,
    #[serde(default)]
    bl_module_x: f64,
    #[serde(default)]
    bl_module_y: f64,
    #[serde(default)]
    br_module_x: f64,
    #[serde(default)]
    br_module_y: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::util::robot_config;

    #[test]
    fn pid_constants_default_the_integral_range() {
        assert_eq!(PidConstants::new(1.0, 2.0, 3.0).i_zone, 1.0);
        assert_eq!(PidConstants::with_i_zone(1.0, 2.0, 3.0, 0.5).i_zone, 0.5);
    }

    #[test]
    fn the_robot_settings_make_a_swerve_config() {
        let c = robot_config();
        assert!(c.is_holonomic);
        assert_eq!(c.num_modules, 4);
        assert_eq!(c.mass_kg, 60.0);
        assert_eq!(c.module_locations[1], Translation2d::new(0.275, -0.275));
    }

    #[test]
    fn an_unknown_motor_type_is_an_error() {
        let text = r#"{"holonomicMode": true, "robotMass": 60, "robotMOI": 7,
            "driveWheelRadius": 0.05, "driveGearing": 6, "maxDriveSpeed": 4.5, "wheelCOF": 2,
            "driveMotorType": "steam", "driveCurrentLimit": 80}"#;
        let Err(Error::Invalid(message)) = RobotConfig::from_gui_settings_json(text) else {
            panic!("expected an error");
        };
        assert_eq!(message, "Invalid motor type: steam");
    }

    #[test]
    fn settings_with_a_missing_value_are_an_error() {
        assert!(matches!(
            RobotConfig::from_gui_settings_json(r#"{"holonomicMode": true}"#),
            Err(Error::Parse(_))
        ));
    }

    #[test]
    fn module_speeds_of_a_spinning_robot_are_equal() {
        let c = robot_config();
        let speeds = c.module_speeds(ChassisSpeeds::new(0.0, 0.0, 2.0));
        let expected = 2.0 * (0.275_f64 * 2.0_f64.sqrt());
        assert!(speeds.iter().all(|s| (s - expected).abs() < 1e-12));
    }

    #[test]
    fn a_differential_config_has_two_modules() {
        let c = RobotConfig::new_differential(
            50.0,
            5.0,
            ModuleConfig::new(0.05, 4.0, 1.0, kraken_x60(1).with_reduction(6.0), 60.0, 1),
            0.6,
        );
        assert!(!c.is_holonomic);
        assert_eq!(c.num_modules, 2);
        assert_eq!(c.module_pivot_distance, [0.3, 0.3]);
        let speeds = c.module_speeds(ChassisSpeeds::new(1.0, 0.0, 1.0));
        assert_eq!(&speeds[..2], [0.7, 1.3]);
        let back = c.to_chassis_speeds(&[
            SwerveModuleState::new(0.7, Default::default()),
            SwerveModuleState::new(1.3, Default::default()),
        ]);
        assert!((back.vx - 1.0).abs() < 1e-12 && (back.omega - 1.0).abs() < 1e-12);
    }

    #[test]
    fn module_configs_with_gearing_match_the_reduced_motor() {
        let a = ModuleConfig::with_gearing(0.05, 4.0, 1.0, kraken_x60(1), 6.0, 60.0, 1);
        let b = ModuleConfig::new(0.05, 4.0, 1.0, kraken_x60(1).with_reduction(6.0), 60.0, 1);
        assert_eq!(a, b);
    }
}
