use crate::spectrum_lib::hardware::rio::Rio;
use phoenix6::configs::{
    CANcoderConfiguration, CurrentLimitsConfigs, Pigeon2Configuration, Slot0Configs,
    TalonFXConfiguration,
};
use phoenix6::signals::StaticFeedforwardSignValue;
use swerve::{
    ClosedLoopOutputType, DrivetrainConstants, ModuleConstants, ModuleConstantsFactory,
    SteerFeedbackType,
};
use wpimath::geometry::Rotation2d;
use wpimath::units::inches_to_meters;

/// Module order is front left, front right, back left, back right.
pub const NUM_MODULES: usize = 4;

/// Everything the swerve drivetrain needs. Lombok getters and setters on the Java class are
/// public fields. Change a field, then call [`SwerveConfig::update_config`] to rebuild the
/// drivetrain and module constants from it.
///
/// Units are Java's: inches, rotations, amps, volts, meters per second. Angular speed is in
/// degrees per second.
#[derive(Clone, Debug)]
pub struct SwerveConfig {
    pub sim_loop_period: f64, // 5 ms

    pub deadband: f64,     // 5% joystick deadband
    pub aim_deadband: f64, // 1% deadband for aiming modes

    pub drive_gear_ratio: f64,
    pub steer_gear_ratio: f64,

    // Fudged from the physical radius so odometry matches a measured run
    pub wheel_radius_inches: f64, // 0.0499 m

    // Theoretical free speed at 12 V
    pub linear_speed_at_12_volts_mps: f64,

    // Theoretical rotational free speed at 12 V
    pub angular_speed_at_12_volts_dps: f64,

    pub k_p_rotation_controller: f64,
    pub k_i_rotation_controller: f64,
    pub k_d_rotation_controller: f64,

    // Blue alliance forward is 0 degrees, toward the red wall
    pub blue_alliance_perspective_rotation: Rotation2d,
    // Red alliance forward is 180 degrees, toward the blue wall
    pub red_alliance_perspective_rotation: Rotation2d,

    pub steer_gains: Slot0Configs,
    pub drive_gains: Slot0Configs,

    pub steer_closed_loop_output: ClosedLoopOutputType,
    pub drive_closed_loop_output: ClosedLoopOutputType,

    // Stator current at which the drive wheels start to slip
    pub slip_current_amps: f64,

    // Cannot be null. Check the with*InitialConfigs() API before changing these.
    pub drive_initial_configs: TalonFXConfiguration,

    // Steer needs little torque, so a low limit avoids brownouts without hurting performance
    pub steer_initial_configs: TalonFXConfiguration,

    pub can_coder_initial_configs: CANcoderConfiguration,

    // `None` skips applying Pigeon 2 configs
    pub pigeon_configs: Option<Pigeon2Configuration>,

    // Drive motor turns per azimuth rotation
    pub couple_ratio: f64,

    pub steer_motor_reversed: bool,
    pub invert_left_side: bool,
    pub invert_right_side: bool,

    pub can_bus_name: String,
    /// Java builds this bus with a hoot file path, which loads that file for replay.
    pub hoot_file_path: String,
    pub pigeon_id: i32,

    // These are only used for simulation
    pub steer_inertia: f64,
    pub drive_inertia: f64,
    // Simulated voltage necessary to overcome friction
    pub steer_friction_voltage: f64,
    pub drive_friction_voltage: f64,

    pub drivetrain_constants: DrivetrainConstants,

    pub constant_creator: ModuleConstantsFactory,

    // Center to module distance. MapleSim calls it a radius, but it is the diagonal.
    pub drivebase_radius_inches: f64,

    pub drivebase_radius_meters: f64,

    pub front_left_drive_motor_id: i32,
    pub front_left_steer_motor_id: i32,
    pub front_left_encoder_id: i32,
    pub front_left_encoder_offset_rot: f64,
    pub front_left_steer_inverted: bool,

    pub front_left_x_pos_inches: f64,
    pub front_left_y_pos_inches: f64,

    pub front_right_drive_motor_id: i32,
    pub front_right_steer_motor_id: i32,
    pub front_right_encoder_id: i32,
    pub front_right_encoder_offset_rot: f64,
    pub front_right_steer_inverted: bool,

    pub front_right_x_pos_inches: f64,
    pub front_right_y_pos_inches: f64,

    pub back_left_drive_motor_id: i32,
    pub back_left_steer_motor_id: i32,
    pub back_left_encoder_id: i32,
    pub back_left_encoder_offset_rot: f64,
    pub back_left_steer_inverted: bool,

    pub back_left_x_pos_inches: f64,
    pub back_left_y_pos_inches: f64,

    pub back_right_drive_motor_id: i32,
    pub back_right_steer_motor_id: i32,
    pub back_right_encoder_id: i32,
    pub back_right_encoder_offset_rot: f64,
    pub back_right_steer_inverted: bool,

    pub back_right_x_pos_inches: f64,
    pub back_right_y_pos_inches: f64,

    pub target_heading: f64,

    pub front_left: ModuleConstants,
    pub front_right: ModuleConstants,
    pub back_left: ModuleConstants,
    pub back_right: ModuleConstants,
}

impl Default for SwerveConfig {
    fn default() -> Self {
        let wheel_base_inches = 21.75;
        let track_width_inches = 21.75;
        let drivebase_radius_inches = (wheel_base_inches / 2.0_f64).hypot(track_width_inches / 2.0);

        let mut config = Self {
            sim_loop_period: 0.005,
            deadband: 0.05,
            aim_deadband: 0.01,
            drive_gear_ratio: 6.03,
            steer_gear_ratio: 26.09,
            wheel_radius_inches: 1.964,
            linear_speed_at_12_volts_mps: 5.12,
            angular_speed_at_12_volts_dps: 540.00,
            k_p_rotation_controller: 5.0,
            k_i_rotation_controller: 0.0,
            k_d_rotation_controller: 0.0,
            blue_alliance_perspective_rotation: Rotation2d::ZERO,
            red_alliance_perspective_rotation: Rotation2d::PI,
            steer_gains: Slot0Configs::default()
                .with_k_p(500.0)
                .with_k_i(0.0)
                .with_k_d(20.0)
                .with_k_s(0.15)
                .with_k_v(1.0)
                .with_k_a(0.0)
                .with_static_feedforward_sign(StaticFeedforwardSignValue::UseClosedLoopSign),
            drive_gains: Slot0Configs::default()
                .with_k_p(10.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(4.0)
                .with_k_v(0.0),
            steer_closed_loop_output: ClosedLoopOutputType::TorqueCurrentFoc,
            drive_closed_loop_output: ClosedLoopOutputType::TorqueCurrentFoc,
            slip_current_amps: 80.0,
            drive_initial_configs: TalonFXConfiguration::default().with_current_limits(
                CurrentLimitsConfigs::default()
                    .with_stator_current_limit(80.0)
                    .with_stator_current_limit_enable(true)
                    .with_supply_current_limit(30.0)
                    .with_supply_current_limit_enable(true)
                    .with_supply_current_lower_limit(30.0),
            ),
            steer_initial_configs: TalonFXConfiguration::default().with_current_limits(
                CurrentLimitsConfigs::default()
                    .with_stator_current_limit(60.0)
                    .with_stator_current_limit_enable(true),
            ),
            can_coder_initial_configs: CANcoderConfiguration::default(),
            pigeon_configs: Some(Pigeon2Configuration::default()),
            couple_ratio: 4.5,
            steer_motor_reversed: false,
            invert_left_side: false,
            invert_right_side: true,
            can_bus_name: Rio::CANIVORE.to_owned(),
            hoot_file_path: "./logs/spectrum.hoot".to_owned(),
            pigeon_id: 0,
            steer_inertia: 0.01,
            drive_inertia: 0.01,
            steer_friction_voltage: 0.25,
            drive_friction_voltage: 0.25,
            drivetrain_constants: DrivetrainConstants::default(),
            constant_creator: ModuleConstantsFactory::default(),
            drivebase_radius_inches,
            drivebase_radius_meters: inches_to_meters(drivebase_radius_inches),
            front_left_drive_motor_id: 1,
            front_left_steer_motor_id: 2,
            front_left_encoder_id: 3,
            front_left_encoder_offset_rot: -0.83544921875,
            front_left_steer_inverted: false,
            front_left_x_pos_inches: wheel_base_inches / 2.0,
            front_left_y_pos_inches: track_width_inches / 2.0,
            front_right_drive_motor_id: 11,
            front_right_steer_motor_id: 12,
            front_right_encoder_id: 13,
            front_right_encoder_offset_rot: -0.15234375,
            front_right_steer_inverted: false,
            front_right_x_pos_inches: wheel_base_inches / 2.0,
            front_right_y_pos_inches: -track_width_inches / 2.0,
            back_left_drive_motor_id: 21,
            back_left_steer_motor_id: 22,
            back_left_encoder_id: 23,
            back_left_encoder_offset_rot: -0.4794921875,
            back_left_steer_inverted: false,
            back_left_x_pos_inches: -wheel_base_inches / 2.0,
            back_left_y_pos_inches: track_width_inches / 2.0,
            back_right_drive_motor_id: 31,
            back_right_steer_motor_id: 32,
            back_right_encoder_id: 33,
            back_right_encoder_offset_rot: -0.84130859375,
            back_right_steer_inverted: false,
            back_right_x_pos_inches: -wheel_base_inches / 2.0,
            back_right_y_pos_inches: -track_width_inches / 2.0,
            target_heading: 0.0,
            front_left: ModuleConstants::default(),
            front_right: ModuleConstants::default(),
            back_left: ModuleConstants::default(),
            back_right: ModuleConstants::default(),
        };
        config.update_config();
        config
    }
}

impl SwerveConfig {
    /// The four module constants in front left, front right, back left, back right order.
    /// Java returns the same objects every call, so edits to them stick. These are copies.
    pub fn get_modules(&self) -> [ModuleConstants; NUM_MODULES] {
        [
            self.front_left.clone(),
            self.front_right.clone(),
            self.back_left.clone(),
            self.back_right.clone(),
        ]
    }

    /// Rebuilds `drivetrain_constants`, `constant_creator` and the four modules from the
    /// current field values.
    pub fn update_config(&mut self) -> &mut Self {
        let mut drivetrain_constants = DrivetrainConstants::default()
            .with_can_bus_name(self.can_bus_name.clone())
            .with_pigeon2_id(self.pigeon_id);
        if let Some(pigeon_configs) = &self.pigeon_configs {
            drivetrain_constants =
                drivetrain_constants.with_pigeon2_configs(pigeon_configs.clone());
        }
        self.drivetrain_constants = drivetrain_constants;

        self.constant_creator = ModuleConstantsFactory {
            base: ModuleConstants {
                drive_motor_gear_ratio: self.drive_gear_ratio,
                steer_motor_gear_ratio: self.steer_gear_ratio,
                wheel_radius_m: inches_to_meters(self.wheel_radius_inches),
                slip_current_a: self.slip_current_amps,
                steer_motor_gains: self.steer_gains.clone(),
                drive_motor_gains: self.drive_gains.clone(),
                steer_motor_closed_loop_output: self.steer_closed_loop_output,
                drive_motor_closed_loop_output: self.drive_closed_loop_output,
                speed_at_12_volts_mps: self.linear_speed_at_12_volts_mps,
                steer_inertia: self.steer_inertia,
                drive_inertia: self.drive_inertia,
                steer_friction_voltage: self.steer_friction_voltage,
                drive_friction_voltage: self.drive_friction_voltage,
                feedback_source: SteerFeedbackType::FusedCANcoder,
                coupling_gear_ratio: self.couple_ratio,
                drive_motor_initial_configs: Some(self.drive_initial_configs.clone()),
                steer_motor_initial_configs: Some(self.steer_initial_configs.clone()),
                encoder_initial_configs: Some(self.can_coder_initial_configs.clone()),
                ..ModuleConstants::default()
            },
        };

        self.front_left = self.constant_creator.create_module_constants(
            self.front_left_steer_motor_id,
            self.front_left_drive_motor_id,
            self.front_left_encoder_id,
            self.front_left_encoder_offset_rot,
            inches_to_meters(self.front_left_x_pos_inches),
            inches_to_meters(self.front_left_y_pos_inches),
            self.invert_left_side,
            self.steer_motor_reversed,
            self.front_left_steer_inverted,
        );

        self.front_right = self.constant_creator.create_module_constants(
            self.front_right_steer_motor_id,
            self.front_right_drive_motor_id,
            self.front_right_encoder_id,
            self.front_right_encoder_offset_rot,
            inches_to_meters(self.front_right_x_pos_inches),
            inches_to_meters(self.front_right_y_pos_inches),
            self.invert_right_side,
            self.steer_motor_reversed,
            self.front_right_steer_inverted,
        );

        self.back_left = self.constant_creator.create_module_constants(
            self.back_left_steer_motor_id,
            self.back_left_drive_motor_id,
            self.back_left_encoder_id,
            self.back_left_encoder_offset_rot,
            inches_to_meters(self.back_left_x_pos_inches),
            inches_to_meters(self.back_left_y_pos_inches),
            self.invert_left_side,
            self.steer_motor_reversed,
            self.back_left_steer_inverted,
        );

        self.back_right = self.constant_creator.create_module_constants(
            self.back_right_steer_motor_id,
            self.back_right_drive_motor_id,
            self.back_right_encoder_id,
            self.back_right_encoder_offset_rot,
            inches_to_meters(self.back_right_x_pos_inches),
            inches_to_meters(self.back_right_y_pos_inches),
            self.invert_right_side,
            self.steer_motor_reversed,
            self.back_right_steer_inverted,
        );

        self
    }

    pub fn config_encoder_offsets(
        &mut self,
        front_left: f64,
        front_right: f64,
        back_left: f64,
        back_right: f64,
    ) -> &mut Self {
        self.front_left_encoder_offset_rot = front_left;
        self.front_right_encoder_offset_rot = front_right;
        self.back_left_encoder_offset_rot = back_left;
        self.back_right_encoder_offset_rot = back_right;
        self.update_config()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-12, "{expected} vs {actual}");
    }

    #[test]
    fn defaults_hold_the_java_robot_geometry() {
        let config = SwerveConfig::default();
        assert_close(21.75_f64.hypot(21.75) / 2.0, config.drivebase_radius_inches);
        assert_close(
            inches_to_meters(config.drivebase_radius_inches),
            config.drivebase_radius_meters,
        );
        assert_eq!(config.front_left.location_x_m, inches_to_meters(10.875));
        assert_eq!(config.front_left.location_y_m, inches_to_meters(10.875));
        assert_eq!(config.back_right.location_x_m, inches_to_meters(-10.875));
        assert_eq!(config.back_right.location_y_m, inches_to_meters(-10.875));
    }

    #[test]
    fn module_ids_and_sides_follow_the_java_constructor_arguments() {
        let config = SwerveConfig::default();
        let [fl, fr, bl, br] = config.get_modules();
        assert_eq!(
            (fl.drive_motor_id, fl.steer_motor_id, fl.encoder_id),
            (1, 2, 3)
        );
        assert_eq!(
            (fr.drive_motor_id, fr.steer_motor_id, fr.encoder_id),
            (11, 12, 13)
        );
        assert_eq!(
            (bl.drive_motor_id, bl.steer_motor_id, bl.encoder_id),
            (21, 22, 23)
        );
        assert_eq!(
            (br.drive_motor_id, br.steer_motor_id, br.encoder_id),
            (31, 32, 33)
        );
        assert!(!fl.drive_motor_inverted && !bl.drive_motor_inverted);
        assert!(fr.drive_motor_inverted && br.drive_motor_inverted);
    }

    #[test]
    fn shared_constants_reach_every_module() {
        let config = SwerveConfig::default();
        for module in config.get_modules() {
            assert_eq!(module.drive_motor_gear_ratio, 6.03);
            assert_eq!(module.steer_motor_gear_ratio, 26.09);
            assert_eq!(module.coupling_gear_ratio, 4.5);
            assert_eq!(module.speed_at_12_volts_mps, 5.12);
            assert_eq!(module.slip_current_a, 80.0);
            assert_close(inches_to_meters(1.964), module.wheel_radius_m);
            assert_eq!(
                module.steer_motor_closed_loop_output,
                ClosedLoopOutputType::TorqueCurrentFoc
            );
            assert_eq!(module.feedback_source, SteerFeedbackType::FusedCANcoder);
            assert!(module.drive_motor_initial_configs.is_some());
        }
    }

    #[test]
    fn encoder_offsets_rebuild_the_modules() {
        let mut config = SwerveConfig::default();
        config.config_encoder_offsets(0.1, 0.2, 0.3, 0.4);
        assert_eq!(config.front_left.encoder_offset_rot, 0.1);
        assert_eq!(config.front_right.encoder_offset_rot, 0.2);
        assert_eq!(config.back_left.encoder_offset_rot, 0.3);
        assert_eq!(config.back_right.encoder_offset_rot, 0.4);
    }

    #[test]
    fn drivetrain_constants_name_the_bus_and_pigeon() {
        let config = SwerveConfig::default();
        assert_eq!(config.drivetrain_constants.can_bus_name, "*");
        assert_eq!(config.drivetrain_constants.pigeon2_id, 0);
        assert!(config.drivetrain_constants.pigeon2_configs.is_some());
    }
}
