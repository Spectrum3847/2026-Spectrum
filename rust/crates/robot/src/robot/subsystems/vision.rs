//! Fuses the three Limelights (back, left, right) into the swerve pose estimator.
//!
//! Only the camera with the best view feeds the estimator. While disabled both MegaTag1 and
//! MegaTag2 estimates are fused to pre-seed the pose; while enabled only MegaTag1 is.
//!
//! The accept and reject rules are pure functions ([`mt1_decision`], [`mt2_decision`],
//! [`rejection_check`], [`reset_pose_to_vision_check`]) so they run without a camera.
//!
//! Java loads the 2026 AprilTag field layout into a static that nothing reads, so it is not
//! loaded here.

use crate::context;
use crate::rebuilt::field_helpers;
use crate::spectrum_lib::util::util;
use crate::spectrum_lib::vision::limelight::{Limelight, LimelightConfig};
use crate::spectrum_lib::vision::limelight_helpers::{LimelightHelpers, RawFiducial};
use crate::spectrum_lib::vision::vision_logger::VisionLogger;
use commands::{Command, CommandScheduler, Subsystem, SubsystemBase, SubsystemId};
use phoenix6::utils;
use std::cell::RefCell;
use std::rc::Rc;
use telemetry::{PrintPriority, telemetry};
use wpilib::driver_station::{self, Alliance};
use wpilib::{nt, timer};
use wpimath::geometry::{Pose2d, Pose3d, Translation2d};
use wpimath::kinematics::ChassisSpeeds;
use wpimath::units::{degrees_to_radians, inches_to_meters};

/// Camera identities, mount poses (translation in metres, rotation in degrees), and fusion
/// covariance.
#[derive(Clone, Debug)]
pub struct VisionConfig {
    pub name: String,

    pub back_ll: String,
    pub back_config: LimelightConfig,

    pub left_ll: String,
    pub left_config: LimelightConfig,

    pub right_ll: String,
    pub right_config: LimelightConfig,

    /// Robot centre to turret pivot offset, in metres.
    pub robot_to_turret_center: Translation2d,

    /// Turret pivot to camera offset, in metres.
    pub turret_center_to_camera: Translation2d,

    pub back_tag_pipeline: i32,
    pub left_tag_pipeline: i32,
    pub right_tag_pipeline: i32,

    /// Default translational standard deviation for a vision measurement, in metres. Lower
    /// values trust vision more, higher values trust odometry more.
    pub vision_std_dev_x: f64,

    /// See `vision_std_dev_x`.
    pub vision_std_dev_y: f64,

    /// Default rotational standard deviation for a vision measurement, in radians. Only the
    /// MT1 path reads it; MT2 heading is always discarded.
    pub vision_std_dev_theta: f64,

    /// Weight large enough to ignore a measurement dimension, such as MT2 heading.
    pub k_large_variance: f64,

    /// Measurements older than this many seconds are not fused.
    pub k_max_time_delta_seconds: f64,

    pub vision_std_matrix: [f64; 3],
}

impl Default for VisionConfig {
    fn default() -> Self {
        let vision_std_dev_x = 0.5;
        let vision_std_dev_y = 0.5;
        let vision_std_dev_theta = 0.2;
        let back_ll = "limelight-back".to_owned();
        let left_ll = "limelight-left".to_owned();
        let right_ll = "limelight-right".to_owned();
        Self {
            name: "Vision".to_owned(),
            back_config: LimelightConfig::new(&back_ll)
                .with_translation(-0.3084987734, 0.2134100126, 0.6502249886)
                .with_rotation(0.0, 0.0, 180.0),
            left_config: LimelightConfig::new(&left_ll)
                .with_translation(0.0, 0.215, 0.188)
                .with_rotation(0.0, 0.0, 90.0),
            right_config: LimelightConfig::new(&right_ll)
                .with_translation(-0.04445, 0.3027487722, 0.7137249886)
                .with_rotation(0.0, 0.0, -90.0),
            back_ll,
            left_ll,
            right_ll,
            robot_to_turret_center: Translation2d::new(
                inches_to_meters(-5.5),
                inches_to_meters(4.7),
            ),
            turret_center_to_camera: Translation2d::new(inches_to_meters(-5.641455), 0.0),
            back_tag_pipeline: 0,
            left_tag_pipeline: 0,
            right_tag_pipeline: 0,
            vision_std_dev_x,
            vision_std_dev_y,
            vision_std_dev_theta,
            k_large_variance: 999999.0,
            k_max_time_delta_seconds: 0.1,
            vision_std_matrix: [vision_std_dev_x, vision_std_dev_y, vision_std_dev_theta],
        }
    }
}

/// A pose estimate bundled with the timestamp and std-devs that `add_vision_measurement`
/// expects.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VisionFieldPoseEstimate {
    pub vision_robot_pose_meters: Pose2d,
    /// Capture time already converted out of the FPGA clock, in seconds.
    pub timestamp_seconds: f64,
    /// Std-devs for `[x, y, theta]`. Larger values mean less trust in that dimension.
    pub vision_measurement_std_devs: [f64; 3],
    pub num_tags: i32,
}

/// Field tags that score for the blue alliance.
const BLUE_TAGS: [i32; 8] = [18, 19, 20, 21, 24, 25, 26, 27];

/// Field tags that score for the red alliance.
const RED_TAGS: [i32; 9] = [2, 3, 4, 5, 8, 9, 10, 11, 12];

/// An estimate that passed the checks, with the status the camera reports and its std-devs.
/// `deg_stds` is in degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Accepted {
    pub status: &'static str,
    pub xy_stds: f64,
    pub deg_stds: f64,
}

/// What a camera and the swerve tell the accept rules.
pub struct Mt1Inputs<'a> {
    pub target_in_view: bool,
    pub multi_tags: bool,
    pub target_size: f64,
    pub pose3d: Pose3d,
    pub tags: &'a [RawFiducial],
    pub robot_speed: ChassisSpeeds,
    pub robot_translation: Translation2d,
    pub force_integrate_xy: bool,
    /// Capture time of the MT1 pose, on the Phoenix clock.
    pub timestamp: f64,
    /// Now, on the Phoenix clock.
    pub current_time: f64,
}

/// `tag_status` is what to set on the camera first, if anything. `outcome` is the estimate's
/// std-devs, or the reason it was rejected.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decision {
    pub tag_status: Option<&'static str>,
    pub outcome: Result<Accepted, &'static str>,
}

/// Rejection gate shared by the MT1 and MT2 paths. Rejects on a pose outside the field, a
/// capture older than `k_max_time_delta_seconds`, a spin rate of at least 1.6 rad/s, and a
/// target area of at most 0.025 %. The staleness check reads the MT1 timestamp, so the MT2 path
/// is gated on the MT1 capture age. Returns the reason to reject.
pub fn rejection_check(
    config: &VisionConfig,
    pose: Pose2d,
    target_size: f64,
    timestamp: f64,
    current_time: f64,
    omega: f64,
) -> Option<&'static str> {
    if field_helpers::pose_out_of_field(pose) {
        return Some("Out of Field Rejection");
    }

    if current_time - timestamp > config.k_max_time_delta_seconds {
        return Some("Stale Timestamp Rejection");
    }

    if omega.abs() >= 1.6 {
        return Some("Rotation Speed Rejection");
    }

    if target_size <= 0.025 {
        return Some("Target Size Rejection");
    }

    None
}

fn tilted(pose3d: &Pose3d) -> bool {
    pose3d.rotation.x().abs() > 5.0_f64.to_radians()
        || pose3d.rotation.y().abs() > 5.0_f64.to_radians()
}

/// Decides whether a MegaTag1 estimate is trustworthy enough to fuse.
///
/// Rejects on any of:
/// - No targets in view.
/// - Any tag ambiguity above 0.9, which risks a pose flip.
/// - Pose outside the field, or a capture older than `k_max_time_delta_seconds`.
/// - Spin rate of at least 1.6 rad/s.
/// - Target area of at most 0.025 %.
/// - Roll or pitch above 5 degrees, which means the camera was bumped.
///
/// Accepted estimates get std-devs from a confidence tier based on tag count, target area, and
/// distance from the current odometry pose. `force_integrate_xy` clamps those to near zero so
/// pre-seeding snaps the estimator to vision.
pub fn mt1_decision(config: &VisionConfig, inputs: &Mt1Inputs<'_>) -> Decision {
    if !inputs.target_in_view {
        return Decision {
            tag_status: Some("No Targets in View"),
            outcome: Err("No Targets in View Rejection"),
        };
    }
    let reject = |reason| Decision {
        tag_status: Some(""),
        outcome: Err(reason),
    };

    let multi_tags = inputs.multi_tags;
    let target_size = inputs.target_size;
    let mega_tag1_pose2d = inputs.pose3d.to_pose2d();
    let mut highest_ambiguity = -1.0;
    let robot_speed = inputs.robot_speed;
    let robot_linear_speed = robot_speed.vx.hypot(robot_speed.vy);

    let mt1_pose_difference = inputs
        .robot_translation
        .distance(mega_tag1_pose2d.translation);

    for tag in inputs.tags {
        if highest_ambiguity < 0.0 || tag.ambiguity > highest_ambiguity {
            highest_ambiguity = tag.ambiguity;
        }
        if tag.ambiguity > 0.9 {
            return reject("High Ambiguity Rejection");
        }
    }

    if let Some(reason) = rejection_check(
        config,
        mega_tag1_pose2d,
        target_size,
        inputs.timestamp,
        inputs.current_time,
        robot_speed.omega,
    ) {
        return reject(reason);
    }

    if tilted(&inputs.pose3d) {
        return reject("Roll/Pitch Rejection");
    }

    let (status, mut xy_stds, mut deg_stds);

    if robot_linear_speed <= 0.2 && target_size > 4.0 {
        status = "Stationary close integration";
        xy_stds = 0.1;
        deg_stds = 0.1;
    } else if multi_tags && target_size > 2.0 {
        status = "Strong Multi integration";
        xy_stds = 0.1;
        deg_stds = 0.1;
    } else if multi_tags && target_size > 0.2 {
        status = "Multi integration";
        xy_stds = 0.25;
        deg_stds = 8.0;
    } else if target_size > 2.0 && mt1_pose_difference < 0.5 {
        status = "Close integration";
        xy_stds = 0.5;
        deg_stds = config.k_large_variance;
    } else if target_size > 1.0 && mt1_pose_difference < 0.25 {
        status = "Proximity integration";
        xy_stds = 1.0;
        deg_stds = config.k_large_variance;
    } else if highest_ambiguity < 0.25 && target_size >= 0.03 {
        status = "Stable integration";
        xy_stds = 1.5;
        deg_stds = config.k_large_variance;
    } else {
        return reject("Integration Criteria not Met");
    }

    if highest_ambiguity > 0.5 {
        deg_stds = f64::max(deg_stds, 15.0);
    }

    if robot_speed.omega.abs() >= 0.5 {
        deg_stds = f64::max(deg_stds, 50.0);
    }

    if inputs.force_integrate_xy {
        xy_stds = 0.01;
        deg_stds = 0.01;
    }

    Decision {
        tag_status: Some(""),
        outcome: Ok(Accepted {
            status,
            xy_stds,
            deg_stds,
        }),
    }
}

/// What a camera and the swerve tell the MegaTag2 accept rules.
pub struct Mt2Inputs {
    pub target_in_view: bool,
    pub multi_tags: bool,
    pub target_size: f64,
    pub pose2d: Pose2d,
    pub robot_speed: ChassisSpeeds,
    pub robot_translation: Translation2d,
    pub disabled: bool,
    /// Capture time of the MT1 pose, which gates the staleness check.
    pub mt1_timestamp: f64,
    pub current_time: f64,
}

/// Decides whether a MegaTag2 estimate is fused.
///
/// MegaTag2 heading is discarded (`k_large_variance`) because it comes from the IMU rather
/// than tag geometry. Only the disabled path calls this, which means the disabled terms in the
/// distance tiers always pass and pose distance never rejects here.
///
/// Rejects on no targets in view, plus everything [`rejection_check`] covers.
pub fn mt2_decision(config: &VisionConfig, inputs: &Mt2Inputs) -> Decision {
    if !inputs.target_in_view {
        return Decision {
            tag_status: Some("No Targets in View"),
            outcome: Err("No Targets in View Rejection"),
        };
    }

    let multi_tags = inputs.multi_tags;
    let target_size = inputs.target_size;
    let robot_speed = inputs.robot_speed;
    let robot_linear_speed = robot_speed.vx.hypot(robot_speed.vy);

    let mt2_pose_difference = inputs.robot_translation.distance(inputs.pose2d.translation);

    // Java: the staleness check reads the MT1 capture time, not the MT2 one.
    if let Some(reason) = rejection_check(
        config,
        inputs.pose2d,
        target_size,
        inputs.mt1_timestamp,
        inputs.current_time,
        robot_speed.omega,
    ) {
        return Decision {
            tag_status: None,
            outcome: Err(reason),
        };
    }

    let (status, xy_stds);

    if robot_linear_speed <= 0.2 && target_size > 4.0 {
        status = "Stationary close integration";
        xy_stds = 0.1;
    } else if multi_tags && target_size > 2.0 {
        status = "Strong Multi integration";
        xy_stds = 0.1;
    } else if multi_tags && target_size > 0.2 {
        status = "Multi integration";
        xy_stds = 0.25;
    } else if target_size > 2.0 && (mt2_pose_difference < 0.5 || inputs.disabled) {
        status = "Close integration";
        xy_stds = 0.5;
    } else if target_size > 1.0 && (mt2_pose_difference < 0.25 || inputs.disabled) {
        status = "Proximity integration";
        xy_stds = 1.0;
    } else if target_size >= 0.03 {
        status = "Stable integration";
        xy_stds = 1.5;
    } else {
        return Decision {
            tag_status: None,
            outcome: Err("Integration Criteria not Met"),
        };
    }

    Decision {
        tag_status: None,
        outcome: Ok(Accepted {
            status,
            xy_stds,
            deg_stds: config.k_large_variance,
        }),
    }
}

/// Checks a pose reset against vision. Rejects on:
/// - No target in view.
/// - Either pose outside the field.
/// - Camera height `|z| > 0.25 m`, which means a floating robot or a bad solve.
/// - Roll or pitch above 5 degrees, which means the camera was bumped.
///
/// `Ok` holds the pose to fuse: the MT2 translation with the MT1 heading. `Err` holds the
/// reason to log, or `None` when no target is in view and nothing is logged.
pub fn reset_pose_to_vision_check(
    target_in_view: bool,
    botpose3d: Pose3d,
    mega_pose: Pose2d,
) -> Result<Pose2d, Option<&'static str>> {
    if !target_in_view {
        return Err(None);
    }

    if field_helpers::pose_out_of_field_3d(botpose3d) {
        return Err(Some("Out of field"));
    }
    if botpose3d.z().abs() > 0.25 {
        return Err(Some("Pose in air"));
    }
    if tilted(&botpose3d) {
        return Err(Some("Pose tilted"));
    }

    if field_helpers::pose_out_of_field(mega_pose) {
        return Err(Some("MegaPose out of field"));
    }

    Ok(Pose2d::new(
        mega_pose.translation,
        botpose3d.to_pose2d().rotation,
    ))
}

/// Index of the camera with the highest tag count plus target size, or 0 (the back camera) when
/// nothing is in view.
pub fn best_limelight_index(scores: &[f64]) -> usize {
    let mut best_index = 0;
    let mut best_score = 0.0;
    for (index, score) in scores.iter().enumerate() {
        if *score > best_score {
            best_score = *score;
            best_index = index;
        }
    }
    best_index
}

/// True when `id` is a scoring tag for the alliance.
pub fn is_scoring_tag(alliance: Alliance, id: i32) -> bool {
    let alliance_tags: &[i32] = if alliance == Alliance::Blue {
        &BLUE_TAGS
    } else {
        &RED_TAGS
    };
    alliance_tags.contains(&id)
}

pub struct Vision {
    base: SubsystemBase,
    config: VisionConfig,

    pub back_ll: Rc<Limelight>,
    pub left_ll: Rc<Limelight>,
    pub right_ll: Rc<Limelight>,

    pub all_limelights: [Rc<Limelight>; 3],

    back_logger: VisionLogger,
    left_logger: VisionLogger,
    right_logger: VisionLogger,

    last_imu_mode_by_ll: RefCell<[Option<i32>; 3]>,
}

impl Vision {
    pub fn new(config: VisionConfig) -> Rc<Vision> {
        let back_ll = Rc::new(Limelight::with_pipeline_and_config(
            &config.back_ll,
            config.back_tag_pipeline,
            config.back_config.clone(),
        ));
        let left_ll = Rc::new(Limelight::with_pipeline_and_config(
            &config.left_ll,
            config.left_tag_pipeline,
            config.left_config.clone(),
        ));
        let right_ll = Rc::new(Limelight::with_pipeline_and_config(
            &config.right_ll,
            config.right_tag_pipeline,
            config.right_config.clone(),
        ));

        let all_limelights = [back_ll.clone(), left_ll.clone(), right_ll.clone()];

        let back_logger = VisionLogger::new("BackLL", back_ll.clone());
        let left_logger = VisionLogger::new("LeftLL", left_ll.clone());
        let right_logger = VisionLogger::new("RightLL", right_ll.clone());

        let vision = Rc::new(Vision {
            base: SubsystemBase::new(config.name.clone()),
            config,
            back_ll,
            left_ll,
            right_ll,
            all_limelights,
            back_logger,
            left_logger,
            right_logger,
            last_imu_mode_by_ll: RefCell::new([None; 3]),
        });

        for index in 0..vision.all_limelights.len() {
            vision.all_limelights[index].set_led_mode(false);
            vision.set_imu_mode_if_changed(index, 1);
        }

        CommandScheduler::get_instance().register_subsystem(vision.clone());
        telemetry::print(&format!("{} Subsystem Initialized", vision.name()));
        vision
    }

    fn all_loggers(&self) -> [&VisionLogger; 3] {
        [&self.back_logger, &self.left_logger, &self.right_logger]
    }

    pub fn log_telemetry(&self) {
        for logger in self.all_loggers() {
            logger.get_camera_connection();
            logger.get_integrating_status();
            logger.get_log_status();
            logger.get_tag_status();
            logger.get_pose();
            logger.get_tag_count();
            logger.get_target_size();
        }

        // MegaTag2 fuses the IMU, so its estimate drifts as soon as the robot moves
        if util::disabled().get_as_boolean() {
            self.back_logger.get_mega_pose();
            self.left_logger.get_mega_pose();
            self.right_logger.get_mega_pose();
        }

        let field2d = context::field2d();
        field2d
            .get_object(self.back_ll.get_camera_name())
            .set_pose(self.get_back_mega_tag1_pose());
        field2d
            .get_object(self.left_ll.get_camera_name())
            .set_pose(self.get_left_mega_tag1_pose());
        field2d
            .get_object(self.right_ll.get_camera_name())
            .set_pose(self.get_right_mega_tag1_pose());
    }

    /// Pushes the current swerve heading to every camera so MegaTag2 fuses the right yaw.
    fn set_limelight_orientation(&self) {
        let yaw = context::swerve().get_robot_pose().rotation.degrees();
        for limelight in &self.all_limelights {
            limelight.set_robot_orientation(yaw);
        }
    }

    /// Pre-seeds the pose estimator from the best camera while the robot is disabled, so the
    /// first enabled loop starts from a vision pose instead of a stale one.
    fn disabled_limelight_updates(&self) {
        if util::disabled().get_as_boolean() {
            let best_limelight = self.get_best_limelight();
            self.integrate_single_estimate(self.get_mt1_vision_estimate(&best_limelight, true));
            self.integrate_single_estimate(self.get_mt2_vision_estimate(&best_limelight));
        }
    }

    /// Fuses the best camera's MegaTag1 estimate in teleop and the two auton states.
    fn enabled_limelight_updates(&self) {
        if util::teleop().get_as_boolean()
            || context::auton_pose_update().get_as_boolean()
            || context::auton_launching().get_as_boolean()
        {
            let best_limelight = self.get_best_limelight();
            self.integrate_single_estimate(self.get_mt1_vision_estimate(&best_limelight, false));
        }
    }

    /// Builds a MegaTag1 estimate for the given camera and decides whether it is trustworthy
    /// enough to fuse. See [`mt1_decision`] for the rules. Returns `None` when rejected.
    fn get_mt1_vision_estimate(
        &self,
        ll: &Limelight,
        force_integrate_xy: bool,
    ) -> Option<VisionFieldPoseEstimate> {
        let swerve = context::swerve();
        let mega_tag1_pose3d = ll.get_mega_tag1_pose3d();
        let tags = ll.get_raw_fiducial();
        let timestamp = utils::fpga_to_current_time(ll.get_mega_tag1_pose_timestamp());
        let decision = mt1_decision(
            &self.config,
            &Mt1Inputs {
                target_in_view: ll.target_in_view(),
                multi_tags: ll.multiple_tags_in_view(),
                target_size: ll.get_target_size(),
                pose3d: mega_tag1_pose3d,
                tags: &tags,
                robot_speed: swerve.get_current_robot_chassis_speeds(),
                robot_translation: swerve.get_robot_pose().translation,
                force_integrate_xy,
                timestamp,
                current_time: utils::fpga_to_current_time(timer::fpga_timestamp()),
            },
        );
        apply_decision(ll, &decision);
        let accepted = decision.outcome.ok()?;

        let mega_tag1_pose2d = mega_tag1_pose3d.to_pose2d();
        let integrated_pose = Pose2d::new(mega_tag1_pose2d.translation, mega_tag1_pose2d.rotation);
        // deg_stds is in degrees while the estimator wants radians
        let std_devs = [
            accepted.xy_stds,
            accepted.xy_stds,
            degrees_to_radians(accepted.deg_stds),
        ];

        Some(VisionFieldPoseEstimate {
            vision_robot_pose_meters: integrated_pose,
            timestamp_seconds: timestamp,
            vision_measurement_std_devs: std_devs,
            num_tags: tags.len() as i32,
        })
    }

    /// Builds a MegaTag2 estimate for the given camera. See [`mt2_decision`] for the rules.
    /// Returns `None` when rejected.
    fn get_mt2_vision_estimate(&self, ll: &Limelight) -> Option<VisionFieldPoseEstimate> {
        let swerve = context::swerve();
        let mega_tag2_pose2d = ll.get_mega_tag2_pose2d();
        let decision = mt2_decision(
            &self.config,
            &Mt2Inputs {
                target_in_view: ll.target_in_view(),
                multi_tags: ll.multiple_tags_in_view(),
                target_size: ll.get_target_size(),
                pose2d: mega_tag2_pose2d,
                robot_speed: swerve.get_current_robot_chassis_speeds(),
                robot_translation: swerve.get_robot_pose().translation,
                disabled: driver_station::is_disabled(),
                mt1_timestamp: utils::fpga_to_current_time(ll.get_mega_tag1_pose_timestamp()),
                current_time: utils::fpga_to_current_time(timer::fpga_timestamp()),
            },
        );
        apply_decision(ll, &decision);
        let accepted = decision.outcome.ok()?;

        let integrated_pose = Pose2d::new(mega_tag2_pose2d.translation, mega_tag2_pose2d.rotation);

        Some(VisionFieldPoseEstimate {
            vision_robot_pose_meters: integrated_pose,
            timestamp_seconds: utils::fpga_to_current_time(ll.get_mega_tag2_pose_timestamp()),
            vision_measurement_std_devs: [
                accepted.xy_stds,
                accepted.xy_stds,
                degrees_to_radians(accepted.deg_stds),
            ],
            num_tags: ll.get_tag_count_in_view() as i32,
        })
    }

    /// Fuses the estimate, ignoring the `None` that a rejected candidate returns.
    fn integrate_single_estimate(&self, estimate: Option<VisionFieldPoseEstimate>) {
        if let Some(estimate) = estimate {
            context::swerve().add_vision_measurement_with_std_devs(
                estimate.vision_robot_pose_meters,
                estimate.timestamp_seconds,
                estimate.vision_measurement_std_devs,
            );
        }
    }

    /// Writes the mode only when it differs from the last one written, to spare NetworkTables.
    /// `desired_mode` 0 takes the heading from the DriverStation, 1 from the camera's own IMU,
    /// 2 fuses the two.
    fn set_imu_mode_if_changed(&self, limelight_index: usize, desired_mode: i32) {
        let last_mode = self.last_imu_mode_by_ll.borrow()[limelight_index];
        if last_mode != Some(desired_mode) {
            self.all_limelights[limelight_index].set_imu_mode(desired_mode);
            self.last_imu_mode_by_ll.borrow_mut()[limelight_index] = Some(desired_mode);
        }
    }

    /// MegaTag1 pose, or zero without a usable estimate.
    pub fn get_back_mega_tag1_pose(&self) -> Pose2d {
        self.back_ll.get_mega_tag1_pose3d().to_pose2d()
    }

    /// MegaTag1 pose, or zero without a usable estimate.
    pub fn get_left_mega_tag1_pose(&self) -> Pose2d {
        self.left_ll.get_mega_tag1_pose3d().to_pose2d()
    }

    /// MegaTag1 pose, or zero without a usable estimate.
    pub fn get_right_mega_tag1_pose(&self) -> Pose2d {
        self.right_ll.get_mega_tag1_pose3d().to_pose2d()
    }

    /// MegaTag2 pose, or zero without a usable estimate.
    pub fn get_back_mega_tag2_pose(&self) -> Pose2d {
        self.back_ll.get_mega_tag2_pose2d()
    }

    /// MegaTag2 pose, or zero without a usable estimate.
    pub fn get_left_mega_tag2_pose(&self) -> Pose2d {
        self.left_ll.get_mega_tag2_pose2d()
    }

    /// MegaTag2 pose, or zero without a usable estimate.
    pub fn get_right_mega_tag2_pose(&self) -> Pose2d {
        self.right_ll.get_mega_tag2_pose2d()
    }

    /// Camera with the highest tag count plus target size, or the back one when nothing is in
    /// view.
    pub fn get_best_limelight(&self) -> Rc<Limelight> {
        let scores = self
            .all_limelights
            .each_ref()
            .map(|ll| ll.get_tag_count_in_view() + ll.get_target_size());
        self.all_limelights[best_limelight_index(&scores)].clone()
    }

    pub fn has_accurate_pose(&self) -> bool {
        self.all_limelights.iter().any(|ll| ll.has_accurate_pose())
    }

    /// True if any camera's primary target is a scoring tag for the current alliance, treating
    /// an unknown alliance as blue.
    pub fn tags_in_view(&self) -> bool {
        let alliance = driver_station::alliance().unwrap_or(Alliance::Blue);
        self.all_limelights
            .iter()
            .any(|ll| is_scoring_tag(alliance, ll.get_closest_tag_id() as i32))
    }

    /// Asks every camera to rewind-capture 165 s, the longest buffer the Limelight keeps.
    pub fn trigger_rewind_capture_for_all_cameras(&self) {
        let helpers = LimelightHelpers::default();
        for limelight in &self.all_limelights {
            helpers.trigger_rewind_capture(limelight.get_name(), 165.0);
        }
    }

    /// Resets the pose from the best camera, taking its MT2 translation and MT1 heading. A
    /// rejected pose leaves the estimator untouched, and this overload gives the caller no way
    /// to tell.
    pub fn reset_pose_to_vision(&self) {
        let ll = self.get_best_limelight();
        self.reset_pose_to_vision_with(
            ll.target_in_view(),
            ll.get_mega_tag1_pose3d(),
            ll.get_mega_tag2_pose2d(),
            ll.get_mega_tag1_pose_timestamp(),
        );
    }

    /// Snaps the pose estimator onto a specific vision pose. The covariance is tiny, so the
    /// estimator jumps to vision instead of blending toward it. See
    /// [`reset_pose_to_vision_check`] for the rejections. Returns true if the reset was
    /// applied.
    pub fn reset_pose_to_vision_with(
        &self,
        target_in_view: bool,
        botpose3d: Pose3d,
        mega_pose: Pose2d,
        pose_timestamp: f64,
    ) -> bool {
        let botpose = botpose3d.to_pose2d();
        let integrated_pose = match reset_pose_to_vision_check(target_in_view, botpose3d, mega_pose)
        {
            Ok(pose) => pose,
            Err(reason) => {
                if let Some(reason) = reason {
                    telemetry::log("Vision/PoseReset/Rejection", reason);
                }
                return false;
            }
        };

        let before = [botpose.x(), botpose.y(), botpose.rotation.degrees()];
        telemetry::log("Vision/PoseReset/Before", &before);

        let swerve = context::swerve();
        swerve.add_vision_measurement_with_std_devs(
            integrated_pose,
            utils::fpga_to_current_time(pose_timestamp),
            [0.00001, 0.00001, 0.00001],
        );

        let updated = swerve.get_robot_pose();
        let after = [updated.x(), updated.y(), updated.rotation.degrees()];
        telemetry::log("Vision/PoseReset/After", &after);

        true
    }

    pub fn set_limelight_pipelines(&self, pipeline: i32) {
        for limelight in &self.all_limelights {
            limelight.set_limelight_pipeline(pipeline);
        }
    }

    /// Blinks every camera's LEDs while the command runs, then turns them off.
    pub fn blink_limelights(&self) -> Command {
        telemetry::print_with("Vision.blinkLimelights", PrintPriority::High);
        let (start_cameras, end_cameras) =
            (self.all_limelights.clone(), self.all_limelights.clone());
        self.start_end(
            move || {
                for limelight in &start_cameras {
                    limelight.blink_leds();
                }
            },
            move || {
                for limelight in &end_cameras {
                    limelight.set_led_mode(false);
                }
            },
        )
        .with_name("Vision.blinkLimelights")
    }

    /// Holds every camera's LEDs on while the command runs, then turns them off.
    pub fn solid_limelight(&self) -> Command {
        let (start_cameras, end_cameras) =
            (self.all_limelights.clone(), self.all_limelights.clone());
        self.start_end(
            move || {
                for limelight in &start_cameras {
                    limelight.set_led_mode(true);
                }
            },
            move || {
                for limelight in &end_cameras {
                    limelight.set_led_mode(false);
                }
            },
        )
        .with_name("Vision.solidLimelight")
    }
}

fn apply_decision(ll: &Limelight, decision: &Decision) {
    if let Some(tag_status) = decision.tag_status {
        ll.set_tag_status(tag_status);
    }
    match decision.outcome {
        Ok(accepted) => ll.send_valid_status(accepted.status),
        Err(reason) => ll.send_invalid_status(reason),
    }
}

impl Subsystem for Vision {
    fn id(&self) -> SubsystemId {
        self.base.id()
    }

    fn name(&self) -> String {
        self.config.name.clone()
    }

    /// Called by the scheduler every loop, so everything here has to stay cheap.
    fn periodic(&self) {
        for limelight in &self.all_limelights {
            limelight.invalidate();
        }
        self.set_limelight_orientation();
        // set_robot_orientation does not flush; one flush sends every camera's heading now.
        nt::Instance::default_instance().flush();
        self.disabled_limelight_updates();
        self.enabled_limelight_updates();
        self.log_telemetry();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wpimath::geometry::{Rotation2d, Rotation3d, Translation3d};

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-12, "{expected} vs {actual}");
    }

    fn fiducial(id: i32, ambiguity: f64) -> RawFiducial {
        RawFiducial {
            id,
            txnc: 0.0,
            tync: 0.0,
            ta: 0.0,
            dist_to_camera: 0.0,
            dist_to_robot: 0.0,
            ambiguity,
        }
    }

    fn pose3d(x: f64, y: f64, z: f64, roll_deg: f64, pitch_deg: f64) -> Pose3d {
        Pose3d {
            translation: Translation3d::new(x, y, z),
            rotation: Rotation3d::from_euler(roll_deg.to_radians(), pitch_deg.to_radians(), 0.3),
        }
    }

    /// A good estimate: in the field, fresh, still, tilt-free. Tests edit what they test.
    fn good<'a>(tags: &'a [RawFiducial]) -> Mt1Inputs<'a> {
        Mt1Inputs {
            target_in_view: true,
            multi_tags: false,
            target_size: 1.0,
            pose3d: pose3d(5.0, 4.0, 0.0, 0.0, 0.0),
            tags,
            robot_speed: ChassisSpeeds::new(0.0, 0.0, 0.0),
            robot_translation: Translation2d::new(5.0, 4.0),
            force_integrate_xy: false,
            timestamp: 10.0,
            current_time: 10.05,
        }
    }

    fn accepted(decision: Decision) -> Accepted {
        decision
            .outcome
            .unwrap_or_else(|reason| panic!("rejected: {reason}"))
    }

    fn rejected(decision: Decision) -> &'static str {
        decision.outcome.expect_err("estimate should be rejected")
    }

    #[test]
    fn no_target_rejects_and_sets_the_tag_status() {
        let config = VisionConfig::default();
        let mut inputs = good(&[]);
        inputs.target_in_view = false;
        let decision = mt1_decision(&config, &inputs);
        assert_eq!(decision.tag_status, Some("No Targets in View"));
        assert_eq!(rejected(decision), "No Targets in View Rejection");
    }

    #[test]
    fn ambiguity_above_point_nine_rejects() {
        let config = VisionConfig::default();
        let tags = [fiducial(1, 0.1), fiducial(2, 0.91)];
        let decision = mt1_decision(&config, &good(&tags));
        assert_eq!(decision.tag_status, Some(""));
        assert_eq!(rejected(decision), "High Ambiguity Rejection");
    }

    #[test]
    fn rejection_check_orders_field_then_age_then_spin_then_size() {
        let config = VisionConfig::default();
        let inside = Pose2d::from_xy(5.0, 4.0, Rotation2d::ZERO);
        let outside = Pose2d::from_xy(-1.0, 4.0, Rotation2d::ZERO);
        assert_eq!(
            rejection_check(&config, outside, 0.0, 0.0, 99.0, 9.0),
            Some("Out of Field Rejection")
        );
        assert_eq!(
            rejection_check(&config, inside, 0.0, 0.0, 99.0, 9.0),
            Some("Stale Timestamp Rejection")
        );
        assert_eq!(
            rejection_check(&config, inside, 0.0, 10.0, 10.05, 1.6),
            Some("Rotation Speed Rejection")
        );
        assert_eq!(
            rejection_check(&config, inside, 0.025, 10.0, 10.05, 1.59),
            Some("Target Size Rejection")
        );
        assert_eq!(
            rejection_check(&config, inside, 0.026, 10.0, 10.05, 1.59),
            None
        );
        // Exactly 0.1 s old is not stale.
        assert_eq!(rejection_check(&config, inside, 1.0, 10.0, 10.1, 0.0), None);
        assert_eq!(
            rejection_check(&config, inside, 1.0, 10.0, 10.1 + 1e-9, 0.0),
            Some("Stale Timestamp Rejection")
        );
    }

    #[test]
    fn a_tilted_camera_rejects_after_the_shared_gate() {
        let config = VisionConfig::default();
        let mut inputs = good(&[]);
        inputs.pose3d = pose3d(5.0, 4.0, 0.0, 6.0, 0.0);
        assert_eq!(
            rejected(mt1_decision(&config, &inputs)),
            "Roll/Pitch Rejection"
        );
        inputs.pose3d = pose3d(5.0, 4.0, 0.0, 0.0, -5.5);
        assert_eq!(
            rejected(mt1_decision(&config, &inputs)),
            "Roll/Pitch Rejection"
        );
        inputs.pose3d = pose3d(5.0, 4.0, 0.0, 4.9, 4.9);
        assert!(mt1_decision(&config, &inputs).outcome.is_ok());
    }

    #[test]
    fn mt1_tiers_pick_the_confidence_by_tags_size_and_distance() {
        let config = VisionConfig::default();
        let tags = [fiducial(1, 0.1)];

        // Stationary and close.
        let mut inputs = good(&tags);
        inputs.target_size = 4.5;
        let a = accepted(mt1_decision(&config, &inputs));
        assert_eq!(a.status, "Stationary close integration");
        assert_close(0.1, a.xy_stds);
        assert_close(0.1, a.deg_stds);

        // Moving, so the stationary tier does not apply; one tag falls to Close.
        inputs.robot_speed = ChassisSpeeds::new(1.0, 0.0, 0.0);
        let a = accepted(mt1_decision(&config, &inputs));
        assert_eq!(a.status, "Close integration");
        assert_close(0.5, a.xy_stds);
        assert_close(config.k_large_variance, a.deg_stds);

        // Strong multi-tag.
        inputs.multi_tags = true;
        inputs.target_size = 2.5;
        let a = accepted(mt1_decision(&config, &inputs));
        assert_eq!(a.status, "Strong Multi integration");
        assert_close(0.1, a.xy_stds);

        // Multi.
        inputs.target_size = 0.5;
        let a = accepted(mt1_decision(&config, &inputs));
        assert_eq!(a.status, "Multi integration");
        assert_close(0.25, a.xy_stds);
        assert_close(8.0, a.deg_stds);

        // Proximity: one tag, size over 1, within 0.25 m of odometry.
        inputs.multi_tags = false;
        inputs.target_size = 1.5;
        let a = accepted(mt1_decision(&config, &inputs));
        assert_eq!(a.status, "Proximity integration");
        assert_close(1.0, a.xy_stds);

        // Stable: small target, low ambiguity, far from odometry.
        inputs.target_size = 0.5;
        inputs.robot_translation = Translation2d::new(9.0, 4.0);
        let a = accepted(mt1_decision(&config, &inputs));
        assert_eq!(a.status, "Stable integration");
        assert_close(1.5, a.xy_stds);

        // Too small, or too ambiguous for Stable.
        inputs.target_size = 0.029;
        assert_eq!(
            rejected(mt1_decision(&config, &inputs)),
            "Integration Criteria not Met"
        );
        let ambiguous = [fiducial(1, 0.3)];
        let mut inputs = good(&ambiguous);
        inputs.target_size = 0.5;
        inputs.robot_translation = Translation2d::new(9.0, 4.0);
        assert_eq!(
            rejected(mt1_decision(&config, &inputs)),
            "Integration Criteria not Met"
        );
    }

    #[test]
    fn ambiguity_and_spin_widen_the_heading_std_dev() {
        let config = VisionConfig::default();
        let tags = [fiducial(1, 0.6)];
        let mut inputs = good(&tags);
        inputs.multi_tags = true;
        inputs.target_size = 2.5;
        let a = accepted(mt1_decision(&config, &inputs));
        assert_close(15.0, a.deg_stds);

        let tags = [fiducial(1, 0.1)];
        let mut inputs = good(&tags);
        inputs.multi_tags = true;
        inputs.target_size = 2.5;
        inputs.robot_speed = ChassisSpeeds::new(0.0, 0.0, 0.5);
        let a = accepted(mt1_decision(&config, &inputs));
        assert_close(50.0, a.deg_stds);
        assert_close(0.1, a.xy_stds);
    }

    #[test]
    fn force_integrate_clamps_to_near_zero() {
        let config = VisionConfig::default();
        let tags = [fiducial(1, 0.1)];
        let mut inputs = good(&tags);
        inputs.force_integrate_xy = true;
        inputs.target_size = 5.0;
        let a = accepted(mt1_decision(&config, &inputs));
        assert_close(0.01, a.xy_stds);
        assert_close(0.01, a.deg_stds);
    }

    fn mt2(pose: Pose2d) -> Mt2Inputs {
        Mt2Inputs {
            target_in_view: true,
            multi_tags: false,
            target_size: 1.5,
            pose2d: pose,
            robot_speed: ChassisSpeeds::new(0.0, 0.0, 0.0),
            robot_translation: Translation2d::new(5.0, 4.0),
            disabled: false,
            mt1_timestamp: 10.0,
            current_time: 10.05,
        }
    }

    #[test]
    fn mt2_discards_the_heading_and_skips_the_distance_gate_when_disabled() {
        let config = VisionConfig::default();
        let near = Pose2d::from_xy(5.0, 4.0, Rotation2d::ZERO);
        let far = Pose2d::from_xy(9.0, 4.0, Rotation2d::ZERO);

        let a = accepted(mt2_decision(&config, &mt2(near)));
        assert_eq!(a.status, "Proximity integration");
        assert_close(config.k_large_variance, a.deg_stds);

        // Far from odometry and enabled: falls through to Stable.
        let a = accepted(mt2_decision(&config, &mt2(far)));
        assert_eq!(a.status, "Stable integration");
        assert_close(1.5, a.xy_stds);

        // Disabled: distance no longer matters for Proximity.
        let mut inputs = mt2(far);
        inputs.disabled = true;
        let a = accepted(mt2_decision(&config, &inputs));
        assert_eq!(a.status, "Proximity integration");
        assert_close(1.0, a.xy_stds);
    }

    #[test]
    fn mt2_is_gated_on_the_mt1_timestamp() {
        let config = VisionConfig::default();
        let mut inputs = mt2(Pose2d::from_xy(5.0, 4.0, Rotation2d::ZERO));
        inputs.mt1_timestamp = 5.0;
        assert_eq!(
            rejected(mt2_decision(&config, &inputs)),
            "Stale Timestamp Rejection"
        );
        let mut inputs = mt2(Pose2d::from_xy(5.0, 4.0, Rotation2d::ZERO));
        inputs.target_in_view = false;
        let decision = mt2_decision(&config, &inputs);
        assert_eq!(decision.tag_status, Some("No Targets in View"));
        assert_eq!(rejected(decision), "No Targets in View Rejection");
    }

    #[test]
    fn mt2_tiny_targets_fail_the_criteria() {
        let config = VisionConfig::default();
        let mut inputs = mt2(Pose2d::from_xy(5.0, 4.0, Rotation2d::ZERO));
        inputs.target_size = 0.026;
        assert_eq!(
            rejected(mt2_decision(&config, &inputs)),
            "Integration Criteria not Met"
        );
    }

    #[test]
    fn reset_to_vision_takes_mt2_translation_and_mt1_heading() {
        let botpose = pose3d(5.0, 4.0, 0.1, 0.0, 0.0);
        let mega = Pose2d::from_xy(6.0, 3.0, Rotation2d::from_degrees(90.0));
        let pose = reset_pose_to_vision_check(true, botpose, mega).expect("accepted");
        assert_eq!(pose.translation, Translation2d::new(6.0, 3.0));
        assert_close(0.3, pose.rotation.radians());
    }

    #[test]
    fn reset_to_vision_rejections_name_their_reason() {
        let mega = Pose2d::from_xy(6.0, 3.0, Rotation2d::ZERO);
        let check =
            |target, pose, mega| reset_pose_to_vision_check(target, pose, mega).unwrap_err();
        assert_eq!(check(false, pose3d(5.0, 4.0, 0.0, 0.0, 0.0), mega), None);
        assert_eq!(
            check(true, pose3d(-1.0, 4.0, 0.0, 0.0, 0.0), mega),
            Some("Out of field")
        );
        assert_eq!(
            check(true, pose3d(5.0, 4.0, 0.3, 0.0, 0.0), mega),
            Some("Pose in air")
        );
        assert_eq!(
            check(true, pose3d(5.0, 4.0, -0.3, 0.0, 0.0), mega),
            Some("Pose in air")
        );
        assert_eq!(
            check(true, pose3d(5.0, 4.0, 0.0, 0.0, 6.0), mega),
            Some("Pose tilted")
        );
        assert_eq!(
            check(
                true,
                pose3d(5.0, 4.0, 0.0, 0.0, 0.0),
                Pose2d::from_xy(5.0, 99.0, Rotation2d::ZERO)
            ),
            Some("MegaPose out of field")
        );
    }

    #[test]
    fn the_best_camera_has_the_top_score_and_back_wins_when_nothing_is_seen() {
        assert_eq!(best_limelight_index(&[0.0, 0.0, 0.0]), 0);
        assert_eq!(best_limelight_index(&[1.0, 3.0, 2.0]), 1);
        assert_eq!(best_limelight_index(&[1.0, 2.0, 2.0]), 1);
        assert_eq!(best_limelight_index(&[0.0, 0.0, 0.5]), 2);
    }

    #[test]
    fn scoring_tags_follow_the_alliance() {
        assert!(is_scoring_tag(Alliance::Blue, 18));
        assert!(is_scoring_tag(Alliance::Blue, 27));
        assert!(!is_scoring_tag(Alliance::Blue, 2));
        assert!(is_scoring_tag(Alliance::Red, 12));
        assert!(!is_scoring_tag(Alliance::Red, 22));
        assert!(!is_scoring_tag(Alliance::Red, 0));
    }

    #[test]
    fn the_default_config_names_the_three_cameras() {
        let config = VisionConfig::default();
        assert_eq!(config.name, "Vision");
        assert_eq!(config.back_config.name, "limelight-back");
        assert_eq!(config.left_config.yaw, 90.0);
        assert_eq!(config.right_config.yaw, -90.0);
        assert_eq!(config.vision_std_matrix, [0.5, 0.5, 0.2]);
    }
}
