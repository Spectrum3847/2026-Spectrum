//! Wraps one Limelight camera for AprilTag pose estimation and basic targeting.
//!
//! Every getter is safe on a detached camera (`is_attached` is false) and returns zero,
//! false, or an empty value.
//!
//! Getters share one NetworkTables sample per robot loop. The first getter to need a topic
//! reads it, and every later getter in that loop reuses the sample, so pose, tag count,
//! fiducials, and timestamp all describe the same camera frame. The owning subsystem calls
//! [`invalidate`](Limelight::invalidate) once per loop before any getter. Returned estimates
//! belong to that sample and are shared, so they are immutable.
//!
//! MegaTag1 solves pose from the tags alone. MegaTag2 fuses in the heading the owning
//! subsystem passes to [`set_robot_orientation`](Limelight::set_robot_orientation).
//!
//! The cache is plain `Cell`s and `RefCell`s: loop thread only, as in Java.

use super::limelight_helpers::{LimelightHelpers, PoseEstimate, RawFiducial, to_pose3d};
use std::cell::{Cell, RefCell};
use std::sync::Arc;
use wpilib::dashboard::smart_dashboard;
use wpimath::geometry::{Pose2d, Pose3d};

/// Network-table name and mounting position for one camera.
#[derive(Clone, Debug)]
pub struct LimelightConfig {
    /// Must match the name given in the LL dashboard.
    pub name: String,
    /// Whether this camera is physically connected to the robot.
    pub attached: bool,
    /// Mount offsets from the robot center, in metres. Forward is positive toward the front.
    pub forward: f64,
    pub right: f64,
    pub up: f64,
    /// Camera orientation in the robot frame, in degrees.
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl LimelightConfig {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            attached: true,
            forward: 0.0,
            right: 0.0,
            up: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }

    pub fn set_name(mut self, name: &str) -> Self {
        self.name = name.to_owned();
        self
    }

    pub fn set_attached(mut self, attached: bool) -> Self {
        self.attached = attached;
        self
    }

    /// Sets the camera's mounting offsets: metres ahead of, right of, and above the robot
    /// center.
    pub fn with_translation(mut self, forward: f64, right: f64, up: f64) -> Self {
        self.forward = forward;
        self.right = right;
        self.up = up;
        self
    }

    /// Sets the camera's orientation in degrees, expressed in the robot frame.
    pub fn with_rotation(mut self, roll: f64, pitch: f64, yaw: f64) -> Self {
        self.roll = roll;
        self.pitch = pitch;
        self.yaw = yaw;
        self
    }
}

/// A raw botpose array with its NetworkTables timestamp in microseconds.
struct Sample {
    value: Vec<f64>,
    timestamp: i64,
}

pub struct Limelight {
    helpers: LimelightHelpers,
    config: LimelightConfig,

    /// Whether the pose estimator is currently fusing a measurement from this camera.
    is_integrating: Cell<bool>,
    log_status: RefCell<String>,
    tag_status: RefCell<String>,

    // Per-loop NetworkTables snapshot, filled lazily on first use and cleared by
    // `invalidate`.
    /// Raw `botpose_wpiblue` sample, empty until the first MT1 read.
    mt1_sample: RefCell<Option<Arc<Sample>>>,
    mt1_estimate: RefCell<Option<Arc<PoseEstimate>>>,
    mt1_pose3d: Cell<Option<Pose3d>>,
    /// MegaTag2 estimate from `botpose_orb_wpiblue`, empty until the first MT2 read.
    mt2_estimate: RefCell<Option<Arc<PoseEstimate>>>,
    tv_cache: Cell<Option<bool>>,
    ta_cache: Cell<Option<f64>>,

    /// Written by the owning subsystem as it accepts or rejects each estimate.
    integrated_this_loop: Cell<bool>,

    /// Seconds from the last estimate's capture time to now, or NaN if this camera produced
    /// none this loop. A large value means the camera's timestamps are not trustworthy.
    last_estimate_age_seconds: Cell<f64>,
}

impl Limelight {
    pub fn from_config(config: LimelightConfig) -> Self {
        Self::with_helpers(LimelightHelpers::default(), config)
    }

    /// [`from_config`](Self::from_config) on a chosen NetworkTables backend.
    pub fn with_helpers(helpers: LimelightHelpers, config: LimelightConfig) -> Self {
        Self {
            helpers,
            config,
            is_integrating: Cell::new(false),
            log_status: RefCell::new(String::new()),
            tag_status: RefCell::new(String::new()),
            mt1_sample: RefCell::new(None),
            mt1_estimate: RefCell::new(None),
            mt1_pose3d: Cell::new(None),
            mt2_estimate: RefCell::new(None),
            tv_cache: Cell::new(None),
            ta_cache: Cell::new(None),
            integrated_this_loop: Cell::new(false),
            last_estimate_age_seconds: Cell::new(f64::NAN),
        }
    }

    pub fn new(name: &str) -> Self {
        Self::from_config(LimelightConfig::new(name))
    }

    pub fn with_attached(name: &str, attached: bool) -> Self {
        Self::from_config(LimelightConfig::new(name).set_attached(attached))
    }

    /// `pipeline` is the index to activate; the indexes come from the robot's vision
    /// configuration.
    pub fn with_pipeline(name: &str, pipeline: i32) -> Self {
        let limelight = Self::new(name);
        limelight.set_limelight_pipeline(pipeline);
        limelight
    }

    // Java: `name` is ignored here, the config's name is used.
    pub fn with_pipeline_and_config(_name: &str, pipeline: i32, config: LimelightConfig) -> Self {
        let limelight = Self::from_config(config);
        limelight.set_limelight_pipeline(pipeline);
        limelight
    }

    /// Drops the per-loop snapshot so the next getter re-reads the camera.
    ///
    /// The owning subsystem calls this once per loop before any getter. Safe on a detached
    /// camera.
    pub fn invalidate(&self) {
        *self.mt1_sample.borrow_mut() = None;
        *self.mt1_estimate.borrow_mut() = None;
        self.mt1_pose3d.set(None);
        *self.mt2_estimate.borrow_mut() = None;
        self.tv_cache.set(None);
        self.ta_cache.set(None);
        self.integrated_this_loop.set(false);
        self.last_estimate_age_seconds.set(f64::NAN);
    }

    /// Raw MegaTag1 sample for this loop. Caller must have checked `is_attached`.
    fn mt1_sample(&self) -> Arc<Sample> {
        self.mt1_sample
            .borrow_mut()
            .get_or_insert_with(|| {
                let (value, timestamp) = self
                    .helpers
                    .get_limelight_double_array_atomic(&self.config.name, "botpose_wpiblue");
                Arc::new(Sample { value, timestamp })
            })
            .clone()
    }

    /// MegaTag1 estimate for this loop. Caller must have checked `is_attached`.
    fn mt1_estimate(&self) -> Arc<PoseEstimate> {
        if let Some(estimate) = self.mt1_estimate.borrow().as_ref() {
            return estimate.clone();
        }
        let sample = self.mt1_sample();
        let estimate = Arc::new(super::limelight_helpers::parse_pose_estimate(
            &sample.value,
            sample.timestamp,
            false,
        ));
        *self.mt1_estimate.borrow_mut() = Some(estimate.clone());
        estimate
    }

    /// MegaTag1 Pose3d for this loop. Caller must have checked `is_attached`.
    fn mt1_pose3d(&self) -> Pose3d {
        if let Some(pose) = self.mt1_pose3d.get() {
            return pose;
        }
        let pose = to_pose3d(&self.mt1_sample().value);
        self.mt1_pose3d.set(Some(pose));
        pose
    }

    /// MegaTag2 estimate for this loop. Caller must have checked `is_attached`.
    fn mt2_estimate(&self) -> Arc<PoseEstimate> {
        self.mt2_estimate
            .borrow_mut()
            .get_or_insert_with(|| {
                Arc::new(
                    self.helpers
                        .get_bot_pose_estimate_wpi_blue_mega_tag2(&self.config.name),
                )
            })
            .clone()
    }

    pub fn get_name(&self) -> &str {
        &self.config.name
    }

    /// Same as [`get_name`](Self::get_name).
    pub fn get_camera_name(&self) -> &str {
        &self.config.name
    }

    pub fn is_attached(&self) -> bool {
        self.config.attached
    }

    pub fn is_integrating(&self) -> bool {
        self.is_integrating.get()
    }

    pub fn set_integrating(&self, integrating: bool) {
        self.is_integrating.set(integrating);
    }

    pub fn get_log_status(&self) -> String {
        self.log_status.borrow().clone()
    }

    pub fn set_log_status(&self, status: &str) {
        status.clone_into(&mut self.log_status.borrow_mut());
    }

    pub fn get_tag_status(&self) -> String {
        self.tag_status.borrow().clone()
    }

    pub fn set_tag_status(&self, status: &str) {
        status.clone_into(&mut self.tag_status.borrow_mut());
    }

    pub fn is_integrated_this_loop(&self) -> bool {
        self.integrated_this_loop.get()
    }

    pub fn set_integrated_this_loop(&self, integrated: bool) {
        self.integrated_this_loop.set(integrated);
    }

    pub fn get_last_estimate_age_seconds(&self) -> f64 {
        self.last_estimate_age_seconds.get()
    }

    pub fn set_last_estimate_age_seconds(&self, seconds: f64) {
        self.last_estimate_age_seconds.set(seconds);
    }

    /// Horizontal offset from the crosshair to the target, in degrees. LL1 spans -27 to 27,
    /// LL2 -29.8 to 29.8.
    pub fn get_horizontal_offset(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        self.helpers.get_tx(&self.config.name)
    }

    /// Vertical offset from the crosshair to the target, in degrees. LL1 spans -20.5 to
    /// 20.5, LL2 -24.85 to 24.85.
    pub fn get_vertical_offset(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        self.helpers.get_ty(&self.config.name)
    }

    pub fn target_in_view(&self) -> bool {
        if !self.is_attached() {
            return false;
        }
        match self.tv_cache.get() {
            Some(tv) => tv,
            None => {
                let tv = self.helpers.get_tv(&self.config.name);
                self.tv_cache.set(Some(tv));
                tv
            }
        }
    }

    pub fn multiple_tags_in_view(&self) -> bool {
        self.get_tag_count_in_view() > 1.0
    }

    pub fn get_tag_count_in_view(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        f64::from(self.mt1_estimate().tag_count)
    }

    pub fn get_closest_tag_id(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        self.helpers.get_fiducial_id(&self.config.name)
    }

    /// Area of the primary target as a percentage of the camera image (0-100).
    pub fn get_target_size(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        match self.ta_cache.get() {
            Some(ta) => ta,
            None => {
                let ta = self.helpers.get_ta(&self.config.name);
                self.ta_cache.set(Some(ta));
                ta
            }
        }
    }

    pub fn get_mega_tag1_pose3d(&self) -> Pose3d {
        if !self.is_attached() {
            return Pose3d::ZERO;
        }
        self.mt1_pose3d()
    }

    pub fn get_mega_tag2_pose2d(&self) -> Pose2d {
        if !self.is_attached() {
            return Pose2d::ZERO;
        }
        self.mt2_estimate().pose
    }

    /// Full MegaTag1 estimate, including timestamp, tag count, and raw fiducials, in the
    /// WPILib Blue origin frame.
    pub fn get_mega_tag1_pose_estimate(&self) -> Arc<PoseEstimate> {
        if !self.is_attached() {
            return Arc::new(PoseEstimate::default());
        }
        self.mt1_estimate()
    }

    /// Full heading-fused MegaTag2 estimate, including timestamp and tag count, in the
    /// WPILib Blue origin frame.
    pub fn get_mega_tag2_pose_estimate(&self) -> Arc<PoseEstimate> {
        if !self.is_attached() {
            return Arc::new(PoseEstimate::default());
        }
        self.mt2_estimate()
    }

    /// Needs more than one tag and a target area above 0.1 percent of the frame.
    pub fn has_accurate_pose(&self) -> bool {
        self.multiple_tags_in_view() && self.get_target_size() > 0.1
    }

    /// Distance to the closest tag, from the X and Z axes of the camera-in-tag-space pose.
    pub fn get_distance_to_tag_from_camera(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        let camera_in_target_space = self
            .helpers
            .get_camera_pose3d_target_space(&self.config.name);
        camera_in_target_space.x().hypot(camera_in_target_space.z())
    }

    /// Raw fiducials from the MegaTag1 estimate, or an empty list when there is no estimate.
    pub fn get_raw_fiducial(&self) -> Vec<RawFiducial> {
        if !self.is_attached() {
            return Vec::new();
        }
        // Shares the per-loop MT1 sample, so this costs no extra NetworkTables read.
        self.mt1_estimate().raw_fiducials.clone()
    }

    /// Timestamp of the MegaTag1 pose estimate, in seconds.
    pub fn get_mega_tag1_pose_timestamp(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        self.mt1_estimate().timestamp_seconds
    }

    /// Timestamp of the MegaTag2 pose estimate, in seconds.
    pub fn get_mega_tag2_pose_timestamp(&self) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        self.mt2_estimate().timestamp_seconds
    }

    /// Distance to a target at a known height, from the camera's height above the floor and
    /// the camera's pitch plus the reported vertical offset. `target_height` is in metres.
    pub fn get_distance_to_target(&self, target_height: f64) -> f64 {
        if !self.is_attached() {
            return 0.0;
        }
        (target_height - self.config.up)
            / (self.config.pitch + self.get_vertical_offset())
                .to_radians()
                .tan()
    }

    /// Marks this camera as integrating and records why.
    pub fn send_valid_status(&self, message: &str) {
        self.is_integrating.set(true);
        self.set_log_status(message);
    }

    /// Marks this camera as not integrating and records why.
    pub fn send_invalid_status(&self, message: &str) {
        self.is_integrating.set(false);
        self.set_log_status(message);
    }

    /// Activates a pipeline by index; the indexes come from the robot's vision
    /// configuration.
    pub fn set_limelight_pipeline(&self, pipeline_index: i32) {
        if !self.is_attached() {
            return;
        }
        self.helpers
            .set_pipeline_index(&self.config.name, pipeline_index);
    }

    /// Sets the robot heading the camera's internal IMU fuses, in degrees.
    ///
    /// Does not flush NetworkTables. The owning subsystem flushes once after all per-loop
    /// writes.
    pub fn set_robot_orientation(&self, degrees: f64) {
        self.set_robot_orientation_with_rate(degrees, 0.0);
    }

    pub fn update_camera_pose(&self, pose: Pose3d) {
        if !self.is_attached() {
            return;
        }
        let rotation = pose.rotation;
        self.helpers.set_camera_pose_robot_space(
            &self.config.name,
            pose.x(),
            -pose.y(),
            pose.z(),
            rotation.x().to_degrees(),
            rotation.y().to_degrees(),
            rotation.z().to_degrees(),
        );
    }

    /// Writes the configured mount offsets ([`LimelightConfig::with_translation`] and
    /// [`LimelightConfig::with_rotation`]) to the camera, so the code is the source of truth
    /// instead of the values typed into the web UI. The six values cross in the order and
    /// sign the camera wants, so a config value can be read straight against the matching
    /// web UI box.
    ///
    /// A camera keeps its own copy in flash and falls back to it on boot, so this has to run
    /// again periodically: one that reboots mid-match reverts to whatever was last typed
    /// into it. Does not flush NetworkTables.
    pub fn push_configured_camera_pose(&self) {
        if !self.is_attached() {
            return;
        }
        self.helpers.set_camera_pose_robot_space(
            &self.config.name,
            self.config.forward,
            self.config.right,
            self.config.up,
            self.config.roll,
            self.config.pitch,
            self.config.yaw,
        );
    }

    /// Sets the heading and yaw rate the camera's internal IMU fuses, for MegaTag2.
    /// `degrees` is the robot heading, positive counter-clockwise, and `angular_rate` the
    /// yaw rate in degrees per second.
    ///
    /// Does not flush NetworkTables. The owning subsystem flushes once after all per-loop
    /// writes.
    pub fn set_robot_orientation_with_rate(&self, degrees: f64, angular_rate: f64) {
        if !self.is_attached() {
            return;
        }
        self.helpers.set_robot_orientation_no_flush(
            &self.config.name,
            degrees,
            angular_rate,
            0.0,
            0.0,
            0.0,
            0.0,
        );
    }

    /// IMU mode index; valid values are in the Limelight documentation.
    pub fn set_imu_mode(&self, mode: i32) {
        if !self.is_attached() {
            return;
        }
        self.helpers.set_imu_mode(&self.config.name, mode);
    }

    /// X offset of the primary target in robot space, in metres along the robot's
    /// left/right axis. Returns -99999 when no target is in view.
    pub fn get_tag_tx(&self) -> f64 {
        if !self.target_in_view() {
            return -99999.0;
        }
        self.helpers
            .get_target_pose3d_robot_space(&self.config.name)
            .x()
    }

    /// Area of the primary target as a percentage of the camera image. Returns -99999 when
    /// no target is in view.
    pub fn get_tag_ta(&self) -> f64 {
        if !self.target_in_view() {
            return -99999.0;
        }
        self.get_target_size()
    }

    /// Z-axis rotation of the primary target in robot space, in degrees. Returns -99999
    /// when no target is in view.
    pub fn get_tag_rotation_degrees(&self) -> f64 {
        if !self.target_in_view() {
            return -99999.0;
        }
        let rotation_radians = self
            .helpers
            .get_target_pose3d_robot_space(&self.config.name)
            .rotation
            .z();
        rotation_radians.to_degrees()
    }

    pub fn set_led_mode(&self, enabled: bool) {
        if !self.is_attached() {
            return;
        }
        if enabled {
            self.helpers.set_led_mode_force_on(&self.config.name);
        } else {
            self.helpers.set_led_mode_force_off(&self.config.name);
        }
    }

    pub fn blink_leds(&self) {
        if !self.is_attached() {
            return;
        }
        self.helpers.set_led_mode_force_blink(&self.config.name);
    }

    /// True when the raw MT1 sample is long enough to hold a pose.
    pub fn is_camera_connected(&self) -> bool {
        if !self.is_attached() {
            return false;
        }
        self.mt1_sample().value.len() >= 6
    }

    pub fn print_debug(&self) {
        if !self.is_attached() {
            return;
        }
        let bot_pose3d = self.get_mega_tag1_pose3d();
        let rotation = bot_pose3d.rotation;
        smart_dashboard::put_string("LimelightX", &format_decimal(bot_pose3d.x()));
        smart_dashboard::put_string("LimelightY", &format_decimal(bot_pose3d.y()));
        smart_dashboard::put_string("LimelightZ", &format_decimal(bot_pose3d.z()));
        smart_dashboard::put_string("LimelightRoll", &format_decimal(rotation.x().to_degrees()));
        smart_dashboard::put_string("LimelightPitch", &format_decimal(rotation.y().to_degrees()));
        smart_dashboard::put_string("LimelightYaw", &format_decimal(rotation.z().to_degrees()));
    }
}

/// Java's default `DecimalFormat` in the US locale: pattern `#,##0.###`, up to three
/// decimals, half-even rounding, comma grouping.
fn format_decimal(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value < 0.0 { "-∞" } else { "∞" }.to_owned();
    }
    let fixed = format!("{value:.3}");
    let (sign, digits) = fixed
        .strip_prefix('-')
        .map_or(("", fixed.as_str()), |d| ("-", d));
    let (whole, fraction) = digits.split_once('.').unwrap_or((digits, ""));
    let mut grouped = String::new();
    for (i, c) in whole.chars().enumerate() {
        if i > 0 && (whole.len() - i) % 3 == 0 {
            grouped.push(',');
        }
        grouped.push(c);
    }
    let fraction = fraction.trim_end_matches('0');
    if fraction.is_empty() {
        format!("{sign}{grouped}")
    } else {
        format!("{sign}{grouped}.{fraction}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectrum_lib::vision::limelight_helpers::test_support::FakeNt;

    const EPS: f64 = 1e-9;

    /// A botpose array: `[x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgDist,
    /// avgArea]`, then 7 values per fiducial.
    fn botpose(x: f64, y: f64, yaw_deg: f64, latency_ms: f64, fiducials: &[[f64; 7]]) -> Vec<f64> {
        let mut out = vec![0.0; 11 + 7 * fiducials.len()];
        out[0] = x;
        out[1] = y;
        out[5] = yaw_deg;
        out[6] = latency_ms;
        out[7] = fiducials.len() as f64;
        for (i, fiducial) in fiducials.iter().enumerate() {
            out[11 + 7 * i..18 + 7 * i].copy_from_slice(fiducial);
        }
        out
    }

    /// One fiducial: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity.
    fn fiducial(id: i32, ambiguity: f64) -> [f64; 7] {
        [f64::from(id), 0.1, 0.2, 1.5, 2.0, 2.5, ambiguity]
    }

    const TABLE: &str = "limelight-test";

    fn setup(attached: bool) -> (Limelight, Arc<FakeNt>) {
        let nt = Arc::new(FakeNt::default());
        let config = LimelightConfig::new(TABLE).set_attached(attached);
        let limelight = Limelight::with_helpers(LimelightHelpers::with_nt(nt.clone()), config);
        (limelight, nt)
    }

    #[test]
    fn mt1_getters_hold_one_sample_until_invalidate() {
        let (limelight, nt) = setup(true);
        nt.publish_array(
            TABLE,
            "botpose_wpiblue",
            &botpose(1.0, 2.0, 30.0, 20.0, &[fiducial(7, 0.1)]),
            1_000_000,
        );
        nt.publish_double(TABLE, "tv", 1.0);
        nt.publish_double(TABLE, "ta", 3.5);

        assert!((limelight.get_mega_tag1_pose3d().x() - 1.0).abs() < EPS);
        assert_eq!(limelight.get_tag_count_in_view(), 1.0);
        assert_eq!(limelight.get_raw_fiducial()[0].id, 7);
        assert!((limelight.get_mega_tag1_pose_timestamp() - (1.0 - 0.020)).abs() < EPS);
        assert!(limelight.target_in_view());
        assert!((limelight.get_target_size() - 3.5).abs() < EPS);
        assert!(limelight.is_camera_connected());

        // A different frame changes nothing until the snapshot is cleared.
        nt.publish_array(
            TABLE,
            "botpose_wpiblue",
            &botpose(5.0, 6.0, 90.0, 10.0, &[fiducial(3, 0.2), fiducial(4, 0.3)]),
            2_000_000,
        );
        nt.publish_double(TABLE, "tv", 0.0);
        nt.publish_double(TABLE, "ta", 0.5);

        assert!((limelight.get_mega_tag1_pose3d().x() - 1.0).abs() < EPS);
        assert_eq!(limelight.get_tag_count_in_view(), 1.0);
        assert_eq!(limelight.get_raw_fiducial()[0].id, 7);
        assert!((limelight.get_mega_tag1_pose_timestamp() - (1.0 - 0.020)).abs() < EPS);
        assert!(limelight.target_in_view());
        assert!((limelight.get_target_size() - 3.5).abs() < EPS);
        assert_eq!(nt.array_reads.load(std::sync::atomic::Ordering::SeqCst), 1);

        limelight.invalidate();

        assert!((limelight.get_mega_tag1_pose3d().x() - 5.0).abs() < EPS);
        assert_eq!(limelight.get_tag_count_in_view(), 2.0);
        assert_eq!(limelight.get_raw_fiducial()[0].id, 3);
        assert_eq!(limelight.get_raw_fiducial()[1].id, 4);
        assert!((limelight.get_mega_tag1_pose_timestamp() - (2.0 - 0.010)).abs() < EPS);
        assert!(!limelight.target_in_view());
        assert!((limelight.get_target_size() - 0.5).abs() < EPS);
    }

    #[test]
    fn all_mt1_getters_describe_the_same_sample_even_if_the_topic_changes_mid_loop() {
        let (limelight, nt) = setup(true);
        nt.publish_array(
            TABLE,
            "botpose_wpiblue",
            &botpose(1.0, 2.0, 30.0, 20.0, &[fiducial(7, 0.1)]),
            1_000_000,
        );
        let pose3d = limelight.get_mega_tag1_pose3d(); // first read snapshots the array

        nt.publish_array(
            TABLE,
            "botpose_wpiblue",
            &botpose(5.0, 6.0, 90.0, 10.0, &[fiducial(3, 0.2), fiducial(4, 0.3)]),
            2_000_000,
        );

        let estimate = limelight.get_mega_tag1_pose_estimate();
        assert_eq!(
            limelight.get_raw_fiducial().len(),
            limelight.get_tag_count_in_view() as usize
        );
        assert_eq!(pose3d.to_pose2d(), estimate.pose);
        assert_eq!(estimate.tag_count, 1);
    }

    #[test]
    fn mt2_cache_is_independent_of_the_mt1_cache() {
        let (limelight, nt) = setup(true);
        nt.publish_array(
            TABLE,
            "botpose_wpiblue",
            &botpose(1.0, 2.0, 30.0, 20.0, &[fiducial(7, 0.1)]),
            1_000_000,
        );
        nt.publish_array(
            TABLE,
            "botpose_orb_wpiblue",
            &botpose(1.1, 2.1, 30.0, 15.0, &[fiducial(7, 0.1)]),
            1_000_000,
        );

        assert!((limelight.get_mega_tag2_pose2d().x() - 1.1).abs() < EPS);
        assert!((limelight.get_mega_tag2_pose_timestamp() - (1.0 - 0.015)).abs() < EPS);
        assert!(limelight.get_mega_tag2_pose_estimate().is_mega_tag2);

        nt.publish_array(
            TABLE,
            "botpose_orb_wpiblue",
            &botpose(9.0, 9.0, 0.0, 5.0, &[fiducial(1, 0.1)]),
            3_000_000,
        );
        assert!((limelight.get_mega_tag2_pose2d().x() - 1.1).abs() < EPS);

        limelight.invalidate();
        assert!((limelight.get_mega_tag2_pose2d().x() - 9.0).abs() < EPS);
        assert!((limelight.get_mega_tag2_pose_timestamp() - (3.0 - 0.005)).abs() < EPS);
        // MT1 was untouched by the MT2 republish.
        assert!((limelight.get_mega_tag1_pose3d().x() - 1.0).abs() < EPS);
        assert!(!limelight.get_mega_tag1_pose_estimate().is_mega_tag2);
    }

    #[test]
    fn empty_botpose_arrays_yield_the_documented_defaults() {
        let (limelight, nt) = setup(true);
        nt.publish_array(TABLE, "botpose_wpiblue", &[], 0);
        nt.publish_array(TABLE, "botpose_orb_wpiblue", &[], 0);

        assert_eq!(limelight.get_mega_tag1_pose3d(), Pose3d::ZERO);
        assert_eq!(limelight.get_mega_tag2_pose2d(), Pose2d::ZERO);
        assert_eq!(limelight.get_tag_count_in_view(), 0.0);
        assert!(limelight.get_raw_fiducial().is_empty());
        assert_eq!(limelight.get_mega_tag1_pose_timestamp(), 0.0);
        assert_eq!(limelight.get_mega_tag2_pose_timestamp(), 0.0);
        assert!(!limelight.target_in_view());
        assert_eq!(limelight.get_target_size(), 0.0);
        assert!(!limelight.is_camera_connected());
        assert_eq!(
            *limelight.get_mega_tag1_pose_estimate(),
            PoseEstimate::default()
        );
    }

    #[test]
    fn detached_camera_returns_defaults_regardless_of_published_data() {
        let (limelight, nt) = setup(false);
        nt.publish_array(
            TABLE,
            "botpose_wpiblue",
            &botpose(1.0, 2.0, 30.0, 20.0, &[fiducial(7, 0.1)]),
            1_000_000,
        );
        nt.publish_double(TABLE, "tv", 1.0);
        nt.publish_double(TABLE, "ta", 5.0);

        limelight.invalidate(); // must not panic
        assert_eq!(limelight.get_mega_tag1_pose3d(), Pose3d::ZERO);
        assert_eq!(limelight.get_mega_tag2_pose2d(), Pose2d::ZERO);
        assert_eq!(limelight.get_tag_count_in_view(), 0.0);
        assert!(limelight.get_raw_fiducial().is_empty());
        assert_eq!(limelight.get_mega_tag1_pose_timestamp(), 0.0);
        assert_eq!(limelight.get_mega_tag2_pose_timestamp(), 0.0);
        assert!(!limelight.target_in_view());
        assert_eq!(limelight.get_target_size(), 0.0);
        assert!(!limelight.is_camera_connected());
        assert_eq!(limelight.get_horizontal_offset(), 0.0);
        assert_eq!(limelight.get_vertical_offset(), 0.0);
        assert_eq!(limelight.get_closest_tag_id(), 0.0);
        assert_eq!(limelight.get_distance_to_tag_from_camera(), 0.0);
        assert_eq!(limelight.get_distance_to_target(2.0), 0.0);

        // Writes are dropped too.
        limelight.set_limelight_pipeline(3);
        limelight.set_robot_orientation(10.0);
        limelight.set_imu_mode(1);
        limelight.set_led_mode(true);
        limelight.blink_leds();
        limelight.push_configured_camera_pose();
        limelight.update_camera_pose(Pose3d::ZERO);
        assert_eq!(nt.total_reads(), 0);
        for (entry, ..) in [("pipeline",), ("ledMode",), ("imumode_set",)] {
            assert_eq!(nt.double(TABLE, entry), None, "{entry}");
        }
        for entry in ["robot_orientation_set", "camerapose_robotspace_set"] {
            assert_eq!(nt.array(TABLE, entry), None, "{entry}");
        }
    }

    #[test]
    fn set_robot_orientation_writes_the_orientation() {
        let (limelight, nt) = setup(true);
        limelight.set_robot_orientation(90.0);
        assert_eq!(
            nt.array(TABLE, "robot_orientation_set").unwrap(),
            [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
        limelight.set_robot_orientation_with_rate(45.0, -12.5);
        assert_eq!(
            nt.array(TABLE, "robot_orientation_set").unwrap(),
            [45.0, -12.5, 0.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn simple_writes_use_the_limelight_topics() {
        let (limelight, nt) = setup(true);
        limelight.set_limelight_pipeline(4);
        assert_eq!(nt.double(TABLE, "pipeline"), Some(4.0));
        limelight.set_imu_mode(2);
        assert_eq!(nt.double(TABLE, "imumode_set"), Some(2.0));
        limelight.set_led_mode(true);
        assert_eq!(nt.double(TABLE, "ledMode"), Some(3.0));
        limelight.set_led_mode(false);
        assert_eq!(nt.double(TABLE, "ledMode"), Some(1.0));
        limelight.blink_leds();
        assert_eq!(nt.double(TABLE, "ledMode"), Some(2.0));
    }

    #[test]
    fn pipeline_write_goes_to_the_config_name() {
        let nt = Arc::new(FakeNt::default());
        let helpers = LimelightHelpers::with_nt(nt.clone());
        let config = LimelightConfig::new("config-name");
        let limelight = Limelight::with_helpers(helpers, config);
        limelight.set_limelight_pipeline(2);
        assert_eq!(nt.double("config-name", "pipeline"), Some(2.0));
        assert_eq!(limelight.get_name(), "config-name");
        assert_eq!(limelight.get_camera_name(), "config-name");
    }

    #[test]
    fn camera_pose_writes_keep_the_web_ui_order_and_negate_y_for_a_pose() {
        let nt = Arc::new(FakeNt::default());
        let config = LimelightConfig::new(TABLE)
            .with_translation(0.1, 0.2, 0.3)
            .with_rotation(4.0, 5.0, 6.0);
        let limelight = Limelight::with_helpers(LimelightHelpers::with_nt(nt.clone()), config);
        limelight.push_configured_camera_pose();
        assert_eq!(
            nt.array(TABLE, "camerapose_robotspace_set").unwrap(),
            [0.1, 0.2, 0.3, 4.0, 5.0, 6.0]
        );

        let pose = Pose3d::new(
            wpimath::geometry::Translation3d::new(1.0, 2.0, 3.0),
            wpimath::geometry::Rotation3d::from_euler(
                10.0_f64.to_radians(),
                20.0_f64.to_radians(),
                30.0_f64.to_radians(),
            ),
        );
        limelight.update_camera_pose(pose);
        let written = nt.array(TABLE, "camerapose_robotspace_set").unwrap();
        assert_eq!(written[..3], [1.0, -2.0, 3.0]);
        for (got, want) in written[3..].iter().zip([10.0, 20.0, 30.0]) {
            assert!((got - want).abs() < 1e-9, "{got} vs {want}");
        }
    }

    #[test]
    fn targeting_getters_read_their_topics() {
        let (limelight, nt) = setup(true);
        nt.publish_double(TABLE, "tx", 4.5);
        nt.publish_double(TABLE, "ty", -2.0);
        nt.publish_double(TABLE, "tid", 18.0);
        assert_eq!(limelight.get_horizontal_offset(), 4.5);
        assert_eq!(limelight.get_vertical_offset(), -2.0);
        assert_eq!(limelight.get_closest_tag_id(), 18.0);
    }

    #[test]
    fn distance_to_target_uses_camera_height_and_pitch() {
        let nt = Arc::new(FakeNt::default());
        let config = LimelightConfig::new(TABLE)
            .with_translation(0.0, 0.0, 0.5)
            .with_rotation(0.0, 20.0, 0.0);
        let limelight = Limelight::with_helpers(LimelightHelpers::with_nt(nt.clone()), config);
        nt.publish_double(TABLE, "ty", 5.0);
        let expected = (2.0 - 0.5) / 25.0_f64.to_radians().tan();
        assert!((limelight.get_distance_to_target(2.0) - expected).abs() < EPS);
    }

    #[test]
    fn distance_to_tag_is_the_xz_hypotenuse() {
        let (limelight, nt) = setup(true);
        nt.publish_double(TABLE, "tv", 1.0);
        nt.publish_array(
            TABLE,
            "camerapose_targetspace",
            &[3.0, 100.0, 4.0, 0.0, 0.0, 0.0],
            0,
        );
        assert!((limelight.get_distance_to_tag_from_camera() - 5.0).abs() < EPS);
    }

    #[test]
    fn tag_values_report_the_sentinel_without_a_target() {
        let (limelight, nt) = setup(true);
        assert_eq!(limelight.get_tag_tx(), -99999.0);
        assert_eq!(limelight.get_tag_ta(), -99999.0);
        assert_eq!(limelight.get_tag_rotation_degrees(), -99999.0);

        limelight.invalidate();
        nt.publish_double(TABLE, "tv", 1.0);
        nt.publish_double(TABLE, "ta", 2.5);
        nt.publish_array(
            TABLE,
            "targetpose_robotspace",
            &[0.75, 0.0, 0.0, 0.0, 0.0, 90.0],
            0,
        );
        assert_eq!(limelight.get_tag_tx(), 0.75);
        assert_eq!(limelight.get_tag_ta(), 2.5);
        assert!((limelight.get_tag_rotation_degrees() - 90.0).abs() < 1e-9);
    }

    #[test]
    fn accurate_pose_needs_two_tags_and_area() {
        let (limelight, nt) = setup(true);
        let one = botpose(1.0, 2.0, 0.0, 0.0, &[fiducial(1, 0.0)]);
        let two = botpose(1.0, 2.0, 0.0, 0.0, &[fiducial(1, 0.0), fiducial(2, 0.0)]);
        nt.publish_double(TABLE, "ta", 1.0);
        nt.publish_array(TABLE, "botpose_wpiblue", &one, 0);
        assert!(!limelight.multiple_tags_in_view());
        assert!(!limelight.has_accurate_pose());

        limelight.invalidate();
        nt.publish_array(TABLE, "botpose_wpiblue", &two, 0);
        assert!(limelight.multiple_tags_in_view());
        assert!(limelight.has_accurate_pose());

        limelight.invalidate();
        nt.publish_double(TABLE, "ta", 0.1);
        assert!(!limelight.has_accurate_pose(), "area must exceed 0.1");
    }

    #[test]
    fn camera_connected_needs_six_values() {
        let (limelight, nt) = setup(true);
        nt.publish_array(TABLE, "botpose_wpiblue", &[0.0; 5], 0);
        assert!(!limelight.is_camera_connected());
        limelight.invalidate();
        nt.publish_array(TABLE, "botpose_wpiblue", &[0.0; 6], 0);
        assert!(limelight.is_camera_connected());
    }

    #[test]
    fn status_and_loop_flags_and_invalidate() {
        let (limelight, _) = setup(true);
        assert!(!limelight.is_integrating());
        limelight.send_valid_status("good");
        assert!(limelight.is_integrating());
        assert_eq!(limelight.get_log_status(), "good");
        limelight.send_invalid_status("bad");
        assert!(!limelight.is_integrating());
        assert_eq!(limelight.get_log_status(), "bad");
        limelight.set_tag_status("tags");
        assert_eq!(limelight.get_tag_status(), "tags");

        assert!(limelight.get_last_estimate_age_seconds().is_nan());
        limelight.set_last_estimate_age_seconds(0.25);
        limelight.set_integrated_this_loop(true);
        limelight.invalidate();
        assert!(limelight.get_last_estimate_age_seconds().is_nan());
        assert!(!limelight.is_integrated_this_loop());
        // Status survives the per-loop reset.
        assert_eq!(limelight.get_tag_status(), "tags");
    }

    #[test]
    fn decimal_format_matches_javas_default_pattern() {
        assert_eq!(format_decimal(0.0), "0");
        assert_eq!(format_decimal(1.5), "1.5");
        assert_eq!(format_decimal(1.0), "1");
        assert_eq!(format_decimal(1234567.891_23), "1,234,567.891");
        assert_eq!(format_decimal(-0.0004), "-0");
        assert_eq!(format_decimal(-12.3456), "-12.346");
        // Rounds on the exact binary value: 0.0005 sits just above the tie.
        assert_eq!(format_decimal(0.0005), "0.001");
        assert_eq!(format_decimal(0.0015), "0.002");
        // Exact ties go to the even digit.
        assert_eq!(format_decimal(0.0625), "0.062");
        assert_eq!(format_decimal(0.1875), "0.188");
        assert_eq!(format_decimal(100.0), "100");
        assert_eq!(format_decimal(999.9996), "1,000");
    }

    /// The same checks against ntcore, as the Java test does. Needs natives.
    mod native {
        use super::*;
        use wpilib::nt::{DoubleArray, Instance, Options};

        fn unique_table() -> String {
            use std::sync::atomic::{AtomicU32, Ordering};
            static NEXT: AtomicU32 = AtomicU32::new(0);
            format!(
                "limelight-test-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::SeqCst)
            )
        }

        #[test]
        #[ignore = "needs natives"]
        fn mt1_cache_holds_until_invalidate_over_ntcore() {
            wpilib::hal::initialize();
            let table = unique_table();
            let topic = |key: &str| {
                Instance::default_instance()
                    .table(&table)
                    .topic::<DoubleArray>(key)
            };
            let pose = topic("botpose_wpiblue").publish(&Options::default());
            let tv = Instance::default_instance()
                .table(&table)
                .topic::<wpilib::nt::Double>("tv")
                .publish(&Options::default());
            let limelight = Limelight::new(&table);

            pose.set(&botpose(1.0, 2.0, 30.0, 20.0, &[fiducial(7, 0.1)]));
            tv.set(1.0);
            assert!((limelight.get_mega_tag1_pose3d().x() - 1.0).abs() < EPS);
            assert_eq!(limelight.get_raw_fiducial()[0].id, 7);
            assert!(limelight.target_in_view());
            assert!(limelight.is_camera_connected());

            pose.set(&botpose(5.0, 6.0, 90.0, 10.0, &[fiducial(3, 0.2)]));
            tv.set(0.0);
            assert!((limelight.get_mega_tag1_pose3d().x() - 1.0).abs() < EPS);
            assert!(limelight.target_in_view());

            limelight.invalidate();
            assert!((limelight.get_mega_tag1_pose3d().x() - 5.0).abs() < EPS);
            assert!(!limelight.target_in_view());
        }

        #[test]
        #[ignore = "needs natives"]
        fn set_robot_orientation_writes_over_ntcore() {
            wpilib::hal::initialize();
            let table = unique_table();
            let limelight = Limelight::new(&table);
            let sub = Instance::default_instance()
                .table(&table)
                .topic::<DoubleArray>("robot_orientation_set")
                .subscribe(Vec::new(), &Options::default());
            limelight.set_robot_orientation(90.0);
            Instance::default_instance().flush_local();
            assert_eq!(sub.get(), [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        }
    }
}
