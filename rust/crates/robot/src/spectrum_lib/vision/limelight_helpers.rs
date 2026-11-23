//! The part of Limelight's `LimelightHelpers` v1.14 the robot uses: AprilTag pose estimates,
//! a few targeting values, and the NetworkTables writes that configure the camera. Topic
//! names, array layouts and parsing follow the vendor file.
//!
//! Not ported because the robot never calls them: JSON results (`getLatestResults` and the
//! `LimelightResults` types), raw targets and detections, the heartbeat, the red-alliance
//! and target-space bot poses, crop and keystone, stream modes, the Python script data,
//! snapshots, the flushing `SetRobotOrientation`, `Flush`, and USB port forwarding.

use std::cmp::Ordering;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock, PoisonError};
use wpilib::nt::{DoubleArray, Entry, Instance, NetworkTableEntry, Options};
use wpimath::geometry::{Pose2d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d};

/// Compares like Java's `Double.compare(a, b) == 0`: NaN equals NaN and `0.0` differs from
/// `-0.0`.
fn same(a: f64, b: f64) -> bool {
    a.total_cmp(&b) == Ordering::Equal
}

/// A Limelight raw fiducial result from the camera's NetworkTables output.
#[derive(Clone, Debug, Default)]
pub struct RawFiducial {
    pub id: i32,
    pub txnc: f64,
    pub tync: f64,
    pub ta: f64,
    pub dist_to_camera: f64,
    pub dist_to_robot: f64,
    pub ambiguity: f64,
}

impl PartialEq for RawFiducial {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
            && same(self.txnc, other.txnc)
            && same(self.tync, other.tync)
            && same(self.ta, other.ta)
            && same(self.dist_to_camera, other.dist_to_camera)
            && same(self.dist_to_robot, other.dist_to_robot)
            && same(self.ambiguity, other.ambiguity)
    }
}

/// A pose estimate with its capture time, tag statistics and the fiducials behind it.
#[derive(Clone, Debug)]
pub struct PoseEstimate {
    pub pose: Pose2d,
    pub timestamp_seconds: f64,
    pub latency: f64,
    pub tag_count: i32,
    pub tag_span: f64,
    pub avg_tag_dist: f64,
    pub avg_tag_area: f64,
    pub raw_fiducials: Vec<RawFiducial>,
    pub is_mega_tag2: bool,
}

impl Default for PoseEstimate {
    fn default() -> Self {
        Self {
            pose: Pose2d::ZERO,
            timestamp_seconds: 0.0,
            latency: 0.0,
            tag_count: 0,
            tag_span: 0.0,
            avg_tag_dist: 0.0,
            avg_tag_area: 0.0,
            raw_fiducials: Vec::new(),
            is_mega_tag2: false,
        }
    }
}

// Java: equality ignores `timestamp_seconds` and `is_mega_tag2`.
impl PartialEq for PoseEstimate {
    fn eq(&self, other: &Self) -> bool {
        same(self.latency, other.latency)
            && self.tag_count == other.tag_count
            && same(self.tag_span, other.tag_span)
            && same(self.avg_tag_dist, other.avg_tag_dist)
            && same(self.avg_tag_area, other.avg_tag_area)
            && self.pose == other.pose
            && self.raw_fiducials == other.raw_fiducials
    }
}

/// `"limelight"` for an empty name, the camera's default table.
pub fn sanitize_name(name: &str) -> &str {
    if name.is_empty() { "limelight" } else { name }
}

/// Takes a pose array `[x, y, z, roll, pitch, yaw]` with angles in degrees. Anything shorter
/// than 6 gives the zero pose.
pub fn to_pose3d(in_data: &[f64]) -> Pose3d {
    if in_data.len() < 6 {
        return Pose3d::ZERO;
    }
    Pose3d::new(
        Translation3d::new(in_data[0], in_data[1], in_data[2]),
        Rotation3d::from_euler(
            in_data[3].to_radians(),
            in_data[4].to_radians(),
            in_data[5].to_radians(),
        ),
    )
}

/// Like [`to_pose3d`] but uses only x, y and yaw.
pub fn to_pose2d(in_data: &[f64]) -> Pose2d {
    if in_data.len() < 6 {
        return Pose2d::ZERO;
    }
    Pose2d::new(
        Translation2d::new(in_data[0], in_data[1]),
        Rotation2d::new(in_data[5].to_radians()),
    )
}

fn extract_array_entry(in_data: &[f64], position: usize) -> f64 {
    if in_data.len() < position + 1 {
        return 0.0;
    }
    in_data[position]
}

/// Parses a raw botpose array, as published on `botpose_wpiblue` or `botpose_orb_wpiblue`,
/// into a [`PoseEstimate`]. Pure: callers that hold a sample get an estimate without a
/// second read.
///
/// Layout: `[x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgDist, avgArea]`,
/// then 7 values per fiducial: `[id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]`.
/// An empty array gives a default estimate. `timestamp_micros` is the NetworkTables
/// timestamp of the sample.
pub fn parse_pose_estimate(
    pose_array: &[f64],
    timestamp_micros: i64,
    is_mega_tag2: bool,
) -> PoseEstimate {
    if pose_array.is_empty() {
        return PoseEstimate::default();
    }

    let pose = to_pose2d(pose_array);
    let latency = extract_array_entry(pose_array, 6);
    let tag_count = extract_array_entry(pose_array, 7) as i32;
    let tag_span = extract_array_entry(pose_array, 8);
    let tag_dist = extract_array_entry(pose_array, 9);
    let tag_area = extract_array_entry(pose_array, 10);

    // Convert the timestamp from microseconds to seconds and adjust for latency.
    let adjusted_timestamp = (timestamp_micros as f64 / 1_000_000.0) - (latency / 1000.0);

    const VALS_PER_FIDUCIAL: i32 = 7;
    let expected_total_vals = 11i32.wrapping_add(VALS_PER_FIDUCIAL.wrapping_mul(tag_count));

    // A size mismatch gives no fiducials rather than a partly filled list.
    let raw_fiducials = if pose_array.len() as i64 != i64::from(expected_total_vals) {
        Vec::new()
    } else {
        (0..tag_count.max(0) as usize)
            .map(|i| {
                let base = 11 + i * VALS_PER_FIDUCIAL as usize;
                RawFiducial {
                    id: pose_array[base] as i32,
                    txnc: pose_array[base + 1],
                    tync: pose_array[base + 2],
                    ta: pose_array[base + 3],
                    dist_to_camera: pose_array[base + 4],
                    dist_to_robot: pose_array[base + 5],
                    ambiguity: pose_array[base + 6],
                }
            })
            .collect()
    };

    PoseEstimate {
        pose,
        timestamp_seconds: adjusted_timestamp,
        latency,
        tag_count,
        tag_span,
        avg_tag_dist: tag_dist,
        avg_tag_area: tag_area,
        raw_fiducials,
        is_mega_tag2,
    }
}

/// The NetworkTables calls the helpers make, on the default instance. A trait so tests run
/// without ntcore. `table` is already sanitized.
pub trait NtAccess: Send + Sync {
    /// The latest array with its timestamp in microseconds. Empty and 0 when unpublished.
    fn double_array_atomic(&self, table: &str, entry: &str) -> (Vec<f64>, i64);
    fn get_double(&self, table: &str, entry: &str) -> f64;
    fn set_double(&self, table: &str, entry: &str, value: f64);
    fn get_double_array(&self, table: &str, entry: &str) -> Vec<f64>;
    fn set_double_array(&self, table: &str, entry: &str, value: &[f64]);
}

/// ntcore on the default instance. Entries are cached by `table/entry`, so per-loop reads
/// and writes skip the name lookup.
#[derive(Default)]
pub struct DefaultNt {
    array_entries: Mutex<HashMap<String, Entry<DoubleArray>>>,
    scalar_entries: Mutex<HashMap<String, NetworkTableEntry>>,
}

impl DefaultNt {
    fn scalar(&self, table: &str, entry: &str) -> NetworkTableEntry {
        let mut entries = self
            .scalar_entries
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        *entries
            .entry(format!("{table}/{entry}"))
            .or_insert_with(|| Instance::default_instance().table(table).get_entry(entry))
    }
}

impl NtAccess for DefaultNt {
    fn double_array_atomic(&self, table: &str, entry: &str) -> (Vec<f64>, i64) {
        let mut entries = self
            .array_entries
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        let entry = entries
            .entry(format!("{table}/{entry}"))
            .or_insert_with(|| {
                Instance::default_instance()
                    .table(table)
                    .topic::<DoubleArray>(entry)
                    .entry(Vec::new(), &Options::default())
            });
        let sample = entry.get_atomic();
        (sample.value, sample.time)
    }

    fn get_double(&self, table: &str, entry: &str) -> f64 {
        self.scalar(table, entry).get_double(0.0)
    }

    fn set_double(&self, table: &str, entry: &str, value: f64) {
        self.scalar(table, entry).set_double(value);
    }

    fn get_double_array(&self, table: &str, entry: &str) -> Vec<f64> {
        self.scalar(table, entry).get_double_array(&[])
    }

    fn set_double_array(&self, table: &str, entry: &str, value: &[f64]) {
        self.scalar(table, entry).set_double_array(value);
    }
}

/// Java's static `LimelightHelpers` methods that touch NetworkTables. Clones share one
/// backend.
#[derive(Clone)]
pub struct LimelightHelpers {
    nt: Arc<dyn NtAccess>,
}

impl Default for LimelightHelpers {
    /// The process-wide ntcore backend, shared so its entry caches live for the whole run
    /// as Java's static maps do.
    fn default() -> Self {
        static SHARED: OnceLock<Arc<DefaultNt>> = OnceLock::new();
        let shared = SHARED.get_or_init(|| Arc::new(DefaultNt::default()));
        Self { nt: shared.clone() }
    }
}

impl LimelightHelpers {
    pub fn with_nt(nt: Arc<dyn NtAccess>) -> Self {
        Self { nt }
    }

    /// Java's `getLimelightDoubleArrayEntry(table, entry).getAtomic()`: the array and its
    /// timestamp in microseconds.
    pub fn get_limelight_double_array_atomic(
        &self,
        table_name: &str,
        entry_name: &str,
    ) -> (Vec<f64>, i64) {
        self.nt
            .double_array_atomic(sanitize_name(table_name), entry_name)
    }

    pub fn get_limelight_nt_double(&self, table_name: &str, entry_name: &str) -> f64 {
        self.nt.get_double(sanitize_name(table_name), entry_name)
    }

    pub fn set_limelight_nt_double(&self, table_name: &str, entry_name: &str, val: f64) {
        self.nt
            .set_double(sanitize_name(table_name), entry_name, val);
    }

    pub fn set_limelight_nt_double_array(&self, table_name: &str, entry_name: &str, val: &[f64]) {
        self.nt
            .set_double_array(sanitize_name(table_name), entry_name, val);
    }

    pub fn get_limelight_nt_double_array(&self, table_name: &str, entry_name: &str) -> Vec<f64> {
        self.nt
            .get_double_array(sanitize_name(table_name), entry_name)
    }

    /// Does the Limelight have a valid target?
    pub fn get_tv(&self, limelight_name: &str) -> bool {
        1.0 == self.get_limelight_nt_double(limelight_name, "tv")
    }

    /// Horizontal offset from the crosshair to the target, in degrees.
    pub fn get_tx(&self, limelight_name: &str) -> f64 {
        self.get_limelight_nt_double(limelight_name, "tx")
    }

    /// Vertical offset from the crosshair to the target, in degrees.
    pub fn get_ty(&self, limelight_name: &str) -> f64 {
        self.get_limelight_nt_double(limelight_name, "ty")
    }

    /// Target area as a percentage of the image (0-100).
    pub fn get_ta(&self, limelight_name: &str) -> f64 {
        self.get_limelight_nt_double(limelight_name, "ta")
    }

    pub fn get_fiducial_id(&self, limelight_name: &str) -> f64 {
        self.get_limelight_nt_double(limelight_name, "tid")
    }

    /// The camera's pose in the target's coordinate system.
    pub fn get_camera_pose3d_target_space(&self, limelight_name: &str) -> Pose3d {
        to_pose3d(&self.get_limelight_nt_double_array(limelight_name, "camerapose_targetspace"))
    }

    /// The target's pose in the robot's coordinate system.
    pub fn get_target_pose3d_robot_space(&self, limelight_name: &str) -> Pose3d {
        to_pose3d(&self.get_limelight_nt_double_array(limelight_name, "targetpose_robotspace"))
    }

    fn get_bot_pose_estimate(
        &self,
        limelight_name: &str,
        entry_name: &str,
        is_mega_tag2: bool,
    ) -> PoseEstimate {
        let (value, timestamp) = self.get_limelight_double_array_atomic(limelight_name, entry_name);
        parse_pose_estimate(&value, timestamp, is_mega_tag2)
    }

    /// The MegaTag1 pose and timestamp in the WPILib Blue alliance coordinate system.
    pub fn get_bot_pose_estimate_wpi_blue(&self, limelight_name: &str) -> PoseEstimate {
        self.get_bot_pose_estimate(limelight_name, "botpose_wpiblue", false)
    }

    /// The MegaTag2 pose and timestamp in the WPILib Blue alliance coordinate system. Call
    /// `set_robot_orientation_no_flush` first.
    pub fn get_bot_pose_estimate_wpi_blue_mega_tag2(&self, limelight_name: &str) -> PoseEstimate {
        self.get_bot_pose_estimate(limelight_name, "botpose_orb_wpiblue", true)
    }

    pub fn set_pipeline_index(&self, limelight_name: &str, pipeline_index: i32) {
        self.set_limelight_nt_double(limelight_name, "pipeline", f64::from(pipeline_index));
    }

    pub fn set_led_mode_force_off(&self, limelight_name: &str) {
        self.set_limelight_nt_double(limelight_name, "ledMode", 1.0);
    }

    pub fn set_led_mode_force_blink(&self, limelight_name: &str) {
        self.set_limelight_nt_double(limelight_name, "ledMode", 2.0);
    }

    pub fn set_led_mode_force_on(&self, limelight_name: &str) {
        self.set_limelight_nt_double(limelight_name, "ledMode", 3.0);
    }

    /// Sets the robot orientation MegaTag2 uses. Does not flush NetworkTables. `yaw` is in
    /// degrees, 0 facing the red alliance wall. The rates are in degrees per second.
    #[allow(clippy::too_many_arguments)] // mirrors the vendor signature
    pub fn set_robot_orientation_no_flush(
        &self,
        limelight_name: &str,
        yaw: f64,
        yaw_rate: f64,
        pitch: f64,
        pitch_rate: f64,
        roll: f64,
        roll_rate: f64,
    ) {
        self.set_limelight_nt_double_array(
            limelight_name,
            "robot_orientation_set",
            &[yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate],
        );
    }

    /// Sets the IMU mode for MegaTag2 localization.
    pub fn set_imu_mode(&self, limelight_name: &str, mode: i32) {
        self.set_limelight_nt_double(limelight_name, "imumode_set", f64::from(mode));
    }

    /// Sets the camera pose relative to the robot. Offsets are in metres, angles in degrees.
    #[allow(clippy::too_many_arguments)] // mirrors the vendor signature
    pub fn set_camera_pose_robot_space(
        &self,
        limelight_name: &str,
        forward: f64,
        side: f64,
        up: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) {
        self.set_limelight_nt_double_array(
            limelight_name,
            "camerapose_robotspace_set",
            &[forward, side, up, roll, pitch, yaw],
        );
    }

    /// Triggers a rewind capture by bumping the counter in `capture_rewind`. The camera caps
    /// the duration at 165 seconds and rate-limits captures.
    pub fn trigger_rewind_capture(&self, limelight_name: &str, duration_seconds: f64) {
        let current = self.get_limelight_nt_double_array(limelight_name, "capture_rewind");
        let counter = current.first().copied().unwrap_or(0.0);
        // Math.min propagates NaN; f64::min does not.
        let duration = if duration_seconds.is_nan() {
            duration_seconds
        } else {
            duration_seconds.min(165.0)
        };
        self.set_limelight_nt_double_array(
            limelight_name,
            "capture_rewind",
            &[counter + 1.0, duration],
        );
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::*;
    use std::collections::HashMap;

    /// In-memory NetworkTables for tests, with Java-visible semantics: missing values read
    /// as empty and 0. Counts array reads so tests can check how often a sample is fetched.
    #[derive(Default)]
    pub struct FakeNt {
        arrays: Mutex<HashMap<String, (Vec<f64>, i64)>>,
        doubles: Mutex<HashMap<String, f64>>,
        pub array_reads: std::sync::atomic::AtomicUsize,
        pub reads: std::sync::atomic::AtomicUsize,
    }

    fn key(table: &str, entry: &str) -> String {
        format!("{table}/{entry}")
    }

    impl FakeNt {
        pub fn publish_array(&self, table: &str, entry: &str, value: &[f64], micros: i64) {
            self.arrays
                .lock()
                .unwrap()
                .insert(key(table, entry), (value.to_vec(), micros));
        }

        pub fn publish_double(&self, table: &str, entry: &str, value: f64) {
            self.doubles
                .lock()
                .unwrap()
                .insert(key(table, entry), value);
        }

        pub fn array(&self, table: &str, entry: &str) -> Option<Vec<f64>> {
            self.arrays
                .lock()
                .unwrap()
                .get(&key(table, entry))
                .map(|a| a.0.clone())
        }

        pub fn double(&self, table: &str, entry: &str) -> Option<f64> {
            self.doubles
                .lock()
                .unwrap()
                .get(&key(table, entry))
                .copied()
        }

        pub fn total_reads(&self) -> usize {
            use std::sync::atomic::Ordering::SeqCst;
            self.array_reads.load(SeqCst) + self.reads.load(SeqCst)
        }
    }

    impl NtAccess for FakeNt {
        fn double_array_atomic(&self, table: &str, entry: &str) -> (Vec<f64>, i64) {
            self.array_reads
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            self.arrays
                .lock()
                .unwrap()
                .get(&key(table, entry))
                .cloned()
                .unwrap_or_default()
        }

        fn get_double(&self, table: &str, entry: &str) -> f64 {
            self.reads.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            self.double(table, entry).unwrap_or(0.0)
        }

        fn set_double(&self, table: &str, entry: &str, value: f64) {
            self.publish_double(table, entry, value);
        }

        fn get_double_array(&self, table: &str, entry: &str) -> Vec<f64> {
            self.reads.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            self.array(table, entry).unwrap_or_default()
        }

        fn set_double_array(&self, table: &str, entry: &str, value: &[f64]) {
            let micros = self
                .arrays
                .lock()
                .unwrap()
                .get(&key(table, entry))
                .map_or(0, |a| a.1);
            self.publish_array(table, entry, value, micros);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::FakeNt;
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-9;

    /// `[x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgDist, avgArea]` then
    /// fiducials of `[id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]`.
    fn array(fiducials: &[[f64; 7]]) -> Vec<f64> {
        let mut out = vec![
            1.5,
            -2.5,
            0.3,
            10.0,
            20.0,
            90.0,
            25.0,
            fiducials.len() as f64,
            1.25,
            3.5,
            0.75,
        ];
        for f in fiducials {
            out.extend_from_slice(f);
        }
        out
    }

    #[test]
    fn pose3d_parses_translation_and_degree_angles() {
        let pose = to_pose3d(&[1.0, 2.0, 3.0, 10.0, 20.0, 30.0]);
        assert_eq!((pose.x(), pose.y(), pose.z()), (1.0, 2.0, 3.0));
        assert!((pose.rotation.x() - 10.0_f64.to_radians()).abs() < EPS);
        assert!((pose.rotation.y() - 20.0_f64.to_radians()).abs() < EPS);
        assert!((pose.rotation.z() - 30.0_f64.to_radians()).abs() < EPS);
    }

    #[test]
    fn short_pose_arrays_give_the_zero_pose() {
        assert_eq!(to_pose3d(&[1.0, 2.0, 3.0, 4.0, 5.0]), Pose3d::ZERO);
        assert_eq!(to_pose2d(&[1.0, 2.0, 3.0, 4.0, 5.0]), Pose2d::ZERO);
        assert_eq!(to_pose2d(&[]), Pose2d::ZERO);
    }

    #[test]
    fn pose2d_uses_x_y_and_yaw_only() {
        let pose = to_pose2d(&[1.0, 2.0, 99.0, 45.0, 45.0, 90.0]);
        assert_eq!((pose.x(), pose.y()), (1.0, 2.0));
        assert!((pose.rotation.radians() - FRAC_PI_2).abs() < EPS);
    }

    #[test]
    fn estimate_reads_each_statistic_from_its_index() {
        let fiducials = [
            [7.0, 0.1, 0.2, 1.5, 2.0, 2.5, 0.05],
            [9.0, -0.1, -0.2, 2.5, 3.0, 3.5, 0.25],
        ];
        let estimate = parse_pose_estimate(&array(&fiducials), 4_000_000, true);
        assert_eq!(estimate.pose.x(), 1.5);
        assert_eq!(estimate.pose.y(), -2.5);
        assert!((estimate.pose.rotation.radians() - FRAC_PI_2).abs() < EPS);
        assert_eq!(estimate.latency, 25.0);
        assert_eq!(estimate.tag_count, 2);
        assert_eq!(estimate.tag_span, 1.25);
        assert_eq!(estimate.avg_tag_dist, 3.5);
        assert_eq!(estimate.avg_tag_area, 0.75);
        assert!(estimate.is_mega_tag2);
        assert_eq!(estimate.raw_fiducials.len(), 2);
        assert_eq!(
            estimate.raw_fiducials[1],
            RawFiducial {
                id: 9,
                txnc: -0.1,
                tync: -0.2,
                ta: 2.5,
                dist_to_camera: 3.0,
                dist_to_robot: 3.5,
                ambiguity: 0.25,
            }
        );
    }

    #[test]
    fn timestamp_is_seconds_minus_latency() {
        let estimate = parse_pose_estimate(&array(&[]), 4_000_000, false);
        assert!((estimate.timestamp_seconds - (4.0 - 0.025)).abs() < EPS);
        assert!(!estimate.is_mega_tag2);
        // Fractional microseconds convert exactly like Java's long / 1e6.
        let estimate = parse_pose_estimate(&array(&[]), 1_234_567, false);
        assert!((estimate.timestamp_seconds - (1.234567 - 0.025)).abs() < EPS);
    }

    #[test]
    fn empty_array_is_the_default_estimate() {
        assert_eq!(parse_pose_estimate(&[], 5, true), PoseEstimate::default());
        assert!(!parse_pose_estimate(&[], 5, true).is_mega_tag2);
        assert_eq!(parse_pose_estimate(&[], 5, true).timestamp_seconds, 0.0);
    }

    #[test]
    fn tag_count_without_matching_fiducial_values_drops_the_fiducials() {
        let mut values = array(&[]);
        values[7] = 2.0;
        let estimate = parse_pose_estimate(&values, 1, false);
        assert_eq!(estimate.tag_count, 2);
        assert!(estimate.raw_fiducials.is_empty());

        // One value too many also fails the length check.
        let mut values = array(&[[1.0; 7]]);
        values.push(0.0);
        let estimate = parse_pose_estimate(&values, 1, false);
        assert_eq!(estimate.tag_count, 1);
        assert!(estimate.raw_fiducials.is_empty());
    }

    #[test]
    fn arrays_shorter_than_a_pose_fill_missing_indexes_with_zero() {
        let estimate = parse_pose_estimate(&[1.0, 2.0, 3.0], 2_000_000, false);
        assert_eq!(estimate.pose, Pose2d::ZERO);
        assert_eq!(estimate.latency, 0.0);
        assert_eq!(estimate.tag_count, 0);
        assert_eq!(estimate.timestamp_seconds, 2.0);
        // 3 values against an expected 11: no fiducials.
        assert!(estimate.raw_fiducials.is_empty());
    }

    #[test]
    fn absurd_tag_counts_do_not_panic() {
        let mut values = array(&[]);
        values[7] = 1e18;
        assert!(
            parse_pose_estimate(&values, 1, false)
                .raw_fiducials
                .is_empty()
        );
        values[7] = f64::NAN;
        assert_eq!(parse_pose_estimate(&values, 1, false).tag_count, 0);
        values[7] = -3.0;
        assert!(
            parse_pose_estimate(&values, 1, false)
                .raw_fiducials
                .is_empty()
        );
    }

    #[test]
    fn estimate_equality_ignores_timestamp_and_megatag_flag_like_java() {
        let a = parse_pose_estimate(&array(&[[1.0; 7]]), 1_000_000, false);
        let b = parse_pose_estimate(&array(&[[1.0; 7]]), 9_000_000, true);
        assert_eq!(a, b);
        let c = parse_pose_estimate(&array(&[[2.0; 7]]), 1_000_000, false);
        assert_ne!(a, c);
    }

    #[test]
    fn double_equality_follows_java_compare() {
        let nan = RawFiducial {
            ta: f64::NAN,
            ..RawFiducial::default()
        };
        assert_eq!(nan, nan.clone());
        let negative_zero = RawFiducial {
            ta: -0.0,
            ..RawFiducial::default()
        };
        assert_ne!(negative_zero, RawFiducial::default());
    }

    #[test]
    fn empty_name_means_the_default_limelight() {
        assert_eq!(sanitize_name(""), "limelight");
        assert_eq!(sanitize_name("limelight-back"), "limelight-back");
    }

    fn helpers() -> (LimelightHelpers, Arc<FakeNt>) {
        let nt = Arc::new(FakeNt::default());
        (LimelightHelpers::with_nt(nt.clone()), nt)
    }

    #[test]
    fn bot_pose_estimates_read_their_topics_with_the_megatag_flag() {
        let (helpers, nt) = helpers();
        let one = array(&[[7.0, 0.1, 0.2, 1.5, 2.0, 2.5, 0.1]]);
        nt.publish_array("ll", "botpose_wpiblue", &one, 4_000_000);
        nt.publish_array("ll", "botpose_orb_wpiblue", &one, 5_000_000);
        let mt1 = helpers.get_bot_pose_estimate_wpi_blue("ll");
        let mt2 = helpers.get_bot_pose_estimate_wpi_blue_mega_tag2("ll");
        assert_eq!(mt1, parse_pose_estimate(&one, 4_000_000, false));
        assert!(!mt1.is_mega_tag2 && mt2.is_mega_tag2);
        assert!((mt1.timestamp_seconds - 3.975).abs() < EPS);
        assert!((mt2.timestamp_seconds - 4.975).abs() < EPS);
    }

    #[test]
    fn empty_names_resolve_to_the_limelight_table() {
        let (helpers, nt) = helpers();
        nt.publish_double("limelight", "tx", 3.0);
        assert_eq!(helpers.get_tx(""), 3.0);
        helpers.set_pipeline_index("", 6);
        assert_eq!(nt.double("limelight", "pipeline"), Some(6.0));
    }

    #[test]
    fn scalar_getters_use_the_vendor_topic_names() {
        let (helpers, nt) = helpers();
        for (entry, value) in [
            ("tv", 1.0),
            ("tx", 1.5),
            ("ty", -2.5),
            ("ta", 3.5),
            ("tid", 12.0),
        ] {
            nt.publish_double("ll", entry, value);
        }
        assert!(helpers.get_tv("ll"));
        assert_eq!(helpers.get_tx("ll"), 1.5);
        assert_eq!(helpers.get_ty("ll"), -2.5);
        assert_eq!(helpers.get_ta("ll"), 3.5);
        assert_eq!(helpers.get_fiducial_id("ll"), 12.0);
        nt.publish_double("ll", "tv", 0.5);
        assert!(!helpers.get_tv("ll"), "only exactly 1.0 is a valid target");
    }

    #[test]
    fn pose_getters_read_their_arrays() {
        let (helpers, nt) = helpers();
        nt.publish_array(
            "ll",
            "camerapose_targetspace",
            &[1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
            0,
        );
        nt.publish_array(
            "ll",
            "targetpose_robotspace",
            &[4.0, 5.0, 6.0, 0.0, 0.0, 90.0],
            0,
        );
        assert_eq!(helpers.get_camera_pose3d_target_space("ll").x(), 1.0);
        let target = helpers.get_target_pose3d_robot_space("ll");
        assert_eq!(target.z(), 6.0);
        assert!((target.rotation.z() - FRAC_PI_2).abs() < EPS);
    }

    #[test]
    fn led_modes_use_vendor_codes() {
        let (helpers, nt) = helpers();
        helpers.set_led_mode_force_off("ll");
        assert_eq!(nt.double("ll", "ledMode"), Some(1.0));
        helpers.set_led_mode_force_blink("ll");
        assert_eq!(nt.double("ll", "ledMode"), Some(2.0));
        helpers.set_led_mode_force_on("ll");
        assert_eq!(nt.double("ll", "ledMode"), Some(3.0));
    }

    #[test]
    fn orientation_and_camera_pose_write_the_documented_layouts() {
        let (helpers, nt) = helpers();
        helpers.set_robot_orientation_no_flush("ll", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(
            nt.array("ll", "robot_orientation_set").unwrap(),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
        helpers.set_camera_pose_robot_space("ll", 0.1, 0.2, 0.3, 4.0, 5.0, 6.0);
        assert_eq!(
            nt.array("ll", "camerapose_robotspace_set").unwrap(),
            [0.1, 0.2, 0.3, 4.0, 5.0, 6.0]
        );
        helpers.set_imu_mode("ll", 4);
        assert_eq!(nt.double("ll", "imumode_set"), Some(4.0));
    }

    #[test]
    fn rewind_capture_bumps_the_counter_and_caps_the_duration() {
        let (helpers, nt) = helpers();
        helpers.trigger_rewind_capture("ll", 30.0);
        assert_eq!(nt.array("ll", "capture_rewind").unwrap(), [1.0, 30.0]);
        helpers.trigger_rewind_capture("ll", 500.0);
        assert_eq!(nt.array("ll", "capture_rewind").unwrap(), [2.0, 165.0]);
        helpers.trigger_rewind_capture("ll", 165.0);
        assert_eq!(nt.array("ll", "capture_rewind").unwrap(), [3.0, 165.0]);
    }

    fn estimate_out(e: &PoseEstimate) -> Vec<f64> {
        let mut out = vec![
            e.pose.x(),
            e.pose.y(),
            e.pose.rotation.radians(),
            e.timestamp_seconds,
            e.latency,
            f64::from(e.tag_count),
            e.tag_span,
            e.avg_tag_dist,
            e.avg_tag_area,
            e.raw_fiducials.len() as f64,
        ];
        for f in &e.raw_fiducials {
            out.extend([
                f64::from(f.id),
                f.txnc,
                f.tync,
                f.ta,
                f.dist_to_camera,
                f.dist_to_robot,
                f.ambiguity,
            ]);
        }
        out
    }

    #[test]
    fn pose_parsing_matches_java() {
        golden::check("vision_pose", 1e-9, |op, x| match op {
            "parse_pose_estimate" => Some(estimate_out(&parse_pose_estimate(
                &x[2..],
                x[1] as i64,
                x[0] == 1.0,
            ))),
            "to_pose3d" => {
                let pose = to_pose3d(x);
                let q = pose.rotation.quaternion();
                Some(vec![
                    pose.x(),
                    pose.y(),
                    pose.z(),
                    pose.rotation.x(),
                    pose.rotation.y(),
                    pose.rotation.z(),
                    q.w,
                    q.x,
                    q.y,
                    q.z,
                ])
            }
            _ => None,
        });
    }
}
