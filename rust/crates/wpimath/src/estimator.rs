//! Swerve pose estimator: odometry corrected by delayed vision measurements.
//!
//! Port of WPILib `PoseEstimator` with a `SwerveDriveOdometry`. WPILib's wall-clock
//! `update` is not ported: pass the timestamp to [`SwerveDrivePoseEstimator::update_with_time`].

use crate::geometry::{Pose2d, Rotation2d, Transform2d, Translation2d};
use crate::interpolation::{Key, TimeInterpolatableBuffer};
use crate::kinematics::{SwerveDriveKinematics, SwerveDriveOdometry, SwerveModulePosition};
use crate::math_util;
use std::collections::BTreeMap;

const BUFFER_DURATION: f64 = 1.5;

#[derive(Clone, Copy, Debug)]
struct VisionUpdate {
    vision_pose: Pose2d,
    odometry_pose: Pose2d,
}

impl VisionUpdate {
    fn compensate(&self, pose: Pose2d) -> Pose2d {
        let delta = pose - self.odometry_pose;
        self.vision_pose + delta
    }
}

#[derive(Clone, Debug)]
pub struct SwerveDrivePoseEstimator<const N: usize> {
    odometry: SwerveDriveOdometry<N>,
    q: [f64; 3],
    vision_k: [f64; 3],
    odometry_pose_buffer: TimeInterpolatableBuffer<Pose2d>,
    vision_updates: BTreeMap<Key, VisionUpdate>,
    pose_estimate: Pose2d,
}

impl<const N: usize> SwerveDrivePoseEstimator<N> {
    /// WPILib defaults are `[0.1; 3]` state and `[0.9; 3]` vision standard deviations.
    pub fn new(
        kinematics: SwerveDriveKinematics<N>,
        gyro_angle: Rotation2d,
        module_positions: [SwerveModulePosition; N],
        initial_pose: Pose2d,
        state_std_devs: [f64; 3],
        vision_measurement_std_devs: [f64; 3],
    ) -> Self {
        let odometry =
            SwerveDriveOdometry::new(kinematics, gyro_angle, module_positions, initial_pose);
        let pose_estimate = odometry.pose();
        let mut est = Self {
            odometry,
            q: state_std_devs.map(|s| s * s),
            vision_k: [0.0; 3],
            odometry_pose_buffer: TimeInterpolatableBuffer::new(BUFFER_DURATION),
            vision_updates: BTreeMap::new(),
            pose_estimate,
        };
        est.set_vision_measurement_std_devs(vision_measurement_std_devs);
        est
    }

    /// Lower vision standard deviations make vision pull the estimate harder.
    pub fn set_vision_measurement_std_devs(&mut self, std_devs: [f64; 3]) {
        let r = std_devs.map(|s| s * s);
        for ((k, q), r) in self.vision_k.iter_mut().zip(self.q).zip(r) {
            *k = if q == 0.0 {
                0.0
            } else {
                q / (q + (q * r).sqrt())
            };
        }
    }

    fn clear_history(&mut self) {
        self.odometry_pose_buffer.clear();
        self.vision_updates.clear();
    }

    pub fn reset_position(
        &mut self,
        gyro_angle: Rotation2d,
        wheel_positions: [SwerveModulePosition; N],
        pose: Pose2d,
    ) {
        self.odometry
            .reset_position(gyro_angle, wheel_positions, pose);
        self.clear_history();
        self.pose_estimate = self.odometry.pose();
    }

    pub fn reset_pose(&mut self, pose: Pose2d) {
        self.odometry.reset_pose(pose);
        self.clear_history();
        self.pose_estimate = self.odometry.pose();
    }

    pub fn reset_translation(&mut self, translation: Translation2d) {
        self.odometry.reset_translation(translation);
        let latest = self.vision_updates.last_key_value().map(|(k, v)| (*k, *v));
        self.clear_history();
        self.pose_estimate = match latest {
            Some((key, latest)) => {
                let update = VisionUpdate {
                    vision_pose: Pose2d::new(translation, latest.vision_pose.rotation),
                    odometry_pose: Pose2d::new(translation, latest.odometry_pose.rotation),
                };
                self.vision_updates.insert(key, update);
                update.compensate(self.odometry.pose())
            }
            None => self.odometry.pose(),
        };
    }

    pub fn reset_rotation(&mut self, rotation: Rotation2d) {
        self.odometry.reset_rotation(rotation);
        let latest = self.vision_updates.last_key_value().map(|(k, v)| (*k, *v));
        self.clear_history();
        self.pose_estimate = match latest {
            Some((key, latest)) => {
                let update = VisionUpdate {
                    vision_pose: Pose2d::new(latest.vision_pose.translation, rotation),
                    odometry_pose: Pose2d::new(latest.odometry_pose.translation, rotation),
                };
                self.vision_updates.insert(key, update);
                update.compensate(self.odometry.pose())
            }
            None => self.odometry.pose(),
        };
    }

    pub fn estimated_position(&self) -> Pose2d {
        self.pose_estimate
    }

    /// Estimated pose at `timestamp_seconds`, clamped to the odometry history.
    pub fn sample_at(&self, timestamp_seconds: f64) -> Option<Pose2d> {
        let oldest = self.odometry_pose_buffer.oldest_time()?;
        let newest = self.odometry_pose_buffer.newest_time()?;
        let timestamp = math_util::clamp(timestamp_seconds, oldest, newest);
        let first_vision = self.vision_updates.first_key_value().map(|(k, _)| k.0);
        match first_vision {
            Some(first) if timestamp >= first => {
                let (_, update) = self.vision_updates.range(..=Key(timestamp)).next_back()?;
                let odometry_estimate = self.odometry_pose_buffer.get_sample(timestamp)?;
                Some(update.compensate(odometry_estimate))
            }
            _ => self.odometry_pose_buffer.get_sample(timestamp),
        }
    }

    fn clean_up_vision_updates(&mut self) {
        let Some(oldest) = self.odometry_pose_buffer.oldest_time() else {
            return;
        };
        let Some((&first, _)) = self.vision_updates.first_key_value() else {
            return;
        };
        if oldest < first.0 {
            return;
        }
        let newest_needed = self
            .vision_updates
            .range(..=Key(oldest))
            .next_back()
            .map(|(k, _)| *k);
        if let Some(needed) = newest_needed {
            self.vision_updates.retain(|k, _| *k >= needed);
        }
    }

    /// Ignores the measurement when `timestamp_seconds` is older than the odometry history.
    pub fn add_vision_measurement(&mut self, vision_robot_pose: Pose2d, timestamp_seconds: f64) {
        let Some(newest) = self.odometry_pose_buffer.newest_time() else {
            return;
        };
        if newest - BUFFER_DURATION > timestamp_seconds {
            return;
        }
        self.clean_up_vision_updates();
        let Some(odometry_sample) = self.odometry_pose_buffer.get_sample(timestamp_seconds) else {
            return;
        };
        let Some(vision_sample) = self.sample_at(timestamp_seconds) else {
            return;
        };
        let transform = vision_robot_pose - vision_sample;
        let k = self.vision_k;
        let scaled = Transform2d::from_xy(
            k[0] * transform.x(),
            k[1] * transform.y(),
            Rotation2d::new(k[2] * transform.rotation.radians()),
        );
        let update = VisionUpdate {
            vision_pose: vision_sample + scaled,
            odometry_pose: odometry_sample,
        };
        self.vision_updates.insert(Key(timestamp_seconds), update);
        // Drop updates newer than this one: their corrections assumed the old history.
        self.vision_updates.retain(|k, _| k.0 <= timestamp_seconds);
        self.pose_estimate = update.compensate(self.odometry.pose());
    }

    pub fn add_vision_measurement_with_std_devs(
        &mut self,
        vision_robot_pose: Pose2d,
        timestamp_seconds: f64,
        vision_measurement_std_devs: [f64; 3],
    ) {
        self.set_vision_measurement_std_devs(vision_measurement_std_devs);
        self.add_vision_measurement(vision_robot_pose, timestamp_seconds);
    }

    pub fn update_with_time(
        &mut self,
        current_time_seconds: f64,
        gyro_angle: Rotation2d,
        wheel_positions: [SwerveModulePosition; N],
    ) -> Pose2d {
        let odometry_estimate = self.odometry.update(gyro_angle, wheel_positions);
        self.odometry_pose_buffer
            .add_sample(current_time_seconds, odometry_estimate);
        self.pose_estimate = match self.vision_updates.last_key_value() {
            Some((_, update)) => update.compensate(odometry_estimate),
            None => odometry_estimate,
        };
        self.pose_estimate
    }
}
