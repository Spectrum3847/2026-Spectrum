use super::{SwerveDriveKinematics, SwerveModulePosition};
use crate::geometry::{Pose2d, Rotation2d, Translation2d};

/// Dead-reckoning pose from gyro angle and module positions.
#[derive(Clone, Debug)]
pub struct SwerveDriveOdometry<const N: usize> {
    kinematics: SwerveDriveKinematics<N>,
    pose: Pose2d,
    gyro_offset: Rotation2d,
    previous_angle: Rotation2d,
    previous_wheel_positions: [SwerveModulePosition; N],
}

impl<const N: usize> SwerveDriveOdometry<N> {
    pub fn new(
        kinematics: SwerveDriveKinematics<N>,
        gyro_angle: Rotation2d,
        wheel_positions: [SwerveModulePosition; N],
        initial_pose: Pose2d,
    ) -> Self {
        Self {
            kinematics,
            pose: initial_pose,
            gyro_offset: (-gyro_angle).rotate_by(initial_pose.rotation),
            previous_angle: initial_pose.rotation,
            previous_wheel_positions: wheel_positions,
        }
    }

    pub fn reset_position(
        &mut self,
        gyro_angle: Rotation2d,
        wheel_positions: [SwerveModulePosition; N],
        pose: Pose2d,
    ) {
        self.pose = pose;
        self.previous_angle = pose.rotation;
        self.gyro_offset = (-gyro_angle).rotate_by(pose.rotation);
        self.previous_wheel_positions = wheel_positions;
    }

    pub fn reset_pose(&mut self, pose: Pose2d) {
        self.gyro_offset = self
            .gyro_offset
            .rotate_by(-self.pose.rotation)
            .rotate_by(pose.rotation);
        self.pose = pose;
        self.previous_angle = pose.rotation;
    }

    pub fn reset_translation(&mut self, translation: Translation2d) {
        self.pose = Pose2d::new(translation, self.pose.rotation);
    }

    pub fn reset_rotation(&mut self, rotation: Rotation2d) {
        self.gyro_offset = self
            .gyro_offset
            .rotate_by(-self.pose.rotation)
            .rotate_by(rotation);
        self.pose = Pose2d::new(self.pose.translation, rotation);
        self.previous_angle = rotation;
    }

    pub fn pose(&self) -> Pose2d {
        self.pose
    }

    pub fn update(
        &mut self,
        gyro_angle: Rotation2d,
        wheel_positions: [SwerveModulePosition; N],
    ) -> Pose2d {
        let angle = gyro_angle.rotate_by(self.gyro_offset);
        let mut twist = self
            .kinematics
            .to_twist2d(&self.previous_wheel_positions, &wheel_positions);
        twist.dtheta = (angle - self.previous_angle).radians();
        let new_pose = self.pose.exp(twist);
        self.previous_wheel_positions = wheel_positions;
        self.previous_angle = angle;
        self.pose = Pose2d::new(new_pose.translation, angle);
        self.pose
    }
}
