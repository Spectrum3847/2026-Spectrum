use super::{Pose2d, Rotation3d, Transform3d, Translation3d};
use std::ops::{Add, Div, Mul, Sub};

/// A 3D pose. WPILib's `exp`, `log` and `interpolate` need native code and are not ported.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose3d {
    pub translation: Translation3d,
    pub rotation: Rotation3d,
}

impl Pose3d {
    pub const ZERO: Self = Self {
        translation: Translation3d::ZERO,
        rotation: Rotation3d::ZERO,
    };

    pub const fn new(translation: Translation3d, rotation: Rotation3d) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    pub fn from_pose2d(pose: Pose2d) -> Self {
        Self::new(
            Translation3d::new(pose.x(), pose.y(), 0.0),
            Rotation3d::from_euler(0.0, 0.0, pose.rotation.radians()),
        )
    }

    pub fn x(&self) -> f64 {
        self.translation.x
    }

    pub fn y(&self) -> f64 {
        self.translation.y
    }

    pub fn z(&self) -> f64 {
        self.translation.z
    }

    pub fn rotate_by(self, other: Rotation3d) -> Self {
        Self::new(
            self.translation.rotate_by(other),
            self.rotation.rotate_by(other),
        )
    }

    pub fn transform_by(self, other: Transform3d) -> Self {
        Self::new(
            self.translation + other.translation.rotate_by(self.rotation),
            other.rotation.rotate_by(self.rotation),
        )
    }

    pub fn relative_to(self, other: Self) -> Self {
        let transform = Transform3d::from_poses(other, self);
        Self::new(transform.translation, transform.rotation)
    }

    pub fn rotate_around(self, point: Translation3d, rot: Rotation3d) -> Self {
        Self::new(
            self.translation.rotate_around(point, rot),
            self.rotation.rotate_by(rot),
        )
    }

    pub fn to_pose2d(&self) -> Pose2d {
        Pose2d::new(
            self.translation.to_translation2d(),
            self.rotation.to_rotation2d(),
        )
    }
}

impl Add<Transform3d> for Pose3d {
    type Output = Self;
    fn add(self, other: Transform3d) -> Self {
        self.transform_by(other)
    }
}

impl Sub for Pose3d {
    type Output = Transform3d;
    fn sub(self, other: Self) -> Transform3d {
        let pose = self.relative_to(other);
        Transform3d::new(pose.translation, pose.rotation)
    }
}

impl Mul<f64> for Pose3d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.translation * scalar, self.rotation * scalar)
    }
}

impl Div<f64> for Pose3d {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        self * (1.0 / scalar)
    }
}
