use super::{Pose3d, Rotation3d, Transform2d, Translation3d};
use std::ops::{Add, Div, Mul};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Transform3d {
    pub translation: Translation3d,
    pub rotation: Rotation3d,
}

impl Transform3d {
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

    /// Transform that moves `initial` onto `last`.
    pub fn from_poses(initial: Pose3d, last: Pose3d) -> Self {
        Self {
            translation: (last.translation - initial.translation).rotate_by(-initial.rotation),
            rotation: last.rotation.relative_to(initial.rotation),
        }
    }

    pub fn from_transform2d(transform: Transform2d) -> Self {
        Self::new(
            Translation3d::from_translation2d(transform.translation),
            Rotation3d::from_rotation2d(transform.rotation),
        )
    }

    pub fn inverse(self) -> Self {
        Self::new(
            (-self.translation).rotate_by(-self.rotation),
            -self.rotation,
        )
    }
}

impl Mul<f64> for Transform3d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.translation * scalar, self.rotation * scalar)
    }
}

impl Div<f64> for Transform3d {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        self * (1.0 / scalar)
    }
}

impl Add for Transform3d {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::from_poses(
            Pose3d::ZERO,
            Pose3d::ZERO.transform_by(self).transform_by(other),
        )
    }
}
