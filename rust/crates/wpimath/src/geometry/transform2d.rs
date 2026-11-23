use super::{Pose2d, Rotation2d, Translation2d};
use std::ops::{Add, Div, Mul};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Transform2d {
    pub translation: Translation2d,
    pub rotation: Rotation2d,
}

impl Transform2d {
    pub const ZERO: Self = Self {
        translation: Translation2d::ZERO,
        rotation: Rotation2d::ZERO,
    };

    pub const fn new(translation: Translation2d, rotation: Rotation2d) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    pub fn from_xy(x: f64, y: f64, rotation: Rotation2d) -> Self {
        Self::new(Translation2d::new(x, y), rotation)
    }

    /// Transform that moves `initial` onto `last`.
    pub fn from_poses(initial: Pose2d, last: Pose2d) -> Self {
        Self {
            translation: (last.translation - initial.translation).rotate_by(-initial.rotation),
            rotation: last.rotation.relative_to(initial.rotation),
        }
    }

    pub fn x(&self) -> f64 {
        self.translation.x
    }

    pub fn y(&self) -> f64 {
        self.translation.y
    }

    pub fn inverse(self) -> Self {
        Self::new(
            (-self.translation).rotate_by(-self.rotation),
            -self.rotation,
        )
    }
}

impl Mul<f64> for Transform2d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.translation * scalar, self.rotation * scalar)
    }
}

impl Div<f64> for Transform2d {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        self * (1.0 / scalar)
    }
}

impl Add for Transform2d {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::from_poses(
            Pose2d::ZERO,
            Pose2d::ZERO.transform_by(self).transform_by(other),
        )
    }
}
