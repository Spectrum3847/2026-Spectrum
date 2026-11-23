use super::{Rotation2d, Transform2d, Translation2d, Twist2d};
use crate::interpolation::Interpolatable;
use std::ops::{Add, Div, Mul, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2d {
    pub translation: Translation2d,
    pub rotation: Rotation2d,
}

impl Pose2d {
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

    pub fn x(&self) -> f64 {
        self.translation.x
    }

    pub fn y(&self) -> f64 {
        self.translation.y
    }

    pub fn rotate_by(self, other: Rotation2d) -> Self {
        Self::new(
            self.translation.rotate_by(other),
            self.rotation.rotate_by(other),
        )
    }

    pub fn transform_by(self, other: Transform2d) -> Self {
        Self::new(
            self.translation + other.translation.rotate_by(self.rotation),
            other.rotation.rotate_by(self.rotation),
        )
    }

    /// This pose expressed in the frame of `other`.
    pub fn relative_to(self, other: Self) -> Self {
        let transform = Transform2d::from_poses(other, self);
        Self::new(transform.translation, transform.rotation)
    }

    pub fn rotate_around(self, point: Translation2d, rot: Rotation2d) -> Self {
        Self::new(
            self.translation.rotate_around(point, rot),
            self.rotation.rotate_by(rot),
        )
    }

    /// Applies a twist as constant-curvature motion.
    pub fn exp(self, twist: Twist2d) -> Self {
        let dtheta = twist.dtheta;
        let sin_theta = dtheta.sin();
        let cos_theta = dtheta.cos();
        let (s, c) = if dtheta.abs() < 1e-9 {
            (1.0 - 1.0 / 6.0 * dtheta * dtheta, 0.5 * dtheta)
        } else {
            (sin_theta / dtheta, (1.0 - cos_theta) / dtheta)
        };
        let transform = Transform2d::new(
            Translation2d::new(twist.dx * s - twist.dy * c, twist.dx * c + twist.dy * s),
            Rotation2d::from_xy(cos_theta, sin_theta),
        );
        self + transform
    }

    /// Twist that carries this pose to `end`.
    pub fn log(self, end: Self) -> Twist2d {
        let transform = end.relative_to(self);
        let dtheta = transform.rotation.radians();
        let half_dtheta = dtheta / 2.0;
        let cos_minus_one = transform.rotation.cos() - 1.0;
        let half_theta_by_tan_of_half_dtheta = if cos_minus_one.abs() < 1e-9 {
            1.0 - 1.0 / 12.0 * dtheta * dtheta
        } else {
            -(half_dtheta * transform.rotation.sin()) / cos_minus_one
        };
        let translation_part = transform.translation.rotate_by(Rotation2d::from_xy(
            half_theta_by_tan_of_half_dtheta,
            -half_dtheta,
        )) * half_theta_by_tan_of_half_dtheta.hypot(half_dtheta);
        Twist2d::new(translation_part.x, translation_part.y, dtheta)
    }
}

impl Add<Transform2d> for Pose2d {
    type Output = Self;
    fn add(self, other: Transform2d) -> Self {
        self.transform_by(other)
    }
}

impl Sub for Pose2d {
    type Output = Transform2d;
    fn sub(self, other: Self) -> Transform2d {
        let pose = self.relative_to(other);
        Transform2d::new(pose.translation, pose.rotation)
    }
}

impl Mul<f64> for Pose2d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.translation * scalar, self.rotation * scalar)
    }
}

impl Div<f64> for Pose2d {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        self * (1.0 / scalar)
    }
}

impl Interpolatable for Pose2d {
    fn interpolate(&self, end: &Self, t: f64) -> Self {
        if t < 0.0 {
            *self
        } else if t >= 1.0 {
            *end
        } else {
            let twist = self.log(*end);
            self.exp(Twist2d::new(twist.dx * t, twist.dy * t, twist.dtheta * t))
        }
    }
}
