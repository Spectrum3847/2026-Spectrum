use super::{Quaternion, Rotation3d, Translation2d};
use crate::interpolation::Interpolatable;
use crate::math_util;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, Default)]
pub struct Translation3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Translation3d {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Vector of length `distance` along the x axis rotated by `angle`.
    pub fn from_polar(distance: f64, angle: Rotation3d) -> Self {
        Self::new(distance, 0.0, 0.0).rotate_by(angle)
    }

    pub fn from_translation2d(translation: Translation2d) -> Self {
        Self::new(translation.x, translation.y, 0.0)
    }

    pub fn distance(self, other: Self) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn squared_distance(self, other: Self) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        dx * dx + dy * dy + dz * dz
    }

    pub fn norm(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn squared_norm(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn rotate_by(self, other: Rotation3d) -> Self {
        let p = Quaternion::new(0.0, self.x, self.y, self.z);
        let q = other.quaternion();
        let qprime = q * p * q.inverse();
        Self::new(qprime.x, qprime.y, qprime.z)
    }

    pub fn rotate_around(self, other: Self, rot: Rotation3d) -> Self {
        (self - other).rotate_by(rot) + other
    }

    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> [f64; 3] {
        [
            self.y * other.z - other.y * self.z,
            self.z * other.x - other.z * self.x,
            self.x * other.y - other.x * self.y,
        ]
    }

    pub fn to_translation2d(self) -> Translation2d {
        Translation2d::new(self.x, self.y)
    }
}

impl Add for Translation3d {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Translation3d {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Neg for Translation3d {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Translation3d {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Div<f64> for Translation3d {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

/// Equal when all components are within 1e-9, like WPILib.
impl PartialEq for Translation3d {
    fn eq(&self, o: &Self) -> bool {
        (o.x - self.x).abs() < 1e-9 && (o.y - self.y).abs() < 1e-9 && (o.z - self.z).abs() < 1e-9
    }
}

impl Interpolatable for Translation3d {
    fn interpolate(&self, end: &Self, t: f64) -> Self {
        Self::new(
            math_util::interpolate(self.x, end.x, t),
            math_util::interpolate(self.y, end.y, t),
            math_util::interpolate(self.z, end.z, t),
        )
    }
}
