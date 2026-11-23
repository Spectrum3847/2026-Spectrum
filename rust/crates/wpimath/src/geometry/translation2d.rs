use super::Rotation2d;
use crate::interpolation::Interpolatable;
use crate::math_util;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, Default)]
pub struct Translation2d {
    pub x: f64,
    pub y: f64,
}

impl Translation2d {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn from_polar(distance: f64, angle: Rotation2d) -> Self {
        Self {
            x: distance * angle.cos(),
            y: distance * angle.sin(),
        }
    }

    pub fn distance(self, other: Self) -> f64 {
        (other.x - self.x).hypot(other.y - self.y)
    }

    pub fn squared_distance(self, other: Self) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dx * dx + dy * dy
    }

    pub fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn squared_norm(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub fn angle(self) -> Rotation2d {
        Rotation2d::from_xy(self.x, self.y)
    }

    pub fn rotate_by(self, other: Rotation2d) -> Self {
        Self {
            x: self.x * other.cos() - self.y * other.sin(),
            y: self.x * other.sin() + self.y * other.cos(),
        }
    }

    pub fn rotate_around(self, other: Self, rot: Rotation2d) -> Self {
        Self {
            x: (self.x - other.x) * rot.cos() - (self.y - other.y) * rot.sin() + other.x,
            y: (self.x - other.x) * rot.sin() + (self.y - other.y) * rot.cos() + other.y,
        }
    }

    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Z component of the 3D cross product.
    pub fn cross(self, other: Self) -> f64 {
        self.x * other.y - self.y * other.x
    }
}

impl Add for Translation2d {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y)
    }
}

impl Sub for Translation2d {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y)
    }
}

impl Neg for Translation2d {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl Mul<f64> for Translation2d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.x * scalar, self.y * scalar)
    }
}

impl Div<f64> for Translation2d {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        Self::new(self.x / scalar, self.y / scalar)
    }
}

/// Equal when both components are within 1e-9, like WPILib.
impl PartialEq for Translation2d {
    fn eq(&self, other: &Self) -> bool {
        (other.x - self.x).abs() < 1e-9 && (other.y - self.y).abs() < 1e-9
    }
}

impl Interpolatable for Translation2d {
    fn interpolate(&self, end: &Self, t: f64) -> Self {
        Self::new(
            math_util::interpolate(self.x, end.x, t),
            math_util::interpolate(self.y, end.y, t),
        )
    }
}
