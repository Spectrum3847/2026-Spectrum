use crate::interpolation::Interpolatable;
use crate::math_util;
use std::ops::{Add, Div, Mul, Neg, Sub};

/// A 2D rotation. Like WPILib it keeps the radians it was built from, so
/// [`radians`](Self::radians) is not wrapped unless the value came from a sum or `from_xy`.
#[derive(Clone, Copy, Debug)]
pub struct Rotation2d {
    value: f64,
    cos: f64,
    sin: f64,
}

impl Rotation2d {
    pub const ZERO: Self = Self {
        value: 0.0,
        cos: 1.0,
        sin: 0.0,
    };
    pub const CW_PI_2: Self = Self {
        value: -std::f64::consts::FRAC_PI_2,
        cos: 6.123233995736766e-17,
        sin: -1.0,
    };
    pub const CCW_PI_2: Self = Self {
        value: std::f64::consts::FRAC_PI_2,
        cos: 6.123233995736766e-17,
        sin: 1.0,
    };
    pub const PI: Self = Self {
        value: std::f64::consts::PI,
        cos: -1.0,
        sin: 1.2246467991473532e-16,
    };

    pub fn new(radians: f64) -> Self {
        Self {
            value: radians,
            cos: radians.cos(),
            sin: radians.sin(),
        }
    }

    /// Rotation pointing at `(x, y)`. A near-zero vector gives the zero rotation.
    pub fn from_xy(x: f64, y: f64) -> Self {
        let magnitude = x.hypot(y);
        let (cos, sin) = if magnitude > 1e-6 {
            (x / magnitude, y / magnitude)
        } else {
            (1.0, 0.0)
        };
        Self {
            value: sin.atan2(cos),
            cos,
            sin,
        }
    }

    pub fn from_degrees(degrees: f64) -> Self {
        Self::new(degrees.to_radians())
    }

    pub fn from_rotations(rotations: f64) -> Self {
        Self::new(crate::units::rotations_to_radians(rotations))
    }

    pub fn rotate_by(self, other: Self) -> Self {
        Self::from_xy(
            self.cos * other.cos - self.sin * other.sin,
            self.cos * other.sin + self.sin * other.cos,
        )
    }

    pub fn relative_to(self, other: Self) -> Self {
        self.rotate_by(-other)
    }

    pub fn radians(&self) -> f64 {
        self.value
    }

    pub fn degrees(&self) -> f64 {
        self.value.to_degrees()
    }

    pub fn rotations(&self) -> f64 {
        crate::units::radians_to_rotations(self.value)
    }

    pub fn cos(&self) -> f64 {
        self.cos
    }

    pub fn sin(&self) -> f64 {
        self.sin
    }

    pub fn tan(&self) -> f64 {
        self.sin / self.cos
    }
}

impl Default for Rotation2d {
    fn default() -> Self {
        Self::ZERO
    }
}

impl Add for Rotation2d {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        self.rotate_by(other)
    }
}

impl Sub for Rotation2d {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        self.rotate_by(-other)
    }
}

impl Neg for Rotation2d {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.value)
    }
}

impl Mul<f64> for Rotation2d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        Self::new(self.value * scalar)
    }
}

impl Div<f64> for Rotation2d {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        self * (1.0 / scalar)
    }
}

/// Equal when the unit vectors are within 1e-9, like WPILib.
impl PartialEq for Rotation2d {
    fn eq(&self, other: &Self) -> bool {
        (self.cos - other.cos).hypot(self.sin - other.sin) < 1e-9
    }
}

impl Interpolatable for Rotation2d {
    fn interpolate(&self, end: &Self, t: f64) -> Self {
        *self + (*end - *self) * math_util::clamp(t, 0.0, 1.0)
    }
}
