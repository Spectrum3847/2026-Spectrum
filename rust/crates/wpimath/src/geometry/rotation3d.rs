use super::{Quaternion, Rotation2d};
use crate::interpolation::Interpolatable;
use crate::math_util;
use std::ops::{Add, Div, Mul, Neg, Sub};

/// A 3D rotation stored as a unit quaternion.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rotation3d {
    q: Quaternion,
}

impl Rotation3d {
    pub const ZERO: Self = Self {
        q: Quaternion::IDENTITY,
    };

    /// Normalizes `q`.
    pub fn from_quaternion(q: Quaternion) -> Self {
        Self { q: q.normalize() }
    }

    /// Intrinsic roll (x), pitch (y), yaw (z) in radians.
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let cr = (roll * 0.5).cos();
        let sr = (roll * 0.5).sin();
        let cp = (pitch * 0.5).cos();
        let sp = (pitch * 0.5).sin();
        let cy = (yaw * 0.5).cos();
        let sy = (yaw * 0.5).sin();
        Self {
            q: Quaternion::new(
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ),
        }
    }

    /// Rotation of `angle_radians` about `axis`. A zero axis gives the identity.
    pub fn from_axis_angle(axis: [f64; 3], angle_radians: f64) -> Self {
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if norm == 0.0 {
            return Self::ZERO;
        }
        let s = (angle_radians / 2.0).sin();
        let scale = 1.0 / norm;
        Self {
            q: Quaternion::new(
                (angle_radians / 2.0).cos(),
                axis[0] * scale * s,
                axis[1] * scale * s,
                axis[2] * scale * s,
            ),
        }
    }

    pub fn from_rotation2d(rotation: Rotation2d) -> Self {
        Self::from_euler(0.0, 0.0, rotation.radians())
    }

    pub fn rotate_by(self, other: Self) -> Self {
        Self::from_quaternion(other.q * self.q)
    }

    pub fn relative_to(self, other: Self) -> Self {
        Self::from_quaternion(other.q.inverse() * self.q)
    }

    pub fn quaternion(&self) -> Quaternion {
        self.q
    }

    /// Rotation about the x axis (roll).
    pub fn x(&self) -> f64 {
        let Quaternion { w, x, y, z } = self.q;
        let cxcy = 1.0 - 2.0 * (x * x + y * y);
        let sxcy = 2.0 * (w * x + y * z);
        let cy_sq = cxcy * cxcy + sxcy * sxcy;
        if cy_sq > 1e-20 { sxcy.atan2(cxcy) } else { 0.0 }
    }

    /// Rotation about the y axis (pitch).
    pub fn y(&self) -> f64 {
        let Quaternion { w, x, y, z } = self.q;
        let ratio = 2.0 * (w * y - z * x);
        if ratio.abs() >= 1.0 {
            std::f64::consts::FRAC_PI_2.copysign(ratio)
        } else {
            ratio.asin()
        }
    }

    /// Rotation about the z axis (yaw).
    pub fn z(&self) -> f64 {
        let Quaternion { w, x, y, z } = self.q;
        let cycz = 1.0 - 2.0 * (y * y + z * z);
        let cysz = 2.0 * (w * z + x * y);
        let cy_sq = cycz * cycz + cysz * cysz;
        if cy_sq > 1e-20 {
            cysz.atan2(cycz)
        } else {
            (2.0 * w * z).atan2(w * w - z * z)
        }
    }

    pub fn axis(&self) -> [f64; 3] {
        let q = self.q;
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
        if norm == 0.0 {
            [0.0; 3]
        } else {
            [q.x / norm, q.y / norm, q.z / norm]
        }
    }

    pub fn angle(&self) -> f64 {
        let q = self.q;
        let norm = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
        2.0 * norm.atan2(q.w)
    }

    pub fn to_rotation2d(&self) -> Rotation2d {
        Rotation2d::new(self.z())
    }
}

impl Add for Rotation3d {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        self.rotate_by(other)
    }
}

impl Sub for Rotation3d {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        self.rotate_by(-other)
    }
}

impl Neg for Rotation3d {
    type Output = Self;
    fn neg(self) -> Self {
        Self::from_quaternion(self.q.inverse())
    }
}

impl Mul<f64> for Rotation3d {
    type Output = Self;
    fn mul(self, scalar: f64) -> Self {
        let q = self.q;
        if q.w >= 0.0 {
            Self::from_axis_angle([q.x, q.y, q.z], 2.0 * scalar * q.w.acos())
        } else {
            Self::from_axis_angle([-q.x, -q.y, -q.z], 2.0 * scalar * (-q.w).acos())
        }
    }
}

impl Div<f64> for Rotation3d {
    type Output = Self;
    fn div(self, scalar: f64) -> Self {
        self * (1.0 / scalar)
    }
}

/// Equal when the quaternions represent the same rotation within 1e-9, like WPILib.
impl PartialEq for Rotation3d {
    fn eq(&self, other: &Self) -> bool {
        (self.q.dot(other.q).abs() - self.q.norm() * other.q.norm()).abs() < 1e-9
    }
}

impl Interpolatable for Rotation3d {
    fn interpolate(&self, end: &Self, t: f64) -> Self {
        (*end - *self) * math_util::clamp(t, 0.0, 1.0) + *self
    }
}
