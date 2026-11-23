use std::ops::{Add, Div, Mul, Sub};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    pub fn dot(self, other: Self) -> f64 {
        self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn inverse(self) -> Self {
        let norm = self.norm();
        self.conjugate() / (norm * norm)
    }

    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Unit quaternion, or the identity when the norm is zero.
    pub fn normalize(self) -> Self {
        let norm = self.norm();
        if norm == 0.0 {
            Self::IDENTITY
        } else {
            Self::new(self.w / norm, self.x / norm, self.y / norm, self.z / norm)
        }
    }
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Add for Quaternion {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.w + o.w, self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Quaternion {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.w - o.w, self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.w * s, self.x * s, self.y * s, self.z * s)
    }
}

impl Div<f64> for Quaternion {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.w / s, self.x / s, self.y / s, self.z / s)
    }
}

/// Hamilton product.
impl Mul for Quaternion {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let r1 = self.w;
        let r2 = o.w;
        let dot = self.x * o.x + self.y * o.y + self.z * o.z;
        let cross_x = self.y * o.z - o.y * self.z;
        let cross_y = o.x * self.z - self.x * o.z;
        let cross_z = self.x * o.y - o.x * self.y;
        Self::new(
            r1 * r2 - dot,
            r1 * o.x + r2 * self.x + cross_x,
            r1 * o.y + r2 * self.y + cross_y,
            r1 * o.z + r2 * self.z + cross_z,
        )
    }
}
