use crate::geometry::Rotation2d;
use crate::interpolation::Interpolatable;
use crate::math_util;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SwerveModuleState {
    pub speed: f64,
    pub angle: Rotation2d,
}

impl SwerveModuleState {
    pub const fn new(speed: f64, angle: Rotation2d) -> Self {
        Self { speed, angle }
    }

    /// Flips the state when that turns the module less than 90 degrees.
    pub fn optimize(&mut self, current_angle: Rotation2d) {
        let delta = self.angle - current_angle;
        if delta.degrees().abs() > 90.0 {
            self.speed *= -1.0;
            self.angle = self.angle.rotate_by(Rotation2d::PI);
        }
    }

    /// Scales speed by the cosine of the steering error.
    pub fn cosine_scale(&mut self, current_angle: Rotation2d) {
        self.speed *= (self.angle - current_angle).cos();
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SwerveModulePosition {
    pub distance: f64,
    pub angle: Rotation2d,
}

impl SwerveModulePosition {
    pub const fn new(distance: f64, angle: Rotation2d) -> Self {
        Self { distance, angle }
    }
}

impl Interpolatable for SwerveModulePosition {
    fn interpolate(&self, end: &Self, t: f64) -> Self {
        Self::new(
            math_util::interpolate(self.distance, end.distance, t),
            self.angle.interpolate(&end.angle, t),
        )
    }
}
