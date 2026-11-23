use crate::geometry::{Pose2d, Rotation2d, Translation2d, Twist2d};
use std::ops::{Add, Div, Mul, Neg, Sub};

/// Robot velocity: forward `vx` and sideways `vy` in m/s, `omega` in rad/s.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ChassisSpeeds {
    pub vx: f64,
    pub vy: f64,
    pub omega: f64,
}

impl ChassisSpeeds {
    pub const fn new(vx: f64, vy: f64, omega: f64) -> Self {
        Self { vx, vy, omega }
    }

    pub fn to_twist2d(self, dt_seconds: f64) -> Twist2d {
        Twist2d::new(
            self.vx * dt_seconds,
            self.vy * dt_seconds,
            self.omega * dt_seconds,
        )
    }

    /// Corrects for drift when the continuous speeds are applied over `dt_seconds`.
    pub fn discretize(self, dt_seconds: f64) -> Self {
        let desired_delta_pose = Pose2d::from_xy(
            self.vx * dt_seconds,
            self.vy * dt_seconds,
            Rotation2d::new(self.omega * dt_seconds),
        );
        let twist = Pose2d::ZERO.log(desired_delta_pose);
        Self::new(
            twist.dx / dt_seconds,
            twist.dy / dt_seconds,
            twist.dtheta / dt_seconds,
        )
    }

    /// Converts field-relative speeds to robot-relative.
    pub fn from_field_relative(speeds: Self, robot_angle: Rotation2d) -> Self {
        let rotated = Translation2d::new(speeds.vx, speeds.vy).rotate_by(-robot_angle);
        Self::new(rotated.x, rotated.y, speeds.omega)
    }

    /// Converts robot-relative speeds to field-relative.
    pub fn from_robot_relative(speeds: Self, robot_angle: Rotation2d) -> Self {
        let rotated = Translation2d::new(speeds.vx, speeds.vy).rotate_by(robot_angle);
        Self::new(rotated.x, rotated.y, speeds.omega)
    }
}

impl Add for ChassisSpeeds {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.vx + o.vx, self.vy + o.vy, self.omega + o.omega)
    }
}

impl Sub for ChassisSpeeds {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.vx - o.vx, self.vy - o.vy, self.omega - o.omega)
    }
}

impl Neg for ChassisSpeeds {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.vx, -self.vy, -self.omega)
    }
}

impl Mul<f64> for ChassisSpeeds {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.vx * s, self.vy * s, self.omega * s)
    }
}

impl Div<f64> for ChassisSpeeds {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.vx / s, self.vy / s, self.omega / s)
    }
}
