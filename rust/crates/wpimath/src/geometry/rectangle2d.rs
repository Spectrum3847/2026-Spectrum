use super::{Pose2d, Rotation2d, Transform2d, Translation2d};
use crate::math_util::clamp;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rectangle2d {
    center: Pose2d,
    x_width: f64,
    y_width: f64,
}

impl Rectangle2d {
    /// # Panics
    /// If a width is negative.
    pub fn new(center: Pose2d, x_width: f64, y_width: f64) -> Self {
        assert!(
            x_width >= 0.0 && y_width >= 0.0,
            "Rectangle2d dimensions cannot be less than 0!"
        );
        Self {
            center,
            x_width,
            y_width,
        }
    }

    /// Axis-aligned rectangle with opposite corners `a` and `b`.
    pub fn from_corners(a: Translation2d, b: Translation2d) -> Self {
        Self::new(
            Pose2d::new((a + b) / 2.0, Rotation2d::ZERO),
            (a.x - b.x).abs(),
            (a.y - b.y).abs(),
        )
    }

    pub fn center(&self) -> Pose2d {
        self.center
    }

    pub fn rotation(&self) -> Rotation2d {
        self.center.rotation
    }

    pub fn x_width(&self) -> f64 {
        self.x_width
    }

    pub fn y_width(&self) -> f64 {
        self.y_width
    }

    pub fn transform_by(self, other: Transform2d) -> Self {
        Self::new(self.center.transform_by(other), self.x_width, self.y_width)
    }

    pub fn rotate_by(self, other: Rotation2d) -> Self {
        Self::new(self.center.rotate_by(other), self.x_width, self.y_width)
    }

    /// True when `point` is on the rectangle's border.
    pub fn intersects(&self, point: Translation2d) -> bool {
        let point = (point - self.center.translation).rotate_by(-self.center.rotation);
        if (point.x.abs() - self.x_width / 2.0).abs() <= 1e-9 {
            point.y.abs() <= self.y_width / 2.0
        } else if (point.y.abs() - self.y_width / 2.0).abs() <= 1e-9 {
            point.x.abs() <= self.x_width / 2.0
        } else {
            false
        }
    }

    /// True when `point` is inside or on the border.
    pub fn contains(&self, point: Translation2d) -> bool {
        let point = point.rotate_around(self.center.translation, -self.center.rotation);
        point.x >= self.center.x() - self.x_width / 2.0
            && point.x <= self.center.x() + self.x_width / 2.0
            && point.y >= self.center.y() - self.y_width / 2.0
            && point.y <= self.center.y() + self.y_width / 2.0
    }

    pub fn distance(&self, point: Translation2d) -> f64 {
        self.nearest(point).distance(point)
    }

    pub fn nearest(&self, point: Translation2d) -> Translation2d {
        if self.contains(point) {
            return point;
        }
        let point = point.rotate_around(self.center.translation, -self.center.rotation);
        let clamped = Translation2d::new(
            clamp(
                point.x,
                self.center.x() - self.x_width / 2.0,
                self.center.x() + self.x_width / 2.0,
            ),
            clamp(
                point.y,
                self.center.y() - self.y_width / 2.0,
                self.center.y() + self.y_width / 2.0,
            ),
        );
        clamped.rotate_around(self.center.translation, self.center.rotation)
    }
}
