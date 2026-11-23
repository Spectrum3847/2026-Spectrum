/// Constant-curvature displacement: forward, sideways, and heading change.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Twist2d {
    pub dx: f64,
    pub dy: f64,
    pub dtheta: f64,
}

impl Twist2d {
    pub const fn new(dx: f64, dy: f64, dtheta: f64) -> Self {
        Self { dx, dy, dtheta }
    }
}
