//! A simulated component that other mechanisms attach to. A mounted mechanism reads these
//! values each period to work out its own canvas position.
//!
//! Mounts are read from the `SimLoop` thread, so implementors are `Send + Sync`.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MountType {
    Linear,
    Arm,
}

pub trait Mount: Send + Sync {
    fn mount_type(&self) -> MountType;

    /// Horizontal displacement from where this mount started, in metres.
    fn displacement_x(&self) -> f64;

    /// Vertical displacement from where this mount started, in metres.
    fn displacement_y(&self) -> f64;

    /// Current angle in radians.
    fn angle(&self) -> f64;

    /// Attachment point X in metres.
    fn mount_x(&self) -> f64;

    /// Attachment point Y in metres.
    fn mount_y(&self) -> f64;
}
