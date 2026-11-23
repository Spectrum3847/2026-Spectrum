use crate::flipping::FlippingUtil;
use commands::Command;
use wpimath::geometry::{Rotation2d, Translation2d};

/// Kinematic limits for following a path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathConstraints {
    pub max_velocity_mps: f64,
    pub max_acceleration_mps_sq: f64,
    pub max_angular_velocity_rad_per_sec: f64,
    pub max_angular_acceleration_rad_per_sec_sq: f64,
    pub nominal_voltage_volts: f64,
    pub unlimited: bool,
}

impl PathConstraints {
    /// Limited constraints for a 12 V battery.
    pub const fn new(
        max_velocity_mps: f64,
        max_acceleration_mps_sq: f64,
        max_angular_velocity_rad_per_sec: f64,
        max_angular_acceleration_rad_per_sec_sq: f64,
    ) -> Self {
        Self {
            max_velocity_mps,
            max_acceleration_mps_sq,
            max_angular_velocity_rad_per_sec,
            max_angular_acceleration_rad_per_sec_sq,
            nominal_voltage_volts: 12.0,
            unlimited: false,
        }
    }

    pub const fn with_nominal_voltage(mut self, volts: f64) -> Self {
        self.nominal_voltage_volts = volts;
        self
    }

    pub const fn unlimited_constraints(nominal_voltage: f64) -> Self {
        Self {
            max_velocity_mps: f64::INFINITY,
            max_acceleration_mps_sq: f64::INFINITY,
            max_angular_velocity_rad_per_sec: f64::INFINITY,
            max_angular_acceleration_rad_per_sec_sq: f64::INFINITY,
            nominal_voltage_volts: nominal_voltage,
            unlimited: true,
        }
    }
}

/// A zone of a path with different constraints. Positions are waypoint relative.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConstraintsZone {
    pub min_position: f64,
    pub max_position: f64,
    pub constraints: PathConstraints,
}

/// Where the robot should end up when it finishes a path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GoalEndState {
    pub velocity_mps: f64,
    pub rotation: Rotation2d,
}

impl GoalEndState {
    pub const fn new(velocity_mps: f64, rotation: Rotation2d) -> Self {
        Self {
            velocity_mps,
            rotation,
        }
    }

    pub fn flip(&self) -> Self {
        Self::new(
            self.velocity_mps,
            FlippingUtil::flip_field_rotation(self.rotation),
        )
    }
}

/// The state the robot is ideally in when it starts a path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IdealStartingState {
    pub velocity_mps: f64,
    pub rotation: Rotation2d,
}

impl IdealStartingState {
    pub const fn new(velocity_mps: f64, rotation: Rotation2d) -> Self {
        Self {
            velocity_mps,
            rotation,
        }
    }

    pub fn flip(&self) -> Self {
        Self::new(
            self.velocity_mps,
            FlippingUtil::flip_field_rotation(self.rotation),
        )
    }
}

/// A target holonomic rotation at a waypoint relative position.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotationTarget {
    pub position: f64,
    pub rotation: Rotation2d,
}

impl RotationTarget {
    pub fn flip(&self) -> Self {
        Self {
            position: self.position,
            rotation: FlippingUtil::flip_field_rotation(self.rotation),
        }
    }
}

/// A zone where the robot points at a field position.
#[derive(Clone, Debug, PartialEq)]
pub struct PointTowardsZone {
    pub name: String,
    pub target_position: Translation2d,
    /// Added to the angle to the target, for example 180 degrees to point away.
    pub rotation_offset: Rotation2d,
    pub min_position: f64,
    pub max_position: f64,
}

impl PointTowardsZone {
    pub fn flip(&self) -> Self {
        Self {
            name: self.name.clone(),
            target_position: FlippingUtil::flip_field_position(self.target_position),
            rotation_offset: self.rotation_offset,
            min_position: self.min_position,
            max_position: self.max_position,
        }
    }
}

/// A position along a path that controls an event trigger and can run a command.
#[derive(Clone, Debug)]
pub struct EventMarker {
    pub trigger_name: String,
    pub position: f64,
    /// End of the marker's zone, or -1.0 for a marker that is not zoned.
    pub end_position: f64,
    pub command: Option<Command>,
}

impl EventMarker {
    pub fn new(trigger_name: impl Into<String>, position: f64) -> Self {
        Self {
            trigger_name: trigger_name.into(),
            position,
            end_position: -1.0,
            command: None,
        }
    }
}

/// A waypoint of a Bezier curve path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint {
    pub prev_control: Option<Translation2d>,
    pub anchor: Translation2d,
    pub next_control: Option<Translation2d>,
}

const AUTO_CONTROL_DISTANCE_FACTOR: f64 = 0.4;

impl Waypoint {
    pub fn flip(&self) -> Self {
        Self {
            prev_control: self.prev_control.map(FlippingUtil::flip_field_position),
            anchor: FlippingUtil::flip_field_position(self.anchor),
            next_control: self.next_control.map(FlippingUtil::flip_field_position),
        }
    }

    /// Control points placed from the neighboring anchors, as `waypointsFromPoses` does.
    pub(crate) fn auto_control_points(
        anchor: Translation2d,
        heading: Rotation2d,
        prev_anchor: Option<Translation2d>,
        next_anchor: Option<Translation2d>,
    ) -> Self {
        let prev_control = prev_anchor.map(|prev| {
            let d = anchor.distance(prev) * AUTO_CONTROL_DISTANCE_FACTOR;
            anchor - Translation2d::from_polar(d, heading)
        });
        let next_control = next_anchor.map(|next| {
            let d = anchor.distance(next) * AUTO_CONTROL_DISTANCE_FACTOR;
            anchor + Translation2d::from_polar(d, heading)
        });
        Self {
            prev_control,
            anchor,
            next_control,
        }
    }
}

/// A sampled point along a path.
#[derive(Clone, Copy, Debug)]
pub struct PathPoint {
    pub position: Translation2d,
    /// Distance from the start of the path in meters.
    pub distance_along_path: f64,
    pub max_v: f64,
    pub rotation_target: Option<RotationTarget>,
    /// Set for every point once the path is built.
    pub constraints: Option<PathConstraints>,
    pub waypoint_relative_pos: f64,
}

impl PathPoint {
    pub(crate) fn new(
        position: Translation2d,
        rotation_target: Option<RotationTarget>,
        constraints: Option<PathConstraints>,
    ) -> Self {
        Self {
            position,
            distance_along_path: 0.0,
            max_v: f64::INFINITY,
            rotation_target,
            constraints,
            waypoint_relative_pos: 0.0,
        }
    }

    pub fn flip(&self) -> Self {
        Self {
            position: FlippingUtil::flip_field_position(self.position),
            rotation_target: self.rotation_target.map(|t| t.flip()),
            ..*self
        }
    }
}
