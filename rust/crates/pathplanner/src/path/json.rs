use super::types::{
    ConstraintsZone, EventMarker, GoalEndState, IdealStartingState, PathConstraints,
    PointTowardsZone, RotationTarget, Waypoint,
};
use crate::command_util;
use serde::Deserialize;
use serde_json::Value;
use wpimath::geometry::{Rotation2d, Translation2d};
use wpimath::units::degrees_to_radians;

#[derive(Deserialize)]
struct Xy {
    x: f64,
    y: f64,
}

impl From<Xy> for Translation2d {
    fn from(p: Xy) -> Self {
        Translation2d::new(p.x, p.y)
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct WaypointJson {
    anchor: Xy,
    prev_control: Option<Xy>,
    next_control: Option<Xy>,
}

impl From<WaypointJson> for Waypoint {
    fn from(w: WaypointJson) -> Self {
        Waypoint {
            prev_control: w.prev_control.map(Into::into),
            anchor: w.anchor.into(),
            next_control: w.next_control.map(Into::into),
        }
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RotationTargetJson {
    waypoint_relative_pos: f64,
    rotation_degrees: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ConstraintsJson {
    max_velocity: f64,
    max_acceleration: f64,
    max_angular_velocity: f64,
    max_angular_acceleration: f64,
    nominal_voltage: f64,
    unlimited: bool,
}

impl From<ConstraintsJson> for PathConstraints {
    fn from(c: ConstraintsJson) -> Self {
        PathConstraints {
            max_velocity_mps: c.max_velocity,
            max_acceleration_mps_sq: c.max_acceleration,
            max_angular_velocity_rad_per_sec: degrees_to_radians(c.max_angular_velocity),
            max_angular_acceleration_rad_per_sec_sq: degrees_to_radians(c.max_angular_acceleration),
            nominal_voltage_volts: c.nominal_voltage,
            unlimited: c.unlimited,
        }
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ConstraintsZoneJson {
    min_waypoint_relative_pos: f64,
    max_waypoint_relative_pos: f64,
    constraints: ConstraintsJson,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct PointTowardsZoneJson {
    name: String,
    field_position: Xy,
    rotation_offset: f64,
    min_waypoint_relative_pos: f64,
    max_waypoint_relative_pos: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct EventMarkerJson {
    name: String,
    waypoint_relative_pos: f64,
    end_waypoint_relative_pos: Option<f64>,
    command: Option<Value>,
}

#[derive(Deserialize)]
struct StateJson {
    velocity: f64,
    rotation: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub(super) struct PathJson {
    pub version: Value,
    waypoints: Vec<WaypointJson>,
    #[serde(default)]
    rotation_targets: Vec<RotationTargetJson>,
    #[serde(default)]
    point_towards_zones: Vec<PointTowardsZoneJson>,
    #[serde(default)]
    constraint_zones: Vec<ConstraintsZoneJson>,
    #[serde(default)]
    event_markers: Vec<EventMarkerJson>,
    global_constraints: ConstraintsJson,
    ideal_starting_state: Option<StateJson>,
    goal_end_state: StateJson,
    #[serde(default)]
    pub reversed: bool,
}

/// The parts of a path file, converted to path types.
pub(super) struct PathParts {
    pub waypoints: Vec<Waypoint>,
    pub rotation_targets: Vec<RotationTarget>,
    pub point_towards_zones: Vec<PointTowardsZone>,
    pub constraint_zones: Vec<ConstraintsZone>,
    pub event_markers: Vec<EventMarker>,
    pub global_constraints: PathConstraints,
    pub ideal_starting_state: Option<IdealStartingState>,
    pub goal_end_state: GoalEndState,
    pub reversed: bool,
}

impl PathJson {
    pub fn into_parts(self) -> PathParts {
        PathParts {
            waypoints: self.waypoints.into_iter().map(Into::into).collect(),
            rotation_targets: self
                .rotation_targets
                .into_iter()
                .map(|t| RotationTarget {
                    position: t.waypoint_relative_pos,
                    rotation: Rotation2d::from_degrees(t.rotation_degrees),
                })
                .collect(),
            point_towards_zones: self
                .point_towards_zones
                .into_iter()
                .map(|z| PointTowardsZone {
                    name: z.name,
                    target_position: z.field_position.into(),
                    rotation_offset: Rotation2d::from_degrees(z.rotation_offset),
                    min_position: z.min_waypoint_relative_pos,
                    max_position: z.max_waypoint_relative_pos,
                })
                .collect(),
            constraint_zones: self
                .constraint_zones
                .into_iter()
                .map(|z| ConstraintsZone {
                    min_position: z.min_waypoint_relative_pos,
                    max_position: z.max_waypoint_relative_pos,
                    constraints: z.constraints.into(),
                })
                .collect(),
            event_markers: self
                .event_markers
                .into_iter()
                .map(|m| EventMarker {
                    trigger_name: m.name,
                    position: m.waypoint_relative_pos,
                    end_position: m.end_waypoint_relative_pos.unwrap_or(-1.0),
                    // A marker command that cannot be built is dropped, as in Java.
                    command: m
                        .command
                        .and_then(|json| command_util::command_from_json(&json, false).ok()),
                })
                .collect(),
            global_constraints: self.global_constraints.into(),
            ideal_starting_state: self
                .ideal_starting_state
                .map(|s| IdealStartingState::new(s.velocity, Rotation2d::from_degrees(s.rotation))),
            goal_end_state: GoalEndState::new(
                self.goal_end_state.velocity,
                Rotation2d::from_degrees(self.goal_end_state.rotation),
            ),
            reversed: self.reversed,
        }
    }
}
