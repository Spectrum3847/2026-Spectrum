mod json;
mod types;

pub use types::{
    ConstraintsZone, EventMarker, GoalEndState, IdealStartingState, PathConstraints, PathPoint,
    PointTowardsZone, RotationTarget, Waypoint,
};

use crate::config::RobotConfig;
use crate::error::{Error, Result};
use crate::flipping::FlippingUtil;
use crate::geometry_util::{calculate_radius, cubic_lerp};
use crate::trajectory::{PathPlannerTrajectory, PathPlannerTrajectoryState};
use crate::util::deploy_directory;
use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::rc::Rc;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::kinematics::ChassisSpeeds;
use wpimath::math_util;

const TARGET_INCREMENT: f64 = 0.05;
const TARGET_SPACING: f64 = 0.2;

thread_local! {
    static PATH_CACHE: RefCell<HashMap<String, Rc<PathPlannerPath>>> = RefCell::new(HashMap::new());
}

/// A PathPlanner path. Not a trajectory: it is not followed directly.
#[derive(Debug)]
pub struct PathPlannerPath {
    /// Name of the path file. Empty for paths that are not loaded from a file.
    pub name: String,
    waypoints: Vec<Waypoint>,
    rotation_targets: Vec<RotationTarget>,
    point_towards_zones: Vec<PointTowardsZone>,
    constraint_zones: Vec<ConstraintsZone>,
    event_markers: Vec<EventMarker>,
    global_constraints: PathConstraints,
    ideal_starting_state: Option<IdealStartingState>,
    goal_end_state: GoalEndState,
    all_points: Vec<PathPoint>,
    reversed: bool,
    ideal_trajectory: RefCell<Option<Rc<PathPlannerTrajectory>>>,
    /// Stops this path from being flipped, for on-the-fly paths that already have the right
    /// coordinates.
    pub prevent_flipping: bool,
}

impl PathPlannerPath {
    /// Creates a path from waypoints. Use [`waypoints_from_poses`](Self::waypoints_from_poses)
    /// to make waypoints for an on-the-fly path.
    ///
    /// # Panics
    /// With fewer than 2 waypoints, or a waypoint without the control points its neighbors need.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        waypoints: Vec<Waypoint>,
        mut rotation_targets: Vec<RotationTarget>,
        point_towards_zones: Vec<PointTowardsZone>,
        constraint_zones: Vec<ConstraintsZone>,
        mut event_markers: Vec<EventMarker>,
        global_constraints: PathConstraints,
        ideal_starting_state: Option<IdealStartingState>,
        goal_end_state: GoalEndState,
        reversed: bool,
    ) -> Self {
        assert!(
            waypoints.len() >= 2,
            "A path must have at least 2 waypoints"
        );
        for (i, w) in waypoints.iter().enumerate() {
            assert!(
                i == 0 || w.prev_control.is_some(),
                "Waypoint {i} needs a previous control point"
            );
            assert!(
                i == waypoints.len() - 1 || w.next_control.is_some(),
                "Waypoint {i} needs a next control point"
            );
        }
        rotation_targets.sort_by(|a, b| a.position.total_cmp(&b.position));
        event_markers.sort_by(|a, b| a.position.total_cmp(&b.position));

        let mut path = Self {
            name: String::new(),
            waypoints,
            rotation_targets,
            point_towards_zones,
            constraint_zones,
            event_markers,
            global_constraints,
            ideal_starting_state,
            goal_end_state,
            all_points: Vec::new(),
            reversed,
            ideal_trajectory: RefCell::new(None),
            prevent_flipping: false,
        };
        path.all_points = path.create_path();
        path.precalc_values();
        path
    }

    /// A path with no rotation targets, zones or event markers.
    pub fn simple(
        waypoints: Vec<Waypoint>,
        constraints: PathConstraints,
        ideal_starting_state: Option<IdealStartingState>,
        goal_end_state: GoalEndState,
    ) -> Self {
        Self::new(
            waypoints,
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            constraints,
            ideal_starting_state,
            goal_end_state,
            false,
        )
    }

    /// Bezier waypoints that pass through the poses, one waypoint per pose.
    ///
    /// # Panics
    /// With fewer than 2 poses.
    pub fn waypoints_from_poses(poses: &[Pose2d]) -> Vec<Waypoint> {
        assert!(poses.len() >= 2, "Not enough poses");
        let n = poses.len();
        let mut waypoints = Vec::with_capacity(n);

        waypoints.push(Waypoint::auto_control_points(
            poses[0].translation,
            poses[0].rotation,
            None,
            Some(poses[1].translation),
        ));
        for i in 1..n - 1 {
            waypoints.push(Waypoint::auto_control_points(
                poses[i].translation,
                poses[i].rotation,
                Some(poses[i - 1].translation),
                Some(poses[i + 1].translation),
            ));
        }
        waypoints.push(Waypoint::auto_control_points(
            poses[n - 1].translation,
            poses[n - 1].rotation,
            Some(poses[n - 2].translation),
            None,
        ));
        waypoints
    }

    /// Loads `pathplanner/paths/<name>.path` from the deploy folder. Loaded paths are cached,
    /// so the same name gives the same path.
    pub fn from_path_file(path_name: &str) -> Result<Rc<PathPlannerPath>> {
        if let Some(path) = PATH_CACHE.with(|c| c.borrow().get(path_name).cloned()) {
            return Ok(path);
        }

        let file = deploy_directory()
            .join("pathplanner/paths")
            .join(format!("{path_name}.path"));
        let text = std::fs::read_to_string(file)?;
        let json: json::PathJson = serde_json::from_str(&text)?;

        let version = match &json.version {
            serde_json::Value::String(s) => s.clone(),
            other => other.to_string(),
        };
        if version.split('.').next() != Some("2025") {
            return Err(Error::Version {
                found: version,
                file: format!("{path_name}.path"),
            });
        }

        let parts = json.into_parts();
        validate_waypoints(&parts.waypoints, path_name)?;
        let mut path = PathPlannerPath::new(
            parts.waypoints,
            parts.rotation_targets,
            parts.point_towards_zones,
            parts.constraint_zones,
            parts.event_markers,
            parts.global_constraints,
            parts.ideal_starting_state,
            parts.goal_end_state,
            parts.reversed,
        );
        path.name = path_name.to_string();

        let path = Rc::new(path);
        PATH_CACHE.with(|c| c.borrow_mut().insert(path_name.to_string(), path.clone()));
        Ok(path)
    }

    /// Forgets the loaded paths.
    pub fn clear_cache() {
        PATH_CACHE.with(|c| c.borrow_mut().clear());
    }

    /// The trajectory from the ideal starting state, generated on first use. It is cached
    /// whatever `config` was the first time. Empty when the path has no ideal starting state.
    pub fn get_ideal_trajectory(&self, config: &RobotConfig) -> Option<Rc<PathPlannerTrajectory>> {
        let missing = self.ideal_trajectory.borrow().is_none();
        if missing && let Some(ideal) = self.ideal_starting_state {
            let heading = self.get_initial_heading();
            let field_speeds = Translation2d::from_polar(ideal.velocity_mps, heading);
            let starting_speeds = ChassisSpeeds::from_field_relative(
                ChassisSpeeds::new(field_speeds.x, field_speeds.y, 0.0),
                ideal.rotation,
            );
            let trajectory = self.generate_trajectory(starting_speeds, ideal.rotation, config);
            *self.ideal_trajectory.borrow_mut() = Some(Rc::new(trajectory));
        }
        self.ideal_trajectory.borrow().clone()
    }

    /// The direction of travel at the start of the path.
    pub fn get_initial_heading(&self) -> Rotation2d {
        (self.point(1).position - self.point(0).position).angle()
    }

    /// The start pose for a differential drive.
    pub fn get_starting_differential_pose(&self) -> Pose2d {
        let start = self.point(0).position;
        let mut heading = self.get_initial_heading();
        if self.reversed {
            heading = Rotation2d::from_degrees(math_util::input_modulus(
                heading.degrees() + 180.0,
                -180.0,
                180.0,
            ));
        }
        Pose2d::new(start, heading)
    }

    /// The start pose for a holonomic drive. Empty without an ideal starting state.
    pub fn get_starting_holonomic_pose(&self) -> Option<Pose2d> {
        let ideal = self.ideal_starting_state?;
        Some(Pose2d::new(self.point(0).position, ideal.rotation))
    }

    fn constraints_for_waypoint_pos(&self, pos: f64) -> PathConstraints {
        for z in &self.constraint_zones {
            if pos >= z.min_position && pos <= z.max_position {
                return z.constraints;
            }
        }

        if self.global_constraints.unlimited {
            return PathConstraints::unlimited_constraints(
                self.global_constraints.nominal_voltage_volts,
            );
        }

        self.global_constraints
    }

    fn point_zone_for_waypoint_pos(&self, pos: f64) -> Option<&PointTowardsZone> {
        self.point_towards_zones
            .iter()
            .find(|z| pos >= z.min_position && pos <= z.max_position)
    }

    fn sample_path(&self, waypoint_relative_pos: f64) -> Translation2d {
        let pos = math_util::clamp(
            waypoint_relative_pos,
            0.0,
            self.waypoints.len() as f64 - 1.0,
        );

        let mut i = pos.floor() as usize;
        if i == self.waypoints.len() - 1 {
            i -= 1;
        }

        let t = pos - i as f64;

        let p1 = self.waypoints[i].anchor;
        let p2 = self.waypoints[i].next_control.expect("checked in new");
        let p3 = self.waypoints[i + 1].prev_control.expect("checked in new");
        let p4 = self.waypoints[i + 1].anchor;
        cubic_lerp(p1, p2, p3, p4, t)
    }

    fn create_path(&self) -> Vec<PathPoint> {
        let mut unadded_targets: VecDeque<RotationTarget> =
            self.rotation_targets.iter().copied().collect();
        let mut points: Vec<PathPoint> = Vec::new();
        let num_segments = (self.waypoints.len() - 1) as f64;

        let make_point = |position: Translation2d,
                          target: Option<RotationTarget>,
                          pos: f64,
                          constraints: PathConstraints| {
            let mut point = PathPoint::new(position, target, Some(constraints));
            point.waypoint_relative_pos = pos;
            point
        };

        points.push(make_point(
            self.sample_path(0.0),
            None,
            0.0,
            self.constraints_for_waypoint_pos(0.0),
        ));

        let mut pos = TARGET_INCREMENT;
        while pos < num_segments {
            let mut position = self.sample_path(pos);

            let distance = points[points.len() - 1].position.distance(position);
            if distance <= 0.01 {
                pos = f64::min(pos + TARGET_INCREMENT, num_segments);
                continue;
            }

            let prev_waypoint_pos = pos - TARGET_INCREMENT;

            let delta = distance - TARGET_SPACING;
            if delta > TARGET_SPACING * 0.25 {
                // Points are too far apart, increment pos by the correct amount.
                let correct_increment = (TARGET_SPACING * TARGET_INCREMENT) / distance;
                pos = pos - TARGET_INCREMENT + correct_increment;

                position = self.sample_path(pos);

                if points[points.len() - 1].position.distance(position) - TARGET_SPACING
                    > TARGET_SPACING * 0.25
                {
                    // Still too far apart, probably from odd control point placement. Halve the
                    // increment and hope for the best.
                    pos -= correct_increment * 0.5;
                    position = self.sample_path(pos);
                }
            } else if delta < -TARGET_SPACING * 0.25 {
                // Points are too close, increment pos by the correct amount.
                let correct_increment = (TARGET_SPACING * TARGET_INCREMENT) / distance;
                pos = pos - TARGET_INCREMENT + correct_increment;

                position = self.sample_path(pos);

                if points[points.len() - 1].position.distance(position) - TARGET_SPACING
                    < -TARGET_SPACING * 0.25
                {
                    pos += correct_increment * 0.5;
                    position = self.sample_path(pos);
                }
            }

            // Rotation targets.
            let mut target = None;
            let prev_point = points.len() - 1;

            while let Some(front) = unadded_targets.front().copied() {
                if !(front.position >= prev_waypoint_pos && front.position <= pos) {
                    break;
                }
                if (front.position - prev_waypoint_pos).abs() < 0.001 {
                    // Close enough to the previous point.
                    points[prev_point].rotation_target = unadded_targets.pop_front();
                } else if (front.position - pos).abs() < 0.001 {
                    // Close enough to the next point.
                    target = unadded_targets.pop_front();
                } else {
                    // Insert a point at the exact position.
                    let t = front;
                    unadded_targets.pop_front();
                    points.push(make_point(
                        self.sample_path(t.position),
                        Some(t),
                        t.position,
                        self.constraints_for_waypoint_pos(t.position),
                    ));
                }
            }

            points.push(make_point(
                position,
                target,
                pos,
                self.constraints_for_waypoint_pos(pos),
            ));
            pos = f64::min(pos + TARGET_INCREMENT, num_segments);
        }

        // Keep trying to add the end point until it is close enough to the previous point.
        let mut true_increment = num_segments - (pos - TARGET_INCREMENT);
        pos = num_segments;
        let mut invalid = true;
        while invalid {
            let mut position = self.sample_path(pos);

            let distance = points[points.len() - 1].position.distance(position);
            if distance <= 0.01 {
                // Make sure there is at least a second point.
                if points.len() < 2 {
                    points.push(make_point(
                        position,
                        None,
                        pos,
                        self.constraints_for_waypoint_pos(pos),
                    ));
                }
                break;
            }

            let prev_pos = pos - true_increment;

            let delta = distance - TARGET_SPACING;
            if delta > TARGET_SPACING * 0.25 {
                let correct_increment = (TARGET_SPACING * true_increment) / distance;
                pos = pos - true_increment + correct_increment;
                true_increment = correct_increment;

                position = self.sample_path(pos);

                if points[points.len() - 1].position.distance(position) - TARGET_SPACING
                    > TARGET_SPACING * 0.25
                {
                    pos -= correct_increment * 0.5;
                    true_increment = correct_increment * 0.5;
                    position = self.sample_path(pos);
                }
            } else {
                invalid = false;
            }

            // Give the previous point the rotation target if it is closer to it than this one.
            if let Some(front) = unadded_targets.front().copied()
                && (front.position - prev_pos).abs() <= (front.position - pos).abs()
            {
                let last = points.len() - 1;
                points[last].rotation_target = unadded_targets.pop_front();
            }

            points.push(make_point(
                position,
                None,
                pos,
                self.constraints_for_waypoint_pos(pos),
            ));
            pos = num_segments;
        }

        let mut i = 1;
        while i < points.len() - 1 {
            // Point towards zones set the rotation target.
            if let Some(zone) = self.point_zone_for_waypoint_pos(points[i].waypoint_relative_pos) {
                let angle_to_target = (zone.target_position - points[i].position).angle();
                let rotation = angle_to_target + zone.rotation_offset;
                points[i].rotation_target = Some(RotationTarget {
                    position: points[i].waypoint_relative_pos,
                    rotation,
                });
            }

            let curve_radius = calculate_radius(
                points[i - 1].position,
                points[i].position,
                points[i + 1].position,
            );

            if !curve_radius.is_finite() {
                i += 1;
                continue;
            }

            let sample_between = |a: usize, b: usize, t: f64, points: &[PathPoint]| {
                math_util::interpolate(
                    points[a].waypoint_relative_pos,
                    points[b].waypoint_relative_pos,
                    t,
                )
            };

            if curve_radius.abs() < 0.25 {
                // Curve radius is too tight for the default spacing, insert 4 more points.
                let before1_pos = sample_between(i - 1, i, 0.33, &points);
                let before2_pos = sample_between(i - 1, i, 0.67, &points);
                let after1_pos = sample_between(i, i + 1, 0.33, &points);
                let after2_pos = sample_between(i, i + 1, 0.67, &points);

                let constraints = points[i].constraints;
                let extra = |pos: f64| {
                    let mut point = PathPoint::new(self.sample_path(pos), None, constraints);
                    point.waypoint_relative_pos = pos;
                    point
                };
                let (before1, before2, after1, after2) = (
                    extra(before1_pos),
                    extra(before2_pos),
                    extra(after1_pos),
                    extra(after2_pos),
                );

                points.insert(i, before2);
                points.insert(i, before1);
                points.insert(i + 3, after2);
                points.insert(i + 3, after1);
                i += 4;
            } else if curve_radius.abs() < 0.5 {
                // Curve radius is too tight for the default spacing, insert 2 more points.
                let before_pos = sample_between(i - 1, i, 0.5, &points);
                let after_pos = sample_between(i, i + 1, 0.5, &points);

                let constraints = points[i].constraints;
                let extra = |pos: f64| {
                    let mut point = PathPoint::new(self.sample_path(pos), None, constraints);
                    point.waypoint_relative_pos = pos;
                    point
                };
                let (before, after) = (extra(before_pos), extra(after_pos));

                points.insert(i, before);
                points.insert(i + 2, after);
                i += 2;
            }
            i += 1;
        }

        points
    }

    fn precalc_values(&mut self) {
        if self.all_points.is_empty() {
            return;
        }

        for i in 0..self.all_points.len() {
            let constraints = self.all_points[i]
                .constraints
                .unwrap_or(self.global_constraints);
            let curve_radius = curve_radius_at_point(i, &self.all_points);

            let max_v = if curve_radius.is_finite() {
                f64::min(
                    (constraints.max_acceleration_mps_sq * curve_radius.abs()).sqrt(),
                    constraints.max_velocity_mps,
                )
            } else {
                constraints.max_velocity_mps
            };
            let distance_along_path = if i != 0 {
                self.all_points[i - 1].distance_along_path
                    + self.all_points[i - 1]
                        .position
                        .distance(self.all_points[i].position)
            } else {
                self.all_points[i].distance_along_path
            };

            let point = &mut self.all_points[i];
            point.constraints = Some(constraints);
            point.max_v = max_v;
            point.distance_along_path = distance_along_path;
        }

        let last = self.all_points.len() - 1;
        self.all_points[last].rotation_target = Some(RotationTarget {
            position: -1.0,
            rotation: self.goal_end_state.rotation,
        });
        self.all_points[last].max_v = self.goal_end_state.velocity_mps;
    }

    pub fn waypoints(&self) -> &[Waypoint] {
        &self.waypoints
    }

    pub fn rotation_targets(&self) -> &[RotationTarget] {
        &self.rotation_targets
    }

    pub fn point_towards_zones(&self) -> &[PointTowardsZone] {
        &self.point_towards_zones
    }

    pub fn constraint_zones(&self) -> &[ConstraintsZone] {
        &self.constraint_zones
    }

    pub fn event_markers(&self) -> &[EventMarker] {
        &self.event_markers
    }

    pub fn get_all_path_points(&self) -> &[PathPoint] {
        &self.all_points
    }

    pub fn num_points(&self) -> usize {
        self.all_points.len()
    }

    pub fn point(&self, index: usize) -> &PathPoint {
        &self.all_points[index]
    }

    pub fn global_constraints(&self) -> &PathConstraints {
        &self.global_constraints
    }

    pub fn goal_end_state(&self) -> &GoalEndState {
        &self.goal_end_state
    }

    pub fn ideal_starting_state(&self) -> Option<&IdealStartingState> {
        self.ideal_starting_state.as_ref()
    }

    /// Always false for the holonomic drive the robot has.
    pub fn is_reversed(&self) -> bool {
        self.reversed
    }

    /// Generates a trajectory. `starting_speeds` are robot relative.
    pub fn generate_trajectory(
        &self,
        starting_speeds: ChassisSpeeds,
        starting_rotation: Rotation2d,
        config: &RobotConfig,
    ) -> PathPlannerTrajectory {
        PathPlannerTrajectory::generate(self, starting_speeds, starting_rotation, config)
    }

    /// This path for the other side of the field, keeping a global blue origin.
    pub fn flip_path(&self) -> PathPlannerPath {
        let flipped_trajectory = self
            .ideal_trajectory
            .borrow()
            .as_ref()
            .map(|t| Rc::new(t.flip()));

        PathPlannerPath {
            name: self.name.clone(),
            waypoints: self.waypoints.iter().map(Waypoint::flip).collect(),
            rotation_targets: self
                .rotation_targets
                .iter()
                .map(RotationTarget::flip)
                .collect(),
            point_towards_zones: self
                .point_towards_zones
                .iter()
                .map(PointTowardsZone::flip)
                .collect(),
            constraint_zones: self.constraint_zones.clone(),
            event_markers: self.event_markers.clone(),
            global_constraints: self.global_constraints,
            ideal_starting_state: self.ideal_starting_state.map(|s| s.flip()),
            goal_end_state: self.goal_end_state.flip(),
            all_points: self.all_points.iter().map(PathPoint::flip).collect(),
            reversed: self.reversed,
            ideal_trajectory: RefCell::new(flipped_trajectory),
            prevent_flipping: self.prevent_flipping,
        }
    }

    /// This path mirrored to the other side of the current alliance. A path on the right of the
    /// blue side becomes a path on the left of the blue side.
    pub fn mirror_path(&self) -> PathPlannerPath {
        let mirrored_trajectory = self.ideal_trajectory.borrow().as_ref().map(|traj| {
            let states = traj
                .states()
                .iter()
                .map(|s| PathPlannerTrajectoryState {
                    time_seconds: s.time_seconds,
                    field_speeds: ChassisSpeeds::new(
                        s.field_speeds.vx,
                        -s.field_speeds.vy,
                        -s.field_speeds.omega,
                    ),
                    pose: Pose2d::new(mirror_translation(s.pose.translation), -s.pose.rotation),
                    linear_velocity: s.linear_velocity,
                    heading: -s.heading,
                    feedforwards: s.feedforwards.mirror(),
                })
                .collect();
            Rc::new(PathPlannerTrajectory::new(states, traj.events().to_vec()))
        });

        let mirror_waypoint = |w: &Waypoint| Waypoint {
            prev_control: w.prev_control.map(mirror_translation),
            anchor: mirror_translation(w.anchor),
            next_control: w.next_control.map(mirror_translation),
        };

        PathPlannerPath {
            name: self.name.clone(),
            waypoints: self.waypoints.iter().map(mirror_waypoint).collect(),
            rotation_targets: self
                .rotation_targets
                .iter()
                .map(|t| RotationTarget {
                    position: t.position,
                    rotation: -t.rotation,
                })
                .collect(),
            point_towards_zones: self
                .point_towards_zones
                .iter()
                .map(|z| PointTowardsZone {
                    name: z.name.clone(),
                    target_position: mirror_translation(z.target_position),
                    rotation_offset: z.rotation_offset,
                    min_position: z.min_position,
                    max_position: z.max_position,
                })
                .collect(),
            constraint_zones: self.constraint_zones.clone(),
            event_markers: self.event_markers.clone(),
            global_constraints: self.global_constraints,
            ideal_starting_state: self
                .ideal_starting_state
                .map(|s| IdealStartingState::new(s.velocity_mps, -s.rotation)),
            goal_end_state: GoalEndState::new(
                self.goal_end_state.velocity_mps,
                -self.goal_end_state.rotation,
            ),
            all_points: self
                .all_points
                .iter()
                .map(|p| PathPoint {
                    position: mirror_translation(p.position),
                    rotation_target: p.rotation_target.map(|t| RotationTarget {
                        position: t.position,
                        rotation: -t.rotation,
                    }),
                    ..*p
                })
                .collect(),
            reversed: self.reversed,
            ideal_trajectory: RefCell::new(mirrored_trajectory),
            prevent_flipping: self.prevent_flipping,
        }
    }

    /// Every path point as a pose with no rotation, for drawing the path on a field widget.
    pub fn get_path_poses(&self) -> Vec<Pose2d> {
        self.all_points
            .iter()
            .map(|p| Pose2d::new(p.position, Rotation2d::ZERO))
            .collect()
    }
}

fn mirror_translation(translation: Translation2d) -> Translation2d {
    Translation2d::new(translation.x, FlippingUtil::field_size_y() - translation.y)
}

fn curve_radius_at_point(index: usize, points: &[PathPoint]) -> f64 {
    if points.len() < 3 {
        return f64::INFINITY;
    }

    if index == 0 {
        calculate_radius(
            points[index].position,
            points[index + 1].position,
            points[index + 2].position,
        )
    } else if index == points.len() - 1 {
        calculate_radius(
            points[index - 2].position,
            points[index - 1].position,
            points[index].position,
        )
    } else {
        calculate_radius(
            points[index - 1].position,
            points[index].position,
            points[index + 1].position,
        )
    }
}

/// Java would hit a null control point while sampling. Report it at load time instead.
fn validate_waypoints(waypoints: &[Waypoint], name: &str) -> Result<()> {
    if waypoints.len() < 2 {
        return Err(Error::Invalid(format!(
            "{name}.path: a path must have at least 2 waypoints"
        )));
    }
    for (i, w) in waypoints.iter().enumerate() {
        if (i != 0 && w.prev_control.is_none())
            || (i != waypoints.len() - 1 && w.next_control.is_none())
        {
            return Err(Error::Invalid(format!(
                "{name}.path: waypoint {i} is missing a control point"
            )));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::util::{TestDeploy, robot_config, use_robot_deploy_folder};

    const STRAIGHT: &str = r#"{
        "version": "2025.0",
        "waypoints": [
            {"anchor": {"x": 1, "y": 1}, "prevControl": null, "nextControl": {"x": 2, "y": 1}},
            {"anchor": {"x": 4, "y": 1}, "prevControl": {"x": 3, "y": 1}, "nextControl": null}
        ],
        "rotationTargets": [],
        "constraintZones": [],
        "pointTowardsZones": [],
        "eventMarkers": [],
        "globalConstraints": {"maxVelocity": 3, "maxAcceleration": 3, "maxAngularVelocity": 360,
            "maxAngularAcceleration": 360, "nominalVoltage": 12, "unlimited": false},
        "goalEndState": {"velocity": 0, "rotation": 90},
        "reversed": false,
        "idealStartingState": {"velocity": 0, "rotation": 0}
    }"#;

    fn straight_path(end_x: f64) -> PathPlannerPath {
        PathPlannerPath::simple(
            PathPlannerPath::waypoints_from_poses(&[
                Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                Pose2d::from_xy(end_x, 0.0, Rotation2d::ZERO),
            ]),
            PathConstraints::new(3.0, 3.0, 4.0, 4.0),
            Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
            GoalEndState::new(0.0, Rotation2d::ZERO),
        )
    }

    #[test]
    fn every_robot_path_loads() {
        use_robot_deploy_folder();
        let mut count = 0;
        for entry in std::fs::read_dir(deploy_directory().join("pathplanner/paths")).unwrap() {
            let file = entry.unwrap().file_name().into_string().unwrap();
            let name = file.strip_suffix(".path").unwrap();
            let path = PathPlannerPath::from_path_file(name).unwrap();
            assert_eq!(path.name, name);
            assert!(path.num_points() >= 2);
            let points = path.get_all_path_points();
            assert!(
                points
                    .windows(2)
                    .all(|w| w[0].distance_along_path < w[1].distance_along_path)
            );
            let last = points.last().unwrap();
            assert_eq!(last.max_v, path.goal_end_state().velocity_mps);
            assert_eq!(
                last.rotation_target.unwrap().rotation,
                path.goal_end_state().rotation
            );
            count += 1;
        }
        assert_eq!(count, 26);
    }

    #[test]
    fn loaded_paths_are_cached_until_the_cache_is_cleared() {
        use_robot_deploy_folder();
        PathPlannerPath::clear_cache();
        let a = PathPlannerPath::from_path_file("1 Meter").unwrap();
        let b = PathPlannerPath::from_path_file("1 Meter").unwrap();
        assert!(Rc::ptr_eq(&a, &b));
        PathPlannerPath::clear_cache();
        let c = PathPlannerPath::from_path_file("1 Meter").unwrap();
        assert!(!Rc::ptr_eq(&a, &c));
    }

    #[test]
    fn load_errors_name_the_problem() {
        let deploy = TestDeploy::new("path-errors");
        deploy.write("paths/old.path", &STRAIGHT.replace("2025.0", "2024.0"));
        deploy.write("paths/broken.path", "{");
        deploy.write(
            "paths/numeric.path",
            &STRAIGHT.replace("\"2025.0\"", "2025.5"),
        );
        deploy.write(
            "paths/nocontrol.path",
            &STRAIGHT.replace(
                r#""nextControl": {"x": 2, "y": 1}"#,
                r#""nextControl": null"#,
            ),
        );
        deploy.write(
            "paths/onepoint.path",
            r#"{"version": "2025.0", "waypoints": [{"anchor": {"x": 1, "y": 1}}],
                "globalConstraints": {"maxVelocity": 3, "maxAcceleration": 3,
                "maxAngularVelocity": 360, "maxAngularAcceleration": 360,
                "nominalVoltage": 12, "unlimited": false},
                "goalEndState": {"velocity": 0, "rotation": 0}}"#,
        );
        PathPlannerPath::clear_cache();

        assert!(matches!(
            PathPlannerPath::from_path_file("missing"),
            Err(Error::Io(_))
        ));
        assert!(matches!(
            PathPlannerPath::from_path_file("broken"),
            Err(Error::Parse(_))
        ));
        let Err(Error::Version { found, file }) = PathPlannerPath::from_path_file("old") else {
            panic!("expected a version error");
        };
        assert_eq!((found.as_str(), file.as_str()), ("2024.0", "old.path"));
        // A numeric version is read as text, and only the major part counts.
        assert!(PathPlannerPath::from_path_file("numeric").is_ok());
        assert!(matches!(
            PathPlannerPath::from_path_file("nocontrol"),
            Err(Error::Invalid(_))
        ));
        assert!(matches!(
            PathPlannerPath::from_path_file("onepoint"),
            Err(Error::Invalid(_))
        ));
    }

    #[test]
    fn a_path_without_an_ideal_state_has_no_starting_pose_or_ideal_trajectory() {
        let deploy = TestDeploy::new("path-no-ideal");
        deploy.write(
            "paths/p.path",
            &STRAIGHT.replace(
                r#""idealStartingState": {"velocity": 0, "rotation": 0}"#,
                r#""idealStartingState": null"#,
            ),
        );
        let path = PathPlannerPath::from_path_file("p").unwrap();
        assert!(path.get_starting_holonomic_pose().is_none());
        assert!(path.get_ideal_trajectory(&robot_config()).is_none());
        // Starting speeds still make a trajectory.
        let t =
            path.generate_trajectory(ChassisSpeeds::default(), Rotation2d::ZERO, &robot_config());
        assert!(t.total_time_seconds().is_finite());
    }

    #[test]
    fn event_markers_load_their_commands() {
        let deploy = TestDeploy::new("path-markers");
        deploy.write(
            "paths/m.path",
            &STRAIGHT.replace(
                r#""eventMarkers": []"#,
                r#""eventMarkers": [
                {"name": "wait", "waypointRelativePos": 0.5, "endWaypointRelativePos": 0.8,
                 "command": {"type": "wait", "data": {"waitTime": 1}}},
                {"name": "late", "waypointRelativePos": 0.9, "endWaypointRelativePos": null,
                 "command": null},
                {"name": "early", "waypointRelativePos": 0.1,
                 "command": {"type": "path", "data": {"pathName": "no such path"}}}
            ]"#,
            ),
        );
        let path = PathPlannerPath::from_path_file("m").unwrap();
        let markers = path.event_markers();
        // Sorted by position. A command that cannot be built is dropped.
        let names: Vec<_> = markers.iter().map(|m| m.trigger_name.as_str()).collect();
        assert_eq!(names, ["early", "wait", "late"]);
        assert!(markers[0].command.is_none());
        assert!(markers[1].command.is_some());
        assert_eq!(markers[1].end_position, 0.8);
        assert!(markers[2].command.is_none());
        assert_eq!(markers[2].end_position, -1.0);
    }

    #[test]
    fn waypoints_from_poses_place_the_control_points() {
        let poses = [
            Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
            Pose2d::from_xy(5.0, 0.0, Rotation2d::from_degrees(90.0)),
            Pose2d::from_xy(5.0, 5.0, Rotation2d::from_degrees(180.0)),
        ];
        let w = PathPlannerPath::waypoints_from_poses(&poses);
        assert_eq!(w.len(), 3);
        assert!(w[0].prev_control.is_none() && w[2].next_control.is_none());
        assert_eq!(w[0].next_control, Some(Translation2d::new(2.0, 0.0)));
        // The middle waypoint's controls lie along its heading, 0.4 of the neighbor distance out.
        assert_eq!(w[1].prev_control, Some(Translation2d::new(5.0, -2.0)));
        assert_eq!(w[1].next_control, Some(Translation2d::new(5.0, 2.0)));
        assert_eq!(w[2].prev_control, Some(Translation2d::new(7.0, 5.0)));
    }

    #[test]
    #[should_panic(expected = "Not enough poses")]
    fn waypoints_need_two_poses() {
        PathPlannerPath::waypoints_from_poses(&[Pose2d::ZERO]);
    }

    #[test]
    #[should_panic(expected = "at least 2 waypoints")]
    fn a_path_needs_two_waypoints() {
        let one = Waypoint {
            prev_control: None,
            anchor: Translation2d::ZERO,
            next_control: None,
        };
        PathPlannerPath::simple(
            vec![one],
            PathConstraints::new(1.0, 1.0, 1.0, 1.0),
            None,
            GoalEndState::new(0.0, Rotation2d::ZERO),
        );
    }

    #[test]
    fn a_straight_path_has_even_spacing_and_the_goal_state_at_the_end() {
        let path = straight_path(5.0);
        let points = path.get_all_path_points();
        assert_eq!(points[0].position, Translation2d::ZERO);
        assert_eq!(
            points.last().unwrap().position,
            Translation2d::new(5.0, 0.0)
        );
        for w in points.windows(2) {
            let spacing = w[0].position.distance(w[1].position);
            assert!((0.15..=0.25).contains(&spacing), "spacing {spacing}");
        }
        assert_eq!(points.last().unwrap().max_v, 0.0);
        assert_eq!(points[3].max_v, 3.0);
    }

    #[test]
    fn flipping_twice_gives_the_path_back() {
        use_robot_deploy_folder();
        let path = PathPlannerPath::from_path_file("TBTT 1").unwrap();
        let twice = path.flip_path().flip_path();
        for (a, b) in path
            .get_all_path_points()
            .iter()
            .zip(twice.get_all_path_points())
        {
            assert!(a.position.distance(b.position) < 1e-12);
        }
        assert_eq!(twice.name, "TBTT 1");
        assert!(
            (twice.goal_end_state().rotation - path.goal_end_state().rotation)
                .radians()
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn mirroring_twice_gives_the_path_back() {
        use_robot_deploy_folder();
        let path = PathPlannerPath::from_path_file("TBTT 1").unwrap();
        let twice = path.mirror_path().mirror_path();
        for (a, b) in path
            .get_all_path_points()
            .iter()
            .zip(twice.get_all_path_points())
        {
            assert!(a.position.distance(b.position) < 1e-12);
        }
        let mirrored = path.mirror_path();
        assert!(
            (mirrored.goal_end_state().rotation + path.goal_end_state().rotation)
                .radians()
                .abs()
                < 1e-12
        );
        assert_eq!(
            mirrored.get_starting_holonomic_pose().unwrap().y(),
            FlippingUtil::field_size_y() - path.get_starting_holonomic_pose().unwrap().y()
        );
    }

    #[test]
    fn the_ideal_trajectory_is_made_once_and_carried_to_flipped_and_mirrored_paths() {
        use_robot_deploy_folder();
        PathPlannerPath::clear_cache();
        let config = robot_config();
        let path = PathPlannerPath::from_path_file("2nd-BBD 2").unwrap();
        assert!(path.ideal_trajectory.borrow().is_none());
        let first = path.get_ideal_trajectory(&config).unwrap();
        assert!(Rc::ptr_eq(
            &first,
            &path.get_ideal_trajectory(&config).unwrap()
        ));

        let flipped = path.flip_path().get_ideal_trajectory(&config).unwrap();
        let start = flipped.initial_state();
        assert_eq!(
            start.pose,
            FlippingUtil::flip_field_pose(first.initial_state().pose)
        );

        let mirrored = path.mirror_path().get_ideal_trajectory(&config).unwrap();
        assert_eq!(mirrored.states().len(), first.states().len());
        assert_eq!(mirrored.total_time_seconds(), first.total_time_seconds());
    }

    #[test]
    fn point_towards_zones_aim_the_robot_at_the_target() {
        let target = Translation2d::new(2.0, 3.0);
        let path = PathPlannerPath::new(
            PathPlannerPath::waypoints_from_poses(&[
                Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                Pose2d::from_xy(4.0, 0.0, Rotation2d::ZERO),
            ]),
            Vec::new(),
            vec![PointTowardsZone {
                name: "aim".into(),
                target_position: target,
                rotation_offset: Rotation2d::from_degrees(180.0),
                min_position: 0.25,
                max_position: 0.75,
            }],
            Vec::new(),
            Vec::new(),
            PathConstraints::new(3.0, 3.0, 4.0, 4.0),
            Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
            GoalEndState::new(0.0, Rotation2d::ZERO),
            false,
        );
        let inside: Vec<_> = path
            .get_all_path_points()
            .iter()
            .filter(|p| (0.25..=0.75).contains(&p.waypoint_relative_pos))
            .filter(|p| p.waypoint_relative_pos < 1.0)
            .collect();
        assert!(!inside.is_empty());
        for p in inside {
            let expected = (target - p.position).angle() + Rotation2d::from_degrees(180.0);
            assert_eq!(p.rotation_target.unwrap().rotation, expected);
        }

        let trajectory =
            path.generate_trajectory(ChassisSpeeds::default(), Rotation2d::ZERO, &robot_config());
        let zone_events: Vec<_> = trajectory
            .events()
            .iter()
            .filter_map(|e| match e {
                crate::Event::PointTowardsZone {
                    timestamp, active, ..
                } => Some((*timestamp, *active)),
                _ => None,
            })
            .collect();
        assert_eq!(zone_events.len(), 2);
        assert!(zone_events[0].1 && !zone_events[1].1);
        assert!(zone_events[0].0 < zone_events[1].0);
    }

    #[test]
    fn a_slower_constraint_zone_limits_the_speed_inside_it() {
        let zone = ConstraintsZone {
            min_position: 0.4,
            max_position: 0.6,
            constraints: PathConstraints::new(1.0, 3.0, 4.0, 4.0),
        };
        let path = PathPlannerPath::new(
            PathPlannerPath::waypoints_from_poses(&[
                Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                Pose2d::from_xy(8.0, 0.0, Rotation2d::ZERO),
            ]),
            Vec::new(),
            Vec::new(),
            vec![zone],
            Vec::new(),
            PathConstraints::new(3.0, 3.0, 4.0, 4.0),
            Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
            GoalEndState::new(0.0, Rotation2d::ZERO),
            false,
        );
        let trajectory =
            path.generate_trajectory(ChassisSpeeds::default(), Rotation2d::ZERO, &robot_config());
        let top_speed = trajectory
            .states()
            .iter()
            .map(|s| s.linear_velocity)
            .fold(0.0, f64::max);
        assert!(top_speed > 2.0, "{top_speed}");
        // Positions 0.4 to 0.6 along a straight bezier with controls at 0.4 of its length.
        let in_zone = |x: f64| (3.3..=4.7).contains(&x);
        for s in trajectory.states().iter().filter(|s| in_zone(s.pose.x())) {
            assert!(s.linear_velocity <= 1.0 + 1e-6, "{}", s.linear_velocity);
        }
    }

    #[test]
    fn unlimited_constraints_are_faster_than_limited_ones() {
        let make = |constraints| {
            PathPlannerPath::simple(
                PathPlannerPath::waypoints_from_poses(&[
                    Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                    Pose2d::from_xy(6.0, 0.0, Rotation2d::ZERO),
                ]),
                constraints,
                Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
                GoalEndState::new(0.0, Rotation2d::ZERO),
            )
            .generate_trajectory(ChassisSpeeds::default(), Rotation2d::ZERO, &robot_config())
            .total_time_seconds()
        };
        let limited = make(PathConstraints::new(1.0, 1.0, 4.0, 4.0));
        let unlimited = make(PathConstraints::unlimited_constraints(12.0));
        assert!(unlimited.is_finite());
        assert!(unlimited < limited / 2.0, "{unlimited} vs {limited}");
    }

    #[test]
    fn a_differential_path_starts_facing_along_the_path() {
        let path = straight_path(5.0);
        assert_eq!(
            path.get_starting_differential_pose(),
            Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO)
        );
        let reversed = PathPlannerPath::new(
            PathPlannerPath::waypoints_from_poses(&[
                Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                Pose2d::from_xy(5.0, 0.0, Rotation2d::ZERO),
            ]),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            PathConstraints::new(1.0, 1.0, 1.0, 1.0),
            None,
            GoalEndState::new(0.0, Rotation2d::ZERO),
            true,
        );
        assert!(reversed.is_reversed());
        let pose = reversed.get_starting_differential_pose();
        assert!((pose.rotation.degrees().abs() - 180.0).abs() < 1e-9);
    }

    #[test]
    fn path_poses_have_no_rotation() {
        let path = straight_path(2.0);
        let poses = path.get_path_poses();
        assert_eq!(poses.len(), path.num_points());
        assert!(poses.iter().all(|p| p.rotation == Rotation2d::ZERO));
    }

    #[test]
    fn a_trajectory_ends_at_the_goal_and_is_sampled_inside_its_time() {
        let path = straight_path(5.0);
        let t =
            path.generate_trajectory(ChassisSpeeds::default(), Rotation2d::ZERO, &robot_config());
        assert_eq!(t.sample(-1.0), *t.initial_state());
        assert_eq!(t.sample(t.total_time_seconds() + 1.0), *t.end_state());
        assert_eq!(t.end_state().linear_velocity, 0.0);
        assert_eq!(
            t.initial_pose(),
            Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO)
        );
        let times: Vec<f64> = t.states().iter().map(|s| s.time_seconds).collect();
        assert!(times.windows(2).all(|w| w[0] <= w[1]));
        let mid = t.sample(t.total_time_seconds() / 2.0);
        assert!(mid.pose.x() > 1.0 && mid.pose.x() < 4.0);
        assert!(mid.linear_velocity > 1.0);
    }

    #[test]
    fn starting_speeds_set_the_first_state() {
        let path = straight_path(5.0);
        let t = path.generate_trajectory(
            ChassisSpeeds::new(2.0, 0.0, 0.0),
            Rotation2d::ZERO,
            &robot_config(),
        );
        assert_eq!(t.initial_state().linear_velocity, 2.0);
        assert_eq!(t.initial_state().field_speeds.vx, 2.0);
    }
}
