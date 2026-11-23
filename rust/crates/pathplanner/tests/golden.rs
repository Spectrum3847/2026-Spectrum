//! Checks the port against fixtures the Java PathPlannerLib writes (`PathPlannerFixtures.java`).
//! Paths, autos and the navgrid come from the real deploy folder.

use pathplanner::*;
use serde_json::json;
use std::collections::HashMap;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::Arc;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::kinematics::{ChassisSpeeds, SwerveModuleState};

const TOL: f64 = 1e-9;

fn use_deploy_folder() {
    set_deploy_directory(Some(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../src/main/deploy"),
    ));
}

fn settings_config() -> RobotConfig {
    use_deploy_folder();
    RobotConfig::from_gui_settings().unwrap()
}

/// The config both warmup commands use.
fn warmup_config() -> RobotConfig {
    RobotConfig::new_differential(
        75.0,
        6.8,
        ModuleConfig::new(0.048, 5.0, 1.2, kraken_x60(1).with_reduction(6.14), 60.0, 1),
        0.55,
    )
}

fn rot(radians: f64) -> Rotation2d {
    Rotation2d::new(radians)
}

fn state_out(s: &PathPlannerTrajectoryState, with_feedforwards: bool) -> Vec<f64> {
    let mut v = vec![
        s.time_seconds,
        s.pose.x(),
        s.pose.y(),
        s.pose.rotation.radians(),
        s.field_speeds.vx,
        s.field_speeds.vy,
        s.field_speeds.omega,
        s.linear_velocity,
        s.heading.radians(),
    ];
    if with_feedforwards {
        let ff = &s.feedforwards;
        for values in [
            &ff.accelerations_mps_sq,
            &ff.linear_forces_newtons,
            &ff.torque_currents_amps,
            &ff.robot_relative_forces_x_newtons,
            &ff.robot_relative_forces_y_newtons,
        ] {
            v.extend_from_slice(values);
        }
    }
    v
}

fn points_out(path: &PathPlannerPath, full: bool) -> Vec<f64> {
    let mut v = vec![path.num_points() as f64];
    for p in path.get_all_path_points() {
        v.extend([p.position.x, p.position.y]);
        if full {
            v.extend([
                p.distance_along_path,
                p.max_v,
                p.waypoint_relative_pos,
                if p.rotation_target.is_some() {
                    1.0
                } else {
                    0.0
                },
                p.rotation_target.map_or(0.0, |t| t.position),
                p.rotation_target.map_or(0.0, |t| t.rotation.radians()),
            ]);
        }
    }
    v
}

fn event_out(trajectory: &PathPlannerTrajectory) -> (Vec<f64>, Vec<String>) {
    let mut out = Vec::new();
    let mut names = Vec::new();
    for e in trajectory.events() {
        match e {
            Event::OneShotTrigger { timestamp, name } => {
                out.extend([1.0, *timestamp, 1.0]);
                names.push(name.clone());
            }
            Event::Trigger {
                timestamp,
                name,
                active,
            } => {
                out.extend([2.0, *timestamp, f64::from(u8::from(*active))]);
                names.push(name.clone());
            }
            other => {
                out.extend([3.0, other.timestamp_seconds(), 0.0]);
                names.push(String::new());
            }
        }
    }
    (out, names)
}

/// Runs the cases that describe one trajectory: summary, events and samples.
fn trajectory_case(
    kind: &str,
    tag: &str,
    suffix: &str,
    input: &[f64],
    trajectory: &PathPlannerTrajectory,
) -> Option<Vec<f64>> {
    match kind {
        "summary" => Some(vec![
            trajectory.total_time_seconds(),
            trajectory.states().len() as f64,
            trajectory.events().len() as f64,
        ]),
        "events" => {
            let (out, names) = event_out(trajectory);
            assert_eq!(names.join(","), suffix, "event names for {tag}");
            Some(out)
        }
        "sample" => Some(state_out(&trajectory.sample(input[0]), true)),
        _ => None,
    }
}

/// Whether output `i` of a trajectory case is an angle. An angle that is pi on one side can come
/// out as -pi on the other, because the sign of a 1e-16 sine decides it.
fn is_angle(op: &str, i: usize) -> bool {
    match op.split(':').next().unwrap_or("") {
        "sample" => i == 3 || i == 8,
        "states" => i >= 1 && matches!((i - 1) % 9, 3 | 8),
        _ => false,
    }
}

/// `golden::check` with angles compared modulo a full turn.
fn check_wrapped(name: &str, eval: impl FnMut(&str, &[f64]) -> Option<Vec<f64>>) {
    golden::check_angles(name, TOL, is_angle, eval);
}

fn generate(path: &PathPlannerPath, config: &RobotConfig) -> PathPlannerTrajectory {
    let rotation = path
        .ideal_starting_state()
        .map_or(Rotation2d::ZERO, |s| s.rotation);
    path.generate_trajectory(ChassisSpeeds::default(), rotation, config)
}

#[test]
fn flipping() {
    golden::check("pathplanner_flipping", TOL, |op, input| {
        let (kind, symmetry) = op.split_once(':')?;
        FlippingUtil::set_symmetry_type(match symmetry {
            "kRotational" => FieldSymmetry::Rotational,
            "kMirrored" => FieldSymmetry::Mirrored,
            _ => return None,
        });
        FlippingUtil::set_field_size(input[0], input[1]);
        let at = |i: usize| input[i];
        Some(match kind {
            "position" => {
                let p = FlippingUtil::flip_field_position(Translation2d::new(at(2), at(3)));
                vec![p.x, p.y]
            }
            "rotation" => {
                let r = FlippingUtil::flip_field_rotation(rot(at(2)));
                vec![r.cos(), r.sin(), r.radians()]
            }
            "pose" => {
                let p = FlippingUtil::flip_field_pose(Pose2d::from_xy(at(2), at(3), rot(at(4))));
                vec![p.x(), p.y(), p.rotation.radians()]
            }
            "speeds" => {
                let s = FlippingUtil::flip_field_speeds(ChassisSpeeds::new(at(2), at(3), at(4)));
                vec![s.vx, s.vy, s.omega]
            }
            "feedforwards" => {
                let ff = PerModule::from_slice(&input[2..6]);
                let mut out = Vec::new();
                out.extend_from_slice(&FlippingUtil::flip_feedforwards(&ff));
                out.extend_from_slice(&FlippingUtil::flip_feedforward_xs(&ff));
                out.extend_from_slice(&FlippingUtil::flip_feedforward_ys(&ff));
                out
            }
            _ => return None,
        })
    });
}

fn settings_json(motor: &str, p: &[f64], swerve: bool) -> String {
    let mut json = json!({
        "holonomicMode": swerve,
        "robotMass": p[0],
        "robotMOI": p[1],
        "driveWheelRadius": p[2],
        "driveGearing": p[3],
        "maxDriveSpeed": p[4],
        "wheelCOF": p[5],
        "driveMotorType": motor,
        "driveCurrentLimit": p[6],
    });
    if swerve {
        json["flModuleX"] = p[7].into();
        json["flModuleY"] = p[8].into();
        json["frModuleX"] = p[9].into();
        json["frModuleY"] = p[10].into();
        json["blModuleX"] = p[11].into();
        json["blModuleY"] = p[12].into();
        json["brModuleX"] = p[13].into();
        json["brModuleY"] = p[14].into();
    } else {
        json["robotTrackwidth"] = p[7].into();
    }
    json.to_string()
}

fn config_out(c: &RobotConfig) -> Vec<f64> {
    let m = &c.module_config;
    let d = &m.drive_motor;
    let mut v = vec![
        c.mass_kg,
        c.moi,
        c.wheel_friction_force,
        c.max_torque_friction,
        m.torque_loss,
        m.drive_current_limit,
        m.max_drive_velocity_rad_per_sec,
        m.wheel_radius_meters,
        m.max_drive_velocity_mps,
        m.wheel_cof,
        d.nominal_voltage_volts,
        d.stall_torque_newton_meters,
        d.stall_current_amps,
        d.free_current_amps,
        d.free_speed_rad_per_sec,
        d.r_ohms,
        d.kv_rad_per_sec_per_volt,
        d.kt_nm_per_amp,
    ];
    v.extend_from_slice(&c.module_pivot_distance);
    v
}

/// [finite, value, is NaN, is +infinity], since JSON has no infinity.
fn radius_out(r: f64) -> Vec<f64> {
    vec![
        f64::from(u8::from(r.is_finite())),
        if r.is_finite() { r } else { 0.0 },
        f64::from(u8::from(r.is_nan())),
        f64::from(u8::from(r == f64::INFINITY)),
    ]
}

#[test]
fn config() {
    use_deploy_folder();
    golden::check("pathplanner_config", TOL, |op, input| {
        let (kind, motor) = op.split_once(':').unwrap_or((op, ""));
        let tr = |i: usize| Translation2d::new(input[i], input[i + 1]);
        match kind {
            "swerve" => {
                let c = RobotConfig::from_gui_settings_json(&settings_json(motor, input, true))
                    .unwrap();
                let (kin, states, forces) = (&input[15..18], &input[18..26], &input[26..29]);
                let speeds = c.module_speeds(ChassisSpeeds::new(kin[0], kin[1], kin[2]));
                let module_states: Vec<SwerveModuleState> = (0..4)
                    .map(|m| SwerveModuleState::new(states[m * 2], rot(states[m * 2 + 1])))
                    .collect();
                let back = c.to_chassis_speeds(&module_states);
                let wheel = c.chassis_forces_to_wheel_force_vectors(ChassisSpeeds::new(
                    forces[0], forces[1], forces[2],
                ));
                let mut out = config_out(&c);
                out.extend_from_slice(&speeds);
                out.extend([back.vx, back.vy, back.omega]);
                for w in wheel {
                    out.extend([w.x, w.y]);
                }
                Some(out)
            }
            "diff" => {
                let c = RobotConfig::from_gui_settings_json(&settings_json(motor, input, false))
                    .unwrap();
                let (kin, states, forces) = (&input[8..11], &input[11..15], &input[15..18]);
                let speeds = c.module_speeds(ChassisSpeeds::new(kin[0], kin[1], kin[2]));
                let back = c.to_chassis_speeds(&[
                    SwerveModuleState::new(states[0], rot(states[1])),
                    SwerveModuleState::new(states[2], rot(states[3])),
                ]);
                let wheel = c.chassis_forces_to_wheel_force_vectors(ChassisSpeeds::new(
                    forces[0], forces[1], forces[2],
                ));
                let mut out = config_out(&c);
                out.extend_from_slice(&speeds[..2]);
                out.extend([back.vx, back.vy, back.omega]);
                for w in &wheel[..2] {
                    out.extend([w.x, w.y]);
                }
                Some(out)
            }
            "settings" => Some(config_out(&settings_config())),
            "warmup" => Some(config_out(&warmup_config())),
            "quadratic_lerp" => {
                let p = quadratic_lerp(tr(0), tr(2), tr(4), input[6]);
                Some(vec![p.x, p.y])
            }
            "cubic_lerp" => {
                let p = cubic_lerp(tr(0), tr(2), tr(4), tr(6), input[8]);
                Some(vec![p.x, p.y])
            }
            "radius" => Some(radius_out(calculate_radius(tr(0), tr(2), tr(4)))),
            _ => None,
        }
    });
}

/// Trajectories of the deploy paths, built the way the Java fixture builds them. The cases
/// arrive path by path in the order the Java generator ran, and each path is loaded fresh,
/// because the ideal trajectory is cached on the path the first time it is asked for.
#[derive(Default)]
struct Trajectories {
    path_name: String,
    path: Option<Rc<PathPlannerPath>>,
    trajectories: HashMap<String, Rc<PathPlannerTrajectory>>,
    moving: HashMap<String, [f64; 4]>,
}

impl Trajectories {
    fn path(&mut self, name: &str) -> Rc<PathPlannerPath> {
        if self.path_name != name || self.path.is_none() {
            PathPlannerPath::clear_cache();
            self.path = Some(PathPlannerPath::from_path_file(name).unwrap());
            self.path_name = name.to_string();
        }
        self.path.clone().unwrap()
    }

    fn trajectory(&mut self, tag: &str, config: &RobotConfig) -> Rc<PathPlannerTrajectory> {
        if let Some(t) = self.trajectories.get(tag) {
            return t.clone();
        }
        let (variant, name) = tag.split_once(':').unwrap_or((tag, ""));
        let trajectory = if variant == "warmup" {
            let waypoints = PathPlannerPath::waypoints_from_poses(&[
                Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                Pose2d::from_xy(6.0, 6.0, Rotation2d::ZERO),
            ]);
            let path = PathPlannerPath::simple(
                waypoints,
                PathConstraints::new(4.0, 4.0, 4.0, 4.0),
                Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
                GoalEndState::new(0.0, Rotation2d::CW_PI_2),
            );
            generate(&path, &warmup_config())
        } else if let Some(i) = variant.strip_prefix("moving") {
            let path = self.path(name);
            let v = self.moving[&format!("{i}:{name}")];
            path.generate_trajectory(ChassisSpeeds::new(v[0], v[1], v[2]), rot(v[3]), config)
        } else {
            let path = self.path(name);
            match variant {
                "orig" => generate(&path, config),
                "flip" => generate(&path.flip_path(), config),
                "mirror" => generate(&path.mirror_path(), config),
                "mirrorflip" => generate(&path.mirror_path().flip_path(), config),
                "idealflip" | "idealmirror" => {
                    path.get_ideal_trajectory(config);
                    let other = if variant == "idealflip" {
                        path.flip_path()
                    } else {
                        path.mirror_path()
                    };
                    (*other.get_ideal_trajectory(config).unwrap()).clone()
                }
                other => panic!("unknown variant {other}"),
            }
        };
        let trajectory = Rc::new(trajectory);
        self.trajectories
            .insert(tag.to_string(), trajectory.clone());
        trajectory
    }
}

#[test]
fn trajectories() {
    let config = settings_config();
    let mut ctx = Trajectories::default();
    check_wrapped("pathplanner_trajectories", |op, input| {
        let (head, suffix) = op.split_once('|').unwrap_or((op, ""));
        let (kind, tag) = head.split_once(':').unwrap_or((head, ""));
        match kind {
            "moving_in" => {
                ctx.moving
                    .insert(tag.to_string(), [input[0], input[1], input[2], input[3]]);
                Some(vec![])
            }
            "warmup_start" => {
                let waypoints = PathPlannerPath::waypoints_from_poses(&[
                    Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
                    Pose2d::from_xy(6.0, 6.0, Rotation2d::ZERO),
                ]);
                let path = PathPlannerPath::simple(
                    waypoints,
                    PathConstraints::new(4.0, 4.0, 4.0, 4.0),
                    Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
                    GoalEndState::new(0.0, Rotation2d::CW_PI_2),
                );
                let p = path.get_starting_differential_pose();
                Some(vec![p.x(), p.y(), p.rotation.radians()])
            }
            _ => {
                let trajectory = ctx.trajectory(tag, &config);
                trajectory_case(kind, tag, suffix, input, &trajectory)
            }
        }
    });
}

#[test]
fn paths() {
    let config = settings_config();
    check_wrapped("pathplanner_paths", |op, _| {
        let mut parts = op.splitn(3, ':');
        let (kind, variant, name) = (parts.next()?, parts.next()?, parts.next()?);
        let path = PathPlannerPath::from_path_file(name).unwrap();
        match (kind, variant) {
            ("points", "orig") => Some(points_out(&path, true)),
            ("points", "flip") => Some(points_out(&path.flip_path(), false)),
            ("points", "mirror") => Some(points_out(&path.mirror_path(), false)),
            ("states", "orig") => {
                let t = generate(&path, &config);
                let mut out = vec![t.states().len() as f64];
                for s in t.states() {
                    out.extend(state_out(s, false));
                }
                Some(out)
            }
            _ => None,
        }
    });
}

#[test]
fn on_the_fly() {
    let config = settings_config();
    let mut current: HashMap<String, Rc<PathPlannerTrajectory>> = HashMap::new();
    check_wrapped("pathplanner_otf", |op, input| {
        let (kind, tag) = op.split_once(':')?;
        if kind == "points" {
            let n = input[0] as usize;
            let poses: Vec<Pose2d> = (0..n)
                .map(|k| Pose2d::from_xy(input[1 + k * 3], input[2 + k * 3], rot(input[3 + k * 3])))
                .collect();
            let at = 1 + n * 3;
            let constraints =
                PathConstraints::new(input[at], input[at + 1], input[at + 2], input[at + 3])
                    .with_nominal_voltage(input[at + 4]);
            let ideal = (input[at + 7] >= 0.0)
                .then(|| IdealStartingState::new(input[at + 7], rot(input[at + 8])));
            let path = PathPlannerPath::simple(
                PathPlannerPath::waypoints_from_poses(&poses),
                constraints,
                ideal,
                GoalEndState::new(input[at + 5], rot(input[at + 6])),
            );
            let trajectory =
                path.generate_trajectory(ChassisSpeeds::default(), rot(input[at + 8]), &config);
            current.insert(tag.to_string(), Rc::new(trajectory));
            return Some(points_out(&path, true));
        }
        let trajectory = current.get(tag)?;
        trajectory_case(kind, tag, "", input, trajectory)
    });
}

#[test]
fn controller() {
    golden::check("pathplanner_controller", TOL, |_, input| {
        let translation = PidConstants::with_i_zone(input[0], input[1], input[2], input[3]);
        let rotation = PidConstants::with_i_zone(input[4], input[5], input[6], input[7]);
        let mut controller = PpHolonomicDriveController::new(translation, rotation);
        controller.set_enabled(input[8] != 0.0);
        let steps = input[9] as usize;
        let mut out = Vec::new();
        for k in 0..steps {
            let s = &input[10 + k * 10..20 + k * 10];
            if s[0] == 1.0 {
                controller.reset(Pose2d::ZERO, ChassisSpeeds::default());
                out.extend([0.0; 3]);
                continue;
            }
            let target = PathPlannerTrajectoryState {
                pose: Pose2d::from_xy(s[4], s[5], rot(s[6])),
                field_speeds: ChassisSpeeds::new(s[7], s[8], s[9]),
                ..Default::default()
            };
            let r = controller
                .calculate_robot_relative_speeds(Pose2d::from_xy(s[1], s[2], rot(s[3])), &target);
            out.extend([r.vx, r.vy, r.omega]);
        }
        Some(out)
    });
}

#[test]
fn pathfinding() {
    use_deploy_folder();
    let grid = Arc::new(Grid::load());
    golden::check("pathplanner_pathfinding", TOL, |_, input| {
        let mut planner = AdStarPlanner::new(grid.clone());
        let plan = planner.plan_to_convergence(
            Translation2d::new(input[0], input[1]),
            Translation2d::new(input[2], input[3]),
        );
        let waypoints = plan.map(|p| p.waypoints).unwrap_or_default();
        if waypoints.len() < 2 {
            return Some(vec![0.0]);
        }
        let mut out = vec![1.0, waypoints.len() as f64];
        for w in &waypoints {
            for p in [w.prev_control, Some(w.anchor), w.next_control] {
                out.extend([
                    if p.is_some() { 1.0 } else { 0.0 },
                    p.map_or(0.0, |p| p.x),
                    p.map_or(0.0, |p| p.y),
                ]);
            }
        }
        let path = PathPlannerPath::simple(
            waypoints,
            PathConstraints::new(3.0, 3.0, 4.0, 4.0),
            None,
            GoalEndState::new(0.0, Rotation2d::ZERO),
        );
        let points = path.get_all_path_points();
        out.push(points.len() as f64);
        out.push(points[points.len() - 1].distance_along_path);
        Some(out)
    });
}
