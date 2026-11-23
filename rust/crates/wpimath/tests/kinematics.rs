//! Chassis speeds, swerve kinematics, odometry, pose estimator and PID against Java fixtures.

use wpimath::controller::PidController;
use wpimath::estimator::SwerveDrivePoseEstimator;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::interpolation::Interpolatable;
use wpimath::kinematics::{
    ChassisSpeeds, SwerveDriveKinematics, SwerveDriveOdometry, SwerveModulePosition,
    SwerveModuleState,
};

fn r(rad: f64) -> Rotation2d {
    Rotation2d::new(rad)
}

fn rot(a: Rotation2d) -> Vec<f64> {
    vec![a.cos(), a.sin(), a.radians()]
}

fn cs(x: &[f64], at: usize) -> ChassisSpeeds {
    ChassisSpeeds::new(x[at], x[at + 1], x[at + 2])
}

fn csv(c: ChassisSpeeds) -> Vec<f64> {
    vec![c.vx, c.vy, c.omega]
}

fn pose_in(x: &[f64], at: usize) -> Pose2d {
    Pose2d::from_xy(x[at], x[at + 1], r(x[at + 2]))
}

fn pose(p: Pose2d) -> Vec<f64> {
    vec![p.x(), p.y(), p.rotation.radians()]
}

#[test]
fn chassis_and_modules_match_java() {
    // Output 3 of module_position_interpolate comes from Rotation2d.interpolate (atan2).
    let is_angle = |op: &str, i: usize| op == "module_position_interpolate" && i == 3;
    golden::check_angles("kinematics", 1e-9, is_angle, |op, x| {
        Some(match op {
            "chassis_discretize" => csv(cs(x, 0).discretize(x[3])),
            "chassis_from_field_relative" => {
                csv(ChassisSpeeds::from_field_relative(cs(x, 0), r(x[3])))
            }
            "chassis_from_robot_relative" => {
                csv(ChassisSpeeds::from_robot_relative(cs(x, 0), r(x[3])))
            }
            "chassis_to_twist" => {
                let t = cs(x, 0).to_twist2d(x[3]);
                vec![t.dx, t.dy, t.dtheta]
            }
            "chassis_ops" => {
                let (a, b) = (cs(x, 0), cs(x, 3));
                [
                    csv(a + b),
                    csv(a - b),
                    csv(a * x[6]),
                    csv(a / x[6]),
                    csv(-a),
                ]
                .concat()
            }
            "module_optimize" => {
                let mut s = SwerveModuleState::new(x[0], r(x[1]));
                s.optimize(r(x[2]));
                [vec![s.speed], rot(s.angle)].concat()
            }
            "module_cosine_scale" => {
                let mut s = SwerveModuleState::new(x[0], r(x[1]));
                s.cosine_scale(r(x[2]));
                [vec![s.speed], rot(s.angle)].concat()
            }
            "module_position_interpolate" => {
                let a = SwerveModulePosition::new(x[0], r(x[1]));
                let b = SwerveModulePosition::new(x[2], r(x[3]));
                let m = a.interpolate(&b, x[4]);
                [vec![m.distance], rot(m.angle)].concat()
            }
            _ => return None,
        })
    });
}

/// Calls `$f::<N>($args)` for the module counts the fixtures use.
macro_rules! with_modules {
    ($n:expr, $f:ident, $($arg:expr),*) => {
        match $n {
            2 => $f::<2>($($arg),*),
            3 => $f::<3>($($arg),*),
            4 => $f::<4>($($arg),*),
            n => panic!("unsupported module count {n}"),
        }
    };
}

fn modules<const N: usize>(x: &[f64]) -> [Translation2d; N] {
    std::array::from_fn(|i| Translation2d::new(x[1 + 2 * i], x[2 + 2 * i]))
}

fn states<const N: usize>(x: &[f64], at: usize) -> [SwerveModuleState; N] {
    std::array::from_fn(|i| SwerveModuleState::new(x[at + 2 * i], r(x[at + 2 * i + 1])))
}

fn positions<const N: usize>(x: &[f64], at: usize) -> [SwerveModulePosition; N] {
    std::array::from_fn(|i| SwerveModulePosition::new(x[at + 2 * i], r(x[at + 2 * i + 1])))
}

fn to_states<const N: usize>(x: &[f64]) -> Vec<f64> {
    let mut kin = SwerveDriveKinematics::<N>::new(modules::<N>(x));
    let lay = 1 + 2 * N;
    let k = x[lay] as usize;
    let mut out = Vec::new();
    for call in x[lay + 1..].chunks(5).take(k) {
        let cor = Translation2d::new(call[3], call[4]);
        for m in kin.to_swerve_module_states(cs(call, 0), cor) {
            out.extend([m.speed, m.angle.cos(), m.angle.sin(), m.angle.radians()]);
        }
    }
    out
}

fn to_chassis<const N: usize>(x: &[f64]) -> Vec<f64> {
    let kin = SwerveDriveKinematics::<N>::new(modules::<N>(x));
    csv(kin.to_chassis_speeds(&states::<N>(x, 1 + 2 * N)))
}

fn to_twist<const N: usize>(x: &[f64]) -> Vec<f64> {
    let kin = SwerveDriveKinematics::<N>::new(modules::<N>(x));
    let lay = 1 + 2 * N;
    let t = kin.to_twist2d(&positions::<N>(x, lay), &positions::<N>(x, lay + 2 * N));
    vec![t.dx, t.dy, t.dtheta]
}

#[test]
fn swerve_kinematics_match_java() {
    golden::check("swerve_kinematics", 1e-9, |op, x| {
        let n = x[0] as usize;
        Some(match op {
            "to_swerve_states" => with_modules!(n, to_states, x),
            "to_chassis_speeds" => with_modules!(n, to_chassis, x),
            "to_twist2d" => with_modules!(n, to_twist, x),
            "desaturate" => {
                let mut s: Vec<SwerveModuleState> = x[2..2 + n]
                    .iter()
                    .map(|v| SwerveModuleState::new(*v, Rotation2d::ZERO))
                    .collect();
                SwerveDriveKinematics::<2>::desaturate_wheel_speeds(&mut s, x[1]);
                s.iter().map(|m| m.speed).collect()
            }
            "desaturate_chassis" => {
                let mut s: Vec<SwerveModuleState> = x[7..7 + n]
                    .iter()
                    .map(|v| SwerveModuleState::new(*v, Rotation2d::ZERO))
                    .collect();
                SwerveDriveKinematics::<2>::desaturate_wheel_speeds_for_chassis(
                    &mut s,
                    cs(x, 4),
                    x[1],
                    x[2],
                    x[3],
                );
                s.iter().map(|m| m.speed).collect()
            }
            _ => return None,
        })
    });
}

fn odometry<const N: usize>(x: &[f64]) -> Vec<f64> {
    let lay = 1 + 2 * N;
    let kin = SwerveDriveKinematics::<N>::new(modules::<N>(x));
    let gyro0 = x[lay];
    let pose0 = pose_in(x, lay + 1);
    let pos0 = positions::<N>(x, lay + 4);
    let mut odo = SwerveDriveOdometry::new(kin, r(gyro0), pos0, pose0);
    let mut at = lay + 4 + 2 * N;
    let events = x[at] as usize;
    at += 1;
    let mut out = Vec::new();
    for _ in 0..events {
        let kind = x[at] as i32;
        at += 1;
        let p = match kind {
            0 => {
                let p = odo.update(r(x[at]), positions::<N>(x, at + 1));
                at += 1 + 2 * N;
                p
            }
            1 => {
                odo.reset_position(
                    r(x[at]),
                    positions::<N>(x, at + 1),
                    pose_in(x, at + 1 + 2 * N),
                );
                at += 4 + 2 * N;
                odo.pose()
            }
            2 => {
                odo.reset_pose(pose_in(x, at));
                at += 3;
                odo.pose()
            }
            3 => {
                odo.reset_translation(Translation2d::new(x[at], x[at + 1]));
                at += 2;
                odo.pose()
            }
            _ => {
                odo.reset_rotation(r(x[at]));
                at += 1;
                odo.pose()
            }
        };
        out.extend(pose(p));
    }
    out
}

#[test]
fn odometry_matches_java() {
    golden::check("odometry", 1e-9, |op, x| {
        (op == "odometry").then(|| with_modules!(x[0] as usize, odometry, x))
    });
}

fn estimator<const N: usize>(x: &[f64]) -> Vec<f64> {
    let lay = 1 + 2 * N;
    let kin = SwerveDriveKinematics::<N>::new(modules::<N>(x));
    let gyro0 = x[lay];
    let pose0 = pose_in(x, lay + 1);
    let pos0 = positions::<N>(x, lay + 4);
    let mut at = lay + 4 + 2 * N;
    let state_sd = [x[at], x[at + 1], x[at + 2]];
    let vision_sd = [x[at + 3], x[at + 4], x[at + 5]];
    at += 6;
    let mut est = SwerveDrivePoseEstimator::new(kin, r(gyro0), pos0, pose0, state_sd, vision_sd);
    let events = x[at] as usize;
    at += 1;
    let mut out = Vec::new();
    for _ in 0..events {
        let kind = x[at] as i32;
        at += 1;
        let mut present = true;
        let p = match kind {
            0 => {
                let p = est.update_with_time(x[at], r(x[at + 1]), positions::<N>(x, at + 2));
                at += 2 + 2 * N;
                p
            }
            1 => {
                est.add_vision_measurement(pose_in(x, at), x[at + 3]);
                at += 4;
                est.estimated_position()
            }
            2 => {
                est.add_vision_measurement_with_std_devs(
                    pose_in(x, at),
                    x[at + 3],
                    [x[at + 4], x[at + 5], x[at + 6]],
                );
                at += 7;
                est.estimated_position()
            }
            3 => {
                let s = est.sample_at(x[at]);
                at += 1;
                present = s.is_some();
                s.unwrap_or(Pose2d::ZERO)
            }
            4 => {
                est.reset_position(
                    r(x[at]),
                    positions::<N>(x, at + 1),
                    pose_in(x, at + 1 + 2 * N),
                );
                at += 4 + 2 * N;
                est.estimated_position()
            }
            5 => {
                est.reset_pose(pose_in(x, at));
                at += 3;
                est.estimated_position()
            }
            6 => {
                est.reset_translation(Translation2d::new(x[at], x[at + 1]));
                at += 2;
                est.estimated_position()
            }
            7 => {
                est.reset_rotation(r(x[at]));
                at += 1;
                est.estimated_position()
            }
            _ => {
                est.set_vision_measurement_std_devs([x[at], x[at + 1], x[at + 2]]);
                at += 3;
                est.estimated_position()
            }
        };
        out.push(f64::from(present));
        out.extend(pose(p));
    }
    out
}

#[test]
fn pose_estimator_matches_java() {
    golden::check("pose_estimator", 1e-9, |op, x| {
        (op == "pose_estimator").then(|| with_modules!(x[0] as usize, estimator, x))
    });
}

#[test]
fn pid_matches_java() {
    golden::check("pid", 1e-9, |op, p| {
        if op != "pid" {
            return None;
        }
        let mut pid = PidController::with_period(p[0], p[1], p[2], p[3]);
        if p[4] >= 0.0 {
            pid.set_i_zone(p[4]);
        }
        pid.set_integrator_range(p[5], p[6]);
        if p[7] >= 0.0 {
            if p[8] >= 0.0 {
                pid.set_tolerance_with_derivative(p[7], p[8]);
            } else {
                pid.set_tolerance(p[7]);
            }
        }
        if p[9] == 1.0 {
            pid.enable_continuous_input(p[10], p[11]);
        }
        let n = p[12] as usize;
        let mut out = Vec::new();
        for e in p[13..].chunks(3).take(n) {
            let output = match e[0] as i32 {
                1 => {
                    pid.set_setpoint(e[1]);
                    0.0
                }
                2 => {
                    pid.reset();
                    0.0
                }
                3 => pid.calculate_measurement(e[1]),
                _ => pid.calculate(e[1], e[2]),
            };
            out.extend([
                output,
                pid.error(),
                pid.error_derivative(),
                pid.accumulated_error(),
                f64::from(pid.at_setpoint()),
                pid.setpoint(),
            ]);
        }
        Some(out)
    });
}

#[test]
#[should_panic(expected = "Kp must be a non-negative number")]
fn pid_rejects_negative_gain() {
    let _ = PidController::new(-1.0, 0.0, 0.0);
}

#[test]
#[should_panic(expected = "at least two modules")]
fn kinematics_needs_two_modules() {
    let _ = SwerveDriveKinematics::<1>::new([Translation2d::ZERO]);
}

#[test]
fn zero_speeds_keep_reset_headings() {
    let mut kin = SwerveDriveKinematics::<2>::new([
        Translation2d::new(0.3, 0.3),
        Translation2d::new(-0.3, -0.3),
    ]);
    let heading = Rotation2d::from_degrees(30.0);
    kin.reset_headings([heading; 2]);
    let states = kin.to_swerve_module_states(ChassisSpeeds::default(), Translation2d::ZERO);
    assert!(states.iter().all(|m| m.speed == 0.0 && m.angle == heading));
}
