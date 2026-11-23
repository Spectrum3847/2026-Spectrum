//! Geometry types against Java golden fixtures.

use wpimath::geometry::{
    Pose2d, Pose3d, Quaternion, Rectangle2d, Rotation2d, Rotation3d, Transform2d, Transform3d,
    Translation2d, Translation3d, Twist2d,
};
use wpimath::interpolation::Interpolatable;

fn r(rad: f64) -> Rotation2d {
    Rotation2d::new(rad)
}

fn rot(a: Rotation2d) -> Vec<f64> {
    vec![a.cos(), a.sin(), a.radians()]
}

fn t(x: &[f64], at: usize) -> Translation2d {
    Translation2d::new(x[at], x[at + 1])
}

fn tr(a: Translation2d) -> Vec<f64> {
    vec![a.x, a.y]
}

fn pose_in(x: &[f64], at: usize) -> Pose2d {
    Pose2d::from_xy(x[at], x[at + 1], r(x[at + 2]))
}

fn pose(p: Pose2d) -> Vec<f64> {
    vec![p.x(), p.y(), p.rotation.radians()]
}

fn tf_in(x: &[f64], at: usize) -> Transform2d {
    Transform2d::from_xy(x[at], x[at + 1], r(x[at + 2]))
}

fn tf(x: Transform2d) -> Vec<f64> {
    vec![x.x(), x.y(), x.rotation.radians()]
}

fn flag(b: bool) -> f64 {
    f64::from(b)
}

fn cat(parts: &[Vec<f64>]) -> Vec<f64> {
    parts.concat()
}

fn rotation2d(op: &str, x: &[f64]) -> Option<Vec<f64>> {
    Some(match op {
        "rotation2d_new" => {
            let a = r(x[0]);
            vec![a.cos(), a.sin(), a.radians(), a.degrees(), a.rotations()]
        }
        "rotation2d_xy" => rot(Rotation2d::from_xy(x[0], x[1])),
        "rotation2d_plus" => rot(r(x[0]) + r(x[1])),
        "rotation2d_minus" => rot(r(x[0]) - r(x[1])),
        "rotation2d_relative_to" => rot(r(x[0]).relative_to(r(x[1]))),
        "rotation2d_neg" => rot(-r(x[0])),
        "rotation2d_times" => rot(r(x[0]) * x[1]),
        "rotation2d_div" => rot(r(x[0]) / x[1]),
        "rotation2d_interpolate" => rot(r(x[0]).interpolate(&r(x[1]), x[2])),
        _ => return None,
    })
}

fn translation2d(op: &str, x: &[f64]) -> Option<Vec<f64>> {
    Some(match op {
        "translation2d_polar" => tr(Translation2d::from_polar(x[0], r(x[1]))),
        "translation2d_distance" => {
            let (a, b) = (t(x, 0), t(x, 2));
            vec![a.distance(b), a.squared_distance(b)]
        }
        "translation2d_norm" => {
            let a = t(x, 0);
            cat(&[vec![a.norm(), a.squared_norm()], rot(a.angle())])
        }
        "translation2d_rotate_by" => tr(t(x, 0).rotate_by(r(x[2]))),
        "translation2d_rotate_around" => tr(t(x, 0).rotate_around(t(x, 2), r(x[4]))),
        "translation2d_ops" => {
            let (a, b, s) = (t(x, 0), t(x, 2), x[4]);
            cat(&[
                tr(a + b),
                tr(a - b),
                tr(a * s),
                tr(a / if s == 0.0 { 1.0 } else { s }),
                tr(-a),
                vec![a.dot(b), a.cross(b)],
            ])
        }
        "translation2d_interpolate" => tr(t(x, 0).interpolate(&t(x, 2), x[4])),
        _ => return None,
    })
}

fn pose2d(op: &str, x: &[f64]) -> Option<Vec<f64>> {
    Some(match op {
        "pose2d_transform_by" => pose(pose_in(x, 0).transform_by(tf_in(x, 3))),
        "pose2d_relative_to" => pose(pose_in(x, 0).relative_to(pose_in(x, 3))),
        "pose2d_minus" => tf(pose_in(x, 0) - pose_in(x, 3)),
        "pose2d_exp" => pose(pose_in(x, 0).exp(Twist2d::new(x[3], x[4], x[5]))),
        "pose2d_log" => {
            let tw = pose_in(x, 0).log(pose_in(x, 3));
            vec![tw.dx, tw.dy, tw.dtheta]
        }
        "pose2d_interpolate" => pose(pose_in(x, 0).interpolate(&pose_in(x, 3), x[6])),
        "pose2d_times" => pose(pose_in(x, 0) * x[3]),
        "pose2d_rotate_by" => pose(pose_in(x, 0).rotate_by(r(x[3]))),
        "pose2d_rotate_around" => pose(pose_in(x, 0).rotate_around(t(x, 3), r(x[5]))),
        "transform2d_from_poses" => tf(Transform2d::from_poses(pose_in(x, 0), pose_in(x, 3))),
        "transform2d_inverse" => tf(tf_in(x, 0).inverse()),
        "transform2d_plus" => tf(tf_in(x, 0) + tf_in(x, 3)),
        "transform2d_times" => tf(tf_in(x, 0) * x[3]),
        _ => return None,
    })
}

fn rectangle2d(op: &str, x: &[f64]) -> Option<Vec<f64>> {
    Some(match op {
        "rect_query" => {
            let rect = Rectangle2d::new(pose_in(x, 0), x[3], x[4]);
            let p = t(x, 5);
            cat(&[
                vec![
                    flag(rect.contains(p)),
                    flag(rect.intersects(p)),
                    rect.distance(p),
                ],
                tr(rect.nearest(p)),
            ])
        }
        "rect_corners" => {
            let rect = Rectangle2d::from_corners(t(x, 0), t(x, 2));
            let p = t(x, 4);
            cat(&[
                pose(rect.center()),
                vec![
                    rect.x_width(),
                    rect.y_width(),
                    flag(rect.contains(p)),
                    rect.distance(p),
                ],
            ])
        }
        "rect_transform" => {
            let rect = Rectangle2d::new(pose_in(x, 0), x[3], x[4]);
            let moved = rect.transform_by(Transform2d::from_xy(x[5], x[6], r(x[7])));
            let turned = rect.rotate_by(r(x[7]));
            let p = t(x, 8);
            cat(&[
                pose(moved.center()),
                pose(turned.center()),
                vec![flag(moved.contains(p)), flag(turned.contains(p))],
            ])
        }
        _ => return None,
    })
}

#[test]
fn geometry2d_matches_java() {
    // Ops whose Java result goes through atan2 (rotateBy, fromXY): output 2 is that angle.
    let is_angle = |op: &str, i: usize| {
        i == 2
            && matches!(
                op,
                "rotation2d_xy"
                    | "rotation2d_plus"
                    | "rotation2d_minus"
                    | "rotation2d_relative_to"
                    | "rotation2d_interpolate"
            )
    };
    golden::check_angles("geometry", 1e-9, is_angle, |op, x| {
        rotation2d(op, x)
            .or_else(|| translation2d(op, x))
            .or_else(|| pose2d(op, x))
            .or_else(|| rectangle2d(op, x))
    });
}

fn q(a: Quaternion) -> Vec<f64> {
    vec![a.w, a.x, a.y, a.z]
}

fn quat_in(x: &[f64], at: usize) -> Quaternion {
    Quaternion::new(x[at], x[at + 1], x[at + 2], x[at + 3])
}

fn r3(x: &[f64], at: usize) -> Rotation3d {
    Rotation3d::from_euler(x[at], x[at + 1], x[at + 2])
}

fn r3out(a: Rotation3d) -> Vec<f64> {
    cat(&[q(a.quaternion()), vec![a.x(), a.y(), a.z(), a.angle()]])
}

fn t3(x: &[f64], at: usize) -> Translation3d {
    Translation3d::new(x[at], x[at + 1], x[at + 2])
}

fn t3out(a: Translation3d) -> Vec<f64> {
    vec![a.x, a.y, a.z]
}

fn p3(x: &[f64], at: usize) -> Pose3d {
    Pose3d::new(t3(x, at), r3(x, at + 3))
}

fn p3out(p: Pose3d) -> Vec<f64> {
    cat(&[t3out(p.translation), q(p.rotation.quaternion())])
}

fn tf3(x: &[f64], at: usize) -> Transform3d {
    Transform3d::new(t3(x, at), r3(x, at + 3))
}

fn tf3out(p: Transform3d) -> Vec<f64> {
    cat(&[t3out(p.translation), q(p.rotation.quaternion())])
}

fn quaternion_and_rotation3d(op: &str, x: &[f64]) -> Option<Vec<f64>> {
    Some(match op {
        "quaternion_ops" => {
            let (a, b) = (quat_in(x, 0), quat_in(x, 4));
            let inverse = if a.norm() == 0.0 {
                vec![0.0; 4]
            } else {
                q(a.inverse())
            };
            cat(&[
                q(a * b),
                q(a + b),
                q(a - b),
                q(a.conjugate()),
                inverse,
                q(a.normalize()),
                q(a * 2.5),
                q(a / 2.5),
                vec![a.norm(), a.dot(b)],
            ])
        }
        "rotation3d_euler" => r3out(r3(x, 0)),
        "rotation3d_axis_angle" => r3out(Rotation3d::from_axis_angle([x[0], x[1], x[2]], x[3])),
        "rotation3d_rotate_by" => r3out(r3(x, 0).rotate_by(r3(x, 3))),
        "rotation3d_minus" => r3out(r3(x, 0) - r3(x, 3)),
        "rotation3d_relative_to" => r3out(r3(x, 0).relative_to(r3(x, 3))),
        "rotation3d_neg" => r3out(-r3(x, 0)),
        "rotation3d_times" => r3out(r3(x, 0) * x[3]),
        "rotation3d_interpolate" => r3out(r3(x, 0).interpolate(&r3(x, 3), x[6])),
        "rotation3d_to_rotation2d" => rot(r3(x, 0).to_rotation2d()),
        _ => return None,
    })
}

fn translation3d(op: &str, x: &[f64]) -> Option<Vec<f64>> {
    Some(match op {
        "translation3d_rotate_by" => t3out(t3(x, 0).rotate_by(r3(x, 3))),
        "translation3d_rotate_around" => t3out(t3(x, 0).rotate_around(t3(x, 3), r3(x, 6))),
        "translation3d_polar" => t3out(Translation3d::from_polar(x[0], r3(x, 1))),
        "translation3d_ops" => {
            let (a, b, s) = (t3(x, 0), t3(x, 3), x[6]);
            let c = a.cross(b);
            cat(&[
                t3out(a + b),
                t3out(a - b),
                t3out(a * s),
                t3out(a / if s == 0.0 { 1.0 } else { s }),
                t3out(-a),
                vec![a.dot(b), c[0], c[1], c[2]],
                vec![
                    a.norm(),
                    a.squared_norm(),
                    a.distance(b),
                    a.squared_distance(b),
                ],
                tr(a.to_translation2d()),
            ])
        }
        "translation3d_interpolate" => t3out(t3(x, 0).interpolate(&t3(x, 3), x[6])),
        _ => return None,
    })
}

fn pose3d(op: &str, x: &[f64]) -> Option<Vec<f64>> {
    Some(match op {
        "pose3d_transform_by" => p3out(p3(x, 0).transform_by(tf3(x, 6))),
        "pose3d_relative_to" => p3out(p3(x, 0).relative_to(p3(x, 6))),
        "pose3d_minus" => tf3out(p3(x, 0) - p3(x, 6)),
        "pose3d_rotate_by" => p3out(p3(x, 0).rotate_by(r3(x, 6))),
        "pose3d_rotate_around" => p3out(p3(x, 0).rotate_around(t3(x, 6), r3(x, 9))),
        "pose3d_from_pose2d" => p3out(Pose3d::from_pose2d(pose_in(x, 0))),
        "pose3d_to_pose2d" => pose(p3(x, 0).to_pose2d()),
        "transform3d_from_poses" => tf3out(Transform3d::from_poses(p3(x, 0), p3(x, 6))),
        "transform3d_inverse" => tf3out(tf3(x, 0).inverse()),
        "transform3d_plus" => tf3out(tf3(x, 0) + tf3(x, 6)),
        "transform3d_times" => tf3out(tf3(x, 0) * x[6]),
        "transform3d_from_transform2d" => tf3out(Transform3d::from_transform2d(tf_in(x, 0))),
        _ => return None,
    })
}

#[test]
fn geometry3d_matches_java() {
    golden::check("geometry3d", 1e-9, |op, x| {
        quaternion_and_rotation3d(op, x)
            .or_else(|| translation3d(op, x))
            .or_else(|| pose3d(op, x))
    });
}

#[test]
fn rotation_constants_match_computed_values() {
    for (constant, rad) in [
        (Rotation2d::CW_PI_2, -std::f64::consts::FRAC_PI_2),
        (Rotation2d::CCW_PI_2, std::f64::consts::FRAC_PI_2),
        (Rotation2d::PI, std::f64::consts::PI),
    ] {
        let computed = Rotation2d::new(rad);
        assert_eq!(constant.cos().to_bits(), computed.cos().to_bits());
        assert_eq!(constant.sin().to_bits(), computed.sin().to_bits());
        assert_eq!(constant.radians(), computed.radians());
    }
}

#[test]
#[should_panic(expected = "cannot be less than 0")]
fn rectangle_rejects_negative_width() {
    let _ = Rectangle2d::new(Pose2d::ZERO, -1.0, 1.0);
}

#[test]
fn rotation_unit_constructors() {
    assert_eq!(Rotation2d::from_degrees(90.0).radians(), 90f64.to_radians());
    let quarter = Rotation2d::from_rotations(0.25);
    assert!((quarter.radians() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    assert!((quarter.degrees() - 90.0).abs() < 1e-9);
}
