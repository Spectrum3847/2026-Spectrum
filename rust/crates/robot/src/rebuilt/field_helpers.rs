//! Flip and mirror helpers between the blue and red field frames.
//!
//! Each `*_if_red` has a `*_for` twin that takes the alliance as `red`, so the math runs
//! without a driver station.

use super::field;
use std::f64::consts::PI;
use wpimath::geometry::{Pose2d, Pose3d, Rotation2d, Translation2d, Translation3d};

pub fn flip_angle(angle: f64) -> f64 {
    (angle + 180.0) % 360.0
}

pub fn flip_angle_rotation(angle: Rotation2d) -> Rotation2d {
    angle.rotate_by(Rotation2d::from_degrees(180.0))
}

pub fn flip_angle_if_red(blue: f64) -> f64 {
    flip_angle_if_red_for(field::is_red(), blue)
}

pub fn flip_angle_if_red_for(red: bool, blue: f64) -> f64 {
    if red {
        return (blue + 180.0) % 360.0;
    }
    blue
}

pub fn flip_angle_rotation_if_red(blue: Rotation2d) -> Rotation2d {
    flip_angle_rotation_if_red_for(field::is_red(), blue)
}

pub fn flip_angle_rotation_if_red_for(red: bool, blue: Rotation2d) -> Rotation2d {
    if red {
        return blue.rotate_by(Rotation2d::from_degrees(180.0));
    }
    blue
}

pub fn flip_if_red_translation2d(blue: Translation2d) -> Translation2d {
    flip_if_red_translation2d_for(field::is_red(), blue)
}

pub fn flip_if_red_translation2d_for(red: bool, blue: Translation2d) -> Translation2d {
    Translation2d::new(
        flip_x_if_red_for(red, blue.x),
        flip_y_if_red_for(red, blue.y),
    )
}

pub fn flip_if_red_translation3d(blue: Translation3d) -> Translation3d {
    flip_if_red_translation3d_for(field::is_red(), blue)
}

pub fn flip_if_red_translation3d_for(red: bool, blue: Translation3d) -> Translation3d {
    Translation3d::new(
        flip_x_if_red_for(red, blue.x),
        flip_y_if_red_for(red, blue.y),
        blue.z,
    )
}

pub fn flip_if_red_pose2d(pose: Pose2d) -> Pose2d {
    flip_if_red_pose2d_for(field::is_red(), pose)
}

pub fn flip_if_red_pose2d_for(red: bool, pose: Pose2d) -> Pose2d {
    Pose2d::new(
        flip_if_red_translation2d_for(red, pose.translation),
        flip_angle_rotation_if_red_for(red, pose.rotation),
    )
}

pub fn flip_x(x_coordinate: f64) -> f64 {
    field::FIELD_LENGTH - x_coordinate
}

pub fn flip_y(y_coordinate: f64) -> f64 {
    field::FIELD_WIDTH - y_coordinate
}

pub fn flip_x_if_red(x_coordinate: f64) -> f64 {
    flip_x_if_red_for(field::is_red(), x_coordinate)
}

pub fn flip_x_if_red_for(red: bool, x_coordinate: f64) -> f64 {
    if red {
        return field::FIELD_LENGTH - x_coordinate;
    }
    x_coordinate
}

pub fn flip_y_if_red(y_coordinate: f64) -> f64 {
    flip_y_if_red_for(field::is_red(), y_coordinate)
}

pub fn flip_y_if_red_for(red: bool, y_coordinate: f64) -> f64 {
    if red {
        return field::FIELD_WIDTH - y_coordinate;
    }
    y_coordinate
}

/// Normalizes an angle in radians to [-pi, pi).
pub fn normalize_angle(angle: f64) -> f64 {
    let mut angle = angle % (2.0 * PI);
    if angle >= PI {
        angle -= 2.0 * PI;
    }
    if angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

pub fn pose_out_of_field(pose2d: Pose2d) -> bool {
    let x = pose2d.x();
    let y = pose2d.y();
    x <= 0.0 || x >= field::FIELD_LENGTH || y <= 0.0 || y >= field::FIELD_WIDTH
}

pub fn pose_out_of_field_3d(pose3d: Pose3d) -> bool {
    pose_out_of_field(pose3d.to_pose2d())
}

#[cfg(test)]
mod tests {
    use super::*;
    use golden::check;

    fn red(flag: f64) -> bool {
        flag != 0.0
    }

    #[test]
    fn matches_java_field_helpers() {
        check("field_helpers", 1e-9, |op, i| {
            Some(match op {
                "flip_angle" => vec![flip_angle(i[0])],
                "flip_angle_if_red" => vec![flip_angle_if_red_for(red(i[0]), i[1])],
                "flip_angle_rotation" => {
                    let r = flip_angle_rotation(Rotation2d::new(i[0]));
                    vec![r.radians()]
                }
                "flip_angle_rotation_if_red" => {
                    let r = flip_angle_rotation_if_red_for(red(i[0]), Rotation2d::new(i[1]));
                    vec![r.radians()]
                }
                "flip_if_red_translation2d" => {
                    let t =
                        flip_if_red_translation2d_for(red(i[0]), Translation2d::new(i[1], i[2]));
                    vec![t.x, t.y]
                }
                "flip_if_red_translation3d" => {
                    let t = flip_if_red_translation3d_for(
                        red(i[0]),
                        Translation3d::new(i[1], i[2], i[3]),
                    );
                    vec![t.x, t.y, t.z]
                }
                "flip_if_red_pose2d" => {
                    let p = flip_if_red_pose2d_for(
                        red(i[0]),
                        Pose2d::new(Translation2d::new(i[1], i[2]), Rotation2d::new(i[3])),
                    );
                    vec![p.x(), p.y(), p.rotation.radians()]
                }
                "flip_x" => vec![flip_x(i[0])],
                "flip_y" => vec![flip_y(i[0])],
                "flip_x_if_red" => vec![flip_x_if_red_for(red(i[0]), i[1])],
                "flip_y_if_red" => vec![flip_y_if_red_for(red(i[0]), i[1])],
                "normalize_angle" => vec![normalize_angle(i[0])],
                "pose_out_of_field" => {
                    let p = Pose2d::new(Translation2d::new(i[0], i[1]), Rotation2d::ZERO);
                    vec![f64::from(u8::from(pose_out_of_field(p)))]
                }
                "blue_to_red" => vec![field::blue_to_red(i[0])],
                "is_blue" => vec![f64::from(u8::from(field::is_blue_for(match i[0] as i32 {
                    0 => None,
                    1 => Some(wpilib::driver_station::Alliance::Blue),
                    _ => Some(wpilib::driver_station::Alliance::Red),
                })))],
                _ => return None,
            })
        });
    }

    #[test]
    fn normalize_angle_keeps_the_half_open_range() {
        assert_eq!(normalize_angle(PI), -PI);
        assert!((normalize_angle(-PI) + PI).abs() < 1e-12);
        assert!((normalize_angle(3.0 * PI) + PI).abs() < 1e-12);
    }

    #[test]
    fn red_flips_x_and_y_and_turns_the_heading() {
        let flipped = flip_if_red_pose2d_for(
            true,
            Pose2d::new(Translation2d::new(1.0, 2.0), Rotation2d::ZERO),
        );
        assert!((flipped.x() - (field::FIELD_LENGTH - 1.0)).abs() < 1e-12);
        assert!((flipped.y() - (field::FIELD_WIDTH - 2.0)).abs() < 1e-12);
        assert!((flipped.rotation.radians().abs() - PI).abs() < 1e-12);
    }
}
