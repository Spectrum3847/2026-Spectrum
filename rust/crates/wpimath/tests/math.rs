//! MathUtil, Units, interpolation and LinearFilter against Java golden fixtures.

use wpimath::filter::LinearFilter;
use wpimath::geometry::{Pose2d, Rotation2d};
use wpimath::interpolation::{InterpolatingDoubleTreeMap, TimeInterpolatableBuffer};
use wpimath::{math_util, units};

fn flag(b: bool) -> f64 {
    f64::from(b)
}

#[test]
fn math_util_and_units_match_java() {
    golden::check("mathutil", 1e-9, |op, x| {
        Some(match op {
            "clamp" => vec![math_util::clamp(x[0], x[1], x[2])],
            "apply_deadband" => vec![math_util::apply_deadband_with_max(x[0], x[1], x[2])],
            "input_modulus" => vec![math_util::input_modulus(x[0], x[1], x[2])],
            "angle_modulus" => vec![math_util::angle_modulus(x[0])],
            "interpolate" => vec![math_util::interpolate(x[0], x[1], x[2])],
            "inverse_interpolate" => vec![math_util::inverse_interpolate(x[0], x[1], x[2])],
            "is_near" => vec![flag(math_util::is_near(x[0], x[1], x[2]))],
            "is_near_range" => vec![flag(math_util::is_near_wrapped(
                x[0], x[1], x[2], x[3], x[4],
            ))],
            "units" => {
                let v = x[0];
                vec![
                    units::meters_to_feet(v),
                    units::feet_to_meters(v),
                    units::meters_to_inches(v),
                    units::inches_to_meters(v),
                    units::degrees_to_radians(v),
                    units::radians_to_degrees(v),
                    units::radians_to_rotations(v),
                    units::degrees_to_rotations(v),
                    units::rotations_to_degrees(v),
                    units::rotations_to_radians(v),
                    units::rotations_per_minute_to_radians_per_second(v),
                    units::radians_per_second_to_rotations_per_minute(v),
                    units::miles_per_hour_to_meters_per_second(v),
                    units::meters_per_second_to_miles_per_hour(v),
                    units::milliseconds_to_seconds(v),
                    units::seconds_to_milliseconds(v),
                    units::kilograms_to_lbs(v),
                    units::lbs_to_kilograms(v),
                ]
            }
            _ => return None,
        })
    });
}

#[test]
fn apply_deadband_defaults_to_unit_range() {
    assert_eq!(
        math_util::apply_deadband(0.5, 0.2),
        math_util::apply_deadband_with_max(0.5, 0.2, 1.0)
    );
}

#[test]
fn clamp_propagates_nan_like_java() {
    assert!(math_util::clamp(f64::NAN, 0.0, 1.0).is_nan());
}

#[test]
#[should_panic(expected = "non-negative")]
fn is_near_rejects_negative_tolerance() {
    math_util::is_near(0.0, 0.0, -1.0);
}

#[test]
fn tree_map_and_time_buffer_match_java() {
    golden::check("interpolation", 1e-9, |op, x| match op {
        "tree_map" => {
            let n = x[0] as usize;
            let mut map = InterpolatingDoubleTreeMap::new();
            for i in 0..n {
                map.put(x[1 + 2 * i], x[2 + 2 * i]);
            }
            Some(
                x[1 + 2 * n..]
                    .iter()
                    .map(|q| map.get(*q).expect("map has entries"))
                    .collect(),
            )
        }
        "time_buffer" => {
            let mut buf = TimeInterpolatableBuffer::<f64>::new(x[0]);
            let mut out = Vec::new();
            for e in x[2..].chunks(3).take(x[1] as usize) {
                if e[0] == 0.0 {
                    buf.add_sample(e[1], e[2]);
                } else {
                    let s = buf.get_sample(e[1]);
                    out.push(flag(s.is_some()));
                    out.push(s.unwrap_or(0.0));
                }
            }
            Some(out)
        }
        _ => None,
    });
}

#[test]
fn pose_buffer_matches_java() {
    golden::check("pose_buffer", 1e-9, |op, x| {
        if op != "pose_buffer" {
            return None;
        }
        let mut buf = TimeInterpolatableBuffer::<Pose2d>::new(x[0]);
        let mut out = Vec::new();
        for e in x[1..].chunks(5) {
            if e[0] == 0.0 {
                buf.add_sample(e[1], Pose2d::from_xy(e[2], e[3], Rotation2d::new(e[4])));
            } else {
                match buf.get_sample(e[1]) {
                    Some(p) => out.extend([1.0, p.x(), p.y(), p.rotation.radians()]),
                    None => out.extend([0.0, 0.0, 0.0, 0.0]),
                }
            }
        }
        Some(out)
    });
}

#[test]
fn empty_tree_map_has_no_value() {
    assert_eq!(InterpolatingDoubleTreeMap::new().get(1.0), None);
}

#[test]
fn linear_filter_matches_java() {
    golden::check("linear_filter", 1e-9, |op, x| {
        if op != "filter" {
            return None;
        }
        let mut f = match x[0] as i32 {
            0 => LinearFilter::moving_average(x[1] as usize),
            1 => LinearFilter::single_pole_iir(x[1], x[2]),
            _ => LinearFilter::high_pass(x[1], x[2]),
        };
        let n = x[3] as usize;
        let mut out: Vec<f64> = x[4..4 + n].iter().map(|v| f.calculate(*v)).collect();
        out.push(f.last_value());
        Some(out)
    });
}

#[test]
fn linear_filter_reset_clears_history() {
    let mut f = LinearFilter::moving_average(3);
    f.calculate(3.0);
    f.reset();
    assert_eq!(f.calculate(3.0), 1.0);
}

#[test]
#[should_panic(expected = "taps")]
fn moving_average_needs_a_tap() {
    let _ = LinearFilter::moving_average(0);
}
