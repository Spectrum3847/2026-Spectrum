//! Math and driver station helpers. Based on 254's Util, imported from 1678-2024.
//!
//! Java overloads become suffixed names: `limit_min_max`, `in_range_min_max`,
//! `in_range_suppliers`, `epsilon_equals_default`, `epsilon_equals_i32`.

use commands::{RobotModeTriggers, Trigger};
use std::fmt::Display;
use wpilib::driver_station;
use wpimath::math_util;

/// Tolerance [`epsilon_equals_default`] uses for floating point equality.
pub const EPSILON: f64 = 1e-12;

/// Clamps `v` to [-`max_magnitude`, `max_magnitude`].
pub fn limit(v: f64, max_magnitude: f64) -> f64 {
    limit_min_max(v, -max_magnitude, max_magnitude)
}

/// Clamps `v` to [`min`, `max`], both bounds inclusive.
pub fn limit_min_max(v: f64, min: f64, max: f64) -> f64 {
    math_util::clamp(v, min, max)
}

/// True when the magnitude of `v` is under `max_magnitude`, bounds exclusive.
pub fn in_range(v: f64, max_magnitude: f64) -> bool {
    in_range_min_max(v, -max_magnitude, max_magnitude)
}

/// True when `v` sits between `min` and `max`, both bounds exclusive.
pub fn in_range_min_max(v: f64, min: f64, max: f64) -> bool {
    v > min && v < max
}

/// Reads `v` once and tests it against the two bounds. Use this instead of
/// [`in_range_min_max`] on values that are still moving.
pub fn in_range_suppliers(
    v: impl Fn() -> f64,
    min: impl Fn() -> f64,
    max: impl Fn() -> f64,
) -> bool {
    let value = v();
    value > min() && value < max()
}

/// Interpolates from `a` at `x = 0` to `b` at `x = 1`, clamping x.
pub fn interpolate(a: f64, b: f64, x: f64) -> f64 {
    math_util::interpolate(a, b, x)
}

pub fn join_strings(delim: &str, strings: &[impl Display]) -> String {
    strings
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(delim)
}

/// True when `a` and `b` differ by `epsilon` or less.
pub fn epsilon_equals(a: f64, b: f64, epsilon: f64) -> bool {
    (a - epsilon <= b) && (a + epsilon >= b)
}

pub fn epsilon_equals_default(a: f64, b: f64) -> bool {
    epsilon_equals(a, b, EPSILON)
}

/// Widens to i64 before subtracting, so comparing values near the int limits cannot
/// overflow into a wrong answer.
pub fn epsilon_equals_i32(a: i32, b: i32, epsilon: i32) -> bool {
    (i64::from(a) - i64::from(epsilon) <= i64::from(b))
        && (i64::from(a) + i64::from(epsilon) >= i64::from(b))
}

pub fn all_close_to(list: &[f64], value: f64, epsilon: f64) -> bool {
    list.iter().all(|v| epsilon_equals(*v, value, epsilon))
}

// Java's static Triggers. Built on first use and handed out as clones, which share the
// same condition and event loop.
thread_local! {
    static TELEOP: Trigger = RobotModeTriggers::teleop();
    static AUTO_MODE: Trigger = RobotModeTriggers::autonomous();
    static TEST_MODE: Trigger = RobotModeTriggers::test();
    static DISABLED: Trigger = RobotModeTriggers::disabled();
    static DS_ATTACHED: Trigger = Trigger::new(driver_station::is_ds_attached);
}

pub fn teleop() -> Trigger {
    TELEOP.with(Trigger::clone)
}

pub fn auto_mode() -> Trigger {
    AUTO_MODE.with(Trigger::clone)
}

pub fn test_mode() -> Trigger {
    TEST_MODE.with(Trigger::clone)
}

pub fn disabled() -> Trigger {
    DISABLED.with(Trigger::clone)
}

pub fn ds_attached() -> Trigger {
    DS_ATTACHED.with(Trigger::clone)
}

#[cfg(test)]
mod tests {
    use super::*;
    use commands::testing::{FakeRobotState, Mode};

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-6, "{expected} vs {actual}");
    }

    #[test]
    fn limit_clamps_to_magnitude_and_bounds() {
        assert_close(5.0, limit(10.0, 5.0));
        assert_close(-5.0, limit(-10.0, 5.0));
        assert_close(3.0, limit(3.0, 5.0));

        assert_close(10.0, limit_min_max(15.0, 0.0, 10.0));
        assert_close(0.0, limit_min_max(-5.0, 0.0, 10.0));
        assert_close(7.0, limit_min_max(7.0, 0.0, 10.0));
    }

    #[test]
    fn in_range_bounds_are_exclusive() {
        assert!(in_range(3.0, 5.0));
        assert!(!in_range(5.0, 5.0));
        assert!(!in_range(6.0, 5.0));

        assert!(in_range_min_max(5.0, 0.0, 10.0));
        assert!(!in_range_min_max(0.0, 0.0, 10.0));
        assert!(!in_range_min_max(10.0, 0.0, 10.0));

        assert!(in_range_suppliers(|| 5.0, || 0.0, || 10.0));
        assert!(!in_range_suppliers(|| 0.0, || 0.0, || 10.0));
    }

    #[test]
    fn in_range_suppliers_read_the_value_once() {
        let reads = std::cell::Cell::new(0);
        in_range_suppliers(
            || {
                reads.set(reads.get() + 1);
                1.0
            },
            || 0.0,
            || 2.0,
        );
        assert_eq!(reads.get(), 1);
    }

    #[test]
    fn interpolate_clamps_the_factor() {
        assert_close(10.0, interpolate(10.0, 20.0, 0.0));
        assert_close(15.0, interpolate(10.0, 20.0, 0.5));
        assert_close(20.0, interpolate(10.0, 20.0, 1.0));
        assert_close(10.0, interpolate(10.0, 20.0, -0.5));
        assert_close(20.0, interpolate(10.0, 20.0, 1.5));
    }

    #[test]
    fn join_strings_uses_the_delimiter() {
        let items = ["apple", "banana", "cherry"];
        assert_eq!(join_strings(", ", &items), "apple, banana, cherry");
        assert_eq!(join_strings("|", &items), "apple|banana|cherry");
        assert_eq!(join_strings(",", &[1, 2, 3]), "1,2,3");
    }

    #[test]
    fn epsilon_equals_for_doubles_and_ints() {
        assert!(epsilon_equals(1.0, 1.000_000_000_000_1, 1e-6));
        assert!(!epsilon_equals(1.0, 1.01, 1e-6));

        assert!(epsilon_equals_default(1.0, 1.0 + EPSILON / 2.0));
        assert!(!epsilon_equals_default(1.0, 1.0 + EPSILON * 2.0));

        assert!(epsilon_equals_i32(10, 12, 2));
        assert!(!epsilon_equals_i32(10, 13, 2));
    }

    #[test]
    fn epsilon_equals_i32_does_not_overflow_near_the_limits() {
        assert!(!epsilon_equals_i32(i32::MIN, i32::MAX, 1));
        assert!(epsilon_equals_i32(i32::MAX, i32::MAX - 1, 1));
    }

    #[test]
    fn all_close_to_checks_every_element() {
        let list = [1.0, 1.0001, 0.9999];
        assert!(all_close_to(&list, 1.0, 1e-3));
        assert!(!all_close_to(&list, 1.0, 1e-5));
    }

    #[test]
    fn mode_triggers_follow_the_robot_state() {
        let state = FakeRobotState::install();
        state.set_mode(Mode::Teleop);
        assert!(teleop().get_as_boolean());
        assert!(!disabled().get_as_boolean());
        state.set_mode(Mode::Disabled);
        assert!(disabled().get_as_boolean());
        assert!(!teleop().get_as_boolean());
        state.set_mode(Mode::Autonomous);
        assert!(auto_mode().get_as_boolean());
        state.set_mode(Mode::Test);
        assert!(test_mode().get_as_boolean());
    }
}
