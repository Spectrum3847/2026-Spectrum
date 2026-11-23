//! Subset of WPILib `MathUtil`.

// Java `Math.min`/`Math.max` return NaN if either input is NaN; Rust's do not.
fn java_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.min(b)
    }
}

fn java_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else {
        a.max(b)
    }
}

/// Unlike `f64::clamp`, accepts `low > high` like WPILib.
pub fn clamp(value: f64, low: f64, high: f64) -> f64 {
    java_max(low, java_min(value, high))
}

/// Zeroes values inside `deadband` and rescales the rest to keep `max_magnitude` reachable.
pub fn apply_deadband_with_max(value: f64, deadband: f64, max_magnitude: f64) -> f64 {
    if value.abs() < deadband {
        return 0.0;
    }
    if value > 0.0 {
        (1.0 + deadband / (max_magnitude - deadband)) * (value - deadband)
    } else {
        (1.0 + deadband / (max_magnitude - deadband)) * (value + deadband)
    }
}

pub fn apply_deadband(value: f64, deadband: f64) -> f64 {
    apply_deadband_with_max(value, deadband, 1.0)
}

/// Wraps `input` into `[minimum_input, maximum_input)`.
pub fn input_modulus(input: f64, minimum_input: f64, maximum_input: f64) -> f64 {
    let modulus = maximum_input - minimum_input;
    // `as i32` truncates toward zero and saturates, like the Java int cast.
    let num_max = ((input - minimum_input) / modulus) as i32;
    let input = input - f64::from(num_max) * modulus;
    let num_min = ((input - maximum_input) / modulus) as i32;
    input - f64::from(num_min) * modulus
}

/// Wraps radians into `[-pi, pi)`.
pub fn angle_modulus(angle_radians: f64) -> f64 {
    input_modulus(angle_radians, -std::f64::consts::PI, std::f64::consts::PI)
}

/// Linear interpolation with `t` clamped to `[0, 1]`.
pub fn interpolate(start_value: f64, end_value: f64, t: f64) -> f64 {
    start_value + (end_value - start_value) * clamp(t, 0.0, 1.0)
}

/// Position of `q` within `[start_value, end_value]` as a fraction in `[0, 1]`.
pub fn inverse_interpolate(start_value: f64, end_value: f64, q: f64) -> f64 {
    let total_range = end_value - start_value;
    if total_range <= 0.0 {
        return 0.0;
    }
    let query_to_start = q - start_value;
    if query_to_start <= 0.0 {
        return 0.0;
    }
    query_to_start / total_range
}

/// # Panics
/// If `tolerance` is negative.
pub fn is_near(expected: f64, actual: f64, tolerance: f64) -> bool {
    assert!(tolerance >= 0.0, "Tolerance must be a non-negative number!");
    (expected - actual).abs() < tolerance
}

/// Like [`is_near`] for a continuous range that wraps from `max` to `min`.
///
/// # Panics
/// If `tolerance` is negative.
pub fn is_near_wrapped(expected: f64, actual: f64, tolerance: f64, min: f64, max: f64) -> bool {
    assert!(tolerance >= 0.0, "Tolerance must be a non-negative number!");
    let error_bound = (max - min) / 2.0;
    let error = input_modulus(expected - actual, -error_bound, error_bound);
    error.abs() < tolerance
}
