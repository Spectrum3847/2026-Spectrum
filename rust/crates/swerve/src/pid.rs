//! `com.ctre.phoenix6.swerve.utility.PhoenixPIDController`: a PID controller that takes the
//! timestamp of each sample, so request timing does not depend on the wall clock.

use wpimath::math_util::{clamp, input_modulus};

#[derive(Clone, Debug)]
pub struct PhoenixPidController {
    kp: f64,
    ki: f64,
    kd: f64,
    i_zone: f64,
    maximum_integral: f64,
    minimum_integral: f64,
    maximum_input: f64,
    minimum_input: f64,
    continuous: bool,
    position_error: f64,
    velocity_error: f64,
    prev_error: f64,
    total_error: f64,
    position_tolerance: f64,
    velocity_tolerance: f64,
    setpoint: f64,
    have_measurement: bool,
    have_setpoint: bool,
    last_applied_output: f64,
    last_timestamp: f64,
}

impl PhoenixPidController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            i_zone: f64::INFINITY,
            maximum_integral: 1.0,
            minimum_integral: -1.0,
            maximum_input: 0.0,
            minimum_input: 0.0,
            continuous: false,
            position_error: 0.0,
            velocity_error: 0.0,
            prev_error: 0.0,
            total_error: 0.0,
            position_tolerance: 0.05,
            velocity_tolerance: f64::INFINITY,
            setpoint: 0.0,
            have_measurement: false,
            have_setpoint: false,
            last_applied_output: 0.0,
            last_timestamp: 0.0,
        }
    }

    pub fn set_pid(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// # Panics
    /// If `i_zone` is negative, like Java's `IllegalArgumentException`.
    pub fn set_i_zone(&mut self, i_zone: f64) {
        assert!(i_zone >= 0.0, "IZone must be a non-negative number!");
        self.i_zone = i_zone;
    }

    pub fn p(&self) -> f64 {
        self.kp
    }

    pub fn i(&self) -> f64 {
        self.ki
    }

    pub fn d(&self) -> f64 {
        self.kd
    }

    pub fn setpoint(&self) -> f64 {
        self.setpoint
    }

    pub fn at_setpoint(&self) -> bool {
        self.have_measurement
            && self.have_setpoint
            && self.position_error.abs() < self.position_tolerance
            && self.velocity_error.abs() < self.velocity_tolerance
    }

    pub fn enable_continuous_input(&mut self, minimum: f64, maximum: f64) {
        self.continuous = true;
        self.minimum_input = minimum;
        self.maximum_input = maximum;
    }

    pub fn disable_continuous_input(&mut self) {
        self.continuous = false;
    }

    pub fn is_continuous_input_enabled(&self) -> bool {
        self.continuous
    }

    pub fn set_integrator_range(&mut self, minimum: f64, maximum: f64) {
        self.minimum_integral = minimum;
        self.maximum_integral = maximum;
    }

    pub fn set_tolerance(&mut self, position: f64, velocity: f64) {
        self.position_tolerance = position;
        self.velocity_tolerance = velocity;
    }

    pub fn position_error(&self) -> f64 {
        self.position_error
    }

    pub fn velocity_error(&self) -> f64 {
        self.velocity_error
    }

    pub fn last_applied_output(&self) -> f64 {
        self.last_applied_output
    }

    /// `timestamp_seconds` is on any clock that only moves forward. The first call sees the
    /// period since time zero, as in Java, so its derivative term is near zero.
    pub fn calculate(&mut self, measurement: f64, setpoint: f64, timestamp_seconds: f64) -> f64 {
        self.setpoint = setpoint;
        self.have_setpoint = true;
        self.have_measurement = true;
        self.prev_error = self.position_error;
        let period = timestamp_seconds - self.last_timestamp;
        self.last_timestamp = timestamp_seconds;

        self.position_error = if self.continuous {
            let bound = (self.maximum_input - self.minimum_input) / 2.0;
            input_modulus(setpoint - measurement, -bound, bound)
        } else {
            setpoint - measurement
        };
        self.velocity_error = (self.position_error - self.prev_error) / period;

        if self.position_error.abs() > self.i_zone {
            self.total_error = 0.0;
        } else if self.ki != 0.0 {
            self.total_error = clamp(
                self.total_error + self.position_error * period,
                self.minimum_integral / self.ki,
                self.maximum_integral / self.ki,
            );
        }
        self.last_applied_output = self.kp * self.position_error
            + self.ki * self.total_error
            + self.kd * self.velocity_error;
        self.last_applied_output
    }

    pub fn reset(&mut self) {
        self.position_error = 0.0;
        self.prev_error = 0.0;
        self.total_error = 0.0;
        self.velocity_error = 0.0;
        self.have_measurement = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn proportional_output() {
        let mut pid = PhoenixPidController::new(2.0, 0.0, 0.0);
        assert_eq!(pid.calculate(1.0, 3.0, 0.02), 4.0);
        assert_eq!(pid.position_error(), 2.0);
        assert_eq!(pid.setpoint(), 3.0);
    }

    #[test]
    fn continuous_input_takes_the_short_way() {
        let mut pid = PhoenixPidController::new(1.0, 0.0, 0.0);
        pid.enable_continuous_input(-PI, PI);
        let out = pid.calculate(PI - 0.1, -PI + 0.1, 0.02);
        assert!((out - 0.2).abs() < 1e-12);
        assert!(pid.is_continuous_input_enabled());
    }

    #[test]
    fn derivative_uses_the_timestamp_difference() {
        let mut pid = PhoenixPidController::new(0.0, 0.0, 1.0);
        pid.calculate(0.0, 1.0, 1.0);
        // error 1 -> 0.5 over 0.25 s
        let out = pid.calculate(0.5, 1.0, 1.25);
        assert!((out - (-2.0)).abs() < 1e-12);
        assert!((pid.velocity_error() + 2.0).abs() < 1e-12);
    }

    #[test]
    fn integral_accumulates_and_is_clamped() {
        let mut pid = PhoenixPidController::new(0.0, 2.0, 0.0);
        pid.calculate(1.0, 1.0, 1.0);
        let out = pid.calculate(0.0, 1.0, 1.1);
        assert!((out - 0.2).abs() < 1e-12);
        // integral range is +-1.0 of output, so total error caps at 0.5
        let out = pid.calculate(0.0, 1.0, 100.0);
        assert!((out - 1.0).abs() < 1e-12);
    }

    #[test]
    fn i_zone_clears_the_integral() {
        let mut pid = PhoenixPidController::new(0.0, 1.0, 0.0);
        pid.set_i_zone(0.5);
        pid.calculate(0.0, 0.25, 1.0);
        pid.calculate(0.0, 0.25, 2.0);
        assert!(pid.last_applied_output() > 0.0);
        let out = pid.calculate(0.0, 2.0, 3.0);
        assert_eq!(out, 0.0);
    }

    #[test]
    fn at_setpoint_needs_a_sample_and_the_tolerance() {
        let mut pid = PhoenixPidController::new(1.0, 0.0, 0.0);
        assert!(!pid.at_setpoint());
        pid.calculate(1.0, 1.01, 0.02);
        assert!(pid.at_setpoint());
        pid.set_tolerance(0.001, f64::INFINITY);
        assert!(!pid.at_setpoint());
        pid.reset();
        assert!(!pid.at_setpoint());
        assert_eq!(pid.position_error(), 0.0);
    }
}
