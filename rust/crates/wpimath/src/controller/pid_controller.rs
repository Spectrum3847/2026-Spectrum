use crate::math_util;

/// PID controller with continuous input, integrator range and I-zone, like WPILib.
#[derive(Clone, Debug)]
pub struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    i_zone: f64,
    period: f64,
    maximum_integral: f64,
    minimum_integral: f64,
    maximum_input: f64,
    minimum_input: f64,
    continuous: bool,
    error: f64,
    error_derivative: f64,
    prev_error: f64,
    total_error: f64,
    error_tolerance: f64,
    error_derivative_tolerance: f64,
    setpoint: f64,
    measurement: f64,
    have_measurement: bool,
    have_setpoint: bool,
}

impl PidController {
    /// Uses WPILib's default 20 ms period.
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self::with_period(kp, ki, kd, 0.02)
    }

    /// # Panics
    /// If a gain is negative or `period` is not positive.
    pub fn with_period(kp: f64, ki: f64, kd: f64, period: f64) -> Self {
        assert!(kp >= 0.0, "Kp must be a non-negative number!");
        assert!(ki >= 0.0, "Ki must be a non-negative number!");
        assert!(kd >= 0.0, "Kd must be a non-negative number!");
        assert!(period > 0.0, "Controller period must be a positive number!");
        Self {
            kp,
            ki,
            kd,
            i_zone: f64::INFINITY,
            period,
            maximum_integral: 1.0,
            minimum_integral: -1.0,
            maximum_input: 0.0,
            minimum_input: 0.0,
            continuous: false,
            error: 0.0,
            error_derivative: 0.0,
            prev_error: 0.0,
            total_error: 0.0,
            error_tolerance: 0.05,
            error_derivative_tolerance: f64::INFINITY,
            setpoint: 0.0,
            measurement: 0.0,
            have_measurement: false,
            have_setpoint: false,
        }
    }

    pub fn set_pid(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Error magnitude above which the integral resets. Infinite by default.
    ///
    /// # Panics
    /// If `i_zone` is negative.
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

    pub fn i_zone(&self) -> f64 {
        self.i_zone
    }

    pub fn period(&self) -> f64 {
        self.period
    }

    pub fn accumulated_error(&self) -> f64 {
        self.total_error
    }

    fn compute_error(&self) -> f64 {
        if self.continuous {
            let error_bound = (self.maximum_input - self.minimum_input) / 2.0;
            math_util::input_modulus(self.setpoint - self.measurement, -error_bound, error_bound)
        } else {
            self.setpoint - self.measurement
        }
    }

    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
        self.have_setpoint = true;
        self.error = self.compute_error();
        self.error_derivative = (self.error - self.prev_error) / self.period;
    }

    pub fn setpoint(&self) -> f64 {
        self.setpoint
    }

    pub fn at_setpoint(&self) -> bool {
        self.have_measurement
            && self.have_setpoint
            && self.error.abs() < self.error_tolerance
            && self.error_derivative.abs() < self.error_derivative_tolerance
    }

    /// Treats inputs as wrapping from `maximum_input` to `minimum_input`.
    pub fn enable_continuous_input(&mut self, minimum_input: f64, maximum_input: f64) {
        self.continuous = true;
        self.minimum_input = minimum_input;
        self.maximum_input = maximum_input;
    }

    pub fn disable_continuous_input(&mut self) {
        self.continuous = false;
    }

    pub fn is_continuous_input_enabled(&self) -> bool {
        self.continuous
    }

    pub fn set_integrator_range(&mut self, minimum_integral: f64, maximum_integral: f64) {
        self.minimum_integral = minimum_integral;
        self.maximum_integral = maximum_integral;
    }

    pub fn set_tolerance(&mut self, error_tolerance: f64) {
        self.set_tolerance_with_derivative(error_tolerance, f64::INFINITY);
    }

    pub fn set_tolerance_with_derivative(
        &mut self,
        error_tolerance: f64,
        error_derivative_tolerance: f64,
    ) {
        self.error_tolerance = error_tolerance;
        self.error_derivative_tolerance = error_derivative_tolerance;
    }

    pub fn error_tolerance(&self) -> f64 {
        self.error_tolerance
    }

    pub fn error_derivative_tolerance(&self) -> f64 {
        self.error_derivative_tolerance
    }

    pub fn error(&self) -> f64 {
        self.error
    }

    pub fn error_derivative(&self) -> f64 {
        self.error_derivative
    }

    pub fn calculate(&mut self, measurement: f64, setpoint: f64) -> f64 {
        self.setpoint = setpoint;
        self.have_setpoint = true;
        self.calculate_measurement(measurement)
    }

    /// Output for `measurement` against the last setpoint.
    pub fn calculate_measurement(&mut self, measurement: f64) -> f64 {
        self.measurement = measurement;
        self.prev_error = self.error;
        self.have_measurement = true;
        self.error = self.compute_error();
        self.error_derivative = (self.error - self.prev_error) / self.period;
        if self.error.abs() > self.i_zone {
            self.total_error = 0.0;
        } else if self.ki != 0.0 {
            self.total_error = math_util::clamp(
                self.total_error + self.error * self.period,
                self.minimum_integral / self.ki,
                self.maximum_integral / self.ki,
            );
        }
        self.kp * self.error + self.ki * self.total_error + self.kd * self.error_derivative
    }

    pub fn reset(&mut self) {
        self.error = 0.0;
        self.prev_error = 0.0;
        self.total_error = 0.0;
        self.error_derivative = 0.0;
        self.have_measurement = false;
    }
}
