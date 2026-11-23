// Spectrum 3847
// Based on Code from FRC# 2363

use crate::spectrum_lib::util::curve::{Curve, CurveBase};
use std::cell::Cell;

/// Maps a stick input in [-1, 1] through an exponential of base `exp_val`, keeping the
/// result in [-1, 1] and preserving sign. The scalar and offset applied after the exponent
/// set the range of the final value.
#[derive(Debug)]
pub struct ExpCurve {
    base: CurveBase,
    exp_val: Cell<f64>,
}

impl Default for ExpCurve {
    fn default() -> Self {
        Self::with_params(1.0, 0.0, 1.0, 0.0)
    }
}

impl ExpCurve {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_params(exp_val: f64, offset: f64, scalar: f64, deadzone: f64) -> Self {
        let curve = Self {
            base: CurveBase::default(),
            exp_val: Cell::new(1.0),
        };
        curve.set_exp_val(exp_val);
        curve.set_offset(offset);
        curve.set_scalar(scalar);
        curve.set_deadzone(deadzone);
        curve
    }

    /// Applies a signed power curve of base `exp_val`. A base above 1 gives fine control
    /// near the middle of the range, a base below 1 near the ends. Both ends still land
    /// on -1 and 1, so the scalar keeps its meaning as the maximum output.
    fn calculate_exp_val(&self, input: f64) -> f64 {
        let exp_val = self.exp_val.get();
        if exp_val == 1.0 {
            return input;
        }
        (exp_val.powf(input.abs()) - 1.0) / (exp_val - 1.0) * input.signum()
    }

    /// A base at or below zero falls back to 1.0, leaving the input uncurved. A negative
    /// base would raise a fractional power and produce imaginary results.
    pub fn set_exp_val(&self, exp_val: f64) {
        self.exp_val.set(if exp_val <= 0.0 { 1.0 } else { exp_val });
    }

    pub fn get_exp_val(&self) -> f64 {
        self.exp_val.get()
    }
}

impl Curve for ExpCurve {
    fn base(&self) -> &CurveBase {
        &self.base
    }

    /// Applies the deadzone, then the exponent, then the scalar, then the offset.
    fn calculate(&self, input: f64) -> f64 {
        self.calculate_offset(
            self.calculate_scalar(self.calculate_exp_val(self.calculate_deadzone(input))),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-6, "{expected} vs {actual}");
    }

    #[test]
    fn default_constructor_values() {
        let curve = ExpCurve::new();
        assert_close(1.0, curve.get_exp_val());
        assert_close(0.0, curve.get_offset());
        assert_close(1.0, curve.get_scalar());
        assert_close(0.0, curve.get_deadzone());
    }

    #[test]
    fn parameterized_constructor_values() {
        let curve = ExpCurve::with_params(2.5, 0.1, 1.2, 0.05);
        assert_close(2.5, curve.get_exp_val());
        assert_close(0.1, curve.get_offset());
        assert_close(1.2, curve.get_scalar());
        assert_close(0.05, curve.get_deadzone());
    }

    #[test]
    fn non_positive_exp_val_falls_back_to_one() {
        let curve = ExpCurve::new();
        curve.set_exp_val(0.0);
        assert_close(1.0, curve.get_exp_val());
        curve.set_exp_val(-5.0);
        assert_close(1.0, curve.get_exp_val());
    }

    #[test]
    fn linear_when_exp_val_is_one() {
        let curve = ExpCurve::with_params(1.0, 0.0, 1.0, 0.0);
        assert_close(0.0, curve.calculate(0.0));
        assert_close(0.5, curve.calculate(0.5));
        assert_close(-0.5, curve.calculate(-0.5));
        assert_close(1.0, curve.calculate(1.0));
        assert_close(-1.0, curve.calculate(-1.0));
    }

    #[test]
    fn exponential_response_and_full_pipeline() {
        let exp_val: f64 = 3.0;
        let curve = ExpCurve::with_params(exp_val, 0.0, 1.0, 0.0);

        let expected_05 = (exp_val.powf(0.5) - 1.0) / (exp_val - 1.0);
        assert_close(expected_05, curve.calculate(0.5));
        assert_close(-expected_05, curve.calculate(-0.5));
        assert_close(1.0, curve.calculate(1.0));
        assert_close(-1.0, curve.calculate(-1.0));

        let (deadzone, scalar, offset) = (0.2, 1.5, 0.5);
        let curve2 = ExpCurve::with_params(exp_val, offset, scalar, deadzone);
        let dead_radius = deadzone / 2.0;

        let deadbanded_05 = (1.0 / (1.0 - dead_radius)) * (0.5 - dead_radius);
        let expected_full_05 =
            ((exp_val.powf(deadbanded_05) - 1.0) / (exp_val - 1.0)) * scalar + offset;
        assert_close(expected_full_05, curve2.calculate(0.5));

        let deadbanded_neg_05 = (1.0 / (1.0 - dead_radius)) * (-0.5 + dead_radius);
        let expected_full_neg_05 =
            -((exp_val.powf(deadbanded_neg_05.abs()) - 1.0) / (exp_val - 1.0)) * scalar + offset;
        assert_close(expected_full_neg_05, curve2.calculate(-0.5));
    }

    #[test]
    fn zero_input_stays_zero_plus_offset() {
        let curve = ExpCurve::with_params(3.0, 0.25, 2.0, 0.0);
        assert_close(0.25, curve.calculate(0.0));
    }
}
