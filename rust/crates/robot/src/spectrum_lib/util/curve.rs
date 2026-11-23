// Spectrum 3847
// Based on Code from FRC# 2363

use crate::spectrum_lib::util::java_double;
use wpimath::math_util;

/// Offset, scalar and deadzone shared by every [`Curve`]. All three start at 0, as in
/// Java. The cells let a gamepad retune a curve through `&self`.
#[derive(Debug, Default)]
pub struct CurveBase {
    offset: std::cell::Cell<f64>,
    scalar: std::cell::Cell<f64>,
    deadzone: std::cell::Cell<f64>,
}

/// Maps controller stick inputs through a curve. Implement this rather than using the
/// base directly, for example [`ExpCurve`](super::exp_curve::ExpCurve).
pub trait Curve {
    fn base(&self) -> &CurveBase;

    /// Maps a stick input, expected in [-1, 1], to the curve's output.
    fn calculate(&self, input: f64) -> f64;

    /// Blanks the middle of the curve, then rescales what is left to fill [-1, 1] again.
    fn calculate_deadzone(&self, input: f64) -> f64 {
        // apply_deadband takes a half width, so halve the full width we were given.
        math_util::apply_deadband(input, self.base().deadzone.get() / 2.0)
    }

    fn calculate_scalar(&self, input: f64) -> f64 {
        input * self.base().scalar.get()
    }

    fn calculate_offset(&self, input: f64) -> f64 {
        input + self.base().offset.get()
    }

    /// Samples the curve at `point_count` evenly spaced inputs across [-1, 1]. With a
    /// count of 1, the single point is the center of the input range.
    ///
    /// Returns [x, y] pairs, one per sample.
    fn get_curve_points(&self, point_count: usize) -> Vec<[f64; 2]> {
        if point_count == 0 {
            return Vec::new();
        }
        if point_count == 1 {
            return vec![[0.0, self.calculate(0.0)]];
        }
        let dx = 2.0 / (point_count - 1) as f64;
        (0..point_count)
            .map(|i| {
                let x = -1.0 + (i as f64 * dx);
                [x, self.calculate(x)]
            })
            .collect()
    }

    /// Prints [x, y] pairs for pasting into Desmos.
    fn print_points(&self, points: &[[f64; 2]]) {
        println!();
        for point in points {
            print!(
                "({},{}),",
                java_double::to_string(point[0]),
                java_double::to_string(point[1])
            );
        }
    }

    /// Prints [`Self::get_curve_points`] for pasting into Desmos.
    fn print_points_count(&self, point_count: usize) {
        self.print_points(&self.get_curve_points(point_count));
    }

    fn set_offset(&self, offset: f64) {
        self.base().offset.set(offset);
    }

    fn set_scalar(&self, scalar: f64) {
        self.base().scalar.set(scalar);
    }

    /// Sets the deadband width centred on zero. A negative width is stored as its absolute
    /// value.
    fn set_deadzone(&self, deadzone: f64) {
        self.base().deadzone.set(deadzone.abs());
    }

    fn get_offset(&self) -> f64 {
        self.base().offset.get()
    }

    fn get_scalar(&self) -> f64 {
        self.base().scalar.get()
    }

    fn get_deadzone(&self) -> f64 {
        self.base().deadzone.get()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct TestCurve {
        base: CurveBase,
    }

    impl Curve for TestCurve {
        fn base(&self) -> &CurveBase {
            &self.base
        }

        /// The full deadzone, scalar, offset pipeline.
        fn calculate(&self, input: f64) -> f64 {
            let deadband_val = self.calculate_deadzone(input);
            let scaled_val = self.calculate_scalar(deadband_val);
            self.calculate_offset(scaled_val)
        }
    }

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-6, "{expected} vs {actual}");
    }

    #[test]
    fn getters_and_setters_including_negative_deadzone() {
        let curve = TestCurve::default();
        curve.set_offset(2.5);
        assert_close(2.5, curve.get_offset());

        curve.set_scalar(1.5);
        assert_close(1.5, curve.get_scalar());

        curve.set_deadzone(0.1);
        assert_close(0.1, curve.get_deadzone());

        curve.set_deadzone(-0.2);
        assert_close(0.2, curve.get_deadzone());
    }

    #[test]
    fn fields_start_at_zero() {
        let curve = TestCurve::default();
        assert_eq!(curve.get_offset(), 0.0);
        assert_eq!(curve.get_scalar(), 0.0);
        assert_eq!(curve.get_deadzone(), 0.0);
    }

    #[test]
    fn deadzone_remaps_inputs_inside_and_outside_the_band() {
        let curve = TestCurve::default();
        curve.set_deadzone(0.2);
        curve.set_scalar(1.0);
        curve.set_offset(0.0);

        assert_close(0.0, curve.calculate(0.0));
        assert_close(0.0, curve.calculate(0.05));
        assert_close(0.0, curve.calculate(-0.05));
        assert_close(0.0, curve.calculate(0.1));
        assert_close(0.0, curve.calculate(-0.1));

        // (1 / (1 - 0.1)) * (0.55 - 0.1) = 0.5
        assert_close(0.5, curve.calculate(0.55));
        assert_close(-0.5, curve.calculate(-0.55));
        assert_close(1.0, curve.calculate(1.0));
        assert_close(-1.0, curve.calculate(-1.0));
    }

    #[test]
    fn scalar_and_offset_apply_after_the_deadzone() {
        let curve = TestCurve::default();
        curve.set_deadzone(0.0);
        curve.set_scalar(2.0);
        curve.set_offset(1.0);

        assert_close(1.0, curve.calculate(0.0));
        assert_close(3.0, curve.calculate(1.0));
        assert_close(-1.0, curve.calculate(-1.0));
    }

    #[test]
    fn curve_points_sample_evenly_and_a_single_point_is_the_center() {
        let curve = TestCurve::default();
        curve.set_deadzone(0.0);
        curve.set_scalar(1.0);
        curve.set_offset(0.5);
        let single = curve.get_curve_points(1);
        curve.set_offset(0.0);
        assert_eq!(single.len(), 1);
        assert_close(0.0, single[0][0]);
        assert_close(0.5, single[0][1]);

        let points = curve.get_curve_points(5);
        assert_eq!(points.len(), 5);
        for (i, x) in [-1.0, -0.5, 0.0, 0.5, 1.0].into_iter().enumerate() {
            assert_close(x, points[i][0]);
            assert_close(x, points[i][1]);
        }
    }

    #[test]
    fn zero_points_is_empty() {
        assert!(TestCurve::default().get_curve_points(0).is_empty());
    }
}
