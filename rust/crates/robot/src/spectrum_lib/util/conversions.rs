pub fn rpm_to_rps(rpm: f64) -> f64 {
    rpm / 60.0
}

/// Java's `RPMtoRPS(DoubleSupplier)` overload.
pub fn rpm_to_rps_supplier(rpm: impl Fn() -> f64) -> f64 {
    rpm() / 60.0
}

pub fn rps_to_rpm(rps: f64) -> f64 {
    rps * 60.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-6, "{expected} vs {actual}");
    }

    #[test]
    fn rpm_to_rps_with_a_value() {
        assert_close(0.0, rpm_to_rps(0.0));
        assert_close(1.0, rpm_to_rps(60.0));
        assert_close(100.0, rpm_to_rps(6000.0));
        assert_close(-50.0, rpm_to_rps(-3000.0));
    }

    #[test]
    fn rpm_to_rps_with_a_supplier() {
        assert_close(1.0, rpm_to_rps_supplier(|| 60.0));
        assert_close(50.0, rpm_to_rps_supplier(|| 3000.0));
    }

    #[test]
    fn rps_to_rpm_scales_by_sixty() {
        assert_close(0.0, rps_to_rpm(0.0));
        assert_close(60.0, rps_to_rpm(1.0));
        assert_close(6000.0, rps_to_rpm(100.0));
        assert_close(-3000.0, rps_to_rpm(-50.0));
    }
}
