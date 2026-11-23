use super::{DcMotor, LinearSystem};
use nalgebra::{SMatrix, SVector};

/// Flywheel with states `[velocity]` (rad/s) and input `[voltage]`.
///
/// # Panics
/// If `j_kg_meters_squared` or `gearing` is not positive.
pub fn create_flywheel_system(
    motor: DcMotor,
    j_kg_meters_squared: f64,
    gearing: f64,
) -> LinearSystem<1, 1, 1> {
    assert!(j_kg_meters_squared > 0.0, "J must be greater than zero.");
    assert!(gearing > 0.0, "gearing must be greater than zero.");
    LinearSystem::new(
        SMatrix::<f64, 1, 1>::new(
            -gearing * gearing * motor.kt_nm_per_amp
                / (motor.kv_rad_per_sec_per_volt * motor.r_ohms * j_kg_meters_squared),
        ),
        SMatrix::<f64, 1, 1>::new(
            gearing * motor.kt_nm_per_amp / (motor.r_ohms * j_kg_meters_squared),
        ),
        SMatrix::<f64, 1, 1>::identity(),
        SMatrix::<f64, 1, 1>::zeros(),
    )
}

/// Elevator with states `[position, velocity]` (m, m/s) and input `[voltage]`.
///
/// # Panics
/// If `mass_kg`, `radius_meters` or `gearing` is not positive.
pub fn create_elevator_system(
    motor: DcMotor,
    mass_kg: f64,
    radius_meters: f64,
    gearing: f64,
) -> LinearSystem<2, 1, 2> {
    assert!(mass_kg > 0.0, "massKg must be greater than zero.");
    assert!(
        radius_meters > 0.0,
        "radiusMeters must be greater than zero."
    );
    assert!(gearing > 0.0, "gearing must be greater than zero.");
    LinearSystem::new(
        SMatrix::<f64, 2, 2>::new(
            0.0,
            1.0,
            0.0,
            -gearing.powi(2) * motor.kt_nm_per_amp
                / (motor.r_ohms
                    * radius_meters
                    * radius_meters
                    * mass_kg
                    * motor.kv_rad_per_sec_per_volt),
        ),
        SVector::<f64, 2>::new(
            0.0,
            gearing * motor.kt_nm_per_amp / (motor.r_ohms * radius_meters * mass_kg),
        ),
        SMatrix::<f64, 2, 2>::identity(),
        SMatrix::<f64, 2, 1>::zeros(),
    )
}

/// Single-jointed arm with states `[angle, velocity]` (rad, rad/s) and input `[voltage]`.
///
/// # Panics
/// If `j_kg_squared_meters` or `gearing` is not positive.
pub fn create_single_jointed_arm_system(
    motor: DcMotor,
    j_kg_squared_meters: f64,
    gearing: f64,
) -> LinearSystem<2, 1, 2> {
    assert!(
        j_kg_squared_meters > 0.0,
        "JKgSquaredMeters must be greater than zero."
    );
    assert!(gearing > 0.0, "gearing must be greater than zero.");
    LinearSystem::new(
        SMatrix::<f64, 2, 2>::new(
            0.0,
            1.0,
            0.0,
            -gearing.powi(2) * motor.kt_nm_per_amp
                / (motor.kv_rad_per_sec_per_volt * motor.r_ohms * j_kg_squared_meters),
        ),
        SVector::<f64, 2>::new(
            0.0,
            gearing * motor.kt_nm_per_amp / (motor.r_ohms * j_kg_squared_meters),
        ),
        SMatrix::<f64, 2, 2>::identity(),
        SMatrix::<f64, 2, 1>::zeros(),
    )
}
