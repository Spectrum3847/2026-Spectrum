//! Motor models, plants, discretization and mechanism sims against Java golden fixtures.

use nalgebra::{DMatrix, SMatrix, SVector};
use wpimath::sim::{ElevatorSim, FlywheelSim, SingleJointedArmSim};
use wpimath::system::{
    DcMotor, create_elevator_system, create_flywheel_system, create_single_jointed_arm_system,
    discretize_ab, matrix_exp, rkdp,
};

fn row_major(m: &DMatrix<f64>) -> Vec<f64> {
    m.transpose().iter().copied().collect()
}

/// `discretize_ab` for the `(states, inputs)` sizes the fixtures use.
fn discretize(n: usize, m: usize, dt: f64, x: &[f64]) -> Vec<f64> {
    macro_rules! case {
        ($s:literal, $i:literal) => {{
            let a = SMatrix::<f64, $s, $s>::from_row_slice(&x[3..3 + $s * $s]);
            let b = SMatrix::<f64, $s, $i>::from_row_slice(&x[3 + $s * $s..3 + $s * $s + $s * $i]);
            let (da, db) = discretize_ab(&a, &b, dt);
            let mut out: Vec<f64> = da.transpose().iter().copied().collect();
            out.extend(db.transpose().iter().copied());
            out
        }};
    }
    match (n, m) {
        (1, 1) => case!(1, 1),
        (1, 2) => case!(1, 2),
        (2, 1) => case!(2, 1),
        (2, 2) => case!(2, 2),
        (3, 1) => case!(3, 1),
        (3, 2) => case!(3, 2),
        _ => panic!("unsupported size {n}x{m}"),
    }
}

#[test]
fn systems_match_java() {
    golden::check("systems", 1e-9, |op, x| {
        Some(match op {
            "dcmotor_kraken_x60_foc" => {
                let m = DcMotor::kraken_x60_foc(x[0] as i32);
                let r = m.with_reduction(x[6]);
                vec![
                    m.r_ohms,
                    m.kv_rad_per_sec_per_volt,
                    m.kt_nm_per_amp,
                    m.stall_torque_newton_meters,
                    m.stall_current_amps,
                    m.free_current_amps,
                    m.free_speed_rad_per_sec,
                    m.nominal_voltage_volts,
                    m.current(x[1], x[2]),
                    m.current_from_torque(x[3]),
                    m.torque(x[4]),
                    m.voltage(x[3], x[1]),
                    m.speed(x[3], x[2]),
                    r.r_ohms,
                    r.kv_rad_per_sec_per_volt,
                    r.kt_nm_per_amp,
                    r.stall_torque_newton_meters,
                    r.free_speed_rad_per_sec,
                ]
            }
            "system_flywheel" => {
                let s = create_flywheel_system(DcMotor::kraken_x60_foc(x[0] as i32), x[1], x[2]);
                vec![s.a()[0], s.b()[0], s.c()[0], s.d()[0]]
            }
            "system_elevator" => {
                let s =
                    create_elevator_system(DcMotor::kraken_x60_foc(x[0] as i32), x[1], x[2], x[3]);
                vec![
                    s.a()[(0, 0)],
                    s.a()[(0, 1)],
                    s.a()[(1, 0)],
                    s.a()[(1, 1)],
                    s.b()[(0, 0)],
                    s.b()[(1, 0)],
                ]
            }
            "system_arm" => {
                let s = create_single_jointed_arm_system(
                    DcMotor::kraken_x60_foc(x[0] as i32),
                    x[1],
                    x[2],
                );
                vec![
                    s.a()[(0, 0)],
                    s.a()[(0, 1)],
                    s.a()[(1, 0)],
                    s.a()[(1, 1)],
                    s.b()[(0, 0)],
                    s.b()[(1, 0)],
                ]
            }
            "matrix_exp" => {
                let n = x[0] as usize;
                row_major(&matrix_exp(&DMatrix::from_row_slice(n, n, &x[1..])))
            }
            "discretize_ab" => discretize(x[0] as usize, x[1] as usize, x[2], x),
            "rkdp_pendulum" => {
                let k = x[4];
                let r = rkdp(
                    |s: &SVector<f64, 2>, u: &SVector<f64, 1>| {
                        SVector::<f64, 2>::new(s[1], -k * s[0].sin() + u[0])
                    },
                    SVector::<f64, 2>::new(x[0], x[1]),
                    SVector::<f64, 1>::new(x[2]),
                    x[3],
                );
                vec![r[0], r[1]]
            }
            _ => return None,
        })
    });
}

#[test]
fn flywheel_sim_matches_java() {
    golden::check("flywheel_sim", 1e-9, |op, x| {
        if op != "flywheel" {
            return None;
        }
        let motor = DcMotor::kraken_x60_foc(x[0] as i32);
        let mut sim = FlywheelSim::new(create_flywheel_system(motor, x[1], x[2]), motor);
        let mut out = Vec::new();
        for e in x[4..].chunks(3).take(x[3] as usize) {
            if e[0] == 1.0 {
                sim.set_angular_velocity(e[1]);
            } else {
                sim.set_input(e[1]);
                sim.update(e[2]);
            }
            out.extend([
                sim.angular_velocity_rad_per_sec(),
                sim.angular_velocity_rpm(),
                sim.angular_acceleration_rad_per_sec_sq(),
                sim.torque_newton_meters(),
                sim.current_draw_amps(),
            ]);
        }
        Some(out)
    });
}

#[test]
fn elevator_sim_matches_java() {
    golden::check("elevator_sim", 1e-9, |op, x| {
        if op != "elevator" {
            return None;
        }
        let mut sim = ElevatorSim::new(
            DcMotor::kraken_x60_foc(x[0] as i32),
            x[1],
            x[2],
            x[3],
            x[4],
            x[5],
            x[6] == 1.0,
            x[7],
        );
        let mut out = Vec::new();
        for e in x[9..].chunks(3).take(x[8] as usize) {
            if e[0] == 1.0 {
                sim.set_state(e[1], e[2]);
            } else {
                sim.set_input(e[1]);
                sim.update(e[2]);
            }
            out.extend([
                sim.position_meters(),
                sim.velocity_meters_per_second(),
                sim.current_draw_amps(),
                f64::from(sim.has_hit_lower_limit()),
                f64::from(sim.has_hit_upper_limit()),
            ]);
        }
        Some(out)
    });
}

#[test]
fn arm_sim_matches_java() {
    // The stiff arm cases amplify last-bit differences in cos and pow between Java's
    // intrinsics and libm. Java's own Math and StrictMath differ by 6e-9 on one case.
    golden::check("arm_sim", 1e-7, |op, x| {
        if op != "arm" {
            return None;
        }
        let mut sim = SingleJointedArmSim::new(
            DcMotor::kraken_x60_foc(x[0] as i32),
            x[1],
            x[2],
            x[3],
            x[4],
            x[5],
            x[6] == 1.0,
            x[7],
        );
        let mut out = Vec::new();
        for e in x[9..].chunks(3).take(x[8] as usize) {
            if e[0] == 1.0 {
                sim.set_state(e[1], e[2]);
            } else {
                sim.set_input(e[1]);
                sim.update(e[2]);
            }
            out.extend([
                sim.angle_rads(),
                sim.velocity_rad_per_sec(),
                sim.current_draw_amps(),
                f64::from(sim.has_hit_lower_limit()),
                f64::from(sim.has_hit_upper_limit()),
            ]);
        }
        Some(out)
    });
}

#[test]
#[should_panic(expected = "gearing must be greater than zero")]
fn flywheel_system_rejects_zero_gearing() {
    let _ = create_flywheel_system(DcMotor::kraken_x60_foc(1), 0.01, 0.0);
}

#[test]
fn elevator_stops_at_the_upper_limit() {
    let mut sim = ElevatorSim::new(
        DcMotor::kraken_x60_foc(1),
        5.0,
        4.0,
        0.02,
        0.0,
        0.5,
        false,
        0.0,
    );
    sim.set_input(12.0);
    for _ in 0..500 {
        sim.update(0.02);
    }
    assert!(sim.has_hit_upper_limit());
    assert_eq!(sim.velocity_meters_per_second(), 0.0);
}
