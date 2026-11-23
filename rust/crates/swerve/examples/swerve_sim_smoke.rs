//! Four-module drivetrain with the robot's FM2026 constants, run on the Phoenix simulator with
//! a crude motor model. Prints odometry and telemetry rate.
//!
//! Run from `rust/` with the natives on the library path:
//! `cargo xtask sim swerve_sim_smoke -p swerve`

use phoenix6::configs::{
    CANcoderConfiguration, CurrentLimitsConfigs, Pigeon2Configuration, Slot0Configs,
    TalonFXConfiguration,
};
use phoenix6::signals::StaticFeedforwardSignValue;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};
use swerve::{
    ApplyRobotSpeeds, ClosedLoopOutputType, DriveRequestType, DrivetrainConstants,
    DrivetrainOptions, ModuleConstants, ModuleConstantsFactory, SteerFeedbackType,
    SteerRequestType, SwerveDrivetrain,
};
use wpilib::driver_station;
use wpimath::kinematics::ChassisSpeeds;

wpilib::native_lib! {
    struct HalSim = "wpiHal" {
        fn HALSIM_SetDriverStationEnabled(enabled: i32);
        fn HALSIM_SetDriverStationDsAttached(attached: i32);
        fn HALSIM_NotifyDriverStationNewData();
    }
}

const INCH: f64 = 0.0254;
const HALF: f64 = 21.75 / 2.0 * INCH;
const DRIVE_GEAR: f64 = 6.03;

/// SwerveConfig.java with the simulation overrides of MapleSimSwerveDrivetrain: no inversions,
/// zero encoder offsets (FM2026's are -0.164, 0.249, 0.272, -0.310 on the robot), sim steer gains.
fn module_constants() -> [ModuleConstants; 4] {
    let steer_gains = Slot0Configs::default()
        .with_k_p(1000.0)
        .with_k_d(60.0)
        .with_k_s(0.15)
        .with_k_v(1.5)
        .with_static_feedforward_sign(StaticFeedforwardSignValue::UseClosedLoopSign);
    let drive_gains = Slot0Configs::default().with_k_p(10.0).with_k_s(4.0);
    let drive_initial = TalonFXConfiguration::default().with_current_limits(
        CurrentLimitsConfigs::default()
            .with_stator_current_limit(80.0)
            .with_stator_current_limit_enable(true)
            .with_supply_current_limit(30.0)
            .with_supply_current_limit_enable(true)
            .with_supply_current_lower_limit(30.0),
    );
    let steer_initial = TalonFXConfiguration::default().with_current_limits(
        CurrentLimitsConfigs::default()
            .with_stator_current_limit(60.0)
            .with_stator_current_limit_enable(true),
    );
    let factory = ModuleConstantsFactory {
        base: ModuleConstants {
            drive_motor_gear_ratio: DRIVE_GEAR,
            steer_motor_gear_ratio: 21.428571428571427,
            wheel_radius_m: 1.964 * INCH,
            slip_current_a: 80.0,
            steer_motor_gains: steer_gains,
            drive_motor_gains: drive_gains,
            steer_motor_closed_loop_output: ClosedLoopOutputType::TorqueCurrentFoc,
            drive_motor_closed_loop_output: ClosedLoopOutputType::TorqueCurrentFoc,
            speed_at_12_volts_mps: 5.12,
            feedback_source: SteerFeedbackType::FusedCANcoder,
            coupling_gear_ratio: 4.5,
            drive_motor_initial_configs: Some(drive_initial),
            steer_motor_initial_configs: Some(steer_initial),
            encoder_initial_configs: Some(CANcoderConfiguration::default()),
            steer_inertia: 0.05,
            drive_inertia: 0.01,
            steer_friction_voltage: 0.05,
            drive_friction_voltage: 0.1,
            ..ModuleConstants::default()
        },
    };
    // (steer id, drive id, encoder id, x sign, y sign)
    let layout = [
        (2, 1, 3, 1.0, 1.0),
        (12, 11, 13, 1.0, -1.0),
        (22, 21, 23, -1.0, 1.0),
        (32, 31, 33, -1.0, -1.0),
    ];
    let mut out = Vec::new();
    for (steer, drive, encoder, x, y) in layout {
        out.push(factory.create_module_constants(
            steer,
            drive,
            encoder,
            0.0,
            x * HALF,
            y * HALF,
            false,
            false,
            false,
        ));
    }
    out.try_into().expect("four modules")
}

fn pause(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}

fn main() {
    wpilib::hal::initialize();
    let bus_name = std::env::var("SWERVE_BUS").unwrap_or_else(|_| "*".into());
    let drivetrain_constants = DrivetrainConstants::default()
        .with_can_bus_name(bus_name)
        .with_pigeon2_id(0)
        .with_pigeon2_configs(Pigeon2Configuration::default());
    let modules = module_constants();
    let drivetrain = SwerveDrivetrain::new(
        &drivetrain_constants,
        &DrivetrainOptions {
            odometry_update_frequency_hz: 250.0,
            ..DrivetrainOptions::default()
        },
        &modules,
    )
    .expect("drivetrain");
    println!(
        "odometry {} Hz, can fd {}",
        drivetrain.get_odometry_frequency(),
        drivetrain.is_on_can_fd()
    );

    let telemetry_calls = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&telemetry_calls);
    drivetrain.register_telemetry(move |_| {
        counter.fetch_add(1, Ordering::Relaxed);
    });

    // SAFETY: signatures from the 2026.2.1 HAL sim headers.
    let hal_sim = unsafe { HalSim::load() }.expect("libwpiHal");
    unsafe {
        (hal_sim.HALSIM_SetDriverStationDsAttached)(1);
        (hal_sim.HALSIM_SetDriverStationEnabled)(1);
        (hal_sim.HALSIM_NotifyDriverStationNewData)();
    }
    pause(50);
    driver_station::refresh_data();

    let sims: Vec<_> = drivetrain
        .get_modules()
        .iter()
        .map(|m| {
            let drive = m.drive_motor().sim_state();
            let steer = m.steer_motor().sim_state();
            let encoder = m.encoder().sim_state();
            drive.set_supply_voltage(12.0);
            steer.set_supply_voltage(12.0);
            encoder.set_supply_voltage(12.0);
            encoder.set_raw_position(0.0);
            (drive, encoder)
        })
        .collect();
    drivetrain
        .get_pigeon2()
        .sim_state()
        .set_supply_voltage(12.0);
    drivetrain.get_pigeon2().sim_state().set_raw_yaw(0.0);

    let request = Arc::new(Mutex::new(ApplyRobotSpeeds {
        speeds: ChassisSpeeds::new(1.0, 0.0, 0.0),
        drive_request_type: DriveRequestType::Velocity,
        steer_request_type: SteerRequestType::Position,
        desaturate_wheel_speeds: true,
        ..ApplyRobotSpeeds::default()
    }));
    drivetrain.set_control(&request);

    // Drive rotor: torque current accelerates it against light damping.
    let kt = 9.37 / 483.0;
    let inertia = 0.002;
    let damping = 0.002;
    let mut rotor_rps = [0.0_f64; 4];
    let dt = 0.005;
    let start = Instant::now();
    let mut last_print = 0.0;
    let mut last_calls = 0;
    while start.elapsed().as_secs_f64() < 3.0 {
        driver_station::refresh_data();
        for (i, (drive, encoder)) in sims.iter().enumerate() {
            let torque = kt * drive.torque_current() - damping * rotor_rps[i];
            rotor_rps[i] += torque / inertia / (2.0 * std::f64::consts::PI) * dt;
            drive.set_rotor_velocity(rotor_rps[i]);
            drive.add_rotor_position(rotor_rps[i] * dt);
            encoder.set_velocity(0.0);
        }
        let t = start.elapsed().as_secs_f64();
        if t - last_print >= 0.5 {
            let state = drivetrain.get_state();
            let calls = telemetry_calls.load(Ordering::Relaxed);
            println!(
                "t={t:.2}s pose=({:.3}, {:.3}, {:.1} deg) vx={:.3} module0 speed={:.3} target={:.3} \
                 telemetry {:.0} Hz, daqs ok/fail {}/{}, period {:.4} s, valid {}",
                state.pose.x(),
                state.pose.y(),
                state.pose.rotation.degrees(),
                state.speeds.vx,
                state.module_states[0].speed,
                state.module_targets[0].speed,
                (calls - last_calls) as f64 / (t - last_print),
                state.successful_daqs,
                state.failed_daqs,
                state.odometry_period,
                drivetrain.is_odometry_valid(),
            );
            last_print = t;
            last_calls = calls;
        }
        pause(5);
    }
    let state = drivetrain.get_state();
    println!(
        "final pose ({:.3}, {:.3}), wheel positions {:?}",
        state.pose.x(),
        state.pose.y(),
        state
            .module_positions
            .iter()
            .map(|p| (p.distance * 1000.0).round() / 1000.0)
            .collect::<Vec<_>>()
    );
    drivetrain.set_control_once(swerve::Idle);
    drop(drivetrain);
    wpilib::hal::shutdown();
}
