//! Drives simulated Phoenix devices through the C layer.
//!
//! Run from `rust/` with the natives on the library path:
//! `cargo xtask sim sim_smoke -p phoenix6`

use phoenix6::configs::{ConfigValue, ParentConfiguration, TalonFXConfiguration};
use phoenix6::controls::{SolidColor, VoltageOut};
use phoenix6::signals::{InvertedValue, NeutralModeValue, RgbwColor};
use phoenix6::status_signal::{latency_compensated_value, refresh_all};
use phoenix6::{CANcoder, CANdle, CanBus, Pigeon2, StatusCode, TalonFX, unmanaged, utils};
use std::time::Duration;
use wpilib::driver_station;

wpilib::native_lib! {
    struct HalSim = "wpiHal" {
        fn HALSIM_SetDriverStationEnabled(enabled: i32);
        fn HALSIM_SetDriverStationDsAttached(attached: i32);
        fn HALSIM_NotifyDriverStationNewData();
    }
}

fn pause(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}

fn main() {
    wpilib::hal::initialize();
    println!(
        "simulation: hal={} phoenix={}",
        wpilib::is_simulation(),
        utils::is_simulation()
    );
    println!(
        "phoenix version {}, api compliancy {}",
        unmanaged::phoenix_version(),
        unmanaged::api_compliancy()
    );

    let bus = CanBus::roborio();
    let talon = TalonFX::new(1, bus.clone()).expect("talon");
    let cancoder = CANcoder::new(2, bus.clone()).expect("cancoder");
    let pigeon = Pigeon2::new(0, bus.clone()).expect("pigeon");
    let candle = CANdle::new(1, bus.clone()).expect("candle");
    println!("bad id: {:?}", TalonFX::new(63, bus.clone()).err());

    // Config round trip through the device's stored text.
    let mut config = TalonFXConfiguration::default();
    config.motor_output.inverted = InvertedValue::Clockwise_Positive;
    config.motor_output.neutral_mode = NeutralModeValue::Brake;
    config.current_limits.supply_current_limit = 40.0;
    config.slot0.k_p = 1.5;
    config.motion_magic.motion_magic_cruise_velocity = 80.0;
    let status = talon.configurator().apply(&config);
    let mut read_back = TalonFXConfiguration::default();
    let refresh = talon.configurator().refresh(&mut read_back);
    // The device stores floats as f32, so compare with that precision.
    let same =
        config
            .items()
            .iter()
            .zip(read_back.items())
            .all(|(a, b)| match (a.value, b.value) {
                (ConfigValue::Double(x), ConfigValue::Double(y)) => (x - y).abs() < 1e-6,
                (x, y) => x == y,
            });
    println!("config apply {status}, refresh {refresh}, round trip equal within f32: {same}");
    println!(
        "serialized head: {:?}",
        config.serialize().lines().take(3).collect::<Vec<_>>()
    );

    pause(300);
    println!(
        "connected: talon={} cancoder={} pigeon={}",
        talon.is_connected(),
        cancoder.is_connected(),
        pigeon.is_connected()
    );

    // Disabled: the feeder sends zero, so the motor stays off.
    let talon_sim = talon.sim_state();
    talon_sim.set_supply_voltage(12.0);
    println!("set_control: {}", talon.set_control(&VoltageOut::new(6.0)));
    driver_station::refresh_data();
    pause(200);
    println!("disabled: motor voltage {:.3} V", talon_sim.motor_voltage());

    // Enable the simulated driver station; the feeder passes it on.
    // SAFETY: signatures from the 2026.2.1 HAL sim headers.
    let hal_sim = unsafe { HalSim::load() }.expect("libwpiHal");
    unsafe {
        (hal_sim.HALSIM_SetDriverStationDsAttached)(1);
        (hal_sim.HALSIM_SetDriverStationEnabled)(1);
        (hal_sim.HALSIM_NotifyDriverStationNewData)();
    }
    pause(50);
    driver_station::refresh_data();
    println!("ds enabled: {}", driver_station::is_enabled());
    for step in 0..5 {
        pause(100);
        println!(
            "  t={}ms enable_state={} motor voltage {:.3} V",
            (step + 1) * 100,
            unmanaged::enable_state(),
            talon_sim.motor_voltage()
        );
    }

    // Physics inputs, then read the signals back.
    talon_sim.set_rotor_velocity(10.0);
    talon_sim.set_raw_rotor_position(2.5);
    let cancoder_sim = cancoder.sim_state();
    cancoder_sim.set_raw_position(0.25);
    cancoder_sim.set_velocity(1.5);
    let pigeon_sim = pigeon.sim_state();
    pigeon_sim.set_raw_yaw(90.0);
    pigeon_sim.set_angular_velocity_z(45.0);
    pause(150);

    let mut position = talon.position(false);
    let mut velocity = talon.velocity(false);
    let mut voltage = talon.motor_voltage(false);
    let mut duty = talon.duty_cycle(false);
    let mut temp = talon.device_temp(false);
    let status = refresh_all(&mut [
        &mut position,
        &mut velocity,
        &mut voltage,
        &mut duty,
        &mut temp,
    ]);
    println!(
        "talon refresh_all {status}: position {:.3} rot, velocity {:.3} rps, voltage {:.3} V, duty {:.3}, temp {:.1} C",
        position.value(),
        velocity.value(),
        voltage.value(),
        duty.value(),
        temp.value()
    );
    println!(
        "  latency {:.4} s, compensated position {:.4}, applied position rate {} Hz, update ok {}",
        position.timestamp().latency(),
        latency_compensated_value(&position, &velocity),
        position.applied_update_frequency(),
        position.set_update_frequency(50.0)
    );
    println!(
        "  optimize bus utilization: {}",
        talon.optimize_bus_utilization()
    );
    println!("  stop_motor: {}", talon.stop_motor());

    let mut absolute = cancoder.absolute_position(true);
    let mut cancoder_velocity = cancoder.velocity(true);
    println!(
        "cancoder absolute {:.3} rot, velocity {:.3} rps ({})",
        absolute.refresh().value(),
        cancoder_velocity.refresh().value(),
        absolute.status()
    );
    println!(
        "pigeon yaw {:.2} deg, pitch {:.2}, z rate {:.2} dps",
        pigeon.yaw(true).value(),
        pigeon.pitch(true).value(),
        pigeon.angular_velocity_z_world(true).value()
    );

    let candle_status: StatusCode =
        candle.set_control(&SolidColor::new(8, 15).with_color(RgbwColor::new(255, 0, 0)));
    println!("candle solid color: {candle_status}");

    let can_status = bus.status();
    println!(
        "bus status {} utilization {:.2} off {} rec {} tec {}",
        can_status.status,
        can_status.bus_utilization,
        can_status.bus_off_count,
        can_status.rec,
        can_status.tec
    );
    println!(
        "hoot load of a missing file: {:?}",
        CanBus::with_hoot_file("rio", "./logs/spectrum.hoot").map(|b| b.name().to_string())
    );
    wpilib::hal::shutdown();
}
