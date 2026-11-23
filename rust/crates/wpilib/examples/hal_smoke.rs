//! Loads the real HAL and exercises the core wpilib pieces for about two seconds.
//!
//! Run from `rust/` with the natives on the library path:
//! `cargo xtask sim hal_smoke -p wpilib`

use std::cell::Cell;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;
use wpilib::driver_station;
use wpilib::filesystem;
use wpilib::hid::GenericHid;
use wpilib::notifier::Notifier;
use wpilib::power_distribution::PowerDistribution;
use wpilib::pwm::Servo;
use wpilib::robot_controller;
use wpilib::timer::{self, Timer};
use wpilib::{Robot, TimedRobot, hal};

#[derive(Default)]
struct CountingRobot {
    robot_init: u32,
    disabled_init: u32,
    robot_periodic: u32,
    disabled_periodic: u32,
    simulation_periodic: u32,
}

impl Robot for CountingRobot {
    fn robot_init(&mut self) {
        self.robot_init += 1;
    }

    fn disabled_init(&mut self) {
        self.disabled_init += 1;
    }

    fn disabled_periodic(&mut self) {
        self.disabled_periodic += 1;
    }

    fn simulation_periodic(&mut self) {
        self.simulation_periodic += 1;
    }

    fn robot_periodic(&mut self) {
        self.robot_periodic += 1;
    }
}

fn main() {
    hal::initialize();
    println!(
        "runtime: {:?}, simulation: {}",
        hal::runtime_type(),
        wpilib::is_simulation()
    );

    let t0 = robot_controller::fpga_time();
    std::thread::sleep(Duration::from_millis(50));
    let t1 = robot_controller::fpga_time();
    println!(
        "fpga time: {t0} us, then {t1} us (+{} us over a 50 ms sleep)",
        t1 - t0
    );
    println!("serial: {:?}", robot_controller::serial_number());
    println!(
        "battery: {:.2} V, input current: {:.2} A, browned out: {}, brownout threshold: {:.2} V",
        robot_controller::battery_voltage(),
        robot_controller::input_current(),
        robot_controller::is_browned_out(),
        robot_controller::brownout_voltage(),
    );
    match robot_controller::set_brownout_voltage(4.6) {
        Ok(()) => println!(
            "brownout set to {:.2} V",
            robot_controller::brownout_voltage()
        ),
        Err(e) => println!("set_brownout_voltage failed: {e}"),
    }

    driver_station::refresh_data();
    println!(
        "ds: enabled={} disabled={} auto={} fms={} attached={} alliance={:?} match_time={}",
        driver_station::is_enabled(),
        driver_station::is_disabled(),
        driver_station::is_autonomous(),
        driver_station::is_fms_attached(),
        driver_station::is_ds_attached(),
        driver_station::alliance(),
        driver_station::match_time(),
    );
    let stick = GenericHid::new(0);
    println!(
        "stick 0: connected={} axes={} buttons={} povs={}",
        stick.is_connected(),
        stick.axis_count(),
        stick.button_count(),
        stick.pov_count(),
    );
    println!("deploy dir: {}", filesystem::deploy_directory().display());

    let ticks = Arc::new(AtomicU32::new(0));
    let counter = Arc::clone(&ticks);
    let notifier = Notifier::new(move || {
        counter.fetch_add(1, Ordering::Relaxed);
    })
    .expect("notifier");
    let begin = timer::fpga_timestamp();
    notifier.start_periodic(0.01);
    std::thread::sleep(Duration::from_millis(105));
    notifier.stop();
    let elapsed = timer::fpga_timestamp() - begin;
    let after_stop = ticks.load(Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(50));
    println!(
        "notifier: {after_stop} ticks of 10 ms in {elapsed:.3} s, {} more after stop()",
        ticks.load(Ordering::Relaxed) - after_stop,
    );
    drop(notifier);

    match PowerDistribution::new() {
        Ok(pd) => println!(
            "pdh: module {} type {:?} channels {} voltage {:.2} V total {:.2} A",
            pd.module(),
            pd.module_type(),
            pd.num_channels(),
            pd.voltage(),
            pd.total_current(),
        ),
        Err(e) => println!("pdh unavailable: {e}"),
    }
    match Servo::new(0) {
        Ok(servo) => {
            servo.set_angle(90.0);
            println!(
                "servo 0: angle {:.1} position {:.2}",
                servo.angle(),
                servo.get()
            );
        }
        Err(e) => println!("servo unavailable: {e}"),
    }

    let fast = Rc::new(Cell::new(0_u32));
    let mut timed = TimedRobot::new(CountingRobot::default());
    timed.set_watchdog_timeout(0.2);
    let fast_counter = Rc::clone(&fast);
    timed.add_periodic(move || fast_counter.set(fast_counter.get() + 1), 0.01, 0.0);
    let stopper = timed.stopper();
    let mut run_time = Timer::new();
    run_time.start();
    timed.add_post_periodic("stop after 1 s", move || {
        if run_time.has_elapsed(1.0) {
            stopper.stop();
        }
    });
    let wall = timer::fpga_timestamp();
    timed.run().expect("run robot loop");
    let r = timed.robot();
    println!(
        "timed loop ran {:.3} s: robot_init={} disabled_init={} robot_periodic={} disabled_periodic={} \
         simulation_periodic={} 100 Hz callback={}",
        timer::fpga_timestamp() - wall,
        r.robot_init,
        r.disabled_init,
        r.robot_periodic,
        r.disabled_periodic,
        r.simulation_periodic,
        fast.get(),
    );
    hal::shutdown();
}
