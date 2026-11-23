//! Starts the logger in simulation with NetworkTables publishing on, logs for two seconds from
//! a 20 ms loop and from a second thread, then reads the log file back and prints entry counts.
//! A client instance in the same process checks that values reached NetworkTables.
//!
//! Run from `rust/` with the natives on the library path:
//! `cargo xtask sim telemetry_smoke -p telemetry`

use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::thread::{self, sleep};
use std::time::{Duration, Instant};
use telemetry::reader::Log;
use telemetry::telemetry::{self as tel, EnumName, PrintPriority};
use telemetry::{BatteryLogger, DogLog, SystemLoadMonitor, Telemetry, TuneValue};
use wpilib::dashboard::alert::{Alert, AlertType};
use wpilib::dashboard::smart_dashboard;
use wpilib::hal;
use wpilib::nt::{Double, Instance, Options, StructKind, Text};
use wpimath::geometry::{Pose2d, Rotation2d};

const PORT: u32 = 5898;

#[derive(Clone, Copy, PartialEq)]
enum Phase {
    Warmup,
    Run,
}

impl EnumName for Phase {
    fn name(&self) -> &'static str {
        match self {
            Self::Warmup => "WARMUP",
            Self::Run => "RUN",
        }
    }
}

fn main() {
    hal::initialize();
    println!("runtime: {:?}", hal::runtime_type());

    // A server for the logger to publish to and a client to look from.
    let server = Instance::default_instance();
    server.start_server("", "127.0.0.1", 0, PORT);
    let client = Instance::create();
    client.start_client4("telemetry_smoke");
    client.set_server("127.0.0.1", PORT);
    let robot = client.table("/Robot");
    let loop_value = robot
        .topic::<Double>("Smoke/loop")
        .subscribe(-1.0, &Options::default());
    let pose = robot.topic::<StructKind<Pose2d>>("Smoke/pose").subscribe(
        Pose2d::from_xy(0.0, 0.0, Rotation2d::new(0.0)),
        &Options::default(),
    );
    let phase = robot
        .topic::<Text>("Smoke/phase")
        .subscribe(String::new(), &Options::default());

    // Mirror every record to NetworkTables, capture the DS, no extras (as the robot).
    tel::start(true, true, false, true, false, true, PrintPriority::Normal);

    // A NetworkTables tunable and a SmartDashboard tune value, both changed by the client.
    let tunable = DogLog::global().tunable("Smoke/tune", 1.5, Some("volts"));
    assert_eq!(tunable.get(), 1.5);
    let tune_value = Rc::new(TuneValue::new("Smoke/tune2", 1.0));
    let tune_supplier = tune_value.supplier();
    let client_tune = client
        .table("/Tunable")
        .topic::<Double>("Smoke/tune")
        .publish(&Options::default());
    client_tune.set(2.5);
    let client_tune2 = client
        .table("/SmartDashboard")
        .topic::<Double>("Smoke/tune2")
        .publish(&Options::default());
    client_tune2.set(4.0);

    // The loop-thread helpers.
    let subsystem = Telemetry::new();
    let monitor = SystemLoadMonitor::new();
    let battery = BatteryLogger::new();
    battery.set_enabled(true);
    let alert = Alert::new("smoke alert", AlertType::Warning);

    let stop = Arc::new(AtomicBool::new(false));
    let thread_logs = Arc::new(AtomicU32::new(0));
    let worker = {
        let (stop, count) = (stop.clone(), thread_logs.clone());
        thread::spawn(move || {
            let mut i = 0.0;
            while !stop.load(Ordering::Relaxed) {
                tel::log_with_unit("Smoke/thread", i, "counts");
                count.fetch_add(1, Ordering::Relaxed);
                i += 1.0;
                sleep(Duration::from_millis(2));
            }
        })
    };

    let start = Instant::now();
    let loop_ticks = 100;
    for i in 0..loop_ticks {
        tel::set_loop_count(i);
        monitor.periodic();
        subsystem.periodic();
        tel::time("Smoke/loopTime");
        battery.report_current_usage("Smoke/Mech", &[3.0, -1.0]);
        battery.log_power();
        if i == 10 {
            alert.set(true);
        }
        smart_dashboard::update_values();
        tel::log_dash_with_unit("Smoke/loop", i as f64, "ticks");
        tel::log_struct(
            "Smoke/pose",
            &Pose2d::from_xy(i as f64 * 0.1, 1.0, Rotation2d::new(0.01 * i as f64)),
        );
        tel::log_state_dash(
            "Smoke/phase",
            if i < 50 { Phase::Warmup } else { Phase::Run },
        );
        tel::log("Smoke/bool", i % 2 == 0);
        tel::log("Smoke/array", &[i as f64, 2.0, 3.0]);
        if i % 25 == 0 {
            tel::print(&format!("tick {i}"));
        }
        tel::time_end("Smoke/loopTime");
        sleep(Duration::from_millis(20));
    }
    let elapsed = start.elapsed();
    println!(
        "tunable = {}, TuneValue = {}",
        tunable.get(),
        tune_supplier()
    );
    assert_eq!(tunable.get(), 2.5, "tunable follows NetworkTables");
    assert_eq!(tune_supplier(), 4.0, "TuneValue follows SmartDashboard");
    stop.store(true, Ordering::Relaxed);
    worker.join().unwrap();

    let dog = DogLog::global();
    dog.flush();
    sleep(Duration::from_millis(400));
    let path = dog
        .data_log()
        .and_then(|l| l.path())
        .expect("log file path");
    println!(
        "ran {:.2} s, second thread logged {} times",
        elapsed.as_secs_f64(),
        thread_logs.load(Ordering::Relaxed)
    );
    println!("log file: {}", path.display());

    // NetworkTables: what a client sees.
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline && phase.get().is_empty() {
        sleep(Duration::from_millis(20));
    }
    println!(
        "NT client sees Smoke/loop = {}, Smoke/pose = ({:.2}, {:.2}), Smoke/phase = {:?}",
        loop_value.get(),
        pose.get().translation.x,
        pose.get().translation.y,
        phase.get()
    );

    // The file.
    let bytes = std::fs::read(&path).expect("read log");
    let (log, error) = Log::parse_lossy(&bytes).expect("parse log");
    println!(
        "file: {} bytes, {} entries, read error: {error:?}",
        bytes.len(),
        log.entries.len()
    );
    for name in [
        "/Robot/Smoke/loop",
        "/Robot/Smoke/thread",
        "/Robot/Smoke/pose",
        "/Robot/Smoke/phase",
        "/Robot/Smoke/bool",
        "/Robot/Smoke/array",
        "/Robot/Smoke/loopTime",
        "/Robot/Prints",
        "/Robot/Alerts",
        "/Robot/System/Loop/MeanPeriodMs",
        "/Robot/BatteryLogger/Current",
        "/Robot/BatteryLogger/Energy",
        "/Robot/BatteryLogger/Power/Smoke/Mech",
        "/Robot/DogLog/QueuedLogs",
        "/Robot/DogLog/Options",
        "DS:enabled",
        "DS:joystick0/axes",
        "systemTime",
    ] {
        match log.entry(name) {
            Some(e) => println!(
                "  {name}: {} records, type {}",
                e.records.len(),
                e.type_name
            ),
            None => println!("  {name}: missing"),
        }
    }
    let total: usize = log.entries.iter().map(|e| e.records.len()).sum();
    println!("total data records: {total}");

    let loops = log
        .entry("/Robot/Smoke/loop")
        .map_or(0, |e| e.records.len());
    let thread = log
        .entry("/Robot/Smoke/thread")
        .map_or(0, |e| e.records.len());
    assert_eq!(loops, loop_ticks as usize, "every loop record arrived");
    assert_eq!(
        thread,
        thread_logs.load(Ordering::Relaxed) as usize,
        "every record of the second thread arrived"
    );

    // Leave nothing behind.
    let _ = std::fs::remove_file(&path);
    if let Some(dir) = path.parent() {
        let _ = std::fs::remove_dir(dir);
    }
    println!("ok");
}
