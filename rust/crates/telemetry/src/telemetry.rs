//! Spectrum's `Telemetry.java`: the DogLog wrapper the robot calls.
//!
//! Three tiers:
//! - [`log`] writes to the wpilog only. A record is skipped when the value has not changed,
//!   so a boolean or a state string costs nothing while it is steady; a double that moves
//!   every loop costs a record every loop.
//! - [`log_dash`] also publishes the value to NetworkTables at 10 Hz, for the Driver Station
//!   dashboard and the robot app. Use it for exactly the keys a dashboard shows. The whole
//!   log is not mirrored to NetworkTables unless the mirror switch is on (see [`start`]).
//! - [`slow_log_this_loop`] is true every fifth loop. Wrap logs that do not need loop-rate
//!   resolution (currents, temperatures, vision status) in it.
//!
//! Java had these as statics inherited from DogLog. They are free functions here, callable
//! from any thread. Where Java overloads `log` on the value type, `log` takes anything that
//! implements [`LogValue`]; units go through the `_with_unit` variants.
//!
//! Left to the robot crate, which owns the command scheduler: registering [`Telemetry`] as a
//! subsystem so `periodic` runs every loop, `SmartDashboard.putData(CommandScheduler)` in
//! `start`, and `Telemetry.log(Command)`. Use [`log_command_init`] and [`log_command_end`]
//! inside that wrapper.

use crate::dog_log::{DogLog, DogLogOptions};
use crate::value::{LogValue, Value};
use std::any::Any;
use std::cell::RefCell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, PoisonError};
use wpilib::dashboard::smart_dashboard;
use wpilib::nt::{self, Instance, Subscriber};
use wpilib::power_distribution::PowerDistribution;
use wpilib::{driver_station, robot_controller, timer};

pub use crate::dog_log::DogLogOptions as Options;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    CameraOffline,
    AutoShotTimeoutTriggered,
    Brownout,
}

/// Priority levels for printing to the console.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrintPriority {
    /// Prints only when the global priority is also `Normal`.
    Normal,
    /// Always prints, whatever the global priority is.
    High,
}

/// Java enum constants print as their names. Robot enums implement this to give the same
/// strings, because dashboards and log tools read them.
pub trait EnumName {
    fn name(&self) -> &'static str;
}

/// SmartDashboard key of the switch that mirrors every log entry to NetworkTables, for live
/// AdvantageScope sessions in the shop. Off by default; always off when the FMS is attached.
pub const NT_MIRROR_SWITCH_KEY: &str = "Telemetry/MirrorLogsToNT";

/// Loops between slow-tier publishes: every fifth 20 ms loop is 10 Hz.
pub const SLOW_LOG_EVERY_LOOPS: u64 = 5;

/// Entries the log queue may hold before it starts dropping them.
///
/// The queue absorbs the gap between a robot thread that produces records in bursts and a
/// log thread that has to get scheduled to drain them. DogLog's default of 1000 was about a
/// third of a second at the 3000 records per second these logs were running at; 5000 covers
/// a stall of about one and a half seconds. Headroom, not a fix: cutting what is logged is
/// the real lever.
const LOG_ENTRY_QUEUE_CAPACITY: usize = 5000;

/// Global console filter: at `High` only `High` messages print.
static PRIORITY: AtomicU8 = AtomicU8::new(PrintPriority::High as u8);

static NT_MIRROR_STARTED: AtomicBool = AtomicBool::new(false);
static NT_MIRROR_DEFAULT: AtomicBool = AtomicBool::new(false);
/// The switch position, read once per loop on the loop thread and consumed on the log thread.
static NT_MIRROR_ENABLED: AtomicBool = AtomicBool::new(false);

/// Set by the robot loop at the top of every loop.
static LOOP_COUNT: AtomicU64 = AtomicU64::new(0);

/// Called by the robot's `RobotLoop::next` with the new loop number. Java read it from
/// `RobotLoop` directly; that class lives in the robot crate, which depends on this one.
pub fn set_loop_count(count: u64) {
    LOOP_COUNT.store(count, Ordering::Relaxed);
}

/// True on the loops the slow telemetry tier publishes on. Every caller in the same loop
/// agrees, so related values land on the same records.
pub fn slow_log_this_loop() -> bool {
    let n = SLOW_LOG_EVERY_LOOPS;
    n <= 1 || LOOP_COUNT.load(Ordering::Relaxed).is_multiple_of(n)
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Starts the telemetry system.
///
/// Values a dashboard needs are published to NetworkTables individually through [`log_dash`];
/// everything else stays in the wpilog. Mirroring every entry is opt-in through the
/// [`NT_MIRROR_SWITCH_KEY`] switch, and is forced off whenever the FMS is attached
/// regardless of the switch.
///
/// `nt_mirror_default` is the initial position of the mirror-everything switch: false for
/// the robot; flip it in Elastic when AdvantageScope needs the full live stream.
pub fn start(
    nt_mirror_default: bool,
    capture_ds: bool,
    capture_nt: bool,
    capture_console: bool,
    log_extras: bool,
    tunable_on_fms: bool,
    priority: PrintPriority,
) {
    PRIORITY.store(priority as u8, Ordering::Relaxed);

    NT_MIRROR_DEFAULT.store(nt_mirror_default, Ordering::Relaxed);
    NT_MIRROR_ENABLED.store(nt_mirror_default, Ordering::Relaxed);
    smart_dashboard::put_boolean(NT_MIRROR_SWITCH_KEY, nt_mirror_default);
    NT_MIRROR_STARTED.store(true, Ordering::Relaxed);

    set_options(
        DogLogOptions::default()
            .with_nt_publish_fn(mirror_logs_to_nt)
            .with_capture_ds(capture_ds)
            .with_capture_nt(capture_nt)
            .with_capture_console(capture_console)
            .with_nt_tunables(tunable_on_fms)
            .with_log_entry_queue_capacity(LOG_ENTRY_QUEUE_CAPACITY)
            .with_log_extras(log_extras),
    );
    match PowerDistribution::new() {
        Ok(pdh) => set_pdh(Some(pdh)),
        Err(e) => e.report(),
    }
}

pub fn set_options(options: DogLogOptions) {
    DogLog::global().set_options(options);
}

pub fn set_pdh(pdh: Option<PowerDistribution>) {
    DogLog::global().set_pdh(pdh);
}

/// Whether DogLog should mirror log entries to NetworkTables right now. Evaluated by the log
/// thread for every record, so it reads two cached booleans and nothing else.
fn mirror_logs_to_nt() -> bool {
    NT_MIRROR_ENABLED.load(Ordering::Relaxed) && !driver_station::is_fms_attached()
}

/// The single point every record leaves through, so tests can capture them.
fn emit(key: &str, force_nt: bool, value: Value, unit: Option<&str>) {
    #[cfg(test)]
    if testing::record(key, force_nt, &value, unit) {
        return;
    }
    DogLog::global().log_value(key, force_nt, value, unit);
}

/// Logs a value to the wpilog.
pub fn log(key: &str, value: impl LogValue) {
    emit(key, false, value.into_value(), None);
}

/// Logs a number or number array with its unit in the entry's metadata.
pub fn log_with_unit(key: &str, value: impl LogValue, unit: &str) {
    emit(key, false, value.into_value(), Some(unit));
}

pub fn log_struct<S: nt::Struct>(key: &str, value: &S) {
    emit(key, false, crate::value::struct_value(value), None);
}

pub fn log_struct_array<S: nt::Struct>(key: &str, values: &[S]) {
    emit(key, false, crate::value::struct_array_value(values), None);
}

/// Logs an enum as its name, which is what DogLog does for an enum with no fields.
pub fn log_enum(key: &str, value: &impl EnumName) {
    emit(key, false, Value::String(value.name().to_owned()), None);
}

//
// log_dash writes the record on every call and publishes it to NetworkTables on slow-tier
// loops, so a value that moves every loop reaches the dashboard at 10 Hz and the log at loop
// rate. The NetworkTables topic only changes when a slow-tier call publishes, so a boolean or
// a string that flips between ticks still arrives within 100 ms. log_dash_always publishes
// on every call, for values logged on their own slower cadence that might never land on a
// tick.

pub fn log_dash(key: &str, value: impl LogValue) {
    emit(key, slow_log_this_loop(), value.into_value(), None);
}

pub fn log_dash_with_unit(key: &str, value: impl LogValue, unit: &str) {
    emit(key, slow_log_this_loop(), value.into_value(), Some(unit));
}

/// Logs a value and publishes it to the dashboard on this very call. For values logged on
/// their own cadence (a timer, a state change) that must not wait for a slow-tier loop.
pub fn log_dash_always(key: &str, value: impl LogValue) {
    emit(key, true, value.into_value(), None);
}

pub fn log_dash_always_with_unit(key: &str, value: impl LogValue, unit: &str) {
    emit(key, true, value.into_value(), Some(unit));
}

//
// A state is logged on the loop it changes and not while it holds. The wpilog ends up the
// same as logging every loop, because DogLog drops a value equal to the last one, but
// every-loop calls still cost a queue entry each for the log thread to throw away.

/// Last state logged per key, so `log_state` can skip unchanged ones.
static LAST_LOGGED_STATES: Mutex<Option<HashMap<String, Box<dyn Any + Send>>>> = Mutex::new(None);

/// Records `state` for `key`. True when it differs from the last one.
fn state_changed<E: Copy + PartialEq + Send + 'static>(key: &str, state: E) -> bool {
    let mut guard = lock(&LAST_LOGGED_STATES);
    let map = guard.get_or_insert_with(HashMap::new);
    match map.get_mut(key) {
        Some(last) if last.downcast_ref::<E>() == Some(&state) => false,
        Some(last) => {
            *last = Box::new(state);
            true
        }
        None => {
            map.insert(key.to_owned(), Box::new(state));
            true
        }
    }
}

/// Logs a state machine's state when it changes: the exact loop of every transition,
/// nothing while it holds.
pub fn log_state<E: EnumName + Copy + PartialEq + Send + 'static>(key: &str, state: E) {
    if state_changed(key, state) {
        log(key, state.name());
    }
}

/// [`log_state`], also published to the dashboard on every change (a change can land between
/// slow-tier ticks, so [`log_dash`] could miss it).
pub fn log_state_dash<E: EnumName + Copy + PartialEq + Send + 'static>(key: &str, state: E) {
    if state_changed(key, state) {
        log_dash_always(key, state.name());
    }
}

/// Start times of open [`time`] spans, in FPGA microseconds.
static EPOCH_START_MICROS: Mutex<Option<HashMap<String, u64>>> = Mutex::new(None);

/// Starts a timed span. Pair with [`time_end`] on the same key.
///
/// Replaces DogLog's timer so the result reaches the dashboard: the `Scheduler/*` spans are
/// on the Elastic layout and are the primary record of loop time.
pub fn time(key: &str) {
    time_at(key, robot_controller::fpga_time());
}

fn time_at(key: &str, now_micros: u64) {
    lock(&EPOCH_START_MICROS)
        .get_or_insert_with(HashMap::new)
        .insert(key.to_owned(), now_micros);
}

/// Ends a timed span and logs its length in seconds, matching the units DogLog's timer used,
/// so existing analysis of the `Scheduler/*` keys keeps working.
pub fn time_end(key: &str) {
    time_end_at(key, robot_controller::fpga_time());
}

fn time_end_at(key: &str, now_micros: u64) {
    let start = lock(&EPOCH_START_MICROS)
        .get_or_insert_with(HashMap::new)
        .remove(key);
    let Some(start) = start else {
        return;
    };
    log_dash_with_unit(
        key,
        now_micros.saturating_sub(start) as f64 / 1_000_000.0,
        "seconds",
    );
}

/// What `Telemetry.log(Command)` logs when a command starts.
pub fn log_command_init(name: &str) {
    log("Commands", format!("Init: {name}"));
}

/// What `Telemetry.log(Command)` logs when a command ends.
pub fn log_command_end(name: &str) {
    log("Commands", format!("End: {name}"));
}

fn print_text(fpga_seconds: f64, output: &str) -> String {
    format!("TIME: {fpga_seconds:.3} || {output}")
}

/// Echoes to the console when the priority allows, and always logs to the "Prints" key.
pub fn print_with(output: &str, priority: PrintPriority) {
    let out = print_text(timer::fpga_timestamp(), output);
    if priority == PrintPriority::High
        || PRIORITY.load(Ordering::Relaxed) == PrintPriority::Normal as u8
    {
        println!("{out}");
    }
    log("Prints", out);
}

/// Prints a message at [`PrintPriority::Normal`] priority. The message is always written to
/// the DogLog "Prints" key but only echoed to stdout when the global priority allows it.
pub fn print(output: &str) {
    print_with(output, PrintPriority::Normal);
}

/// Alerts in `output` that are not in `previous`, as `"<prefix>: <alert>"` log lines.
fn new_alert_lines(previous: &[String], current: &[String], prefix: &str) -> Vec<String> {
    current
        .iter()
        .filter(|a| !previous.contains(a))
        .map(|a| format!("{prefix}: {a}"))
        .collect()
}

const ALERT_TYPES: [(&str, &str); 3] = [
    ("errors", "ERROR"),
    ("warnings", "WARNING"),
    ("infos", "INFO"),
];

#[derive(Default)]
struct AlertState {
    /// Live subscribers for each alert severity, created on first use.
    subscribers: HashMap<&'static str, Subscriber<nt::StringArray>>,
    /// Most recent set of active alerts per severity key, to avoid duplicate log entries.
    previous: HashMap<&'static str, Vec<String>>,
}

/// Java's `Telemetry` as a subsystem: `periodic` runs once per loop on the loop thread.
#[derive(Default)]
pub struct Telemetry {
    alerts: RefCell<AlertState>,
}

impl Telemetry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn periodic(&self) {
        refresh_nt_mirror_switch();
        self.log_alerts();
    }

    /// Logs any alert newly published under `SmartDashboard/Alerts` (errors, warnings,
    /// infos) to the "Alerts" key.
    ///
    /// Reads the subscribers' change queues rather than the current arrays, so a loop on
    /// which nothing changed reads nothing.
    pub fn log_alerts(&self) {
        for (key, prefix) in ALERT_TYPES {
            self.log_alert_type(key, prefix);
        }
    }

    fn log_alert_type(&self, key: &'static str, prefix: &str) {
        let changes = {
            let mut state = self.alerts.borrow_mut();
            let subscriber = state.subscribers.entry(key).or_insert_with(|| {
                Instance::default_instance()
                    .table("SmartDashboard/Alerts")
                    .topic::<nt::StringArray>(key)
                    .subscribe(Vec::new(), &nt::Options::default())
            });
            subscriber.read_queue_values()
        };
        for current in changes {
            let lines = {
                let state = self.alerts.borrow();
                let previous = state.previous.get(key).map_or(&[][..], Vec::as_slice);
                new_alert_lines(previous, &current, prefix)
            };
            for line in lines {
                log("Alerts", line);
            }
            self.alerts.borrow_mut().previous.insert(key, current);
        }
    }
}

fn refresh_nt_mirror_switch() {
    if NT_MIRROR_STARTED.load(Ordering::Relaxed) {
        let default = NT_MIRROR_DEFAULT.load(Ordering::Relaxed);
        NT_MIRROR_ENABLED.store(
            smart_dashboard::get_boolean(NT_MIRROR_SWITCH_KEY, default),
            Ordering::Relaxed,
        );
    }
}

/// Records what the free functions log, per test thread, instead of sending it to DogLog.
#[cfg(test)]
pub(crate) mod testing {
    use super::*;

    #[derive(Clone)]
    pub struct Logged {
        pub key: String,
        pub force_nt: bool,
        pub value: Value,
        pub unit: Option<String>,
    }

    /// Tests that set the loop count or read captures hold this, as the count is global.
    pub fn serial() -> MutexGuard<'static, ()> {
        static SERIAL: Mutex<()> = Mutex::new(());
        SERIAL.lock().unwrap_or_else(PoisonError::into_inner)
    }

    thread_local! {
        static CAPTURE: RefCell<Option<Vec<Logged>>> = const { RefCell::new(None) };
    }

    pub fn record(key: &str, force_nt: bool, value: &Value, unit: Option<&str>) -> bool {
        CAPTURE.with(|c| {
            let mut c = c.borrow_mut();
            let Some(out) = c.as_mut() else {
                return false;
            };
            out.push(Logged {
                key: key.to_owned(),
                force_nt,
                value: value.clone(),
                unit: unit.map(str::to_owned),
            });
            true
        })
    }

    /// Runs `f` and returns what it logged.
    pub fn capture(f: impl FnOnce()) -> Vec<Logged> {
        CAPTURE.with(|c| *c.borrow_mut() = Some(Vec::new()));
        f();
        CAPTURE.with(|c| c.borrow_mut().take().unwrap_or_default())
    }

    impl Logged {
        pub fn double(&self) -> f64 {
            match self.value {
                Value::Double(v) => v,
                _ => panic!("{} is not a double", self.key),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::testing::*;
    use super::*;

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum State {
        Idle,
        Shooting,
    }

    impl EnumName for State {
        fn name(&self) -> &'static str {
            match self {
                Self::Idle => "IDLE",
                Self::Shooting => "SHOOTING",
            }
        }
    }

    /// The loop count and the keys these tests use are shared, so each test uses its own keys
    /// and sets the count itself; threads keep the captures apart.
    fn on_loop(count: u64) {
        set_loop_count(count);
    }

    #[test]
    fn slow_tier_is_every_fifth_loop() {
        let _g = serial();
        let ticks: Vec<bool> = (0..11)
            .map(|n| {
                on_loop(n);
                slow_log_this_loop()
            })
            .collect();
        assert_eq!(
            ticks,
            [
                true, false, false, false, false, true, false, false, false, false, true
            ]
        );
    }

    #[test]
    fn log_dash_publishes_on_slow_loops_only() {
        let _g = serial();
        on_loop(5);
        let on = capture(|| log_dash("a", 1.0));
        assert!(on[0].force_nt);
        on_loop(6);
        let off = capture(|| {
            log_dash("a", 1.0);
            log_dash_with_unit("b", 2.0, "volts");
        });
        assert!(!off[0].force_nt && !off[1].force_nt);
        assert_eq!(off[1].unit.as_deref(), Some("volts"));
    }

    #[test]
    fn log_dash_always_always_publishes() {
        let _g = serial();
        on_loop(6);
        let out = capture(|| {
            log_dash_always("a", true);
            log_dash_always_with_unit("b", 2.0, "ms");
        });
        assert!(out.iter().all(|l| l.force_nt));
        assert_eq!(out[1].unit.as_deref(), Some("ms"));
    }

    #[test]
    fn plain_log_never_publishes() {
        let _g = serial();
        on_loop(0);
        let out = capture(|| {
            log("a", "text");
            log_with_unit("b", 1.0, "m");
            log_enum("c", &State::Idle);
        });
        assert!(out.iter().all(|l| !l.force_nt));
        assert!(matches!(&out[2].value, Value::String(s) if s == "IDLE"));
    }

    #[test]
    fn states_log_only_when_they_change() {
        let _g = serial();
        on_loop(6);
        let out = capture(|| {
            log_state("state_test/a", State::Idle);
            log_state("state_test/a", State::Idle);
            log_state("state_test/a", State::Shooting);
            log_state("state_test/a", State::Shooting);
            log_state("state_test/a", State::Idle);
            log_state("state_test/b", State::Idle);
        });
        let names: Vec<_> = out
            .iter()
            .map(|l| match &l.value {
                Value::String(s) => format!("{}={s}", l.key),
                _ => unreachable!(),
            })
            .collect();
        assert_eq!(
            names,
            [
                "state_test/a=IDLE",
                "state_test/a=SHOOTING",
                "state_test/a=IDLE",
                "state_test/b=IDLE"
            ]
        );
        assert!(out.iter().all(|l| !l.force_nt));
    }

    #[test]
    fn state_dash_publishes_each_change() {
        let _g = serial();
        on_loop(6);
        let out = capture(|| {
            log_state_dash("state_test/dash", State::Idle);
            log_state_dash("state_test/dash", State::Idle);
            log_state_dash("state_test/dash", State::Shooting);
        });
        assert_eq!(out.len(), 2);
        assert!(out.iter().all(|l| l.force_nt));
    }

    #[test]
    fn timed_spans_log_seconds() {
        let _g = serial();
        on_loop(5);
        let out = capture(|| {
            time_at("time_test/loop", 1_000_000);
            time_end_at("time_test/loop", 1_025_000);
            // A second end finds no start.
            time_end_at("time_test/loop", 2_000_000);
            time_end_at("time_test/never", 2_000_000);
        });
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].double(), 0.025);
        assert_eq!(out[0].unit.as_deref(), Some("seconds"));
        assert!(
            out[0].force_nt,
            "span lengths reach the dashboard on slow loops"
        );
    }

    #[test]
    fn command_messages() {
        let _g = serial();
        let out = capture(|| {
            log_command_init("Shoot");
            log_command_end("Shoot");
        });
        assert_eq!(out[0].key, "Commands");
        assert!(matches!(&out[0].value, Value::String(s) if s == "Init: Shoot"));
        assert!(matches!(&out[1].value, Value::String(s) if s == "End: Shoot"));
    }

    #[test]
    fn print_text_has_three_decimals() {
        assert_eq!(print_text(12.3456, "hello"), "TIME: 12.346 || hello");
        assert_eq!(print_text(0.0, ""), "TIME: 0.000 || ");
    }

    #[test]
    fn only_new_alerts_are_logged() {
        let s = |v: &[&str]| v.iter().map(|x| (*x).to_owned()).collect::<Vec<_>>();
        assert_eq!(
            new_alert_lines(&s(&["a"]), &s(&["a", "b"]), "ERROR"),
            ["ERROR: b"]
        );
        assert!(new_alert_lines(&s(&["a", "b"]), &s(&["b"]), "INFO").is_empty());
        assert_eq!(new_alert_lines(&[], &s(&["x"]), "WARNING"), ["WARNING: x"]);
    }

    #[test]
    fn mirror_follows_the_switch_and_the_fms() {
        NT_MIRROR_ENABLED.store(false, Ordering::Relaxed);
        assert!(!mirror_logs_to_nt());
        NT_MIRROR_ENABLED.store(true, Ordering::Relaxed);
        assert!(mirror_logs_to_nt(), "no FMS in a unit test");
        NT_MIRROR_ENABLED.store(false, Ordering::Relaxed);
    }

    #[test]
    fn priority_default_is_high() {
        assert_eq!(PRIORITY.load(Ordering::Relaxed), PrintPriority::High as u8);
        assert_ne!(Fault::CameraOffline, Fault::Brownout);
    }
}
