//! Watches the things that made the robot fall behind on 2026-09-05 and says so on the
//! dashboard.
//!
//! That day the roboRIO CPU sat at 92 to 95 percent and the loop ran at 30 ms, and nothing on
//! the Driver Station said so: the overrun watchdog is set to 200 ms and the CPU figure is
//! buried in the DS log viewer. This publishes the numbers once a second and raises an
//! [`Alert`] when one of them has been bad for long enough to matter:
//!
//! - CPU from `/proc/stat`, alert after [`CPU_HOLD_SECONDS`] s at or above
//!   [`CPU_ALERT_PERCENT`] percent.
//! - Loop overruns: the share of loops longer than [`LOOP_OVERRUN_MS`] ms, alert after
//!   [`LOOP_OVERRUN_HOLD_SECONDS`] s above [`LOOP_OVERRUN_ALERT_PERCENT`] percent.
//! - Loop stall: a single loop over [`LOOP_STALL_MS`] ms while enabled, latched for
//!   [`LATCH_SECONDS`] s so it is seen. Stalls while disabled are PathPlanner warmup and
//!   are ignored.
//! - Memory: `MemAvailable` from `/proc/meminfo`, alert after [`MEMORY_HOLD_SECONDS`] s under
//!   [`MEMORY_ALERT_MB`] MB.
//!
//! Java also watched the JVM garbage collector and heap (`System/Gc/*`, `System/HeapUsedMB`,
//! the GC alert). Rust has neither, so those channels are gone.
//!
//! Call [`SystemLoadMonitor::periodic`] once per loop, first thing in the robot's periodic.
//! Per loop it does a subtraction; once a second it reads two small files under `/proc`. Off
//! the rio (no `/proc`) the CPU and memory channels go quiet and the rest keeps working.

use crate::telemetry::log_dash_always_with_unit;
use std::cell::{Cell, RefCell};
use std::fs;
use std::path::PathBuf;
use wpilib::dashboard::alert::{Alert, AlertType};
use wpilib::{driver_station, timer};

/// Seconds between samples and dashboard publishes.
pub const SAMPLE_PERIOD_SECONDS: f64 = 1.0;

/// A loop longer than this counts as an overrun. The budget is 20 ms.
pub const LOOP_OVERRUN_MS: f64 = 25.0;

/// Share of overrunning loops that starts the loop alert clock.
pub const LOOP_OVERRUN_ALERT_PERCENT: f64 = 50.0;

/// Share of overrunning loops below which the loop alert clears.
pub const LOOP_OVERRUN_CLEAR_PERCENT: f64 = 25.0;

/// Seconds the overrun share must stay high before the loop alert shows.
pub const LOOP_OVERRUN_HOLD_SECONDS: f64 = 5.0;

/// A single enabled loop longer than this is a stall worth an error.
pub const LOOP_STALL_MS: f64 = 200.0;

/// CPU percent that starts the CPU alert clock. 2026-09-05 ran at 92 to 95; a loop with a
/// real margin needs the two cores under about 80.
pub const CPU_ALERT_PERCENT: f64 = 85.0;

/// CPU percent below which the CPU alert clears.
pub const CPU_CLEAR_PERCENT: f64 = 80.0;

/// Seconds the CPU must stay high before the alert shows.
pub const CPU_HOLD_SECONDS: f64 = 10.0;

/// Available memory below which the memory alert clock starts.
pub const MEMORY_ALERT_MB: f64 = 24.0;

/// Seconds memory must stay low before the alert shows.
pub const MEMORY_HOLD_SECONDS: f64 = 10.0;

/// How long a one-shot alert (stall) stays up so someone sees it.
pub const LATCH_SECONDS: f64 = 10.0;

/// Samples between refreshes of a sustained alert's text, to keep the Alerts log quiet.
const TEXT_REFRESH_SAMPLES: u64 = 5;

/// What the monitor needs from an alert. [`Alert`] is the real one.
pub trait AlertLike {
    fn get(&self) -> bool;
    fn set(&self, active: bool);
    fn set_text(&self, text: &str);
}

impl AlertLike for Alert {
    fn get(&self) -> bool {
        Alert::get(self)
    }

    fn set(&self, active: bool) {
        Alert::set(self, active);
    }

    fn set_text(&self, text: &str) {
        Alert::set_text(self, text);
    }
}

/// An alert raised once a condition has held for a while, with its text refreshed while it
/// stays up, and cleared once the condition has recovered.
struct SustainedAlert<A> {
    alert: A,
    hold_seconds: f64,
    since_seconds: Cell<Option<f64>>,
}

impl<A: AlertLike> SustainedAlert<A> {
    fn new(alert: A, hold_seconds: f64) -> Self {
        Self {
            alert,
            hold_seconds,
            since_seconds: Cell::new(None),
        }
    }

    /// Updates the alert from this sample.
    ///
    /// `bad` is whether the condition is in its alert band. `recovered` is whether it has
    /// recovered enough to clear; with hysteresis this is a stricter test than `!bad`, and
    /// an in-between sample changes nothing. `refresh_text` is whether a raised alert's text
    /// should be rewritten this sample. `text` builds the alert text from how long the
    /// condition has held, in seconds.
    fn update(
        &self,
        now: f64,
        bad: bool,
        recovered: bool,
        refresh_text: bool,
        text: impl FnOnce(f64) -> String,
    ) {
        if bad {
            let since = self.since_seconds.get().unwrap_or(now);
            self.since_seconds.set(Some(since));
            let held = now - since;
            if held >= self.hold_seconds && (!self.alert.get() || refresh_text) {
                self.alert.set_text(&text(held));
                self.alert.set(true);
            }
        } else if recovered {
            self.since_seconds.set(None);
            self.alert.set(false);
        }
    }
}

/// Parses the first line of `/proc/stat`: `cpu user nice system idle iowait irq softirq
/// steal ...`. Returns busy plus idle jiffies and idle jiffies.
fn parse_cpu_line(line: &str) -> Option<(i64, i64)> {
    if !line.starts_with("cpu ") {
        return None;
    }
    let mut total = 0;
    let mut idle = 0;
    for (i, field) in line.split_whitespace().enumerate().take(9).skip(1) {
        let value: i64 = field.parse().ok()?;
        total += value;
        if i == 4 || i == 5 {
            idle += value;
        }
    }
    Some((total, idle))
}

/// `MemAvailable` in megabytes from the first ten lines of `/proc/meminfo`. MemAvailable
/// counts reclaimable page cache, unlike the MemFree the Driver Station shows, which is why
/// the DS read 4 to 5 MB free all day on 2026-09-05 with nothing actually wrong.
fn parse_mem_available_mb(meminfo: &str) -> Option<f64> {
    let line = meminfo
        .lines()
        .take(10)
        .find(|l| l.starts_with("MemAvailable:"))?;
    let kb: f64 = line.split_whitespace().nth(1)?.parse().ok()?;
    Some(kb / 1024.0)
}

struct State {
    // Per-loop bookkeeping
    last_loop_seconds: Option<f64>,
    bucket_start_seconds: Option<f64>,
    bucket_loops: u32,
    bucket_overruns: u32,
    bucket_sum_ms: f64,
    bucket_max_ms: f64,
    sample_count: u64,

    stall_latch_until_seconds: f64,

    // /proc counters
    proc_stat_available: bool,
    proc_meminfo_available: bool,
    last_cpu_total_jiffies: Option<i64>,
    last_cpu_idle_jiffies: i64,
}

pub struct SystemLoadMonitor<A: AlertLike = Alert> {
    cpu_alert: SustainedAlert<A>,
    loop_alert: SustainedAlert<A>,
    stall_alert: A,
    memory_alert: SustainedAlert<A>,
    proc_stat: PathBuf,
    proc_meminfo: PathBuf,
    state: RefCell<State>,
}

impl SystemLoadMonitor<Alert> {
    pub fn new() -> Self {
        Self::with_alerts(Alert::new)
    }
}

impl Default for SystemLoadMonitor<Alert> {
    fn default() -> Self {
        Self::new()
    }
}

impl<A: AlertLike> SystemLoadMonitor<A> {
    /// A monitor whose alerts come from `make`.
    pub fn with_alerts(make: impl Fn(&str, AlertType) -> A) -> Self {
        Self {
            cpu_alert: SustainedAlert::new(
                make("roboRIO CPU high", AlertType::Warning),
                CPU_HOLD_SECONDS,
            ),
            loop_alert: SustainedAlert::new(
                make("Robot loop overrunning", AlertType::Warning),
                LOOP_OVERRUN_HOLD_SECONDS,
            ),
            stall_alert: make("Robot loop stalled while enabled", AlertType::Error),
            memory_alert: SustainedAlert::new(
                make("roboRIO memory low", AlertType::Warning),
                MEMORY_HOLD_SECONDS,
            ),
            proc_stat: PathBuf::from("/proc/stat"),
            proc_meminfo: PathBuf::from("/proc/meminfo"),
            state: RefCell::new(State {
                last_loop_seconds: None,
                bucket_start_seconds: None,
                bucket_loops: 0,
                bucket_overruns: 0,
                bucket_sum_ms: 0.0,
                bucket_max_ms: 0.0,
                sample_count: 0,
                stall_latch_until_seconds: f64::NEG_INFINITY,
                proc_stat_available: true,
                proc_meminfo_available: true,
                last_cpu_total_jiffies: None,
                last_cpu_idle_jiffies: 0,
            }),
        }
    }

    /// Reads CPU and memory from other files than `/proc/stat` and `/proc/meminfo`.
    pub fn with_proc_files(
        mut self,
        stat: impl Into<PathBuf>,
        meminfo: impl Into<PathBuf>,
    ) -> Self {
        self.proc_stat = stat.into();
        self.proc_meminfo = meminfo.into();
        self
    }

    /// Records this loop and, once a second, samples the system and updates the alerts.
    ///
    /// Call once per loop, before anything else in the robot's periodic, so the loop period
    /// it measures is the whole period.
    pub fn periodic(&self) {
        self.periodic_at(timer::fpga_timestamp(), driver_station::is_enabled());
    }

    fn periodic_at(&self, now: f64, enabled: bool) {
        self.record_loop(now, enabled);

        let sample_due = {
            let mut s = self.state.borrow_mut();
            match s.bucket_start_seconds {
                None => {
                    s.bucket_start_seconds = Some(now);
                    false
                }
                Some(start) => now - start >= SAMPLE_PERIOD_SECONDS,
            }
        };
        if sample_due {
            self.sample(now);
            self.state.borrow_mut().bucket_start_seconds = Some(now);
        }
    }

    fn record_loop(&self, now: f64, enabled: bool) {
        let mut s = self.state.borrow_mut();
        let Some(last) = s.last_loop_seconds.replace(now) else {
            return;
        };
        let period_ms = (now - last) * 1000.0;

        s.bucket_loops += 1;
        s.bucket_sum_ms += period_ms;
        s.bucket_max_ms = s.bucket_max_ms.max(period_ms);
        if period_ms > LOOP_OVERRUN_MS {
            s.bucket_overruns += 1;
        }

        if period_ms > LOOP_STALL_MS && enabled {
            self.stall_alert.set_text(&format!(
                "Robot loop stalled {period_ms:.0} ms while enabled"
            ));
            self.stall_alert.set(true);
            s.stall_latch_until_seconds = now + LATCH_SECONDS;
        }
    }

    fn sample(&self, now: f64) {
        let (refresh_text, overrun_percent, mean_ms, max_ms) = {
            let mut s = self.state.borrow_mut();
            s.sample_count += 1;
            let refresh_text = s.sample_count.is_multiple_of(TEXT_REFRESH_SAMPLES);

            // Loop
            let loops = f64::from(s.bucket_loops);
            let overrun_percent = if s.bucket_loops > 0 {
                100.0 * f64::from(s.bucket_overruns) / loops
            } else {
                0.0
            };
            let mean_ms = if s.bucket_loops > 0 {
                s.bucket_sum_ms / loops
            } else {
                0.0
            };
            let max_ms = s.bucket_max_ms;
            s.bucket_loops = 0;
            s.bucket_overruns = 0;
            s.bucket_sum_ms = 0.0;
            s.bucket_max_ms = 0.0;
            (refresh_text, overrun_percent, mean_ms, max_ms)
        };
        log_dash_always_with_unit("System/Loop/MeanPeriodMs", mean_ms, "ms");
        log_dash_always_with_unit("System/Loop/MaxPeriodMs", max_ms, "ms");
        log_dash_always_with_unit("System/Loop/OverrunPercent", overrun_percent, "%");

        self.loop_alert.update(
            now,
            overrun_percent >= LOOP_OVERRUN_ALERT_PERCENT,
            overrun_percent < LOOP_OVERRUN_CLEAR_PERCENT,
            refresh_text,
            |held| {
                format!(
                    "Robot loop overrunning: {overrun_percent:.0}% of loops over \
                     {LOOP_OVERRUN_MS:.0} ms for {held:.0} s (mean {mean_ms:.1} ms)"
                )
            },
        );

        // CPU
        if let Some(cpu_percent) = self.read_cpu_percent() {
            log_dash_always_with_unit("System/CpuPercent", cpu_percent, "%");
            self.cpu_alert.update(
                now,
                cpu_percent >= CPU_ALERT_PERCENT,
                cpu_percent < CPU_CLEAR_PERCENT,
                refresh_text,
                |held| {
                    format!(
                        "roboRIO CPU at {cpu_percent:.0}% for {held:.0} s - loop budget at risk"
                    )
                },
            );
        }

        // Memory
        if let Some(available_mb) = self.read_mem_available_mb() {
            log_dash_always_with_unit("System/MemAvailableMB", available_mb, "MB");
            self.memory_alert.update(
                now,
                available_mb < MEMORY_ALERT_MB,
                true,
                refresh_text,
                |held| {
                    format!("roboRIO memory low: {available_mb:.0} MB available for {held:.0} s")
                },
            );
        }

        // Latched one-shots
        if now > self.state.borrow().stall_latch_until_seconds {
            self.stall_alert.set(false);
        }
    }

    /// Whole-machine CPU busy percent since the previous call, from the first line of
    /// `/proc/stat`, or `None` on the first call or where `/proc` does not exist.
    fn read_cpu_percent(&self) -> Option<f64> {
        let mut s = self.state.borrow_mut();
        if !s.proc_stat_available {
            return None;
        }
        let parsed = fs::read_to_string(&self.proc_stat)
            .ok()
            .and_then(|text| text.lines().next().map(parse_cpu_line));
        let Some(Some((total, idle))) = parsed else {
            // Not Linux, or an unexpected format: stop trying rather than fail every second.
            s.proc_stat_available = false;
            return None;
        };
        let Some(last_total) = s.last_cpu_total_jiffies.replace(total) else {
            s.last_cpu_idle_jiffies = idle;
            return None;
        };
        let delta_total = total - last_total;
        let delta_idle = idle - s.last_cpu_idle_jiffies;
        s.last_cpu_idle_jiffies = idle;
        (delta_total > 0).then(|| 100.0 * (delta_total - delta_idle) as f64 / delta_total as f64)
    }

    /// `MemAvailable` in megabytes, or `None` where it does not exist.
    fn read_mem_available_mb(&self) -> Option<f64> {
        let mut s = self.state.borrow_mut();
        if !s.proc_meminfo_available {
            return None;
        }
        match fs::read_to_string(&self.proc_meminfo) {
            Ok(text) => parse_mem_available_mb(&text),
            Err(_) => {
                s.proc_meminfo_available = false;
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::testing;
    use std::rc::Rc;

    #[derive(Default)]
    struct FakeAlert {
        active: Cell<bool>,
        text: RefCell<String>,
    }

    impl AlertLike for Rc<FakeAlert> {
        fn get(&self) -> bool {
            self.active.get()
        }

        fn set(&self, active: bool) {
            self.active.set(active);
        }

        fn set_text(&self, text: &str) {
            *self.text.borrow_mut() = text.to_owned();
        }
    }

    type Alerts = Rc<RefCell<Vec<(String, Rc<FakeAlert>)>>>;

    fn monitor(stat: &str, meminfo: &str) -> (SystemLoadMonitor<Rc<FakeAlert>>, Alerts) {
        let alerts: Alerts = Rc::default();
        let registry = alerts.clone();
        let monitor = SystemLoadMonitor::with_alerts(move |text, _| {
            let alert = Rc::new(FakeAlert::default());
            registry.borrow_mut().push((text.to_owned(), alert.clone()));
            alert
        })
        .with_proc_files(stat, meminfo);
        (monitor, alerts)
    }

    fn alert(alerts: &Alerts, name: &str) -> Rc<FakeAlert> {
        alerts
            .borrow()
            .iter()
            .find(|(n, _)| n == name)
            .unwrap()
            .1
            .clone()
    }

    /// Runs the monitor at a fixed loop period for `seconds`.
    fn run(m: &SystemLoadMonitor<Rc<FakeAlert>>, start: f64, period: f64, seconds: f64) -> f64 {
        let mut now = start;
        while now < start + seconds {
            m.periodic_at(now, true);
            now += period;
        }
        now
    }

    fn last(out: &[testing::Logged], key: &str) -> f64 {
        out.iter().rev().find(|l| l.key == key).unwrap().double()
    }

    #[test]
    fn cpu_line_parsing() {
        assert_eq!(
            parse_cpu_line("cpu  100 20 30 400 50 6 7 8 9 10"),
            Some((100 + 20 + 30 + 400 + 50 + 6 + 7 + 8, 450))
        );
        assert_eq!(parse_cpu_line("cpu0 1 2 3 4 5 6 7 8"), None);
        assert_eq!(parse_cpu_line("cpu  1 2 x 4 5"), None);
        assert_eq!(parse_cpu_line("cpu  1 2 3 4"), Some((10, 4)));
    }

    #[test]
    fn meminfo_parsing() {
        let text = "MemTotal: 250000 kB\nMemFree: 4500 kB\nMemAvailable: 51200 kB\n";
        assert_eq!(parse_mem_available_mb(text), Some(50.0));
        assert_eq!(parse_mem_available_mb("MemTotal: 1 kB\n"), None);
        let late = format!("{}MemAvailable: 1024 kB\n", "Other: 1 kB\n".repeat(10));
        assert_eq!(
            parse_mem_available_mb(&late),
            None,
            "only the first ten lines count"
        );
        assert_eq!(parse_mem_available_mb("MemAvailable: lots kB\n"), None);
    }

    #[test]
    fn sustained_alert_needs_the_hold_time_and_hysteresis() {
        let fake = Rc::new(FakeAlert::default());
        let a = SustainedAlert::new(fake.clone(), 10.0);
        let text = |held: f64| format!("held {held:.0}");
        a.update(0.0, true, false, false, text);
        a.update(9.0, true, false, false, text);
        assert!(!fake.active.get());
        a.update(10.0, true, false, false, text);
        assert!(fake.active.get());
        assert_eq!(*fake.text.borrow(), "held 10");
        // Text only rewrites when asked.
        a.update(12.0, true, false, false, text);
        assert_eq!(*fake.text.borrow(), "held 10");
        a.update(13.0, true, false, true, text);
        assert_eq!(*fake.text.borrow(), "held 13");
        // In between: neither bad nor recovered, nothing changes.
        a.update(14.0, false, false, false, text);
        assert!(fake.active.get());
        a.update(15.0, false, true, false, text);
        assert!(!fake.active.get());
        // The clock restarts.
        a.update(16.0, true, false, false, text);
        a.update(20.0, true, false, false, text);
        assert!(!fake.active.get());
    }

    #[test]
    fn overrunning_loops_raise_the_alert_after_five_seconds() {
        let _g = testing::serial();
        let (m, alerts) = monitor("/nonexistent/stat", "/nonexistent/meminfo");
        let out = testing::capture(|| {
            run(&m, 0.0, 0.03, 4.0);
        });
        assert!(!alert(&alerts, "Robot loop overrunning").active.get());
        assert!((last(&out, "System/Loop/MeanPeriodMs") - 30.0).abs() < 1e-6);
        assert!((last(&out, "System/Loop/MaxPeriodMs") - 30.0).abs() < 1e-6);
        assert_eq!(last(&out, "System/Loop/OverrunPercent"), 100.0);
        assert!(out.iter().all(|l| l.force_nt), "published on every sample");

        testing::capture(|| {
            run(&m, 4.0, 0.03, 4.0);
        });
        let a = alert(&alerts, "Robot loop overrunning");
        assert!(a.active.get());
        assert!(
            a.text
                .borrow()
                .starts_with("Robot loop overrunning: 100% of loops over 25 ms")
        );

        // Back to 20 ms: clears once under 25 percent.
        testing::capture(|| {
            run(&m, 8.0, 0.02, 3.0);
        });
        assert!(!a.active.get());
    }

    #[test]
    fn healthy_loops_raise_nothing() {
        let _g = testing::serial();
        let (m, alerts) = monitor("/nonexistent/stat", "/nonexistent/meminfo");
        let out = testing::capture(|| {
            run(&m, 0.0, 0.02, 12.0);
        });
        assert_eq!(last(&out, "System/Loop/OverrunPercent"), 0.0);
        assert!(alerts.borrow().iter().all(|(_, a)| !a.active.get()));
        assert!(!out.iter().any(|l| l.key == "System/CpuPercent"));
        assert!(!out.iter().any(|l| l.key == "System/MemAvailableMB"));
    }

    #[test]
    fn a_stall_while_enabled_latches_for_ten_seconds() {
        let _g = testing::serial();
        let (m, alerts) = monitor("/nonexistent/stat", "/nonexistent/meminfo");
        testing::capture(|| {
            run(&m, 0.0, 0.02, 1.0);
            m.periodic_at(1.5, true);
        });
        let stall = alert(&alerts, "Robot loop stalled while enabled");
        assert!(stall.active.get());
        assert!(stall.text.borrow().starts_with("Robot loop stalled "));
        testing::capture(|| {
            run(&m, 1.5, 0.02, 9.0);
        });
        assert!(stall.active.get(), "still latched at 10.5 s");
        testing::capture(|| {
            run(&m, 10.5, 0.02, 3.0);
        });
        assert!(!stall.active.get());
    }

    #[test]
    fn a_stall_while_disabled_is_ignored() {
        let _g = testing::serial();
        let (m, alerts) = monitor("/nonexistent/stat", "/nonexistent/meminfo");
        testing::capture(|| {
            m.periodic_at(0.0, false);
            m.periodic_at(0.02, false);
            m.periodic_at(1.0, false);
        });
        assert!(
            !alert(&alerts, "Robot loop stalled while enabled")
                .active
                .get()
        );
    }

    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            let dir =
                std::env::temp_dir().join(format!("telemetry-slm-{name}-{}", std::process::id()));
            let _ = fs::remove_dir_all(&dir);
            fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn cpu_and_memory_come_from_the_proc_files() {
        let _g = testing::serial();
        let dir = TempDir::new("proc");
        let stat = dir.0.join("stat");
        let meminfo = dir.0.join("meminfo");
        fs::write(&meminfo, "MemAvailable: 10240 kB\n").unwrap();
        let (m, alerts) = monitor(stat.to_str().unwrap(), meminfo.to_str().unwrap());

        // 90 busy of 100 jiffies each second.
        let mut busy = 0;
        let mut idle = 0;
        let mut out = Vec::new();
        for second in 0..14 {
            busy += 90;
            idle += 10;
            fs::write(
                &stat,
                format!("cpu  {busy} 0 0 {idle} 0 0 0 0\ncpu0 1 1 1 1\n"),
            )
            .unwrap();
            out.extend(testing::capture(|| {
                m.periodic_at(f64::from(second), true);
            }));
        }
        // The first read only sets the baseline.
        let cpu: Vec<_> = out
            .iter()
            .filter(|l| l.key == "System/CpuPercent")
            .collect();
        assert!(cpu.len() >= 12);
        assert!((cpu[0].double() - 90.0).abs() < 1e-9);
        let mem = last(&out, "System/MemAvailableMB");
        assert_eq!(mem, 10.0);
        assert!(alert(&alerts, "roboRIO CPU high").active.get());
        assert!(alert(&alerts, "roboRIO memory low").active.get());
        assert!(
            alert(&alerts, "roboRIO CPU high")
                .text
                .borrow()
                .starts_with("roboRIO CPU at 90% for ")
        );
    }

    #[test]
    fn missing_proc_files_stop_being_read() {
        let _g = testing::serial();
        let dir = TempDir::new("gone");
        let stat = dir.0.join("stat");
        fs::write(&stat, "cpu  1 0 0 1 0 0 0 0\n").unwrap();
        let (m, _alerts) = monitor(stat.to_str().unwrap(), "/nonexistent/meminfo");
        testing::capture(|| {
            m.periodic_at(0.0, true);
            m.periodic_at(1.0, true);
        });
        assert!(m.state.borrow().proc_stat_available);
        fs::remove_file(&stat).unwrap();
        testing::capture(|| m.periodic_at(2.0, true));
        assert!(!m.state.borrow().proc_stat_available);
        assert!(!m.state.borrow().proc_meminfo_available);
    }
}
