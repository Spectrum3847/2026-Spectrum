//! WPILib's `DataLogManager`: picks the log folder and file name, renames the file when the
//! driver station connects, pauses while it is gone, clears old logs and adds the captures.
//!
//! A file starts as `FRC_TBD_<random>.wpilog` and becomes `FRC_<UTC time>.wpilog` once the
//! driver station has been attached for a second, or
//! `FRC_<UTC time>_<event>_<P|Q|E><match>.wpilog` once the FMS has sent the match.

use crate::capture::{ConsoleTail, DsCapture, DsReading};
use crate::datalog::{self, Config, DataLog};
use crate::native;
use crate::wpilog::payload;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use wpilib::driver_station::{self, MatchType};

/// Below this much free space, old logs are deleted.
const FREE_SPACE_THRESHOLD: u64 = 50_000_000;
/// Cleanup keeps at least this many logs.
const FILE_COUNT_THRESHOLD: usize = 10;
const ROBORIO_LOG_DIR: &str = "/home/lvuser/logs";
const USB_LOG_DIR: &str = "/u/logs";
const CONSOLE_FILE: &str = "/home/lvuser/FRC_UserProgram.log";
/// How often the manager thread looks at the driver station.
const TICK: Duration = Duration::from_millis(20);
const FLUSH_PERIOD: Duration = Duration::from_millis(250);

/// Where logs go: a USB stick's `logs` folder if one is writable at `/u`, else
/// `/home/lvuser/logs` on a robot, and `logs` in the working directory in simulation.
pub fn make_log_dir(real: bool) -> PathBuf {
    if !real {
        let dir = std::env::current_dir().unwrap_or_default().join("logs");
        let _ = fs::create_dir(&dir);
        return dir;
    }
    if let Ok(usb) = fs::canonicalize("/u")
        && is_writable_dir(&usb)
    {
        let _ = fs::create_dir(USB_LOG_DIR);
        return PathBuf::from(USB_LOG_DIR);
    }
    if wpilib::hal::runtime_type() == wpilib::RuntimeType::RoboRio {
        wpilib::driver_station::report_warning(
            "DataLogManager: Logging to RoboRIO 1 internal storage is not recommended! \
             Plug in a FAT32 formatted flash drive!",
        );
    }
    let _ = fs::create_dir(ROBORIO_LOG_DIR);
    PathBuf::from(ROBORIO_LOG_DIR)
}

fn is_writable_dir(path: &Path) -> bool {
    fs::metadata(path).is_ok_and(|m| m.is_dir() && !m.permissions().readonly())
}

/// True on a roboRIO 1 that logs to its internal flash. DogLog warns about that.
pub fn is_unsafe_destination(dir: &Path) -> bool {
    dir == Path::new(ROBORIO_LOG_DIR) && wpilib::hal::runtime_type() == wpilib::RuntimeType::RoboRio
}

/// `FRC_TBD_` and 16 hex digits.
pub fn temporary_filename() -> String {
    let mut name = String::from("FRC_TBD_");
    for _ in 0..4 {
        name.push_str(&format!("{:04x}", datalog::random_bits() & 0xffff));
    }
    name.push_str(".wpilog");
    name
}

/// Deletes the leftover temporary files of earlier runs that never met a driver station.
pub fn delete_temporary_logs(dir: &Path) {
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with("FRC_TBD_")
            && name.ends_with(".wpilog")
            && let Err(e) = fs::remove_file(entry.path())
        {
            eprintln!("DataLogManager: could not delete {name}: {e}");
        }
    }
}

/// Deletes the oldest `FRC_*.wpilog` files while `free_space` is under the threshold, down to
/// at most [`FILE_COUNT_THRESHOLD`] files. Returns the names it deleted.
pub fn delete_old_logs(dir: &Path, mut free_space: u64) -> Vec<String> {
    let mut deleted = Vec::new();
    if free_space >= FREE_SPACE_THRESHOLD {
        if free_space < 2 * FREE_SPACE_THRESHOLD {
            wpilib::driver_station::report_warning(&format!(
                "DataLogManager: Log storage device has {} MB of free space remaining! Logs will \
                 get deleted below {} MB of free space. Consider deleting logs off the storage \
                 device.",
                free_space / 1_000_000,
                FREE_SPACE_THRESHOLD / 1_000_000
            ));
        }
        return deleted;
    }
    let Ok(entries) = fs::read_dir(dir) else {
        return deleted;
    };
    let mut files: Vec<(SystemTime, String, PathBuf, u64)> = entries
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            if !(name.starts_with("FRC_") && name.ends_with(".wpilog"))
                || name.starts_with("FRC_TBD_")
            {
                return None;
            }
            let meta = e.metadata().ok()?;
            Some((meta.modified().ok()?, name, e.path(), meta.len()))
        })
        .collect();
    files.sort();
    let mut count = files.len();
    for (_, name, path, len) in files {
        count -= 1;
        if count < FILE_COUNT_THRESHOLD {
            break;
        }
        match fs::remove_file(&path) {
            Ok(()) => {
                wpilib::driver_station::report_warning(&format!("DataLogManager: Deleted {name}"));
                free_space += len;
                deleted.push(name);
                if free_space >= FREE_SPACE_THRESHOLD {
                    break;
                }
            }
            Err(e) => eprintln!("DataLogManager: could not delete {name}: {e}"),
        }
    }
    deleted
}

/// `yyyyMMdd_HHmmss` in UTC for a Unix time in seconds.
pub fn utc_stamp(unix_seconds: i64) -> String {
    let days = unix_seconds.div_euclid(86_400);
    let secs = unix_seconds.rem_euclid(86_400);
    // Civil date from a day count (H. Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}{month:02}{day:02}_{:02}{:02}{:02}",
        secs / 3600,
        secs % 3600 / 60,
        secs % 60
    )
}

fn unix_micros() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_micros() as i64)
}

/// What the manager thread sees each tick.
pub struct Tick<'a> {
    /// The driver station is sending data. Always true in simulation, where nothing says
    /// otherwise.
    pub data_flowing: bool,
    pub ds_attached: bool,
    pub fms_attached: bool,
    pub system_time_valid: bool,
    pub match_type: MatchType,
    pub event_name: &'a str,
    pub match_number: u16,
    pub utc: &'a dyn Fn() -> String,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Action {
    Rename(String),
    Pause,
    Resume,
    /// Append the system time to `systemTime`.
    SystemTime,
}

/// The renaming, pausing and clock logging rules, one call per [`TICK`].
#[derive(Default)]
pub struct Monitor {
    timeout_ticks: u32,
    paused: bool,
    ds_attach_ticks: u32,
    fms_attach_ticks: u32,
    system_time_ticks: u32,
    ds_renamed: bool,
    fms_renamed: bool,
}

impl Monitor {
    /// Java waits 0.25 s per timeout and gives up after 40, which is 10 s.
    const PAUSE_AFTER_TICKS: u32 = 500;
    /// Java counts driver station packets, 50 per second.
    const DS_RENAME_TICKS: u32 = 50;
    const FMS_RENAME_TICKS: u32 = 250;
    const SYSTEM_TIME_TICKS: u32 = 250;

    /// A monitor for a file that already has its final name, as when one was given.
    pub fn named() -> Self {
        Self {
            ds_renamed: true,
            fms_renamed: true,
            ..Self::default()
        }
    }

    pub fn step(&mut self, t: &Tick) -> Vec<Action> {
        let mut actions = Vec::new();
        if !t.data_flowing {
            self.timeout_ticks += 1;
            if self.timeout_ticks > Self::PAUSE_AFTER_TICKS && !self.paused {
                self.timeout_ticks = 0;
                self.paused = true;
                actions.push(Action::Pause);
            }
            return actions;
        }
        self.timeout_ticks = 0;
        if self.paused {
            self.paused = false;
            actions.push(Action::Resume);
        }

        if !self.ds_renamed {
            self.ds_attach_ticks = if t.ds_attached {
                self.ds_attach_ticks + 1
            } else {
                0
            };
            if self.ds_attach_ticks > Self::DS_RENAME_TICKS {
                if t.system_time_valid {
                    actions.push(Action::Rename(format!("FRC_{}.wpilog", (t.utc)())));
                    self.ds_renamed = true;
                } else {
                    self.ds_attach_ticks = 0;
                }
            }
        }

        if !self.fms_renamed {
            self.fms_attach_ticks = if t.fms_attached {
                self.fms_attach_ticks + 1
            } else {
                0
            };
            // Match info arrives over TCP, so check it came.
            if self.fms_attach_ticks > Self::FMS_RENAME_TICKS && t.match_type != MatchType::None {
                let kind = match t.match_type {
                    MatchType::Practice => 'P',
                    MatchType::Qualification => 'Q',
                    MatchType::Elimination => 'E',
                    MatchType::None => '_',
                };
                actions.push(Action::Rename(format!(
                    "FRC_{}_{}_{kind}{}.wpilog",
                    (t.utc)(),
                    t.event_name,
                    t.match_number
                )));
                self.fms_renamed = true;
                // Do not override the FMS name.
                self.ds_renamed = true;
            }
        }

        self.system_time_ticks += 1;
        if self.system_time_ticks >= Self::SYSTEM_TIME_TICKS {
            self.system_time_ticks = 0;
            if t.system_time_valid {
                actions.push(Action::SystemTime);
            }
        }
        actions
    }
}

struct Shared {
    stop: Mutex<bool>,
    cond: Condvar,
    ds_capture: AtomicBool,
    console: AtomicBool,
}

/// The managed log and the thread that looks after it.
pub struct DataLogManager {
    log: Arc<DataLog>,
    dir: PathBuf,
    shared: Arc<Shared>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl DataLogManager {
    /// Opens a log in the default folder with a temporary name. `real` is true on a robot.
    pub fn start(real: bool) -> Self {
        let dir = make_log_dir(real);
        delete_temporary_logs(&dir);
        let log = Arc::new(DataLog::new({
            let mut config = Config::directory(&dir, &temporary_filename());
            config.period = FLUSH_PERIOD;
            config.clock = || wpilib::robot_controller::fpga_time();
            config
        }));
        let shared = Arc::new(Shared {
            stop: Mutex::new(false),
            cond: Condvar::new(),
            ds_capture: AtomicBool::new(false),
            console: AtomicBool::new(false),
        });
        let thread = thread::Builder::new()
            .name("DataLogDS".into())
            .spawn({
                let (log, shared, dir) = (log.clone(), shared.clone(), dir.clone());
                move || manager_main(&log, &shared, &dir, real)
            })
            .ok();
        Self {
            log,
            dir,
            shared,
            thread: Mutex::new(thread),
        }
    }

    pub fn log(&self) -> &Arc<DataLog> {
        &self.log
    }

    pub fn log_dir(&self) -> &Path {
        &self.dir
    }

    /// Copies `/home/lvuser/FRC_UserProgram.log` into the `console` entry. Only on a robot.
    pub fn set_console_logging(&self, enabled: bool) {
        self.shared.console.store(enabled, Ordering::Relaxed);
    }

    /// Logs control word and joystick changes. Repeated calls change nothing.
    pub fn start_ds_capture(&self) {
        self.shared.ds_capture.store(true, Ordering::Relaxed);
    }
}

impl DataLogManager {
    /// Stops the manager thread, then the log's writer, which writes out its buffer first.
    /// Safe to call twice.
    pub fn shutdown(&self) {
        *self
            .shared
            .stop
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = true;
        self.shared.cond.notify_all();
        let thread = self
            .thread
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        if let Some(thread) = thread {
            let _ = thread.join();
        }
        self.log.shutdown();
    }
}

impl Drop for DataLogManager {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn manager_main(log: &Arc<DataLog>, shared: &Shared, dir: &Path, real: bool) {
    if let Some(free) = datalog::free_space(dir) {
        delete_old_logs(dir, free);
    }
    let system_time = log
        .start(
            "systemTime",
            payload::INT64,
            "{\"source\":\"DataLogManager\",\"format\":\"time_t_us\"}",
            0,
        )
        .unwrap_or(0);
    let mut monitor = Monitor::default();
    let mut ds: Option<DsCapture> = None;
    let mut console: Option<(ConsoleTail, u32)> = None;
    let mut ticks = 0u32;

    loop {
        {
            let stop = shared.stop.lock().unwrap_or_else(PoisonError::into_inner);
            let (stop, _) = shared
                .cond
                .wait_timeout(stop, TICK)
                .unwrap_or_else(PoisonError::into_inner);
            if *stop {
                return;
            }
        }
        ticks = ticks.wrapping_add(1);
        let now = wpilib::robot_controller::fpga_time();

        if shared.ds_capture.load(Ordering::Relaxed) {
            let reading = DsReading::read();
            match ds.as_mut() {
                Some(capture) => capture.poll(&reading, now),
                None => ds = Some(DsCapture::new(log.clone(), reading, now)),
            }
        }

        if real && shared.console.load(Ordering::Relaxed) {
            let (tail, entry) = console.get_or_insert_with(|| {
                let id = log.start("console", payload::STRING, "", 0).unwrap_or(0);
                (ConsoleTail::new(Path::new(CONSOLE_FILE)), id)
            });
            // FileLogger checks about every 100 ms.
            if ticks.is_multiple_of(5) {
                let entry = *entry;
                tail.poll(|text| log.append(entry, now, text.as_bytes()));
            }
        }

        let ds_attached = driver_station::is_ds_attached();
        let fms_attached = driver_station::is_fms_attached();
        let match_info = if fms_attached {
            driver_station::match_info()
        } else {
            Default::default()
        };
        let tick = Tick {
            // Java pauses in simulation too when no driver station talks to it; a simulated
            // run without one would lose its log after 10 s, so it never pauses here.
            data_flowing: !real || ds_attached,
            ds_attached,
            fms_attached,
            system_time_valid: native::system_time_valid(),
            match_type: match_info.match_type,
            event_name: &match_info.event_name,
            match_number: match_info.match_number,
            utc: &|| utc_stamp(unix_micros() / 1_000_000),
        };
        for action in monitor.step(&tick) {
            match action {
                Action::Rename(name) => log.set_filename(&name),
                Action::Pause => log.set_paused(true),
                Action::Resume => log.set_paused(false),
                Action::SystemTime => {
                    let mut p = Vec::new();
                    payload::int64(&mut p, unix_micros());
                    log.append(system_time, now, &p);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tick<'a>(utc: &'a dyn Fn() -> String) -> Tick<'a> {
        Tick {
            data_flowing: true,
            ds_attached: true,
            fms_attached: false,
            system_time_valid: true,
            match_type: MatchType::None,
            event_name: "",
            match_number: 0,
            utc,
        }
    }

    #[test]
    fn utc_stamps() {
        assert_eq!(utc_stamp(0), "19700101_000000");
        assert_eq!(utc_stamp(1_700_000_000), "20231114_221320");
        assert_eq!(utc_stamp(951_782_400), "20000229_000000");
        assert_eq!(utc_stamp(1_709_251_199), "20240229_235959");
        assert_eq!(utc_stamp(-1), "19691231_235959");
    }

    #[test]
    fn temporary_names() {
        let name = temporary_filename();
        assert!(name.starts_with("FRC_TBD_") && name.ends_with(".wpilog"));
        assert_eq!(name.len(), "FRC_TBD_".len() + 16 + ".wpilog".len());
        assert_ne!(name, temporary_filename());
    }

    #[test]
    fn renames_after_a_second_of_ds_data() {
        let utc = || "20260101_120000".to_owned();
        let mut m = Monitor::default();
        for _ in 0..50 {
            assert!(m.step(&tick(&utc)).is_empty());
        }
        assert_eq!(
            m.step(&tick(&utc)),
            [Action::Rename("FRC_20260101_120000.wpilog".into())]
        );
        for _ in 0..100 {
            assert!(!m.step(&tick(&utc)).contains(&Action::Rename(String::new())));
        }
    }

    #[test]
    fn waits_for_a_valid_system_time_before_renaming() {
        let utc = || "t".to_owned();
        let mut m = Monitor::default();
        let mut invalid = tick(&utc);
        invalid.system_time_valid = false;
        for _ in 0..200 {
            assert!(
                !m.step(&invalid)
                    .contains(&Action::Rename("FRC_t.wpilog".into()))
            );
        }
        // The count starts over each time the time is invalid, so a rename comes within 51
        // ticks of it turning valid.
        let renamed =
            (0..52).any(|_| m.step(&tick(&utc)) == [Action::Rename("FRC_t.wpilog".into())]);
        assert!(renamed);
    }

    #[test]
    fn fms_names_the_match_and_wins() {
        let utc = || "t".to_owned();
        let mut m = Monitor::default();
        let mut fms = tick(&utc);
        fms.fms_attached = true;
        fms.match_type = MatchType::Qualification;
        fms.event_name = "TXDA";
        fms.match_number = 12;
        let mut names = Vec::new();
        for _ in 0..400 {
            for a in m.step(&fms) {
                if let Action::Rename(n) = a {
                    names.push(n);
                }
            }
        }
        // The driver station rename comes first, as in Java, then the FMS one.
        assert_eq!(names, ["FRC_t.wpilog", "FRC_t_TXDA_Q12.wpilog"]);
    }

    #[test]
    fn fms_waits_for_match_info() {
        let utc = || "t".to_owned();
        let mut m = Monitor {
            ds_renamed: true,
            ..Monitor::default()
        };
        let mut fms = tick(&utc);
        fms.fms_attached = true;
        for _ in 0..400 {
            let actions = m.step(&fms);
            assert!(actions.iter().all(|a| *a == Action::SystemTime));
        }
        fms.match_type = MatchType::Elimination;
        fms.event_name = "E";
        fms.match_number = 3;
        assert!(
            m.step(&fms)
                .contains(&Action::Rename("FRC_t_E_E3.wpilog".into()))
        );
    }

    #[test]
    fn pauses_after_ten_seconds_without_data_and_resumes() {
        let utc = || "t".to_owned();
        let mut m = Monitor::named();
        let mut gone = tick(&utc);
        gone.data_flowing = false;
        for _ in 0..500 {
            assert!(m.step(&gone).is_empty());
        }
        assert_eq!(m.step(&gone), [Action::Pause]);
        assert!(m.step(&gone).is_empty());
        assert_eq!(m.step(&tick(&utc)), [Action::Resume]);
    }

    #[test]
    fn logs_the_system_time_every_five_seconds() {
        let utc = || "t".to_owned();
        let mut m = Monitor::named();
        let mut count = 0;
        for _ in 0..500 {
            count += m
                .step(&tick(&utc))
                .iter()
                .filter(|a| **a == Action::SystemTime)
                .count();
        }
        assert_eq!(count, 2);
    }

    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            let dir =
                std::env::temp_dir().join(format!("telemetry-dlm-{name}-{}", std::process::id()));
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
    fn temporary_logs_are_deleted_and_others_kept() {
        let dir = TempDir::new("tbd");
        for name in [
            "FRC_TBD_1.wpilog",
            "FRC_20260101_000000.wpilog",
            "notes.txt",
        ] {
            fs::write(dir.0.join(name), "x").unwrap();
        }
        delete_temporary_logs(&dir.0);
        let mut left: Vec<_> = fs::read_dir(&dir.0)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        left.sort();
        assert_eq!(left, ["FRC_20260101_000000.wpilog", "notes.txt"]);
    }

    #[test]
    fn cleanup_keeps_ten_logs_and_deletes_oldest_first() {
        let dir = TempDir::new("old");
        let base = SystemTime::now() - Duration::from_secs(10_000);
        for i in 0..14u64 {
            let path = dir.0.join(format!("FRC_{i:02}.wpilog"));
            let file = fs::File::create(&path).unwrap();
            file.set_modified(base + Duration::from_secs(i * 100))
                .unwrap();
        }
        fs::write(dir.0.join("FRC_TBD_x.wpilog"), "x").unwrap();
        // Plenty of space: nothing goes.
        assert!(delete_old_logs(&dir.0, 10 * FREE_SPACE_THRESHOLD).is_empty());
        // Short of space, and each file frees nothing: all but ten go, oldest first.
        let deleted = delete_old_logs(&dir.0, 0);
        assert_eq!(
            deleted,
            [
                "FRC_00.wpilog",
                "FRC_01.wpilog",
                "FRC_02.wpilog",
                "FRC_03.wpilog"
            ]
        );
        assert!(dir.0.join("FRC_TBD_x.wpilog").exists());
    }

    #[test]
    fn cleanup_stops_once_enough_space_is_free() {
        let dir = TempDir::new("space");
        let base = SystemTime::now() - Duration::from_secs(10_000);
        for i in 0..14u64 {
            let path = dir.0.join(format!("FRC_{i:02}.wpilog"));
            fs::write(&path, vec![0u8; 20]).unwrap();
            fs::File::options()
                .write(true)
                .open(&path)
                .unwrap()
                .set_modified(base + Duration::from_secs(i * 100))
                .unwrap();
        }
        // 20 bytes per file: freeing one is enough from just under the threshold.
        let deleted = delete_old_logs(&dir.0, FREE_SPACE_THRESHOLD - 10);
        assert_eq!(deleted, ["FRC_00.wpilog"]);
    }

    #[test]
    fn sim_log_dir_is_logs_under_the_working_directory() {
        let dir = make_log_dir(false);
        assert_eq!(dir, std::env::current_dir().unwrap().join("logs"));
        let _ = fs::remove_dir(&dir);
    }
}
