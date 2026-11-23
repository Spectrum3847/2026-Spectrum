//! What DataLogManager and DriverStation copy into the log without being asked: the driver
//! station inputs and the console.

use crate::datalog::DataLog;
use crate::wpilog::payload;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use wpilib::driver_station as ds;

pub const JOYSTICK_PORTS: usize = 6;

/// The driver station inputs at one moment.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DsReading {
    pub enabled: bool,
    pub autonomous: bool,
    pub test: bool,
    pub estop: bool,
    pub sticks: [StickReading; JOYSTICK_PORTS],
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct StickReading {
    pub buttons: u32,
    pub button_count: u8,
    pub axes: Vec<f32>,
    pub povs: Vec<i16>,
}

impl DsReading {
    /// Reads the driver station data the robot loop last refreshed.
    pub fn read() -> Self {
        let word = ds::control_word();
        let mut out = Self {
            enabled: word.enabled(),
            autonomous: word.autonomous(),
            test: word.test(),
            estop: word.estop(),
            sticks: Default::default(),
        };
        for (i, stick) in out.sticks.iter_mut().enumerate() {
            let port = i as i32;
            stick.buttons = ds::stick_buttons(port);
            stick.button_count = ds::stick_button_count(port) as u8;
            stick.axes = (0..ds::stick_axis_count(port))
                .map(|a| ds::stick_axis(port, a as i32) as f32)
                .collect();
            stick.povs = (0..ds::stick_pov_count(port))
                .map(|p| ds::stick_pov(port, p as i32) as i16)
                .collect();
        }
        out
    }
}

struct StickIds {
    buttons: u32,
    axes: u32,
    povs: u32,
}

/// Writes `DS:*` entries and appends a record when a value changes, as WPILib's
/// `DriverStation.startDataLog` does.
pub struct DsCapture {
    log: Arc<DataLog>,
    enabled: u32,
    autonomous: u32,
    test: u32,
    estop: u32,
    sticks: Vec<StickIds>,
    last: DsReading,
    scratch: Vec<u8>,
}

impl DsCapture {
    /// Starts the entries and records the first reading.
    pub fn new(log: Arc<DataLog>, reading: DsReading, ts: u64) -> Self {
        let start = |name: &str, type_name: &str| log.start(name, type_name, "", ts).unwrap_or(0);
        let enabled = start("DS:enabled", payload::BOOLEAN);
        let autonomous = start("DS:autonomous", payload::BOOLEAN);
        let test = start("DS:test", payload::BOOLEAN);
        let estop = start("DS:estop", payload::BOOLEAN);
        let sticks = (0..JOYSTICK_PORTS)
            .map(|i| StickIds {
                buttons: start(&format!("DS:joystick{i}/buttons"), payload::BOOLEAN_ARRAY),
                axes: start(&format!("DS:joystick{i}/axes"), payload::FLOAT_ARRAY),
                povs: start(&format!("DS:joystick{i}/povs"), payload::INT64_ARRAY),
            })
            .collect();
        let mut capture = Self {
            log: log.clone(),
            enabled,
            autonomous,
            test,
            estop,
            sticks,
            last: reading.clone(),
            scratch: Vec::new(),
        };
        for (id, value) in capture.flags(&reading) {
            capture.append_bool(id, value, ts);
        }
        for i in 0..JOYSTICK_PORTS {
            capture.append_buttons(i, &reading.sticks[i], ts);
            capture.append_axes(i, &reading.sticks[i], ts);
            capture.append_povs(i, &reading.sticks[i], ts);
        }
        capture
    }

    fn flags(&self, r: &DsReading) -> [(u32, bool); 4] {
        [
            (self.enabled, r.enabled),
            (self.autonomous, r.autonomous),
            (self.test, r.test),
            (self.estop, r.estop),
        ]
    }

    /// Appends what changed since the last call.
    pub fn poll(&mut self, reading: &DsReading, ts: u64) {
        let last = std::mem::replace(&mut self.last, reading.clone());
        let old = self.flags(&last);
        for (i, (id, value)) in self.flags(reading).into_iter().enumerate() {
            if value != old[i].1 {
                self.append_bool(id, value, ts);
            }
        }
        for i in 0..JOYSTICK_PORTS {
            let (new, prev) = (&reading.sticks[i], &last.sticks[i]);
            if new.button_count != prev.button_count || new.buttons != prev.buttons {
                self.append_buttons(i, new, ts);
            }
            if new.axes != prev.axes {
                self.append_axes(i, new, ts);
            }
            if new.povs != prev.povs {
                self.append_povs(i, new, ts);
            }
        }
    }

    fn append_bool(&mut self, id: u32, value: bool, ts: u64) {
        self.scratch.clear();
        payload::boolean(&mut self.scratch, value);
        self.log.append(id, ts, &self.scratch);
    }

    fn append_buttons(&mut self, stick: usize, r: &StickReading, ts: u64) {
        let bits: Vec<bool> = (0..u32::from(r.button_count))
            .map(|b| r.buttons & (1 << b) != 0)
            .collect();
        self.scratch.clear();
        payload::boolean_array(&mut self.scratch, &bits);
        self.log
            .append(self.sticks[stick].buttons, ts, &self.scratch);
    }

    fn append_axes(&mut self, stick: usize, r: &StickReading, ts: u64) {
        self.scratch.clear();
        payload::float_array(&mut self.scratch, &r.axes);
        self.log.append(self.sticks[stick].axes, ts, &self.scratch);
    }

    fn append_povs(&mut self, stick: usize, r: &StickReading, ts: u64) {
        let povs: Vec<i64> = r.povs.iter().map(|&p| i64::from(p)).collect();
        self.scratch.clear();
        payload::int64_array(&mut self.scratch, &povs);
        self.log.append(self.sticks[stick].povs, ts, &self.scratch);
    }
}

/// Follows a text file that another process appends to, as WPILib's `FileLogger` does: it
/// starts at the end of the file and reports everything up to the last newline in one piece.
pub struct ConsoleTail {
    path: PathBuf,
    file: Option<File>,
    pending: Vec<u8>,
}

impl ConsoleTail {
    pub fn new(path: &Path) -> Self {
        let mut tail = Self {
            path: path.to_owned(),
            file: None,
            pending: Vec::new(),
        };
        tail.open_at_end();
        tail
    }

    fn open_at_end(&mut self) {
        self.file = File::open(&self.path).ok().and_then(|mut f| {
            f.seek(SeekFrom::End(0)).ok()?;
            Some(f)
        });
    }

    /// Reads what was appended since the last call and passes complete lines to `emit`.
    pub fn poll(&mut self, mut emit: impl FnMut(&str)) {
        if self.file.is_none() {
            // The file may appear after the program starts.
            self.file = File::open(&self.path).ok();
        }
        let Some(file) = self.file.as_mut() else {
            return;
        };
        let mut chunk = Vec::new();
        if file.read_to_end(&mut chunk).is_err() || chunk.is_empty() {
            return;
        }
        self.pending.extend_from_slice(&chunk);
        let Some(last_newline) = self.pending.iter().rposition(|&b| b == b'\n') else {
            return;
        };
        let rest = self.pending.split_off(last_newline + 1);
        self.pending.pop();
        emit(&String::from_utf8_lossy(&self.pending));
        self.pending = rest;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datalog::Config;
    use crate::reader::{Log, decode};
    use std::io::Write;
    use std::sync::Mutex;

    fn memory_log() -> (Arc<DataLog>, Arc<Mutex<Vec<u8>>>) {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let sink = bytes.clone();
        let log = Arc::new(DataLog::new(Config::callback(move |b| {
            sink.lock().unwrap().extend_from_slice(b);
        })));
        (log, bytes)
    }

    fn parsed(log: Arc<DataLog>, bytes: &Arc<Mutex<Vec<u8>>>) -> Log {
        drop(Arc::into_inner(log).unwrap());
        Log::parse(&bytes.lock().unwrap()).unwrap()
    }

    #[test]
    fn ds_capture_logs_the_first_reading_then_changes() {
        let (log, bytes) = memory_log();
        let mut reading = DsReading::default();
        reading.sticks[0] = StickReading {
            buttons: 0b101,
            button_count: 3,
            axes: vec![0.5, -1.0],
            povs: vec![-1],
        };
        let mut capture = DsCapture::new(log.clone(), reading.clone(), 100);

        capture.poll(&reading, 200);
        reading.enabled = true;
        reading.sticks[0].axes[0] = 0.75;
        capture.poll(&reading, 300);
        drop(capture);

        let out = parsed(log, &bytes);
        let enabled = out.entry("DS:enabled").unwrap();
        assert_eq!(
            enabled.records,
            [(100, vec![0]), (300, vec![1])],
            "unchanged polls add nothing"
        );
        assert_eq!(out.entry("DS:estop").unwrap().records.len(), 1);
        let buttons = out.entry("DS:joystick0/buttons").unwrap();
        assert_eq!(buttons.type_name, "boolean[]");
        assert_eq!(buttons.records, [(100, vec![1, 0, 1])]);
        let axes = out.entry("DS:joystick0/axes").unwrap();
        assert_eq!(axes.type_name, "float[]");
        assert_eq!(axes.records.len(), 2);
        assert_eq!(
            decode::float_array(&axes.records[1].1),
            Some(vec![0.75, -1.0])
        );
        let povs = out.entry("DS:joystick0/povs").unwrap();
        assert_eq!(decode::int64_array(&povs.records[0].1), Some(vec![-1]));
        assert_eq!(
            out.entry("DS:joystick5/axes").unwrap().records[0].1.len(),
            0
        );
        assert_eq!(out.entry("DS:enabled").unwrap().metadata, "");
    }

    #[test]
    fn console_tail_reports_whole_lines_from_the_end() {
        let dir = std::env::temp_dir().join(format!("telemetry-tail-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("FRC_UserProgram.log");
        std::fs::write(&path, "old line\n").unwrap();

        let mut tail = ConsoleTail::new(&path);
        let mut out = Vec::new();
        tail.poll(|s| out.push(s.to_owned()));
        assert!(out.is_empty(), "text before the start is not reported");

        let mut file = std::fs::OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap();
        write!(file, "one\ntwo\nthr").unwrap();
        tail.poll(|s| out.push(s.to_owned()));
        assert_eq!(out, ["one\ntwo"]);

        writeln!(file, "ee").unwrap();
        tail.poll(|s| out.push(s.to_owned()));
        assert_eq!(out, ["one\ntwo", "three"]);

        tail.poll(|s| out.push(s.to_owned()));
        assert_eq!(out.len(), 2);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn console_tail_waits_for_a_missing_file() {
        let dir = std::env::temp_dir().join(format!("telemetry-tail2-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("later.log");
        let mut tail = ConsoleTail::new(&path);
        let mut out = Vec::new();
        tail.poll(|s| out.push(s.to_owned()));
        std::fs::write(&path, "hello\n").unwrap();
        tail.poll(|s| out.push(s.to_owned()));
        assert_eq!(out, ["hello"]);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
