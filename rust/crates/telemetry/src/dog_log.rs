//! DogLog 2026.5.0: a logger any thread may call. Calls queue a record. One log thread drains
//! the queue into the wpilog and, when asked, NetworkTables.
//!
//! Dropped from DogLog because the robot never uses them: logging on the calling thread
//! (`useLogThread` false), float values, `Measure` and `Unit` overloads, raw values, record
//! (reflection) logging, tunables other than doubles and their change callbacks, fault
//! alerts, and `captureNt`.

use crate::data_log_manager::{self, DataLogManager};
use crate::datalog::DataLog;
use crate::extras::Extras;
use crate::value::{Entry, LogValue, Value, struct_array_value, struct_value};
use crate::writers::{DataLogWriter, NetworkTablesWriter, NtWriter};
use std::collections::VecDeque;
use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock, PoisonError, RwLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use wpilib::driver_station;
use wpilib::nt::{self, Instance};
use wpilib::power_distribution::PowerDistribution;

/// The log thread writes its own queue diagnostics at most this often.
const DIAGNOSTICS_PERIOD: Duration = Duration::from_millis(20);
const MAX_QUEUE_FULL_MESSAGES: u32 = 50;

pub type BoolFn = Arc<dyn Fn() -> bool + Send + Sync>;

fn not_on_fms() -> bool {
    !driver_station::is_fms_attached()
}

/// DogLog's `DogLogOptions`.
#[derive(Clone)]
pub struct DogLogOptions {
    /// Evaluated by the log thread for every record.
    pub nt_publish: BoolFn,
    /// Not supported here: logging every NetworkTables change needs ntcore's native data log.
    pub capture_nt: bool,
    pub capture_ds: bool,
    pub log_extras: bool,
    pub capture_console: bool,
    pub log_entry_queue_capacity: usize,
    pub nt_tunables: BoolFn,
}

impl Default for DogLogOptions {
    fn default() -> Self {
        Self {
            nt_publish: Arc::new(not_on_fms),
            capture_nt: false,
            capture_ds: false,
            log_extras: true,
            capture_console: true,
            log_entry_queue_capacity: 1000,
            nt_tunables: Arc::new(not_on_fms),
        }
    }
}

impl DogLogOptions {
    pub fn with_nt_publish(mut self, publish: bool) -> Self {
        self.nt_publish = Arc::new(move || publish);
        self
    }

    pub fn with_nt_publish_fn(
        mut self,
        publish: impl Fn() -> bool + Send + Sync + 'static,
    ) -> Self {
        self.nt_publish = Arc::new(publish);
        self
    }

    pub fn with_capture_nt(mut self, capture: bool) -> Self {
        self.capture_nt = capture;
        self
    }

    pub fn with_capture_ds(mut self, capture: bool) -> Self {
        self.capture_ds = capture;
        self
    }

    pub fn with_log_extras(mut self, extras: bool) -> Self {
        self.log_extras = extras;
        self
    }

    pub fn with_capture_console(mut self, capture: bool) -> Self {
        self.capture_console = capture;
        self
    }

    pub fn with_log_entry_queue_capacity(mut self, capacity: usize) -> Self {
        self.log_entry_queue_capacity = capacity;
        self
    }

    pub fn with_nt_tunables(mut self, tunables: bool) -> Self {
        self.nt_tunables = Arc::new(move || tunables);
        self
    }

    pub fn with_nt_tunables_fn(
        mut self,
        tunables: impl Fn() -> bool + Send + Sync + 'static,
    ) -> Self {
        self.nt_tunables = Arc::new(tunables);
        self
    }
}

impl fmt::Debug for DogLogOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DogLogOptions")
            .field("ntPublish", &(self.nt_publish)())
            .field("captureNt", &self.capture_nt)
            .field("captureDs", &self.capture_ds)
            .field("logExtras", &self.log_extras)
            .field("captureConsole", &self.capture_console)
            .field("logEntryQueueCapacity", &self.log_entry_queue_capacity)
            .field("ntTunables", &(self.nt_tunables)())
            .finish()
    }
}

struct Queue {
    items: VecDeque<Entry>,
    capacity: usize,
    /// The log thread is writing a batch it took out of `items`.
    busy: bool,
    shutdown: bool,
}

#[derive(Default)]
struct Faults {
    seen: Vec<String>,
    counts: Vec<i64>,
    active: Vec<String>,
}

impl Faults {
    fn index(&self, name: &str) -> Option<usize> {
        self.seen.iter().position(|n| n == name)
    }
}

struct Inner {
    queue: Mutex<Queue>,
    /// Signals the log thread that records arrived.
    wake: Condvar,
    /// Signals `flush` that the thread went idle.
    idle: Condvar,
    options: RwLock<Arc<DogLogOptions>>,
    enabled: AtomicBool,
    clock: fn() -> u64,
    full_count: AtomicU32,
    faults: Mutex<Faults>,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl Inner {
    fn now(&self) -> u64 {
        (self.clock)()
    }

    fn enqueue(&self, entry: Entry) {
        let key_if_dropped = {
            let mut q = lock(&self.queue);
            // No log thread after shutdown, and flush would wait on the queue forever.
            if q.shutdown {
                return;
            }
            if q.items.len() >= q.capacity {
                Some(entry.key)
            } else {
                let was_empty = q.items.is_empty();
                q.items.push_back(entry);
                drop(q);
                // The thread only sleeps on an empty queue.
                if was_empty {
                    self.wake.notify_one();
                }
                None
            }
        };
        if let Some(key) = key_if_dropped {
            self.queue_full(&key);
        }
    }

    fn queue_full(&self, key: &str) {
        let count = self.full_count.fetch_add(1, Ordering::Relaxed) + 1;
        if count == 1 {
            // Only the first time: a fault is itself a log, which would overflow further.
            self.add_fault("[DogLog] MAX_QUEUED_LOGS");
        }
        if count == MAX_QUEUE_FULL_MESSAGES {
            driver_station::report_error_with_trace(&format!(
                "[DogLog] MAX_QUEUED_LOGS: Log queue is full, dropping log entry for {key} \
                 (additional messages will not be printed)"
            ));
        } else if count < MAX_QUEUE_FULL_MESSAGES {
            driver_station::report_error(&format!(
                "[DogLog] MAX_QUEUED_LOGS: Log queue is full, dropping log entry for {key}"
            ));
        }
    }

    fn log(&self, key: &str, force_nt: bool, value: Value, unit: Option<&str>) {
        if !self.enabled.load(Ordering::Relaxed) {
            return;
        }
        self.enqueue(Entry {
            key: key.to_owned(),
            force_nt,
            timestamp: self.now(),
            value,
            unit: unit.map(str::to_owned),
        });
    }

    fn log_at(&self, ts: u64, key: &str, value: Value) {
        self.enqueue(Entry {
            key: key.to_owned(),
            force_nt: false,
            timestamp: ts,
            value,
            unit: None,
        });
    }

    fn add_fault(&self, name: &str) {
        let now = self.now();
        let mut faults = lock(&self.faults);
        let (previous, count) = match faults.index(name) {
            Some(i) => {
                faults.counts[i] += 1;
                (Some(faults.counts[i] - 1), faults.counts[i])
            }
            None => {
                faults.seen.push(name.to_owned());
                faults.counts.push(1);
                (None, 1)
            }
        };
        if previous.is_none() {
            let seen = faults.seen.clone();
            self.log_at(now, "Faults/Seen", Value::StringArray(seen));
        }
        if previous.is_none_or(|p| p == 0) {
            faults.active.push(name.to_owned());
            let active = faults.active.clone();
            self.log_at(now, "Faults/Active", Value::StringArray(active));
        }
        self.log_at(now, &format!("Faults/Counts/{name}"), Value::Integer(count));
    }

    fn decrease_fault(&self, name: &str) {
        let now = self.now();
        let mut faults = lock(&self.faults);
        let Some(i) = faults.index(name).filter(|&i| faults.counts[i] != 0) else {
            return;
        };
        faults.counts[i] -= 1;
        let count = faults.counts[i];
        self.log_at(now, &format!("Faults/Counts/{name}"), Value::Integer(count));
        if count == 0 {
            faults.active.retain(|n| n != name);
            let active = faults.active.clone();
            self.log_at(now, "Faults/Active", Value::StringArray(active));
        }
    }

    fn clear_fault(&self, name: &str) {
        let now = self.now();
        let mut faults = lock(&self.faults);
        let Some(i) = faults.index(name) else {
            return;
        };
        faults.counts[i] = 0;
        self.log_at(now, &format!("Faults/Counts/{name}"), Value::Integer(0));
        faults.active.retain(|n| n != name);
        let active = faults.active.clone();
        self.log_at(now, "Faults/Active", Value::StringArray(active));
    }
}

/// Sends one record to the wpilog, and to NetworkTables when forced or the options say so.
fn process(
    options: &DogLogOptions,
    datalog: &mut DataLogWriter,
    nt: &mut dyn NtWriter,
    entry: &Entry,
) {
    datalog.log(entry);
    if entry.force_nt || (options.nt_publish)() {
        nt.publish(entry);
    }
}

fn log_thread(inner: &Inner, mut datalog: DataLogWriter, mut nt: Box<dyn NtWriter>) {
    println!("[DogLog] LogThread started");
    let mut diagnostics = Instant::now();
    let mut batch = VecDeque::new();
    loop {
        {
            let mut q = lock(&inner.queue);
            q.busy = false;
            inner.idle.notify_all();
            while q.items.is_empty() && !q.shutdown {
                q = inner.wake.wait(q).unwrap_or_else(PoisonError::into_inner);
            }
            if q.items.is_empty() {
                return;
            }
            std::mem::swap(&mut q.items, &mut batch);
            q.busy = true;
        }
        let options = inner
            .options
            .read()
            .unwrap_or_else(PoisonError::into_inner)
            .clone();
        for entry in batch.drain(..) {
            process(&options, &mut datalog, nt.as_mut(), &entry);
        }
        if diagnostics.elapsed() >= DIAGNOSTICS_PERIOD {
            diagnostics = Instant::now();
            let now = inner.now();
            let (queued, remaining) = {
                let q = lock(&inner.queue);
                (q.items.len(), q.capacity.saturating_sub(q.items.len()))
            };
            for (key, value) in [
                ("DogLog/QueuedLogs", queued),
                ("DogLog/QueueRemainingCapacity", remaining),
            ] {
                let entry = Entry {
                    key: key.to_owned(),
                    force_nt: false,
                    timestamp: now,
                    value: Value::Integer(value as i64),
                    unit: None,
                };
                process(&options, &mut datalog, nt.as_mut(), &entry);
            }
        }
    }
}

/// What a running logger owns besides the log thread.
struct Services {
    manager: Option<DataLogManager>,
    extras: Option<Extras>,
}

static GLOBAL: OnceLock<DogLog> = OnceLock::new();

pub struct DogLog {
    inner: Arc<Inner>,
    thread: Mutex<Option<JoinHandle<()>>>,
    services: Services,
}

impl DogLog {
    /// A logger over the given sinks. The log thread starts at once.
    pub fn with_sinks(
        options: DogLogOptions,
        datalog: Arc<DataLog>,
        nt: Box<dyn NtWriter>,
        clock: fn() -> u64,
    ) -> Self {
        Self::build(options, datalog, nt, clock, None)
    }

    fn build(
        options: DogLogOptions,
        datalog: Arc<DataLog>,
        nt: Box<dyn NtWriter>,
        clock: fn() -> u64,
        manager: Option<DataLogManager>,
    ) -> Self {
        let inner = Arc::new(Inner {
            queue: Mutex::new(Queue {
                items: VecDeque::new(),
                capacity: options.log_entry_queue_capacity,
                busy: false,
                shutdown: false,
            }),
            wake: Condvar::new(),
            idle: Condvar::new(),
            options: RwLock::new(Arc::new(options)),
            enabled: AtomicBool::new(true),
            clock,
            full_count: AtomicU32::new(0),
            faults: Mutex::new(Faults::default()),
        });
        let thread = thread::Builder::new()
            .name("DogLog LogThread".into())
            .spawn({
                let inner = inner.clone();
                let writer = DataLogWriter::new(datalog);
                move || log_thread(&inner, writer, nt)
            })
            .ok();
        let extras = manager.is_some().then(|| {
            let queue = inner.clone();
            Extras::new(Arc::new(move |entry| queue.enqueue(entry)))
        });
        Self {
            inner,
            thread: Mutex::new(thread),
            services: Services { manager, extras },
        }
    }

    /// The logger the robot uses: the managed log file and NetworkTables.
    pub fn start(options: DogLogOptions) -> Self {
        let manager = DataLogManager::start(wpilib::is_real());
        let unsafe_destination = data_log_manager::is_unsafe_destination(manager.log_dir());
        let log = Self::build(
            options.clone(),
            manager.log().clone(),
            Box::new(NetworkTablesWriter::new()),
            wpilib::robot_controller::fpga_time,
            Some(manager),
        );
        if unsafe_destination {
            log.log_fault("[DogLog] UNSAFE_LOG_DESTINATION");
        }
        log.apply_options(&options);
        log
    }

    /// The logger every static call uses. The first call starts it with default options.
    pub fn global() -> &'static Self {
        GLOBAL.get_or_init(|| {
            wpilib::shutdown::register(|| {
                if let Some(log) = GLOBAL.get() {
                    log.shutdown();
                }
            });
            Self::start(DogLogOptions::default())
        })
    }

    /// Stops the extras notifiers and the log and manager threads, after writing out what is
    /// queued. Later records are dropped. Safe to call twice.
    pub fn shutdown(&self) {
        if let Some(extras) = &self.services.extras {
            extras.set_enabled(false);
        }
        lock(&self.inner.queue).shutdown = true;
        self.inner.wake.notify_all();
        let thread = lock(&self.thread).take();
        if let Some(thread) = thread {
            let _ = thread.join();
        }
        if let Some(manager) = &self.services.manager {
            manager.shutdown();
        }
    }

    pub fn options(&self) -> Arc<DogLogOptions> {
        self.inner
            .options
            .read()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    pub fn set_options(&self, options: DogLogOptions) {
        println!("[DogLog] Options changed: {options:?}");
        lock(&self.inner.queue).capacity = options.log_entry_queue_capacity;
        self.apply_options(&options);
        let text = format!("{options:?}");
        *self
            .inner
            .options
            .write()
            .unwrap_or_else(PoisonError::into_inner) = Arc::new(options);
        self.inner
            .log_at(self.inner.now(), "DogLog/Options", Value::String(text));
    }

    /// Starts or stops what the options switch: console and driver station capture, extras.
    fn apply_options(&self, options: &DogLogOptions) {
        if options.capture_nt {
            eprintln!("[DogLog] captureNt is not supported and is ignored");
        }
        if let Some(manager) = &self.services.manager {
            manager.set_console_logging(options.capture_console);
            if options.capture_ds {
                manager.start_ds_capture();
            }
        }
        if let Some(extras) = &self.services.extras {
            extras.set_enabled(options.log_extras);
        }
    }

    pub fn set_pdh(&self, pdh: Option<PowerDistribution>) {
        if let Some(extras) = &self.services.extras {
            extras.set_pdh(pdh);
        }
    }

    pub fn set_enabled(&self, enabled: bool) {
        self.inner.enabled.store(enabled, Ordering::Relaxed);
    }

    pub fn is_enabled(&self) -> bool {
        self.inner.enabled.load(Ordering::Relaxed)
    }

    /// The managed log, when this logger owns one.
    pub fn data_log(&self) -> Option<&Arc<DataLog>> {
        self.services.manager.as_ref().map(DataLogManager::log)
    }

    /// Queues a finished value. The typed methods below call this.
    pub fn log_value(&self, key: &str, force_nt: bool, value: Value, unit: Option<&str>) {
        self.inner.log(key, force_nt, value, unit);
    }

    /// Logs a value to the wpilog.
    pub fn log(&self, key: &str, value: impl LogValue) {
        self.inner.log(key, false, value.into_value(), None);
    }

    /// Logs a number or number array with a unit written to the entry's metadata.
    pub fn log_with_unit(&self, key: &str, value: impl LogValue, unit: &str) {
        self.inner.log(key, false, value.into_value(), Some(unit));
    }

    /// Logs to the wpilog and NetworkTables, whatever `nt_publish` says. DogLog's `forceNt`.
    pub fn log_force_nt(&self, key: &str, value: impl LogValue) {
        self.inner.log(key, true, value.into_value(), None);
    }

    pub fn log_force_nt_with_unit(&self, key: &str, value: impl LogValue, unit: &str) {
        self.inner.log(key, true, value.into_value(), Some(unit));
    }

    pub fn log_struct<S: nt::Struct>(&self, key: &str, value: &S) {
        self.inner.log(key, false, struct_value(value), None);
    }

    pub fn log_struct_array<S: nt::Struct>(&self, key: &str, values: &[S]) {
        self.inner.log(key, false, struct_array_value(values), None);
    }

    /// Logs a custom type string, such as `json`, with a string payload.
    pub fn log_custom_string(&self, key: &str, value: &str, type_name: &str) {
        self.inner.log(
            key,
            false,
            Value::CustomString {
                value: value.to_owned(),
                type_name: type_name.to_owned(),
            },
            None,
        );
    }

    /// Counts a fault, adds it to `Faults/Active` and logs its count. DogLog also raises an
    /// alert; alerts here belong to the loop thread, so this does not.
    pub fn log_fault(&self, name: &str) {
        if self.is_enabled() {
            self.inner.add_fault(name);
        }
    }

    pub fn decrease_fault(&self, name: &str) {
        if self.is_enabled() {
            self.inner.decrease_fault(name);
        }
    }

    pub fn clear_fault(&self, name: &str) {
        if self.is_enabled() {
            self.inner.clear_fault(name);
        }
    }

    pub fn faults_logged(&self) -> bool {
        !lock(&self.inner.faults).seen.is_empty()
    }

    pub fn faults_active(&self) -> bool {
        !lock(&self.inner.faults).active.is_empty()
    }

    /// Blocks until every queued record is written to the log and NetworkTables, then writes
    /// the file buffer out.
    pub fn flush(&self) {
        let mut q = lock(&self.inner.queue);
        while !q.items.is_empty() || q.busy {
            q = self
                .inner
                .idle
                .wait(q)
                .unwrap_or_else(PoisonError::into_inner);
        }
        drop(q);
        if let Some(log) = self.data_log() {
            log.flush();
        }
    }

    /// A number NetworkTables can change at run time, at `/Tunable/<key>`. Reads the default
    /// while the `nt_tunables` option is false.
    pub fn tunable(&self, key: &str, default: f64, unit: Option<&str>) -> TunableDouble {
        let topic = Instance::default_instance()
            .table("/Tunable")
            .topic::<nt::Double>(key);
        let entry = topic.entry(default, &nt::Options::default());
        entry.set(default);
        if let Some(unit) = unit {
            crate::native::set_topic_unit(topic.handle(), unit);
        }
        TunableDouble {
            entry,
            default,
            inner: self.inner.clone(),
        }
    }
}

impl Drop for DogLog {
    fn drop(&mut self) {
        self.shutdown();
    }
}

pub struct TunableDouble {
    entry: nt::Entry<nt::Double>,
    default: f64,
    inner: Arc<Inner>,
}

impl TunableDouble {
    pub fn get(&self) -> f64 {
        let use_nt = {
            let options = self
                .inner
                .options
                .read()
                .unwrap_or_else(PoisonError::into_inner);
            (options.nt_tunables)()
        };
        if use_nt {
            self.entry.get()
        } else {
            self.default
        }
    }
}

/// An in-memory NetworkTables sink that remembers what was published. For tests.
#[cfg(test)]
pub(crate) mod testing {
    use super::*;
    use crate::datalog::Config;
    use std::sync::Mutex;

    #[derive(Clone, Default)]
    pub struct Recorder(pub Arc<Mutex<Vec<Entry>>>);

    impl NtWriter for Recorder {
        fn publish(&mut self, entry: &Entry) {
            self.0.lock().unwrap().push(entry.clone());
        }
    }

    pub fn fake_clock() -> u64 {
        1234
    }

    pub struct Harness {
        pub dog: DogLog,
        pub nt: Recorder,
        pub bytes: Arc<Mutex<Vec<u8>>>,
        log: Arc<DataLog>,
    }

    impl Harness {
        pub fn new(options: DogLogOptions) -> Self {
            let bytes = Arc::new(Mutex::new(Vec::new()));
            let sink = bytes.clone();
            let log = Arc::new(DataLog::new(Config::callback(move |b| {
                sink.lock().unwrap().extend_from_slice(b);
            })));
            let nt = Recorder::default();
            let dog = DogLog::with_sinks(options, log.clone(), Box::new(nt.clone()), fake_clock);
            Self {
                dog,
                nt,
                bytes,
                log,
            }
        }

        /// Everything the log holds once the queue drained and the file buffer was written.
        pub fn read(&self) -> crate::reader::Log {
            self.dog.flush();
            self.log.flush();
            let mut last = usize::MAX;
            for _ in 0..200 {
                std::thread::sleep(Duration::from_millis(15));
                let len = self.bytes.lock().unwrap().len();
                if len == last && len > 0 {
                    break;
                }
                last = len;
            }
            crate::reader::Log::parse(&self.bytes.lock().unwrap()).unwrap()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::testing::*;
    use super::*;
    use crate::reader::decode;
    use wpimath::geometry::{Pose2d, Rotation2d};

    fn options() -> DogLogOptions {
        DogLogOptions::default()
            .with_nt_publish(false)
            .with_log_extras(false)
    }

    #[test]
    fn shutdown_is_idempotent_and_later_records_are_dropped() {
        let h = Harness::new(options());
        h.dog.log("before", 1_i64);
        h.dog.shutdown();
        h.dog.shutdown();
        h.dog.log("after", 2_i64);
        h.dog.flush();
        let log = h.read();
        assert!(log.entry("/Robot/before").is_some());
        assert!(log.entry("/Robot/after").is_none());
    }

    #[test]
    fn logs_every_type_under_the_robot_table() {
        let h = Harness::new(options());
        let d = &h.dog;
        d.log("bool", true);
        d.log("double", 1.5);
        d.log("int", 7i32);
        d.log("long", -9i64);
        d.log("string", "hi");
        d.log("doubles", &[1.0, 2.0]);
        d.log("bools", &[true, false]);
        d.log("strings", &["a", "b"]);
        d.log_with_unit("volts", 12.5, "Volt");
        d.log_struct("pose", &Pose2d::from_xy(1.0, 2.0, Rotation2d::new(0.5)));
        d.log_custom_string("json", "{}", "json");
        let log = h.read();
        let get = |k: &str| log.entry(&format!("/Robot/{k}")).unwrap();
        assert_eq!(decode::boolean(&get("bool").records[0].1), Some(true));
        assert_eq!(
            get("bool").records[0].0,
            1234,
            "timestamp comes from the clock"
        );
        assert_eq!(decode::double(&get("double").records[0].1), Some(1.5));
        assert_eq!(decode::int64(&get("int").records[0].1), Some(7));
        assert_eq!(decode::int64(&get("long").records[0].1), Some(-9));
        assert_eq!(decode::string(&get("string").records[0].1), "hi");
        assert_eq!(
            decode::double_array(&get("doubles").records[0].1),
            Some(vec![1.0, 2.0])
        );
        assert_eq!(
            decode::boolean_array(&get("bools").records[0].1),
            vec![true, false]
        );
        assert_eq!(
            decode::string_array(&get("strings").records[0].1),
            Some(vec!["a".to_owned(), "b".to_owned()])
        );
        assert_eq!(
            get("volts").metadata,
            "{\"source\":\"DogLog\",\"unit\":\"Volt\"}"
        );
        assert_eq!(get("pose").type_name, "struct:Pose2d");
        assert_eq!(get("json").type_name, "json");
        assert!(log.entry("/.schema/struct:Pose2d").is_some());
    }

    #[test]
    fn nt_gets_records_only_when_allowed_or_forced() {
        let h = Harness::new(options());
        h.dog.log("quiet", 1.0);
        h.dog.log_force_nt("loud", 2.0);
        h.dog.log_force_nt_with_unit("loud_unit", 3.0, "Volt");
        h.dog.flush();
        let published: Vec<_> =
            h.nt.0
                .lock()
                .unwrap()
                .iter()
                .map(|e| e.key.clone())
                .collect();
        assert_eq!(published, ["loud", "loud_unit"]);
        assert_eq!(h.nt.0.lock().unwrap()[1].unit.as_deref(), Some("Volt"));

        let h = Harness::new(options().with_nt_publish(true));
        h.dog.log("now", 1.0);
        h.dog.flush();
        assert!(h.nt.0.lock().unwrap().iter().any(|e| e.key == "now"));
    }

    #[test]
    fn nt_publish_is_checked_per_record() {
        let allowed = Arc::new(AtomicBool::new(false));
        let flag = allowed.clone();
        let h = Harness::new(options().with_nt_publish_fn(move || flag.load(Ordering::Relaxed)));
        h.dog.log("a", 1.0);
        h.dog.flush();
        allowed.store(true, Ordering::Relaxed);
        h.dog.log("b", 1.0);
        h.dog.flush();
        let keys: Vec<_> =
            h.nt.0
                .lock()
                .unwrap()
                .iter()
                .map(|e| e.key.clone())
                .collect();
        assert_eq!(keys, ["b"]);
    }

    #[test]
    fn disabled_logger_ignores_calls() {
        let h = Harness::new(options());
        h.dog.set_enabled(false);
        assert!(!h.dog.is_enabled());
        h.dog.log("x", 1.0);
        h.dog.set_enabled(true);
        h.dog.log("y", 1.0);
        let log = h.read();
        assert!(log.entry("/Robot/x").is_none());
        assert!(log.entry("/Robot/y").is_some());
    }

    #[test]
    fn many_threads_lose_no_records() {
        let h = Arc::new(Harness::new(
            options().with_log_entry_queue_capacity(1_000_000),
        ));
        const THREADS: usize = 8;
        const PER_THREAD: usize = 500;
        let handles: Vec<_> = (0..THREADS)
            .map(|t| {
                let h = h.clone();
                std::thread::spawn(move || {
                    for i in 0..PER_THREAD {
                        h.dog.log(&format!("thread{t}"), i as i64);
                        h.dog.log("shared", (t * PER_THREAD + i) as i64);
                    }
                })
            })
            .collect();
        for handle in handles {
            handle.join().unwrap();
        }
        let log = h.read();
        for t in 0..THREADS {
            let entry = log.entry(&format!("/Robot/thread{t}")).unwrap();
            let values: Vec<i64> = entry
                .records
                .iter()
                .map(|r| decode::int64(&r.1).unwrap())
                .collect();
            let want: Vec<i64> = (0..PER_THREAD as i64).collect();
            assert_eq!(values, want, "thread {t}: records arrive once, in order");
        }
        let shared = log.entry("/Robot/shared").unwrap();
        assert_eq!(shared.records.len(), THREADS * PER_THREAD);
        let mut seen: Vec<i64> = shared
            .records
            .iter()
            .map(|r| decode::int64(&r.1).unwrap())
            .collect();
        seen.sort_unstable();
        assert_eq!(seen, (0..(THREADS * PER_THREAD) as i64).collect::<Vec<_>>());
    }

    #[test]
    fn a_full_queue_drops_records_and_logs_one_fault() {
        let h = Harness::new(options().with_log_entry_queue_capacity(4));
        lock(&h.dog.inner.queue).capacity = 0;
        for i in 0..10 {
            h.dog.log("k", i64::from(i));
        }
        assert!(h.dog.faults_active());
        // Ten dropped calls, and the three records the first drop's fault tried to queue.
        assert_eq!(h.dog.inner.full_count.load(Ordering::Relaxed), 13);
        lock(&h.dog.inner.queue).capacity = 100;
        let log = h.read();
        assert!(log.entry("/Robot/k").is_none());
    }

    #[test]
    fn faults_count_and_clear() {
        let h = Harness::new(options());
        h.dog.log_fault("A");
        h.dog.log_fault("A");
        h.dog.log_fault("B");
        assert!(h.dog.faults_logged() && h.dog.faults_active());
        h.dog.decrease_fault("A");
        h.dog.decrease_fault("never");
        h.dog.clear_fault("A");
        h.dog.clear_fault("never seen");
        let log = h.read();
        let counts = |name: &str| -> Vec<i64> {
            log.entry(&format!("/Robot/Faults/Counts/{name}"))
                .unwrap()
                .records
                .iter()
                .map(|r| decode::int64(&r.1).unwrap())
                .collect()
        };
        assert_eq!(counts("A"), [1, 2, 1, 0]);
        assert_eq!(counts("B"), [1]);
        let active = log.entry("/Robot/Faults/Active").unwrap();
        let last = decode::string_array(&active.records.last().unwrap().1).unwrap();
        assert_eq!(last, ["B"]);
        assert!(log.entry("/Robot/Faults/Counts/never seen").is_none());
        let seen = log.entry("/Robot/Faults/Seen").unwrap();
        assert_eq!(
            decode::string_array(&seen.records.last().unwrap().1).unwrap(),
            ["A", "B"]
        );
    }

    #[test]
    fn set_options_logs_them_and_resizes_the_queue() {
        let h = Harness::new(options());
        h.dog
            .set_options(options().with_log_entry_queue_capacity(77));
        assert_eq!(h.dog.options().log_entry_queue_capacity, 77);
        assert_eq!(lock(&h.dog.inner.queue).capacity, 77);
        let log = h.read();
        let text = decode::string(&log.entry("/Robot/DogLog/Options").unwrap().records[0].1);
        assert!(text.contains("logEntryQueueCapacity: 77"), "{text}");
    }

    #[test]
    fn queue_diagnostics_are_logged() {
        let h = Harness::new(options());
        h.dog.log("a", 1.0);
        std::thread::sleep(Duration::from_millis(30));
        h.dog.log("b", 1.0);
        let log = h.read();
        assert!(log.entry("/Robot/DogLog/QueuedLogs").is_some());
        assert!(log.entry("/Robot/DogLog/QueueRemainingCapacity").is_some());
    }

    #[test]
    fn default_options_match_dog_log() {
        let o = DogLogOptions::default();
        assert!(!o.capture_nt && !o.capture_ds);
        assert!(o.log_extras && o.capture_console);
        assert_eq!(o.log_entry_queue_capacity, 1000);
    }
}
