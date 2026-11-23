//! A wpilog writer with a background flush thread: WPILib's `DataLog` and
//! `DataLogBackgroundWriter` in Rust.
//!
//! Callers append records from any thread into one buffer. The writer thread swaps the buffer
//! out every `period`, writes it and syncs the file.

use crate::wpilog;
use std::collections::HashMap;
use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// Writing stops below this much free space.
const MIN_FREE_SPACE: u64 = 5 * 1024 * 1024;
/// Appends are dropped while this much is waiting for the writer.
const MAX_BUFFER: usize = 1024 * 1024;
/// The writer is woken early at this much.
const HALF_BUFFER: usize = MAX_BUFFER / 2;
/// A new file starts after this many bytes.
const DEFAULT_MAX_FILE_BYTES: u64 = 1_800_000_000;

/// Receives each flushed block of the log.
pub type WriteBlock = Box<dyn FnMut(&[u8]) + Send>;

pub enum Sink {
    /// A file in `dir`. `filename` may be empty to pick a random name.
    Directory { dir: PathBuf, filename: String },
    /// Receives each flushed block. The first block starts with the file header.
    Callback(WriteBlock),
}

pub struct Config {
    pub sink: Sink,
    pub period: Duration,
    pub extra_header: String,
    /// Microsecond clock used for the start records re-emitted when a file restarts.
    pub clock: fn() -> u64,
    pub max_file_bytes: u64,
}

impl Config {
    pub fn directory(dir: impl Into<PathBuf>, filename: &str) -> Self {
        Self::new(Sink::Directory {
            dir: dir.into(),
            filename: filename.to_owned(),
        })
    }

    pub fn callback(write: impl FnMut(&[u8]) + Send + 'static) -> Self {
        Self::new(Sink::Callback(Box::new(write)))
    }

    fn new(sink: Sink) -> Self {
        Self {
            sink,
            period: Duration::from_millis(250),
            extra_header: String::new(),
            clock: monotonic_micros,
            max_file_bytes: DEFAULT_MAX_FILE_BYTES,
        }
    }
}

/// Microseconds since the first call.
pub fn monotonic_micros() -> u64 {
    use std::sync::OnceLock;
    use std::time::Instant;
    static START: OnceLock<Instant> = OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_micros() as u64
}

struct NameInfo {
    id: u32,
    type_name: String,
    /// Schema text of `/.schema/*` entries, re-sent when a file restarts.
    schema_data: Vec<u8>,
}

struct IdInfo {
    name: String,
    count: u32,
    metadata: String,
}

struct State {
    names: HashMap<String, NameInfo>,
    ids: HashMap<u32, IdInfo>,
    last_id: u32,
    outgoing: Vec<u8>,
    new_filename: Option<String>,
    do_flush: bool,
    shutdown: bool,
    dropped: u64,
    paused: bool,
    path: Option<PathBuf>,
    extra_header: String,
}

struct Shared {
    state: Mutex<State>,
    cond: Condvar,
}

impl Shared {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

impl State {
    fn append_start(&mut self, id: u32, name: &str, type_name: &str, metadata: &str, ts: u64) {
        wpilog::write_start(&mut self.outgoing, id, name, type_name, metadata, ts);
    }

    /// File header, then a start record for every live entry and the data of every schema.
    fn restart_prefix(&self, now: u64) -> Vec<u8> {
        let mut out = Vec::new();
        wpilog::write_header(&mut out, &self.extra_header);
        let mut ids: Vec<_> = self.ids.iter().collect();
        ids.sort_by_key(|(id, _)| **id);
        for (&id, info) in ids {
            let type_name = self
                .names
                .get(&info.name)
                .map_or("", |n| n.type_name.as_str());
            wpilog::write_start(&mut out, id, &info.name, type_name, &info.metadata, now);
            if let Some(n) = self.names.get(&info.name)
                && !n.schema_data.is_empty()
            {
                wpilog::write_record(&mut out, id, now, &n.schema_data);
            }
        }
        out
    }
}

/// A log being written. Methods take `&self` and work from any thread.
pub struct DataLog {
    shared: Arc<Shared>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl DataLog {
    /// Starts the writer thread.
    pub fn new(config: Config) -> Self {
        let mut outgoing = Vec::new();
        wpilog::write_header(&mut outgoing, &config.extra_header);
        let new_filename = match &config.sink {
            Sink::Directory { filename, .. } if !filename.is_empty() => Some(filename.clone()),
            _ => None,
        };
        let shared = Arc::new(Shared {
            state: Mutex::new(State {
                names: HashMap::new(),
                ids: HashMap::new(),
                last_id: 0,
                outgoing,
                new_filename,
                do_flush: false,
                shutdown: false,
                dropped: 0,
                paused: false,
                path: None,
                extra_header: config.extra_header.clone(),
            }),
            cond: Condvar::new(),
        });
        let thread = thread::Builder::new()
            .name("DataLog writer".into())
            .spawn({
                let shared = shared.clone();
                move || writer_main(&shared, config)
            })
            .ok();
        Self {
            shared,
            thread: Mutex::new(thread),
        }
    }

    /// Starts an entry. A repeat of a name and type returns the same id. `None` when the name
    /// is already in use with another type.
    pub fn start(&self, name: &str, type_name: &str, metadata: &str, ts: u64) -> Option<u32> {
        let mut s = self.shared.lock();
        start_locked(&mut s, name, type_name, metadata, ts)
    }

    /// Ends an entry once every start of it has been finished.
    pub fn finish(&self, id: u32, ts: u64) {
        let mut s = self.shared.lock();
        let Some(info) = s.ids.get_mut(&id) else {
            return;
        };
        info.count -= 1;
        if info.count > 0 {
            return;
        }
        if let Some(info) = s.ids.remove(&id) {
            s.names.remove(&info.name);
        }
        let mut out = std::mem::take(&mut s.outgoing);
        wpilog::write_finish(&mut out, id, ts);
        s.outgoing = out;
    }

    pub fn set_metadata(&self, id: u32, metadata: &str, ts: u64) {
        let mut s = self.shared.lock();
        let Some(info) = s.ids.get_mut(&id) else {
            return;
        };
        metadata.clone_into(&mut info.metadata);
        let mut out = std::mem::take(&mut s.outgoing);
        wpilog::write_set_metadata(&mut out, id, metadata, ts);
        s.outgoing = out;
    }

    /// Appends one record. Ignored for an unknown id.
    pub fn append(&self, id: u32, ts: u64, payload: &[u8]) {
        let mut s = self.shared.lock();
        if !s.ids.contains_key(&id) {
            return;
        }
        append_locked(&self.shared, &mut s, id, ts, payload);
    }

    pub fn has_schema(&self, name: &str) -> bool {
        self.shared.lock().names.contains_key(&schema_entry(name))
    }

    /// Adds `/.schema/<name>` once. `type_name` is `structschema` for struct schemas.
    pub fn add_schema(&self, name: &str, type_name: &str, schema: &[u8], ts: u64) {
        let mut s = self.shared.lock();
        let full = schema_entry(name);
        if s.names.contains_key(&full) {
            return;
        }
        let Some(id) = start_locked(&mut s, &full, type_name, "", ts) else {
            return;
        };
        if let Some(n) = s.names.get_mut(&full) {
            n.schema_data = schema.to_vec();
        }
        append_locked(&self.shared, &mut s, id, ts, schema);
    }

    /// Renames the file once it is open.
    pub fn set_filename(&self, filename: &str) {
        self.shared.lock().new_filename = Some(filename.to_owned());
        self.shared.cond.notify_all();
    }

    /// Writes everything appended so far at the next wakeup of the writer.
    pub fn flush(&self) {
        self.shared.lock().do_flush = true;
        self.shared.cond.notify_all();
    }

    /// While paused, data records are discarded. Control records still go through.
    pub fn set_paused(&self, paused: bool) {
        self.shared.lock().paused = paused;
    }

    /// Records dropped because the writer fell behind.
    pub fn dropped(&self) -> u64 {
        self.shared.lock().dropped
    }

    /// The file being written, once the writer opened it.
    pub fn path(&self) -> Option<PathBuf> {
        self.shared.lock().path.clone()
    }
}

impl DataLog {
    /// Writes out what is buffered, then stops and joins the writer thread. Records appended
    /// afterwards stay in memory. Safe to call twice.
    pub fn shutdown(&self) {
        {
            let mut s = self.shared.lock();
            s.shutdown = true;
            s.do_flush = true;
        }
        self.shared.cond.notify_all();
        let thread = self
            .thread
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        if let Some(thread) = thread {
            let _ = thread.join();
        }
    }
}

impl Drop for DataLog {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn schema_entry(name: &str) -> String {
    format!("/.schema/{name}")
}

fn start_locked(
    s: &mut State,
    name: &str,
    type_name: &str,
    metadata: &str,
    ts: u64,
) -> Option<u32> {
    if let Some(existing) = s.names.get(name) {
        if existing.type_name != type_name {
            eprintln!(
                "DataLog: type mismatch for '{name}': was '{}', requested '{type_name}'; ignoring",
                existing.type_name
            );
            return None;
        }
        let id = existing.id;
        if let Some(info) = s.ids.get_mut(&id) {
            info.count += 1;
        }
        return Some(id);
    }
    s.last_id += 1;
    let id = s.last_id;
    s.names.insert(
        name.to_owned(),
        NameInfo {
            id,
            type_name: type_name.to_owned(),
            schema_data: Vec::new(),
        },
    );
    s.ids.insert(
        id,
        IdInfo {
            name: name.to_owned(),
            count: 1,
            metadata: metadata.to_owned(),
        },
    );
    s.append_start(id, name, type_name, metadata, ts);
    Some(id)
}

fn append_locked(shared: &Shared, s: &mut State, id: u32, ts: u64, payload: &[u8]) {
    if s.paused {
        return;
    }
    if s.outgoing.len() + payload.len() + wpilog::MAX_RECORD_HEADER > MAX_BUFFER {
        s.dropped += 1;
        return;
    }
    wpilog::write_record(&mut s.outgoing, id, ts, payload);
    if s.outgoing.len() >= HALF_BUFFER && !s.do_flush {
        s.do_flush = true;
        shared.cond.notify_all();
    }
}

/// Free bytes for unprivileged writers on the filesystem of `path`. `None` where unknown.
pub fn free_space(path: &Path) -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        use std::ffi::CString;
        use std::os::raw::{c_char, c_int};
        use std::os::unix::ffi::OsStrExt;

        // glibc `struct statvfs64` is 8 byte aligned in its block counts on 32 and 64 bit.
        #[repr(C)]
        struct StatVfs64 {
            raw: [u64; 16],
        }
        unsafe extern "C" {
            fn statvfs64(path: *const c_char, buf: *mut StatVfs64) -> c_int;
        }
        let c_path = CString::new(path.as_os_str().as_bytes()).ok()?;
        let mut buf = StatVfs64 { raw: [0; 16] };
        // SAFETY: c_path is NUL-terminated; buf is larger than glibc's statvfs64 (112 bytes).
        if unsafe { statvfs64(c_path.as_ptr(), &mut buf) } != 0 {
            return None;
        }
        let raw = &buf.raw;
        let (frsize, avail) = if cfg!(target_pointer_width = "64") {
            (raw[1], raw[4])
        } else {
            // 32 bit: two 4 byte longs, then the 8 byte counts
            (raw[0] >> 32, raw[3])
        };
        Some(avail.saturating_mul(frsize))
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = path;
        None
    }
}

/// Random bits from the standard library's per-process hasher keys.
pub(crate) fn random_bits() -> u64 {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    RandomState::new().build_hasher().finish()
}

fn random_filename() -> String {
    let mut name = String::from("wpilog_");
    for _ in 0..2 {
        name.push_str(&format!("{:016x}", random_bits())[..8]);
    }
    name.push_str(".wpilog");
    name
}

/// `base.wpilog` becomes `base.2.wpilog`.
fn numbered(base: &str, count: u32) -> String {
    let path = Path::new(base);
    let stem = path
        .file_stem()
        .map_or(String::new(), |s| s.to_string_lossy().into_owned());
    match path.extension() {
        Some(ext) => format!("{stem}.{count}.{}", ext.to_string_lossy()),
        None => format!("{stem}.{count}"),
    }
}

struct FileState {
    dir: PathBuf,
    base_filename: String,
    filename: String,
    path: PathBuf,
    file: Option<File>,
    free_space: u64,
    segment: u32,
}

impl FileState {
    fn set_filename(&mut self, name: &str) {
        name.clone_into(&mut self.base_filename);
        name.clone_into(&mut self.filename);
        self.path = self.dir.join(name);
        self.segment = 1;
    }

    fn increment_filename(&mut self) {
        self.segment += 1;
        self.filename = numbered(&self.base_filename, self.segment);
        self.path = self.dir.join(&self.filename);
    }

    fn refresh_free_space(&mut self) {
        self.free_space = free_space(&self.dir).unwrap_or(u64::MAX);
    }
}

fn open_new(path: &Path) -> std::io::Result<File> {
    OpenOptions::new().append(true).create_new(true).open(path)
}

/// Opens the file and writes `prefix` to it. Tries a few random names when the first is taken.
fn start_log_file(shared: &Shared, f: &mut FileState, prefix: &[u8]) {
    if f.filename.is_empty() {
        let name = random_filename();
        f.set_filename(&name);
    }
    f.refresh_free_space();
    if f.free_space < MIN_FREE_SPACE {
        eprintln!(
            "DataLog: insufficient free space ({} bytes available), no log being saved",
            f.free_space
        );
        return;
    }
    for _ in 0..5 {
        match open_new(&f.path) {
            Ok(file) => {
                f.file = Some(file);
                break;
            }
            Err(e) => {
                eprintln!(
                    "DataLog: could not open log file '{}': {e}",
                    f.path.display()
                );
                let name = random_filename();
                f.set_filename(&name);
            }
        }
    }
    let Some(file) = f.file.as_mut() else {
        eprintln!("DataLog: could not open log file, no log being saved");
        return;
    };
    println!(
        "DataLog: logging to '{}' ({} bytes free space)",
        f.path.display(),
        f.free_space
    );
    shared.lock().path = Some(f.path.clone());
    if let Err(e) = file.write_all(prefix) {
        eprintln!("DataLog: error writing to log file '{}': {e}", f.filename);
    }
}

fn writer_main(shared: &Shared, config: Config) {
    let Config {
        sink,
        period,
        clock,
        max_file_bytes,
        ..
    } = config;
    match sink {
        Sink::Callback(write) => callback_loop(shared, period, write),
        Sink::Directory { dir, .. } => file_loop(shared, period, clock, max_file_bytes, dir),
    }
}

/// Waits one period, or less when asked to flush. Returns the pending buffer.
fn wait_and_take(shared: &Shared, period: Duration) -> (Vec<u8>, bool, Option<String>) {
    let mut s = shared.lock();
    if !s.do_flush && !s.shutdown {
        s = shared
            .cond
            .wait_timeout(s, period)
            .unwrap_or_else(PoisonError::into_inner)
            .0;
    }
    s.do_flush = false;
    let rename = s.new_filename.take();
    (std::mem::take(&mut s.outgoing), s.shutdown, rename)
}

fn callback_loop(shared: &Shared, period: Duration, mut write: WriteBlock) {
    loop {
        let (buf, shutdown, _) = wait_and_take(shared, period);
        if !buf.is_empty() {
            write(&buf);
        }
        if shutdown {
            return;
        }
    }
}

fn file_loop(shared: &Shared, period: Duration, clock: fn() -> u64, max_bytes: u64, dir: PathBuf) {
    let mut f = FileState {
        dir,
        base_filename: String::new(),
        filename: String::new(),
        path: PathBuf::new(),
        file: None,
        free_space: u64::MAX,
        segment: 1,
    };
    let (initial_name, prefix) = {
        let mut s = shared.lock();
        (s.new_filename.take(), std::mem::take(&mut s.outgoing))
    };
    if let Some(name) = initial_name {
        f.set_filename(&name);
    }
    // The prefix is the header plus whatever was appended before the thread started.
    start_log_file(shared, &mut f, &prefix);

    let mut flush_count = 0u32;
    let mut loop_count = 0u32;
    let mut written = prefix.len() as u64;
    let mut blocked = false;
    loop {
        let (buf, shutdown, rename) = wait_and_take(shared, period);

        loop_count += 1;
        let mut restart = false;
        if loop_count >= 10 {
            loop_count = 0;
            if f.file.is_some() && !f.path.exists() {
                f.file = None;
                f.increment_filename();
                println!(
                    "DataLog: log file deleted, recreating as fresh log '{}'",
                    f.filename
                );
                restart = true;
            }
        }
        if written > max_bytes {
            f.file = None;
            f.increment_filename();
            println!(
                "DataLog: log file reached its size limit, starting new file '{}'",
                f.filename
            );
            restart = true;
        }
        if restart {
            let prefix = shared.lock().restart_prefix(clock());
            start_log_file(shared, &mut f, &prefix);
            written = prefix.len() as u64;
            blocked = false;
        }

        if let Some(new_name) = rename
            && f.file.is_some()
            && !new_name.is_empty()
        {
            if f.filename != new_name {
                match fs::rename(&f.path, f.dir.join(&new_name)) {
                    Ok(()) => println!(
                        "DataLog: renamed log file from '{}' to '{new_name}'",
                        f.filename
                    ),
                    Err(e) => eprintln!(
                        "DataLog: could not rename log file from '{}' to '{new_name}': {e}",
                        f.filename
                    ),
                }
            }
            f.set_filename(&new_name);
            shared.lock().path = Some(f.path.clone());
        }

        if !buf.is_empty() && f.file.is_some() && !blocked {
            flush_count += 1;
            if flush_count >= 10 {
                flush_count = 0;
                f.refresh_free_space();
            }
            f.free_space = f.free_space.saturating_sub(buf.len() as u64);
            if f.free_space < MIN_FREE_SPACE {
                eprintln!(
                    "DataLog: stopped logging due to low free space ({} bytes available)",
                    f.free_space
                );
                blocked = true;
            } else if let Some(file) = f.file.as_mut() {
                written += buf.len() as u64;
                if let Err(e) = file.write_all(&buf) {
                    eprintln!("DataLog: error writing to log file '{}': {e}", f.filename);
                }
                let _ = file.sync_data();
            }
        }
        if shutdown {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reader::{Control, Log, Reader};
    use crate::wpilog::payload;

    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            let dir = std::env::temp_dir().join(format!("telemetry-{name}-{}", std::process::id()));
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

    fn memory_log() -> (DataLog, Arc<Mutex<Vec<u8>>>) {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let sink = bytes.clone();
        let log = DataLog::new(Config::callback(move |b| {
            sink.lock().unwrap().extend_from_slice(b);
        }));
        (log, bytes)
    }

    fn double(v: f64) -> Vec<u8> {
        let mut p = Vec::new();
        payload::double(&mut p, v);
        p
    }

    #[test]
    fn shutdown_writes_the_buffer_and_is_idempotent() {
        let (log, bytes) = memory_log();
        let id = log.start("a", "int64", "", 1).unwrap();
        log.append(id, 2, &[1, 0, 0, 0, 0, 0, 0, 0]);
        log.shutdown();
        log.shutdown();
        let written = bytes.lock().unwrap().clone();
        assert!(
            crate::reader::Log::parse(&written)
                .unwrap()
                .entry("a")
                .is_some()
        );
    }

    #[test]
    fn writes_header_start_and_records() {
        let (log, bytes) = memory_log();
        let id = log.start("test", "double", "{}", 5).unwrap();
        log.append(id, 6, &double(1.5));
        drop(log);
        let data = bytes.lock().unwrap().clone();
        assert_eq!(&data[..12], b"WPILOG\x00\x01\x00\x00\x00\x00");
        let parsed = Log::parse(&data).unwrap();
        let entry = parsed.entry("test").unwrap();
        assert_eq!(entry.id, 1);
        assert_eq!(entry.type_name, "double");
        assert_eq!(entry.records, [(6, double(1.5))]);
    }

    #[test]
    fn repeated_start_shares_the_id_and_mismatch_is_refused() {
        let (log, _bytes) = memory_log();
        let a = log.start("x", "double", "", 1).unwrap();
        assert_eq!(log.start("x", "double", "", 2), Some(a));
        assert_eq!(log.start("x", "string", "", 3), None);
        assert_ne!(log.start("y", "double", "", 4), Some(a));
    }

    #[test]
    fn finish_is_reference_counted_and_set_metadata_replaces() {
        let (log, bytes) = memory_log();
        let id = log.start("x", "double", "a", 1).unwrap();
        log.start("x", "double", "a", 1);
        log.set_metadata(id, "b", 2);
        log.set_metadata(id, "b", 3);
        log.finish(id, 4);
        log.append(id, 5, &double(1.0));
        log.finish(id, 6);
        log.append(id, 7, &double(2.0));
        drop(log);
        let data = bytes.lock().unwrap().clone();
        let controls: Vec<_> = Reader::new(&data)
            .unwrap()
            .filter_map(|r| r.unwrap().control())
            .collect();
        assert_eq!(controls.len(), 4);
        assert!(matches!(controls[1], Control::SetMetadata { entry: 1, .. }));
        assert_eq!(controls[3], Control::Finish { entry: 1 });
        let parsed = Log::parse(&data).unwrap();
        let entry = parsed.entry("x").unwrap();
        assert_eq!(entry.metadata, "b");
        assert!(entry.finished);
        assert_eq!(entry.records.len(), 1);
    }

    #[test]
    fn schemas_are_added_once_as_entries() {
        let (log, bytes) = memory_log();
        assert!(!log.has_schema("struct:A"));
        log.add_schema("struct:A", "structschema", b"double x", 9);
        log.add_schema("struct:A", "structschema", b"double y", 9);
        assert!(log.has_schema("struct:A"));
        drop(log);
        let parsed = Log::parse(&bytes.lock().unwrap()).unwrap();
        let entry = parsed.entry("/.schema/struct:A").unwrap();
        assert_eq!(entry.type_name, "structschema");
        assert_eq!(entry.records, [(9, b"double x".to_vec())]);
    }

    #[test]
    fn paused_log_drops_data_and_keeps_control_records() {
        let (log, bytes) = memory_log();
        let id = log.start("x", "double", "", 1).unwrap();
        log.set_paused(true);
        log.append(id, 2, &double(1.0));
        log.start("y", "double", "", 3).unwrap();
        log.set_paused(false);
        log.append(id, 4, &double(2.0));
        drop(log);
        let parsed = Log::parse(&bytes.lock().unwrap()).unwrap();
        assert_eq!(parsed.entry("x").unwrap().records, [(4, double(2.0))]);
        assert!(parsed.entry("y").is_some());
    }

    #[test]
    fn full_buffer_drops_and_counts() {
        let (log, _bytes) = memory_log();
        let id = log.start("big", "raw", "", 1).unwrap();
        let chunk = vec![0u8; 300 * 1024];
        for _ in 0..5 {
            log.append(id, 2, &chunk);
        }
        assert!(log.dropped() >= 1);
    }

    #[test]
    fn writes_a_file_and_renames_it() {
        let dir = TempDir::new("file");
        let log = DataLog::new(Config::directory(&dir.0, "first.wpilog"));
        let id = log.start("v", "double", "", 1).unwrap();
        log.append(id, 2, &double(4.0));
        log.flush();
        let first = loop {
            thread::sleep(Duration::from_millis(10));
            if let Some(p) = log.path()
                && fs::metadata(&p).is_ok_and(|m| m.len() > 12)
            {
                break p;
            }
        };
        assert_eq!(first, dir.0.join("first.wpilog"));
        log.set_filename("second.wpilog");
        while log.path() == Some(first.clone()) {
            thread::sleep(Duration::from_millis(10));
        }
        log.append(id, 3, &double(5.0));
        drop(log);
        assert!(!first.exists());
        let parsed = Log::parse(&fs::read(dir.0.join("second.wpilog")).unwrap()).unwrap();
        assert_eq!(parsed.entry("v").unwrap().records.len(), 2);
    }

    #[test]
    fn random_name_when_none_given() {
        let dir = TempDir::new("random");
        let log = DataLog::new(Config::directory(&dir.0, ""));
        let id = log.start("v", "double", "", 1).unwrap();
        log.append(id, 2, &double(1.0));
        drop(log);
        let names: Vec<_> = fs::read_dir(&dir.0)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        assert_eq!(names.len(), 1);
        assert!(names[0].starts_with("wpilog_") && names[0].ends_with(".wpilog"));
    }

    #[test]
    fn deleted_file_restarts_with_every_entry() {
        let dir = TempDir::new("deleted");
        let mut config = Config::directory(&dir.0, "a.wpilog");
        config.period = Duration::from_millis(5);
        let log = DataLog::new(config);
        let id = log.start("v", "double", "{\"k\":1}", 1).unwrap();
        log.add_schema("struct:S", "structschema", b"double x", 1);
        log.append(id, 2, &double(1.0));
        log.flush();
        let path = loop {
            thread::sleep(Duration::from_millis(10));
            if let Some(p) = log.path()
                && p.exists()
            {
                break p;
            }
        };
        thread::sleep(Duration::from_millis(30));
        fs::remove_file(&path).unwrap();
        while !dir.0.join("a.2.wpilog").exists() {
            log.append(id, 3, &double(2.0));
            thread::sleep(Duration::from_millis(10));
        }
        drop(log);
        let parsed = Log::parse(&fs::read(dir.0.join("a.2.wpilog")).unwrap()).unwrap();
        assert_eq!(parsed.entry("v").unwrap().metadata, "{\"k\":1}");
        assert_eq!(
            parsed.entry("/.schema/struct:S").unwrap().records[0].1,
            b"double x"
        );
    }

    #[test]
    fn size_limit_starts_a_new_file() {
        let dir = TempDir::new("limit");
        let mut config = Config::directory(&dir.0, "b.wpilog");
        config.period = Duration::from_millis(5);
        config.max_file_bytes = 200;
        let log = DataLog::new(config);
        let id = log.start("v", "raw", "", 1).unwrap();
        for _ in 0..3 {
            log.append(id, 2, &[7u8; 300]);
            log.flush();
            thread::sleep(Duration::from_millis(40));
        }
        drop(log);
        let second = Log::parse(&fs::read(dir.0.join("b.2.wpilog")).unwrap()).unwrap();
        assert_eq!(second.entry("v").unwrap().type_name, "raw");
        assert!(dir.0.join("b.wpilog").exists());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn free_space_matches_df() {
        let dir = TempDir::new("df");
        let Ok(out) = std::process::Command::new("df")
            .args(["-B1", "--output=avail"])
            .arg(&dir.0)
            .output()
        else {
            return;
        };
        let text = String::from_utf8_lossy(&out.stdout);
        let Some(want) = text
            .lines()
            .nth(1)
            .and_then(|l| l.trim().parse::<u64>().ok())
        else {
            return;
        };
        let got = free_space(&dir.0).unwrap();
        assert!(got.abs_diff(want) < 64 * 1024 * 1024, "{got} vs {want}");
        assert!(free_space(&dir.0.join("missing")).is_none());
    }

    #[test]
    fn numbered_names() {
        assert_eq!(numbered("a.wpilog", 2), "a.2.wpilog");
        assert_eq!(numbered("a", 3), "a.3");
    }
}
