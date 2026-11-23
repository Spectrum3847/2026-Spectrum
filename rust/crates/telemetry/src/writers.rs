//! The two places DogLog sends a record: the wpilog and NetworkTables. Both run on the log
//! thread only, so they hold plain maps.

use crate::datalog::DataLog;
use crate::value::{Entry, Value};
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use wpilib::nt::ffi::{Handle, WpiStr, core};
use wpilib::nt::{self, Instance};

/// Every DogLog key lives under this table.
pub const LOG_TABLE: &str = "/Robot";

const ENTRY_METADATA: &str = "{\"source\":\"DogLog\"}";

/// Schemas in the order WPILib registers them: the types a struct contains before the struct,
/// each type once. `list` has a type before the ones it contains, as `Struct::add_schemas`
/// reports them.
pub(crate) fn schema_order(list: &[(String, String)]) -> Vec<&(String, String)> {
    fn visit<'a>(
        i: usize,
        list: &'a [(String, String)],
        seen: &mut [bool],
        out: &mut Vec<&'a (String, String)>,
    ) {
        if std::mem::replace(&mut seen[i], true) {
            return;
        }
        for field in list[i].1.split(';') {
            let Some(type_name) = field.split_whitespace().next() else {
                continue;
            };
            let type_name = type_name.split('[').next().unwrap_or(type_name);
            if let Some(j) = list.iter().position(|(name, _)| name == type_name) {
                visit(j, list, seen, out);
            }
        }
        out.push(&list[i]);
    }
    let mut out = Vec::with_capacity(list.len());
    let mut seen = vec![false; list.len()];
    for i in 0..list.len() {
        visit(i, list, &mut seen, &mut out);
    }
    out
}

fn struct_schemas(info: &crate::value::StructInfo) -> Vec<(String, String)> {
    let mut schemas = Vec::new();
    (info.add_schemas)(&mut |name, schema| {
        schemas.push((name.to_owned(), schema.to_owned()));
    });
    schemas
}

fn metadata_with_unit(unit: &str) -> String {
    format!("{{\"source\":\"DogLog\",\"unit\":\"{unit}\"}}")
}

fn prefixed(key: &str) -> String {
    format!("{LOG_TABLE}/{key}")
}

struct FileEntry {
    /// `None` while the type conflicts with an earlier use of the key.
    id: Option<u32>,
    type_name: String,
    last: Vec<u8>,
    has_last: bool,
    last_double: Option<f64>,
    unit: Option<String>,
}

/// Writes entries to a [`DataLog`], skipping values equal to the last one of the key.
pub struct DataLogWriter {
    log: Arc<DataLog>,
    entries: HashMap<String, FileEntry>,
    scratch: Vec<u8>,
    reported: HashSet<String>,
}

impl DataLogWriter {
    pub fn new(log: Arc<DataLog>) -> Self {
        Self {
            log,
            entries: HashMap::new(),
            scratch: Vec::new(),
            reported: HashSet::new(),
        }
    }

    pub fn log(&mut self, e: &Entry) {
        let type_name = e.value.log_type();
        if !self.entries.contains_key(&e.key) {
            let entry = self.create(e, &type_name);
            self.entries.insert(e.key.clone(), entry);
        }
        let Some(st) = self.entries.get_mut(&e.key) else {
            return;
        };
        if st.type_name != type_name {
            if self.reported.insert(e.key.clone()) {
                eprintln!(
                    "DogLog: type mismatch for '{}': was '{}', requested '{type_name}'; ignoring",
                    e.key, st.type_name
                );
            }
            return;
        }
        let Some(id) = st.id else {
            return;
        };

        self.scratch.clear();
        e.value.encode(&mut self.scratch);
        let changed = if let Value::Double(v) = e.value {
            // Java: DoubleLogEntry.update compares with `!=`, so NaN is logged every time and
            // 0.0 equals -0.0.
            let changed = st.last_double != Some(v) || v.is_nan();
            st.last_double = Some(v);
            changed
        } else {
            let changed = !st.has_last || st.last != self.scratch;
            if changed {
                st.last.clone_from(&self.scratch);
                st.has_last = true;
            }
            changed
        };
        if changed {
            self.log.append(id, e.timestamp, &self.scratch);
        }

        if let Some(unit) = &e.unit
            && e.value.takes_unit()
            && st.unit.as_deref() != Some(unit)
        {
            // Java: the first call with a unit sets the metadata it just created the entry with.
            self.log
                .set_metadata(id, &metadata_with_unit(unit), e.timestamp);
            st.unit = Some(unit.clone());
        }
    }

    fn create(&self, e: &Entry, type_name: &str) -> FileEntry {
        let metadata = match &e.unit {
            Some(unit) if e.value.takes_unit() => metadata_with_unit(unit),
            _ => ENTRY_METADATA.to_owned(),
        };
        let id = self
            .log
            .start(&prefixed(&e.key), type_name, &metadata, e.timestamp);
        if let Value::Struct { info, .. } | Value::StructArray { info, .. } = &e.value {
            let schemas = struct_schemas(info);
            for (name, schema) in schema_order(&schemas) {
                self.log.add_schema(
                    &format!("struct:{name}"),
                    "structschema",
                    schema.as_bytes(),
                    e.timestamp,
                );
            }
        }
        FileEntry {
            id,
            type_name: type_name.to_owned(),
            last: Vec::new(),
            has_last: false,
            last_double: None,
            unit: None,
        }
    }
}

/// Where NetworkTables records go. A trait so tests can record calls.
pub trait NtWriter: Send {
    fn publish(&mut self, entry: &Entry);
}

pub struct NoNt;

impl NtWriter for NoNt {
    fn publish(&mut self, _entry: &Entry) {}
}

struct Published {
    handle: Handle,
    type_name: String,
    unit: Option<String>,
}

impl Drop for Published {
    fn drop(&mut self) {
        // SAFETY: the handle was returned by NT_PublishEx and is released once.
        unsafe { (core().NT_Unpublish)(self.handle) }
    }
}

/// Publishes entries to `/Robot/<key>` with ntcore, one publisher per key.
pub struct NetworkTablesWriter {
    inst: Instance,
    publishers: HashMap<String, Published>,
    reported: HashSet<String>,
}

impl NetworkTablesWriter {
    pub fn new() -> Self {
        Self {
            inst: Instance::default_instance(),
            publishers: HashMap::new(),
            reported: HashSet::new(),
        }
    }

    fn nt_value(value: &Value) -> nt::Value {
        match value {
            Value::Boolean(v) => nt::Value::Boolean(*v),
            Value::BooleanArray(v) => nt::Value::BooleanArray(v.clone()),
            Value::Double(v) => nt::Value::Double(*v),
            Value::DoubleArray(v) => nt::Value::DoubleArray(v.clone()),
            Value::Integer(v) => nt::Value::Integer(*v),
            Value::IntegerArray(v) => nt::Value::IntegerArray(v.clone()),
            Value::String(v) | Value::CustomString { value: v, .. } => nt::Value::String(v.clone()),
            Value::StringArray(v) => nt::Value::StringArray(v.clone()),
            Value::Struct { bytes, .. } | Value::StructArray { bytes, .. } => {
                nt::Value::Raw(bytes.clone())
            }
        }
    }

    fn create(&self, name: &str, type_name: &str, value: &nt::Value) -> Option<Published> {
        let topic = self.inst.topic::<nt::Raw>(name).handle();
        let options = nt::Options::default().send_all(true).raw();
        let props = "{\"source\":\"DogLog\"}";
        let (text, props, nt_type) = (
            WpiStr::borrow(type_name),
            WpiStr::borrow(props),
            value.nt_type(),
        );
        // SAFETY: the strings and options outlive the call; ntcore copies them.
        let handle = unsafe { (core().NT_PublishEx)(topic, nt_type, &text, &props, &options) };
        (handle != 0).then(|| Published {
            handle,
            type_name: type_name.to_owned(),
            unit: None,
        })
    }

    fn set_unit(&self, name: &str, unit: &str) {
        let topic = self.inst.topic::<nt::Raw>(name).handle();
        crate::native::set_topic_unit(topic, unit);
    }
}

impl Default for NetworkTablesWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl NtWriter for NetworkTablesWriter {
    fn publish(&mut self, e: &Entry) {
        let name = prefixed(&e.key);
        let type_name = e.value.nt_type();
        let value = Self::nt_value(&e.value);
        if !self.publishers.contains_key(&e.key) {
            if let Value::Struct { info, .. } | Value::StructArray { info, .. } = &e.value {
                let schemas = struct_schemas(info);
                for (n, schema) in schema_order(&schemas) {
                    self.inst
                        .add_schema(&format!("struct:{n}"), "structschema", schema.as_bytes());
                }
            }
            let Some(created) = self.create(&name, &type_name, &value) else {
                return;
            };
            self.publishers.insert(e.key.clone(), created);
        }
        let Some(p) = self.publishers.get_mut(&e.key) else {
            return;
        };
        if p.type_name != type_name {
            if self.reported.insert(e.key.clone()) {
                eprintln!(
                    "DogLog: NetworkTables type mismatch for '{}': was '{}', requested '{type_name}'",
                    e.key, p.type_name
                );
            }
            return;
        }
        value.publish(p.handle, e.timestamp as i64);

        if let Some(unit) = &e.unit
            && e.value.takes_unit()
            && p.unit.as_deref() != Some(unit)
        {
            p.unit = Some(unit.clone());
            let unit = unit.clone();
            self.set_unit(&name, &unit);
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::datalog::Config;
    use crate::reader::{Log, decode};
    use crate::value::{LogValue, struct_value};
    use std::sync::Mutex;
    use wpimath::geometry::{Pose2d, Rotation2d};

    pub fn entry(key: &str, ts: u64, value: Value, unit: Option<&str>) -> Entry {
        Entry {
            key: key.to_owned(),
            force_nt: false,
            timestamp: ts,
            value,
            unit: unit.map(str::to_owned),
        }
    }

    /// A writer over an in-memory log; `finish` returns what the log holds.
    fn written(f: impl FnOnce(&mut DataLogWriter)) -> Log {
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let sink = bytes.clone();
        let log = Arc::new(DataLog::new(Config::callback(move |b| {
            sink.lock().unwrap().extend_from_slice(b);
        })));
        let mut writer = DataLogWriter::new(log.clone());
        f(&mut writer);
        drop(writer);
        drop(Arc::into_inner(log).unwrap());
        Log::parse(&bytes.lock().unwrap()).unwrap()
    }

    #[test]
    fn equal_values_are_skipped() {
        let log = written(|w| {
            w.log(&entry("a", 1, Value::Boolean(true), None));
            w.log(&entry("a", 2, Value::Boolean(true), None));
            w.log(&entry("a", 3, Value::Boolean(false), None));
            w.log(&entry("s", 1, "x".into_value(), None));
            w.log(&entry("s", 2, "x".into_value(), None));
            w.log(&entry("arr", 1, [1.0, 2.0].as_slice().into_value(), None));
            w.log(&entry("arr", 2, [1.0, 2.0].as_slice().into_value(), None));
            w.log(&entry("arr", 3, [1.0, 3.0].as_slice().into_value(), None));
        });
        assert_eq!(log.entry("/Robot/a").unwrap().records.len(), 2);
        assert_eq!(log.entry("/Robot/s").unwrap().records.len(), 1);
        assert_eq!(log.entry("/Robot/arr").unwrap().records.len(), 2);
    }

    #[test]
    fn doubles_follow_java_equality() {
        let log = written(|w| {
            for (ts, v) in [(1, 0.0), (2, -0.0), (3, f64::NAN), (4, f64::NAN), (5, 1.0)] {
                w.log(&entry("d", ts, Value::Double(v), None));
            }
        });
        let times: Vec<u64> = log
            .entry("/Robot/d")
            .unwrap()
            .records
            .iter()
            .map(|r| r.0)
            .collect();
        // -0.0 equals 0.0; NaN never equals itself.
        assert_eq!(times, [1, 3, 4, 5]);
    }

    #[test]
    fn metadata_and_units() {
        let log = written(|w| {
            w.log(&entry("plain", 1, Value::Double(1.0), None));
            w.log(&entry("u", 1, Value::Double(1.0), Some("volts")));
            w.log(&entry("u", 2, Value::Double(2.0), Some("volts")));
            w.log(&entry("u", 3, Value::Double(3.0), Some("amps")));
            w.log(&entry("flag", 1, Value::Boolean(true), Some("ignored")));
        });
        assert_eq!(
            log.entry("/Robot/plain").unwrap().metadata,
            "{\"source\":\"DogLog\"}"
        );
        let u = log.entry("/Robot/u").unwrap();
        assert_eq!(u.metadata, "{\"source\":\"DogLog\",\"unit\":\"amps\"}");
        // start, the first call's redundant set, then the change to amps
        assert_eq!(u.metadata_history.len(), 3);
        assert_eq!(u.records.len(), 3);
        assert_eq!(
            log.entry("/Robot/flag").unwrap().metadata,
            "{\"source\":\"DogLog\"}"
        );
    }

    #[test]
    fn schema_order_puts_contained_types_first() {
        let pose3d: Vec<(String, String)> = [
            ("Pose3d", "Translation3d translation;Rotation3d rotation"),
            ("Translation3d", "double x;double y;double z"),
            ("Rotation3d", "Quaternion q"),
            ("Quaternion", "double w;double x;double y;double z"),
        ]
        .map(|(n, s)| (n.to_owned(), s.to_owned()))
        .to_vec();
        let order: Vec<&str> = schema_order(&pose3d)
            .iter()
            .map(|(n, _)| n.as_str())
            .collect();
        assert_eq!(
            order,
            ["Translation3d", "Quaternion", "Rotation3d", "Pose3d"]
        );

        // A repeated type once, arrays by their element type.
        let list: Vec<(String, String)> = [("A", "B first;B[4] second"), ("B", "double v")]
            .map(|(n, s)| (n.to_owned(), s.to_owned()))
            .to_vec();
        let order: Vec<&str> = schema_order(&list)
            .iter()
            .map(|(n, _)| n.as_str())
            .collect();
        assert_eq!(order, ["B", "A"]);
    }

    #[test]
    fn type_conflict_keeps_the_first_type() {
        let log = written(|w| {
            w.log(&entry("k", 1, Value::Double(1.0), None));
            w.log(&entry("k", 2, "text".into_value(), None));
            w.log(&entry("k", 3, Value::Double(2.0), None));
        });
        let e = log.entry("/Robot/k").unwrap();
        assert_eq!(e.type_name, "double");
        assert_eq!(e.records.len(), 2);
        assert_eq!(decode::double(&e.records[1].1), Some(2.0));
    }

    #[test]
    fn structs_register_schemas_after_the_entry_contained_types_first() {
        let log = written(|w| {
            let pose = Pose2d::from_xy(1.0, 2.0, Rotation2d::new(0.5));
            w.log(&entry("pose", 7, struct_value(&pose), None));
            w.log(&entry("pose", 8, struct_value(&pose), None));
        });
        assert_eq!(
            log.names(),
            [
                "/Robot/pose",
                "/.schema/struct:Translation2d",
                "/.schema/struct:Rotation2d",
                "/.schema/struct:Pose2d"
            ]
        );
        let schema = log.entry("/.schema/struct:Pose2d").unwrap();
        assert_eq!(schema.type_name, "structschema");
        assert_eq!(
            schema.records[0].1,
            b"Translation2d translation;Rotation2d rotation"
        );
        let pose = log.entry("/Robot/pose").unwrap();
        assert_eq!(pose.type_name, "struct:Pose2d");
        assert_eq!(pose.records.len(), 1);
        assert_eq!(pose.records[0].1.len(), 24);
    }
}
