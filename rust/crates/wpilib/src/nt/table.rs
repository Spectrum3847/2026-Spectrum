//! `NetworkTable` path helpers and untyped entries.

use super::Instance;
use super::ffi::Handle;
use super::topic::Topic;
use super::value::{self, Kind, Timestamped, Value};

/// A prefix of topic names. Java's `NetworkTable`.
#[derive(Clone, Debug)]
pub struct NetworkTable {
    inst: Instance,
    path: String,
    path_with_sep: String,
}

impl NetworkTable {
    /// Java's `NetworkTableInstance.getTable`: `""` and `"/"` are the root, and a missing
    /// leading slash is added.
    pub(crate) fn root(inst: Instance, key: &str) -> Self {
        let path = if key.is_empty() || key == "/" {
            String::new()
        } else if key.starts_with('/') {
            key.to_owned()
        } else {
            format!("/{key}")
        };
        Self::new(inst, path)
    }

    fn new(inst: Instance, path: String) -> Self {
        let path_with_sep = format!("{path}/");
        Self {
            inst,
            path,
            path_with_sep,
        }
    }

    /// Full path, `""` for the root.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// Full topic name of `key` in this table.
    pub fn key_path(&self, key: &str) -> String {
        format!("{}{key}", self.path_with_sep)
    }

    pub fn get_sub_table(&self, key: &str) -> NetworkTable {
        Self::new(self.inst, self.key_path(key))
    }

    pub fn topic<K: Kind>(&self, key: &str) -> Topic<K> {
        self.inst.topic::<K>(&self.key_path(key))
    }

    /// An untyped entry, Java's `NetworkTable.getEntry`.
    pub fn get_entry(&self, key: &str) -> NetworkTableEntry {
        self.inst.entry(&self.key_path(key))
    }

    pub fn contains_key(&self, key: &str) -> bool {
        self.inst
            .topic::<super::value::Raw>(&self.key_path(key))
            .exists()
    }
}

/// An untyped read and write handle on one topic. Java's `NetworkTableEntry`.
#[derive(Clone, Copy, Debug)]
pub struct NetworkTableEntry {
    handle: Handle,
}

impl NetworkTableEntry {
    pub(crate) fn new(handle: Handle) -> Self {
        Self { handle }
    }

    pub fn handle(&self) -> Handle {
        self.handle
    }

    /// The current value, `None` when the topic has none.
    pub fn get_value(&self) -> Option<Value> {
        self.get_atomic().map(|t| t.value)
    }

    pub fn get_atomic(&self) -> Option<Timestamped<Value>> {
        value::read_value(self.handle)
    }

    pub fn set_value(&self, value: &Value) -> bool {
        value.publish(self.handle, 0)
    }

    pub fn set_default_value(&self, value: &Value) -> bool {
        value.publish_default(self.handle)
    }

    pub fn get_boolean(&self, default: bool) -> bool {
        match self.get_value() {
            Some(Value::Boolean(b)) => b,
            _ => default,
        }
    }

    pub fn get_double(&self, default: f64) -> f64 {
        match self.get_value() {
            Some(Value::Double(d)) => d,
            Some(Value::Integer(i)) => i as f64,
            _ => default,
        }
    }

    pub fn get_string(&self, default: &str) -> String {
        match self.get_value() {
            Some(Value::String(s)) => s,
            _ => default.to_owned(),
        }
    }

    pub fn get_double_array(&self, default: &[f64]) -> Vec<f64> {
        match self.get_value() {
            Some(Value::DoubleArray(v)) => v,
            Some(Value::IntegerArray(v)) => v.into_iter().map(|i| i as f64).collect(),
            _ => default.to_vec(),
        }
    }

    pub fn get_string_array(&self, default: &[String]) -> Vec<String> {
        match self.get_value() {
            Some(Value::StringArray(v)) => v,
            _ => default.to_vec(),
        }
    }

    pub fn set_boolean(&self, value: bool) -> bool {
        self.set_value(&Value::Boolean(value))
    }

    pub fn set_integer(&self, value: i64) -> bool {
        self.set_value(&Value::Integer(value))
    }

    pub fn set_double(&self, value: f64) -> bool {
        self.set_value(&Value::Double(value))
    }

    pub fn set_string(&self, value: &str) -> bool {
        self.set_value(&Value::String(value.to_owned()))
    }

    pub fn set_double_array(&self, value: &[f64]) -> bool {
        self.set_value(&Value::DoubleArray(value.to_vec()))
    }

    pub fn set_string_array(&self, value: &[String]) -> bool {
        self.set_value(&Value::StringArray(value.to_vec()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table(key: &str) -> NetworkTable {
        NetworkTable::root(Instance::from_handle(0), key)
    }

    #[test]
    fn root_normalizes_like_java() {
        assert_eq!(table("").path(), "");
        assert_eq!(table("/").path(), "");
        assert_eq!(table("SmartDashboard").path(), "/SmartDashboard");
        assert_eq!(table("/SmartDashboard").path(), "/SmartDashboard");
    }

    #[test]
    fn keys_join_with_one_separator() {
        assert_eq!(table("limelight-back").key_path("tv"), "/limelight-back/tv");
        assert_eq!(table("").key_path("x"), "/x");
        let sub = table("SmartDashboard").get_sub_table("Auto Chooser");
        assert_eq!(sub.path(), "/SmartDashboard/Auto Chooser");
        assert_eq!(sub.key_path(".type"), "/SmartDashboard/Auto Chooser/.type");
        assert_eq!(
            sub.get_sub_table("a/b").key_path("c"),
            "/SmartDashboard/Auto Chooser/a/b/c"
        );
    }
}
