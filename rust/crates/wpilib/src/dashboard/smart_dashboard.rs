//! Java's `SmartDashboard`: simple values under the `SmartDashboard` table, and Sendables
//! published as sub-tables.
//!
//! Sendables register per thread, because they hold single-threaded handles. Call
//! [`put_data`] and [`update_values`] from the robot loop thread. The value functions work
//! from any thread.

use super::io::{NtIo, SharedIo};
use super::sendable::{Sendable, SendableBuilder};
use crate::hal;
use crate::nt::{Instance, NetworkTableEntry, Value};
use std::cell::RefCell;
use std::collections::HashMap;
use std::mem;
use std::rc::Rc;
use std::sync::{Mutex, Once, OnceLock, PoisonError};
use std::thread::{self, ThreadId};

const TABLE: &str = "/SmartDashboard";

// hal/FRCUsageReporting.h: kResourceType_SmartDashboard, kSmartDashboard_Instance
const RESOURCE_SMART_DASHBOARD: i32 = 43;
const INSTANCE_SMART_DASHBOARD: i32 = 1;

struct Registered {
    key: String,
    identity: usize,
    builder: SendableBuilder,
}

/// The Sendables on the dashboard, in registration order.
#[derive(Default)]
struct Registry {
    entries: Vec<Registered>,
}

impl Registry {
    /// Publishes `data` under `key` unless that same object already is. A different object
    /// replaces the old one.
    fn put(&mut self, key: &str, data: &dyn Sendable, io: SharedIo) {
        let identity = data.identity();
        if self
            .entries
            .iter()
            .any(|e| e.key == key && e.identity == identity)
        {
            return;
        }
        let table = format!("{TABLE}/{key}");
        let mut builder = SendableBuilder::new(table.clone(), io.clone());
        data.init_sendable(&mut builder);
        builder.start_listeners();
        io.borrow_mut()
            .set(&format!("{table}/.name"), &Value::String(key.to_owned()));
        let entry = Registered {
            key: key.to_owned(),
            identity,
            builder,
        };
        match self.entries.iter().position(|e| e.key == key) {
            Some(i) => self.entries[i] = entry,
            None => self.entries.push(entry),
        }
    }

    fn update(&mut self) {
        for entry in &mut self.entries {
            entry.builder.update();
        }
    }

    fn contains(&self, key: &str) -> bool {
        self.entries.iter().any(|e| e.key == key)
    }
}

thread_local! {
    static REGISTRY: RefCell<Registry> = RefCell::new(Registry::default());
}

static LOOP_THREAD: OnceLock<ThreadId> = OnceLock::new();

/// Marks the calling thread as the robot loop thread, the only one whose registry is updated.
pub(super) fn claim_loop_thread() {
    let _ = LOOP_THREAD.set(thread::current().id());
}

/// The registry is per thread, so use from another thread would silently never reach the
/// dashboard. Fail loudly instead.
#[track_caller]
pub(super) fn check_loop_thread() {
    if let Some(owner) = LOOP_THREAD.get() {
        assert!(
            *owner == thread::current().id(),
            "SmartDashboard, Sendables and Alerts must be used from the robot loop thread"
        );
    }
}

fn report_usage() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| hal::report_usage(RESOURCE_SMART_DASHBOARD, INSTANCE_SMART_DASHBOARD));
}

fn new_io() -> SharedIo {
    Rc::new(RefCell::new(NtIo::new(Instance::default_instance())))
}

/// Publishes a Sendable under `SmartDashboard/<key>`. Putting the same object again does
/// nothing.
pub fn put_data(key: &str, data: &impl Sendable) {
    check_loop_thread();
    report_usage();
    let io = new_io();
    // Registering can run user code that registers more, so the registry is not held then.
    let mut taken = REGISTRY.with(|r| mem::take(&mut *r.borrow_mut()));
    taken.put(key, data, io);
    merge_back(taken);
}

fn merge_back(mut taken: Registry) {
    REGISTRY.with(|r| {
        let added = mem::take(&mut *r.borrow_mut());
        for entry in added.entries {
            match taken.entries.iter().position(|e| e.key == entry.key) {
                Some(i) => taken.entries[i] = entry,
                None => taken.entries.push(entry),
            }
        }
        *r.borrow_mut() = taken;
    });
}

/// Whether a Sendable is registered under `key`.
pub fn contains_data(key: &str) -> bool {
    REGISTRY.with(|r| r.borrow().contains(key))
}

/// Publishes getters and applies dashboard changes for every Sendable. The robot loop calls
/// this once per pass, after `robot_periodic`.
pub fn update_values() {
    check_loop_thread();
    let mut taken = REGISTRY.with(|r| mem::take(&mut *r.borrow_mut()));
    taken.update();
    merge_back(taken);
}

fn entry(key: &str) -> NetworkTableEntry {
    static ENTRIES: Mutex<Option<HashMap<String, NetworkTableEntry>>> = Mutex::new(None);
    report_usage();
    let mut entries = ENTRIES.lock().unwrap_or_else(PoisonError::into_inner);
    *entries
        .get_or_insert_with(HashMap::new)
        .entry(key.to_owned())
        .or_insert_with(|| Instance::default_instance().entry(&format!("{TABLE}/{key}")))
}

pub fn contains_key(key: &str) -> bool {
    check_loop_thread();
    Instance::default_instance().table(TABLE).contains_key(key)
}

pub fn put_boolean(key: &str, value: bool) -> bool {
    entry(key).set_boolean(value)
}

pub fn get_boolean(key: &str, default: bool) -> bool {
    entry(key).get_boolean(default)
}

pub fn set_default_boolean(key: &str, default: bool) -> bool {
    entry(key).set_default_value(&Value::Boolean(default))
}

pub fn put_number(key: &str, value: f64) -> bool {
    entry(key).set_double(value)
}

pub fn get_number(key: &str, default: f64) -> f64 {
    entry(key).get_double(default)
}

pub fn set_default_number(key: &str, default: f64) -> bool {
    entry(key).set_default_value(&Value::Double(default))
}

pub fn put_string(key: &str, value: &str) -> bool {
    entry(key).set_string(value)
}

pub fn get_string(key: &str, default: &str) -> String {
    entry(key).get_string(default)
}

pub fn set_default_string(key: &str, default: &str) -> bool {
    entry(key).set_default_value(&Value::String(default.to_owned()))
}

pub fn put_number_array(key: &str, value: &[f64]) -> bool {
    entry(key).set_double_array(value)
}

pub fn get_number_array(key: &str, default: &[f64]) -> Vec<f64> {
    entry(key).get_double_array(default)
}

pub fn put_string_array(key: &str, value: &[String]) -> bool {
    entry(key).set_string_array(value)
}

pub fn get_string_array(key: &str, default: &[String]) -> Vec<String> {
    entry(key).get_string_array(default)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dashboard::io::FakeIo;
    use std::cell::Cell;

    struct Counter {
        state: Rc<Cell<f64>>,
    }

    impl Sendable for Counter {
        fn init_sendable(&self, builder: &mut SendableBuilder) {
            builder.set_smartdashboard_type("Counter");
            let state = self.state.clone();
            builder.add_double_getter("n", move || state.get());
        }

        fn identity(&self) -> usize {
            Rc::as_ptr(&self.state) as usize
        }
    }

    fn counter() -> Counter {
        Counter {
            state: Rc::new(Cell::new(0.0)),
        }
    }

    #[test]
    fn put_publishes_type_name_and_controllable() {
        let fake = FakeIo::shared();
        let mut reg = Registry::default();
        reg.put("Count", &counter(), fake.clone());
        let io = fake.borrow();
        assert_eq!(io.text("/SmartDashboard/Count/.type"), "Counter");
        assert_eq!(io.text("/SmartDashboard/Count/.name"), "Count");
        assert_eq!(
            io.values["/SmartDashboard/Count/.controllable"],
            Value::Boolean(true)
        );
    }

    #[test]
    fn putting_the_same_object_again_does_nothing() {
        let fake = FakeIo::shared();
        let mut reg = Registry::default();
        let c = counter();
        reg.put("Count", &c, fake.clone());
        let writes = fake.borrow().writes.len();
        reg.put("Count", &c, fake.clone());
        assert_eq!(fake.borrow().writes.len(), writes);
        assert_eq!(reg.entries.len(), 1);
    }

    #[test]
    fn a_different_object_replaces_the_key() {
        let fake = FakeIo::shared();
        let mut reg = Registry::default();
        reg.put("Count", &counter(), fake.clone());
        let b = counter();
        b.state.set(5.0);
        reg.put("Count", &b, fake.clone());
        assert_eq!(reg.entries.len(), 1);
        reg.update();
        assert_eq!(fake.borrow().double("/SmartDashboard/Count/n"), 5.0);
    }

    #[test]
    fn update_runs_every_sendable() {
        let fake = FakeIo::shared();
        let mut reg = Registry::default();
        let (a, b) = (counter(), counter());
        a.state.set(1.0);
        b.state.set(2.0);
        reg.put("A", &a, fake.clone());
        reg.put("B", &b, fake.clone());
        reg.update();
        let io = fake.borrow();
        assert_eq!(io.double("/SmartDashboard/A/n"), 1.0);
        assert_eq!(io.double("/SmartDashboard/B/n"), 2.0);
        assert!(reg.contains("A") && !reg.contains("C"));
    }
}
