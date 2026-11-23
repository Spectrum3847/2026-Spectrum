//! The topic reads and writes dashboard classes make, behind a trait so the topic layout
//! of each class can be tested without ntcore.

use crate::nt::{DynPublisher, DynSubscriber, Instance, Options, Value};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

/// Writes and reads topics by full path. Publishers and subscribers are created on first
/// use and live as long as the implementation.
pub trait TopicIo {
    /// Announces `path` with the type of `proto`, without a value.
    fn publish(&mut self, path: &str, proto: &Value);

    /// Like [`publish`](Self::publish), with initial topic properties as a JSON object.
    fn publish_with_properties(&mut self, path: &str, properties: &str, proto: &Value);

    /// Writes `value`, publishing `path` first if needed.
    fn set(&mut self, path: &str, value: &Value);

    /// Writes `value` unless the topic already has one.
    fn set_default(&mut self, path: &str, value: &Value);

    /// Starts queueing changes to `path` of the type of `proto`, ignoring this io's own
    /// publisher on the same path.
    fn subscribe(&mut self, path: &str, proto: &Value);

    /// Changes to a subscribed path since the last call, oldest first.
    fn read_queue(&mut self, path: &str) -> Vec<Value>;
}

/// A shared io, for objects that write between dashboard updates.
pub type SharedIo = Rc<RefCell<dyn TopicIo>>;

/// [`TopicIo`] over an ntcore instance.
pub struct NtIo {
    inst: Instance,
    publishers: HashMap<String, DynPublisher>,
    subscribers: HashMap<String, DynSubscriber>,
}

impl NtIo {
    pub fn new(inst: Instance) -> Self {
        Self {
            inst,
            publishers: HashMap::new(),
            subscribers: HashMap::new(),
        }
    }

    fn publisher(&mut self, path: &str, properties: &str, proto: &Value) -> &DynPublisher {
        self.publishers.entry(path.to_owned()).or_insert_with(|| {
            DynPublisher::new(self.inst, path, properties, proto, &Options::default())
        })
    }
}

impl TopicIo for NtIo {
    fn publish(&mut self, path: &str, proto: &Value) {
        self.publisher(path, "", proto);
    }

    fn publish_with_properties(&mut self, path: &str, properties: &str, proto: &Value) {
        self.publisher(path, properties, proto);
    }

    fn set(&mut self, path: &str, value: &Value) {
        self.publisher(path, "", value).set(value);
    }

    fn set_default(&mut self, path: &str, value: &Value) {
        self.publisher(path, "", value).set_default(value);
    }

    fn subscribe(&mut self, path: &str, proto: &Value) {
        let exclude = self.publishers.get(path).map_or(0, DynPublisher::handle);
        let inst = self.inst;
        self.subscribers.entry(path.to_owned()).or_insert_with(|| {
            DynSubscriber::new(
                inst,
                path,
                proto,
                &Options::default().exclude_publisher(exclude),
            )
        });
    }

    fn read_queue(&mut self, path: &str) -> Vec<Value> {
        self.subscribers
            .get(path)
            .map(DynSubscriber::read_queue)
            .unwrap_or_default()
    }
}

/// In-memory [`TopicIo`] that records what was written.
#[cfg(test)]
#[derive(Default)]
pub struct FakeIo {
    pub values: std::collections::BTreeMap<String, Value>,
    pub properties: std::collections::BTreeMap<String, String>,
    pub published: std::collections::BTreeMap<String, Value>,
    pub subscribed: Vec<String>,
    pub pending: HashMap<String, Vec<Value>>,
    pub writes: Vec<String>,
}

#[cfg(test)]
impl FakeIo {
    pub fn shared() -> Rc<RefCell<FakeIo>> {
        Rc::new(RefCell::new(FakeIo::default()))
    }

    pub fn double(&self, path: &str) -> f64 {
        match self.values.get(path) {
            Some(Value::Double(d)) => *d,
            other => panic!("{path}: {other:?}"),
        }
    }

    pub fn text(&self, path: &str) -> String {
        match self.values.get(path) {
            Some(Value::String(s)) => s.clone(),
            other => panic!("{path}: {other:?}"),
        }
    }

    pub fn strings(&self, path: &str) -> Vec<String> {
        match self.values.get(path) {
            Some(Value::StringArray(s)) => s.clone(),
            other => panic!("{path}: {other:?}"),
        }
    }

    pub fn doubles(&self, path: &str) -> Vec<f64> {
        match self.values.get(path) {
            Some(Value::DoubleArray(s)) => s.clone(),
            other => panic!("{path}: {other:?}"),
        }
    }

    pub fn push(&mut self, path: &str, value: Value) {
        self.pending.entry(path.to_owned()).or_default().push(value);
    }
}

#[cfg(test)]
impl TopicIo for FakeIo {
    fn publish(&mut self, path: &str, proto: &Value) {
        self.published
            .entry(path.to_owned())
            .or_insert_with(|| proto.clone());
    }

    fn publish_with_properties(&mut self, path: &str, properties: &str, proto: &Value) {
        self.publish(path, proto);
        self.properties
            .insert(path.to_owned(), properties.to_owned());
    }

    fn set(&mut self, path: &str, value: &Value) {
        self.publish(path, value);
        self.values.insert(path.to_owned(), value.clone());
        self.writes.push(path.to_owned());
    }

    fn set_default(&mut self, path: &str, value: &Value) {
        self.publish(path, value);
        self.values
            .entry(path.to_owned())
            .or_insert_with(|| value.clone());
    }

    fn subscribe(&mut self, path: &str, _proto: &Value) {
        self.subscribed.push(path.to_owned());
    }

    fn read_queue(&mut self, path: &str) -> Vec<Value> {
        self.pending.remove(path).unwrap_or_default()
    }
}

#[cfg(test)]
impl TopicIo for Rc<RefCell<FakeIo>> {
    fn publish(&mut self, path: &str, proto: &Value) {
        self.borrow_mut().publish(path, proto);
    }

    fn publish_with_properties(&mut self, path: &str, properties: &str, proto: &Value) {
        self.borrow_mut()
            .publish_with_properties(path, properties, proto);
    }

    fn set(&mut self, path: &str, value: &Value) {
        self.borrow_mut().set(path, value);
    }

    fn set_default(&mut self, path: &str, value: &Value) {
        self.borrow_mut().set_default(path, value);
    }

    fn subscribe(&mut self, path: &str, proto: &Value) {
        self.borrow_mut().subscribe(path, proto);
    }

    fn read_queue(&mut self, path: &str) -> Vec<Value> {
        self.borrow_mut().read_queue(path)
    }
}
