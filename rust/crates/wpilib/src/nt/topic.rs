//! Typed topics with their publishers, subscribers and entries.

use super::Instance;
use super::ffi::{Handle, PubSubOptionsRaw, WpiStr, core};
use super::value::{self, Kind, Timestamped, Value};
use std::marker::PhantomData;
use std::ops::Deref;

/// Publish and subscribe options, Java's `PubSubOption`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Options {
    /// Updates a subscriber queues. 0 uses ntcore's default.
    pub poll_storage: u32,
    /// Seconds between network sends. 0 uses ntcore's default of 0.1.
    pub periodic: f64,
    /// Publisher whose updates a subscriber ignores. 0 for none.
    pub exclude_publisher: Handle,
    pub send_all: bool,
    pub topics_only: bool,
    pub keep_duplicates: bool,
    pub disable_remote: bool,
    pub disable_local: bool,
    pub exclude_self: bool,
    pub hidden: bool,
}

impl Options {
    pub fn keep_duplicates(mut self, on: bool) -> Self {
        self.keep_duplicates = on;
        self
    }

    pub fn send_all(mut self, on: bool) -> Self {
        self.send_all = on;
        self
    }

    pub fn periodic(mut self, seconds: f64) -> Self {
        self.periodic = seconds;
        self
    }

    pub fn poll_storage(mut self, depth: u32) -> Self {
        self.poll_storage = depth;
        self
    }

    pub fn exclude_publisher(mut self, publisher: Handle) -> Self {
        self.exclude_publisher = publisher;
        self
    }

    pub fn exclude_self(mut self, on: bool) -> Self {
        self.exclude_self = on;
        self
    }

    pub fn disable_local(mut self, on: bool) -> Self {
        self.disable_local = on;
        self
    }

    pub fn disable_remote(mut self, on: bool) -> Self {
        self.disable_remote = on;
        self
    }

    pub fn topics_only(mut self, on: bool) -> Self {
        self.topics_only = on;
        self
    }

    pub fn hidden(mut self, on: bool) -> Self {
        self.hidden = on;
        self
    }

    /// The C struct, for crates that call ntcore directly.
    pub fn raw(&self) -> PubSubOptionsRaw {
        let flag = |on: bool| i32::from(on);
        PubSubOptionsRaw {
            struct_size: size_of::<PubSubOptionsRaw>() as u32,
            poll_storage: self.poll_storage,
            periodic: self.periodic,
            exclude_publisher: self.exclude_publisher,
            send_all: flag(self.send_all),
            topics_only: flag(self.topics_only),
            prefix_match: 0,
            keep_duplicates: flag(self.keep_duplicates),
            disable_remote: flag(self.disable_remote),
            disable_local: flag(self.disable_local),
            exclude_self: flag(self.exclude_self),
            hidden: flag(self.hidden),
        }
    }
}

/// A named value channel of type `K`.
pub struct Topic<K: Kind> {
    inst: Instance,
    handle: Handle,
    kind: PhantomData<fn() -> K>,
}

impl<K: Kind> Topic<K> {
    pub(crate) fn new(inst: Instance, handle: Handle) -> Self {
        Self {
            inst,
            handle,
            kind: PhantomData,
        }
    }

    pub fn handle(&self) -> Handle {
        self.handle
    }

    pub fn exists(&self) -> bool {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_GetTopicExists)(self.handle) != 0 }
    }

    pub fn publish(&self, options: &Options) -> Publisher<K> {
        K::prepare(self.inst);
        let type_string = K::type_string();
        let (text, raw) = (WpiStr::borrow(&type_string), options.raw());
        // SAFETY: text and raw outlive the call; ntcore copies them.
        let handle = unsafe { (core().NT_Publish)(self.handle, K::NT_TYPE, &text, &raw) };
        Publisher::new(handle)
    }

    /// Publishes with initial topic properties, a JSON object.
    pub fn publish_ex(&self, properties: &str, options: &Options) -> Publisher<K> {
        K::prepare(self.inst);
        let type_string = K::type_string();
        let (text, props, raw) = (
            WpiStr::borrow(&type_string),
            WpiStr::borrow(properties),
            options.raw(),
        );
        // SAFETY: all three outlive the call; ntcore copies them.
        let handle = unsafe { (core().NT_PublishEx)(self.handle, K::NT_TYPE, &text, &props, &raw) };
        Publisher::new(handle)
    }

    /// `default` is returned while the topic has no value.
    pub fn subscribe(&self, default: K::Out, options: &Options) -> Subscriber<K> {
        K::prepare(self.inst);
        let type_string = K::type_string();
        let (text, raw) = (WpiStr::borrow(&type_string), options.raw());
        // SAFETY: text and raw outlive the call; ntcore copies them.
        let handle = unsafe { (core().NT_Subscribe)(self.handle, K::NT_TYPE, &text, &raw) };
        Subscriber(Reader::new(handle, default))
    }

    /// A subscriber and weak publisher in one handle.
    pub fn entry(&self, default: K::Out, options: &Options) -> Entry<K> {
        K::prepare(self.inst);
        let type_string = K::type_string();
        let (text, raw) = (WpiStr::borrow(&type_string), options.raw());
        // SAFETY: text and raw outlive the call; ntcore copies them.
        let handle = unsafe { (core().NT_GetEntryEx)(self.handle, K::NT_TYPE, &text, &raw) };
        Entry(Reader::new(handle, default))
    }
}

/// Writes values of type `K`. Unpublishes on drop.
pub struct Publisher<K: Kind> {
    handle: Handle,
    kind: PhantomData<fn() -> K>,
}

impl<K: Kind> Publisher<K> {
    fn new(handle: Handle) -> Self {
        Self {
            handle,
            kind: PhantomData,
        }
    }

    pub fn handle(&self) -> Handle {
        self.handle
    }

    pub fn set(&self, value: K::In<'_>) -> bool {
        K::set(self.handle, 0, value)
    }

    /// Writes with an explicit timestamp in microseconds on the ntcore clock.
    pub fn set_at(&self, value: K::In<'_>, time: i64) -> bool {
        K::set(self.handle, time, value)
    }

    /// Publishes `value` unless a value already exists.
    pub fn set_default(&self, value: K::In<'_>) -> bool {
        K::set_default(self.handle, value)
    }
}

impl<K: Kind> Drop for Publisher<K> {
    fn drop(&mut self) {
        // SAFETY: the handle is released once.
        unsafe { (core().NT_Unpublish)(self.handle) }
    }
}

/// Reads for subscribers and entries.
pub struct Reader<K: Kind> {
    handle: Handle,
    default: K::Out,
}

impl<K: Kind> Reader<K> {
    fn new(handle: Handle, default: K::Out) -> Self {
        Self { handle, default }
    }

    pub fn handle(&self) -> Handle {
        self.handle
    }

    /// The latest value, or the default when there is none.
    pub fn get(&self) -> K::Out {
        self.get_atomic().value
    }

    /// The latest value with its timestamps. Both are 0 and the value is the default when
    /// nothing has been published.
    pub fn get_atomic(&self) -> Timestamped<K::Out> {
        value::read_value(self.handle)
            .and_then(|t| {
                K::from_value(&t.value).map(|value| Timestamped {
                    time: t.time,
                    server_time: t.server_time,
                    value,
                })
            })
            .unwrap_or_else(|| Timestamped {
                time: 0,
                server_time: 0,
                value: self.default.clone(),
            })
    }

    /// Changes since the last call, oldest first.
    pub fn read_queue(&self) -> Vec<Timestamped<K::Out>> {
        value::read_queue(self.handle)
            .into_iter()
            .filter_map(|t| {
                K::from_value(&t.value).map(|value| Timestamped {
                    time: t.time,
                    server_time: t.server_time,
                    value,
                })
            })
            .collect()
    }

    pub fn read_queue_values(&self) -> Vec<K::Out> {
        self.read_queue().into_iter().map(|t| t.value).collect()
    }
}

/// Reads values of type `K`. Unsubscribes on drop.
pub struct Subscriber<K: Kind>(Reader<K>);

impl<K: Kind> Deref for Subscriber<K> {
    type Target = Reader<K>;

    fn deref(&self) -> &Reader<K> {
        &self.0
    }
}

impl<K: Kind> Drop for Subscriber<K> {
    fn drop(&mut self) {
        // SAFETY: the handle is released once.
        unsafe { (core().NT_Unsubscribe)(self.0.handle) }
    }
}

/// Reads and writes values of type `K`. Released on drop.
pub struct Entry<K: Kind>(Reader<K>);

impl<K: Kind> Entry<K> {
    pub fn set(&self, value: K::In<'_>) -> bool {
        K::set(self.0.handle, 0, value)
    }

    pub fn set_at(&self, value: K::In<'_>, time: i64) -> bool {
        K::set(self.0.handle, time, value)
    }

    pub fn set_default(&self, value: K::In<'_>) -> bool {
        K::set_default(self.0.handle, value)
    }
}

impl<K: Kind> Deref for Entry<K> {
    type Target = Reader<K>;

    fn deref(&self) -> &Reader<K> {
        &self.0
    }
}

impl<K: Kind> Drop for Entry<K> {
    fn drop(&mut self) {
        // SAFETY: the handle is released once.
        unsafe { (core().NT_ReleaseEntry)(self.0.handle) }
    }
}

/// A publisher whose type is chosen at run time by the value it is created with.
pub struct DynPublisher {
    handle: Handle,
}

impl DynPublisher {
    /// Publishes `name` with the type of `proto`. `properties` is a JSON object or empty.
    pub fn new(
        inst: Instance,
        name: &str,
        properties: &str,
        proto: &Value,
        options: &Options,
    ) -> Self {
        let topic = inst.topic_handle(name);
        let (text, props, raw) = (
            WpiStr::borrow(proto.type_string()),
            WpiStr::borrow(properties),
            options.raw(),
        );
        // SAFETY: all three outlive the call; ntcore copies them.
        let handle = unsafe { (core().NT_PublishEx)(topic, proto.nt_type(), &text, &props, &raw) };
        Self { handle }
    }

    pub fn handle(&self) -> Handle {
        self.handle
    }

    pub fn set(&self, value: &Value) -> bool {
        value.publish(self.handle, 0)
    }

    pub fn set_default(&self, value: &Value) -> bool {
        value.publish_default(self.handle)
    }
}

impl Drop for DynPublisher {
    fn drop(&mut self) {
        // SAFETY: the handle is released once.
        unsafe { (core().NT_Unpublish)(self.handle) }
    }
}

/// A subscriber whose type is chosen at run time.
pub struct DynSubscriber {
    handle: Handle,
}

impl DynSubscriber {
    pub fn new(inst: Instance, name: &str, proto: &Value, options: &Options) -> Self {
        let topic = inst.topic_handle(name);
        let (text, raw) = (WpiStr::borrow(proto.type_string()), options.raw());
        // SAFETY: text and raw outlive the call; ntcore copies them.
        let handle = unsafe { (core().NT_Subscribe)(topic, proto.nt_type(), &text, &raw) };
        Self { handle }
    }

    /// Changes since the last call, oldest first.
    pub fn read_queue(&self) -> Vec<Value> {
        value::read_queue(self.handle)
            .into_iter()
            .map(|t| t.value)
            .collect()
    }
}

impl Drop for DynSubscriber {
    fn drop(&mut self) {
        // SAFETY: the handle is released once.
        unsafe { (core().NT_Unsubscribe)(self.handle) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn options_fill_struct_size_and_flags() {
        let raw = Options::default()
            .keep_duplicates(true)
            .send_all(true)
            .periodic(0.02)
            .poll_storage(8)
            .exclude_publisher(7)
            .raw();
        assert_eq!(raw.struct_size, 56);
        assert_eq!(raw.keep_duplicates, 1);
        assert_eq!(raw.send_all, 1);
        assert_eq!(raw.periodic, 0.02);
        assert_eq!(raw.poll_storage, 8);
        assert_eq!(raw.exclude_publisher, 7);
        assert_eq!(raw.hidden, 0);
        assert_eq!(raw.prefix_match, 0);
    }

    #[test]
    fn default_options_use_ntcore_defaults() {
        let raw = Options::default().raw();
        assert_eq!((raw.periodic, raw.poll_storage, raw.send_all), (0.0, 0, 0));
    }
}
