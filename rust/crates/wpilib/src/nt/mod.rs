//! NetworkTables over the ntcore C API: instances, typed topics, publishers, subscribers,
//! entries, tables and WPILib structs.

pub mod ffi;
pub mod structs;
pub mod table;
pub mod topic;
pub mod value;

use self::ffi::{DEFAULT_PORT3, DEFAULT_PORT4, Handle, NET_MODE_STARTING, WpiStr, core};
use std::thread;
use std::time::Duration;

pub use self::structs::{Struct, StructArrayKind, StructKind};
pub use self::table::{NetworkTable, NetworkTableEntry};
pub use self::topic::{DynPublisher, DynSubscriber, Entry, Options, Publisher, Subscriber, Topic};
pub use self::value::{
    Boolean, BooleanArray, Double, DoubleArray, Integer, IntegerArray, Kind, Raw, StringArray,
    Text, Timestamped, Value,
};

pub type BooleanTopic = Topic<Boolean>;
pub type IntegerTopic = Topic<Integer>;
pub type DoubleTopic = Topic<Double>;
pub type StringTopic = Topic<Text>;
pub type RawTopic = Topic<Raw>;
pub type BooleanArrayTopic = Topic<BooleanArray>;
pub type IntegerArrayTopic = Topic<IntegerArray>;
pub type DoubleArrayTopic = Topic<DoubleArray>;
pub type StringArrayTopic = Topic<StringArray>;
pub type StructTopic<S> = Topic<StructKind<S>>;
pub type StructArrayTopic<S> = Topic<StructArrayKind<S>>;

/// An ntcore instance. Copies refer to the same instance.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Instance(Handle);

impl Instance {
    /// The instance behind Java's `NetworkTableInstance.getDefault()`.
    pub fn default_instance() -> Self {
        // SAFETY: no arguments, no preconditions.
        Self(unsafe { (core().NT_GetDefaultInstance)() })
    }

    pub fn create() -> Self {
        // SAFETY: no arguments, no preconditions.
        Self(unsafe { (core().NT_CreateInstance)() })
    }

    /// Frees the instance. Handles made from it are invalid afterwards.
    pub fn destroy(self) {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_DestroyInstance)(self.0) }
    }

    pub fn handle(self) -> Handle {
        self.0
    }

    #[cfg(test)]
    pub(crate) const fn from_handle(handle: Handle) -> Self {
        Self(handle)
    }

    /// Starts an NT4 server. An empty `listen` address listens on every interface.
    pub fn start_server(self, persist_file: &str, listen: &str, port3: u32, port4: u32) {
        let (persist, listen) = (cstr(persist_file), cstr(listen));
        // SAFETY: both strings are NUL-terminated and outlive the call; ntcore copies them.
        unsafe {
            (core().NT_StartServer)(self.0, &persist.wpi(), &listen.wpi(), port3, port4);
        }
    }

    pub fn stop_server(self) {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_StopServer)(self.0) }
    }

    pub fn start_client4(self, identity: &str) {
        let identity = cstr(identity);
        // SAFETY: the string is NUL-terminated and outlives the call.
        unsafe { (core().NT_StartClient4)(self.0, &identity.wpi()) }
    }

    pub fn set_server(self, host: &str, port: u32) {
        let host = cstr(host);
        // SAFETY: the string is NUL-terminated and outlives the call.
        unsafe { (core().NT_SetServer)(self.0, &host.wpi(), port) }
    }

    pub fn stop_client(self) {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_StopClient)(self.0) }
    }

    /// Bitmask of `NT_NetworkMode`.
    pub fn network_mode(self) -> u32 {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_GetNetworkMode)(self.0) }
    }

    pub fn is_connected(self) -> bool {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_IsConnected)(self.0) != 0 }
    }

    /// Sends local changes to the network now.
    pub fn flush(self) {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_Flush)(self.0) }
    }

    /// Delivers local changes to local subscribers now.
    pub fn flush_local(self) {
        // SAFETY: plain handle argument.
        unsafe { (core().NT_FlushLocal)(self.0) }
    }

    pub fn topic<K: Kind>(self, name: &str) -> Topic<K> {
        Topic::new(self, self.topic_handle(name))
    }

    /// An untyped entry, Java's `NetworkTableInstance.getEntry`.
    pub fn entry(self, name: &str) -> NetworkTableEntry {
        let name = WpiStr::borrow(name);
        // SAFETY: name borrows a string that outlives the call; ntcore copies it.
        NetworkTableEntry::new(unsafe { (core().NT_GetEntry)(self.0, &name) })
    }

    /// The table at `path`, Java's `NetworkTableInstance.getTable`.
    pub fn table(self, path: &str) -> NetworkTable {
        NetworkTable::root(self, path)
    }

    pub(crate) fn topic_handle(self, name: &str) -> Handle {
        let name = WpiStr::borrow(name);
        // SAFETY: name borrows a string that outlives the call; ntcore copies it.
        unsafe { (core().NT_GetTopic)(self.0, &name) }
    }

    pub fn has_schema(self, name: &str) -> bool {
        let name = WpiStr::borrow(name);
        // SAFETY: name borrows a string that outlives the call.
        unsafe { (core().NT_HasSchema)(self.0, &name) != 0 }
    }

    /// Publishes a retained schema topic `/.schema/<name>`. Repeats are ignored.
    pub fn add_schema(self, name: &str, kind: &str, schema: &[u8]) {
        let (name, kind) = (WpiStr::borrow(name), WpiStr::borrow(kind));
        // SAFETY: the strings and schema outlive the call; ntcore copies them.
        unsafe { (core().NT_AddSchema)(self.0, &name, &kind, schema.as_ptr(), schema.len()) }
    }
}

/// Microseconds on the ntcore clock, the base of NetworkTables timestamps.
pub fn now() -> i64 {
    // SAFETY: no arguments, no preconditions.
    unsafe { (core().NT_Now)() }
}

/// Starts the default instance as an NT4 server, as Java's `RobotBase` constructor does:
/// persistent values in `/home/lvuser/networktables.json` on a robot and `networktables.json`
/// in simulation. Waits up to one second for the server to come up.
///
/// Java also subscribes to every topic so persistent values reach local readers. That is
/// skipped because the robot stores no persistent values.
pub fn start_default_server() {
    let inst = Instance::default_instance();
    let persist = if crate::hal::is_real() {
        "/home/lvuser/networktables.json"
    } else {
        "networktables.json"
    };
    inst.start_server(persist, "", DEFAULT_PORT3, DEFAULT_PORT4);
    for _ in 0..100 {
        if inst.network_mode() & NET_MODE_STARTING == 0 {
            return;
        }
        thread::sleep(Duration::from_millis(10));
    }
    eprintln!("timed out while waiting for NT server to start");
}

/// A NUL-free copy of a string, kept alive while a `WpiStr` borrows it.
pub(crate) struct Text8(String);

pub(crate) fn cstr(text: &str) -> Text8 {
    Text8(text.replace('\0', ""))
}

impl Text8 {
    pub(crate) fn wpi(&self) -> WpiStr {
        WpiStr::borrow(&self.0)
    }
}
