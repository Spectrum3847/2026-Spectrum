//! The ntcore C API (`ntcore_c.h`, `ntcore_c_types.h`). All `unsafe` FFI lives here and in the
//! safe wrappers next to it.

use crate::native_lib;
use std::ffi::c_char;
use std::sync::OnceLock;

pub type Handle = u32;

// wpi/string.h: `WPI_String`
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WpiStr {
    pub str: *const c_char,
    pub len: usize,
}

impl WpiStr {
    pub const EMPTY: Self = Self {
        str: std::ptr::null(),
        len: 0,
    };

    /// Borrows `text`; the result must not outlive it.
    pub fn borrow(text: &str) -> Self {
        Self {
            str: text.as_ptr().cast(),
            len: text.len(),
        }
    }
}

// ntcore_c.h: `enum NT_Type`
pub const NT_BOOLEAN: i32 = 0x01;
pub const NT_DOUBLE: i32 = 0x02;
pub const NT_STRING: i32 = 0x04;
pub const NT_RAW: i32 = 0x08;
pub const NT_BOOLEAN_ARRAY: i32 = 0x10;
pub const NT_DOUBLE_ARRAY: i32 = 0x20;
pub const NT_STRING_ARRAY: i32 = 0x40;
pub const NT_INTEGER: i32 = 0x100;
pub const NT_FLOAT: i32 = 0x200;
pub const NT_INTEGER_ARRAY: i32 = 0x400;
pub const NT_FLOAT_ARRAY: i32 = 0x800;

// ntcore_c.h: `enum NT_NetworkMode`
pub const NET_MODE_STARTING: u32 = 0x08;

pub const DEFAULT_PORT3: u32 = 1735;
pub const DEFAULT_PORT4: u32 = 5810;

// ntcore_c.h: `struct { T* ptr; size_t size; }` members of `NT_Value`'s union
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RawArray<T> {
    pub ptr: *mut T,
    pub size: usize,
}

// ntcore_c.h: the anonymous union in `NT_Value`
#[repr(C)]
#[derive(Clone, Copy)]
pub union ValueData {
    pub v_boolean: i32,
    pub v_int: i64,
    pub v_float: f32,
    pub v_double: f64,
    pub v_string: WpiStr,
    pub v_raw: RawArray<u8>,
    pub arr_boolean: RawArray<i32>,
    pub arr_double: RawArray<f64>,
    pub arr_float: RawArray<f32>,
    pub arr_int: RawArray<i64>,
    pub arr_string: RawArray<WpiStr>,
}

// ntcore_c.h: `struct NT_Value`
#[repr(C)]
#[derive(Clone, Copy)]
pub struct NtValue {
    pub ty: i32,
    pub last_change: i64,
    pub server_time: i64,
    pub data: ValueData,
}

impl NtValue {
    pub fn unassigned() -> Self {
        Self {
            ty: 0,
            last_change: 0,
            server_time: 0,
            data: ValueData { v_int: 0 },
        }
    }
}

// ntcore_c.h: `struct NT_PubSubOptions`
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct PubSubOptionsRaw {
    pub struct_size: u32,
    pub poll_storage: u32,
    pub periodic: f64,
    pub exclude_publisher: u32,
    pub send_all: i32,
    pub topics_only: i32,
    pub prefix_match: i32,
    pub keep_duplicates: i32,
    pub disable_remote: i32,
    pub disable_local: i32,
    pub exclude_self: i32,
    pub hidden: i32,
}

// Sizes from the C layout rules; a mismatch here means a wrong struct, not a wrong build.
// Pointer-sized fields: these sizes hold on 64-bit hosts only (roboRIO is 32-bit).
// Sizes and offsets from the C compilers: host gcc for 64-bit, the roboRIO
// arm-frc2025 gcc for 32-bit, both on the WPILib 2026.2.1 headers.
#[cfg(target_pointer_width = "64")]
const _: () = {
    assert!(size_of::<WpiStr>() == 16);
    assert!(size_of::<ValueData>() == 16);
    assert!(size_of::<NtValue>() == 40);
};
#[cfg(target_pointer_width = "32")]
const _: () = {
    assert!(size_of::<WpiStr>() == 8);
    assert!(size_of::<ValueData>() == 8);
    assert!(size_of::<NtValue>() == 32);
};
const _: () = {
    assert!(size_of::<PubSubOptionsRaw>() == 56);
    assert!(align_of::<NtValue>() == 8);
    assert!(std::mem::offset_of!(NtValue, last_change) == 8);
    assert!(std::mem::offset_of!(NtValue, data) == 24);
};

native_lib! {
    pub struct NtCore = "ntcore" {
        // ntcore_c.h: instances
        fn NT_GetDefaultInstance() -> Handle;
        fn NT_CreateInstance() -> Handle;
        fn NT_DestroyInstance(inst: Handle);
        fn NT_StartServer(
            inst: Handle,
            persist_filename: *const WpiStr,
            listen_address: *const WpiStr,
            port3: u32,
            port4: u32,
        );
        fn NT_StopServer(inst: Handle);
        fn NT_StartClient4(inst: Handle, identity: *const WpiStr);
        fn NT_StopClient(inst: Handle);
        fn NT_SetServer(inst: Handle, server_name: *const WpiStr, port: u32);
        fn NT_GetNetworkMode(inst: Handle) -> u32;
        fn NT_IsConnected(inst: Handle) -> i32;
        fn NT_Flush(inst: Handle);
        fn NT_FlushLocal(inst: Handle);
        fn NT_Now() -> i64;
        fn NT_HasSchema(inst: Handle, name: *const WpiStr) -> i32;
        fn NT_AddSchema(
            inst: Handle,
            name: *const WpiStr,
            kind: *const WpiStr,
            schema: *const u8,
            schema_size: usize,
        );
        // ntcore_c.h: topics and handles
        fn NT_GetTopic(inst: Handle, name: *const WpiStr) -> Handle;
        fn NT_GetEntry(inst: Handle, name: *const WpiStr) -> Handle;
        fn NT_GetTopicExists(handle: Handle) -> i32;
        fn NT_Subscribe(
            topic: Handle,
            ty: i32,
            type_str: *const WpiStr,
            options: *const PubSubOptionsRaw,
        ) -> Handle;
        fn NT_Unsubscribe(sub: Handle);
        fn NT_Publish(
            topic: Handle,
            ty: i32,
            type_str: *const WpiStr,
            options: *const PubSubOptionsRaw,
        ) -> Handle;
        fn NT_PublishEx(
            topic: Handle,
            ty: i32,
            type_str: *const WpiStr,
            properties: *const WpiStr,
            options: *const PubSubOptionsRaw,
        ) -> Handle;
        fn NT_Unpublish(pubentry: Handle);
        fn NT_GetEntryEx(
            topic: Handle,
            ty: i32,
            type_str: *const WpiStr,
            options: *const PubSubOptionsRaw,
        ) -> Handle;
        fn NT_ReleaseEntry(entry: Handle);
        // ntcore_c.h: generic value reads
        fn NT_GetEntryValue(entry: Handle, value: *mut NtValue);
        fn NT_ReadQueueValue(subentry: Handle, count: *mut usize) -> *mut NtValue;
        fn NT_DisposeValue(value: *mut NtValue);
        fn NT_DisposeValueArray(arr: *mut NtValue, count: usize);
        // ntcore_c_types.h: typed writes
        fn NT_SetBoolean(pubentry: Handle, time: i64, value: i32) -> i32;
        fn NT_SetInteger(pubentry: Handle, time: i64, value: i64) -> i32;
        fn NT_SetDouble(pubentry: Handle, time: i64, value: f64) -> i32;
        fn NT_SetString(pubentry: Handle, time: i64, value: *const WpiStr) -> i32;
        fn NT_SetRaw(pubentry: Handle, time: i64, value: *const u8, len: usize) -> i32;
        fn NT_SetBooleanArray(pubentry: Handle, time: i64, value: *const i32, len: usize) -> i32;
        fn NT_SetIntegerArray(pubentry: Handle, time: i64, value: *const i64, len: usize) -> i32;
        fn NT_SetDoubleArray(pubentry: Handle, time: i64, value: *const f64, len: usize) -> i32;
        fn NT_SetStringArray(pubentry: Handle, time: i64, value: *const WpiStr, len: usize) -> i32;
        fn NT_SetDefaultBoolean(pubentry: Handle, default_value: i32) -> i32;
        fn NT_SetDefaultInteger(pubentry: Handle, default_value: i64) -> i32;
        fn NT_SetDefaultDouble(pubentry: Handle, default_value: f64) -> i32;
        fn NT_SetDefaultString(pubentry: Handle, default_value: *const WpiStr) -> i32;
        fn NT_SetDefaultRaw(pubentry: Handle, default_value: *const u8, len: usize) -> i32;
        fn NT_SetDefaultBooleanArray(pubentry: Handle, default_value: *const i32, len: usize) -> i32;
        fn NT_SetDefaultIntegerArray(pubentry: Handle, default_value: *const i64, len: usize) -> i32;
        fn NT_SetDefaultDoubleArray(pubentry: Handle, default_value: *const f64, len: usize) -> i32;
        fn NT_SetDefaultStringArray(pubentry: Handle, default_value: *const WpiStr, len: usize) -> i32;
    }
}

static CORE: OnceLock<NtCore> = OnceLock::new();

/// The loaded ntcore.
///
/// # Panics
/// When the library cannot be loaded.
pub fn core() -> &'static NtCore {
    CORE.get_or_init(|| {
        // SAFETY: the declared signatures come from the 2026.2.1 headers; the library's own
        // initializers are trusted.
        unsafe { NtCore::load() }.unwrap_or_else(|e| {
            panic!(
                "{}",
                crate::native::load_failure(&NtCore::file(), &e, "WPILib")
            )
        })
    })
}
