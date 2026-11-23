//! Serialized config text through the C layer (`ConfigJNI.Serialize*` and `Deserialize*`).

use super::{ConfigItem, ConfigSource, ConfigValue};
use crate::native::phoenix;
use std::ffi::{CStr, CString, c_char};
use std::ptr;

/// The text for all items, one `spn,kind_value` line each.
pub(super) fn serialize(items: &[ConfigItem]) -> String {
    items.iter().map(|item| serialize_item(*item)).collect()
}

/// One item's line. Empty when the C layer rejects the SPN.
fn serialize_item(item: ConfigItem) -> String {
    let p = phoenix();
    let mut out: *mut c_char = ptr::null_mut();
    // SAFETY: `out` is a valid out pointer for the whole call.
    let status = unsafe {
        match item.value {
            ConfigValue::Double(v) => (p.c_ctre_phoenix6_serialize_double)(item.spn, v, &mut out),
            ConfigValue::Int(v) => (p.c_ctre_phoenix6_serialize_int)(item.spn, v, &mut out),
            ConfigValue::Bool(v) => (p.c_ctre_phoenix6_serialize_bool)(item.spn, v, &mut out),
        }
    };
    let text = take_string(&mut out);
    if status == 0 { text } else { String::new() }
}

/// Copies a string the C layer allocated and gives it back.
pub(crate) fn take_string(ptr: &mut *mut c_char) -> String {
    if ptr.is_null() {
        return String::new();
    }
    // SAFETY: the library returned a NUL terminated string; it is copied before being freed.
    let text = unsafe { CStr::from_ptr(*ptr) }
        .to_string_lossy()
        .into_owned();
    // SAFETY: `ptr` came from the library's allocator and is freed once.
    unsafe { (phoenix().c_ctre_phoenix6_free_memory)(ptr) };
    text
}

/// Device config text read through the C deserializers.
pub(super) struct NativeSource {
    text: CString,
}

impl NativeSource {
    pub(super) fn new(text: &str) -> Self {
        Self {
            text: CString::new(text.replace('\0', "")).unwrap_or_default(),
        }
    }

    fn len(&self) -> u32 {
        u32::try_from(self.text.as_bytes().len()).unwrap_or(u32::MAX)
    }
}

impl ConfigSource for NativeSource {
    fn double(&self, spn: i32) -> Option<f64> {
        let mut value = 0.0;
        // SAFETY: the text is NUL terminated with the stated length; `value` is valid.
        let status = unsafe {
            (phoenix().c_ctre_phoenix6_deserialize_double)(
                spn,
                self.text.as_ptr(),
                self.len(),
                &mut value,
            )
        };
        (status == 0).then_some(value)
    }

    fn int(&self, spn: i32) -> Option<i32> {
        let mut value = 0;
        // SAFETY: as in `double`.
        let status = unsafe {
            (phoenix().c_ctre_phoenix6_deserialize_int)(
                spn,
                self.text.as_ptr(),
                self.len(),
                &mut value,
            )
        };
        (status == 0).then_some(value)
    }

    fn boolean(&self, spn: i32) -> Option<bool> {
        let mut value = false;
        // SAFETY: as in `double`.
        let status = unsafe {
            (phoenix().c_ctre_phoenix6_deserialize_bool)(
                spn,
                self.text.as_ptr(),
                self.len(),
                &mut value,
            )
        };
        (status == 0).then_some(value)
    }
}
