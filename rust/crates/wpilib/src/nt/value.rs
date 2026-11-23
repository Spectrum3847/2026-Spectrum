//! Dynamically typed NetworkTables values and the typed kinds that publishers and
//! subscribers are generic over.

use super::ffi::{
    Handle, NT_BOOLEAN, NT_BOOLEAN_ARRAY, NT_DOUBLE, NT_DOUBLE_ARRAY, NT_FLOAT, NT_FLOAT_ARRAY,
    NT_INTEGER, NT_INTEGER_ARRAY, NT_RAW, NT_STRING, NT_STRING_ARRAY, NtValue, RawArray, WpiStr,
    core,
};

/// A value of any NetworkTables type the robot uses. Floats read as doubles.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Boolean(bool),
    Integer(i64),
    Double(f64),
    String(String),
    Raw(Vec<u8>),
    BooleanArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

/// A value with the times ntcore recorded for it, in microseconds.
#[derive(Clone, Debug, PartialEq)]
pub struct Timestamped<T> {
    /// Local time base. 0 when no value has been published.
    pub time: i64,
    /// Server time base. May be 0 or 1 for locally set values.
    pub server_time: i64,
    pub value: T,
}

impl Value {
    pub fn nt_type(&self) -> i32 {
        match self {
            Self::Boolean(_) => NT_BOOLEAN,
            Self::Integer(_) => NT_INTEGER,
            Self::Double(_) => NT_DOUBLE,
            Self::String(_) => NT_STRING,
            Self::Raw(_) => NT_RAW,
            Self::BooleanArray(_) => NT_BOOLEAN_ARRAY,
            Self::IntegerArray(_) => NT_INTEGER_ARRAY,
            Self::DoubleArray(_) => NT_DOUBLE_ARRAY,
            Self::StringArray(_) => NT_STRING_ARRAY,
        }
    }

    /// The type string ntcore announces, as in Java's `*Topic.kTypeString`.
    pub fn type_string(&self) -> &'static str {
        match self {
            Self::Boolean(_) => Boolean::TYPE_STRING,
            Self::Integer(_) => Integer::TYPE_STRING,
            Self::Double(_) => Double::TYPE_STRING,
            Self::String(_) => Text::TYPE_STRING,
            Self::Raw(_) => Raw::TYPE_STRING,
            Self::BooleanArray(_) => BooleanArray::TYPE_STRING,
            Self::IntegerArray(_) => IntegerArray::TYPE_STRING,
            Self::DoubleArray(_) => DoubleArray::TYPE_STRING,
            Self::StringArray(_) => StringArray::TYPE_STRING,
        }
    }

    /// Writes to a publisher or entry handle. `time` 0 means now.
    pub fn publish(&self, handle: Handle, time: i64) -> bool {
        match self {
            Self::Boolean(v) => Boolean::set(handle, time, *v),
            Self::Integer(v) => Integer::set(handle, time, *v),
            Self::Double(v) => Double::set(handle, time, *v),
            Self::String(v) => Text::set(handle, time, v),
            Self::Raw(v) => Raw::set(handle, time, v),
            Self::BooleanArray(v) => BooleanArray::set(handle, time, v),
            Self::IntegerArray(v) => IntegerArray::set(handle, time, v),
            Self::DoubleArray(v) => DoubleArray::set(handle, time, v),
            Self::StringArray(v) => StringArray::set(handle, time, v),
        }
    }

    pub fn publish_default(&self, handle: Handle) -> bool {
        match self {
            Self::Boolean(v) => Boolean::set_default(handle, *v),
            Self::Integer(v) => Integer::set_default(handle, *v),
            Self::Double(v) => Double::set_default(handle, *v),
            Self::String(v) => Text::set_default(handle, v),
            Self::Raw(v) => Raw::set_default(handle, v),
            Self::BooleanArray(v) => BooleanArray::set_default(handle, v),
            Self::IntegerArray(v) => IntegerArray::set_default(handle, v),
            Self::DoubleArray(v) => DoubleArray::set_default(handle, v),
            Self::StringArray(v) => StringArray::set_default(handle, v),
        }
    }

    /// Copies a value out of ntcore memory. `None` for unassigned or unsupported types.
    fn from_nt(raw: &NtValue) -> Option<Self> {
        // SAFETY: each union field read matches the type tag ntcore set, and the pointers are
        // valid until the caller disposes the value.
        unsafe {
            Some(match raw.ty {
                NT_BOOLEAN => Self::Boolean(raw.data.v_boolean != 0),
                NT_INTEGER => Self::Integer(raw.data.v_int),
                NT_DOUBLE => Self::Double(raw.data.v_double),
                NT_FLOAT => Self::Double(f64::from(raw.data.v_float)),
                NT_STRING => Self::String(wpi_text(raw.data.v_string)),
                NT_RAW => Self::Raw(slice(raw.data.v_raw).to_vec()),
                NT_BOOLEAN_ARRAY => Self::BooleanArray(
                    slice(raw.data.arr_boolean)
                        .iter()
                        .map(|b| *b != 0)
                        .collect(),
                ),
                NT_INTEGER_ARRAY => Self::IntegerArray(slice(raw.data.arr_int).to_vec()),
                NT_DOUBLE_ARRAY => Self::DoubleArray(slice(raw.data.arr_double).to_vec()),
                NT_FLOAT_ARRAY => Self::DoubleArray(
                    slice(raw.data.arr_float)
                        .iter()
                        .map(|f| f64::from(*f))
                        .collect(),
                ),
                NT_STRING_ARRAY => Self::StringArray(
                    slice(raw.data.arr_string)
                        .iter()
                        .map(|s| wpi_text(*s))
                        .collect(),
                ),
                _ => return None,
            })
        }
    }
}

/// # Safety
/// `array` must point at `size` valid elements, or be null or empty.
unsafe fn slice<'a, T>(array: RawArray<T>) -> &'a [T] {
    if array.ptr.is_null() || array.size == 0 {
        &[]
    } else {
        // SAFETY: the caller guarantees ptr and size describe a live allocation.
        unsafe { std::slice::from_raw_parts(array.ptr, array.size) }
    }
}

/// # Safety
/// `text` must point at `len` valid bytes, or be null or empty.
unsafe fn wpi_text(text: WpiStr) -> String {
    if text.str.is_null() || text.len == 0 {
        return String::new();
    }
    // SAFETY: the caller guarantees str and len describe a live allocation.
    let bytes = unsafe { std::slice::from_raw_parts(text.str.cast::<u8>(), text.len) };
    String::from_utf8_lossy(bytes).into_owned()
}

/// The current value of an entry, subscriber or publisher handle.
pub fn read_value(handle: Handle) -> Option<Timestamped<Value>> {
    let mut raw = NtValue::unassigned();
    // SAFETY: raw is a valid out pointer; the value is disposed after the copy.
    unsafe { (core().NT_GetEntryValue)(handle, &mut raw) };
    let value = Value::from_nt(&raw).map(|value| Timestamped {
        time: raw.last_change,
        server_time: raw.server_time,
        value,
    });
    // SAFETY: raw was filled by NT_GetEntryValue and is disposed once.
    unsafe { (core().NT_DisposeValue)(&mut raw) };
    value
}

/// Values queued on a subscriber or entry since the last call, oldest first.
pub fn read_queue(handle: Handle) -> Vec<Timestamped<Value>> {
    let mut count = 0usize;
    // SAFETY: count is a valid out pointer.
    let values = unsafe { (core().NT_ReadQueueValue)(handle, &mut count) };
    if values.is_null() || count == 0 {
        return Vec::new();
    }
    // SAFETY: ntcore returned count initialized values at values.
    let raw = unsafe { std::slice::from_raw_parts(values, count) };
    let out = raw
        .iter()
        .filter_map(|r| {
            Value::from_nt(r).map(|value| Timestamped {
                time: r.last_change,
                server_time: r.server_time,
                value,
            })
        })
        .collect();
    // SAFETY: the array came from NT_ReadQueueValue and is disposed once, after the copy.
    unsafe { (core().NT_DisposeValueArray)(values, count) };
    out
}

fn ok(status: i32) -> bool {
    status != 0
}

/// A NetworkTables value type. Publishers, subscribers and entries are generic over it.
pub trait Kind: 'static {
    /// Owned value, as read back.
    type Out: Clone;
    /// Borrowed value, as written.
    type In<'a>;
    const NT_TYPE: i32;

    fn type_string() -> String;
    fn set(handle: Handle, time: i64, value: Self::In<'_>) -> bool;
    fn set_default(handle: Handle, value: Self::In<'_>) -> bool;
    fn from_value(value: &Value) -> Option<Self::Out>;
    fn as_in(value: &Self::Out) -> Self::In<'_>;

    /// Registers what dashboards need to decode this type. Called before publishing.
    fn prepare(_inst: super::Instance) {}
}

macro_rules! scalar_kind {
    ($name:ident, $out:ty, $nt:expr, $text:literal, $set:ident, $default:ident, |$v:ident| $arg:expr, $variant:pat => $conv:expr) => {
        pub struct $name;

        impl $name {
            pub const TYPE_STRING: &'static str = $text;
        }

        impl Kind for $name {
            type Out = $out;
            type In<'a> = $out;
            const NT_TYPE: i32 = $nt;

            fn type_string() -> String {
                $text.to_owned()
            }

            fn set(handle: Handle, time: i64, $v: $out) -> bool {
                // SAFETY: plain scalar arguments.
                ok(unsafe { (core().$set)(handle, time, $arg) })
            }

            fn set_default(handle: Handle, $v: $out) -> bool {
                // SAFETY: plain scalar arguments.
                ok(unsafe { (core().$default)(handle, $arg) })
            }

            fn from_value(value: &Value) -> Option<$out> {
                match value {
                    $variant => Some($conv),
                    _ => None,
                }
            }

            fn as_in(value: &$out) -> $out {
                *value
            }
        }
    };
}

scalar_kind!(Boolean, bool, NT_BOOLEAN, "boolean", NT_SetBoolean, NT_SetDefaultBoolean,
    |v| i32::from(v), Value::Boolean(b) => *b);
scalar_kind!(Integer, i64, NT_INTEGER, "int", NT_SetInteger, NT_SetDefaultInteger,
    |v| v, Value::Integer(i) => *i);

pub struct Double;

impl Double {
    pub const TYPE_STRING: &'static str = "double";
}

impl Kind for Double {
    type Out = f64;
    type In<'a> = f64;
    const NT_TYPE: i32 = NT_DOUBLE;

    fn type_string() -> String {
        Self::TYPE_STRING.to_owned()
    }

    fn set(handle: Handle, time: i64, value: f64) -> bool {
        // SAFETY: plain scalar arguments.
        ok(unsafe { (core().NT_SetDouble)(handle, time, value) })
    }

    fn set_default(handle: Handle, value: f64) -> bool {
        // SAFETY: plain scalar arguments.
        ok(unsafe { (core().NT_SetDefaultDouble)(handle, value) })
    }

    fn from_value(value: &Value) -> Option<f64> {
        match value {
            Value::Double(d) => Some(*d),
            Value::Integer(i) => Some(*i as f64),
            _ => None,
        }
    }

    fn as_in(value: &f64) -> f64 {
        *value
    }
}

/// NetworkTables `string`.
pub struct Text;

impl Text {
    pub const TYPE_STRING: &'static str = "string";
}

impl Kind for Text {
    type Out = String;
    type In<'a> = &'a str;
    const NT_TYPE: i32 = NT_STRING;

    fn type_string() -> String {
        Self::TYPE_STRING.to_owned()
    }

    fn set(handle: Handle, time: i64, value: &str) -> bool {
        let text = WpiStr::borrow(value);
        // SAFETY: text borrows value, which outlives the call; ntcore copies it.
        ok(unsafe { (core().NT_SetString)(handle, time, &text) })
    }

    fn set_default(handle: Handle, value: &str) -> bool {
        let text = WpiStr::borrow(value);
        // SAFETY: as in `set`.
        ok(unsafe { (core().NT_SetDefaultString)(handle, &text) })
    }

    fn from_value(value: &Value) -> Option<String> {
        match value {
            Value::String(s) => Some(s.clone()),
            _ => None,
        }
    }

    fn as_in(value: &String) -> &str {
        value
    }
}

pub struct Raw;

impl Raw {
    pub const TYPE_STRING: &'static str = "raw";
}

impl Kind for Raw {
    type Out = Vec<u8>;
    type In<'a> = &'a [u8];
    const NT_TYPE: i32 = NT_RAW;

    fn type_string() -> String {
        Self::TYPE_STRING.to_owned()
    }

    fn set(handle: Handle, time: i64, value: &[u8]) -> bool {
        // SAFETY: the pointer and length describe value, which outlives the call.
        ok(unsafe { (core().NT_SetRaw)(handle, time, value.as_ptr(), value.len()) })
    }

    fn set_default(handle: Handle, value: &[u8]) -> bool {
        // SAFETY: as in `set`.
        ok(unsafe { (core().NT_SetDefaultRaw)(handle, value.as_ptr(), value.len()) })
    }

    fn from_value(value: &Value) -> Option<Vec<u8>> {
        match value {
            Value::Raw(b) => Some(b.clone()),
            _ => None,
        }
    }

    fn as_in(value: &Vec<u8>) -> &[u8] {
        value
    }
}

macro_rules! array_kind {
    ($name:ident, $elem:ty, $nt:expr, $text:literal, $set:ident, $default:ident, $variant:ident) => {
        pub struct $name;

        impl $name {
            pub const TYPE_STRING: &'static str = $text;
        }

        impl Kind for $name {
            type Out = Vec<$elem>;
            type In<'a> = &'a [$elem];
            const NT_TYPE: i32 = $nt;

            fn type_string() -> String {
                $text.to_owned()
            }

            fn set(handle: Handle, time: i64, value: &[$elem]) -> bool {
                // SAFETY: the pointer and length describe value, which outlives the call.
                ok(unsafe { (core().$set)(handle, time, value.as_ptr(), value.len()) })
            }

            fn set_default(handle: Handle, value: &[$elem]) -> bool {
                // SAFETY: as in `set`.
                ok(unsafe { (core().$default)(handle, value.as_ptr(), value.len()) })
            }

            fn from_value(value: &Value) -> Option<Vec<$elem>> {
                match value {
                    Value::$variant(v) => Some(v.clone()),
                    _ => None,
                }
            }

            fn as_in(value: &Vec<$elem>) -> &[$elem] {
                value
            }
        }
    };
}

array_kind!(
    IntegerArray,
    i64,
    NT_INTEGER_ARRAY,
    "int[]",
    NT_SetIntegerArray,
    NT_SetDefaultIntegerArray,
    IntegerArray
);

pub struct DoubleArray;

impl DoubleArray {
    pub const TYPE_STRING: &'static str = "double[]";
}

impl Kind for DoubleArray {
    type Out = Vec<f64>;
    type In<'a> = &'a [f64];
    const NT_TYPE: i32 = NT_DOUBLE_ARRAY;

    fn type_string() -> String {
        Self::TYPE_STRING.to_owned()
    }

    fn set(handle: Handle, time: i64, value: &[f64]) -> bool {
        // SAFETY: the pointer and length describe value, which outlives the call.
        ok(unsafe { (core().NT_SetDoubleArray)(handle, time, value.as_ptr(), value.len()) })
    }

    fn set_default(handle: Handle, value: &[f64]) -> bool {
        // SAFETY: as in `set`.
        ok(unsafe { (core().NT_SetDefaultDoubleArray)(handle, value.as_ptr(), value.len()) })
    }

    fn from_value(value: &Value) -> Option<Vec<f64>> {
        match value {
            Value::DoubleArray(v) => Some(v.clone()),
            Value::IntegerArray(v) => Some(v.iter().map(|i| *i as f64).collect()),
            _ => None,
        }
    }

    fn as_in(value: &Vec<f64>) -> &[f64] {
        value
    }
}

pub struct BooleanArray;

impl BooleanArray {
    pub const TYPE_STRING: &'static str = "boolean[]";
}

impl Kind for BooleanArray {
    type Out = Vec<bool>;
    type In<'a> = &'a [bool];
    const NT_TYPE: i32 = NT_BOOLEAN_ARRAY;

    fn type_string() -> String {
        Self::TYPE_STRING.to_owned()
    }

    fn set(handle: Handle, time: i64, value: &[bool]) -> bool {
        let ints: Vec<i32> = value.iter().map(|b| i32::from(*b)).collect();
        // SAFETY: the pointer and length describe ints, which outlives the call.
        ok(unsafe { (core().NT_SetBooleanArray)(handle, time, ints.as_ptr(), ints.len()) })
    }

    fn set_default(handle: Handle, value: &[bool]) -> bool {
        let ints: Vec<i32> = value.iter().map(|b| i32::from(*b)).collect();
        // SAFETY: as in `set`.
        ok(unsafe { (core().NT_SetDefaultBooleanArray)(handle, ints.as_ptr(), ints.len()) })
    }

    fn from_value(value: &Value) -> Option<Vec<bool>> {
        match value {
            Value::BooleanArray(v) => Some(v.clone()),
            _ => None,
        }
    }

    fn as_in(value: &Vec<bool>) -> &[bool] {
        value
    }
}

pub struct StringArray;

impl StringArray {
    pub const TYPE_STRING: &'static str = "string[]";
}

impl Kind for StringArray {
    type Out = Vec<String>;
    type In<'a> = &'a [String];
    const NT_TYPE: i32 = NT_STRING_ARRAY;

    fn type_string() -> String {
        Self::TYPE_STRING.to_owned()
    }

    fn set(handle: Handle, time: i64, value: &[String]) -> bool {
        let texts: Vec<WpiStr> = value.iter().map(|s| WpiStr::borrow(s)).collect();
        // SAFETY: texts borrows value, and both outlive the call; ntcore copies the strings.
        ok(unsafe { (core().NT_SetStringArray)(handle, time, texts.as_ptr(), texts.len()) })
    }

    fn set_default(handle: Handle, value: &[String]) -> bool {
        let texts: Vec<WpiStr> = value.iter().map(|s| WpiStr::borrow(s)).collect();
        // SAFETY: as in `set`.
        ok(unsafe { (core().NT_SetDefaultStringArray)(handle, texts.as_ptr(), texts.len()) })
    }

    fn from_value(value: &Value) -> Option<Vec<String>> {
        match value {
            Value::StringArray(v) => Some(v.clone()),
            _ => None,
        }
    }

    fn as_in(value: &Vec<String>) -> &[String] {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type_strings_match_java() {
        assert_eq!(Value::Boolean(true).type_string(), "boolean");
        assert_eq!(Value::Integer(1).type_string(), "int");
        assert_eq!(Value::Double(1.0).type_string(), "double");
        assert_eq!(Value::String(String::new()).type_string(), "string");
        assert_eq!(Value::Raw(vec![]).type_string(), "raw");
        assert_eq!(Value::BooleanArray(vec![]).type_string(), "boolean[]");
        assert_eq!(Value::IntegerArray(vec![]).type_string(), "int[]");
        assert_eq!(Value::DoubleArray(vec![]).type_string(), "double[]");
        assert_eq!(Value::StringArray(vec![]).type_string(), "string[]");
    }

    #[test]
    fn double_kinds_accept_integers() {
        assert_eq!(Double::from_value(&Value::Integer(3)), Some(3.0));
        assert_eq!(
            DoubleArray::from_value(&Value::IntegerArray(vec![1, 2])),
            Some(vec![1.0, 2.0])
        );
        assert_eq!(Double::from_value(&Value::String("x".into())), None);
    }

    #[test]
    fn from_nt_copies_each_type() {
        let mut v = NtValue::unassigned();
        v.ty = NT_DOUBLE;
        v.data.v_double = 2.5;
        assert_eq!(Value::from_nt(&v), Some(Value::Double(2.5)));
        v.ty = NT_FLOAT;
        v.data.v_float = 1.5;
        assert_eq!(Value::from_nt(&v), Some(Value::Double(1.5)));
        v.ty = NT_BOOLEAN;
        v.data.v_boolean = 1;
        assert_eq!(Value::from_nt(&v), Some(Value::Boolean(true)));

        let mut doubles = [1.0, 2.0];
        v.ty = NT_DOUBLE_ARRAY;
        v.data.arr_double = RawArray {
            ptr: doubles.as_mut_ptr(),
            size: 2,
        };
        assert_eq!(Value::from_nt(&v), Some(Value::DoubleArray(vec![1.0, 2.0])));

        let text = "hi";
        v.ty = NT_STRING;
        v.data.v_string = WpiStr::borrow(text);
        assert_eq!(Value::from_nt(&v), Some(Value::String("hi".into())));

        v.ty = 0;
        assert_eq!(Value::from_nt(&v), None);
    }

    #[test]
    fn empty_arrays_may_have_null_pointers() {
        let mut v = NtValue::unassigned();
        v.ty = NT_STRING_ARRAY;
        v.data.arr_string = RawArray {
            ptr: std::ptr::null_mut(),
            size: 0,
        };
        assert_eq!(Value::from_nt(&v), Some(Value::StringArray(vec![])));
    }
}
