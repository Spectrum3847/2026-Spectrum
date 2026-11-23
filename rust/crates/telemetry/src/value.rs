//! What the logger queues: a typed value, its key and its timestamp.

use crate::wpilog::payload;
use std::borrow::Cow;
use wpilib::nt::Struct;

/// Reports a struct type's schema, then those of the types it contains.
pub type AddSchemas = fn(&mut dyn FnMut(&str, &str));

/// What the log thread needs to register a struct type.
#[derive(Clone, Copy)]
pub struct StructInfo {
    pub type_name: &'static str,
    pub add_schemas: AddSchemas,
}

impl StructInfo {
    pub fn of<S: Struct>() -> Self {
        Self {
            type_name: S::TYPE_NAME,
            add_schemas: S::add_schemas,
        }
    }
}

#[derive(Clone)]
pub enum Value {
    Boolean(bool),
    BooleanArray(Vec<bool>),
    Double(f64),
    DoubleArray(Vec<f64>),
    Integer(i64),
    IntegerArray(Vec<i64>),
    String(String),
    StringArray(Vec<String>),
    /// A string with its own type string, such as `json`.
    CustomString {
        value: String,
        type_name: String,
    },
    Struct {
        info: StructInfo,
        bytes: Vec<u8>,
    },
    StructArray {
        info: StructInfo,
        bytes: Vec<u8>,
    },
}

impl Value {
    /// The wpilog type string.
    pub fn log_type(&self) -> Cow<'_, str> {
        Cow::Borrowed(match self {
            Self::Boolean(_) => payload::BOOLEAN,
            Self::BooleanArray(_) => payload::BOOLEAN_ARRAY,
            Self::Double(_) => payload::DOUBLE,
            Self::DoubleArray(_) => payload::DOUBLE_ARRAY,
            Self::Integer(_) => payload::INT64,
            Self::IntegerArray(_) => payload::INT64_ARRAY,
            Self::String(_) => payload::STRING,
            Self::StringArray(_) => payload::STRING_ARRAY,
            Self::CustomString { type_name, .. } => return Cow::Borrowed(type_name),
            Self::Struct { info, .. } => return Cow::Owned(format!("struct:{}", info.type_name)),
            Self::StructArray { info, .. } => {
                return Cow::Owned(format!("struct:{}[]", info.type_name));
            }
        })
    }

    /// The NetworkTables type string. NetworkTables calls integers `int`.
    pub fn nt_type(&self) -> Cow<'_, str> {
        match self {
            Self::Integer(_) => Cow::Borrowed("int"),
            Self::IntegerArray(_) => Cow::Borrowed("int[]"),
            other => other.log_type(),
        }
    }

    /// Appends the wpilog payload.
    pub fn encode(&self, out: &mut Vec<u8>) {
        match self {
            Self::Boolean(v) => payload::boolean(out, *v),
            Self::BooleanArray(v) => payload::boolean_array(out, v),
            Self::Double(v) => payload::double(out, *v),
            Self::DoubleArray(v) => payload::double_array(out, v),
            Self::Integer(v) => payload::int64(out, *v),
            Self::IntegerArray(v) => payload::int64_array(out, v),
            Self::String(v) => payload::string(out, v),
            Self::StringArray(v) => payload::string_array(out, v),
            Self::CustomString { value, .. } => payload::string(out, value),
            Self::Struct { bytes, .. } | Self::StructArray { bytes, .. } => {
                out.extend_from_slice(bytes);
            }
        }
    }

    /// Whether a unit applies: DogLog attaches units to numbers and number arrays only.
    pub fn takes_unit(&self) -> bool {
        matches!(
            self,
            Self::Double(_) | Self::DoubleArray(_) | Self::Integer(_) | Self::IntegerArray(_)
        )
    }
}

/// One queued log call.
#[derive(Clone)]
pub struct Entry {
    pub key: String,
    /// Publish to NetworkTables even when the NetworkTables switch is off.
    pub force_nt: bool,
    /// FPGA microseconds.
    pub timestamp: u64,
    pub value: Value,
    pub unit: Option<String>,
}

/// Values `log` accepts.
pub trait LogValue {
    fn into_value(self) -> Value;
}

macro_rules! scalar {
    ($ty:ty, |$v:ident| $conv:expr) => {
        impl LogValue for $ty {
            fn into_value(self) -> Value {
                let $v = self;
                $conv
            }
        }
    };
}

scalar!(bool, |v| Value::Boolean(v));
scalar!(f64, |v| Value::Double(v));
scalar!(i64, |v| Value::Integer(v));
scalar!(i32, |v| Value::Integer(i64::from(v)));
scalar!(u32, |v| Value::Integer(i64::from(v)));
scalar!(usize, |v| Value::Integer(v as i64));
scalar!(&str, |v| Value::String(v.to_owned()));
scalar!(String, |v| Value::String(v));
scalar!(&String, |v| Value::String(v.clone()));
scalar!(&[bool], |v| Value::BooleanArray(v.to_vec()));
scalar!(&[f64], |v| Value::DoubleArray(v.to_vec()));
scalar!(&[i64], |v| Value::IntegerArray(v.to_vec()));
scalar!(&[String], |v| Value::StringArray(v.to_vec()));
scalar!(&[&str], |v| Value::StringArray(
    v.iter().map(|s| (*s).to_owned()).collect()
));
scalar!(&Vec<bool>, |v| Value::BooleanArray(v.clone()));
scalar!(&Vec<f64>, |v| Value::DoubleArray(v.clone()));
scalar!(&Vec<i64>, |v| Value::IntegerArray(v.clone()));
scalar!(&Vec<String>, |v| Value::StringArray(v.clone()));
scalar!(Vec<f64>, |v| Value::DoubleArray(v));
scalar!(Vec<bool>, |v| Value::BooleanArray(v));
scalar!(Vec<i64>, |v| Value::IntegerArray(v));
scalar!(Vec<String>, |v| Value::StringArray(v));

impl<const N: usize> LogValue for &[f64; N] {
    fn into_value(self) -> Value {
        Value::DoubleArray(self.to_vec())
    }
}

impl<const N: usize> LogValue for &[bool; N] {
    fn into_value(self) -> Value {
        Value::BooleanArray(self.to_vec())
    }
}

impl<const N: usize> LogValue for &[&str; N] {
    fn into_value(self) -> Value {
        Value::StringArray(self.iter().map(|s| (*s).to_owned()).collect())
    }
}

/// A struct value in its binary form.
pub fn struct_value<S: Struct>(value: &S) -> Value {
    let mut bytes = Vec::with_capacity(S::SIZE);
    value.pack(&mut bytes);
    Value::Struct {
        info: StructInfo::of::<S>(),
        bytes,
    }
}

/// An array of structs, packed back to back.
pub fn struct_array_value<S: Struct>(values: &[S]) -> Value {
    let mut bytes = Vec::with_capacity(S::SIZE * values.len());
    for v in values {
        v.pack(&mut bytes);
    }
    Value::StructArray {
        info: StructInfo::of::<S>(),
        bytes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wpimath::geometry::{Pose2d, Rotation2d};

    #[test]
    fn type_strings() {
        assert_eq!(Value::Double(1.0).log_type(), "double");
        assert_eq!(Value::Integer(1).log_type(), "int64");
        assert_eq!(Value::Integer(1).nt_type(), "int");
        assert_eq!(Value::IntegerArray(vec![]).nt_type(), "int[]");
        assert_eq!(Value::StringArray(vec![]).log_type(), "string[]");
        let json = Value::CustomString {
            value: "{}".into(),
            type_name: "json".into(),
        };
        assert_eq!(json.log_type(), "json");
        assert_eq!(json.nt_type(), "json");
        let pose = struct_value(&Pose2d::from_xy(1.0, 2.0, Rotation2d::new(0.5)));
        assert_eq!(pose.log_type(), "struct:Pose2d");
        assert_eq!(pose.nt_type(), "struct:Pose2d");
        let poses = struct_array_value(&[Pose2d::default(), Pose2d::default()]);
        assert_eq!(poses.log_type(), "struct:Pose2d[]");
        let Value::StructArray { bytes, .. } = poses else {
            panic!("not an array");
        };
        assert_eq!(bytes.len(), 48);
    }

    #[test]
    fn conversions() {
        assert!(matches!(3i32.into_value(), Value::Integer(3)));
        assert!(matches!(true.into_value(), Value::Boolean(true)));
        assert!(matches!("a".into_value(), Value::String(s) if s == "a"));
        assert!(
            matches!([1.0, 2.0].as_slice().into_value(), Value::DoubleArray(v) if v.len() == 2)
        );
        assert!(matches!((&[1.0, 2.0]).into_value(), Value::DoubleArray(v) if v.len() == 2));
        assert!(matches!((&["a", "b"]).into_value(), Value::StringArray(v) if v[1] == "b"));
    }

    #[test]
    fn encoding_and_units() {
        let mut out = Vec::new();
        Value::IntegerArray(vec![1, 2]).encode(&mut out);
        assert_eq!(out.len(), 16);
        assert!(Value::Double(0.0).takes_unit());
        assert!(!Value::Boolean(true).takes_unit());
        assert!(!Value::String(String::new()).takes_unit());
    }
}
