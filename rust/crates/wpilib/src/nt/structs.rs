//! WPILib struct serialization: fixed-size little-endian records that AdvantageScope and
//! Elastic decode from the published schema.
//!
//! Byte layouts follow the `*Struct` classes under `wpimath/.../struct` in WPILib v2026.2.1.

use super::Instance;
use super::value::{Kind, Raw, Value};
use std::marker::PhantomData;
use wpimath::geometry::{
    Pose2d, Pose3d, Quaternion, Rotation2d, Rotation3d, Transform2d, Transform3d, Translation2d,
    Translation3d, Twist2d,
};
use wpimath::kinematics::{ChassisSpeeds, SwerveModulePosition, SwerveModuleState};

/// A type with a fixed-size binary form.
pub trait Struct: Sized + Clone + 'static {
    const TYPE_NAME: &'static str;
    const SIZE: usize;
    const SCHEMA: &'static str;

    /// Appends exactly `SIZE` bytes.
    fn pack(&self, out: &mut Vec<u8>);

    /// Reads from the start of `bytes`, which holds at least `SIZE` bytes.
    fn unpack(bytes: &[u8]) -> Self;

    /// Reports this type's schema, then those of the types it contains.
    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
    }

    fn type_string() -> String {
        format!("struct:{}", Self::TYPE_NAME)
    }
}

fn put(out: &mut Vec<u8>, value: f64) {
    out.extend_from_slice(&value.to_le_bytes());
}

/// The `index`th double of a record.
fn get(bytes: &[u8], index: usize) -> f64 {
    let at = index * 8;
    let mut word = [0u8; 8];
    word.copy_from_slice(&bytes[at..at + 8]);
    f64::from_le_bytes(word)
}

impl Struct for Rotation2d {
    const TYPE_NAME: &'static str = "Rotation2d";
    const SIZE: usize = 8;
    const SCHEMA: &'static str = "double value";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.radians());
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self::new(get(bytes, 0))
    }
}

impl Struct for Translation2d {
    const TYPE_NAME: &'static str = "Translation2d";
    const SIZE: usize = 16;
    const SCHEMA: &'static str = "double x;double y";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.x);
        put(out, self.y);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self::new(get(bytes, 0), get(bytes, 1))
    }
}

impl Struct for Pose2d {
    const TYPE_NAME: &'static str = "Pose2d";
    const SIZE: usize = Translation2d::SIZE + Rotation2d::SIZE;
    const SCHEMA: &'static str = "Translation2d translation;Rotation2d rotation";

    fn pack(&self, out: &mut Vec<u8>) {
        self.translation.pack(out);
        self.rotation.pack(out);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self {
            translation: Translation2d::unpack(bytes),
            rotation: Rotation2d::unpack(&bytes[Translation2d::SIZE..]),
        }
    }

    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
        Translation2d::add_schemas(add);
        Rotation2d::add_schemas(add);
    }
}

impl Struct for Transform2d {
    const TYPE_NAME: &'static str = "Transform2d";
    const SIZE: usize = Translation2d::SIZE + Rotation2d::SIZE;
    const SCHEMA: &'static str = "Translation2d translation;Rotation2d rotation";

    fn pack(&self, out: &mut Vec<u8>) {
        self.translation.pack(out);
        self.rotation.pack(out);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self {
            translation: Translation2d::unpack(bytes),
            rotation: Rotation2d::unpack(&bytes[Translation2d::SIZE..]),
        }
    }

    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
        Translation2d::add_schemas(add);
        Rotation2d::add_schemas(add);
    }
}

impl Struct for Twist2d {
    const TYPE_NAME: &'static str = "Twist2d";
    const SIZE: usize = 24;
    const SCHEMA: &'static str = "double dx;double dy;double dtheta";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.dx);
        put(out, self.dy);
        put(out, self.dtheta);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self::new(get(bytes, 0), get(bytes, 1), get(bytes, 2))
    }
}

impl Struct for Translation3d {
    const TYPE_NAME: &'static str = "Translation3d";
    const SIZE: usize = 24;
    const SCHEMA: &'static str = "double x;double y;double z";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.x);
        put(out, self.y);
        put(out, self.z);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self::new(get(bytes, 0), get(bytes, 1), get(bytes, 2))
    }
}

impl Struct for Quaternion {
    const TYPE_NAME: &'static str = "Quaternion";
    const SIZE: usize = 32;
    const SCHEMA: &'static str = "double w;double x;double y;double z";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.w);
        put(out, self.x);
        put(out, self.y);
        put(out, self.z);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self {
            w: get(bytes, 0),
            x: get(bytes, 1),
            y: get(bytes, 2),
            z: get(bytes, 3),
        }
    }
}

impl Struct for Rotation3d {
    const TYPE_NAME: &'static str = "Rotation3d";
    const SIZE: usize = Quaternion::SIZE;
    const SCHEMA: &'static str = "Quaternion q";

    fn pack(&self, out: &mut Vec<u8>) {
        self.quaternion().pack(out);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self::from_quaternion(Quaternion::unpack(bytes))
    }

    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
        Quaternion::add_schemas(add);
    }
}

impl Struct for Pose3d {
    const TYPE_NAME: &'static str = "Pose3d";
    const SIZE: usize = Translation3d::SIZE + Rotation3d::SIZE;
    const SCHEMA: &'static str = "Translation3d translation;Rotation3d rotation";

    fn pack(&self, out: &mut Vec<u8>) {
        self.translation.pack(out);
        self.rotation.pack(out);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self {
            translation: Translation3d::unpack(bytes),
            rotation: Rotation3d::unpack(&bytes[Translation3d::SIZE..]),
        }
    }

    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
        Translation3d::add_schemas(add);
        Rotation3d::add_schemas(add);
    }
}

impl Struct for Transform3d {
    const TYPE_NAME: &'static str = "Transform3d";
    const SIZE: usize = Translation3d::SIZE + Rotation3d::SIZE;
    const SCHEMA: &'static str = "Translation3d translation;Rotation3d rotation";

    fn pack(&self, out: &mut Vec<u8>) {
        self.translation.pack(out);
        self.rotation.pack(out);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self {
            translation: Translation3d::unpack(bytes),
            rotation: Rotation3d::unpack(&bytes[Translation3d::SIZE..]),
        }
    }

    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
        Translation3d::add_schemas(add);
        Rotation3d::add_schemas(add);
    }
}

impl Struct for ChassisSpeeds {
    const TYPE_NAME: &'static str = "ChassisSpeeds";
    const SIZE: usize = 24;
    const SCHEMA: &'static str = "double vx;double vy;double omega";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.vx);
        put(out, self.vy);
        put(out, self.omega);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self::new(get(bytes, 0), get(bytes, 1), get(bytes, 2))
    }
}

impl Struct for SwerveModuleState {
    const TYPE_NAME: &'static str = "SwerveModuleState";
    const SIZE: usize = 8 + Rotation2d::SIZE;
    const SCHEMA: &'static str = "double speed;Rotation2d angle";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.speed);
        self.angle.pack(out);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self::new(get(bytes, 0), Rotation2d::unpack(&bytes[8..]))
    }

    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
        Rotation2d::add_schemas(add);
    }
}

impl Struct for SwerveModulePosition {
    const TYPE_NAME: &'static str = "SwerveModulePosition";
    const SIZE: usize = 8 + Rotation2d::SIZE;
    const SCHEMA: &'static str = "double distance;Rotation2d angle";

    fn pack(&self, out: &mut Vec<u8>) {
        put(out, self.distance);
        self.angle.pack(out);
    }

    fn unpack(bytes: &[u8]) -> Self {
        Self {
            distance: get(bytes, 0),
            angle: Rotation2d::unpack(&bytes[8..]),
        }
    }

    fn add_schemas(add: &mut dyn FnMut(&str, &str)) {
        add(Self::TYPE_NAME, Self::SCHEMA);
        Rotation2d::add_schemas(add);
    }
}

/// Packs `values` back to back, as Java's `StructArrayPublisher` does.
pub fn pack_array<S: Struct>(values: &[S]) -> Vec<u8> {
    let mut out = Vec::with_capacity(values.len() * S::SIZE);
    for value in values {
        value.pack(&mut out);
    }
    out
}

/// `None` when the length is not a whole number of records.
pub fn unpack_array<S: Struct>(bytes: &[u8]) -> Option<Vec<S>> {
    if !bytes.len().is_multiple_of(S::SIZE) {
        return None;
    }
    Some(bytes.chunks_exact(S::SIZE).map(S::unpack).collect())
}

impl Instance {
    /// Registers `S` and the structs it contains. Repeats are ignored by ntcore.
    pub fn add_struct_schemas<S: Struct>(self) {
        S::add_schemas(&mut |name, schema| {
            self.add_schema(&format!("struct:{name}"), "structschema", schema.as_bytes());
        });
    }
}

/// One struct, sent as raw bytes with type string `struct:<Name>`.
pub struct StructKind<S>(PhantomData<fn() -> S>);

impl<S: Struct> Kind for StructKind<S> {
    type Out = S;
    type In<'a> = &'a S;
    const NT_TYPE: i32 = Raw::NT_TYPE;

    fn type_string() -> String {
        S::type_string()
    }

    fn set(handle: u32, time: i64, value: &S) -> bool {
        let mut bytes = Vec::with_capacity(S::SIZE);
        value.pack(&mut bytes);
        Raw::set(handle, time, &bytes)
    }

    fn set_default(handle: u32, value: &S) -> bool {
        let mut bytes = Vec::with_capacity(S::SIZE);
        value.pack(&mut bytes);
        Raw::set_default(handle, &bytes)
    }

    fn from_value(value: &Value) -> Option<S> {
        match value {
            Value::Raw(bytes) if bytes.len() == S::SIZE => Some(S::unpack(bytes)),
            _ => None,
        }
    }

    fn as_in(value: &S) -> &S {
        value
    }

    fn prepare(inst: Instance) {
        inst.add_struct_schemas::<S>();
    }
}

/// An array of structs, sent as concatenated records with type string `struct:<Name>[]`.
pub struct StructArrayKind<S>(PhantomData<fn() -> S>);

impl<S: Struct> Kind for StructArrayKind<S> {
    type Out = Vec<S>;
    type In<'a> = &'a [S];
    const NT_TYPE: i32 = Raw::NT_TYPE;

    fn type_string() -> String {
        format!("{}[]", S::type_string())
    }

    fn set(handle: u32, time: i64, value: &[S]) -> bool {
        Raw::set(handle, time, &pack_array(value))
    }

    fn set_default(handle: u32, value: &[S]) -> bool {
        Raw::set_default(handle, &pack_array(value))
    }

    fn from_value(value: &Value) -> Option<Vec<S>> {
        match value {
            Value::Raw(bytes) => unpack_array(bytes),
            _ => None,
        }
    }

    fn as_in(value: &Vec<S>) -> &[S] {
        value
    }

    fn prepare(inst: Instance) {
        inst.add_struct_schemas::<S>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bytes<S: Struct>(value: &S) -> Vec<u8> {
        let mut out = Vec::new();
        value.pack(&mut out);
        out
    }

    // IEEE 754 little-endian: 1.0, 2.0, 0.5, -1.0
    const ONE: [u8; 8] = [0, 0, 0, 0, 0, 0, 0xF0, 0x3F];
    const TWO: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0x40];
    const HALF: [u8; 8] = [0, 0, 0, 0, 0, 0, 0xE0, 0x3F];
    const NEG_ONE: [u8; 8] = [0, 0, 0, 0, 0, 0, 0xF0, 0xBF];
    const ZERO: [u8; 8] = [0; 8];

    fn concat(parts: &[&[u8]]) -> Vec<u8> {
        parts.concat()
    }

    #[test]
    fn rotation2d_packs_radians() {
        assert_eq!(bytes(&Rotation2d::new(0.5)), HALF);
    }

    #[test]
    fn translation2d_packs_x_then_y() {
        assert_eq!(bytes(&Translation2d::new(1.0, 2.0)), concat(&[&ONE, &TWO]));
    }

    #[test]
    fn pose2d_packs_translation_then_rotation() {
        let pose = Pose2d::from_xy(1.0, 2.0, Rotation2d::new(0.5));
        assert_eq!(bytes(&pose), concat(&[&ONE, &TWO, &HALF]));
        assert_eq!(Pose2d::SIZE, 24);
    }

    #[test]
    fn transform2d_and_twist2d() {
        let transform = Transform2d::from_xy(1.0, 2.0, Rotation2d::new(0.5));
        assert_eq!(bytes(&transform), concat(&[&ONE, &TWO, &HALF]));
        assert_eq!(
            bytes(&Twist2d::new(1.0, 2.0, 0.5)),
            concat(&[&ONE, &TWO, &HALF])
        );
    }

    #[test]
    fn translation3d_packs_xyz() {
        assert_eq!(
            bytes(&Translation3d::new(1.0, 2.0, 0.5)),
            concat(&[&ONE, &TWO, &HALF])
        );
    }

    #[test]
    fn quaternion_packs_wxyz() {
        let q = Quaternion {
            w: 1.0,
            x: 2.0,
            y: 0.5,
            z: -1.0,
        };
        assert_eq!(bytes(&q), concat(&[&ONE, &TWO, &HALF, &NEG_ONE]));
    }

    #[test]
    fn identity_rotation3d_is_w_one() {
        assert_eq!(
            bytes(&Rotation3d::ZERO),
            concat(&[&ONE, &ZERO, &ZERO, &ZERO])
        );
    }

    #[test]
    fn pose3d_is_translation_then_quaternion() {
        let pose = Pose3d {
            translation: Translation3d::new(1.0, 2.0, 0.5),
            rotation: Rotation3d::ZERO,
        };
        assert_eq!(
            bytes(&pose),
            concat(&[&ONE, &TWO, &HALF, &ONE, &ZERO, &ZERO, &ZERO])
        );
        assert_eq!(Pose3d::SIZE, 56);
        assert_eq!(Transform3d::SIZE, 56);
    }

    #[test]
    fn chassis_speeds_and_swerve_types() {
        assert_eq!(
            bytes(&ChassisSpeeds::new(1.0, 2.0, 0.5)),
            concat(&[&ONE, &TWO, &HALF])
        );
        assert_eq!(
            bytes(&SwerveModuleState::new(1.0, Rotation2d::new(0.5))),
            concat(&[&ONE, &HALF])
        );
        assert_eq!(
            bytes(&SwerveModulePosition {
                distance: 2.0,
                angle: Rotation2d::new(1.0)
            }),
            concat(&[&TWO, &ONE])
        );
    }

    #[test]
    fn round_trips() {
        let pose = Pose2d::from_xy(1.5, -2.5, Rotation2d::new(0.25));
        assert_eq!(Pose2d::unpack(&bytes(&pose)), pose);
        let pose3 = Pose3d {
            translation: Translation3d::new(1.0, 2.0, 3.0),
            rotation: Rotation3d::from_euler(0.1, 0.2, 0.3),
        };
        let back = Pose3d::unpack(&bytes(&pose3));
        assert!((back.rotation.quaternion().w - pose3.rotation.quaternion().w).abs() < 1e-12);
        assert_eq!(back.translation, pose3.translation);
        let state = SwerveModuleState::new(3.0, Rotation2d::new(1.0));
        assert_eq!(SwerveModuleState::unpack(&bytes(&state)), state);
    }

    #[test]
    fn arrays_are_concatenated_records() {
        let poses = [Translation2d::new(1.0, 2.0), Translation2d::new(2.0, 1.0)];
        assert_eq!(pack_array(&poses), concat(&[&ONE, &TWO, &TWO, &ONE]));
        assert_eq!(
            unpack_array::<Translation2d>(&pack_array(&poses)),
            Some(poses.to_vec())
        );
        assert_eq!(unpack_array::<Translation2d>(&[0u8; 17]), None);
        assert_eq!(unpack_array::<Translation2d>(&[]), Some(vec![]));
    }

    #[test]
    fn struct_kinds_decode_exact_sizes_only() {
        let raw = Value::Raw(bytes(&Translation2d::new(1.0, 2.0)));
        assert_eq!(
            StructKind::<Translation2d>::from_value(&raw),
            Some(Translation2d::new(1.0, 2.0))
        );
        assert_eq!(
            StructKind::<Translation2d>::from_value(&Value::Raw(vec![0; 15])),
            None
        );
        assert_eq!(
            StructKind::<Translation2d>::from_value(&Value::Double(1.0)),
            None
        );
        assert_eq!(
            StructArrayKind::<Translation2d>::from_value(&Value::Raw(vec![0; 32])),
            Some(vec![Translation2d::ZERO; 2])
        );
    }

    #[test]
    fn type_strings() {
        assert_eq!(StructKind::<Pose2d>::type_string(), "struct:Pose2d");
        assert_eq!(
            StructArrayKind::<Translation3d>::type_string(),
            "struct:Translation3d[]"
        );
    }

    fn schemas<S: Struct>() -> Vec<(String, String)> {
        let mut out = Vec::new();
        S::add_schemas(&mut |n, s| out.push((n.to_owned(), s.to_owned())));
        out
    }

    #[test]
    fn schemas_list_self_then_nested() {
        let names: Vec<_> = schemas::<Pose3d>().into_iter().map(|(n, _)| n).collect();
        assert_eq!(
            names,
            ["Pose3d", "Translation3d", "Rotation3d", "Quaternion"]
        );
        let all = schemas::<Pose2d>();
        assert_eq!(
            all[0],
            (
                "Pose2d".to_owned(),
                "Translation2d translation;Rotation2d rotation".to_owned()
            )
        );
        assert_eq!(all[1].1, "double x;double y");
        assert_eq!(all[2].1, "double value");
        assert_eq!(schemas::<SwerveModuleState>().len(), 2);
    }
}
