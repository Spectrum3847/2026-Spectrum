//! Generated from the Phoenix 6 26.3.0 Java enums (com.ctre.phoenix6.signals). Variant names
//! match Java so ported code reads the same.
#![allow(non_camel_case_types, clippy::enum_variant_names)]

use crate::configs::{ConfigField, ConfigSource, ConfigValue};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AnimationDirectionValue {
    Forward = 0,
    Backward = 1,
}

impl AnimationDirectionValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Backward,
            _ => Self::Forward,
        }
    }
}

impl ConfigField for AnimationDirectionValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DifferentialSensorSourceValue {
    Disabled = 0,
    RemoteTalonFX_HalfDiff = 1,
    RemotePigeon2Yaw = 2,
    RemotePigeon2Pitch = 3,
    RemotePigeon2Roll = 4,
    RemoteCANcoder = 5,
    RemoteCANdiPWM1 = 6,
    RemoteCANdiPWM2 = 7,
    RemoteCANdiQuadrature = 8,
}

impl DifferentialSensorSourceValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::RemoteTalonFX_HalfDiff,
            2 => Self::RemotePigeon2Yaw,
            3 => Self::RemotePigeon2Pitch,
            4 => Self::RemotePigeon2Roll,
            5 => Self::RemoteCANcoder,
            6 => Self::RemoteCANdiPWM1,
            7 => Self::RemoteCANdiPWM2,
            8 => Self::RemoteCANdiQuadrature,
            _ => Self::Disabled,
        }
    }
}

impl ConfigField for DifferentialSensorSourceValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Enable5VRailValue {
    Enabled = 0,
    Disabled = 1,
}

impl Enable5VRailValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Disabled,
            _ => Self::Enabled,
        }
    }
}

impl ConfigField for Enable5VRailValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FeedbackSensorSourceValue {
    RotorSensor = 0,
    RemoteCANcoder = 1,
    RemotePigeon2Yaw = 2,
    RemotePigeon2Pitch = 3,
    RemotePigeon2Roll = 4,
    FusedCANcoder = 5,
    SyncCANcoder = 6,
    RemoteCANdiPWM1 = 9,
    RemoteCANdiPWM2 = 10,
    RemoteCANdiQuadrature = 11,
    FusedCANdiPWM1 = 12,
    FusedCANdiPWM2 = 13,
    FusedCANdiQuadrature = 14,
    SyncCANdiPWM1 = 15,
    SyncCANdiPWM2 = 16,
}

impl FeedbackSensorSourceValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::RemoteCANcoder,
            2 => Self::RemotePigeon2Yaw,
            3 => Self::RemotePigeon2Pitch,
            4 => Self::RemotePigeon2Roll,
            5 => Self::FusedCANcoder,
            6 => Self::SyncCANcoder,
            9 => Self::RemoteCANdiPWM1,
            10 => Self::RemoteCANdiPWM2,
            11 => Self::RemoteCANdiQuadrature,
            12 => Self::FusedCANdiPWM1,
            13 => Self::FusedCANdiPWM2,
            14 => Self::FusedCANdiQuadrature,
            15 => Self::SyncCANdiPWM1,
            16 => Self::SyncCANdiPWM2,
            _ => Self::RotorSensor,
        }
    }
}

impl ConfigField for FeedbackSensorSourceValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ForwardLimitSourceValue {
    LimitSwitchPin = 0,
    RemoteTalonFX = 1,
    RemoteCANifier = 2,
    RemoteCANcoder = 4,
    RemoteCANrange = 6,
    RemoteCANdiS1 = 7,
    RemoteCANdiS2 = 8,
    Disabled = 3,
}

impl ForwardLimitSourceValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::RemoteTalonFX,
            2 => Self::RemoteCANifier,
            4 => Self::RemoteCANcoder,
            6 => Self::RemoteCANrange,
            7 => Self::RemoteCANdiS1,
            8 => Self::RemoteCANdiS2,
            3 => Self::Disabled,
            _ => Self::LimitSwitchPin,
        }
    }
}

impl ConfigField for ForwardLimitSourceValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ForwardLimitTypeValue {
    NormallyOpen = 0,
    NormallyClosed = 1,
}

impl ForwardLimitTypeValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::NormallyClosed,
            _ => Self::NormallyOpen,
        }
    }
}

impl ConfigField for ForwardLimitTypeValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GainSchedBehaviorValue {
    Inactive = 0,
    ZeroOutput = 1,
    UseSlot0 = 2,
    UseSlot1 = 3,
    UseSlot2 = 4,
}

impl GainSchedBehaviorValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::ZeroOutput,
            2 => Self::UseSlot0,
            3 => Self::UseSlot1,
            4 => Self::UseSlot2,
            _ => Self::Inactive,
        }
    }
}

impl ConfigField for GainSchedBehaviorValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GainSchedKpBehaviorValue {
    Continuous = 0,
    Discontinuous = 1,
}

impl GainSchedKpBehaviorValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Discontinuous,
            _ => Self::Continuous,
        }
    }
}

impl ConfigField for GainSchedKpBehaviorValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GravityTypeValue {
    Elevator_Static = 0,
    Arm_Cosine = 1,
}

impl GravityTypeValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Arm_Cosine,
            _ => Self::Elevator_Static,
        }
    }
}

impl ConfigField for GravityTypeValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InvertedValue {
    CounterClockwise_Positive = 0,
    Clockwise_Positive = 1,
}

impl InvertedValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Clockwise_Positive,
            _ => Self::CounterClockwise_Positive,
        }
    }
}

impl ConfigField for InvertedValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LarsonBounceValue {
    Front = 0,
    Center = 1,
    Back = 2,
}

impl LarsonBounceValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Center,
            2 => Self::Back,
            _ => Self::Front,
        }
    }
}

impl ConfigField for LarsonBounceValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LossOfSignalBehaviorValue {
    KeepRunning = 0,
    DisableLEDs = 1,
}

impl LossOfSignalBehaviorValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::DisableLEDs,
            _ => Self::KeepRunning,
        }
    }
}

impl ConfigField for LossOfSignalBehaviorValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MagnetHealthValue {
    Magnet_Red = 1,
    Magnet_Orange = 2,
    Magnet_Green = 3,
    Magnet_Invalid = 0,
}

impl MagnetHealthValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            2 => Self::Magnet_Orange,
            3 => Self::Magnet_Green,
            0 => Self::Magnet_Invalid,
            _ => Self::Magnet_Red,
        }
    }
}

impl ConfigField for MagnetHealthValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotorAlignmentValue {
    Aligned = 0,
    Opposed = 1,
}

impl MotorAlignmentValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Opposed,
            _ => Self::Aligned,
        }
    }
}

impl ConfigField for MotorAlignmentValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NeutralModeValue {
    Coast = 0,
    Brake = 1,
}

impl NeutralModeValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Brake,
            _ => Self::Coast,
        }
    }
}

impl ConfigField for NeutralModeValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReverseLimitSourceValue {
    LimitSwitchPin = 0,
    RemoteTalonFX = 1,
    RemoteCANifier = 2,
    RemoteCANcoder = 4,
    RemoteCANrange = 6,
    RemoteCANdiS1 = 7,
    RemoteCANdiS2 = 8,
    Disabled = 3,
}

impl ReverseLimitSourceValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::RemoteTalonFX,
            2 => Self::RemoteCANifier,
            4 => Self::RemoteCANcoder,
            6 => Self::RemoteCANrange,
            7 => Self::RemoteCANdiS1,
            8 => Self::RemoteCANdiS2,
            3 => Self::Disabled,
            _ => Self::LimitSwitchPin,
        }
    }
}

impl ConfigField for ReverseLimitSourceValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ReverseLimitTypeValue {
    NormallyOpen = 0,
    NormallyClosed = 1,
}

impl ReverseLimitTypeValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::NormallyClosed,
            _ => Self::NormallyOpen,
        }
    }
}

impl ConfigField for ReverseLimitTypeValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorDirectionValue {
    CounterClockwise_Positive = 0,
    Clockwise_Positive = 1,
}

impl SensorDirectionValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Clockwise_Positive,
            _ => Self::CounterClockwise_Positive,
        }
    }
}

impl ConfigField for SensorDirectionValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StaticFeedforwardSignValue {
    UseVelocitySign = 0,
    UseClosedLoopSign = 1,
}

impl StaticFeedforwardSignValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::UseClosedLoopSign,
            _ => Self::UseVelocitySign,
        }
    }
}

impl ConfigField for StaticFeedforwardSignValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StatusLedWhenActiveValue {
    Enabled = 0,
    Disabled = 1,
}

impl StatusLedWhenActiveValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Disabled,
            _ => Self::Enabled,
        }
    }
}

impl ConfigField for StatusLedWhenActiveValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StripTypeValue {
    GRB = 0,
    RGB = 1,
    BRG = 2,
    GRBW = 6,
    RGBW = 7,
    BRGW = 8,
}

impl StripTypeValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::RGB,
            2 => Self::BRG,
            6 => Self::GRBW,
            7 => Self::RGBW,
            8 => Self::BRGW,
            _ => Self::GRB,
        }
    }
}

impl ConfigField for StripTypeValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VBatOutputModeValue {
    On = 0,
    Off = 1,
    Modulated = 2,
}

impl VBatOutputModeValue {
    pub const fn value(self) -> i32 {
        self as i32
    }

    /// Unknown values map to the first variant, as in Java.
    pub const fn from_value(value: i32) -> Self {
        match value {
            1 => Self::Off,
            2 => Self::Modulated,
            _ => Self::On,
        }
    }
}

impl ConfigField for VBatOutputModeValue {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self.value())
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).map_or(current, Self::from_value)
    }
}
