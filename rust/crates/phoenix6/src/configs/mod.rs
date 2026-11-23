//! Device configuration (`com.ctre.phoenix6.configs`).
//!
//! A configuration is a tree of groups. Each group lists its fields as (SPN, value) items in
//! the order Java's `serialize()` writes them. [`ParentConfiguration::serialize`] turns the
//! items into text with the C serializer, and `deserialize` reads device text back the same
//! way. Applying a full configuration writes every field, so fields you leave alone return to
//! their defaults on the device.

mod codec;
mod groups;

pub use groups::*;

pub(crate) use codec::take_string;

/// The C layer's text for a list of items.
pub(crate) fn serialize_items(items: &[ConfigItem]) -> String {
    codec::serialize(items)
}

/// One field value in the C layer's three serialized kinds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConfigValue {
    Double(f64),
    Int(i32),
    Bool(bool),
}

/// A field ready for the serializer: the SPN and its value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConfigItem {
    pub spn: i32,
    pub value: ConfigValue,
}

/// Where deserialized values come from. `None` means the SPN is not in the text.
pub trait ConfigSource {
    fn double(&self, spn: i32) -> Option<f64>;
    fn int(&self, spn: i32) -> Option<i32>;
    fn boolean(&self, spn: i32) -> Option<bool>;
}

/// A field type a group can hold.
pub trait ConfigField: Copy {
    fn to_value(self) -> ConfigValue;
    /// The value for `spn`, or `current` when the source has none.
    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self;
}

impl ConfigField for f64 {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Double(self)
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.double(spn).unwrap_or(current)
    }
}

impl ConfigField for i32 {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Int(self)
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.int(spn).unwrap_or(current)
    }
}

impl ConfigField for bool {
    fn to_value(self) -> ConfigValue {
        ConfigValue::Bool(self)
    }

    fn from_source(spn: i32, src: &dyn ConfigSource, current: Self) -> Self {
        src.boolean(spn).unwrap_or(current)
    }
}

/// A group or a whole device configuration (Java `ParentConfiguration`).
pub trait ParentConfiguration {
    /// Appends this configuration's items in serialization order.
    fn collect_items(&self, out: &mut Vec<ConfigItem>);

    /// Reads every field from `src`. A field missing from `src` keeps its value.
    fn load(&mut self, src: &dyn ConfigSource);

    /// Java's `FutureProofConfigs`. Only device configurations carry it.
    fn future_proof(&self) -> bool {
        false
    }

    fn items(&self) -> Vec<ConfigItem> {
        let mut out = Vec::new();
        self.collect_items(&mut out);
        out
    }

    /// Needs the natives: the C layer formats each item.
    fn serialize(&self) -> String {
        codec::serialize(&self.items())
    }

    /// Needs the natives: the C layer parses the text.
    fn deserialize(&mut self, text: &str) {
        self.load(&codec::NativeSource::new(text));
    }
}

macro_rules! config_group {
    (
        $(#[$meta:meta])*
        pub struct $name:ident {
            $(
                $(#[$fmeta:meta])*
                $field:ident / $with:ident : $ty:ty = $default:expr => $spn:ident
            ),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq)]
        pub struct $name {
            $( $(#[$fmeta])* pub $field: $ty, )*
        }

        impl Default for $name {
            fn default() -> Self {
                Self { $( $field: $default, )* }
            }
        }

        impl $name {
            $(
                #[must_use]
                pub fn $with(mut self, value: $ty) -> Self {
                    self.$field = value;
                    self
                }
            )*
        }

        impl $crate::configs::ParentConfiguration for $name {
            fn collect_items(&self, out: &mut Vec<$crate::configs::ConfigItem>) {
                $(
                    out.push($crate::configs::ConfigItem {
                        spn: $crate::spn::$spn,
                        value: $crate::configs::ConfigField::to_value(self.$field),
                    });
                )*
            }

            fn load(&mut self, src: &dyn $crate::configs::ConfigSource) {
                $(
                    self.$field = $crate::configs::ConfigField::from_source(
                        $crate::spn::$spn,
                        src,
                        self.$field,
                    );
                )*
            }
        }
    };
}
pub(crate) use config_group;

macro_rules! configuration {
    (
        $(#[$meta:meta])*
        pub struct $name:ident {
            $( $field:ident / $with:ident : $ty:ty ),* $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq)]
        pub struct $name {
            /// Lets the device ignore settings its firmware does not know instead of failing.
            pub future_proof_configs: bool,
            $( pub $field: $ty, )*
        }

        impl Default for $name {
            fn default() -> Self {
                Self {
                    future_proof_configs: true,
                    $( $field: <$ty>::default(), )*
                }
            }
        }

        impl $name {
            #[must_use]
            pub fn with_future_proof_configs(mut self, value: bool) -> Self {
                self.future_proof_configs = value;
                self
            }

            $(
                #[must_use]
                pub fn $with(mut self, value: $ty) -> Self {
                    self.$field = value;
                    self
                }
            )*
        }

        impl $crate::configs::ParentConfiguration for $name {
            fn collect_items(&self, out: &mut Vec<$crate::configs::ConfigItem>) {
                $( self.$field.collect_items(out); )*
            }

            fn load(&mut self, src: &dyn $crate::configs::ConfigSource) {
                $( self.$field.load(src); )*
            }

            fn future_proof(&self) -> bool {
                self.future_proof_configs
            }
        }
    };
}

configuration! {
    /// All TalonFX configuration, in Java's serialization order.
    pub struct TalonFXConfiguration {
        motor_output / with_motor_output: MotorOutputConfigs,
        current_limits / with_current_limits: CurrentLimitsConfigs,
        voltage / with_voltage: VoltageConfigs,
        torque_current / with_torque_current: TorqueCurrentConfigs,
        feedback / with_feedback: FeedbackConfigs,
        differential_sensors / with_differential_sensors: DifferentialSensorsConfigs,
        differential_constants / with_differential_constants: DifferentialConstantsConfigs,
        open_loop_ramps / with_open_loop_ramps: OpenLoopRampsConfigs,
        closed_loop_ramps / with_closed_loop_ramps: ClosedLoopRampsConfigs,
        hardware_limit_switch / with_hardware_limit_switch: HardwareLimitSwitchConfigs,
        audio / with_audio: AudioConfigs,
        software_limit_switch / with_software_limit_switch: SoftwareLimitSwitchConfigs,
        motion_magic / with_motion_magic: MotionMagicConfigs,
        custom_params / with_custom_params: CustomParamsConfigs,
        closed_loop_general / with_closed_loop_general: ClosedLoopGeneralConfigs,
        slot0 / with_slot0: Slot0Configs,
        slot1 / with_slot1: Slot1Configs,
        slot2 / with_slot2: Slot2Configs,
    }
}

configuration! {
    /// All CANcoder configuration.
    pub struct CANcoderConfiguration {
        magnet_sensor / with_magnet_sensor: MagnetSensorConfigs,
        custom_params / with_custom_params: CustomParamsConfigs,
    }
}

configuration! {
    /// All Pigeon 2 configuration.
    pub struct Pigeon2Configuration {
        mount_pose / with_mount_pose: MountPoseConfigs,
        gyro_trim / with_gyro_trim: GyroTrimConfigs,
        pigeon2_features / with_pigeon2_features: Pigeon2FeaturesConfigs,
        custom_params / with_custom_params: CustomParamsConfigs,
    }
}

configuration! {
    /// All CANdle configuration.
    pub struct CANdleConfiguration {
        custom_params / with_custom_params: CustomParamsConfigs,
        led / with_led: LEDConfigs,
        candle_features / with_candle_features: CANdleFeaturesConfigs,
    }
}
