//! Runtime loading of CTRE's C layer: `CTRE_PhoenixTools_Sim` in simulation,
//! `CTRE_PhoenixTools` on the roboRIO. Signatures come from the 26.3.0 headers in
//! `ctre/phoenix6/networking/interfaces`, `ctre/phoenix6/platform` and `ctre/phoenix6/unmanaged`.

use std::any::Any;
use std::ffi::c_char;
use std::sync::OnceLock;

/// `network_signal_t` (DataStructs_Interface.h): a network name plus a device and SPN.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct NetworkSignal {
    pub network: *const c_char,
    pub device_hash: u32,
    pub spn: u32,
}

macro_rules! phoenix_natives {
    ( $( fn $f:ident( $( $arg:ident : $ty:ty ),* ) $( -> $ret:ty )? ; )* ) => {
        mod real {
            use super::*;
            wpilib::native_lib! {
                pub struct Lib = "CTRE_PhoenixTools" {
                    $( fn $f( $( $arg : $ty ),* ) $( -> $ret )? ; )*
                }
            }
        }

        mod sim {
            use super::*;
            wpilib::native_lib! {
                pub struct Lib = "CTRE_PhoenixTools_Sim" {
                    $( fn $f( $( $arg : $ty ),* ) $( -> $ret )? ; )*
                }
            }
        }

        /// The C function table, copied out of whichever library loaded.
        #[allow(non_snake_case)]
        pub struct Phoenix {
            $( pub $f: unsafe extern "C" fn( $( $ty ),* ) $( -> $ret )?, )*
            _lib: Box<dyn Any + Send + Sync>,
        }

        impl Phoenix {
            fn from_real(lib: real::Lib) -> Self {
                Self { $( $f: lib.$f, )* _lib: Box::new(lib) }
            }

            fn from_sim(lib: sim::Lib) -> Self {
                Self { $( $f: lib.$f, )* _lib: Box::new(lib) }
            }
        }
    };
}

phoenix_natives! {
        fn c_ctre_phoenix6_set_configs(context: i32, network: *const c_char, device_hash: i32, timeout_seconds: f64, values: *const c_char, value_len: u32, future_proof_configs: bool, override_if_duplicate: bool, use_did: bool) -> i32;
        fn c_ctre_phoenix6_get_configs(context: i32, network: *const c_char, device_hash: i32, timeout_seconds: f64, str: *mut *mut c_char, use_did: bool) -> i32;
        fn c_ctre_phoenix6_free_memory(str: *mut *mut c_char);
        fn c_ctre_phoenix6_serialize_double(spn: i32, value: f64, str: *mut *mut c_char) -> i32;
        fn c_ctre_phoenix6_serialize_int(spn: i32, value: i32, str: *mut *mut c_char) -> i32;
        fn c_ctre_phoenix6_serialize_bool(spn: i32, value: bool, str: *mut *mut c_char) -> i32;
        fn c_ctre_phoenix6_deserialize_double(spn: i32, str: *const c_char, strlen: u32, val: *mut f64) -> i32;
        fn c_ctre_phoenix6_deserialize_int(spn: i32, str: *const c_char, strlen: u32, val: *mut i32) -> i32;
        fn c_ctre_phoenix6_deserialize_bool(spn: i32, str: *const c_char, strlen: u32, val: *mut bool) -> i32;
        fn c_ctre_phoenix6_encode_device(device_id: i32, model: *const c_char, canbus: *const c_char, device_encoding: *mut u32) -> i32;
        fn c_ctre_phoenix_report_error(is_error: i32, error_code: i32, is_lv_code: i32, details: *const c_char, location: *const c_char, call_stack: *const c_char);
        fn c_ctre_phoenix_get_status_string(status_code: i32, to_fill: *mut c_char, to_fill_length: u32);
        fn c_ctre_phoenix6_SetUpdateFrequency(context: i32, network: *const c_char, device_hash: u32, spn: u16, frequency_hz: f64, timeout_seconds: f64) -> i32;
        fn c_ctre_phoenix6_SetUpdateFrequencyForAll(context: i32, signals_list: *const NetworkSignal, num_signals: usize, frequency_hz: f64, timeout_seconds: f64) -> i32;
        fn c_ctre_phoenix6_GetUpdateFrequency(network: *const c_char, device_hash: u32, spn: u16) -> f64;
        fn c_ctre_phoenix6_OptimizeUpdateFrequencies(context: i32, network: *const c_char, device_hash: u32, optimized_freq_hz: f64, timeout_seconds: f64) -> i32;
        fn c_ctre_phoenix6_get_signal_simplified(network: *const c_char, b_wait_for_update: i32, timeout_seconds: f64, num_signals: u32, device_hashes: *const u32, spns: *const u32, values: *mut f64, hw_timestamp_seconds: *mut f64, sw_timestamp_seconds: *mut f64, ecu_timestamp_seconds: *mut f64) -> i32;
        fn c_ctre_phoenix6_get_current_time_seconds() -> f64;
        fn c_ctre_phoenix6_get_system_time_seconds() -> f64;
        fn c_ctre_phoenix6_is_simulation() -> i32;
        fn c_ctre_phoenix6_is_replay() -> i32;
        fn c_ctre_phoenix6_unmanaged_feed_enable(timeout_ms: i32);
        fn c_ctre_phoenix6_unmanaged_get_enable_state() -> bool;
        fn c_ctre_phoenix6_unmanaged_get_api_compliancy() -> i32;
        fn c_ctre_phoenix6_unmanaged_get_phoenix_version() -> i32;
        fn c_Phoenix_Diagnostics_SetSecondsToStart(seconds_to_start: f64);
        fn c_ctre_phoenix6_platform_sim_create(device_type: i32, id: i32) -> i32;
        fn c_ctre_phoenix6_platform_sim_set_physics_input(device_type: i32, id: i32, physics_type: *const c_char, value: f64) -> i32;
        fn c_ctre_phoenix6_platform_sim_get_physics_value(device_type: i32, id: i32, physics_type: *const c_char, value: *mut f64) -> i32;
        fn c_ctre_phoenix6_platform_sim_get_last_error(device_type: i32, id: i32) -> i32;
        fn c_ctre_phoenix6_platform_canbus_get_status(bus_util_perc: *mut f32, bus_off_count: *mut u32, tx_full_count: *mut u32, rec: *mut u32, tec: *mut u32, canbus: *const c_char, print_err: bool) -> i32;
        fn c_ctre_phoenix6_platform_replay_load_file(filepath: *const c_char) -> i32;
        fn c_ctre_phoenix6_platform_set_logger_path(path: *const c_char) -> i32;
        fn c_ctre_phoenix6_platform_start_logger() -> i32;
        fn c_ctre_phoenix6_platform_stop_logger() -> i32;
        fn c_ctre_phoenix6_platform_enable_auto_logging(enable: bool) -> i32;
        fn c_ctre_phoenix6_platform_write_string(name: *const c_char, value: *const c_char, latency_seconds: f64) -> i32;
        fn c_ctre_phoenix6_platform_write_boolean(name: *const c_char, value: bool, latency_seconds: f64) -> i32;
        fn c_ctre_phoenix6_platform_write_double(name: *const c_char, value: f64, units: *const c_char, latency_seconds: f64) -> i32;
        fn c_ctre_phoenix6_platform_write_integer(name: *const c_char, value: i64, units: *const c_char, latency_seconds: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlEmpty(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlDutyCycleOut(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, output: f64, enable_foc: bool, override_brake_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlTorqueCurrentFOC(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, output: f64, max_abs_duty_cycle: f64, deadband: f64, override_coast_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlVoltageOut(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, output: f64, enable_foc: bool, override_brake_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlPositionVoltage(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, position: f64, velocity: f64, enable_foc: bool, feed_forward: f64, slot: i32, override_brake_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlPositionTorqueCurrentFOC(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, position: f64, velocity: f64, feed_forward: f64, slot: i32, override_coast_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlVelocityVoltage(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, velocity: f64, acceleration: f64, enable_foc: bool, feed_forward: f64, slot: i32, override_brake_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlVelocityTorqueCurrentFOC(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, velocity: f64, acceleration: f64, feed_forward: f64, slot: i32, override_coast_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlMotionMagicVoltage(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, position: f64, enable_foc: bool, feed_forward: f64, slot: i32, override_brake_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlMotionMagicTorqueCurrentFOC(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, position: f64, feed_forward: f64, slot: i32, override_coast_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlMotionMagicVelocityVoltage(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, velocity: f64, acceleration: f64, enable_foc: bool, feed_forward: f64, slot: i32, override_brake_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlMotionMagicVelocityTorqueCurrentFOC(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, velocity: f64, acceleration: f64, feed_forward: f64, slot: i32, override_coast_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlDynamicMotionMagicVoltage(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, position: f64, velocity: f64, acceleration: f64, jerk: f64, enable_foc: bool, feed_forward: f64, slot: i32, override_brake_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlDynamicMotionMagicTorqueCurrentFOC(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, position: f64, velocity: f64, acceleration: f64, jerk: f64, feed_forward: f64, slot: i32, override_coast_dur_neutral: bool, limit_forward_motion: bool, limit_reverse_motion: bool, ignore_hardware_limits: bool, ignore_software_limits: bool, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlFollower(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, leader_id: i32, motor_alignment: i32) -> i32;
        fn c_ctre_phoenix6_RequestControlNeutralOut(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlCoastOut(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlStaticBrake(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, use_timesync: bool) -> i32;
        fn c_ctre_phoenix6_RequestControlSolidColor(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, color_red: i32, color_green: i32, color_blue: i32, color_white: i32) -> i32;
        fn c_ctre_phoenix6_RequestControlRainbowAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, slot: i32, brightness: f64, direction: i32, frame_rate: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlRgbFadeAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, slot: i32, brightness: f64, frame_rate: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlSingleFadeAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, slot: i32, color_red: i32, color_green: i32, color_blue: i32, color_white: i32, frame_rate: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlStrobeAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, slot: i32, color_red: i32, color_green: i32, color_blue: i32, color_white: i32, frame_rate: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlLarsonAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, slot: i32, color_red: i32, color_green: i32, color_blue: i32, color_white: i32, size: i32, bounce_mode: i32, frame_rate: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlFireAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, slot: i32, brightness: f64, direction: i32, sparking: f64, cooling: f64, frame_rate: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlColorFlowAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, led_start_index: i32, led_end_index: i32, slot: i32, color_red: i32, color_green: i32, color_blue: i32, color_white: i32, direction: i32, frame_rate: f64) -> i32;
        fn c_ctre_phoenix6_RequestControlEmptyAnimation(canbus: *const c_char, ecu_encoding: u32, update_frequency: f64, slot: i32) -> i32;
}

static PHOENIX: OnceLock<Phoenix> = OnceLock::new();

/// The loaded C layer. The first call picks the library from the HAL runtime type, which
/// loads and initializes the HAL.
///
/// # Panics
/// When the library cannot be loaded.
pub fn phoenix() -> &'static Phoenix {
    PHOENIX.get_or_init(|| {
        let (file, loaded) = if wpilib::is_simulation() {
            // SAFETY: the declared signatures come from the 26.3.0 headers; the library's own
            // initializers are trusted.
            (
                sim::Lib::file(),
                unsafe { sim::Lib::load() }.map(Phoenix::from_sim),
            )
        } else {
            // SAFETY: as above.
            (
                real::Lib::file(),
                unsafe { real::Lib::load() }.map(Phoenix::from_real),
            )
        };
        loaded.unwrap_or_else(|e| panic!("{}", wpilib::native::load_failure(&file, &e, "CTRE")))
    })
}
