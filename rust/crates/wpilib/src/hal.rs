//! The HAL C API. All `unsafe` FFI lives here, behind safe functions.

use crate::native_lib;
use std::backtrace::Backtrace;
use std::ffi::{CStr, CString, c_char};
use std::fmt;
use std::panic::Location;
use std::ptr;
use std::sync::OnceLock;

/// `HAL_Bool` (types.h) is an `int32_t`.
type HalBool = i32;

// wpi/string.h
#[repr(C)]
pub struct WpiString {
    str: *const c_char,
    len: usize,
}

// hal/DriverStationTypes.h: `HAL_ControlWord` is one `uint32_t` of bitfields, low bit first.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct ControlWordRaw {
    pub bits: u32,
}

pub const MAX_JOYSTICK_AXES: usize = 12;
pub const MAX_JOYSTICK_POVS: usize = 12;

// hal/DriverStationTypes.h: HAL_JoystickAxes
#[repr(C)]
#[derive(Clone, Copy)]
pub struct JoystickAxesRaw {
    pub count: i16,
    pub axes: [f32; MAX_JOYSTICK_AXES],
    pub raw: [u8; MAX_JOYSTICK_AXES],
}

// hal/DriverStationTypes.h: HAL_JoystickPOVs
#[repr(C)]
#[derive(Clone, Copy)]
pub struct JoystickPovsRaw {
    pub count: i16,
    pub povs: [i16; MAX_JOYSTICK_POVS],
}

// hal/DriverStationTypes.h: HAL_JoystickButtons
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct JoystickButtonsRaw {
    pub buttons: u32,
    pub count: u8,
}

// hal/DriverStationTypes.h: HAL_MatchInfo
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MatchInfoRaw {
    pub event_name: [u8; 64],
    pub match_type: i32,
    pub match_number: u16,
    pub replay_number: u8,
    pub game_specific_message: [u8; 64],
    pub game_specific_message_size: u16,
}

// Sizes from the C layout rules; a mismatch here means a wrong struct, not a wrong build.
const _: () = assert!(size_of::<ControlWordRaw>() == 4);
const _: () = assert!(size_of::<JoystickAxesRaw>() == 64);
const _: () = assert!(size_of::<JoystickPovsRaw>() == 26);
const _: () = assert!(size_of::<JoystickButtonsRaw>() == 8);
const _: () = assert!(size_of::<MatchInfoRaw>() == 140);

native_lib! {
    pub struct Hal = "wpiHal" {
        // hal/HALBase.h
        fn HAL_Initialize(timeout: i32, mode: i32) -> HalBool;
        fn HAL_Shutdown();
        fn HAL_GetRuntimeType() -> i32;
        fn HAL_GetLastError(status: *mut i32) -> *const c_char;
        fn HAL_GetFPGATime(status: *mut i32) -> u64;
        fn HAL_GetSerialNumber(serial_number: *mut WpiString);
        fn HAL_GetBrownedOut(status: *mut i32) -> HalBool;
        // hal/Main.h
        fn HAL_HasMain() -> HalBool;
        fn HAL_RunMain();
        fn HAL_ExitMain();
        fn HAL_SimPeriodicBefore();
        fn HAL_SimPeriodicAfter();
        fn HAL_GetPort(channel: i32) -> i32;
        // hal/FRCUsageReporting.h
        fn HAL_Report(resource: i32, instance_number: i32, context: i32, feature: *const c_char) -> i64;
        // hal/DriverStation.h
        fn HAL_SendError(
            is_error: HalBool,
            error_code: i32,
            is_lv_code: HalBool,
            details: *const c_char,
            location: *const c_char,
            call_stack: *const c_char,
            print_msg: HalBool,
        ) -> i32;
        fn HAL_GetControlWord(control_word: *mut ControlWordRaw) -> i32;
        fn HAL_GetAllianceStation(status: *mut i32) -> i32;
        fn HAL_GetJoystickAxes(joystick_num: i32, axes: *mut JoystickAxesRaw) -> i32;
        fn HAL_GetJoystickPOVs(joystick_num: i32, povs: *mut JoystickPovsRaw) -> i32;
        fn HAL_GetJoystickButtons(joystick_num: i32, buttons: *mut JoystickButtonsRaw) -> i32;
        fn HAL_GetJoystickIsXbox(joystick_num: i32) -> HalBool;
        fn HAL_GetJoystickType(joystick_num: i32) -> i32;
        fn HAL_SetJoystickOutputs(joystick_num: i32, outputs: i64, left_rumble: i32, right_rumble: i32) -> i32;
        fn HAL_GetMatchTime(status: *mut i32) -> f64;
        fn HAL_GetMatchInfo(info: *mut MatchInfoRaw) -> i32;
        fn HAL_RefreshDSData() -> HalBool;
        fn HAL_ObserveUserProgramStarting();
        fn HAL_ObserveUserProgramDisabled();
        fn HAL_ObserveUserProgramAutonomous();
        fn HAL_ObserveUserProgramTeleop();
        fn HAL_ObserveUserProgramTest();
        // hal/Power.h
        fn HAL_GetVinVoltage(status: *mut i32) -> f64;
        fn HAL_GetVinCurrent(status: *mut i32) -> f64;
        fn HAL_GetBrownoutVoltage(status: *mut i32) -> f64;
        fn HAL_SetBrownoutVoltage(voltage: f64, status: *mut i32);
        // hal/Notifier.h
        fn HAL_InitializeNotifier(status: *mut i32) -> i32;
        fn HAL_SetNotifierThreadPriority(real_time: HalBool, priority: i32, status: *mut i32) -> HalBool;
        fn HAL_SetNotifierName(notifier_handle: i32, name: *const c_char, status: *mut i32);
        fn HAL_StopNotifier(notifier_handle: i32, status: *mut i32);
        fn HAL_CleanNotifier(notifier_handle: i32);
        fn HAL_UpdateNotifierAlarm(notifier_handle: i32, trigger_time: u64, status: *mut i32);
        fn HAL_CancelNotifierAlarm(notifier_handle: i32, status: *mut i32);
        fn HAL_WaitForNotifierAlarm(notifier_handle: i32, status: *mut i32) -> u64;
        // hal/PWM.h
        fn HAL_InitializePWMPort(port_handle: i32, allocation_location: *const c_char, status: *mut i32) -> i32;
        fn HAL_FreePWMPort(pwm_port_handle: i32);
        fn HAL_CheckPWMChannel(channel: i32) -> HalBool;
        fn HAL_SetPWMConfigMicroseconds(
            pwm_port_handle: i32,
            max_pwm: i32,
            deadband_max_pwm: i32,
            center_pwm: i32,
            deadband_min_pwm: i32,
            min_pwm: i32,
            status: *mut i32,
        );
        fn HAL_SetPWMEliminateDeadband(pwm_port_handle: i32, eliminate_deadband: HalBool, status: *mut i32);
        fn HAL_SetPWMPosition(pwm_port_handle: i32, position: f64, status: *mut i32);
        fn HAL_GetPWMPosition(pwm_port_handle: i32, status: *mut i32) -> f64;
        fn HAL_SetPWMDisabled(pwm_port_handle: i32, status: *mut i32);
        fn HAL_SetPWMPeriodScale(pwm_port_handle: i32, squelch_mask: i32, status: *mut i32);
        // hal/PowerDistribution.h
        fn HAL_InitializePowerDistribution(
            module_number: i32,
            kind: i32,
            allocation_location: *const c_char,
            status: *mut i32,
        ) -> i32;
        fn HAL_GetPowerDistributionModuleNumber(handle: i32, status: *mut i32) -> i32;
        fn HAL_CleanPowerDistribution(handle: i32);
        fn HAL_GetPowerDistributionType(handle: i32, status: *mut i32) -> i32;
        fn HAL_GetPowerDistributionNumChannels(handle: i32, status: *mut i32) -> i32;
        fn HAL_GetPowerDistributionTemperature(handle: i32, status: *mut i32) -> f64;
        fn HAL_GetPowerDistributionVoltage(handle: i32, status: *mut i32) -> f64;
        fn HAL_GetPowerDistributionChannelCurrent(handle: i32, channel: i32, status: *mut i32) -> f64;
        fn HAL_GetPowerDistributionAllChannelCurrents(
            handle: i32,
            currents: *mut f64,
            currents_length: i32,
            status: *mut i32,
        );
        fn HAL_GetPowerDistributionTotalCurrent(handle: i32, status: *mut i32) -> f64;
        fn HAL_GetPowerDistributionTotalPower(handle: i32, status: *mut i32) -> f64;
        fn HAL_GetPowerDistributionTotalEnergy(handle: i32, status: *mut i32) -> f64;
        fn HAL_ResetPowerDistributionTotalEnergy(handle: i32, status: *mut i32);
        fn HAL_ClearPowerDistributionStickyFaults(handle: i32, status: *mut i32);
        fn HAL_SetPowerDistributionSwitchableChannel(handle: i32, enabled: HalBool, status: *mut i32);
        fn HAL_GetPowerDistributionSwitchableChannel(handle: i32, status: *mut i32) -> HalBool;
    }
}

native_lib! {
    pub struct WpiUtil = "wpiutil" {
        // wpi/string.h
        fn WPI_FreeString(wpi_string: *const WpiString);
    }
}

static HAL: OnceLock<Hal> = OnceLock::new();

/// The loaded HAL. The first call loads the library and runs `HAL_Initialize(500, 0)`, as
/// Java's `DriverStation` class initializer does.
///
/// # Panics
/// When the library cannot be loaded or the HAL fails to initialize.
pub fn hal() -> &'static Hal {
    HAL.get_or_init(|| {
        // SAFETY: the declared signatures come from the 2026.2.1 headers; the library's own
        // initializers are trusted.
        let hal = unsafe { Hal::load() }.unwrap_or_else(|e| {
            panic!(
                "{}",
                crate::native::load_failure(&Hal::file(), &e, "WPILib")
            )
        });
        // SAFETY: plain integer arguments; the HAL tolerates repeated calls.
        let ok = unsafe { (hal.HAL_Initialize)(500, 0) };
        assert!(ok != 0, "HAL_Initialize failed");
        hal
    })
}

/// The HAL if something already loaded it. Error reporting uses this so it never panics.
pub fn try_hal() -> Option<&'static Hal> {
    HAL.get()
}

/// A nonzero HAL status with its message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HalError {
    pub code: i32,
    pub message: String,
}

impl fmt::Display for HalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "HAL status {}: {}", self.code, self.message)
    }
}

impl std::error::Error for HalError {}

impl HalError {
    /// For failures that are not a HAL status (bad channel, missing handle).
    pub fn invalid(message: impl Into<String>) -> Self {
        Self {
            code: -1,
            message: message.into(),
        }
    }

    /// Sends this error to the driver station.
    #[track_caller]
    pub fn report(&self) {
        send_error(
            true,
            self.code,
            &self.to_string(),
            &caller_location(),
            "",
            true,
        );
    }
}

fn caller_location() -> String {
    let at = Location::caller();
    format!("{}:{}", at.file(), at.line())
}

/// Runs a HAL call that fills a status out-parameter and checks the status.
fn with_status<T>(f: impl FnOnce(&Hal, *mut i32) -> T) -> Result<T, HalError> {
    let hal = hal();
    let mut status = 0;
    let value = f(hal, &mut status);
    if status == 0 {
        return Ok(value);
    }
    // SAFETY: status is a valid out pointer. The returned string is owned by the HAL and
    // valid until the next HAL call on this thread; it is copied immediately.
    let message = unsafe {
        let text = (hal.HAL_GetLastError)(&mut status);
        if text.is_null() {
            String::new()
        } else {
            CStr::from_ptr(text).to_string_lossy().into_owned()
        }
    };
    let error = HalError {
        code: status,
        message,
    };
    // Java's JNI ReportError throws only on negative status. A positive status
    // is reported and the value is still used.
    if status > 0 {
        error.report();
        return Ok(value);
    }
    Err(error)
}

/// Reports a failed HAL read to the driver station and returns the zero value.
#[track_caller]
pub fn or_report<T: Default>(result: Result<T, HalError>) -> T {
    match result {
        Ok(value) => value,
        Err(e) => {
            e.report();
            T::default()
        }
    }
}

fn cstring(text: &str) -> CString {
    CString::new(text.replace('\0', "")).unwrap_or_default()
}

fn flag(value: HalBool) -> bool {
    value != 0
}

fn hal_bool(value: bool) -> HalBool {
    HalBool::from(value)
}

// Runtime

/// Where the program runs. Java's `RuntimeType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeType {
    RoboRio,
    RoboRio2,
    Simulation,
}

impl RuntimeType {
    /// Unknown values mean simulation, as in Java.
    pub fn from_hal(value: i32) -> Self {
        match value {
            0 => Self::RoboRio,
            1 => Self::RoboRio2,
            _ => Self::Simulation,
        }
    }
}

pub fn runtime_type() -> RuntimeType {
    static TYPE: OnceLock<RuntimeType> = OnceLock::new();
    *TYPE.get_or_init(|| {
        // SAFETY: no arguments, no preconditions.
        RuntimeType::from_hal(unsafe { (hal().HAL_GetRuntimeType)() })
    })
}

pub fn is_simulation() -> bool {
    runtime_type() == RuntimeType::Simulation
}

pub fn is_real() -> bool {
    matches!(runtime_type(), RuntimeType::RoboRio | RuntimeType::RoboRio2)
}

/// Initializes the HAL (idempotent) and returns it.
pub fn initialize() -> &'static Hal {
    hal()
}

pub fn shutdown() {
    if let Some(hal) = try_hal() {
        // SAFETY: no arguments; called after the robot loop has ended.
        unsafe { (hal.HAL_Shutdown)() }
    }
}

/// True when a simulation extension (the Sim GUI) needs the process main thread.
pub fn has_main() -> bool {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_HasMain)() != 0 }
}

/// Runs the extension's main loop on this thread until [`exit_main`] or the extension quits.
pub fn run_main() {
    // SAFETY: no arguments; only called after `has_main` returned true.
    unsafe { (hal().HAL_RunMain)() }
}

/// Makes [`run_main`] return.
pub fn exit_main() {
    // SAFETY: no arguments; only called after `has_main` returned true.
    unsafe { (hal().HAL_ExitMain)() }
}

pub fn sim_periodic_before() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_SimPeriodicBefore)() }
}

pub fn sim_periodic_after() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_SimPeriodicAfter)() }
}

pub fn fpga_time() -> Result<u64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetFPGATime)(s) })
}

pub fn serial_number() -> String {
    let mut serial = WpiString {
        str: ptr::null(),
        len: 0,
    };
    // SAFETY: the out pointer is valid. The HAL fills it with a string allocated by
    // WPI_AllocateString.
    unsafe { (hal().HAL_GetSerialNumber)(&mut serial) };
    if serial.str.is_null() {
        return String::new();
    }
    // SAFETY: str points at len bytes the HAL just allocated.
    let bytes = unsafe { std::slice::from_raw_parts(serial.str.cast::<u8>(), serial.len) };
    let text = String::from_utf8_lossy(bytes).into_owned();
    static WPIUTIL: OnceLock<Option<WpiUtil>> = OnceLock::new();
    // SAFETY: signature from wpi/string.h.
    let util = WPIUTIL.get_or_init(|| unsafe { WpiUtil::load() }.ok());
    if let Some(util) = util {
        // SAFETY: serial was allocated by the WPILib string allocator and is freed once.
        unsafe { (util.WPI_FreeString)(&serial) };
    }
    text
}

pub fn browned_out() -> Result<bool, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetBrownedOut)(s) }).map(flag)
}

// Usage reporting (hal/FRCUsageReporting.h, HALUsageReporting enums)

pub const RESOURCE_LANGUAGE: i32 = 2;
pub const RESOURCE_FRAMEWORK: i32 = 22;
pub const RESOURCE_PWM: i32 = 29;
pub const RESOURCE_SERVO: i32 = 33;
pub const RESOURCE_PDP: i32 = 59;
pub const INSTANCE_LANGUAGE_RUST: i32 = 7;
pub const INSTANCE_FRAMEWORK_TIMED: i32 = 4;
pub const INSTANCE_PDP_CTRE: i32 = 1;
pub const INSTANCE_PDP_REV: i32 = 2;

pub fn report_usage(resource: i32, instance: i32) {
    report_usage_with_feature(resource, instance, None);
}

pub fn report_usage_with_feature(resource: i32, instance: i32, feature: Option<&str>) {
    let feature = feature.map(cstring);
    let feature_ptr = feature.as_ref().map_or(ptr::null(), |f| f.as_ptr());
    // SAFETY: feature_ptr is null or a NUL-terminated string that outlives the call.
    unsafe { (hal().HAL_Report)(resource, instance, 0, feature_ptr) };
}

// Driver station errors

/// Sends a message to the driver station console. Falls back to stderr when the HAL is not
/// loaded, so this is safe to call anywhere.
pub fn send_error(
    is_error: bool,
    code: i32,
    details: &str,
    location: &str,
    call_stack: &str,
    print: bool,
) {
    let Some(hal) = try_hal() else {
        eprintln!("{}: {details}", if is_error { "ERROR" } else { "WARNING" });
        return;
    };
    let (details, location, call_stack) =
        (cstring(details), cstring(location), cstring(call_stack));
    // SAFETY: all three strings are NUL-terminated and outlive the call.
    unsafe {
        (hal.HAL_SendError)(
            hal_bool(is_error),
            code,
            0,
            details.as_ptr(),
            location.as_ptr(),
            call_stack.as_ptr(),
            hal_bool(print),
        );
    }
}

/// Call site and optional stack, the Rust version of Java's stack trace walk.
pub fn trace_text(print_trace: bool) -> String {
    if print_trace {
        Backtrace::force_capture().to_string()
    } else {
        String::new()
    }
}

// Driver station data

pub fn control_word() -> ControlWordRaw {
    let mut word = ControlWordRaw::default();
    // SAFETY: the out pointer is valid for one HAL_ControlWord.
    unsafe { (hal().HAL_GetControlWord)(&mut word) };
    word
}

pub fn refresh_ds_data() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_RefreshDSData)() };
}

pub fn joystick_axes(stick: i32) -> JoystickAxesRaw {
    let mut axes = JoystickAxesRaw {
        count: 0,
        axes: [0.0; MAX_JOYSTICK_AXES],
        raw: [0; MAX_JOYSTICK_AXES],
    };
    // SAFETY: the out pointer is valid for one HAL_JoystickAxes; stick is 0..6.
    unsafe { (hal().HAL_GetJoystickAxes)(stick, &mut axes) };
    axes
}

pub fn joystick_povs(stick: i32) -> JoystickPovsRaw {
    let mut povs = JoystickPovsRaw {
        count: 0,
        povs: [-1; MAX_JOYSTICK_POVS],
    };
    // SAFETY: the out pointer is valid for one HAL_JoystickPOVs; stick is 0..6.
    unsafe { (hal().HAL_GetJoystickPOVs)(stick, &mut povs) };
    povs
}

pub fn joystick_buttons(stick: i32) -> JoystickButtonsRaw {
    let mut buttons = JoystickButtonsRaw::default();
    // SAFETY: the out pointer is valid for one HAL_JoystickButtons; stick is 0..6.
    unsafe { (hal().HAL_GetJoystickButtons)(stick, &mut buttons) };
    buttons
}

pub fn match_info() -> MatchInfoRaw {
    let mut info = MatchInfoRaw {
        event_name: [0; 64],
        match_type: 0,
        match_number: 0,
        replay_number: 0,
        game_specific_message: [0; 64],
        game_specific_message_size: 0,
    };
    // SAFETY: the out pointer is valid for one HAL_MatchInfo.
    unsafe { (hal().HAL_GetMatchInfo)(&mut info) };
    info
}

pub fn alliance_station() -> Result<i32, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetAllianceStation)(s) })
}

pub fn match_time() -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetMatchTime)(s) })
}

pub fn joystick_is_xbox(stick: i32) -> bool {
    // SAFETY: plain integer argument; stick is 0..6.
    flag(unsafe { (hal().HAL_GetJoystickIsXbox)(stick) })
}

pub fn joystick_type(stick: i32) -> i32 {
    // SAFETY: plain integer argument; stick is 0..6.
    unsafe { (hal().HAL_GetJoystickType)(stick) }
}

pub fn set_joystick_outputs(stick: i32, outputs: i64, left_rumble: i32, right_rumble: i32) {
    // SAFETY: plain integer arguments; stick is 0..6.
    unsafe { (hal().HAL_SetJoystickOutputs)(stick, outputs, left_rumble, right_rumble) };
}

pub fn observe_user_program_starting() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_ObserveUserProgramStarting)() }
}

pub fn observe_user_program_disabled() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_ObserveUserProgramDisabled)() }
}

pub fn observe_user_program_autonomous() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_ObserveUserProgramAutonomous)() }
}

pub fn observe_user_program_teleop() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_ObserveUserProgramTeleop)() }
}

pub fn observe_user_program_test() {
    // SAFETY: no arguments, no preconditions.
    unsafe { (hal().HAL_ObserveUserProgramTest)() }
}

// Power

pub fn vin_voltage() -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetVinVoltage)(s) })
}

pub fn vin_current() -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetVinCurrent)(s) })
}

pub fn brownout_voltage() -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetBrownoutVoltage)(s) })
}

pub fn set_brownout_voltage(volts: f64) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_SetBrownoutVoltage)(volts, s) })
}

// Notifier

pub fn initialize_notifier() -> Result<i32, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_InitializeNotifier)(s) })
}

pub fn set_notifier_thread_priority(real_time: bool, priority: i32) -> bool {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe {
        (h.HAL_SetNotifierThreadPriority)(hal_bool(real_time), priority, s)
    })
    .is_ok_and(flag)
}

pub fn set_notifier_name(handle: i32, name: &str) {
    let name = cstring(name);
    // SAFETY: name is NUL-terminated and outlives the call; status is a valid out pointer.
    let _ = with_status(|h, s| unsafe { (h.HAL_SetNotifierName)(handle, name.as_ptr(), s) });
}

pub fn stop_notifier(handle: i32) {
    // SAFETY: status is a valid out pointer; a stale handle only yields a status error.
    let _ = with_status(|h, s| unsafe { (h.HAL_StopNotifier)(handle, s) });
}

pub fn clean_notifier(handle: i32) {
    // SAFETY: a stale handle is ignored by the HAL.
    unsafe { (hal().HAL_CleanNotifier)(handle) }
}

pub fn update_notifier_alarm(handle: i32, trigger_micros: u64) {
    // SAFETY: status is a valid out pointer; a stale handle only yields a status error.
    let _ = with_status(|h, s| unsafe { (h.HAL_UpdateNotifierAlarm)(handle, trigger_micros, s) });
}

pub fn cancel_notifier_alarm(handle: i32) {
    // SAFETY: status is a valid out pointer; a stale handle only yields a status error.
    let _ = with_status(|h, s| unsafe { (h.HAL_CancelNotifierAlarm)(handle, s) });
}

/// Blocks until the alarm fires. Returns 0 when the notifier was stopped or the call failed.
pub fn wait_for_notifier_alarm(handle: i32) -> u64 {
    // SAFETY: status is a valid out pointer; the call blocks this thread only.
    with_status(|h, s| unsafe { (h.HAL_WaitForNotifierAlarm)(handle, s) }).unwrap_or_else(|e| {
        e.report();
        0
    })
}

// PWM

pub fn check_pwm_channel(channel: i32) -> bool {
    // SAFETY: plain integer argument.
    flag(unsafe { (hal().HAL_CheckPWMChannel)(channel) })
}

pub fn initialize_pwm_port(channel: i32, location: &str) -> Result<i32, HalError> {
    let location = cstring(location);
    // SAFETY: location is NUL-terminated and outlives the call; status is a valid out
    // pointer; HAL_GetPort takes any channel number.
    with_status(|h, s| unsafe {
        (h.HAL_InitializePWMPort)((h.HAL_GetPort)(channel), location.as_ptr(), s)
    })
}

pub fn free_pwm_port(handle: i32) {
    // SAFETY: the handle came from initialize_pwm_port and is freed once.
    unsafe { (hal().HAL_FreePWMPort)(handle) }
}

pub fn set_pwm_config_microseconds(
    handle: i32,
    max: i32,
    deadband_max: i32,
    center: i32,
    deadband_min: i32,
    min: i32,
) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe {
        (h.HAL_SetPWMConfigMicroseconds)(handle, max, deadband_max, center, deadband_min, min, s)
    })
}

pub fn set_pwm_eliminate_deadband(handle: i32, eliminate: bool) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_SetPWMEliminateDeadband)(handle, hal_bool(eliminate), s) })
}

pub fn set_pwm_position(handle: i32, position: f64) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_SetPWMPosition)(handle, position, s) })
}

pub fn pwm_position(handle: i32) -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPWMPosition)(handle, s) })
}

pub fn set_pwm_disabled(handle: i32) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_SetPWMDisabled)(handle, s) })
}

pub fn set_pwm_period_scale(handle: i32, squelch_mask: i32) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_SetPWMPeriodScale)(handle, squelch_mask, s) })
}

// Power distribution

pub fn initialize_power_distribution(
    module: i32,
    kind: i32,
    location: &str,
) -> Result<i32, HalError> {
    let location = cstring(location);
    // SAFETY: location is NUL-terminated and outlives the call; status is a valid out pointer.
    with_status(|h, s| unsafe {
        (h.HAL_InitializePowerDistribution)(module, kind, location.as_ptr(), s)
    })
}

pub fn clean_power_distribution(handle: i32) {
    // SAFETY: the handle came from initialize_power_distribution and is cleaned once.
    unsafe { (hal().HAL_CleanPowerDistribution)(handle) }
}

pub fn pd_module_number(handle: i32) -> Result<i32, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionModuleNumber)(handle, s) })
}

pub fn pd_type(handle: i32) -> Result<i32, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionType)(handle, s) })
}

pub fn pd_num_channels(handle: i32) -> Result<i32, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionNumChannels)(handle, s) })
}

pub fn pd_temperature(handle: i32) -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionTemperature)(handle, s) })
}

pub fn pd_voltage(handle: i32) -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionVoltage)(handle, s) })
}

pub fn pd_channel_current(handle: i32, channel: i32) -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionChannelCurrent)(handle, channel, s) })
}

/// Fills `currents` with one value per channel; the HAL checks the length.
pub fn pd_all_channel_currents(handle: i32, currents: &mut [f64]) -> Result<(), HalError> {
    let len = i32::try_from(currents.len()).unwrap_or(i32::MAX);
    // SAFETY: the pointer and length describe a live, writable slice; status is a valid out
    // pointer.
    with_status(|h, s| unsafe {
        (h.HAL_GetPowerDistributionAllChannelCurrents)(handle, currents.as_mut_ptr(), len, s)
    })
}

pub fn pd_total_current(handle: i32) -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionTotalCurrent)(handle, s) })
}

pub fn pd_total_power(handle: i32) -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionTotalPower)(handle, s) })
}

pub fn pd_total_energy(handle: i32) -> Result<f64, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionTotalEnergy)(handle, s) })
}

pub fn pd_reset_total_energy(handle: i32) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_ResetPowerDistributionTotalEnergy)(handle, s) })
}

pub fn pd_clear_sticky_faults(handle: i32) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_ClearPowerDistributionStickyFaults)(handle, s) })
}

pub fn pd_set_switchable_channel(handle: i32, enabled: bool) -> Result<(), HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe {
        (h.HAL_SetPowerDistributionSwitchableChannel)(handle, hal_bool(enabled), s)
    })
}

pub fn pd_switchable_channel(handle: i32) -> Result<bool, HalError> {
    // SAFETY: status is a valid out pointer.
    with_status(|h, s| unsafe { (h.HAL_GetPowerDistributionSwitchableChannel)(handle, s) })
        .map(flag)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn runtime_type_maps_unknown_to_simulation() {
        assert_eq!(RuntimeType::from_hal(0), RuntimeType::RoboRio);
        assert_eq!(RuntimeType::from_hal(1), RuntimeType::RoboRio2);
        assert_eq!(RuntimeType::from_hal(2), RuntimeType::Simulation);
        assert_eq!(RuntimeType::from_hal(9), RuntimeType::Simulation);
    }

    #[test]
    fn cstring_drops_interior_nuls() {
        assert_eq!(cstring("a\0b").to_str().unwrap(), "ab");
    }

    #[test]
    fn send_error_without_hal_does_not_panic() {
        send_error(false, 1, "no natives here", "", "", true);
    }
}
