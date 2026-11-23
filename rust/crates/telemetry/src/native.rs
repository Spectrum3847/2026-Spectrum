//! HAL and ntcore calls that `wpilib` does not wrap yet, loaded the same way: by name from the
//! shared libraries on the linker path. Everything here has a fallback so a missing library
//! reads as zeros and logs nothing.

use std::sync::OnceLock;
use wpilib::native_lib;

use wpilib::nt::ffi::WpiStr;

type HalBool = i32;

native_lib! {
    pub struct HalExtra = "wpiHal" {
        fn HAL_GetFPGAVersion(status: *mut i32) -> i32;
        fn HAL_GetFPGARevision(status: *mut i32) -> i64;
        fn HAL_GetComments(comments: *mut WpiStr);
        fn HAL_GetTeamNumber() -> i32;
        fn HAL_GetFPGAButton(status: *mut i32) -> HalBool;
        fn HAL_GetSystemActive(status: *mut i32) -> HalBool;
        fn HAL_GetRSLState(status: *mut i32) -> HalBool;
        fn HAL_GetSystemTimeValid(status: *mut i32) -> HalBool;
        fn HAL_GetCPUTemp(status: *mut i32) -> f64;
        fn HAL_GetUserVoltage3V3(status: *mut i32) -> f64;
        fn HAL_GetUserCurrent3V3(status: *mut i32) -> f64;
        fn HAL_GetUserActive3V3(status: *mut i32) -> HalBool;
        fn HAL_GetUserCurrentFaults3V3(status: *mut i32) -> i32;
        fn HAL_GetUserVoltage5V(status: *mut i32) -> f64;
        fn HAL_GetUserCurrent5V(status: *mut i32) -> f64;
        fn HAL_GetUserActive5V(status: *mut i32) -> HalBool;
        fn HAL_GetUserCurrentFaults5V(status: *mut i32) -> i32;
        fn HAL_GetUserVoltage6V(status: *mut i32) -> f64;
        fn HAL_GetUserCurrent6V(status: *mut i32) -> f64;
        fn HAL_GetUserActive6V(status: *mut i32) -> HalBool;
        fn HAL_GetUserCurrentFaults6V(status: *mut i32) -> i32;
        fn HAL_CAN_GetCANStatus(
            percent_bus_utilization: *mut f32,
            bus_off_count: *mut u32,
            tx_full_count: *mut u32,
            receive_error_count: *mut u32,
            transmit_error_count: *mut u32,
            status: *mut i32,
        );
    }
}

/// The extra HAL entry points, once `wpilib` has loaded and initialized the HAL.
pub fn hal_extra() -> Option<&'static HalExtra> {
    static EXTRA: OnceLock<Option<HalExtra>> = OnceLock::new();
    wpilib::hal::try_hal()?;
    // SAFETY: signatures come from hal/HALBase.h, hal/Power.h and hal/CAN.h of v2026.2.1.
    EXTRA
        .get_or_init(|| unsafe { HalExtra::load() }.ok())
        .as_ref()
}

native_lib! {
    pub struct NtExtra = "ntcore" {
        fn NT_SetTopicProperty(
            topic: u32,
            name: *const wpilib::nt::ffi::WpiStr,
            value: *const wpilib::nt::ffi::WpiStr,
        ) -> i32;
        fn NT_SetFloat(pubentry: u32, time: i64, value: f32) -> i32;
        fn NT_SetFloatArray(pubentry: u32, time: i64, value: *const f32, len: usize) -> i32;
    }
}

pub fn nt_extra() -> Option<&'static NtExtra> {
    static EXTRA: OnceLock<Option<NtExtra>> = OnceLock::new();
    // SAFETY: signatures come from ntcore_c.h of v2026.2.1.
    EXTRA
        .get_or_init(|| unsafe { NtExtra::load() }.ok())
        .as_ref()
}

/// A HAL getter with a status out parameter. Zero when the call fails or the HAL is missing.
fn status<T: Default>(call: impl FnOnce(&'static HalExtra, *mut i32) -> T) -> T {
    let Some(hal) = hal_extra() else {
        return T::default();
    };
    let mut code = 0;
    let value = call(hal, &mut code);
    if code == 0 { value } else { T::default() }
}

macro_rules! getter {
    ($name:ident, $f:ident, $ty:ty) => {
        pub fn $name() -> $ty {
            // SAFETY: status is a valid out pointer.
            status(|h, s| unsafe { (h.$f)(s) })
        }
    };
}

getter!(fpga_version, HAL_GetFPGAVersion, i32);
getter!(fpga_revision, HAL_GetFPGARevision, i64);
getter!(cpu_temp, HAL_GetCPUTemp, f64);
getter!(voltage_3v3, HAL_GetUserVoltage3V3, f64);
getter!(current_3v3, HAL_GetUserCurrent3V3, f64);
getter!(current_faults_3v3, HAL_GetUserCurrentFaults3V3, i32);
getter!(voltage_5v, HAL_GetUserVoltage5V, f64);
getter!(current_5v, HAL_GetUserCurrent5V, f64);
getter!(current_faults_5v, HAL_GetUserCurrentFaults5V, i32);
getter!(voltage_6v, HAL_GetUserVoltage6V, f64);
getter!(current_6v, HAL_GetUserCurrent6V, f64);
getter!(current_faults_6v, HAL_GetUserCurrentFaults6V, i32);

macro_rules! flag {
    ($name:ident, $f:ident) => {
        pub fn $name() -> bool {
            // SAFETY: status is a valid out pointer.
            status(|h, s| unsafe { (h.$f)(s) } != 0)
        }
    };
}

flag!(fpga_button, HAL_GetFPGAButton);
flag!(system_active, HAL_GetSystemActive);
flag!(rsl_state, HAL_GetRSLState);
flag!(system_time_valid, HAL_GetSystemTimeValid);
flag!(active_3v3, HAL_GetUserActive3V3);
flag!(active_5v, HAL_GetUserActive5V);
flag!(active_6v, HAL_GetUserActive6V);

pub fn team_number() -> i32 {
    // SAFETY: no arguments.
    hal_extra().map_or(0, |h| unsafe { (h.HAL_GetTeamNumber)() })
}

/// The roboRIO comments field.
pub fn comments() -> String {
    let Some(hal) = hal_extra() else {
        return String::new();
    };
    let mut text = WpiStr::EMPTY;
    // SAFETY: the out pointer is valid; the HAL fills it with an allocated string.
    unsafe { (hal.HAL_GetComments)(&mut text) };
    if text.str.is_null() {
        return String::new();
    }
    // SAFETY: str points at len bytes the HAL just allocated.
    let bytes = unsafe { std::slice::from_raw_parts(text.str.cast::<u8>(), text.len) };
    let out = String::from_utf8_lossy(bytes).into_owned();
    static UTIL: OnceLock<Option<wpilib::hal::WpiUtil>> = OnceLock::new();
    // SAFETY: signature from wpi/string.h.
    if let Some(util) = UTIL.get_or_init(|| unsafe { wpilib::hal::WpiUtil::load() }.ok()) {
        // SAFETY: the string came from the WPILib allocator and is freed once; WpiStr and
        // WPI_String have one layout.
        unsafe { (util.WPI_FreeString)((&raw const text).cast()) };
    }
    out
}

#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct CanStatus {
    pub percent_bus_utilization: f32,
    pub bus_off_count: u32,
    pub tx_full_count: u32,
    pub receive_error_count: u32,
    pub transmit_error_count: u32,
}

pub fn can_status() -> CanStatus {
    let mut out = CanStatus::default();
    let Some(hal) = hal_extra() else {
        return out;
    };
    let mut code = 0;
    // SAFETY: every out pointer is valid.
    unsafe {
        (hal.HAL_CAN_GetCANStatus)(
            &mut out.percent_bus_utilization,
            &mut out.bus_off_count,
            &mut out.tx_full_count,
            &mut out.receive_error_count,
            &mut out.transmit_error_count,
            &mut code,
        );
    }
    if code == 0 { out } else { CanStatus::default() }
}

/// Sets the `unit` property of a topic. NetworkTables stores property values as JSON.
pub fn set_topic_unit(topic: u32, unit: &str) {
    let Some(extra) = nt_extra() else {
        return;
    };
    let (prop, json) = (WpiStr::borrow("unit"), format!("\"{unit}\""));
    let json = WpiStr::borrow(&json);
    // SAFETY: both strings outlive the call; ntcore copies them.
    unsafe { (extra.NT_SetTopicProperty)(topic, &prop, &json) };
}
