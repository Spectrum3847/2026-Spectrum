//! Status code reporting to the driver station (`ErrorReportingJNI.reportStatusCode`).

use crate::native::phoenix;
use crate::status_code::StatusCode;
use std::ffi::{CString, c_char};

pub(crate) fn cstring(text: &str) -> CString {
    CString::new(text.replace('\0', "")).unwrap_or_default()
}

/// Sends a non-OK status with where it happened. The C layer prints it in simulation and
/// forwards it to the driver station on the roboRIO.
pub fn report_status_code(code: StatusCode, location: &str) {
    let details = cstring(&code.description());
    let location = cstring(location);
    let call_stack = cstring("");
    // SAFETY: all three strings are NUL terminated and outlive the call.
    unsafe {
        (phoenix().c_ctre_phoenix_report_error)(
            i32::from(code.is_error()),
            code.value(),
            0,
            details.as_ptr(),
            location.as_ptr(),
            call_stack.as_ptr(),
        );
    }
}

/// The C layer's name for a status code. Used to cross-check the generated table.
pub fn native_status_name(code: StatusCode) -> String {
    let mut buffer = [0 as c_char; 128];
    // SAFETY: the buffer is writable for its stated length; the library NUL terminates it.
    unsafe {
        (phoenix().c_ctre_phoenix_get_status_string)(code.value(), buffer.as_mut_ptr(), 128);
        std::ffi::CStr::from_ptr(buffer.as_ptr())
            .to_string_lossy()
            .into_owned()
    }
}
