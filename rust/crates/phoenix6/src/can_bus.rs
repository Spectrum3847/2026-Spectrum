//! CAN bus handle (`com.ctre.phoenix6.CANBus`).

use crate::error_report::cstring;
use crate::native::phoenix;
use crate::status_code::StatusCode;
use std::fmt;

/// A CAN bus by name: `"rio"` for the roboRIO bus, `"*"` for the first CANivore found, or a
/// CANivore name. The empty name is the default bus.
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CanBus {
    name: String,
}

/// Bus health counters. `status` is `InvalidNetwork` when the bus is unknown.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CanBusStatus {
    pub status: StatusCode,
    /// Fraction of the bus bandwidth in use, 0 to 1.
    pub bus_utilization: f32,
    pub bus_off_count: u32,
    pub tx_full_count: u32,
    pub rec: u32,
    pub tec: u32,
}

impl CanBus {
    pub fn new(name: impl Into<String>) -> Self {
        Self { name: name.into() }
    }

    pub fn roborio() -> Self {
        Self::new("rio")
    }

    /// A bus that also loads a hoot log for replay. Fails for the codes where Java throws:
    /// `InvalidFile`, `UnlicensedHootLog`, `HootLogTooOld` and `HootLogTooNew`. Any other status,
    /// such as `NotSupported` from a library without replay, is not an error.
    pub fn with_hoot_file(name: impl Into<String>, hoot_path: &str) -> Result<Self, StatusCode> {
        load_hoot_file(hoot_path)?;
        Ok(Self::new(name))
    }

    pub fn roborio_with_hoot_file(hoot_path: &str) -> Result<Self, StatusCode> {
        Self::with_hoot_file("rio", hoot_path)
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn status(&self) -> CanBusStatus {
        let name = cstring(&self.name);
        let mut utilization = 0.0_f32;
        let (mut bus_off, mut tx_full, mut rec, mut tec) = (0_u32, 0_u32, 0_u32, 0_u32);
        // SAFETY: every out pointer is valid for one write and the name is NUL terminated.
        let error = unsafe {
            (phoenix().c_ctre_phoenix6_platform_canbus_get_status)(
                &mut utilization,
                &mut bus_off,
                &mut tx_full,
                &mut rec,
                &mut tec,
                name.as_ptr(),
                false,
            )
        };
        CanBusStatus {
            status: if error == 0 {
                StatusCode::OK
            } else {
                StatusCode::InvalidNetwork
            },
            bus_utilization: utilization,
            bus_off_count: bus_off,
            tx_full_count: tx_full,
            rec,
            tec,
        }
    }
}

fn load_hoot_file(path: &str) -> Result<(), StatusCode> {
    let path = cstring(path);
    // SAFETY: the path is NUL terminated and outlives the call.
    let code =
        StatusCode(unsafe { (phoenix().c_ctre_phoenix6_platform_replay_load_file)(path.as_ptr()) });
    if hoot_load_failed(code) {
        Err(code)
    } else {
        Ok(())
    }
}

fn hoot_load_failed(code: StatusCode) -> bool {
    matches!(
        code,
        StatusCode::InvalidFile
            | StatusCode::UnlicensedHootLog
            | StatusCode::HootLogTooOld
            | StatusCode::HootLogTooNew
    )
}

impl fmt::Display for CanBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_and_equality() {
        assert_eq!(CanBus::roborio().name(), "rio");
        assert_eq!(CanBus::default().name(), "");
        assert_eq!(CanBus::new("*"), CanBus::new("*"));
        assert_ne!(CanBus::new("rio"), CanBus::new("*"));
        assert_eq!(CanBus::new("canivore").to_string(), "canivore");
    }

    #[test]
    fn hoot_failures_match_java() {
        for code in [
            StatusCode::InvalidFile,
            StatusCode::UnlicensedHootLog,
            StatusCode::HootLogTooOld,
            StatusCode::HootLogTooNew,
        ] {
            assert!(hoot_load_failed(code));
        }
        assert!(!hoot_load_failed(StatusCode::OK));
        assert!(!hoot_load_failed(StatusCode::NotSupported));
    }
}
