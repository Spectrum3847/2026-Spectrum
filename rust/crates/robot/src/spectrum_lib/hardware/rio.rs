//! Identifies the roboRIO that is running, keyed by its serial number, so a robot-specific
//! config can be selected at startup.
//!
//! Serial numbers are on the label on the back of the roboRIO, and a short one needs a
//! leading zero. Reflashing the roboRIO can change the serial number.
//!
//! Based on:
//! https://github.com/Team100/all24/blob/2a109b28467cfddcafb93c7fc85ef60b56a628a2/lib/src/main/java/org/team100/lib/config/Identity.java

use std::cell::RefCell;
use std::sync::OnceLock;
use telemetry::telemetry;
use wpilib::dashboard::{Alert, AlertType};
use wpilib::robot_controller;

/// Variant names are the Java constants in camel case. [`Rio::name`] gives the Java
/// constant name that alerts and logs show.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rio {
    Photon2026,
    Pm2026,
    Fm2026,
    Om2026,
    Fm2025,
    Fm2024,
    Sim,
    Unknown,
}

static ID: OnceLock<Rio> = OnceLock::new();

thread_local! {
    // Alerts clear when dropped, so the ones `check_id` raises live for the loop thread.
    static ALERTS: RefCell<Vec<Alert>> = const { RefCell::new(Vec::new()) };
}

impl Rio {
    /// Bus name that selects the first CANivore the system has.
    pub const CANIVORE: &'static str = "*";
    /// Bus name of the roboRIO's own CAN interface.
    pub const RIO_CANBUS: &'static str = "rio";

    const ALL: [Rio; 8] = [
        Rio::Photon2026,
        Rio::Pm2026,
        Rio::Fm2026,
        Rio::Om2026,
        Rio::Fm2025,
        Rio::Fm2024,
        Rio::Sim,
        Rio::Unknown,
    ];

    /// `None` for [`Rio::Unknown`]. An empty serial is a placeholder for a roboRIO nobody
    /// has recorded yet.
    fn serial_number(self) -> Option<&'static str> {
        match self {
            Rio::Photon2026 => Some("032B4BB3"),
            Rio::Pm2026 => Some("0329AD07"),
            Rio::Fm2026 | Rio::Om2026 => Some(""),
            Rio::Fm2025 => Some("0329F2D1"),
            Rio::Fm2024 => Some("032B1F69"),
            // The only constant allowed a blank serial, so it claims the simulator.
            Rio::Sim => Some(""),
            Rio::Unknown => None,
        }
    }

    /// The Java constant name.
    pub fn name(self) -> &'static str {
        match self {
            Rio::Photon2026 => "PHOTON2026",
            Rio::Pm2026 => "PM_2026",
            Rio::Fm2026 => "FM_2026",
            Rio::Om2026 => "OM_2026",
            Rio::Fm2025 => "FM_2025",
            Rio::Fm2024 => "FM_2024",
            Rio::Sim => "SIM",
            Rio::Unknown => "UNKNOWN",
        }
    }

    /// True for a RIO 2.0 controller, false for a RIO 1.0.
    pub fn is_rio2(self) -> bool {
        true
    }

    /// The constant that owns `serial`. A blank serial is a placeholder for a roboRIO
    /// nobody has recorded yet. Only `Sim` may hold one, or whichever blank constant came
    /// last would claim the simulator.
    pub fn from_serial(serial: &str) -> Option<Rio> {
        Self::ALL.into_iter().find(|rio| {
            rio.serial_number()
                .is_some_and(|s| (*rio == Rio::Sim || !s.is_empty()) && s == serial)
        })
    }

    /// The constant matching the roboRIO this code is running on, resolved on first use.
    /// The first call raises the alerts, so make it on the loop thread during init.
    pub fn id() -> Rio {
        *ID.get_or_init(check_id)
    }
}

fn check_id() -> Rio {
    let rio_id_alert = Alert::new("RIO: ", AlertType::Info);
    let rio_id_unknown = Alert::new("UNKNOWN RIO: ", AlertType::Error);
    let rio1_alert = Alert::new("RIO 1.0", AlertType::Warning);

    let mut serial_number = String::new();
    if wpilib::is_real() {
        // Java: RobotController.getSerialNumber SEGVs under a VS Code unit test, so it
        // only runs on real hardware.
        serial_number = robot_controller::serial_number();
        telemetry::print(&format!("RIO SERIAL: {serial_number}"));
    }

    let id = match Rio::from_serial(&serial_number) {
        Some(id) => {
            rio_id_alert.set_text(&format!("Rio: {}", id.name()));
            rio_id_alert.set(true);

            telemetry::print(&format!("RIO NAME: {}", id.name()));
            if !id.is_rio2() {
                rio1_alert.set(true);
            }
            id
        }
        None => {
            rio_id_unknown.set_text(&format!("Unknown Rio: {serial_number}"));
            rio_id_unknown.set(true);
            Rio::Unknown
        }
    };
    ALERTS.with(|a| {
        a.borrow_mut()
            .extend([rio_id_alert, rio_id_unknown, rio1_alert])
    });
    id
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_serials_map_to_their_rio() {
        assert_eq!(Rio::from_serial("032B4BB3"), Some(Rio::Photon2026));
        assert_eq!(Rio::from_serial("0329AD07"), Some(Rio::Pm2026));
        assert_eq!(Rio::from_serial("0329F2D1"), Some(Rio::Fm2025));
        assert_eq!(Rio::from_serial("032B1F69"), Some(Rio::Fm2024));
    }

    #[test]
    fn a_blank_serial_is_the_simulator() {
        assert_eq!(Rio::from_serial(""), Some(Rio::Sim));
    }

    #[test]
    fn blank_placeholder_constants_never_claim_a_serial() {
        for rio in [Rio::Fm2026, Rio::Om2026] {
            assert_eq!(rio.serial_number(), Some(""));
            assert_ne!(Rio::from_serial(""), Some(rio));
        }
    }

    #[test]
    fn unknown_serials_have_no_match() {
        assert_eq!(Rio::from_serial("DEADBEEF"), None);
        assert_eq!(Rio::from_serial("32B4BB3"), None);
        assert_eq!(Rio::from_serial("032b4bb3"), None);
    }

    #[test]
    fn unknown_has_no_serial_and_is_never_looked_up() {
        assert_eq!(Rio::Unknown.serial_number(), None);
        assert!(Rio::ALL.iter().all(|r| Rio::from_serial("x") != Some(*r)));
    }

    #[test]
    fn names_are_the_java_constants() {
        assert_eq!(Rio::Photon2026.name(), "PHOTON2026");
        assert_eq!(Rio::Pm2026.name(), "PM_2026");
        assert_eq!(Rio::Fm2026.name(), "FM_2026");
        assert_eq!(Rio::Om2026.name(), "OM_2026");
        assert_eq!(Rio::Fm2025.name(), "FM_2025");
        assert_eq!(Rio::Fm2024.name(), "FM_2024");
        assert_eq!(Rio::Sim.name(), "SIM");
        assert_eq!(Rio::Unknown.name(), "UNKNOWN");
    }

    #[test]
    fn every_rio_is_a_rio2() {
        assert!(Rio::ALL.iter().all(|r| r.is_rio2()));
    }

    #[test]
    fn bus_names() {
        assert_eq!(Rio::CANIVORE, "*");
        assert_eq!(Rio::RIO_CANBUS, "rio");
    }
}
