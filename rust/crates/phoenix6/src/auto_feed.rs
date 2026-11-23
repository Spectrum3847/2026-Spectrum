//! Simulation enable feeder (`com.ctre.phoenix6.wpiutils.AutoFeedEnable`).
//!
//! Simulated devices stay disabled until something feeds the enable every few tens of
//! milliseconds. Java starts a 10 ms notifier when the first device is built in simulation and
//! stops it when the last one closes; this does the same. It also writes the robot mode and
//! alliance station to the hoot log.
//!
//! On the roboRIO there is no feeder: the C layer reads the robot enable state itself, which
//! is why Java builds the feeder only when `RobotBase.isSimulation()`.

use crate::device::lock;
use crate::signal_logger;
use crate::unmanaged;
use std::sync::{Mutex, Once};
use wpilib::driver_station::{self, AllianceStation};
use wpilib::notifier::Notifier;
use wpilib::shutdown;

const PERIOD_SECONDS: f64 = 0.010;
const ENABLE_TIMEOUT_MS: i32 = 100;

struct Feeder {
    users: u32,
    notifier: Option<Notifier>,
}

static FEEDER: Mutex<Feeder> = Mutex::new(Feeder {
    users: 0,
    notifier: None,
});

static REGISTERED: Once = Once::new();

/// Counts one more device. The first one starts the feeder.
pub fn start() {
    let mut feeder = lock(&FEEDER);
    if feeder.users == u32::MAX {
        return;
    }
    feeder.users += 1;
    if feeder.users != 1 {
        return;
    }
    if feeder.notifier.is_none() {
        match Notifier::new(feed) {
            Ok(notifier) => {
                feeder.notifier = Some(notifier);
                REGISTERED.call_once(|| shutdown::register(shutdown));
            }
            Err(error) => {
                driver_station::report_warning(&format!("Phoenix enable feeder: {error}"));
                return;
            }
        }
    }
    if let Some(notifier) = &feeder.notifier {
        notifier.start_periodic(PERIOD_SECONDS);
    }
}

/// Counts one device less. The last one stops the feeder.
pub fn stop() {
    let mut feeder = lock(&FEEDER);
    if feeder.users == 0 {
        return;
    }
    feeder.users -= 1;
    if feeder.users == 0
        && let Some(notifier) = &feeder.notifier
    {
        notifier.stop();
    }
}

/// Stops the feeder thread and releases its HAL notifier. A later device starts a new one.
pub fn shutdown() {
    let notifier = {
        let mut feeder = lock(&FEEDER);
        feeder.users = 0;
        feeder.notifier.take()
    };
    // Dropped outside the lock: it joins the thread.
    drop(notifier);
}

/// What one feeder tick does, as data, so the mode names can be tested.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Tick {
    enable_timeout_ms: i32,
    mode: &'static str,
}

fn tick(enabled: bool, autonomous: bool, test: bool, estopped: bool) -> Tick {
    if enabled {
        Tick {
            enable_timeout_ms: ENABLE_TIMEOUT_MS,
            mode: if autonomous {
                "Autonomous"
            } else if test {
                "Test"
            } else {
                "Teleop"
            },
        }
    } else {
        Tick {
            enable_timeout_ms: 0,
            mode: if estopped { "EStopped" } else { "Disabled" },
        }
    }
}

fn station_name(station: AllianceStation) -> &'static str {
    match station {
        AllianceStation::Red1 => "Red 1",
        AllianceStation::Red2 => "Red 2",
        AllianceStation::Red3 => "Red 3",
        AllianceStation::Blue1 => "Blue 1",
        AllianceStation::Blue2 => "Blue 2",
        AllianceStation::Blue3 => "Blue 3",
        AllianceStation::Unknown => "",
    }
}

fn feed() {
    let tick = tick(
        driver_station::is_enabled(),
        driver_station::is_autonomous(),
        driver_station::is_test(),
        driver_station::is_estopped(),
    );
    unmanaged::feed_enable(tick.enable_timeout_ms);
    signal_logger::write_string("RobotMode", tick.mode);
    signal_logger::write_string(
        "AllianceStation",
        station_name(driver_station::raw_alliance_station()),
    );
    signal_logger::write_boolean("DS:IsDSAttached", driver_station::is_ds_attached());
    signal_logger::write_boolean("DS:IsFMSAttached", driver_station::is_fms_attached());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shutdown_without_a_feeder_is_idempotent() {
        shutdown();
        shutdown();
        assert!(lock(&FEEDER).notifier.is_none());
    }

    #[test]
    fn enabled_ticks_feed_100_ms_and_name_the_mode() {
        assert_eq!(
            tick(true, true, false, false),
            Tick {
                enable_timeout_ms: 100,
                mode: "Autonomous"
            }
        );
        assert_eq!(tick(true, false, true, false).mode, "Test");
        assert_eq!(tick(true, false, false, false).mode, "Teleop");
        // Autonomous wins over test, as in Java's if chain.
        assert_eq!(tick(true, true, true, false).mode, "Autonomous");
    }

    #[test]
    fn disabled_ticks_feed_zero() {
        assert_eq!(
            tick(false, false, false, false),
            Tick {
                enable_timeout_ms: 0,
                mode: "Disabled"
            }
        );
        assert_eq!(tick(false, false, false, true).mode, "EStopped");
    }

    #[test]
    fn station_names_match_java() {
        assert_eq!(station_name(AllianceStation::Red1), "Red 1");
        assert_eq!(station_name(AllianceStation::Blue3), "Blue 3");
        assert_eq!(station_name(AllianceStation::Unknown), "");
    }
}
