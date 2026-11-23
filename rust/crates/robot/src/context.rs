//! Replacements for the Java static getters (`Robot.getBatteryLogger()` and the like).
//! Each item is set once during robot init on the loop thread and read through a free
//! function. Reading an item before it is set panics with its name.

use crate::rebuilt::shot_calculator::ShotCalculator;
use crate::robot::subsystems::hood::Hood;
use crate::robot::subsystems::intake_extension::IntakeExtension;
use crate::robot::subsystems::super_structure::SuperStructure;
use crate::robot::subsystems::swerve::swerve::{PilotDriveInputs, Swerve};
use commands::Trigger;
use std::cell::OnceCell;
use std::rc::Rc;
use telemetry::BatteryLogger;
use wpilib::dashboard::{Field2d, Mechanism2d};

/// Declares one item: a thread-local slot, a setter that panics if the slot is already
/// full, and a getter that panics with the item name if it is empty. The getter returns a
/// clone, so items are `Rc` handles.
macro_rules! context_item {
    ($(#[$meta:meta])* $item:literal, $setter:ident, $getter:ident, $ty:ty) => {
        mod $getter {
            use super::*;

            thread_local! {
                pub(super) static SLOT: OnceCell<$ty> = const { OnceCell::new() };
            }
        }

        $(#[$meta])*
        pub fn $setter(value: $ty) {
            $getter::SLOT.with(|slot| {
                assert!(slot.set(value).is_ok(), concat!($item, " is already set"));
            });
        }

        $(#[$meta])*
        pub fn $getter() -> $ty {
            $getter::SLOT.with(|slot| {
                slot.get()
                    .unwrap_or_else(|| panic!(concat!($item, " is not set")))
                    .clone()
            })
        }
    };
}

context_item!(
    /// `Robot.getBatteryLogger()`.
    "battery_logger",
    set_battery_logger,
    battery_logger,
    Rc<BatteryLogger>
);

context_item!(
    /// `Robot.getSwerve()`.
    "swerve",
    set_swerve,
    swerve,
    Rc<Swerve>
);

context_item!(
    /// `Robot.getPilot()`, as the drive stick values `Swerve` reads. The T17 `Pilot`
    /// implements [`PilotDriveInputs`].
    "pilot",
    set_pilot,
    pilot,
    Rc<dyn PilotDriveInputs>
);

context_item!(
    /// `Robot.getSuperStructure()`.
    "super_structure",
    set_super_structure,
    super_structure,
    Rc<SuperStructure>
);

context_item!(
    /// `Robot.getHood()`.
    "hood",
    set_hood,
    hood,
    Rc<Hood>
);

context_item!(
    /// `Robot.getIntakeExtension()`.
    "intake_extension",
    set_intake_extension,
    intake_extension,
    Rc<IntakeExtension>
);

context_item!(
    /// `ShotCalculator.getInstance()`.
    "shot_calculator",
    set_shot_calculator,
    shot_calculator,
    Rc<ShotCalculator>
);

context_item!(
    /// `Robot.getField2d()`.
    "field2d",
    set_field2d,
    field2d,
    Field2d
);

context_item!(
    /// `Auton.autonPoseUpdate`, the T17 trigger for fusing vision during auton.
    "auton_pose_update",
    set_auton_pose_update,
    auton_pose_update,
    Trigger
);

context_item!(
    /// `Auton.autonLaunching`, the T17 trigger that is true while auton launches.
    "auton_launching",
    set_auton_launching,
    auton_launching,
    Trigger
);

context_item!(
    /// `RobotSim.leftView`. The subsystem sims draw on it, so set it before they are built,
    /// on the real robot too.
    "left_view",
    set_left_view,
    left_view,
    Mechanism2d
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_getter_returns_what_the_setter_stored() {
        let logger = Rc::new(BatteryLogger::new());
        set_battery_logger(logger.clone());
        assert!(Rc::ptr_eq(&battery_logger(), &logger));
    }

    #[test]
    #[should_panic(expected = "battery_logger is not set")]
    fn reading_an_unset_item_panics_with_its_name() {
        battery_logger();
    }

    #[test]
    #[should_panic(expected = "battery_logger is already set")]
    fn setting_an_item_twice_panics() {
        set_battery_logger(Rc::new(BatteryLogger::new()));
        set_battery_logger(Rc::new(BatteryLogger::new()));
    }

    #[test]
    fn items_are_per_thread() {
        set_battery_logger(Rc::new(BatteryLogger::new()));
        let other_thread_result = std::thread::spawn(|| {
            std::panic::catch_unwind(|| {
                battery_logger();
            })
            .is_err()
        })
        .join()
        .unwrap_or(false);
        assert!(other_thread_result);
    }
}
