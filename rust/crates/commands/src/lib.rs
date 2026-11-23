//! Command-based framework: scheduler, subsystems, triggers.
//!
//! Port of the parts of WPILib v2026.2.1 `wpilibj2.command` that the robot
//! uses. Java names map to snake_case. Classes Java builds with `new` are
//! [`Commands`] factories. The scheduler is a thread-local singleton, like
//! `CommandScheduler.getInstance()`, and the framework is single-threaded.
//! Time and robot mode come from [`env`], which the `wpilib` crate fills in.
//!
//! Ported from WPILib (BSD-3, see `WPILib-License.md` at the repo root).

mod command;
pub mod env;
mod event_loop;
mod factories;
mod groups;
mod scheduler;
mod subsystem;
pub mod sysid;
pub mod testing;
mod timer;
mod trigger;

pub use command::{
    BooleanSupplier, Command, CommandBody, CommandBuilder, InterruptionBehavior, IntoCommands,
};
pub use env::RobotState;
pub use event_loop::EventLoop;
pub use factories::Commands;
pub use scheduler::CommandScheduler;
pub use subsystem::{Subsystem, SubsystemBase, SubsystemId};
pub use trigger::{RobotModeTriggers, Trigger};
