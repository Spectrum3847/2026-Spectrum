//! WPILib core over the native HAL C API: driver station, timers, notifiers, the timed
//! robot loop, joysticks, PWM and power distribution, plus NetworkTables and dashboards.
//!
//! Natives load at runtime, so everything that is pure logic builds and tests without them.
//! Getters report HAL failures to the driver station and return a zero value, where Java
//! would throw. Constructors return `Result`.

pub mod color;
pub mod dashboard;
pub mod driver_station;
pub mod filesystem;
pub mod hal;
pub mod hid;
pub mod hooks;
pub mod native;
pub mod notifier;
pub mod nt;
pub mod power_distribution;
pub mod pwm;
pub mod robot;
pub mod robot_controller;
pub mod shutdown;
pub mod timer;
pub mod watchdog;
pub mod web_server;

pub use hal::{HalError, RuntimeType, is_real, is_simulation};
pub use robot::{Robot, Stopper, TimedRobot, start_robot};
