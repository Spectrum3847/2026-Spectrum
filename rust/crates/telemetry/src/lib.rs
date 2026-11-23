//! Logging and telemetry: a pure Rust wpilog writer and the DogLog-style logger behind the
//! robot's `Telemetry`.
//!
//! The wpilog layer ([`wpilog`], [`reader`], [`datalog`]) needs no natives. The logger
//! ([`dog_log`]) publishes to NetworkTables and reads the HAL through `wpilib`, which load
//! their libraries at runtime. Robot code calls the free functions in [`telemetry`].

pub mod battery_logger;
pub mod capture;
pub mod data_log_manager;
pub mod datalog;
pub mod dog_log;
pub mod extras;
pub mod native;
pub mod reader;
pub mod system_load_monitor;
pub mod telemetry;
pub mod tune_value;
pub mod value;
pub mod wpilog;
pub mod writers;

pub use battery_logger::BatteryLogger;
pub use dog_log::{DogLog, DogLogOptions};
pub use system_load_monitor::SystemLoadMonitor;
pub use telemetry::{EnumName, PrintPriority, Telemetry};
pub use tune_value::TuneValue;
pub use value::LogValue;
