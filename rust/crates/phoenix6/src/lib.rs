//! CTRE Phoenix 6 devices over the Phoenix C API.
//!
//! The API mirrors CTRE Phoenix 6 Java 26.3.0 and calls CTRE's C layer (`c_ctre_phoenix6_*`),
//! which CTRE does not document as public. The version is pinned. Natives load at runtime, so
//! everything that is pure logic builds and tests without them.
//!
//! Units are Phoenix-native: rotations (`*_rot`), rotations per second (`*_rps`), volts, amps,
//! degrees for the Pigeon 2 and seconds. Constructors return `Result` where Java throws.
//! Getters on devices follow Java: `refresh` argument true reads the bus right away.
//!
//! Behavior notes against Java:
//! - A `StatusSignal` holds one `f64`. The C layer reports one status per call, so after
//!   `refresh_all` every signal in the call shares that status.
//! - `StatusCode` keeps unknown numbers. Java turns them into `OK`.
//! - In simulation a device creation starts the enable feeder Java's `AutoFeedEnable` runs.
//!   On the roboRIO nothing is needed: the library reads the enable state itself.
//!
//! Ported from CTRE sources under the CTRE license. See `THIRD_PARTY.md`.

pub mod auto_feed;
pub mod can_bus;
pub mod cancoder;
pub mod candle;
pub mod configs;
pub mod configurator;
pub mod controls;
pub mod device;
pub mod error_report;
pub mod native;
pub mod pigeon2;
pub mod signal_logger;
pub mod signals;
pub mod sim;
pub mod spn;
pub mod status_code;
pub mod status_signal;
pub mod talon_fx;
pub mod unmanaged;
pub mod utils;

pub use can_bus::{CanBus, CanBusStatus};
pub use cancoder::CANcoder;
pub use candle::CANdle;
pub use configs::{
    CANcoderConfiguration, CANdleConfiguration, ParentConfiguration, Pigeon2Configuration,
    TalonFXConfiguration,
};
pub use configurator::Configurator;
pub use controls::ControlRequest;
pub use pigeon2::Pigeon2;
pub use status_code::StatusCode;
pub use status_signal::{StatusSignal, Timestamp, TimestampSource};
pub use talon_fx::TalonFX;

#[cfg(test)]
mod tests;
