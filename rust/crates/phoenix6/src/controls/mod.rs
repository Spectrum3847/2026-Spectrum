//! Control requests (`com.ctre.phoenix6.controls`).
//!
//! A request goes to the C layer on every `set_control`. `update_freq_hz` is how often the C
//! layer repeats it by itself: 100 Hz for the closed-loop requests, 20 Hz for followers, neutral
//! and animations. Zero sends it once. Java does not resend either, so a robot loop that calls
//! `set_control` each cycle works the same here.

use crate::native::phoenix;
use crate::status_code::StatusCode;
use std::ffi::CStr;

mod generated;

pub use generated::*;

/// A request a device can send (Java `ControlRequest`).
pub trait ControlRequest {
    /// The Java class name, used in error locations.
    fn name(&self) -> &'static str;

    /// Sends the request to one device. `network` is the NUL terminated bus name.
    fn send(&self, network: &CStr, device_hash: u32) -> StatusCode;
}

/// Requests a TalonFX accepts.
pub trait TalonFxControl: ControlRequest {}

/// Requests a CANdle accepts.
pub trait CandleControl: ControlRequest {}

/// Clears the applied request (Java `EmptyControl`). Sent when firmware and API disagree.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EmptyControl;

impl ControlRequest for EmptyControl {
    fn name(&self) -> &'static str {
        "EmptyControl"
    }

    fn send(&self, network: &CStr, device_hash: u32) -> StatusCode {
        // SAFETY: the signature matches the 26.3.0 header; `network` is NUL terminated.
        StatusCode(unsafe {
            (phoenix().c_ctre_phoenix6_RequestControlEmpty)(network.as_ptr(), device_hash, 0.0)
        })
    }
}

impl TalonFxControl for EmptyControl {}
impl CandleControl for EmptyControl {}

macro_rules! control {
    (
        $(#[$meta:meta])*
        pub struct $name:ident($label:literal, $cfn:ident, $marker:ident) {
            new { $( $nf:ident / $nwith:ident : $nty:ty ),* $(,)? }
            defaults { $( $df:ident / $dwith:ident : $dty:ty = $dval:expr ),* $(,)? }
            send($this:ident) [ $( $arg:expr ),* $(,)? ]
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, PartialEq)]
        pub struct $name {
            $( pub $nf: $nty, )*
            $( pub $df: $dty, )*
        }

        impl $name {
            #[allow(clippy::new_without_default)]
            pub fn new( $( $nf: $nty ),* ) -> Self {
                Self { $( $nf, )* $( $df: $dval, )* }
            }

            $(
                #[must_use]
                pub fn $nwith(mut self, value: $nty) -> Self {
                    self.$nf = value;
                    self
                }
            )*

            $(
                #[must_use]
                pub fn $dwith(mut self, value: $dty) -> Self {
                    self.$df = value;
                    self
                }
            )*
        }

        impl $crate::controls::ControlRequest for $name {
            fn name(&self) -> &'static str {
                $label
            }

            fn send(&self, network: &::std::ffi::CStr, device_hash: u32) -> $crate::status_code::StatusCode {
                let $this = self;
                let request = $crate::native::phoenix().$cfn;
                // SAFETY: the signature matches the 26.3.0 header; `network` is NUL terminated.
                $crate::status_code::StatusCode(unsafe {
                    request(network.as_ptr(), device_hash, $( $arg ),*)
                })
            }
        }

        $crate::controls::control!(@marker $marker $name);
    };
    (@marker TalonFx $name:ident) => {
        impl $crate::controls::TalonFxControl for $name {}
    };
    (@marker Candle $name:ident) => {
        impl $crate::controls::CandleControl for $name {}
    };
}
pub(crate) use control;
