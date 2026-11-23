//! Device enums and the LED color type (`com.ctre.phoenix6.signals`).

mod enums;

pub use enums::*;

/// An LED color with channels 0 to 255 (`RGBWColor`). Channels are not clamped, as in Java.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct RgbwColor {
    pub red: i32,
    pub green: i32,
    pub blue: i32,
    pub white: i32,
}

impl RgbwColor {
    pub const fn new(red: i32, green: i32, blue: i32) -> Self {
        Self::new_rgbw(red, green, blue, 0)
    }

    pub const fn new_rgbw(red: i32, green: i32, blue: i32, white: i32) -> Self {
        Self {
            red,
            green,
            blue,
            white,
        }
    }
}
