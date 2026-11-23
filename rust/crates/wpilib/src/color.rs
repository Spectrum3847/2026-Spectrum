//! WPILib's `edu.wpi.first.wpilibj.util.Color`.
//!
//! Ported from WPILib v2026.2.1 (BSD-3, see `WPILib-License.md` at the repo root).

use wpimath::math_util;

/// An RGB color with components from 0 to 1, each rounded up to a multiple of 1/4096 as
/// WPILib stores them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Color {
    pub red: f64,
    pub green: f64,
    pub blue: f64,
}

fn round_and_clamp(value: f64) -> f64 {
    math_util::clamp((value * 4096.0).ceil() / 4096.0, 0.0, 1.0)
}

impl Color {
    pub fn new(red: f64, green: f64, blue: f64) -> Self {
        Self {
            red: round_and_clamp(red),
            green: round_and_clamp(green),
            blue: round_and_clamp(blue),
        }
    }

    /// Components from 0 to 255.
    pub fn from_rgb(red: i32, green: i32, blue: i32) -> Self {
        Self::new(
            f64::from(red) / 255.0,
            f64::from(green) / 255.0,
            f64::from(blue) / 255.0,
        )
    }

    /// WPILib writes its named colors as float literals, which widen to these doubles.
    fn named(red: f32, green: f32, blue: f32) -> Self {
        Self::new(f64::from(red), f64::from(green), f64::from(blue))
    }

    pub fn k_black() -> Self {
        Self::named(0.0, 0.0, 0.0)
    }

    pub fn k_blue() -> Self {
        Self::named(0.0, 0.0, 1.0)
    }

    pub fn k_orange() -> Self {
        Self::named(1.0, 0.647_058_84, 0.0)
    }

    pub fn k_purple() -> Self {
        Self::named(0.501_960_8, 0.0, 0.501_960_8)
    }

    pub fn k_red() -> Self {
        Self::named(1.0, 0.0, 0.0)
    }

    pub fn k_white() -> Self {
        Self::named(1.0, 1.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn components_round_up_to_the_1_4096_grid() {
        let color = Color::new(0.0001, 0.5, 1.5);
        assert_eq!(color.red, 1.0 / 4096.0);
        assert_eq!(color.green, 0.5);
        assert_eq!(color.blue, 1.0);
        assert_eq!(Color::new(-1.0, 0.0, 0.0).red, 0.0);
    }

    #[test]
    fn integer_components_divide_by_255() {
        let purple = Color::from_rgb(130, 103, 185);
        assert_eq!(purple.red, round_and_clamp(130.0 / 255.0));
        assert_eq!(purple.green, round_and_clamp(103.0 / 255.0));
        assert_eq!(purple.blue, round_and_clamp(185.0 / 255.0));
    }

    #[test]
    fn eight_bit_round_trip_survives_the_grid() {
        // The LEDs truncate color * 255, so rounding up must never drop a level.
        for level in 0..=255 {
            let color = Color::from_rgb(level, level, level);
            assert_eq!((color.red * 255.0) as i32, level, "level {level}");
        }
    }

    #[test]
    fn named_colors_match_wpilib() {
        assert_eq!(Color::k_black(), Color::new(0.0, 0.0, 0.0));
        assert_eq!(Color::k_white(), Color::new(1.0, 1.0, 1.0));
        assert_eq!(Color::k_red(), Color::new(1.0, 0.0, 0.0));
        assert_eq!(Color::k_blue(), Color::new(0.0, 0.0, 1.0));
        // 0xFFA500 and 0x800080.
        assert_eq!((Color::k_orange().green * 255.0) as i32, 165);
        assert_eq!((Color::k_purple().red * 255.0) as i32, 128);
        assert_eq!((Color::k_purple().blue * 255.0) as i32, 128);
        assert_ne!(Color::k_red(), Color::k_blue());
    }
}
