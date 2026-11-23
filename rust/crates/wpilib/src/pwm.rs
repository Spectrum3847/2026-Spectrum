//! Java `PWM` and `Servo` on the 2026 HAL, where the pulse range is set in microseconds.

use crate::hal::{self, HalError, or_report};
use std::panic::Location;

/// How many output cycles the HAL skips between pulses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeriodMultiplier {
    X1,
    X2,
    X4,
}

impl PeriodMultiplier {
    /// Squelch mask for `HAL_SetPWMPeriodScale`: outputs dropped per cycle.
    fn squelch_mask(self) -> i32 {
        match self {
            Self::X4 => 3,
            Self::X2 => 1,
            Self::X1 => 0,
        }
    }
}

/// One PWM output. Disabled while idle and on drop.
#[derive(Debug)]
pub struct Pwm {
    channel: i32,
    handle: i32,
}

impl Pwm {
    #[track_caller]
    pub fn new(channel: i32) -> Result<Self, HalError> {
        Self::init(channel, Location::caller())
    }

    fn init(channel: i32, at: &Location<'_>) -> Result<Self, HalError> {
        if !hal::check_pwm_channel(channel) {
            return Err(HalError::invalid(format!(
                "PWM channel {channel} is out of range"
            )));
        }
        let location = format!("{}:{}", at.file(), at.line());
        let handle = hal::initialize_pwm_port(channel, &location)?;
        let pwm = Self { channel, handle };
        hal::set_pwm_disabled(handle)?;
        hal::set_pwm_eliminate_deadband(handle, false)?;
        hal::report_usage(hal::RESOURCE_PWM, channel + 1);
        Ok(pwm)
    }

    pub fn channel(&self) -> i32 {
        self.channel
    }

    #[track_caller]
    pub fn set_bounds_microseconds(
        &self,
        max: i32,
        deadband_max: i32,
        center: i32,
        deadband_min: i32,
        min: i32,
    ) {
        or_report(hal::set_pwm_config_microseconds(
            self.handle,
            max,
            deadband_max,
            center,
            deadband_min,
            min,
        ));
    }

    #[track_caller]
    pub fn set_period_multiplier(&self, multiplier: PeriodMultiplier) {
        or_report(hal::set_pwm_period_scale(
            self.handle,
            multiplier.squelch_mask(),
        ));
    }

    /// Output position, 0 to 1. Values outside the range are a HAL error.
    #[track_caller]
    pub fn set_position(&self, position: f64) {
        or_report(hal::set_pwm_position(self.handle, position));
    }

    #[track_caller]
    pub fn position(&self) -> f64 {
        or_report(hal::pwm_position(self.handle))
    }

    #[track_caller]
    pub fn set_disabled(&self) {
        or_report(hal::set_pwm_disabled(self.handle));
    }
}

impl Drop for Pwm {
    fn drop(&mut self) {
        let _ = hal::set_pwm_disabled(self.handle);
        hal::free_pwm_port(self.handle);
    }
}

const MAX_ANGLE: f64 = 180.0;
const MIN_ANGLE: f64 = 0.0;
const DEFAULT_MAX_PULSE_US: i32 = 2400;
const DEFAULT_MIN_PULSE_US: i32 = 600;

/// Angle in degrees to PWM position, clamped to the servo range.
fn angle_to_position(degrees: f64) -> f64 {
    (degrees.clamp(MIN_ANGLE, MAX_ANGLE) - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE)
}

fn position_to_angle(position: f64) -> f64 {
    position * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE
}

/// A hobby servo, 0 to 180 degrees over 600 to 2400 microseconds.
#[derive(Debug)]
pub struct Servo {
    pwm: Pwm,
}

impl Servo {
    #[track_caller]
    pub fn new(channel: i32) -> Result<Self, HalError> {
        let pwm = Pwm::init(channel, Location::caller())?;
        pwm.set_bounds_microseconds(DEFAULT_MAX_PULSE_US, 0, 0, 0, DEFAULT_MIN_PULSE_US);
        pwm.set_period_multiplier(PeriodMultiplier::X4);
        hal::report_usage(hal::RESOURCE_SERVO, channel + 1);
        Ok(Self { pwm })
    }

    pub fn channel(&self) -> i32 {
        self.pwm.channel()
    }

    /// Sets the position, 0 to 1.
    #[track_caller]
    pub fn set(&self, value: f64) {
        self.pwm.set_position(value);
    }

    #[track_caller]
    pub fn get(&self) -> f64 {
        self.pwm.position()
    }

    /// Sets the angle in degrees, clamped to 0..=180.
    #[track_caller]
    pub fn set_angle(&self, degrees: f64) {
        self.pwm.set_position(angle_to_position(degrees));
    }

    #[track_caller]
    pub fn angle(&self) -> f64 {
        position_to_angle(self.pwm.position())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn squelch_masks_match_the_java_multipliers() {
        assert_eq!(PeriodMultiplier::X1.squelch_mask(), 0);
        assert_eq!(PeriodMultiplier::X2.squelch_mask(), 1);
        assert_eq!(PeriodMultiplier::X4.squelch_mask(), 3);
    }

    #[test]
    fn angle_clamps_and_scales() {
        assert_eq!(angle_to_position(-10.0), 0.0);
        assert_eq!(angle_to_position(90.0), 0.5);
        assert_eq!(angle_to_position(180.0), 1.0);
        assert_eq!(angle_to_position(500.0), 1.0);
    }

    #[test]
    fn position_converts_back_to_degrees() {
        assert_eq!(position_to_angle(0.25), 45.0);
        assert_eq!(position_to_angle(angle_to_position(120.0)), 120.0);
    }
}
