//! Unit conversions from WPILib `Units`.

const INCHES_PER_FOOT: f64 = 12.0;
const METERS_PER_INCH: f64 = 0.0254;
const METERS_PER_MILE: f64 = 1609.344;
const SECONDS_PER_MINUTE: f64 = 60.0;
const MINUTES_PER_HOUR: f64 = 60.0;
const MILLISECONDS_PER_SECOND: f64 = 1000.0;
const KILOGRAMS_PER_LB: f64 = 0.453592;

pub fn meters_to_feet(meters: f64) -> f64 {
    meters_to_inches(meters) / INCHES_PER_FOOT
}

pub fn feet_to_meters(feet: f64) -> f64 {
    inches_to_meters(feet * INCHES_PER_FOOT)
}

pub fn meters_to_inches(meters: f64) -> f64 {
    meters / METERS_PER_INCH
}

pub fn inches_to_meters(inches: f64) -> f64 {
    inches * METERS_PER_INCH
}

pub fn degrees_to_radians(degrees: f64) -> f64 {
    degrees.to_radians()
}

pub fn radians_to_degrees(radians: f64) -> f64 {
    radians.to_degrees()
}

pub fn radians_to_rotations(radians: f64) -> f64 {
    radians / (std::f64::consts::PI * 2.0)
}

pub fn degrees_to_rotations(degrees: f64) -> f64 {
    degrees / 360.0
}

pub fn rotations_to_degrees(rotations: f64) -> f64 {
    rotations * 360.0
}

pub fn rotations_to_radians(rotations: f64) -> f64 {
    rotations * 2.0 * std::f64::consts::PI
}

pub fn rotations_per_minute_to_radians_per_second(rpm: f64) -> f64 {
    rpm * std::f64::consts::PI / (SECONDS_PER_MINUTE / 2.0)
}

pub fn radians_per_second_to_rotations_per_minute(radians_per_second: f64) -> f64 {
    radians_per_second * (SECONDS_PER_MINUTE / 2.0) / std::f64::consts::PI
}

pub fn miles_per_hour_to_meters_per_second(mph: f64) -> f64 {
    mph * METERS_PER_MILE / (SECONDS_PER_MINUTE * MINUTES_PER_HOUR)
}

pub fn meters_per_second_to_miles_per_hour(meters_per_second: f64) -> f64 {
    meters_per_second / METERS_PER_MILE * (SECONDS_PER_MINUTE * MINUTES_PER_HOUR)
}

pub fn milliseconds_to_seconds(milliseconds: f64) -> f64 {
    milliseconds / MILLISECONDS_PER_SECOND
}

pub fn seconds_to_milliseconds(seconds: f64) -> f64 {
    seconds * MILLISECONDS_PER_SECOND
}

pub fn kilograms_to_lbs(kilograms: f64) -> f64 {
    kilograms / KILOGRAMS_PER_LB
}

pub fn lbs_to_kilograms(lbs: f64) -> f64 {
    lbs * KILOGRAMS_PER_LB
}
