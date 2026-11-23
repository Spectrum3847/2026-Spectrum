//! Simulation inputs and outputs (`com.ctre.phoenix6.sim`).
//!
//! A sim state writes physics inputs into the C layer's simulated device, and reads the outputs
//! the simulated controller computes. Call the setters every simulation loop, after reading
//! `motor_voltage` for the physics step.

use crate::error_report::cstring;
use crate::native::phoenix;
use crate::signals::MagnetHealthValue;
use crate::status_code::StatusCode;

/// Device type numbers of the C layer (`ctre/phoenix/platform/DeviceType.hpp`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceType {
    TalonFx = 4,
    CanCoder = 5,
    Pigeon2 = 6,
    CanDle = 10,
}

/// Which way the simulated mechanism counts positive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum ChassisReference {
    CounterClockwise_Positive,
    Clockwise_Positive,
}

impl ChassisReference {
    /// Orientation negates both what you read from and write to the sim.
    fn sign(self) -> f64 {
        match self {
            Self::CounterClockwise_Positive => 1.0,
            Self::Clockwise_Positive => -1.0,
        }
    }
}

/// Registers a simulated device. The C layer ignores it on a real robot.
pub(crate) fn create(device_type: DeviceType, device_id: i32) -> StatusCode {
    // SAFETY: plain integer arguments.
    let status = StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_platform_sim_create)(device_type as i32, device_id)
    });
    // Java creates a 12 V `supplyVoltage` sim input for every device and pushes it at creation.
    // Without it a Talon FX reports no motor voltage until the robot sets a supply voltage.
    if wpilib::is_simulation() {
        set_input(device_type, device_id, "SupplyVoltage", 12.0);
    }
    status
}

fn set_input(device_type: DeviceType, device_id: i32, name: &str, value: f64) -> StatusCode {
    let name = cstring(name);
    // SAFETY: the name is NUL terminated and outlives the call.
    StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_platform_sim_set_physics_input)(
            device_type as i32,
            device_id,
            name.as_ptr(),
            value,
        )
    })
}

/// A physics value, or zero when the C layer has none by that name. Java returns the number
/// without the status, and so does this. `last_status` has the status.
fn physics_value(device_type: DeviceType, device_id: i32, name: &str) -> f64 {
    let name = cstring(name);
    let mut value = 0.0;
    // SAFETY: the name is NUL terminated and `value` is valid for one write.
    unsafe {
        (phoenix().c_ctre_phoenix6_platform_sim_get_physics_value)(
            device_type as i32,
            device_id,
            name.as_ptr(),
            &mut value,
        );
    }
    value
}

fn last_status(device_type: DeviceType, device_id: i32) -> StatusCode {
    // SAFETY: plain integer arguments.
    StatusCode(unsafe {
        (phoenix().c_ctre_phoenix6_platform_sim_get_last_error)(device_type as i32, device_id)
    })
}

/// Simulated TalonFX (`TalonFXSimState`).
#[derive(Debug, Clone, Copy)]
pub struct TalonFXSimState {
    id: i32,
    /// Mechanical orientation of the sim. Set `Clockwise_Positive` when the simulated
    /// mechanism turns opposite to the rotor.
    pub orientation: ChassisReference,
}

impl TalonFXSimState {
    pub(crate) fn new(device_id: i32) -> Self {
        Self {
            id: device_id,
            orientation: ChassisReference::CounterClockwise_Positive,
        }
    }

    const TYPE: DeviceType = DeviceType::TalonFx;

    /// Status of the last sim call on this device.
    pub fn last_status_code(&self) -> StatusCode {
        last_status(Self::TYPE, self.id)
    }

    /// Voltage the controller applies, in volts.
    pub fn motor_voltage(&self) -> f64 {
        self.orientation.sign() * physics_value(Self::TYPE, self.id, "MotorVoltage")
    }

    /// Torque current the controller applies, in amps.
    pub fn torque_current(&self) -> f64 {
        self.orientation.sign() * physics_value(Self::TYPE, self.id, "TorqueCurrent")
    }

    pub fn supply_current(&self) -> f64 {
        physics_value(Self::TYPE, self.id, "SupplyCurrent")
    }

    pub fn set_supply_voltage(&self, volts: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "SupplyVoltage", volts)
    }

    /// Whether the forward limit switch input is closed.
    pub fn set_forward_limit(&self, closed: bool) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "ForwardLimit",
            f64::from(u8::from(closed)),
        )
    }

    pub fn set_reverse_limit(&self, closed: bool) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "ReverseLimit",
            f64::from(u8::from(closed)),
        )
    }

    /// Sets the rotor position, in rotations, as the sensor would read it before offsets.
    pub fn set_raw_rotor_position(&self, rotations: f64) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "RawRotorPosition",
            self.orientation.sign() * rotations,
        )
    }

    pub fn add_rotor_position(&self, delta_rotations: f64) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "AddRotorPosition",
            self.orientation.sign() * delta_rotations,
        )
    }

    /// Sets the rotor velocity, in rotations per second.
    pub fn set_rotor_velocity(&self, rps: f64) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "RotorVelocity",
            self.orientation.sign() * rps,
        )
    }

    /// Sets the rotor acceleration, in rotations per second squared.
    pub fn set_rotor_acceleration(&self, rps_per_second: f64) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "RotorAcceleration",
            self.orientation.sign() * rps_per_second,
        )
    }
}

/// Simulated CANcoder (`CANcoderSimState`).
#[derive(Debug, Clone, Copy)]
pub struct CANcoderSimState {
    id: i32,
    pub orientation: ChassisReference,
    /// Subtracted from raw positions you set, in rotations.
    pub sensor_offset: f64,
}

impl CANcoderSimState {
    pub(crate) fn new(device_id: i32) -> Self {
        Self {
            id: device_id,
            orientation: ChassisReference::CounterClockwise_Positive,
            sensor_offset: 0.0,
        }
    }

    const TYPE: DeviceType = DeviceType::CanCoder;

    pub fn set_supply_voltage(&self, volts: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "SupplyVoltage", volts)
    }

    /// Sets the position, in rotations, before the magnet offset.
    pub fn set_raw_position(&self, rotations: f64) -> StatusCode {
        let value = (rotations - self.sensor_offset) * self.orientation.sign();
        set_input(Self::TYPE, self.id, "RawPosition", value)
    }

    pub fn add_position(&self, delta_rotations: f64) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "AddPosition",
            self.orientation.sign() * delta_rotations,
        )
    }

    /// Sets the velocity, in rotations per second.
    pub fn set_velocity(&self, rps: f64) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "Velocity",
            self.orientation.sign() * rps,
        )
    }

    pub fn set_magnet_health(&self, health: MagnetHealthValue) -> StatusCode {
        set_input(
            Self::TYPE,
            self.id,
            "MagnetHealth",
            f64::from(health.value()),
        )
    }
}

/// Simulated Pigeon 2 (`Pigeon2SimState`). Angles are degrees, rates degrees per second.
#[derive(Debug, Clone, Copy)]
pub struct Pigeon2SimState {
    id: i32,
}

impl Pigeon2SimState {
    pub(crate) fn new(device_id: i32) -> Self {
        Self { id: device_id }
    }

    const TYPE: DeviceType = DeviceType::Pigeon2;

    pub fn set_supply_voltage(&self, volts: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "SupplyVoltage", volts)
    }

    pub fn set_raw_yaw(&self, degrees: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "RawYaw", degrees)
    }

    pub fn add_yaw(&self, delta_degrees: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "AddYaw", delta_degrees)
    }

    pub fn set_pitch(&self, degrees: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "Pitch", degrees)
    }

    pub fn set_roll(&self, degrees: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "Roll", degrees)
    }

    pub fn set_angular_velocity_x(&self, dps: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "AngularVelocityX", dps)
    }

    pub fn set_angular_velocity_y(&self, dps: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "AngularVelocityY", dps)
    }

    pub fn set_angular_velocity_z(&self, dps: f64) -> StatusCode {
        set_input(Self::TYPE, self.id, "AngularVelocityZ", dps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn device_types_match_the_c_layer() {
        assert_eq!(DeviceType::TalonFx as i32, 4);
        assert_eq!(DeviceType::CanCoder as i32, 5);
        assert_eq!(DeviceType::Pigeon2 as i32, 6);
        assert_eq!(DeviceType::CanDle as i32, 10);
    }

    #[test]
    fn orientation_sign() {
        assert_eq!(ChassisReference::CounterClockwise_Positive.sign(), 1.0);
        assert_eq!(ChassisReference::Clockwise_Positive.sign(), -1.0);
    }

    #[test]
    fn new_states_start_counterclockwise() {
        assert_eq!(
            TalonFXSimState::new(1).orientation,
            ChassisReference::CounterClockwise_Positive
        );
        let cancoder = CANcoderSimState::new(2);
        assert_eq!(
            cancoder.orientation,
            ChassisReference::CounterClockwise_Positive
        );
        assert_eq!(cancoder.sensor_offset, 0.0);
    }
}
