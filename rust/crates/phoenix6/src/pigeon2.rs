//! Pigeon 2 gyro (`com.ctre.phoenix6.hardware.Pigeon2`). Angles are degrees, rates degrees per
//! second and accelerations g, the units the device reports.

use crate::can_bus::CanBus;
use crate::configurator::DEFAULT_TIMEOUT_SECONDS;
use crate::device::Device;
use crate::sim::{DeviceType, Pigeon2SimState};
use crate::spn;
use crate::status_code::StatusCode;
use crate::status_signal::StatusSignal;
use std::ops::Deref;

#[derive(Debug)]
pub struct Pigeon2 {
    device: Device,
}

impl Deref for Pigeon2 {
    type Target = Device;

    fn deref(&self) -> &Device {
        &self.device
    }
}

impl Pigeon2 {
    /// Fails with `InvalidDeviceId` for an ID outside 0 to 62.
    pub fn new(device_id: i32, network: CanBus) -> Result<Self, StatusCode> {
        Ok(Self {
            device: Device::new(device_id, "pigeon 2", network, DeviceType::Pigeon2)?,
        })
    }

    /// Sets the yaw reading, in degrees.
    pub fn set_yaw(&self, new_value_deg: f64) -> StatusCode {
        self.configurator().set_command_value(
            spn::Pigeon2_SetYaw,
            new_value_deg,
            DEFAULT_TIMEOUT_SECONDS,
        )
    }

    /// Sets the yaw reading to zero.
    pub fn reset(&self) -> StatusCode {
        self.set_yaw(0.0)
    }

    pub fn clear_sticky_faults(&self) -> StatusCode {
        self.configurator().set_command_value(
            spn::SPN_ClearStickyFaults,
            0.0,
            DEFAULT_TIMEOUT_SECONDS,
        )
    }

    pub fn sim_state(&self) -> Pigeon2SimState {
        Pigeon2SimState::new(self.device_id())
    }

    /// Yaw, in degrees. Not wrapped.
    pub fn yaw(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::Pigeon2Yaw, "Yaw", true, refresh)
    }

    /// Pitch, in degrees, from -90 to 90.
    pub fn pitch(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::Pigeon2Pitch, "Pitch", true, refresh)
    }

    /// Roll, in degrees, from -180 to 180.
    pub fn roll(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::Pigeon2Roll, "Roll", true, refresh)
    }

    /// Angular velocity about the world X axis, in degrees per second.
    pub fn angular_velocity_x_world(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::Pigeon2AngularVelocityXWorld,
            "AngularVelocityXWorld",
            true,
            refresh,
        )
    }

    pub fn angular_velocity_y_world(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::Pigeon2AngularVelocityYWorld,
            "AngularVelocityYWorld",
            true,
            refresh,
        )
    }

    pub fn angular_velocity_z_world(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::Pigeon2AngularVelocityZWorld,
            "AngularVelocityZWorld",
            true,
            refresh,
        )
    }

    /// Acceleration along the device X axis, in g.
    pub fn acceleration_x(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::Pigeon2AccelerationX, "AccelerationX", true, refresh)
    }

    pub fn acceleration_y(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::Pigeon2AccelerationY, "AccelerationY", true, refresh)
    }
}
