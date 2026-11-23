//! CANcoder absolute encoder (`com.ctre.phoenix6.hardware.CANcoder`).

use crate::can_bus::CanBus;
use crate::configurator::DEFAULT_TIMEOUT_SECONDS;
use crate::device::Device;
use crate::sim::{CANcoderSimState, DeviceType};
use crate::spn;
use crate::status_code::StatusCode;
use crate::status_signal::StatusSignal;
use std::ops::Deref;

#[derive(Debug)]
pub struct CANcoder {
    device: Device,
}

impl Deref for CANcoder {
    type Target = Device;

    fn deref(&self) -> &Device {
        &self.device
    }
}

impl CANcoder {
    /// Fails with `InvalidDeviceId` for an ID outside 0 to 62.
    pub fn new(device_id: i32, network: CanBus) -> Result<Self, StatusCode> {
        Ok(Self {
            device: Device::new(device_id, "cancoder", network, DeviceType::CanCoder)?,
        })
    }

    /// Sets the position reading, in rotations. The absolute position does not change.
    pub fn set_position(&self, new_value_rot: f64) -> StatusCode {
        self.configurator().set_command_value(
            spn::CANcoder_SetSensorPosition,
            new_value_rot,
            DEFAULT_TIMEOUT_SECONDS,
        )
    }

    pub fn clear_sticky_faults(&self) -> StatusCode {
        self.configurator().set_command_value(
            spn::SPN_ClearStickyFaults,
            0.0,
            DEFAULT_TIMEOUT_SECONDS,
        )
    }

    pub fn sim_state(&self) -> CANcoderSimState {
        CANcoderSimState::new(self.device_id())
    }

    /// Position since boot plus the offset, in rotations. Not wrapped.
    pub fn position(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::CANcoder_Position, "Position", true, refresh)
    }

    /// Velocity, in rotations per second.
    pub fn velocity(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::CANcoder_Velocity, "Velocity", true, refresh)
    }

    /// Absolute position, in rotations, wrapped at the discontinuity point.
    pub fn absolute_position(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::CANcoder_AbsPosition, "AbsolutePosition", true, refresh)
    }
}
