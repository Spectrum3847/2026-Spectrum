//! CANdle LED controller (`com.ctre.phoenix6.hardware.CANdle`).

use crate::can_bus::CanBus;
use crate::configurator::DEFAULT_TIMEOUT_SECONDS;
use crate::controls::CandleControl;
use crate::device::Device;
use crate::sim::DeviceType;
use crate::spn;
use crate::status_code::StatusCode;
use std::ops::Deref;

/// Animation slots 0 to 7 (`CANdle.kAnimSlotCount`).
pub const ANIM_SLOT_COUNT: i32 = 8;

#[derive(Debug)]
pub struct CANdle {
    device: Device,
}

impl Deref for CANdle {
    type Target = Device;

    fn deref(&self) -> &Device {
        &self.device
    }
}

impl CANdle {
    /// Fails with `InvalidDeviceId` for an ID outside 0 to 62.
    pub fn new(device_id: i32, network: CanBus) -> Result<Self, StatusCode> {
        Ok(Self {
            device: Device::new(device_id, "candle", network, DeviceType::CanDle)?,
        })
    }

    pub fn set_control(&self, request: &(impl CandleControl + ?Sized)) -> StatusCode {
        self.device.set_control_request(request)
    }

    pub fn clear_sticky_faults(&self) -> StatusCode {
        self.configurator().set_command_value(
            spn::SPN_ClearStickyFaults,
            0.0,
            DEFAULT_TIMEOUT_SECONDS,
        )
    }
}
