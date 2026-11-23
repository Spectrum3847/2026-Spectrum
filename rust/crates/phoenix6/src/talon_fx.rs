//! TalonFX motor controller (`com.ctre.phoenix6.hardware.TalonFX`).

use crate::can_bus::CanBus;
use crate::configurator::DEFAULT_TIMEOUT_SECONDS;
use crate::controls::{NeutralOut, TalonFxControl};
use crate::device::Device;
use crate::sim::{DeviceType, TalonFXSimState};
use crate::spn;
use crate::status_code::StatusCode;
use crate::status_signal::StatusSignal;
use std::ops::Deref;

#[derive(Debug)]
pub struct TalonFX {
    device: Device,
    neutral: NeutralOut,
}

impl Deref for TalonFX {
    type Target = Device;

    fn deref(&self) -> &Device {
        &self.device
    }
}

impl TalonFX {
    /// Fails with `InvalidDeviceId` for an ID outside 0 to 62.
    pub fn new(device_id: i32, network: CanBus) -> Result<Self, StatusCode> {
        let device = Device::new(device_id, "talon fx", network, DeviceType::TalonFx)?;
        Ok(Self {
            device,
            neutral: NeutralOut::new(),
        })
    }

    pub fn set_control(&self, request: &(impl TalonFxControl + ?Sized)) -> StatusCode {
        self.device.set_control_request(request)
    }

    /// Requests neutral output, so the motor brakes or coasts per its neutral mode.
    pub fn stop_motor(&self) -> StatusCode {
        self.set_control(&self.neutral)
    }

    /// Sets the position reading, in rotations, without moving the motor.
    pub fn set_position(&self, new_value_rot: f64) -> StatusCode {
        self.set_position_with_timeout(new_value_rot, DEFAULT_TIMEOUT_SECONDS)
    }

    pub fn set_position_with_timeout(
        &self,
        new_value_rot: f64,
        timeout_seconds: f64,
    ) -> StatusCode {
        self.configurator().set_command_value(
            spn::TalonFX_SetSensorPosition,
            new_value_rot,
            timeout_seconds,
        )
    }

    pub fn clear_sticky_faults(&self) -> StatusCode {
        self.clear_sticky_faults_with_timeout(DEFAULT_TIMEOUT_SECONDS)
    }

    pub fn clear_sticky_faults_with_timeout(&self, timeout_seconds: f64) -> StatusCode {
        self.configurator()
            .set_command_value(spn::SPN_ClearStickyFaults, 0.0, timeout_seconds)
    }

    pub fn sim_state(&self) -> TalonFXSimState {
        TalonFXSimState::new(self.device_id())
    }

    /// Mechanism position, in rotations, after the sensor and gear ratios.
    pub fn position(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::PRO_PosAndVel_Position, "Position", true, refresh)
    }

    /// Mechanism velocity, in rotations per second.
    pub fn velocity(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::PRO_PosAndVel_Velocity, "Velocity", true, refresh)
    }

    /// Voltage applied to the motor, in volts.
    pub fn motor_voltage(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::PRO_MotorOutput_MotorVoltage,
            "MotorVoltage",
            true,
            refresh,
        )
    }

    /// Duty cycle, from -1 to 1.
    pub fn duty_cycle(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::PRO_MotorOutput_DutyCycle, "DutyCycle", true, refresh)
    }

    /// Torque current, in amps.
    pub fn torque_current(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::PRO_MotorOutput_TorqueCurrent,
            "TorqueCurrent",
            true,
            refresh,
        )
    }

    /// Torque constant of the motor, in N*m per amp.
    pub fn motor_kt(&self, refresh: bool) -> StatusSignal {
        self.signal(spn::TalonFX_MotorConstants_kT, "MotorKT", true, refresh)
    }

    /// Stall current of the motor, in amps.
    pub fn motor_stall_current(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::TalonFX_MotorConstants_StallCurrent,
            "MotorStallCurrent",
            true,
            refresh,
        )
    }

    /// Stator current magnitude, in amps.
    pub fn stator_current(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::PRO_SupplyAndTemp_StatorCurrent,
            "StatorCurrent",
            true,
            refresh,
        )
    }

    /// Supply current, in amps.
    pub fn supply_current(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::PRO_SupplyAndTemp_SupplyCurrent,
            "SupplyCurrent",
            true,
            refresh,
        )
    }

    /// Device temperature, in degrees Celsius.
    pub fn device_temp(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::PRO_SupplyAndTemp_DeviceTemp,
            "DeviceTemp",
            true,
            refresh,
        )
    }

    /// Rotor velocity before any ratios, in rotations per second.
    pub fn rotor_velocity(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::PRO_RotorPosAndVel_Velocity,
            "RotorVelocity",
            true,
            refresh,
        )
    }

    /// Rotor position before any ratios, in rotations.
    pub fn rotor_position(&self, refresh: bool) -> StatusSignal {
        self.signal(
            spn::PRO_RotorPosAndVel_Position,
            "RotorPosition",
            true,
            refresh,
        )
    }
}
