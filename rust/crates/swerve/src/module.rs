//! One swerve module: a drive Talon FX, a steer Talon FX and a CANcoder, configured the way
//! CTRE's `SwerveModule` constructor does.

use crate::constants::{ModuleConstants, SteerFeedbackType};
use crate::module_math::{
    DriveCommand, ModuleGeometry, ModuleReadings, ModuleRequest, SteerCommand,
};
use phoenix6::configs::{CANcoderConfiguration, TalonFXConfiguration};
use phoenix6::controls::{
    CoastOut, PositionTorqueCurrentFOC, PositionVoltage, VelocityTorqueCurrentFOC, VelocityVoltage,
    VoltageOut,
};
use phoenix6::signals::{
    FeedbackSensorSourceValue, InvertedValue, NeutralModeValue, SensorDirectionValue,
};
use phoenix6::status_signal::{
    StatusSignal, latency_compensated_value, refresh_all, set_update_frequency_for_all,
};
use phoenix6::{CANcoder, CanBus, StatusCode, TalonFX};
use std::sync::{Mutex, MutexGuard, PoisonError};
use wpimath::geometry::Rotation2d;
use wpimath::kinematics::{SwerveModulePosition, SwerveModuleState};

const CONFIG_ATTEMPTS: usize = 2;

/// Rate of the encoder position and velocity frames the fused steer feedback reads.
const ENCODER_FEEDBACK_HZ: f64 = 100.0;

pub(crate) fn lock<T: ?Sized>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The four signals the odometry thread waits on for each module.
pub(crate) struct ModuleSignals {
    pub drive_position: StatusSignal,
    pub drive_velocity: StatusSignal,
    pub steer_position: StatusSignal,
    pub steer_velocity: StatusSignal,
}

impl ModuleSignals {
    pub(crate) fn iter_mut(&mut self) -> impl Iterator<Item = &mut StatusSignal> {
        [
            &mut self.drive_position,
            &mut self.drive_velocity,
            &mut self.steer_position,
            &mut self.steer_velocity,
        ]
        .into_iter()
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = &StatusSignal> {
        [
            &self.drive_position,
            &self.drive_velocity,
            &self.steer_position,
            &self.steer_velocity,
        ]
        .into_iter()
    }

    /// Latency-compensated drive and steer rotations.
    pub(crate) fn compensated_rotations(&self) -> (f64, f64) {
        (
            latency_compensated_value(&self.drive_position, &self.drive_velocity),
            latency_compensated_value(&self.steer_position, &self.steer_velocity),
        )
    }
}

/// The drive Talon's torque constant and stall current. CTRE reads them once from the device.
struct MotorConstantSignals {
    kt: StatusSignal,
    stall_current: StatusSignal,
    known: bool,
}

struct Cache {
    position: SwerveModulePosition,
    readings: ModuleReadings,
    target: SwerveModuleState,
    requested_angle: Rotation2d,
}

pub struct SwerveModule {
    drive: TalonFX,
    steer: TalonFX,
    encoder: CANcoder,
    geometry: ModuleGeometry,
    pub(crate) signals: Mutex<ModuleSignals>,
    motor_constants: Mutex<MotorConstantSignals>,
    cache: Mutex<Cache>,
}

fn apply_with_retry(label: &str, id: i32, mut apply: impl FnMut() -> StatusCode) {
    let mut status = StatusCode::OK;
    for _ in 0..CONFIG_ATTEMPTS {
        status = apply();
        if status.is_ok() {
            return;
        }
    }
    eprintln!("{label} ID {id} failed config with error {status}");
}

fn drive_configs(c: &ModuleConstants) -> TalonFXConfiguration {
    let mut config = c.drive_motor_initial_configs.clone().unwrap_or_default();
    config.motor_output.neutral_mode = NeutralModeValue::Brake;
    config.slot0 = c.drive_motor_gains.clone();
    config.torque_current.peak_forward_torque_current = c.slip_current_a;
    config.torque_current.peak_reverse_torque_current = -c.slip_current_a;
    config.current_limits.stator_current_limit = c.slip_current_a;
    config.current_limits.stator_current_limit_enable = true;
    config.feedback.rotor_to_sensor_ratio = 1.0;
    config.feedback.sensor_to_mechanism_ratio = 1.0;
    config.motor_output.inverted = if c.drive_motor_inverted {
        InvertedValue::Clockwise_Positive
    } else {
        InvertedValue::CounterClockwise_Positive
    };
    config
}

fn steer_configs(c: &ModuleConstants) -> TalonFXConfiguration {
    let mut config = c.steer_motor_initial_configs.clone().unwrap_or_default();
    config.motor_output.neutral_mode = NeutralModeValue::Brake;
    config.slot0 = c.steer_motor_gains.clone();
    config.feedback.feedback_remote_sensor_id = c.encoder_id;
    config.feedback.feedback_sensor_source = match c.feedback_source {
        SteerFeedbackType::FusedCANcoder => FeedbackSensorSourceValue::FusedCANcoder,
        SteerFeedbackType::SyncCANcoder => FeedbackSensorSourceValue::SyncCANcoder,
        SteerFeedbackType::RemoteCANcoder => FeedbackSensorSourceValue::RemoteCANcoder,
    };
    config.feedback.rotor_to_sensor_ratio = c.steer_motor_gear_ratio;
    config.feedback.sensor_to_mechanism_ratio = 1.0;
    config.motion_magic.motion_magic_expo_k_v = 0.12 * c.steer_motor_gear_ratio;
    config.motion_magic.motion_magic_expo_k_a = 1.2 / c.steer_motor_gear_ratio;
    config.closed_loop_general.continuous_wrap = true;
    config.motor_output.inverted = if c.steer_motor_inverted {
        InvertedValue::Clockwise_Positive
    } else {
        InvertedValue::CounterClockwise_Positive
    };
    config
}

fn encoder_configs(c: &ModuleConstants) -> CANcoderConfiguration {
    let mut config = c.encoder_initial_configs.clone().unwrap_or_default();
    config.magnet_sensor.magnet_offset = c.encoder_offset_rot;
    config.magnet_sensor.sensor_direction = if c.encoder_inverted {
        SensorDirectionValue::Clockwise_Positive
    } else {
        SensorDirectionValue::CounterClockwise_Positive
    };
    config
}

impl SwerveModule {
    /// Creates the three devices and applies their configuration, retrying once like CTRE.
    /// A failed configuration is printed, not returned.
    pub fn new(constants: &ModuleConstants, bus: &CanBus) -> Result<Self, StatusCode> {
        let drive = TalonFX::new(constants.drive_motor_id, bus.clone())?;
        let steer = TalonFX::new(constants.steer_motor_id, bus.clone())?;
        let encoder = CANcoder::new(constants.encoder_id, bus.clone())?;

        let config = drive_configs(constants);
        apply_with_retry("Talon", drive.device_id(), || {
            drive.configurator().apply(&config)
        });
        let config = steer_configs(constants);
        apply_with_retry("Talon", steer.device_id(), || {
            steer.configurator().apply(&config)
        });
        let config = encoder_configs(constants);
        apply_with_retry("Encoder", encoder.device_id(), || {
            encoder.configurator().apply(&config)
        });

        // The steer Talon reads these frames for fused feedback. Optimizing bus utilization turns
        // off every signal nobody requested.
        set_update_frequency_for_all(
            ENCODER_FEEDBACK_HZ,
            &[encoder.position(false), encoder.velocity(false)],
        );

        let signals = ModuleSignals {
            drive_position: drive.position(false),
            drive_velocity: drive.velocity(false),
            steer_position: steer.position(false),
            steer_velocity: steer.velocity(false),
        };
        let motor_constants = MotorConstantSignals {
            kt: drive.motor_kt(false),
            stall_current: drive.motor_stall_current(false),
            known: false,
        };
        Ok(Self {
            drive,
            steer,
            encoder,
            geometry: ModuleGeometry::new(constants),
            signals: Mutex::new(signals),
            motor_constants: Mutex::new(motor_constants),
            cache: Mutex::new(Cache {
                position: SwerveModulePosition::default(),
                readings: ModuleReadings::default(),
                target: SwerveModuleState::default(),
                requested_angle: Rotation2d::ZERO,
            }),
        })
    }

    pub fn drive_motor(&self) -> &TalonFX {
        &self.drive
    }

    pub fn steer_motor(&self) -> &TalonFX {
        &self.steer
    }

    pub fn encoder(&self) -> &CANcoder {
        &self.encoder
    }

    pub fn steer_closed_loop_output(&self) -> crate::constants::ClosedLoopOutputType {
        self.geometry.steer_output()
    }

    /// Stores what the odometry thread just read, so requests and getters see it.
    pub(crate) fn store_readings(&self, signals: &ModuleSignals) -> SwerveModulePosition {
        let (drive_rot, steer_rot) = signals.compensated_rotations();
        let position = self.geometry.position(drive_rot, steer_rot);
        let mut cache = lock(&self.cache);
        cache.position = position;
        cache.readings = ModuleReadings {
            drive_velocity_rps: signals.drive_velocity.value(),
            steer_position_rot: signals.steer_position.value(),
            steer_velocity_rps: signals.steer_velocity.value(),
            filtered_steer_velocity_rps: cache.readings.filtered_steer_velocity_rps,
            drive_kt: cache.readings.drive_kt,
            drive_stall_current_a: cache.readings.drive_stall_current_a,
        };
        position
    }

    /// The module position with latency compensated values. With `refresh` it reads the bus
    /// and blocks until the odometry thread is done with the signals.
    pub fn get_position(&self, refresh: bool) -> SwerveModulePosition {
        let mut signals = lock(&self.signals);
        if refresh {
            let mut refs: Vec<&mut StatusSignal> = signals.iter_mut().collect();
            refresh_all(&mut refs);
        }
        self.store_readings(&signals)
    }

    /// The last position the odometry thread stored, with no refresh.
    pub fn get_cached_position(&self) -> SwerveModulePosition {
        lock(&self.cache).position
    }

    /// Speed and angle from the last signals. Typically used for telemetry.
    pub fn get_current_state(&self) -> SwerveModuleState {
        let readings = lock(&self.cache).readings;
        self.geometry
            .state(readings.drive_velocity_rps, readings.steer_position_rot)
    }

    pub fn get_target_state(&self) -> SwerveModuleState {
        lock(&self.cache).target
    }

    /// Zeroes the drive motor position. The steer position is untouched.
    pub fn reset_position(&self) -> StatusCode {
        self.drive.set_position(0.0)
    }

    /// Applies a request: optimizes the state, scales and compensates the drive speed, and
    /// sends one control to each motor.
    pub fn apply(&self, request: &ModuleRequest) {
        let motor_constants = self.read_motor_constants();
        let plan = {
            let mut cache = lock(&self.cache);
            if let Some((kt, stall_current)) = motor_constants {
                cache.readings.drive_kt = kt;
                cache.readings.drive_stall_current_a = stall_current;
            }
            let plan = self
                .geometry
                .plan(request, &cache.readings, cache.requested_angle);
            cache.target = plan.target;
            cache.requested_angle = plan.requested_angle;
            cache.readings.filtered_steer_velocity_rps = plan.filtered_steer_velocity_rps;
            plan
        };
        self.send(&plan.drive, &plan.steer);
    }

    /// The torque constant and stall current the drive Talon reports, until both are valid.
    fn read_motor_constants(&self) -> Option<(f64, f64)> {
        let mut constants = lock(&self.motor_constants);
        if constants.known {
            return None;
        }
        constants.kt.refresh_with(false);
        constants.stall_current.refresh_with(false);
        let (kt, stall_current) = (constants.kt.value(), constants.stall_current.value());
        if constants.kt.status().is_ok()
            && constants.stall_current.status().is_ok()
            && kt > 0.0
            && stall_current > 0.0
        {
            constants.known = true;
            return Some((kt, stall_current));
        }
        None
    }

    /// Sends controls directly. For characterization only.
    pub fn apply_commands(&self, drive: DriveCommand, steer: SteerCommand) {
        self.send(&drive, &steer);
    }

    fn send(&self, drive: &DriveCommand, steer: &SteerCommand) {
        // Zero update frequency: the odometry loop resends every cycle.
        match *drive {
            DriveCommand::Coast => {
                self.drive
                    .set_control(&CoastOut::new().with_update_freq_hz(0.0));
            }
            DriveCommand::Voltage { volts, enable_foc } => {
                self.drive.set_control(
                    &VoltageOut::new(volts)
                        .with_enable_foc(enable_foc)
                        .with_update_freq_hz(0.0),
                );
            }
            DriveCommand::VelocityVoltage {
                rps,
                feed_forward_volts,
                enable_foc,
            } => {
                self.drive.set_control(
                    &VelocityVoltage::new(rps)
                        .with_feed_forward(feed_forward_volts)
                        .with_enable_foc(enable_foc)
                        .with_update_freq_hz(0.0),
                );
            }
            DriveCommand::VelocityTorqueCurrent {
                rps,
                feed_forward_amps,
            } => {
                self.drive.set_control(
                    &VelocityTorqueCurrentFOC::new(rps)
                        .with_feed_forward(feed_forward_amps)
                        .with_update_freq_hz(0.0),
                );
            }
        }
        match *steer {
            SteerCommand::Voltage { volts, enable_foc } => {
                self.steer.set_control(
                    &VoltageOut::new(volts)
                        .with_enable_foc(enable_foc)
                        .with_update_freq_hz(0.0),
                );
            }
            SteerCommand::PositionVoltage {
                rot,
                velocity_rps,
                enable_foc,
            } => {
                self.steer.set_control(
                    &PositionVoltage::new(rot)
                        .with_velocity(velocity_rps)
                        .with_enable_foc(enable_foc)
                        .with_update_freq_hz(0.0),
                );
            }
            SteerCommand::PositionTorqueCurrent { rot, velocity_rps } => {
                self.steer.set_control(
                    &PositionTorqueCurrentFOC::new(rot)
                        .with_velocity(velocity_rps)
                        .with_update_freq_hz(0.0),
                );
            }
        }
    }
}

/// What a swerve request needs from a module. Lets request math run against a fake.
pub trait ModuleSink {
    fn apply(&self, request: &ModuleRequest);
    fn apply_commands(&self, drive: DriveCommand, steer: SteerCommand);
    fn steer_closed_loop_output(&self) -> crate::constants::ClosedLoopOutputType;
}

impl ModuleSink for SwerveModule {
    fn apply(&self, request: &ModuleRequest) {
        SwerveModule::apply(self, request);
    }

    fn apply_commands(&self, drive: DriveCommand, steer: SteerCommand) {
        SwerveModule::apply_commands(self, drive, steer);
    }

    fn steer_closed_loop_output(&self) -> crate::constants::ClosedLoopOutputType {
        SwerveModule::steer_closed_loop_output(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::ClosedLoopOutputType;

    fn constants() -> ModuleConstants {
        ModuleConstants {
            steer_motor_id: 2,
            drive_motor_id: 1,
            encoder_id: 3,
            encoder_offset_rot: -0.25,
            encoder_inverted: true,
            drive_motor_inverted: true,
            drive_motor_gear_ratio: 6.03,
            steer_motor_gear_ratio: 26.09,
            slip_current_a: 80.0,
            drive_motor_closed_loop_output: ClosedLoopOutputType::TorqueCurrentFoc,
            ..ModuleConstants::default()
        }
    }

    #[test]
    fn drive_configuration_follows_ctre() {
        let c = drive_configs(&constants());
        assert_eq!(c.motor_output.neutral_mode, NeutralModeValue::Brake);
        assert_eq!(c.motor_output.inverted, InvertedValue::Clockwise_Positive);
        assert_eq!(c.torque_current.peak_forward_torque_current, 80.0);
        assert_eq!(c.torque_current.peak_reverse_torque_current, -80.0);
        assert_eq!(c.current_limits.stator_current_limit, 80.0);
        assert!(c.current_limits.stator_current_limit_enable);
        assert_eq!(c.feedback.rotor_to_sensor_ratio, 1.0);
        assert_eq!(c.feedback.sensor_to_mechanism_ratio, 1.0);
    }

    #[test]
    fn drive_configuration_keeps_the_initial_configs() {
        let mut k = constants();
        let mut initial = TalonFXConfiguration::default();
        initial.current_limits.supply_current_limit = 30.0;
        k.drive_motor_initial_configs = Some(initial);
        let c = drive_configs(&k);
        assert_eq!(c.current_limits.supply_current_limit, 30.0);
        assert_eq!(c.current_limits.stator_current_limit, 80.0);
    }

    #[test]
    fn steer_configuration_fuses_the_cancoder() {
        let c = steer_configs(&constants());
        assert_eq!(
            c.feedback.feedback_sensor_source,
            FeedbackSensorSourceValue::FusedCANcoder
        );
        assert_eq!(c.feedback.feedback_remote_sensor_id, 3);
        assert_eq!(c.feedback.rotor_to_sensor_ratio, 26.09);
        assert_eq!(c.feedback.sensor_to_mechanism_ratio, 1.0);
        assert!((c.motion_magic.motion_magic_expo_k_v - 0.12 * 26.09).abs() < 1e-12);
        assert!((c.motion_magic.motion_magic_expo_k_a - 1.2 / 26.09).abs() < 1e-12);
        assert!(c.closed_loop_general.continuous_wrap);
        assert_eq!(
            c.motor_output.inverted,
            InvertedValue::CounterClockwise_Positive
        );
    }

    #[test]
    fn encoder_configuration_sets_offset_and_direction() {
        let c = encoder_configs(&constants());
        assert_eq!(c.magnet_sensor.magnet_offset, -0.25);
        assert_eq!(
            c.magnet_sensor.sensor_direction,
            SensorDirectionValue::Clockwise_Positive
        );
    }
}
