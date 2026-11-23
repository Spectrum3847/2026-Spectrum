//! The math of one module, free of devices: what a request becomes as motor commands, and how
//! motor readings become a module position and state.
//!
//! CTRE compiles this part. It is reconstructed from the declared members of
//! `SwerveModuleImpl` (`kDriveRotationsPerMeter`, `kDriveNmPerWheelN`,
//! `kCouplingRatioDriveRotorToEncoder`, `kSpeedAt12Volts`), the `ModuleRequest` docs and
//! WPILib's `optimize` and `cosineScale`. Two details come from measuring CTRE's module in the
//! Java sim: the wheel force is projected on the measured steer angle, and the steer velocity
//! behind the coupling correction goes through a single pole low pass filter.

use crate::constants::{ClosedLoopOutputType, ModuleConstants};
use wpimath::kinematics::{SwerveModulePosition, SwerveModuleState};
use wpimath::{geometry::Rotation2d, math_util::input_modulus};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveRequestType {
    OpenLoopVoltage,
    Velocity,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SteerRequestType {
    /// The robot never selects this. The phoenix6 crate has no Motion Magic Expo request, so
    /// it is applied as `Position`.
    MotionMagicExpo,
    Position,
}

/// What a swerve request asks of one module (`SwerveModule.ModuleRequest`).
#[derive(Clone, Copy, Debug)]
pub struct ModuleRequest {
    /// Speed in m/s and direction. Optimized by the module.
    pub state: SwerveModuleState,
    /// Robot-centric wheel force feedforward in newtons, X forward.
    pub wheel_force_feedforward_x: f64,
    pub wheel_force_feedforward_y: f64,
    pub drive_request: DriveRequestType,
    pub steer_request: SteerRequestType,
    /// Seconds. Non-zero adds a velocity feedforward to the steer motor.
    pub update_period: f64,
    /// Voltage control only: FOC commutation, which needs Phoenix Pro.
    pub enable_foc: bool,
}

impl Default for ModuleRequest {
    fn default() -> Self {
        Self {
            state: SwerveModuleState::default(),
            wheel_force_feedforward_x: 0.0,
            wheel_force_feedforward_y: 0.0,
            drive_request: DriveRequestType::OpenLoopVoltage,
            steer_request: SteerRequestType::Position,
            update_period: 0.0,
            enable_foc: true,
        }
    }
}

/// A drive motor output, in the units the phoenix6 controls take.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DriveCommand {
    Coast,
    Voltage {
        volts: f64,
        enable_foc: bool,
    },
    VelocityVoltage {
        rps: f64,
        feed_forward_volts: f64,
        enable_foc: bool,
    },
    VelocityTorqueCurrent {
        rps: f64,
        feed_forward_amps: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SteerCommand {
    Voltage {
        volts: f64,
        enable_foc: bool,
    },
    PositionVoltage {
        rot: f64,
        velocity_rps: f64,
        enable_foc: bool,
    },
    PositionTorqueCurrent {
        rot: f64,
        velocity_rps: f64,
    },
}

/// The motor signals a request reads, in rotations and rotations per second.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModuleReadings {
    pub drive_velocity_rps: f64,
    pub steer_position_rot: f64,
    pub steer_velocity_rps: f64,
    /// Previous output of the steer velocity filter, in rotations per second.
    pub filtered_steer_velocity_rps: f64,
    /// Torque constant and stall current the drive Talon reports. Zero until the Talon has
    /// reported them, which makes the constants model stand in.
    pub drive_kt: f64,
    pub drive_stall_current_a: f64,
}

/// Weight of the newest steer velocity in the coupling filter: a single pole filter with a
/// 50 ms time constant stepped once per 4 ms odometry cycle (`1 - exp(-0.004 / 0.05)`).
pub const COUPLING_FILTER_GAIN: f64 = 0.076_883_653_613_364;

/// A request turned into commands, plus the state to report as the module target.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModulePlan {
    /// Optimized, before cosine scaling.
    pub target: SwerveModuleState,
    pub drive: DriveCommand,
    pub steer: SteerCommand,
    /// The unoptimized direction, kept to derive the next steer velocity feedforward.
    pub requested_angle: Rotation2d,
    /// Steer velocity filter output, kept as the next `filtered_steer_velocity_rps`.
    pub filtered_steer_velocity_rps: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ModuleGeometry {
    rotations_per_meter: f64,
    nm_per_wheel_n: f64,
    coupling_ratio: f64,
    speed_at_12_volts: f64,
    kt: f64,
    stall_current: f64,
    drive_output: ClosedLoopOutputType,
    steer_output: ClosedLoopOutputType,
}

impl ModuleGeometry {
    pub fn new(c: &ModuleConstants) -> Self {
        Self {
            rotations_per_meter: c.drive_rotations_per_meter(),
            nm_per_wheel_n: c.wheel_radius_m / c.drive_motor_gear_ratio,
            coupling_ratio: c.coupling_gear_ratio,
            speed_at_12_volts: c.speed_at_12_volts_mps,
            kt: c.drive_motor_model.kt,
            stall_current: c.drive_motor_model.stall_current_a,
            drive_output: c.drive_motor_closed_loop_output,
            steer_output: c.steer_motor_closed_loop_output,
        }
    }

    pub fn steer_output(&self) -> ClosedLoopOutputType {
        self.steer_output
    }

    /// Odometry position from latency-compensated rotations. The drive rotations the azimuth
    /// caused are removed, since they did not move the robot.
    pub fn position(&self, drive_rot: f64, steer_rot: f64) -> SwerveModulePosition {
        let wheel_rot = drive_rot - steer_rot * self.coupling_ratio;
        SwerveModulePosition::new(
            wheel_rot / self.rotations_per_meter,
            Rotation2d::from_rotations(steer_rot),
        )
    }

    /// Telemetry state from the raw signals. No coupling correction, as in CTRE's header.
    pub fn state(&self, drive_velocity_rps: f64, steer_rot: f64) -> SwerveModuleState {
        SwerveModuleState::new(
            drive_velocity_rps / self.rotations_per_meter,
            Rotation2d::from_rotations(steer_rot),
        )
    }

    /// Motor feedforwards for a robot-centric wheel force, projected on the wheel direction.
    /// Returns (torque current in amps, voltage).
    fn force_feedforwards(
        &self,
        fx: f64,
        fy: f64,
        wheel_angle: Rotation2d,
        readings: &ModuleReadings,
    ) -> (f64, f64) {
        let kt = if readings.drive_kt > 0.0 {
            readings.drive_kt
        } else {
            self.kt
        };
        let stall_current = if readings.drive_stall_current_a > 0.0 {
            readings.drive_stall_current_a
        } else {
            self.stall_current
        };
        let force = fx * wheel_angle.cos() + fy * wheel_angle.sin();
        let torque = force * self.nm_per_wheel_n;
        let amps = torque / kt;
        (amps, amps / stall_current * 12.0)
    }

    pub fn plan(
        &self,
        request: &ModuleRequest,
        readings: &ModuleReadings,
        previous_requested_angle: Rotation2d,
    ) -> ModulePlan {
        let current_angle = Rotation2d::from_rotations(readings.steer_position_rot);
        let mut target = request.state;
        target.optimize(current_angle);
        let mut scaled = target;
        scaled.cosine_scale(current_angle);

        // The azimuth turning drags the drive rotor, so command that much more.
        let filtered_steer_velocity_rps = readings.filtered_steer_velocity_rps
            + COUPLING_FILTER_GAIN
                * (readings.steer_velocity_rps - readings.filtered_steer_velocity_rps);
        let velocity_rps = scaled.speed * self.rotations_per_meter
            + filtered_steer_velocity_rps * self.coupling_ratio;
        let (ff_amps, ff_volts) = self.force_feedforwards(
            request.wheel_force_feedforward_x,
            request.wheel_force_feedforward_y,
            current_angle,
            readings,
        );

        let drive = match (request.drive_request, self.drive_output) {
            (DriveRequestType::OpenLoopVoltage, _) => {
                let volts = if self.speed_at_12_volts == 0.0 {
                    0.0
                } else {
                    velocity_rps / self.rotations_per_meter / self.speed_at_12_volts * 12.0
                };
                DriveCommand::Voltage {
                    volts: volts + ff_volts,
                    enable_foc: request.enable_foc,
                }
            }
            (DriveRequestType::Velocity, ClosedLoopOutputType::Voltage) => {
                DriveCommand::VelocityVoltage {
                    rps: velocity_rps,
                    feed_forward_volts: ff_volts,
                    enable_foc: request.enable_foc,
                }
            }
            (DriveRequestType::Velocity, ClosedLoopOutputType::TorqueCurrentFoc) => {
                DriveCommand::VelocityTorqueCurrent {
                    rps: velocity_rps,
                    feed_forward_amps: ff_amps,
                }
            }
        };

        let steer_velocity_rps = if request.update_period > 0.0 {
            input_modulus(
                request.state.angle.rotations() - previous_requested_angle.rotations(),
                -0.5,
                0.5,
            ) / request.update_period
        } else {
            0.0
        };
        let rot = target.angle.rotations();
        let steer = match self.steer_output {
            ClosedLoopOutputType::Voltage => SteerCommand::PositionVoltage {
                rot,
                velocity_rps: steer_velocity_rps,
                enable_foc: request.enable_foc,
            },
            ClosedLoopOutputType::TorqueCurrentFoc => SteerCommand::PositionTorqueCurrent {
                rot,
                velocity_rps: steer_velocity_rps,
            },
        };

        ModulePlan {
            target,
            drive,
            steer,
            requested_angle: request.state.angle,
            filtered_steer_velocity_rps,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn constants(drive: ClosedLoopOutputType, steer: ClosedLoopOutputType) -> ModuleConstants {
        ModuleConstants {
            drive_motor_gear_ratio: 6.03,
            steer_motor_gear_ratio: 26.09,
            coupling_gear_ratio: 4.5,
            wheel_radius_m: 0.0499,
            speed_at_12_volts_mps: 5.12,
            drive_motor_closed_loop_output: drive,
            steer_motor_closed_loop_output: steer,
            ..ModuleConstants::default()
        }
    }

    fn geometry() -> ModuleGeometry {
        ModuleGeometry::new(&constants(
            ClosedLoopOutputType::TorqueCurrentFoc,
            ClosedLoopOutputType::TorqueCurrentFoc,
        ))
    }

    fn request(speed: f64, angle_rot: f64) -> ModuleRequest {
        ModuleRequest {
            state: SwerveModuleState::new(speed, Rotation2d::from_rotations(angle_rot)),
            drive_request: DriveRequestType::Velocity,
            ..ModuleRequest::default()
        }
    }

    #[test]
    fn position_removes_coupling_and_converts_to_meters() {
        let g = geometry();
        let rpm = 6.03 / (2.0 * PI * 0.0499);
        // Wheel rolled 2 m while the azimuth turned a quarter rotation.
        let p = g.position(2.0 * rpm + 0.25 * 4.5, 0.25);
        assert!((p.distance - 2.0).abs() < 1e-9);
        assert!((p.angle.rotations() - 0.25).abs() < 1e-12);
    }

    #[test]
    fn state_uses_raw_velocity_and_steer_angle() {
        let g = geometry();
        let rpm = 6.03 / (2.0 * PI * 0.0499);
        let s = g.state(3.0 * rpm, 0.1);
        assert!((s.speed - 3.0).abs() < 1e-9);
        assert!((s.angle.rotations() - 0.1).abs() < 1e-12);
    }

    #[test]
    fn aligned_module_gets_full_velocity() {
        let g = geometry();
        let rpm = 6.03 / (2.0 * PI * 0.0499);
        let plan = g.plan(
            &request(2.0, 0.0),
            &ModuleReadings::default(),
            Rotation2d::ZERO,
        );
        match plan.drive {
            DriveCommand::VelocityTorqueCurrent {
                rps,
                feed_forward_amps,
            } => {
                assert!((rps - 2.0 * rpm).abs() < 1e-9);
                assert_eq!(feed_forward_amps, 0.0);
            }
            other => panic!("unexpected {other:?}"),
        }
        assert_eq!(plan.target.speed, 2.0);
    }

    #[test]
    fn optimize_flips_a_module_that_would_turn_past_90_degrees() {
        let g = geometry();
        let rpm = 6.03 / (2.0 * PI * 0.0499);
        // Module at 0, target 170 degrees: drive backwards toward -10 degrees instead.
        let plan = g.plan(
            &request(1.0, 170.0 / 360.0),
            &ModuleReadings::default(),
            Rotation2d::ZERO,
        );
        assert!((plan.target.speed + 1.0).abs() < 1e-12);
        assert!((plan.target.angle.degrees() + 10.0).abs() < 1e-9);
        let DriveCommand::VelocityTorqueCurrent { rps, .. } = plan.drive else {
            panic!("wrong command");
        };
        // cos(10 degrees) scaling on the flipped speed
        let expected = -(10.0_f64.to_radians()).cos() * rpm;
        assert!((rps - expected).abs() < 1e-9);
    }

    #[test]
    fn cosine_scale_cuts_speed_while_the_module_is_off_target() {
        let g = geometry();
        let rpm = 6.03 / (2.0 * PI * 0.0499);
        let readings = ModuleReadings {
            steer_position_rot: -0.125,
            ..ModuleReadings::default()
        };
        let plan = g.plan(&request(1.0, 0.0), &readings, Rotation2d::ZERO);
        let DriveCommand::VelocityTorqueCurrent { rps, .. } = plan.drive else {
            panic!("wrong command");
        };
        assert!((rps - rpm * (PI / 4.0).cos()).abs() < 1e-9);
    }

    #[test]
    fn steer_motion_adds_coupled_drive_velocity_through_a_low_pass_filter() {
        let g = geometry();
        let mut readings = ModuleReadings {
            steer_velocity_rps: 2.0,
            ..ModuleReadings::default()
        };
        let plan = g.plan(&request(0.0, 0.0), &readings, Rotation2d::ZERO);
        let DriveCommand::VelocityTorqueCurrent { rps, .. } = plan.drive else {
            panic!("wrong command");
        };
        assert!((rps - 9.0 * COUPLING_FILTER_GAIN).abs() < 1e-12);

        // Keep applying: the correction approaches the unfiltered 9 rps and never passes it.
        for _ in 0..500 {
            readings.filtered_steer_velocity_rps = plan_filtered(&g, &readings);
        }
        let plan = g.plan(&request(0.0, 0.0), &readings, Rotation2d::ZERO);
        let DriveCommand::VelocityTorqueCurrent { rps, .. } = plan.drive else {
            panic!("wrong command");
        };
        assert!((rps - 9.0).abs() < 1e-6);
    }

    fn plan_filtered(g: &ModuleGeometry, readings: &ModuleReadings) -> f64 {
        g.plan(&request(0.0, 0.0), readings, Rotation2d::ZERO)
            .filtered_steer_velocity_rps
    }

    #[test]
    fn coupling_filter_gain_is_a_50_ms_pole_at_4_ms() {
        assert!((COUPLING_FILTER_GAIN - (1.0 - (-0.004_f64 / 0.05).exp())).abs() < 1e-9);
    }

    #[test]
    fn open_loop_scales_by_speed_at_12_volts() {
        let g = ModuleGeometry::new(&constants(
            ClosedLoopOutputType::Voltage,
            ClosedLoopOutputType::Voltage,
        ));
        let mut req = request(2.56, 0.0);
        req.drive_request = DriveRequestType::OpenLoopVoltage;
        req.enable_foc = false;
        let plan = g.plan(&req, &ModuleReadings::default(), Rotation2d::ZERO);
        let DriveCommand::Voltage { volts, enable_foc } = plan.drive else {
            panic!("wrong command");
        };
        assert!((volts - 6.0).abs() < 1e-9);
        assert!(!enable_foc);
    }

    #[test]
    fn voltage_closed_loop_selects_voltage_requests() {
        let g = ModuleGeometry::new(&constants(
            ClosedLoopOutputType::Voltage,
            ClosedLoopOutputType::Voltage,
        ));
        let plan = g.plan(
            &request(1.0, 0.25),
            &ModuleReadings::default(),
            Rotation2d::ZERO,
        );
        assert!(matches!(plan.drive, DriveCommand::VelocityVoltage { .. }));
        let SteerCommand::PositionVoltage { rot, .. } = plan.steer else {
            panic!("wrong command");
        };
        assert!((rot - 0.25).abs() < 1e-12);
    }

    #[test]
    fn wheel_force_becomes_torque_current_along_the_wheel() {
        let g = geometry();
        let mut req = request(0.0, 0.0);
        req.wheel_force_feedforward_x = 100.0;
        req.wheel_force_feedforward_y = 50.0;
        let plan = g.plan(&req, &ModuleReadings::default(), Rotation2d::ZERO);
        let DriveCommand::VelocityTorqueCurrent {
            feed_forward_amps, ..
        } = plan.drive
        else {
            panic!("wrong command");
        };
        // Wheel points forward, so only the X force counts.
        let expected = 100.0 * (0.0499 / 6.03) / (9.37 / 483.0);
        assert!((feed_forward_amps - expected).abs() < 1e-9);

        let mut left = request(0.0, 0.25);
        left.wheel_force_feedforward_x = 100.0;
        left.wheel_force_feedforward_y = 50.0;
        let at_left = ModuleReadings {
            steer_position_rot: 0.25,
            ..ModuleReadings::default()
        };
        let plan = g.plan(&left, &at_left, Rotation2d::ZERO);
        let DriveCommand::VelocityTorqueCurrent {
            feed_forward_amps, ..
        } = plan.drive
        else {
            panic!("wrong command");
        };
        assert!((feed_forward_amps - expected / 2.0).abs() < 1e-9);
    }

    #[test]
    fn wheel_force_projects_on_the_measured_angle_not_the_target() {
        let g = geometry();
        // Module still at 0 while the target is 90 degrees: the force counts along 0.
        let mut req = request(0.0, 0.25);
        req.wheel_force_feedforward_x = 100.0;
        let plan = g.plan(&req, &ModuleReadings::default(), Rotation2d::ZERO);
        let DriveCommand::VelocityTorqueCurrent {
            feed_forward_amps, ..
        } = plan.drive
        else {
            panic!("wrong command");
        };
        let full = 100.0 * (0.0499 / 6.03) / (9.37 / 483.0);
        assert!((feed_forward_amps - full).abs() < 1e-9);
    }

    #[test]
    fn reported_motor_constants_replace_the_model() {
        let g = geometry();
        let mut req = request(0.0, 0.0);
        req.wheel_force_feedforward_x = 100.0;
        let readings = ModuleReadings {
            drive_kt: 0.0198,
            drive_stall_current_a: 476.0,
            ..ModuleReadings::default()
        };
        let plan = g.plan(&req, &readings, Rotation2d::ZERO);
        let DriveCommand::VelocityTorqueCurrent {
            feed_forward_amps, ..
        } = plan.drive
        else {
            panic!("wrong command");
        };
        assert!((feed_forward_amps - 100.0 * (0.0499 / 6.03) / 0.0198).abs() < 1e-9);
    }

    #[test]
    fn voltage_feedforward_follows_current_over_stall() {
        let g = ModuleGeometry::new(&constants(
            ClosedLoopOutputType::Voltage,
            ClosedLoopOutputType::Voltage,
        ));
        let mut req = request(0.0, 0.0);
        req.wheel_force_feedforward_x = 100.0;
        let plan = g.plan(&req, &ModuleReadings::default(), Rotation2d::ZERO);
        let DriveCommand::VelocityVoltage {
            feed_forward_volts, ..
        } = plan.drive
        else {
            panic!("wrong command");
        };
        let amps = 100.0 * (0.0499 / 6.03) / (9.37 / 483.0);
        assert!((feed_forward_volts - amps / 483.0 * 12.0).abs() < 1e-9);
    }

    #[test]
    fn steer_velocity_feedforward_needs_an_update_period() {
        let g = geometry();
        let mut req = request(0.0, 0.01);
        let plan = g.plan(&req, &ModuleReadings::default(), Rotation2d::ZERO);
        let SteerCommand::PositionTorqueCurrent { velocity_rps, .. } = plan.steer else {
            panic!("wrong command");
        };
        assert_eq!(velocity_rps, 0.0);

        req.update_period = 0.004;
        let plan = g.plan(&req, &ModuleReadings::default(), Rotation2d::ZERO);
        let SteerCommand::PositionTorqueCurrent { velocity_rps, .. } = plan.steer else {
            panic!("wrong command");
        };
        assert!((velocity_rps - 0.01 / 0.004).abs() < 1e-9);
    }

    #[test]
    fn steer_velocity_feedforward_ignores_the_optimizer_flip() {
        let g = geometry();
        let mut req = request(1.0, 0.45);
        req.update_period = 0.004;
        // Requested angle barely moved, though the module is at 0 and the optimizer flips it.
        let plan = g.plan(
            &req,
            &ModuleReadings::default(),
            Rotation2d::from_rotations(0.449),
        );
        let SteerCommand::PositionTorqueCurrent { velocity_rps, .. } = plan.steer else {
            panic!("wrong command");
        };
        assert!((velocity_rps - 0.001 / 0.004).abs() < 1e-9);
    }
}
