//! Swerve requests (`SwerveRequest`): what the drivetrain applies to its modules every
//! odometry cycle.
//!
//! Direct ports of the `Apply` bodies in CTRE's `SwerveRequest.hpp`. Kept: `Idle`,
//! `SwerveDriveBrake`, `FieldCentric`, `ApplyRobotSpeeds`, `ApplyFieldSpeeds`,
//! `FieldCentricFacingAngle` and the three SysId requests. Not ported, the robot never uses
//! them: `RobotCentric`, `PointWheelsAt`, `RobotCentricFacingAngle`.

use crate::constants::ClosedLoopOutputType;
use crate::module::ModuleSink;
use crate::module_math::{
    DriveCommand, DriveRequestType, ModuleRequest, SteerCommand, SteerRequestType,
};
use crate::pid::PhoenixPidController;
use phoenix6::StatusCode;
use std::f64::consts::PI;
use wpimath::geometry::{Rotation2d, Translation2d};
use wpimath::kinematics::{ChassisSpeeds, SwerveDriveKinematics, SwerveModuleState};

pub const NUM_MODULES: usize = 4;

/// What the drivetrain tells a request on each cycle.
pub struct ControlParameters<'a> {
    pub kinematics: &'a mut SwerveDriveKinematics<NUM_MODULES>,
    pub module_locations: [Translation2d; NUM_MODULES],
    /// Fastest module speed at 12 V, m/s.
    pub max_speed: f64,
    pub operator_forward_direction: Rotation2d,
    pub current_chassis_speed: ChassisSpeeds,
    pub current_pose: wpimath::geometry::Pose2d,
    /// Phoenix clock, seconds.
    pub timestamp: f64,
    /// Seconds since the previous cycle.
    pub update_period: f64,
}

/// Applied by the odometry thread each cycle. Use it through `Arc<Mutex<_>>` so the robot can
/// edit it between cycles, like the Java robot edits its request objects.
pub trait SwerveRequest: Send {
    fn apply(
        &mut self,
        parameters: &mut ControlParameters<'_>,
        modules: &[&dyn ModuleSink],
    ) -> StatusCode;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ForwardPerspective {
    /// Forward is the direction set with `set_operator_perspective_forward`.
    OperatorPerspective,
    /// Forward is the blue alliance wall direction, +X on the field.
    BlueAlliance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetDirectionPerspective {
    UseForwardPerspective,
    BlueAlliance,
}

fn rotate_xy(x: f64, y: f64, by: Rotation2d) -> (f64, f64) {
    let t = Translation2d::new(x, y).rotate_by(by);
    (t.x, t.y)
}

/// Sends the states to the modules, with optional per-module wheel force feedforwards.
fn apply_states(
    parameters: &ControlParameters<'_>,
    modules: &[&dyn ModuleSink],
    states: &[SwerveModuleState],
    drive_request: DriveRequestType,
    steer_request: SteerRequestType,
    force_feedforwards: &[(f64, f64)],
) {
    let mut request = ModuleRequest {
        drive_request,
        steer_request,
        update_period: parameters.update_period,
        ..ModuleRequest::default()
    };
    for (i, module) in modules.iter().enumerate() {
        // Like CTRE, once a module has a feedforward the next ones keep using the last value
        // when the lists run short.
        if let Some(&(x, y)) = force_feedforwards.get(i) {
            request.wheel_force_feedforward_x = x;
            request.wheel_force_feedforward_y = y;
        }
        request.state = states[i];
        module.apply(&request);
    }
}

fn desaturate(
    states: &mut [SwerveModuleState],
    desaturate_wheel_speeds: bool,
    parameters: &ControlParameters<'_>,
) {
    if desaturate_wheel_speeds && parameters.max_speed > 0.0 {
        SwerveDriveKinematics::<NUM_MODULES>::desaturate_wheel_speeds(states, parameters.max_speed);
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Idle;

impl SwerveRequest for Idle {
    fn apply(&mut self, _: &mut ControlParameters<'_>, _: &[&dyn ModuleSink]) -> StatusCode {
        StatusCode::OK
    }
}

/// Points every wheel at the robot center (an X) with no speed.
#[derive(Clone, Copy, Debug)]
pub struct SwerveDriveBrake {
    pub drive_request_type: DriveRequestType,
    pub steer_request_type: SteerRequestType,
}

impl Default for SwerveDriveBrake {
    fn default() -> Self {
        Self {
            drive_request_type: DriveRequestType::OpenLoopVoltage,
            steer_request_type: SteerRequestType::Position,
        }
    }
}

impl SwerveRequest for SwerveDriveBrake {
    fn apply(
        &mut self,
        parameters: &mut ControlParameters<'_>,
        modules: &[&dyn ModuleSink],
    ) -> StatusCode {
        let mut request = ModuleRequest {
            drive_request: self.drive_request_type,
            steer_request: self.steer_request_type,
            update_period: parameters.update_period,
            ..ModuleRequest::default()
        };
        for (i, module) in modules.iter().enumerate() {
            request.state = SwerveModuleState::new(0.0, parameters.module_locations[i].angle());
            module.apply(&request);
        }
        StatusCode::OK
    }
}

/// Drives with field-centric velocities, rotated by the operator perspective by default.
#[derive(Clone, Copy, Debug)]
pub struct FieldCentric {
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub rotational_rate: f64,
    pub deadband: f64,
    pub rotational_deadband: f64,
    pub center_of_rotation: Translation2d,
    pub drive_request_type: DriveRequestType,
    pub steer_request_type: SteerRequestType,
    pub desaturate_wheel_speeds: bool,
    pub forward_perspective: ForwardPerspective,
}

impl Default for FieldCentric {
    fn default() -> Self {
        Self {
            velocity_x: 0.0,
            velocity_y: 0.0,
            rotational_rate: 0.0,
            deadband: 0.0,
            rotational_deadband: 0.0,
            center_of_rotation: Translation2d::ZERO,
            drive_request_type: DriveRequestType::OpenLoopVoltage,
            steer_request_type: SteerRequestType::Position,
            desaturate_wheel_speeds: true,
            forward_perspective: ForwardPerspective::OperatorPerspective,
        }
    }
}

impl SwerveRequest for FieldCentric {
    fn apply(
        &mut self,
        parameters: &mut ControlParameters<'_>,
        modules: &[&dyn ModuleSink],
    ) -> StatusCode {
        let (mut x, mut y, mut omega) = (self.velocity_x, self.velocity_y, self.rotational_rate);
        if self.forward_perspective == ForwardPerspective::OperatorPerspective {
            (x, y) = rotate_xy(x, y, parameters.operator_forward_direction);
        }
        if x.hypot(y) < self.deadband {
            x = 0.0;
            y = 0.0;
        }
        if omega.abs() < self.rotational_deadband {
            omega = 0.0;
        }
        let speeds = ChassisSpeeds::from_field_relative(
            ChassisSpeeds::new(x, y, omega),
            parameters.current_pose.rotation,
        )
        .discretize(parameters.update_period);
        let mut states = parameters
            .kinematics
            .to_swerve_module_states(speeds, self.center_of_rotation);
        desaturate(&mut states, self.desaturate_wheel_speeds, parameters);
        apply_states(
            parameters,
            modules,
            &states,
            self.drive_request_type,
            self.steer_request_type,
            &[],
        );
        StatusCode::OK
    }
}

/// Drives with robot-centric speeds. PathPlanner uses it with wheel force feedforwards.
#[derive(Clone, Debug)]
pub struct ApplyRobotSpeeds {
    pub speeds: ChassisSpeeds,
    /// Robot-centric force per module, newtons. When the lists are shorter than the module
    /// count, the modules past the end keep the last value, as in CTRE.
    pub wheel_force_feedforwards_x: Vec<f64>,
    pub wheel_force_feedforwards_y: Vec<f64>,
    pub center_of_rotation: Translation2d,
    pub drive_request_type: DriveRequestType,
    pub steer_request_type: SteerRequestType,
    pub desaturate_wheel_speeds: bool,
}

impl Default for ApplyRobotSpeeds {
    fn default() -> Self {
        Self {
            speeds: ChassisSpeeds::default(),
            wheel_force_feedforwards_x: Vec::new(),
            wheel_force_feedforwards_y: Vec::new(),
            center_of_rotation: Translation2d::ZERO,
            drive_request_type: DriveRequestType::OpenLoopVoltage,
            steer_request_type: SteerRequestType::Position,
            desaturate_wheel_speeds: true,
        }
    }
}

/// Pairs the X and Y lists, stopping at the shorter one.
fn paired(x: &[f64], y: &[f64]) -> Vec<(f64, f64)> {
    x.iter().copied().zip(y.iter().copied()).collect()
}

impl SwerveRequest for ApplyRobotSpeeds {
    fn apply(
        &mut self,
        parameters: &mut ControlParameters<'_>,
        modules: &[&dyn ModuleSink],
    ) -> StatusCode {
        let mut states = parameters
            .kinematics
            .to_swerve_module_states(self.speeds, self.center_of_rotation);
        desaturate(&mut states, self.desaturate_wheel_speeds, parameters);
        apply_states(
            parameters,
            modules,
            &states,
            self.drive_request_type,
            self.steer_request_type,
            &paired(
                &self.wheel_force_feedforwards_x,
                &self.wheel_force_feedforwards_y,
            ),
        );
        StatusCode::OK
    }
}

/// Drives with field-relative speeds (a `ChassisSpeeds`). Forces are field-relative too, and
/// become robot-centric here.
#[derive(Clone, Debug)]
pub struct ApplyFieldSpeeds {
    pub speeds: ChassisSpeeds,
    pub wheel_force_feedforwards_x: Vec<f64>,
    pub wheel_force_feedforwards_y: Vec<f64>,
    pub center_of_rotation: Translation2d,
    pub drive_request_type: DriveRequestType,
    pub steer_request_type: SteerRequestType,
    pub desaturate_wheel_speeds: bool,
    pub forward_perspective: ForwardPerspective,
}

impl Default for ApplyFieldSpeeds {
    fn default() -> Self {
        Self {
            speeds: ChassisSpeeds::default(),
            wheel_force_feedforwards_x: Vec::new(),
            wheel_force_feedforwards_y: Vec::new(),
            center_of_rotation: Translation2d::ZERO,
            drive_request_type: DriveRequestType::OpenLoopVoltage,
            steer_request_type: SteerRequestType::Position,
            desaturate_wheel_speeds: true,
            forward_perspective: ForwardPerspective::BlueAlliance,
        }
    }
}

impl SwerveRequest for ApplyFieldSpeeds {
    fn apply(
        &mut self,
        parameters: &mut ControlParameters<'_>,
        modules: &[&dyn ModuleSink],
    ) -> StatusCode {
        let operator = self.forward_perspective == ForwardPerspective::OperatorPerspective;
        let mut field = self.speeds;
        if operator {
            (field.vx, field.vy) = rotate_xy(
                self.speeds.vx,
                self.speeds.vy,
                parameters.operator_forward_direction,
            );
        }
        let robot_speeds =
            ChassisSpeeds::from_field_relative(field, parameters.current_pose.rotation)
                .discretize(parameters.update_period);
        let mut states = parameters
            .kinematics
            .to_swerve_module_states(robot_speeds, self.center_of_rotation);
        desaturate(&mut states, self.desaturate_wheel_speeds, parameters);

        let forces: Vec<(f64, f64)> = paired(
            &self.wheel_force_feedforwards_x,
            &self.wheel_force_feedforwards_y,
        )
        .into_iter()
        .map(|(fx, fy)| {
            let (mut x, mut y) = (fx, fy);
            if operator {
                (x, y) = rotate_xy(x, y, parameters.operator_forward_direction);
            }
            rotate_xy(x, y, -parameters.current_pose.rotation)
        })
        .collect();
        apply_states(
            parameters,
            modules,
            &states,
            self.drive_request_type,
            self.steer_request_type,
            &forces,
        );
        StatusCode::OK
    }
}

/// Field-centric driving that holds a heading with a PID controller.
#[derive(Clone, Debug)]
pub struct FieldCentricFacingAngle {
    pub velocity_x: f64,
    pub velocity_y: f64,
    pub target_direction: Rotation2d,
    pub target_rate_feedforward: f64,
    pub deadband: f64,
    pub rotational_deadband: f64,
    /// Zero or less means no limit.
    pub max_abs_rotational_rate: f64,
    pub center_of_rotation: Translation2d,
    pub drive_request_type: DriveRequestType,
    pub steer_request_type: SteerRequestType,
    pub desaturate_wheel_speeds: bool,
    pub forward_perspective: ForwardPerspective,
    pub target_direction_perspective: TargetDirectionPerspective,
    pub heading_controller: PhoenixPidController,
}

impl Default for FieldCentricFacingAngle {
    fn default() -> Self {
        let mut heading_controller = PhoenixPidController::new(0.0, 0.0, 0.0);
        heading_controller.enable_continuous_input(-PI, PI);
        Self {
            velocity_x: 0.0,
            velocity_y: 0.0,
            target_direction: Rotation2d::ZERO,
            target_rate_feedforward: 0.0,
            deadband: 0.0,
            rotational_deadband: 0.0,
            max_abs_rotational_rate: 0.0,
            center_of_rotation: Translation2d::ZERO,
            drive_request_type: DriveRequestType::OpenLoopVoltage,
            steer_request_type: SteerRequestType::Position,
            desaturate_wheel_speeds: true,
            forward_perspective: ForwardPerspective::OperatorPerspective,
            target_direction_perspective: TargetDirectionPerspective::UseForwardPerspective,
            heading_controller,
        }
    }
}

impl FieldCentricFacingAngle {
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn with_heading_pid(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.heading_controller.set_pid(kp, ki, kd);
        self
    }
}

impl SwerveRequest for FieldCentricFacingAngle {
    fn apply(
        &mut self,
        parameters: &mut ControlParameters<'_>,
        modules: &[&dyn ModuleSink],
    ) -> StatusCode {
        let mut angle_to_face = self.target_direction;
        if self.target_direction_perspective == TargetDirectionPerspective::UseForwardPerspective
            && self.forward_perspective == ForwardPerspective::OperatorPerspective
        {
            angle_to_face = angle_to_face.rotate_by(parameters.operator_forward_direction);
        }
        let mut omega = self.target_rate_feedforward
            + self.heading_controller.calculate(
                parameters.current_pose.rotation.radians(),
                angle_to_face.radians(),
                parameters.timestamp,
            );
        if self.max_abs_rotational_rate > 0.0 {
            omega = omega.clamp(-self.max_abs_rotational_rate, self.max_abs_rotational_rate);
        }
        FieldCentric {
            velocity_x: self.velocity_x,
            velocity_y: self.velocity_y,
            rotational_rate: omega,
            deadband: self.deadband,
            rotational_deadband: self.rotational_deadband,
            center_of_rotation: self.center_of_rotation,
            drive_request_type: self.drive_request_type,
            steer_request_type: self.steer_request_type,
            desaturate_wheel_speeds: self.desaturate_wheel_speeds,
            forward_perspective: self.forward_perspective,
        }
        .apply(parameters, modules)
    }
}

fn steer_position(output: ClosedLoopOutputType, rot: f64) -> SteerCommand {
    match output {
        ClosedLoopOutputType::Voltage => SteerCommand::PositionVoltage {
            rot,
            velocity_rps: 0.0,
            enable_foc: true,
        },
        ClosedLoopOutputType::TorqueCurrentFoc => SteerCommand::PositionTorqueCurrent {
            rot,
            velocity_rps: 0.0,
        },
    }
}

/// Characterizes drive translation: the same voltage on every drive motor, wheels straight.
#[derive(Clone, Copy, Debug, Default)]
pub struct SysIdSwerveTranslation {
    pub volts_to_apply: f64,
}

impl SwerveRequest for SysIdSwerveTranslation {
    fn apply(&mut self, _: &mut ControlParameters<'_>, modules: &[&dyn ModuleSink]) -> StatusCode {
        for module in modules {
            module.apply_commands(
                DriveCommand::Voltage {
                    volts: self.volts_to_apply,
                    enable_foc: true,
                },
                steer_position(module.steer_closed_loop_output(), 0.0),
            );
        }
        StatusCode::OK
    }
}

/// Characterizes rotation: wheels tangent to the robot center, drive voltage from the rate.
#[derive(Clone, Copy, Debug, Default)]
pub struct SysIdSwerveRotation {
    /// rad/s
    pub rotational_rate: f64,
}

impl SwerveRequest for SysIdSwerveRotation {
    fn apply(
        &mut self,
        parameters: &mut ControlParameters<'_>,
        modules: &[&dyn ModuleSink],
    ) -> StatusCode {
        for (i, module) in modules.iter().enumerate() {
            let location = parameters.module_locations[i];
            let speed = self.rotational_rate * location.norm();
            let volts = speed / parameters.max_speed * 12.0;
            let angle = location.angle() + Rotation2d::CCW_PI_2;
            module.apply_commands(
                DriveCommand::Voltage {
                    volts,
                    enable_foc: true,
                },
                steer_position(module.steer_closed_loop_output(), angle.rotations()),
            );
        }
        StatusCode::OK
    }
}

/// Characterizes steer gains: drive coasts, steer gets the voltage.
#[derive(Clone, Copy, Debug, Default)]
pub struct SysIdSwerveSteerGains {
    pub volts_to_apply: f64,
}

impl SwerveRequest for SysIdSwerveSteerGains {
    fn apply(&mut self, _: &mut ControlParameters<'_>, modules: &[&dyn ModuleSink]) -> StatusCode {
        for module in modules {
            module.apply_commands(
                DriveCommand::Coast,
                SteerCommand::Voltage {
                    volts: self.volts_to_apply,
                    enable_foc: true,
                },
            );
        }
        StatusCode::OK
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use wpimath::geometry::Pose2d;

    #[derive(Default)]
    struct Recorder {
        requests: RefCell<Vec<ModuleRequest>>,
        commands: RefCell<Vec<(DriveCommand, SteerCommand)>>,
        output: Option<ClosedLoopOutputType>,
    }

    impl ModuleSink for Recorder {
        fn apply(&self, request: &ModuleRequest) {
            self.requests.borrow_mut().push(*request);
        }

        fn apply_commands(&self, drive: DriveCommand, steer: SteerCommand) {
            self.commands.borrow_mut().push((drive, steer));
        }

        fn steer_closed_loop_output(&self) -> ClosedLoopOutputType {
            self.output.unwrap_or(ClosedLoopOutputType::Voltage)
        }
    }

    const HALF: f64 = 0.276;

    fn locations() -> [Translation2d; 4] {
        [
            Translation2d::new(HALF, HALF),
            Translation2d::new(HALF, -HALF),
            Translation2d::new(-HALF, HALF),
            Translation2d::new(-HALF, -HALF),
        ]
    }

    struct Rig {
        kinematics: SwerveDriveKinematics<4>,
        modules: [Recorder; 4],
    }

    impl Rig {
        fn new() -> Self {
            Self {
                kinematics: SwerveDriveKinematics::new(locations()),
                modules: Default::default(),
            }
        }

        fn run(
            &mut self,
            request: &mut dyn SwerveRequest,
            pose: Pose2d,
            operator: Rotation2d,
        ) -> [ModuleRequest; 4] {
            let mut params = ControlParameters {
                kinematics: &mut self.kinematics,
                module_locations: locations(),
                max_speed: 5.0,
                operator_forward_direction: operator,
                current_chassis_speed: ChassisSpeeds::default(),
                current_pose: pose,
                timestamp: 10.0,
                update_period: 0.004,
            };
            let sinks: Vec<&dyn ModuleSink> = self.modules.iter().map(|m| m as _).collect();
            assert_eq!(request.apply(&mut params, &sinks), StatusCode::OK);
            std::array::from_fn(|i| {
                let last = *self.modules[i].requests.borrow().last().expect("request");
                self.modules[i].requests.borrow_mut().clear();
                last
            })
        }
    }

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn idle_touches_nothing() {
        let mut rig = Rig::new();
        let mut params = ControlParameters {
            kinematics: &mut rig.kinematics,
            module_locations: locations(),
            max_speed: 5.0,
            operator_forward_direction: Rotation2d::ZERO,
            current_chassis_speed: ChassisSpeeds::default(),
            current_pose: Pose2d::ZERO,
            timestamp: 0.0,
            update_period: 0.004,
        };
        let sinks: Vec<&dyn ModuleSink> = rig.modules.iter().map(|m| m as _).collect();
        Idle.apply(&mut params, &sinks);
        assert!(rig.modules.iter().all(|m| m.requests.borrow().is_empty()));
    }

    #[test]
    fn brake_points_wheels_at_the_center() {
        let mut rig = Rig::new();
        let out = rig.run(
            &mut SwerveDriveBrake::default(),
            Pose2d::ZERO,
            Rotation2d::ZERO,
        );
        for (i, request) in out.iter().enumerate() {
            assert_eq!(request.state.speed, 0.0);
            assert!(near(
                request.state.angle.radians(),
                locations()[i].angle().radians()
            ));
            assert_eq!(request.update_period, 0.004);
        }
    }

    #[test]
    fn robot_speeds_forward_gives_equal_forward_modules() {
        let mut rig = Rig::new();
        let mut request = ApplyRobotSpeeds {
            speeds: ChassisSpeeds::new(2.0, 0.0, 0.0),
            drive_request_type: DriveRequestType::Velocity,
            ..ApplyRobotSpeeds::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        for r in out {
            assert!(near(r.state.speed, 2.0));
            assert!(near(r.state.angle.radians(), 0.0));
            assert_eq!(r.drive_request, DriveRequestType::Velocity);
        }
    }

    #[test]
    fn robot_speeds_rotation_gives_tangent_modules() {
        let mut rig = Rig::new();
        let mut request = ApplyRobotSpeeds {
            speeds: ChassisSpeeds::new(0.0, 0.0, 2.0),
            ..ApplyRobotSpeeds::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        let radius = HALF.hypot(HALF);
        for (i, r) in out.iter().enumerate() {
            assert!(near(r.state.speed, 2.0 * radius));
            let tangent = locations()[i].angle() + Rotation2d::CCW_PI_2;
            assert!(near(r.state.angle.radians(), tangent.radians()));
        }
    }

    #[test]
    fn robot_speeds_desaturate_to_max_speed() {
        let mut rig = Rig::new();
        let mut request = ApplyRobotSpeeds {
            speeds: ChassisSpeeds::new(10.0, 0.0, 0.0),
            ..ApplyRobotSpeeds::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        assert!(out.iter().all(|r| near(r.state.speed, 5.0)));
        request.desaturate_wheel_speeds = false;
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        assert!(out.iter().all(|r| near(r.state.speed, 10.0)));
    }

    #[test]
    fn center_of_rotation_moves_the_pivot() {
        let mut rig = Rig::new();
        let mut request = ApplyRobotSpeeds {
            speeds: ChassisSpeeds::new(0.0, 0.0, 1.0),
            center_of_rotation: locations()[0],
            ..ApplyRobotSpeeds::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        // The module at the pivot barely moves.
        assert!(out[0].state.speed < 1e-9);
        assert!(out[3].state.speed > 0.5);
    }

    #[test]
    fn force_feedforwards_pair_up_and_short_lists_keep_the_last_value() {
        let mut rig = Rig::new();
        let mut request = ApplyRobotSpeeds {
            wheel_force_feedforwards_x: vec![1.0, 2.0],
            wheel_force_feedforwards_y: vec![10.0, 20.0, 30.0],
            ..ApplyRobotSpeeds::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        assert_eq!(out[0].wheel_force_feedforward_x, 1.0);
        assert_eq!(out[1].wheel_force_feedforward_y, 20.0);
        assert_eq!(out[3].wheel_force_feedforward_x, 2.0);
        assert_eq!(out[3].wheel_force_feedforward_y, 20.0);
    }

    #[test]
    fn field_speeds_follow_the_robot_heading() {
        let mut rig = Rig::new();
        let mut request = ApplyFieldSpeeds {
            speeds: ChassisSpeeds::new(1.0, 0.0, 0.0),
            ..ApplyFieldSpeeds::default()
        };
        // Robot faces +Y, field +X is to its right: modules point at -90 degrees.
        let out = rig.run(
            &mut request,
            Pose2d::from_xy(0.0, 0.0, Rotation2d::from_degrees(90.0)),
            Rotation2d::ZERO,
        );
        for r in out {
            assert!(near(r.state.speed, 1.0));
            assert!(near(r.state.angle.degrees(), -90.0));
        }
    }

    #[test]
    fn field_speeds_blue_perspective_ignores_the_operator_direction() {
        let mut rig = Rig::new();
        let mut request = ApplyFieldSpeeds {
            speeds: ChassisSpeeds::new(1.0, 0.0, 0.0),
            ..ApplyFieldSpeeds::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::PI);
        assert!(out.iter().all(|r| near(r.state.angle.radians(), 0.0)));

        request.forward_perspective = ForwardPerspective::OperatorPerspective;
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::PI);
        assert!(out.iter().all(|r| near(r.state.angle.radians().abs(), PI)));
    }

    #[test]
    fn field_speeds_force_feedforward_becomes_robot_centric() {
        let mut rig = Rig::new();
        let mut request = ApplyFieldSpeeds {
            wheel_force_feedforwards_x: vec![10.0; 4],
            wheel_force_feedforwards_y: vec![0.0; 4],
            ..ApplyFieldSpeeds::default()
        };
        let out = rig.run(
            &mut request,
            Pose2d::from_xy(0.0, 0.0, Rotation2d::from_degrees(90.0)),
            Rotation2d::ZERO,
        );
        for r in out {
            assert!(near(r.wheel_force_feedforward_x, 0.0));
            assert!(near(r.wheel_force_feedforward_y, -10.0));
        }
    }

    #[test]
    fn field_centric_applies_deadbands() {
        let mut rig = Rig::new();
        let mut request = FieldCentric {
            velocity_x: 0.03,
            velocity_y: 0.04,
            rotational_rate: 0.1,
            deadband: 0.06,
            rotational_deadband: 0.2,
            ..FieldCentric::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        assert!(out.iter().all(|r| r.state.speed == 0.0));
        request.deadband = 0.04;
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        assert!(out.iter().all(|r| r.state.speed > 0.04));
    }

    #[test]
    fn field_centric_operator_perspective_rotates_the_input() {
        let mut rig = Rig::new();
        let mut request = FieldCentric {
            velocity_x: 1.0,
            ..FieldCentric::default()
        };
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::PI);
        assert!(out.iter().all(|r| near(r.state.angle.radians().abs(), PI)));
    }

    #[test]
    fn facing_angle_turns_toward_the_target_and_clamps() {
        let mut rig = Rig::new();
        let mut request = FieldCentricFacingAngle::new().with_heading_pid(5.0, 0.0, 0.0);
        request.target_direction = Rotation2d::from_degrees(90.0);
        request.max_abs_rotational_rate = 3.0;
        let out = rig.run(&mut request, Pose2d::ZERO, Rotation2d::ZERO);
        // error pi/2 * 5 = 7.85, clamped to 3 rad/s: speed 3 * module radius
        let radius = HALF.hypot(HALF);
        assert!(out.iter().all(|r| near(r.state.speed, 3.0 * radius)));
        assert!(near(request.heading_controller.position_error(), PI / 2.0));
    }

    #[test]
    fn facing_angle_rotates_the_target_by_the_operator_direction() {
        let mut rig = Rig::new();
        let mut request = FieldCentricFacingAngle::new().with_heading_pid(1.0, 0.0, 0.0);
        request.target_direction = Rotation2d::ZERO;
        rig.run(&mut request, Pose2d::ZERO, Rotation2d::from_degrees(90.0));
        assert!(near(request.heading_controller.setpoint(), PI / 2.0));

        request.target_direction_perspective = TargetDirectionPerspective::BlueAlliance;
        rig.run(&mut request, Pose2d::ZERO, Rotation2d::from_degrees(90.0));
        assert!(near(request.heading_controller.setpoint(), 0.0));
    }

    #[test]
    fn sysid_translation_sends_the_same_voltage_with_straight_wheels() {
        let mut rig = Rig::new();
        let mut params = ControlParameters {
            kinematics: &mut rig.kinematics,
            module_locations: locations(),
            max_speed: 5.0,
            operator_forward_direction: Rotation2d::ZERO,
            current_chassis_speed: ChassisSpeeds::default(),
            current_pose: Pose2d::ZERO,
            timestamp: 0.0,
            update_period: 0.004,
        };
        let sinks: Vec<&dyn ModuleSink> = rig.modules.iter().map(|m| m as _).collect();
        SysIdSwerveTranslation {
            volts_to_apply: 3.0,
        }
        .apply(&mut params, &sinks);
        for m in &rig.modules {
            let (drive, steer) = m.commands.borrow()[0];
            assert_eq!(
                drive,
                DriveCommand::Voltage {
                    volts: 3.0,
                    enable_foc: true
                }
            );
            assert_eq!(
                steer,
                SteerCommand::PositionVoltage {
                    rot: 0.0,
                    velocity_rps: 0.0,
                    enable_foc: true
                }
            );
        }
    }

    #[test]
    fn sysid_rotation_uses_torque_steer_when_configured() {
        let mut rig = Rig::new();
        for m in &mut rig.modules {
            m.output = Some(ClosedLoopOutputType::TorqueCurrentFoc);
        }
        let mut params = ControlParameters {
            kinematics: &mut rig.kinematics,
            module_locations: locations(),
            max_speed: 5.0,
            operator_forward_direction: Rotation2d::ZERO,
            current_chassis_speed: ChassisSpeeds::default(),
            current_pose: Pose2d::ZERO,
            timestamp: 0.0,
            update_period: 0.004,
        };
        let sinks: Vec<&dyn ModuleSink> = rig.modules.iter().map(|m| m as _).collect();
        SysIdSwerveRotation {
            rotational_rate: 2.0,
        }
        .apply(&mut params, &sinks);
        let radius = HALF.hypot(HALF);
        let (drive, steer) = rig.modules[0].commands.borrow()[0];
        let DriveCommand::Voltage { volts, .. } = drive else {
            panic!("wrong drive command");
        };
        assert!(near(volts, 2.0 * radius / 5.0 * 12.0));
        let SteerCommand::PositionTorqueCurrent { rot, .. } = steer else {
            panic!("wrong steer command");
        };
        // module at +45 degrees, wheel tangent at 135
        assert!(near(rot, 135.0 / 360.0));
    }

    #[test]
    fn sysid_steer_gains_coasts_the_drive() {
        let mut rig = Rig::new();
        let mut params = ControlParameters {
            kinematics: &mut rig.kinematics,
            module_locations: locations(),
            max_speed: 5.0,
            operator_forward_direction: Rotation2d::ZERO,
            current_chassis_speed: ChassisSpeeds::default(),
            current_pose: Pose2d::ZERO,
            timestamp: 0.0,
            update_period: 0.004,
        };
        let sinks: Vec<&dyn ModuleSink> = rig.modules.iter().map(|m| m as _).collect();
        SysIdSwerveSteerGains {
            volts_to_apply: 7.0,
        }
        .apply(&mut params, &sinks);
        let (drive, steer) = rig.modules[2].commands.borrow()[0];
        assert_eq!(drive, DriveCommand::Coast);
        assert_eq!(
            steer,
            SteerCommand::Voltage {
                volts: 7.0,
                enable_foc: true
            }
        );
    }
}
