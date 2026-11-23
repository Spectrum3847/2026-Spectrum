//! Phoenix swerve drivetrain wired up as a scheduler subsystem.
//!
//! Java's `Swerve extends SwerveDrivetrain`. Here `Swerve` derefs to the
//! [`swerve::SwerveDrivetrain`] it holds, so the inherited calls (`get_state`,
//! `add_vision_measurement`, `get_kinematics` and the rest) read the same. `reset_pose` is the
//! one Java override and shadows the drivetrain's.
//!
//! Sim seam (T18): Java's `MapleSimSwerveDrivetrain` and `RobotBumpSim` are reached through
//! [`SwerveSimBackend`]. [`regulate_module_constants_for_simulation`] is the matching pass
//! over the module constants.

use super::swerve_config::{NUM_MODULES, SwerveConfig};
use crate::context;
use crate::rebuilt::robot_bump_sim::RobotBumpSim;
use crate::rebuilt::{field, field_helpers};
use crate::spectrum_lib::swerve::maple_sim_swerve_drivetrain::{
    MapleSimBackend, MapleSimSwerveDrivetrain,
};
use crate::spectrum_lib::util::util;
use commands::{Command, Subsystem, SubsystemBase, SubsystemId, Trigger};
use pathplanner::{AutoBuilder, PidConstants, PpHolonomicDriveController, RobotConfig};
use phoenix6::configs::Slot0Configs;
use phoenix6::signals::StaticFeedforwardSignValue;
use phoenix6::status_signal::set_update_frequency_for_all;
use phoenix6::{CanBus, utils};
use std::cell::{Cell, RefCell};
use std::ops::Deref;
use std::rc::{Rc, Weak};
use std::sync::{Arc, Mutex, PoisonError};
use swerve::{
    ApplyFieldSpeeds, ApplyRobotSpeeds, DriveRequestType, DrivetrainOptions,
    FieldCentricFacingAngle, Idle, ModuleConstants, PhoenixPidController, SteerRequestType,
    SwerveDriveBrake, SwerveDrivetrain, SwerveRequest,
};
use telemetry::{EnumName, telemetry};
use wpilib::dashboard::{Alert, AlertType};
use wpilib::driver_station::{self, Alliance};
use wpilib::timer;
use wpimath::geometry::{Pose2d, Pose3d, Rectangle2d, Rotation2d, Translation2d};
use wpimath::kinematics::ChassisSpeeds;
use wpimath::system::DcMotor;
use wpimath::units::{degrees_to_radians, feet_to_meters, inches_to_meters, lbs_to_kilograms};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WantedState {
    TeleopDrive,
    PilotAimAtTarget,
    XBrake,
    Idle,
}

impl EnumName for WantedState {
    fn name(&self) -> &'static str {
        match self {
            Self::TeleopDrive => "TELEOP_DRIVE",
            Self::PilotAimAtTarget => "PILOT_AIM_AT_TARGET",
            Self::XBrake => "X_BRAKE",
            Self::Idle => "IDLE",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    TeleopDrive,
    PilotAimAtTarget,
    XBrake,
    Idle,
}

impl EnumName for SystemState {
    fn name(&self) -> &'static str {
        match self {
            Self::TeleopDrive => "TELEOP_DRIVE",
            Self::PilotAimAtTarget => "PILOT_AIM_AT_TARGET",
            Self::XBrake => "X_BRAKE",
            Self::Idle => "IDLE",
        }
    }
}

pub const TRANSLATION_ERROR_MARGIN_METERS: f64 = 0.0254; // Units.inchesToMeters(1.0)
pub const DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT: f64 = 0.02;
const SKEW_COMPENSATION_SCALAR: f64 = -0.03;

/// The pilot's drive stick values. Java's `Robot.getPilot()`; the T17 `Pilot` implements it.
pub trait PilotDriveInputs {
    fn get_drive_fwd_positive(&self) -> f64;
    fn get_drive_left_positive(&self) -> f64;
    fn get_drive_ccw_positive(&self) -> f64;
}

/// The simulated drivetrain behind `Swerve` in sim (Java's `MapleSimSwerveDrivetrain` and
/// `RobotBumpSim`). Implementors keep their own locking, because the sim thread steps them.
pub trait SwerveSimBackend {
    /// `mapleSimDrive.getSimulatedDriveTrainPose()`.
    fn simulated_pose(&self) -> Pose2d;

    /// `mapleSimDrive.setSimulationWorldPose(pose)`.
    fn set_simulation_world_pose(&self, pose: Pose2d);

    /// The bump sim block of `Swerve.periodic`: steps `RobotBumpSim`, lifts the sim pose when
    /// on a ramp and returns the 3D pose to log. `None` when there is no bump sim.
    fn update_bump_sim(&self) -> Option<Pose3d>;
}

/// Simulation-only changes to the module constants. Java's
/// `MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation`, which leaves a real robot
/// alone. Inverted drive configs upset the drive PID and a non-zero CANcoder offset upsets
/// module state optimization in the sim.
pub fn regulate_module_constants_for_simulation(
    mut modules: [ModuleConstants; NUM_MODULES],
) -> [ModuleConstants; NUM_MODULES] {
    if wpilib::is_real() {
        return modules;
    }
    for module in &mut modules {
        regulate_module_constant_for_simulation(module);
    }
    modules
}

fn regulate_module_constant_for_simulation(module: &mut ModuleConstants) {
    module.encoder_offset_rot = 0.0;
    module.drive_motor_inverted = false;
    module.steer_motor_inverted = false;
    module.encoder_inverted = false;
    module.steer_motor_gains = Slot0Configs::default()
        .with_k_p(1000.0)
        .with_k_i(0.0)
        .with_k_d(60.0)
        .with_k_s(0.15)
        .with_k_v(1.5)
        .with_k_a(0.0)
        .with_static_feedforward_sign(StaticFeedforwardSignValue::UseClosedLoopSign);
    module.steer_motor_gear_ratio = 21.428571428571427;
    module.drive_friction_voltage = 0.1;
    module.steer_friction_voltage = 0.05;
    module.steer_inertia = 0.05;
}

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

const NEUTRAL_DEPTH_METERS: f64 = 283.0 * 0.0254;
const NEUTRAL_LENGTH_METERS: f64 = 317.7 * 0.0254;
const ENEMY_ALLIANCE_DEPTH_METERS: f64 = 180.0 * 0.0254;

thread_local! {
    static NEUTRAL_ZONE: Rectangle2d = Rectangle2d::from_corners(
        Translation2d::new(
            field::FIELD_LENGTH / 2.0 - NEUTRAL_DEPTH_METERS / 2.0,
            field::FIELD_WIDTH / 2.0 - NEUTRAL_LENGTH_METERS / 2.0,
        ),
        Translation2d::new(
            field::FIELD_LENGTH / 2.0 + NEUTRAL_DEPTH_METERS / 2.0,
            field::FIELD_WIDTH / 2.0 + NEUTRAL_LENGTH_METERS / 2.0,
        ),
    );

    static ENEMY_ALLIANCE_ZONE: Rectangle2d = Rectangle2d::from_corners(
        Translation2d::new(field::FIELD_LENGTH - ENEMY_ALLIANCE_DEPTH_METERS, 0.0),
        Translation2d::new(field::FIELD_LENGTH, field::FIELD_WIDTH),
    );
}

pub fn handle_state_transition(wanted_state: WantedState) -> SystemState {
    match wanted_state {
        WantedState::TeleopDrive => SystemState::TeleopDrive,
        WantedState::PilotAimAtTarget => SystemState::PilotAimAtTarget,
        WantedState::XBrake => SystemState::XBrake,
        WantedState::Idle => SystemState::Idle,
    }
}

/// Field-relative stick speeds turned robot-relative, with skew compensation. `alliance` of
/// `None` gives zero speeds. `measured_omega` is the measured spin, rad/s.
#[allow(clippy::too_many_arguments)]
pub fn joystick_speeds(
    alliance: Option<Alliance>,
    x_magnitude: f64,
    y_magnitude: f64,
    angular_magnitude: f64,
    teleop_velocity_coefficient: f64,
    rotation_velocity_coefficient: f64,
    measured_omega: f64,
    robot_rotation: Rotation2d,
) -> ChassisSpeeds {
    let Some(alliance) = alliance else {
        return ChassisSpeeds::new(0.0, 0.0, 0.0);
    };
    let blue = alliance == Alliance::Blue;

    let x_velocity = (if blue { x_magnitude } else { -x_magnitude }) * teleop_velocity_coefficient;
    let y_velocity = (if blue { y_magnitude } else { -y_magnitude }) * teleop_velocity_coefficient;
    let angular_velocity = angular_magnitude * rotation_velocity_coefficient;

    let skew_compensation_factor = Rotation2d::new(measured_omega * SKEW_COMPENSATION_SCALAR);

    // Java: the result is field-relative, and the inner conversion only turns the sticks by the
    // skew angle.
    ChassisSpeeds::from_robot_relative(
        ChassisSpeeds::from_field_relative(
            ChassisSpeeds::new(x_velocity, y_velocity, angular_velocity),
            robot_rotation,
        ),
        robot_rotation + skew_compensation_factor,
    )
}

/// Nearest of 0, 90, 180, 270 degrees to a heading in radians.
pub fn closest_cardinal_degrees(heading_radians: f64) -> f64 {
    use std::f64::consts::PI;
    let heading = heading_radians;
    if heading > -PI / 4.0 && heading <= PI / 4.0 {
        0.0
    } else if heading > PI / 4.0 && heading <= 3.0 * PI / 4.0 {
        90.0
    } else if heading > 3.0 * PI / 4.0 || heading <= -3.0 * PI / 4.0 {
        180.0
    } else {
        270.0
    }
}

/// Whether the robot's front is nearer `angle_degrees` than its back is.
pub fn front_closest_to_angle_for(heading_degrees: f64, angle_degrees: f64) -> bool {
    let flipped_heading = if heading_degrees > 0.0 {
        heading_degrees - 180.0
    } else {
        heading_degrees + 180.0
    };
    let front_difference = get_rotation_difference(heading_degrees, angle_degrees);
    let flipped_difference = get_rotation_difference(flipped_heading, angle_degrees);

    front_difference < flipped_difference
}

pub fn get_rotation_difference(angle1: f64, angle2: f64) -> f64 {
    let diff = (angle1 - angle2).abs() % 360.0;
    if diff > 180.0 { 360.0 - diff } else { diff }
}

pub fn neutral_zone_contains(pose: Pose2d) -> bool {
    NEUTRAL_ZONE.with(|zone| zone.contains(pose.translation))
}

/// `red` flips the pose X so the same rectangle works for both alliances.
pub fn enemy_alliance_zone_contains(red: bool, pose: Pose2d) -> bool {
    ENEMY_ALLIANCE_ZONE.with(|zone| {
        zone.contains(Translation2d::new(
            field_helpers::flip_x_if_red_for(red, pose.x()),
            pose.y(),
        ))
    })
}

/// Java: the midline is 27 ft over two, not the real half width of the field (317.7 in), so it
/// sits 0.1 m high.
fn field_midline_y() -> f64 {
    feet_to_meters(27.0) / 2.0
}

pub fn is_in_field_right(pose: Pose2d) -> bool {
    pose.y() < field_midline_y()
}

pub fn is_in_field_left(pose: Pose2d) -> bool {
    pose.y() >= field_midline_y()
}

pub fn going_too_fast_for(speeds: ChassisSpeeds, threshold_speed: f64) -> bool {
    let linear_speed = speeds.vx.hypot(speeds.vy);
    linear_speed > threshold_speed
}

pub struct Swerve {
    me: Weak<Swerve>,
    base: SubsystemBase,
    drivetrain: SwerveDrivetrain,

    wanted_state: Cell<WantedState>,
    system_state: Cell<SystemState>,

    teleop_velocity_coefficient: Cell<f64>,
    rotation_velocity_coefficient: Cell<f64>,

    pub config: SwerveConfig,

    pigeon_alert: Alert,

    auto_request: Arc<Mutex<ApplyRobotSpeeds>>,
    field_centric_drive: Arc<Mutex<ApplyFieldSpeeds>>,
    drive_at_angle_request: Arc<Mutex<FieldCentricFacingAngle>>,
    x_brake: Arc<Mutex<SwerveDriveBrake>>,

    sim: RefCell<Option<Rc<dyn SwerveSimBackend>>>,
    sim_robot_pose3d: Cell<Pose3d>,
}

impl Deref for Swerve {
    type Target = SwerveDrivetrain;

    fn deref(&self) -> &SwerveDrivetrain {
        &self.drivetrain
    }
}

impl Swerve {
    /// Builds the drivetrain and registers with the scheduler. Panics when a device ID is
    /// invalid, where Java throws.
    pub fn new(config: SwerveConfig) -> Rc<Swerve> {
        // Java: building the CANBus with a hoot path loads that file for replay.
        if let Err(code) = CanBus::with_hoot_file(&config.can_bus_name, &config.hoot_file_path) {
            driver_station::report_warning(&format!(
                "Could not load hoot file {}: {code}",
                config.hoot_file_path
            ));
        }

        let options = DrivetrainOptions {
            odometry_update_frequency_hz: 250.0,
            ..DrivetrainOptions::default()
        };
        let modules = regulate_module_constants_for_simulation(config.get_modules());
        let drivetrain = SwerveDrivetrain::new(&config.drivetrain_constants, &options, &modules)
            .unwrap_or_else(|code| panic!("Could not create the swerve drivetrain: {code}"));

        let auto_request = ApplyRobotSpeeds {
            drive_request_type: DriveRequestType::Velocity,
            steer_request_type: SteerRequestType::Position,
            desaturate_wheel_speeds: true,
            ..ApplyRobotSpeeds::default()
        };
        let field_centric_drive = ApplyFieldSpeeds {
            drive_request_type: DriveRequestType::Velocity,
            steer_request_type: SteerRequestType::Position,
            ..ApplyFieldSpeeds::default()
        };
        let mut drive_at_angle_request = FieldCentricFacingAngle {
            drive_request_type: DriveRequestType::Velocity,
            steer_request_type: SteerRequestType::Position,
            ..FieldCentricFacingAngle::default()
        };
        drive_at_angle_request.heading_controller = PhoenixPidController::new(
            config.k_p_rotation_controller,
            config.k_i_rotation_controller,
            config.k_d_rotation_controller,
        );
        drive_at_angle_request
            .heading_controller
            .enable_continuous_input(-std::f64::consts::PI, std::f64::consts::PI);
        let angular_speed_at_12_volts = degrees_to_radians(config.angular_speed_at_12_volts_dps);
        drive_at_angle_request.deadband = config.linear_speed_at_12_volts_mps * config.aim_deadband;
        drive_at_angle_request.rotational_deadband =
            angular_speed_at_12_volts * config.aim_deadband;
        drive_at_angle_request.max_abs_rotational_rate = angular_speed_at_12_volts;

        let swerve = Rc::new_cyclic(|me| Swerve {
            me: me.clone(),
            base: SubsystemBase::new("Swerve"),
            drivetrain,
            wanted_state: Cell::new(WantedState::Idle),
            system_state: Cell::new(SystemState::Idle),
            teleop_velocity_coefficient: Cell::new(1.0),
            rotation_velocity_coefficient: Cell::new(1.0),
            config,
            pigeon_alert: Alert::new("Pigeon IMU Disconnected", AlertType::Error),
            auto_request: Arc::new(Mutex::new(auto_request)),
            field_centric_drive: Arc::new(Mutex::new(field_centric_drive)),
            drive_at_angle_request: Arc::new(Mutex::new(drive_at_angle_request)),
            x_brake: Arc::new(Mutex::new(SwerveDriveBrake::default())),
            sim: RefCell::new(None),
            sim_robot_pose3d: Cell::new(Pose3d::ZERO),
        });

        if utils::is_simulation() {
            swerve.start_sim_thread();
        }

        swerve.configure_path_planner();

        commands::CommandScheduler::get_instance().register_subsystem(swerve.clone());

        // Java: the signals sit in fields that nothing reads after this.
        let pigeon = swerve.drivetrain.get_pigeon2();
        let angular_pitch_velocity = pigeon.angular_velocity_y_world(false);
        let angular_roll_velocity = pigeon.angular_velocity_x_world(false);
        let angular_yaw_velocity = pigeon.angular_velocity_z_world(false);
        let roll = pigeon.roll(false);
        let pitch = pigeon.pitch(false);
        let acceleration_x = pigeon.acceleration_x(false);
        let acceleration_y = pigeon.acceleration_y(false);

        set_update_frequency_for_all(250.0, &[angular_yaw_velocity]);
        set_update_frequency_for_all(
            100.0,
            &[
                angular_pitch_velocity,
                angular_roll_velocity,
                roll,
                pitch,
                acceleration_x,
                acceleration_y,
            ],
        );

        pigeon.optimize_bus_utilization();
        swerve.drivetrain.optimize_bus_utilization();

        swerve.drivetrain.register_telemetry(log);

        telemetry::print(&format!("{} Subsystem Initialized", swerve.name()));
        swerve
    }

    fn rc(&self) -> Rc<Swerve> {
        self.me
            .upgrade()
            .expect("Swerve is only built through Swerve::new, which returns an Rc")
    }

    fn sim_backend(&self) -> Option<Rc<dyn SwerveSimBackend>> {
        self.sim.borrow().clone()
    }

    /// Hands over the simulated drivetrain. Java builds it in `startSimThread`.
    pub fn set_sim_backend(&self, backend: Rc<dyn SwerveSimBackend>) {
        *self.sim.borrow_mut() = Some(backend);
    }

    pub fn get_sim_robot_pose3d(&self) -> Pose3d {
        self.sim_robot_pose3d.get()
    }

    pub fn get_teleop_velocity_coefficient(&self) -> f64 {
        self.teleop_velocity_coefficient.get()
    }

    pub fn set_teleop_velocity_coefficient(&self, value: f64) {
        self.teleop_velocity_coefficient.set(value);
    }

    pub fn get_rotation_velocity_coefficient(&self) -> f64 {
        self.rotation_velocity_coefficient.get()
    }

    pub fn set_rotation_velocity_coefficient(&self, value: f64) {
        self.rotation_velocity_coefficient.set(value);
    }

    pub fn get_config(&self) -> &SwerveConfig {
        &self.config
    }

    pub fn get_wanted_state(&self) -> WantedState {
        self.wanted_state.get()
    }

    pub fn get_system_state(&self) -> SystemState {
        self.system_state.get()
    }

    /// Hands a request to the odometry thread. Edit the request through its mutex.
    pub fn set_control<R: SwerveRequest + 'static>(&self, request: &Arc<Mutex<R>>) {
        self.drivetrain.set_control(request);
    }

    fn log_battery_usage(&self) {
        let steer_motor_current = self.get_steer_motor_supply_currents();
        let drive_motor_current = self.get_drive_motor_supply_currents();
        let battery_logger = context::battery_logger();
        battery_logger.report_current_usage("Mechanisms/SwerveSteer", &[steer_motor_current]);
        battery_logger.report_current_usage("Mechanisms/SwerveDrive", &[drive_motor_current]);

        telemetry::log(
            "Swerve/Currents/DriveStatorCurrent",
            self.get_drive_motor_stator_currents(),
        );
        telemetry::log(
            "Swerve/Currents/SteerStatorCurrent",
            self.get_steer_motor_stator_currents(),
        );
        telemetry::log(
            "Swerve/Currents/DriveSupplyCurrent",
            self.get_drive_motor_supply_currents(),
        );
        telemetry::log(
            "Swerve/Currents/SteerSupplyCurrent",
            self.get_steer_motor_supply_currents(),
        );
    }

    fn get_drive_motor_stator_currents(&self) -> f64 {
        self.drivetrain
            .get_modules()
            .iter()
            .map(|module| module.drive_motor().stator_current(true).value())
            .sum()
    }

    fn get_steer_motor_stator_currents(&self) -> f64 {
        self.drivetrain
            .get_modules()
            .iter()
            .map(|module| module.steer_motor().stator_current(true).value())
            .sum()
    }

    fn get_drive_motor_supply_currents(&self) -> f64 {
        self.drivetrain
            .get_modules()
            .iter()
            .map(|module| module.drive_motor().supply_current(true).value())
            .sum()
    }

    fn get_steer_motor_supply_currents(&self) -> f64 {
        self.drivetrain
            .get_modules()
            .iter()
            .map(|module| module.steer_motor().supply_current(true).value())
            .sum()
    }

    fn get_current_command_name(&self) -> String {
        match self.get_current_command() {
            Some(command) => command.get_name(),
            None => "none".to_owned(),
        }
    }

    fn apply_states(&self) {
        match self.system_state.get() {
            SystemState::Idle => self.drivetrain.set_control_once(Idle),
            SystemState::PilotAimAtTarget => {
                let params = context::shot_calculator().get_parameters();
                let joystick_speeds = self.calculate_speeds_based_on_joystick_inputs();
                {
                    let mut request = lock(&self.drive_at_angle_request);
                    request.velocity_x = joystick_speeds.vx;
                    request.velocity_y = joystick_speeds.vy;
                    request.target_direction = params.drive_angle;
                }
                self.drivetrain.set_control(&self.drive_at_angle_request);
            }
            SystemState::TeleopDrive => {
                let speeds = self.calculate_speeds_based_on_joystick_inputs();
                lock(&self.field_centric_drive).speeds = speeds;
                self.drivetrain.set_control(&self.field_centric_drive);
            }
            SystemState::XBrake => self.drivetrain.set_control(&self.x_brake),
        }
    }

    fn calculate_speeds_based_on_joystick_inputs(&self) -> ChassisSpeeds {
        if driver_station::alliance().is_none() {
            return ChassisSpeeds::new(0.0, 0.0, 0.0);
        }

        let pilot = context::pilot();
        let x_magnitude = pilot.get_drive_fwd_positive();
        let y_magnitude = pilot.get_drive_left_positive();
        let angular_magnitude = pilot.get_drive_ccw_positive();

        joystick_speeds(
            driver_station::alliance(),
            x_magnitude,
            y_magnitude,
            angular_magnitude,
            self.teleop_velocity_coefficient.get(),
            self.rotation_velocity_coefficient.get(),
            self.get_current_robot_chassis_speeds().omega,
            self.get_robot_pose().rotation,
        )
    }

    /// Returns the simulated pose in sim, otherwise the pose odometry reports.
    pub fn get_robot_pose(&self) -> Pose2d {
        if let Some(sim) = self.sim_backend() {
            return sim.simulated_pose();
        }
        self.drivetrain.get_state().pose
    }

    /// Raises the alert that surfaces an unconnected Pigeon IMU in Elastic.
    fn check_pigeon_connection(&self) {
        self.pigeon_alert
            .set(!self.drivetrain.get_pigeon2().is_connected());
    }

    /// Samples the pose buffer at a RoboRIO timestamp, as in `fpga_to_current_time`. Gives the
    /// current pose when the timestamp is not buffered.
    pub fn get_pose_at_timestamp(&self, timestamp_seconds: f64) -> Pose2d {
        self.drivetrain
            .sample_pose_at(utils::fpga_to_current_time(timestamp_seconds))
            .unwrap_or_else(|| self.get_robot_pose())
    }

    pub fn reset_pose(&self, pose: Pose2d) {
        if let Some(sim) = self.sim_backend() {
            sim.set_simulation_world_pose(pose);
            timer::delay(0.05); // let the sim apply the new pose first
        }
        self.drivetrain.reset_pose(pose);
    }

    pub fn in_x_zone(&self, min_x_meter: f64, max_x_meter: f64) -> Trigger {
        let me = self.rc();
        Trigger::new(move || {
            util::in_range_suppliers(|| me.get_robot_pose().x(), || min_x_meter, || max_x_meter)
        })
    }

    pub fn in_y_zone(&self, min_y_meter: f64, max_y_meter: f64) -> Trigger {
        let me = self.rc();
        Trigger::new(move || {
            util::in_range_suppliers(|| me.get_robot_pose().y(), || min_y_meter, || max_y_meter)
        })
    }

    // The Alliance variants take bounds in the blue alliance frame and flip the pose instead of
    // the bounds, so callers never have to think about which alliance they are on.
    pub fn in_x_zone_alliance(&self, min_x_meter: f64, max_x_meter: f64) -> Trigger {
        let me = self.rc();
        Trigger::new(move || {
            util::in_range_min_max(
                field_helpers::flip_x_if_red(me.get_robot_pose().x()),
                min_x_meter,
                max_x_meter,
            )
        })
    }

    pub fn in_y_zone_alliance(&self, min_y_meter: f64, max_y_meter: f64) -> Trigger {
        let me = self.rc();
        Trigger::new(move || {
            util::in_range_min_max(
                field_helpers::flip_y_if_red(me.get_robot_pose().y()),
                min_y_meter,
                max_y_meter,
            )
        })
    }

    /// Returns true when the robot is inside the neutral zone. Allocation-free.
    pub fn is_in_neutral_zone(&self) -> bool {
        neutral_zone_contains(self.get_robot_pose())
    }

    /// Returns true when the robot is inside the opposing alliance's zone (pose X is flipped for
    /// red so the same rectangle works for both alliances).
    pub fn is_in_enemy_alliance_zone(&self) -> bool {
        enemy_alliance_zone_contains(field::is_red(), self.get_robot_pose())
    }

    pub fn in_neutral_zone(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_in_neutral_zone())
    }

    pub fn in_enemy_alliance_zone(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_in_enemy_alliance_zone())
    }

    pub fn in_field_right(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || is_in_field_right(me.get_robot_pose()))
    }

    pub fn in_field_left(&self) -> Trigger {
        let me = self.rc();
        Trigger::new(move || is_in_field_left(me.get_robot_pose()))
    }

    pub fn is_going_too_fast(&self, threshold_speed: f64) -> bool {
        going_too_fast_for(self.get_current_robot_chassis_speeds(), threshold_speed)
    }

    pub fn over_speed_trigger(&self, threshold_speed: f64) -> Trigger {
        let me = self.rc();
        Trigger::new(move || me.is_going_too_fast(threshold_speed))
    }

    pub fn get_current_robot_chassis_speeds(&self) -> ChassisSpeeds {
        self.drivetrain
            .get_kinematics()
            .to_chassis_speeds(&self.drivetrain.get_state().module_states)
    }

    fn apply_reorient(&self, angle_degrees: f64) {
        self.reset_pose(Pose2d::from_xy(
            self.get_robot_pose().x(),
            self.get_robot_pose().y(),
            Rotation2d::from_degrees(angle_degrees),
        ));
    }

    pub fn reorient(&self, angle_degrees: f64) -> Command {
        let me = self.rc();
        self.run_once(move || me.apply_reorient(angle_degrees))
    }

    pub fn reorient_forward(&self) -> Command {
        self.reorient(0.0).with_name("reorientForward")
    }

    pub fn reorient_left(&self) -> Command {
        self.reorient(90.0).with_name("reorientLeft")
    }

    pub fn reorient_back(&self) -> Command {
        self.reorient(180.0).with_name("reorientBack")
    }

    pub fn reorient_right(&self) -> Command {
        self.reorient(270.0).with_name("reorientRight")
    }

    pub fn get_closest_cardinal(&self) -> f64 {
        closest_cardinal_degrees(self.get_rotation().radians())
    }

    pub fn cardinal_reorient(&self) -> Command {
        let me = self.rc();
        self.run_once(move || {
            let angle_degrees = me.get_closest_cardinal();
            me.apply_reorient(angle_degrees);
        })
    }

    pub fn front_closest_to_angle(&self, angle_degrees: f64) -> bool {
        front_closest_to_angle_for(self.get_rotation().degrees(), angle_degrees)
    }

    pub fn get_rotation_difference(&self, angle1: f64, angle2: f64) -> f64 {
        get_rotation_difference(angle1, angle2)
    }

    pub fn get_rotation(&self) -> Rotation2d {
        self.get_robot_pose().rotation
    }

    pub fn get_rotation_radians(&self) -> f64 {
        self.get_robot_pose().rotation.radians()
    }

    // Runs the request every cycle, even while disabled, so aiming commands stay continuous.
    pub fn apply_request<R: SwerveRequest + 'static>(
        &self,
        request_supplier: impl Fn() -> Arc<Mutex<R>> + 'static,
    ) -> Command {
        let me = self.rc();
        self.run(move || me.set_control(&request_supplier()))
            .ignoring_disable(true)
    }

    pub fn set_wanted_state(&self, state: WantedState) {
        self.wanted_state.set(state);
    }

    pub fn is_at_desired_rotation(&self) -> bool {
        self.is_at_desired_rotation_within(degrees_to_radians(10.0))
    }

    pub fn is_at_desired_rotation_within(&self, tolerance_radians: f64) -> bool {
        if self.system_state.get() != SystemState::PilotAimAtTarget {
            return false;
        }
        lock(&self.drive_at_angle_request)
            .heading_controller
            .position_error()
            .abs()
            < tolerance_radians
    }

    fn configure_path_planner(&self) {
        // Paths reseed this, so the pre-match pose only has to be plausible
        self.reset_pose(Pose2d::from_xy(
            field::BLUE_HUB_CENTER.x - 2.0,
            field::BLUE_HUB_CENTER.y,
            Rotation2d::from_degrees(0.0),
        ));

        let robot_config = match RobotConfig::from_gui_settings() {
            Ok(config) => config,
            Err(error) => {
                driver_station::report_error(&format!(
                    "Failed to load PathPlanner config and configure AutoBuilder: {error}"
                ));
                return;
            }
        };

        let (pose_swerve, reset_swerve, speeds_swerve, output_swerve, requirement) =
            (self.rc(), self.rc(), self.rc(), self.rc(), self.id());
        AutoBuilder::configure(
            Rc::new(move || pose_swerve.get_robot_pose()),
            Rc::new(move |pose| reset_swerve.reset_pose(pose)),
            Rc::new(move || speeds_swerve.get_current_robot_chassis_speeds()),
            Rc::new(move |speeds, feedforwards| {
                {
                    let mut request = lock(&output_swerve.auto_request);
                    request.speeds = speeds.discretize(0.020);
                    request.wheel_force_feedforwards_x = feedforwards
                        .robot_relative_forces_x_newtons
                        .as_slice()
                        .to_vec();
                    request.wheel_force_feedforwards_y = feedforwards
                        .robot_relative_forces_y_newtons
                        .as_slice()
                        .to_vec();
                }
                output_swerve
                    .drivetrain
                    .set_control(&output_swerve.auto_request);
            }),
            Rc::new(RefCell::new(PpHolonomicDriveController::new(
                PidConstants::new(4.0, 0.0, 0.0), // translation
                PidConstants::new(3.0, 0.0, 0.0), // rotation
            ))),
            robot_config,
            // Paths are authored against blue, so PathPlanner mirrors them for red
            Rc::new(|| driver_station::alliance().unwrap_or(Alliance::Blue) == Alliance::Red),
            &[requirement],
        );
    }

    /// Java's `startSimThread`: builds `MapleSimSwerveDrivetrain` and `RobotBumpSim` and starts
    /// the sim notifier.
    fn start_sim_thread(&self) {
        // Java: the sim reads the regulated constants through the shared config objects
        let modules = regulate_module_constants_for_simulation(self.config.get_modules());
        let drivetrain = MapleSimSwerveDrivetrain::new(
            self.config.sim_loop_period,
            lbs_to_kilograms(115.0),
            inches_to_meters(30.0),
            inches_to_meters(30.0),
            DcMotor::kraken_x60_foc(1),
            DcMotor::kraken_x60_foc(1),
            1.2, // wheel COF
            self.get_module_locations(),
            self.get_pigeon2(),
            self.get_modules(),
            &modules,
        );
        let backend =
            MapleSimBackend::new(drivetrain, RobotBumpSim::new(*self.get_module_locations()));

        // Run faster than real time so PID gains behave more reasonably
        backend.start_notifier(self.config.sim_loop_period);
        self.set_sim_backend(Rc::new(backend));
    }
}

fn log(state: &swerve::SwerveDriveState) {
    telemetry::log_struct("Swerve/State/Pose", &state.pose);
    telemetry::log_struct_array("Swerve/State/TargetStates", &state.module_targets);
    telemetry::log_struct_array("Swerve/State/MeasuredStates", &state.module_states);
    telemetry::log_struct("Swerve/State/MeasuredSpeeds", &state.speeds);
}

impl Subsystem for Swerve {
    fn id(&self) -> SubsystemId {
        self.base.id()
    }

    fn name(&self) -> String {
        self.base.name().to_owned()
    }

    /// Advances the state machine and the sim pose. The scheduler calls this every 20 ms.
    fn periodic(&self) {
        self.system_state
            .set(handle_state_transition(self.wanted_state.get()));
        self.apply_states();

        telemetry::log_dash("Swerve/CurrentCommand", self.get_current_command_name());
        telemetry::log_dash(
            "Swerve/TeleopVelocityCoefficient",
            self.get_teleop_velocity_coefficient(),
        );
        telemetry::log_dash(
            "Swerve/RotationVelocityCoefficient",
            self.get_rotation_velocity_coefficient(),
        );
        self.log_battery_usage();
        self.check_pigeon_connection();

        if utils::is_simulation() {
            telemetry::log_struct("Sim/SimPose", &self.get_robot_pose());
            if let Some(sim) = self.sim_backend()
                && let Some(pose3d) = sim.update_bump_sim()
            {
                self.sim_robot_pose3d.set(pose3d);
                telemetry::log_struct("Sim/RobotPose3d", &pose3d);
            }
        }

        telemetry::log_dash("Swerve/WantedState", self.wanted_state.get().name());
        telemetry::log_dash("Swerve/SystemState", self.system_state.get().name());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wpimath::units::inches_to_meters;

    fn assert_close(expected: f64, actual: f64) {
        assert!((expected - actual).abs() < 1e-9, "{expected} vs {actual}");
    }

    #[test]
    fn matches_java_swerve_logic() {
        golden::check("swerve_logic", 1e-9, |op, i| {
            let flag = |b: bool| vec![f64::from(u8::from(b))];
            let pose = |x: f64, y: f64| Pose2d::from_xy(x, y, Rotation2d::ZERO);
            Some(match op {
                "enemy_zone" => flag(enemy_alliance_zone_contains(i[0] != 0.0, pose(i[1], i[2]))),
                "neutral_zone" => flag(neutral_zone_contains(pose(i[0], i[1]))),
                "field_left" => flag(is_in_field_left(pose(i[0], i[1]))),
                "field_right" => flag(is_in_field_right(pose(i[0], i[1]))),
                "closest_cardinal" => vec![closest_cardinal_degrees(i[0])],
                "front_closest_to_angle" => flag(front_closest_to_angle_for(i[0], i[1])),
                "rotation_difference" => vec![get_rotation_difference(i[0], i[1])],
                "going_too_fast" => flag(going_too_fast_for(
                    ChassisSpeeds::new(i[0], i[1], 0.0),
                    i[2],
                )),
                _ => return None,
            })
        });
    }

    #[test]
    fn each_wanted_state_maps_to_the_system_state_of_the_same_name() {
        assert_eq!(
            handle_state_transition(WantedState::TeleopDrive),
            SystemState::TeleopDrive
        );
        assert_eq!(
            handle_state_transition(WantedState::PilotAimAtTarget),
            SystemState::PilotAimAtTarget
        );
        assert_eq!(
            handle_state_transition(WantedState::XBrake),
            SystemState::XBrake
        );
        assert_eq!(
            handle_state_transition(WantedState::Idle),
            SystemState::Idle
        );
    }

    #[test]
    fn state_names_match_the_java_enum_constants() {
        assert_eq!(WantedState::PilotAimAtTarget.name(), "PILOT_AIM_AT_TARGET");
        assert_eq!(SystemState::XBrake.name(), "X_BRAKE");
    }

    #[test]
    fn no_alliance_gives_zero_joystick_speeds() {
        let speeds = joystick_speeds(None, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, Rotation2d::ZERO);
        assert_eq!(speeds, ChassisSpeeds::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn red_inverts_the_stick_translation_and_blue_does_not() {
        let blue = joystick_speeds(
            Some(Alliance::Blue),
            1.0,
            0.5,
            0.25,
            1.0,
            1.0,
            0.0,
            Rotation2d::ZERO,
        );
        let red = joystick_speeds(
            Some(Alliance::Red),
            1.0,
            0.5,
            0.25,
            1.0,
            1.0,
            0.0,
            Rotation2d::ZERO,
        );
        assert_close(1.0, blue.vx);
        assert_close(0.5, blue.vy);
        assert_close(-1.0, red.vx);
        assert_close(-0.5, red.vy);
        assert_close(0.25, blue.omega);
        assert_close(0.25, red.omega);
    }

    #[test]
    fn coefficients_scale_translation_and_rotation_separately() {
        let speeds = joystick_speeds(
            Some(Alliance::Blue),
            1.0,
            1.0,
            1.0,
            0.1,
            0.5,
            0.0,
            Rotation2d::ZERO,
        );
        assert_close(0.1, speeds.vx);
        assert_close(0.1, speeds.vy);
        assert_close(0.5, speeds.omega);
    }

    #[test]
    fn without_spin_the_result_is_the_field_speeds_unchanged() {
        // Java: the result is field-relative, ready for ApplyFieldSpeeds.
        let speeds = joystick_speeds(
            Some(Alliance::Blue),
            1.0,
            0.0,
            0.0,
            1.0,
            1.0,
            0.0,
            Rotation2d::from_degrees(90.0),
        );
        assert_close(1.0, speeds.vx);
        assert_close(0.0, speeds.vy);
    }

    #[test]
    fn spin_rotates_the_field_speeds_by_the_skew_angle() {
        let speeds = joystick_speeds(
            Some(Alliance::Blue),
            1.0,
            0.0,
            0.0,
            1.0,
            1.0,
            1.0,
            Rotation2d::from_degrees(90.0),
        );
        assert_close((-0.03_f64).cos(), speeds.vx);
        assert_close((-0.03_f64).sin(), speeds.vy);
    }

    #[test]
    fn closest_cardinal_snaps_to_quarter_turns() {
        use std::f64::consts::PI;
        assert_eq!(closest_cardinal_degrees(0.0), 0.0);
        assert_eq!(closest_cardinal_degrees(PI / 4.0), 0.0);
        assert_eq!(closest_cardinal_degrees(PI / 4.0 + 0.01), 90.0);
        assert_eq!(closest_cardinal_degrees(PI / 2.0), 90.0);
        assert_eq!(closest_cardinal_degrees(PI), 180.0);
        assert_eq!(closest_cardinal_degrees(-PI), 180.0);
        assert_eq!(closest_cardinal_degrees(-3.0 * PI / 4.0), 180.0);
        assert_eq!(closest_cardinal_degrees(-PI / 2.0), 270.0);
        assert_eq!(closest_cardinal_degrees(-PI / 4.0), 270.0);
    }

    #[test]
    fn rotation_difference_wraps_to_at_most_180() {
        assert_close(10.0, get_rotation_difference(5.0, 355.0));
        assert_close(180.0, get_rotation_difference(0.0, 180.0));
        assert_close(170.0, get_rotation_difference(-90.0, 80.0));
        assert_close(0.0, get_rotation_difference(30.0, 30.0));
    }

    #[test]
    fn front_closest_compares_the_front_and_the_back_to_the_target() {
        assert!(front_closest_to_angle_for(10.0, 0.0));
        assert!(!front_closest_to_angle_for(170.0, 0.0));
        assert!(front_closest_to_angle_for(-170.0, 180.0));
    }

    #[test]
    fn zones_use_the_neutral_and_enemy_rectangles() {
        let center = Pose2d::from_xy(
            field::FIELD_LENGTH / 2.0,
            field::FIELD_WIDTH / 2.0,
            Rotation2d::ZERO,
        );
        assert!(neutral_zone_contains(center));
        let near_blue_wall = Pose2d::from_xy(1.0, 4.0, Rotation2d::ZERO);
        assert!(!neutral_zone_contains(near_blue_wall));

        // Blue sees the red end as the enemy zone, red sees the blue end.
        let red_end = Pose2d::from_xy(field::FIELD_LENGTH - 1.0, 4.0, Rotation2d::ZERO);
        assert!(enemy_alliance_zone_contains(false, red_end));
        assert!(!enemy_alliance_zone_contains(true, red_end));
        assert!(enemy_alliance_zone_contains(true, near_blue_wall));
        assert!(!enemy_alliance_zone_contains(false, near_blue_wall));
    }

    #[test]
    fn the_inch_constants_match_unit_conversion() {
        assert_close(inches_to_meters(1.0), TRANSLATION_ERROR_MARGIN_METERS);
        assert_close(inches_to_meters(283.0), NEUTRAL_DEPTH_METERS);
        assert_close(inches_to_meters(317.7), NEUTRAL_LENGTH_METERS);
        assert_close(inches_to_meters(180.0), ENEMY_ALLIANCE_DEPTH_METERS);
    }

    #[test]
    fn simulation_regulation_replaces_inversions_offsets_and_steer_model() {
        let mut m = ModuleConstants {
            encoder_offset_rot: -0.164,
            drive_motor_inverted: true,
            steer_motor_inverted: true,
            encoder_inverted: true,
            steer_motor_gear_ratio: 26.09,
            drive_motor_gear_ratio: 6.03,
            ..ModuleConstants::default()
        };
        regulate_module_constant_for_simulation(&mut m);
        assert_eq!(m.encoder_offset_rot, 0.0);
        assert!(!m.drive_motor_inverted && !m.steer_motor_inverted && !m.encoder_inverted);
        assert_eq!(m.steer_motor_gains.k_p, 1000.0);
        assert_eq!(m.steer_motor_gains.k_d, 60.0);
        assert_eq!(m.steer_motor_gains.k_s, 0.15);
        assert_eq!(m.steer_motor_gains.k_v, 1.5);
        assert_eq!(m.steer_motor_gear_ratio, 21.428571428571427);
        assert_eq!(m.drive_friction_voltage, 0.1);
        assert_eq!(m.steer_friction_voltage, 0.05);
        assert_eq!(m.steer_inertia, 0.05);
        // untouched by the Java
        assert_eq!(m.drive_motor_gear_ratio, 6.03);
    }
}
