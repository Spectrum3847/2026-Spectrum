//! Port of the maple-sim 0.4.0-beta classes that `MapleSimSwerveDrivetrain` uses.
//!
//! Source: github.com/Shenzhen-Robotics-Alliance/maple-sim tag v0.4.0-beta, MIT, Copyright (c)
//! 2024-2025 Team 5516 "Iron Maple". Ported: `SimMotorConfigs`, `MapleMotorSim`,
//! `SimMotorState`, `SimulatedMotorController` and its generic controller, `SimulatedBattery`,
//! `SwerveModuleSimulationConfig`, `SwerveModuleSimulation`, `DriveTrainSimulationConfig`,
//! `GyroSimulation`, `SwerveDriveSimulation`, `AbstractDriveTrainSimulation` and
//! `SimulatedArena` with the `Arena2026Rebuilt` field obstacles. Left out: game pieces, the hub
//! and outpost simulations, the Reset Field and match breakdown dashboard topics, the encoder
//! caches and the relative encoder offset (nothing in the robot reads them), and hard and
//! software motor limits (unset in maple-sim's defaults). Units are SI, angles in radians.

use super::physics::{Obstacle, RigidBody, Shape, Transform, V2, World};
use std::time::{SystemTime, UNIX_EPOCH};
use wpilib::driver_station;
use wpimath::filter::LinearFilter;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::kinematics::{ChassisSpeeds, SwerveDriveKinematics, SwerveModuleState};
use wpimath::math_util;
use wpimath::system::DcMotor;

const BUMPER_COEFFICIENT_OF_FRICTION: f64 = 0.65;
const BUMPER_COEFFICIENT_OF_RESTITUTION: f64 = 0.08;
const BATTERY_NOMINAL_VOLTAGE: f64 = 13.5;
const BATTERY_INTERNAL_RESISTANCE_OHMS: f64 = 0.02;
const DEFAULT_CURRENT_LIMIT_AMPS: f64 = 150.0;
const CURRENT_THRESHOLD: f64 = 1.2;
const GRAVITY: f64 = 9.8;
const OBSTACLE_FRICTION: f64 = 0.6;
const OBSTACLE_RESTITUTION: f64 = 0.3;
const INCHES: f64 = 0.0254;

fn bounding_check(value: f64, lower: f64, upper: f64, name: &str, unit: &str) {
    if (lower..=upper).contains(&value) {
        return;
    }
    driver_station::report_error(&format!(
        "The provided \"{name}\" is {value}{unit}, which seems abnormal, please check its correctness"
    ));
}

/// Wraps like adding `Rotation2d`s does.
fn wrap_angle(radians: f64) -> f64 {
    radians.sin().atan2(radians.cos())
}

/// Small xorshift generator. maple-sim uses `java.util.Random`, and nothing here needs the
/// same sequence.
struct Rng(u64);

impl Rng {
    fn from_clock() -> Self {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(1, |d| d.as_nanos() as u64);
        Self::new(nanos)
    }

    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        let bits = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11;
        bits as f64 / (1u64 << 53) as f64
    }

    /// `MapleCommonMath.generateRandomNormal`: Box-Muller.
    fn normal(&mut self, mean: f64, std_dev: f64) -> f64 {
        let u1 = self.next_f64();
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos() * std_dev + mean
    }
}

/// `MapleCommonMath.getAngle`: zero for a vector too short to have a direction.
fn angle_of(v: Translation2d) -> Rotation2d {
    if v.norm() < 1e-6 {
        Rotation2d::ZERO
    } else {
        v.angle()
    }
}

/// `SimMotorConfigs`. `friction` is a torque in N*m.
#[derive(Clone, Copy, Debug)]
pub struct SimMotorConfigs {
    pub motor: DcMotor,
    pub gearing: f64,
    pub load_moi: f64,
    pub friction: f64,
}

impl SimMotorConfigs {
    pub fn new(motor: DcMotor, gearing: f64, load_moi: f64, friction_voltage: f64) -> Self {
        // maple-sim: the friction torque is on the motor shaft, not scaled by the gearing
        let friction = motor.torque(motor.current(0.0, friction_voltage));
        Self {
            motor,
            gearing,
            load_moi,
            friction,
        }
    }

    pub fn calculate_mechanism_velocity(&self, current: f64, voltage: f64) -> f64 {
        self.motor.speed(self.motor.torque(current), voltage) / self.gearing
    }

    pub fn calculate_current(&self, mechanism_velocity: f64, voltage: f64) -> f64 {
        self.motor
            .current(mechanism_velocity * self.gearing, voltage)
    }

    pub fn calculate_current_from_torque(&self, torque: f64) -> f64 {
        self.motor.current_from_torque(torque / self.gearing)
    }

    pub fn calculate_torque(&self, current: f64) -> f64 {
        self.motor.torque(current) * self.gearing
    }

    pub fn free_spin_mechanism_velocity(&self) -> f64 {
        self.motor.free_speed_rad_per_sec / self.gearing
    }
}

/// `SimulatedMotorController`. maple-sim reads the battery voltage from a static; here it is
/// passed in. `request_voltage` is only for the generic controller.
pub trait SimulatedMotorController: Send {
    fn update_control_signal(
        &mut self,
        mechanism_angle: f64,
        mechanism_velocity: f64,
        encoder_angle: f64,
        encoder_velocity: f64,
        battery_voltage: f64,
    ) -> f64;

    fn request_voltage(&mut self, _volts: f64) {}
}

/// `SimulatedMotorController.GenericMotorController` with its default 150 A current limit.
pub struct GenericMotorController {
    model: DcMotor,
    requested_voltage: f64,
}

impl GenericMotorController {
    pub fn new(model: DcMotor) -> Self {
        Self {
            model,
            requested_voltage: 0.0,
        }
    }

    fn constrain_output_voltage(&self, encoder_velocity: f64, requested: f64) -> f64 {
        let current_at_requested = self.model.current(encoder_velocity, requested);
        let mut limited = requested;
        if current_at_requested.abs() > CURRENT_THRESHOLD * DEFAULT_CURRENT_LIMIT_AMPS {
            let limited_current = DEFAULT_CURRENT_LIMIT_AMPS.copysign(current_at_requested);
            limited = self
                .model
                .voltage(self.model.torque(limited_current), encoder_velocity);
        }
        // The limit may not raise the output voltage
        if limited.abs() > requested.abs() {
            limited = requested;
        }
        limited
    }
}

impl SimulatedMotorController for GenericMotorController {
    fn update_control_signal(
        &mut self,
        _mechanism_angle: f64,
        _mechanism_velocity: f64,
        _encoder_angle: f64,
        encoder_velocity: f64,
        _battery_voltage: f64,
    ) -> f64 {
        self.constrain_output_voltage(encoder_velocity, self.requested_voltage)
    }

    fn request_voltage(&mut self, volts: f64) {
        self.requested_voltage = volts;
    }
}

/// What `SimulatedBattery` reads and writes outside the sim: `RoboRioSim` and the dashboard.
pub trait RoboRioSim: Send {
    fn set_vin_voltage(&mut self, volts: f64);
    fn brownout_voltage(&self) -> f64;
    fn publish(&mut self, total_current_amps: f64, battery_volts: f64);
}

/// `SimulatedBattery`: the battery voltage sags with the total supply current.
pub struct SimulatedBattery {
    current_filter: LinearFilter,
    voltage: f64,
    roborio: Box<dyn RoboRioSim>,
}

impl SimulatedBattery {
    pub fn new(roborio: Box<dyn RoboRioSim>) -> Self {
        Self {
            current_filter: LinearFilter::moving_average(50),
            voltage: BATTERY_NOMINAL_VOLTAGE,
            roborio,
        }
    }

    pub fn voltage(&self) -> f64 {
        self.voltage
    }

    pub fn clamp(&self, volts: f64) -> f64 {
        math_util::clamp(volts, -self.voltage, self.voltage)
    }

    pub fn simulation_sub_tick(&mut self, total_current_drawn: f64) {
        let total_current = self.current_filter.calculate(total_current_drawn);
        let loaded = BATTERY_NOMINAL_VOLTAGE - total_current * BATTERY_INTERNAL_RESISTANCE_OHMS;
        if loaded.is_nan() {
            self.voltage = 12.0;
            driver_station::report_error(
                "[MapleSim] Internal Library Error: Calculated battery voltage is invalid, reverting to normal operation voltage...",
            );
        } else {
            self.voltage = loaded.max(0.0);
        }
        let brownout = self.roborio.brownout_voltage();
        if self.voltage < brownout {
            self.voltage = brownout;
            driver_station::report_error(
                "[MapleSim] BrownOut Detected, protecting battery voltage...",
            );
        }
        self.roborio.set_vin_voltage(self.voltage);
        self.roborio.publish(total_current, self.voltage);
    }
}

/// `MapleMotorSim`: a motor with a load, stepped each sub tick.
pub struct MapleMotorSim {
    configs: SimMotorConfigs,
    angular_position: f64,
    velocity: f64,
    controller: Box<dyn SimulatedMotorController>,
    applied_voltage: f64,
    stator_current: f64,
}

impl MapleMotorSim {
    pub fn new(configs: SimMotorConfigs) -> Self {
        Self {
            configs,
            angular_position: 0.0,
            velocity: 0.0,
            controller: Box::new(GenericMotorController::new(configs.motor)),
            applied_voltage: 0.0,
            stator_current: 0.0,
        }
    }

    pub fn update(&mut self, dt: f64, battery: &SimulatedBattery) {
        let gearing = self.configs.gearing;
        self.applied_voltage = self.controller.update_control_signal(
            self.angular_position,
            self.velocity,
            self.angular_position * gearing,
            self.velocity * gearing,
            battery.voltage(),
        );
        self.applied_voltage = battery.clamp(self.applied_voltage);
        self.stator_current = self
            .configs
            .calculate_current(self.velocity, self.applied_voltage);
        self.step(self.configs.calculate_torque(self.stator_current), dt);
    }

    /// `SimMotorState.step`: electric torque, then friction that stops the load instead of
    /// reversing it.
    fn step(&mut self, electric_torque: f64, dt: f64) {
        let moi = self.configs.load_moi;
        let mut velocity = self.velocity + electric_torque / moi * dt;
        let friction_delta = self.configs.friction.copysign(-velocity) / moi * dt;
        if (velocity + friction_delta) * velocity <= 0.0 {
            velocity = 0.0;
        } else {
            velocity += friction_delta;
        }
        self.angular_position += velocity * dt;
        self.velocity = velocity;
    }

    pub fn angular_position(&self) -> f64 {
        self.angular_position
    }

    pub fn velocity(&self) -> f64 {
        self.velocity
    }

    pub fn applied_voltage(&self) -> f64 {
        self.applied_voltage
    }

    pub fn supply_current(&self, battery_voltage: f64) -> f64 {
        self.stator_current * (self.applied_voltage / battery_voltage)
    }

    pub fn use_motor_controller(&mut self, controller: Box<dyn SimulatedMotorController>) {
        self.controller = controller;
    }

    pub fn controller_mut(&mut self) -> &mut dyn SimulatedMotorController {
        &mut *self.controller
    }
}

/// `SwerveModuleSimulationConfig`.
#[derive(Clone, Copy, Debug)]
pub struct SwerveModuleSimulationConfig {
    pub drive_motor_configs: SimMotorConfigs,
    pub steer_motor_configs: SimMotorConfigs,
    pub drive_gear_ratio: f64,
    pub steer_gear_ratio: f64,
    pub wheels_coefficient_of_friction: f64,
    pub wheel_radius: f64,
}

impl SwerveModuleSimulationConfig {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        drive_motor_model: DcMotor,
        steer_motor_model: DcMotor,
        drive_gear_ratio: f64,
        steer_gear_ratio: f64,
        drive_friction_voltage: f64,
        steer_friction_voltage: f64,
        wheel_radius: f64,
        steer_rotational_inertia: f64,
        wheels_coefficient_of_friction: f64,
    ) -> Self {
        bounding_check(
            drive_gear_ratio,
            4.0,
            24.0,
            "drive gear ratio",
            "times reduction",
        );
        bounding_check(
            steer_gear_ratio,
            6.0,
            50.0,
            "steer gear ratio",
            "times reduction",
        );
        bounding_check(
            drive_friction_voltage,
            0.01,
            0.35,
            "drive friction voltage",
            "volts",
        );
        bounding_check(
            steer_friction_voltage,
            0.01,
            0.6,
            "steer friction voltage",
            "volts",
        );
        bounding_check(
            wheel_radius / INCHES,
            1.0,
            3.2,
            "drive wheel radius",
            "inches",
        );
        bounding_check(
            steer_rotational_inertia,
            0.005,
            0.06,
            "steer rotation inertia",
            "kg * m^2",
        );
        bounding_check(
            wheels_coefficient_of_friction,
            0.6,
            1.9,
            "tire coefficient of friction",
            "",
        );
        Self {
            drive_motor_configs: SimMotorConfigs::new(
                drive_motor_model,
                drive_gear_ratio,
                0.0,
                drive_friction_voltage,
            ),
            steer_motor_configs: SimMotorConfigs::new(
                steer_motor_model,
                steer_gear_ratio,
                steer_rotational_inertia,
                steer_friction_voltage,
            ),
            drive_gear_ratio,
            steer_gear_ratio,
            wheels_coefficient_of_friction,
            wheel_radius,
        }
    }

    pub fn gripping_force_newtons(&self, gravity_force_on_module_newtons: f64) -> f64 {
        gravity_force_on_module_newtons * self.wheels_coefficient_of_friction
    }

    /// The theoretical top speed of a module, in m/s.
    pub fn maximum_ground_speed(&self) -> f64 {
        self.drive_motor_configs.free_spin_mechanism_velocity() * self.wheel_radius
    }
}

/// `SwerveModuleSimulation`.
pub struct SwerveModuleSimulation {
    pub config: SwerveModuleSimulationConfig,
    steer_motor_sim: MapleMotorSim,
    drive_motor_applied_voltage: f64,
    drive_motor_stator_current: f64,
    drive_wheel_final_position: f64,
    drive_wheel_final_speed: f64,
    drive_motor_controller: Box<dyn SimulatedMotorController>,
}

impl SwerveModuleSimulation {
    pub fn new(config: SwerveModuleSimulationConfig) -> Self {
        Self {
            config,
            steer_motor_sim: MapleMotorSim::new(config.steer_motor_configs),
            drive_motor_applied_voltage: 0.0,
            drive_motor_stator_current: 0.0,
            drive_wheel_final_position: 0.0,
            drive_wheel_final_speed: 0.0,
            drive_motor_controller: Box::new(GenericMotorController::new(
                config.drive_motor_configs.motor,
            )),
        }
    }

    pub fn use_drive_motor_controller(&mut self, controller: Box<dyn SimulatedMotorController>) {
        self.drive_motor_controller = controller;
    }

    pub fn use_steer_motor_controller(&mut self, controller: Box<dyn SimulatedMotorController>) {
        self.steer_motor_sim.use_motor_controller(controller);
    }

    pub fn drive_motor_controller_mut(&mut self) -> &mut dyn SimulatedMotorController {
        &mut *self.drive_motor_controller
    }

    pub fn steer_motor_controller_mut(&mut self) -> &mut dyn SimulatedMotorController {
        self.steer_motor_sim.controller_mut()
    }

    /// Steps the steer motor, then returns the force this module pushes the chassis with.
    pub fn update_simulation_sub_tick_get_module_force(
        &mut self,
        module_current_ground_velocity_world_relative: V2,
        robot_facing: f64,
        gravity_force_on_module_newtons: f64,
        dt: f64,
        battery: &SimulatedBattery,
    ) -> V2 {
        self.steer_motor_sim.update(dt, battery);

        let gripping_force = self
            .config
            .gripping_force_newtons(gravity_force_on_module_newtons);
        let module_world_facing = self.steer_absolute_angle() + robot_facing;
        let propelling_force = self.propelling_force(
            gripping_force,
            module_world_facing,
            module_current_ground_velocity_world_relative,
            battery,
        );

        self.drive_wheel_final_position += self.drive_wheel_final_speed * dt;
        propelling_force
    }

    fn propelling_force(
        &mut self,
        gripping_force_newtons: f64,
        module_world_facing: f64,
        module_current_ground_velocity: V2,
        battery: &SimulatedBattery,
    ) -> V2 {
        let wheel_radius = self.config.wheel_radius;
        let drive_wheel_torque = self.drive_wheel_torque(battery);
        let mut propelling_force_newtons = drive_wheel_torque / wheel_radius;
        let skidding = propelling_force_newtons.abs() > gripping_force_newtons;
        if skidding {
            propelling_force_newtons = gripping_force_newtons.copysign(propelling_force_newtons);
        }

        let facing = V2::polar(1.0, module_world_facing);
        let floor_velocity_on_wheel_direction = module_current_ground_velocity.magnitude()
            * module_current_ground_velocity.angle_between(facing).cos();

        // A gripped wheel turns with the floor
        self.drive_wheel_final_speed = floor_velocity_on_wheel_direction / wheel_radius;

        if skidding {
            let drive = &self.config.drive_motor_configs;
            let skidding_equilibrium_wheel_speed = drive.calculate_mechanism_velocity(
                drive.calculate_current_from_torque(propelling_force_newtons * wheel_radius),
                self.drive_motor_applied_voltage,
            );
            self.drive_wheel_final_speed =
                self.drive_wheel_final_speed * 0.5 + skidding_equilibrium_wheel_speed * 0.5;
        }

        V2::polar(propelling_force_newtons, module_world_facing)
    }

    fn drive_wheel_torque(&mut self, battery: &SimulatedBattery) -> f64 {
        let ratio = self.config.drive_gear_ratio;
        self.drive_motor_applied_voltage = self.drive_motor_controller.update_control_signal(
            self.drive_wheel_final_position,
            self.drive_wheel_final_speed,
            self.drive_wheel_final_position * ratio,
            self.drive_wheel_final_speed * ratio,
            battery.voltage(),
        );
        self.drive_motor_applied_voltage = battery.clamp(self.drive_motor_applied_voltage);

        let drive = &self.config.drive_motor_configs;
        self.drive_motor_stator_current = drive.calculate_current(
            self.drive_wheel_final_speed,
            self.drive_motor_applied_voltage,
        );
        let torque = drive.calculate_torque(self.drive_motor_stator_current);
        math_util::apply_deadband_with_max(torque, drive.friction, f64::INFINITY)
    }

    pub fn current_state(&self) -> SwerveModuleState {
        SwerveModuleState::new(
            self.drive_wheel_final_speed * self.config.wheel_radius,
            Rotation2d::new(self.steer_absolute_angle()),
        )
    }

    /// The state the module reaches with the current drive voltage and nothing but friction.
    fn free_spin_state(&self) -> SwerveModuleState {
        let drive = &self.config.drive_motor_configs;
        let speed = drive.calculate_mechanism_velocity(
            drive.calculate_current_from_torque(drive.friction),
            self.drive_motor_applied_voltage,
        );
        SwerveModuleState::new(
            speed * self.config.wheel_radius,
            Rotation2d::new(self.steer_absolute_angle()),
        )
    }

    pub fn drive_motor_applied_voltage(&self) -> f64 {
        self.drive_motor_applied_voltage
    }

    pub fn drive_motor_stator_current(&self) -> f64 {
        self.drive_motor_stator_current
    }

    pub fn drive_motor_supply_current(&self, battery_voltage: f64) -> f64 {
        self.drive_motor_stator_current * (self.drive_motor_applied_voltage / battery_voltage)
    }

    pub fn steer_motor_supply_current(&self, battery_voltage: f64) -> f64 {
        self.steer_motor_sim.supply_current(battery_voltage)
    }

    pub fn steer_motor_applied_voltage(&self) -> f64 {
        self.steer_motor_sim.applied_voltage()
    }

    pub fn drive_wheel_final_position(&self) -> f64 {
        self.drive_wheel_final_position
    }

    pub fn drive_wheel_final_speed(&self) -> f64 {
        self.drive_wheel_final_speed
    }

    pub fn steer_absolute_angle(&self) -> f64 {
        self.steer_motor_sim.angular_position()
    }

    pub fn steer_absolute_encoder_speed(&self) -> f64 {
        self.steer_motor_sim.velocity()
    }
}

/// `GyroSimulation`, as `COTS.ofPigeon2()` configures it. The drivetrain never reads it.
pub struct GyroSimulation {
    average_drifting_in_30_secs_motionless_deg: f64,
    velocity_measurement_standard_deviation_percent: f64,
    gyro_reading: f64,
    measured_angular_velocity: f64,
    previous_angular_velocity: f64,
    rng: Rng,
}

/// Angular acceleration, in rad/s^2, above which the chassis counts as hit.
const ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING: f64 = 500.0;
/// Drift per multiple of the threshold. maple-sim applies it as radians.
const DRIFT_DUE_TO_IMPACT_COEFFICIENT: f64 = std::f64::consts::PI / 180.0;

impl GyroSimulation {
    pub fn pigeon2() -> Self {
        Self::new(0.5, 0.02)
    }

    pub fn new(
        average_drifting_in_30_secs_motionless_deg: f64,
        velocity_measurement_standard_deviation_percent: f64,
    ) -> Self {
        Self {
            average_drifting_in_30_secs_motionless_deg,
            velocity_measurement_standard_deviation_percent,
            gyro_reading: 0.0,
            measured_angular_velocity: 0.0,
            previous_angular_velocity: 0.0,
            rng: Rng::from_clock(),
        }
    }

    pub fn gyro_reading(&self) -> f64 {
        self.gyro_reading
    }

    pub fn measured_angular_velocity(&self) -> f64 {
        self.measured_angular_velocity
    }

    pub fn update_simulation_sub_tick(&mut self, actual_angular_velocity: f64, dt: f64) {
        let impact = self.drifting_due_to_impact(actual_angular_velocity, dt);
        self.gyro_reading = wrap_angle(self.gyro_reading + impact);
        let d_theta = self.gyro_d_theta(actual_angular_velocity, dt);
        self.gyro_reading = wrap_angle(self.gyro_reading + d_theta);
        let no_motion = self.no_motion_drifting(dt);
        self.gyro_reading = wrap_angle(self.gyro_reading + no_motion);
    }

    fn drifting_due_to_impact(&mut self, actual_angular_velocity: f64, dt: f64) -> f64 {
        let acceleration = (actual_angular_velocity - self.previous_angular_velocity) / dt;
        let drift = if acceleration.abs() > ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING {
            acceleration.abs() / ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING
                * DRIFT_DUE_TO_IMPACT_COEFFICIENT
        } else {
            0.0
        };
        self.previous_angular_velocity = actual_angular_velocity;
        drift.copysign(-acceleration)
    }

    fn gyro_d_theta(&mut self, actual_angular_velocity: f64, dt: f64) -> f64 {
        self.measured_angular_velocity = self.rng.normal(
            actual_angular_velocity,
            self.velocity_measurement_standard_deviation_percent * actual_angular_velocity.abs(),
        );
        self.measured_angular_velocity * dt
    }

    fn no_motion_drifting(&mut self, dt: f64) -> f64 {
        let average_drifting_per_period =
            self.average_drifting_in_30_secs_motionless_deg / 30.0 * dt;
        self.rng
            .normal(0.0, average_drifting_per_period)
            .to_radians()
    }
}

/// `DriveTrainSimulationConfig`.
#[derive(Clone, Debug)]
pub struct DriveTrainSimulationConfig {
    pub robot_mass: f64,
    pub bumper_length_x: f64,
    pub bumper_width_y: f64,
    pub module_translations: Vec<Translation2d>,
    pub module: SwerveModuleSimulationConfig,
}

impl DriveTrainSimulationConfig {
    pub fn new(
        robot_mass: f64,
        bumper_length_x: f64,
        bumper_width_y: f64,
        module_translations: Vec<Translation2d>,
        module: SwerveModuleSimulationConfig,
    ) -> Self {
        bounding_check(robot_mass, 10.0, 80.0, "robot mass", "kg");
        bounding_check(bumper_length_x, 0.5, 1.5, "bumper length", "meters");
        bounding_check(bumper_width_y, 0.5, 1.5, "bumper width", "meters");
        Self {
            robot_mass,
            bumper_length_x,
            bumper_width_y,
            module_translations,
            module,
        }
    }

    pub fn density_kg_per_squared_meters(&self) -> f64 {
        self.robot_mass / (self.bumper_length_x * self.bumper_width_y)
    }

    fn extent(&self, coordinate: impl Fn(&Translation2d) -> f64) -> f64 {
        let max = self
            .module_translations
            .iter()
            .map(&coordinate)
            .fold(f64::NEG_INFINITY, f64::max);
        let min = self
            .module_translations
            .iter()
            .map(&coordinate)
            .fold(f64::INFINITY, f64::min);
        max - min
    }

    pub fn drive_base_radius(&self) -> f64 {
        self.extent(|t| t.x).hypot(self.extent(|t| t.y))
    }
}

/// `SwerveDriveSimulation` with `AbstractDriveTrainSimulation`.
pub struct SwerveDriveSimulation {
    pub config: DriveTrainSimulationConfig,
    body: RigidBody,
    modules: Vec<SwerveModuleSimulation>,
    gyro: GyroSimulation,
    kinematics: SwerveDriveKinematics<4>,
    gravity_force_on_each_module: f64,
    previous_module_speeds_field_relative: Translation2d,
}

impl SwerveDriveSimulation {
    /// # Panics
    /// If the config does not have exactly four module translations.
    pub fn new(config: DriveTrainSimulationConfig, initial_pose_on_field: Pose2d) -> Self {
        let translations: [Translation2d; 4] = config
            .module_translations
            .clone()
            .try_into()
            .expect("a swerve drivetrain simulation needs four modules");
        let mut body = RigidBody::rectangle(
            config.bumper_length_x,
            config.bumper_width_y,
            config.density_kg_per_squared_meters(),
            BUMPER_COEFFICIENT_OF_FRICTION,
            BUMPER_COEFFICIENT_OF_RESTITUTION,
        );
        body.linear_damping = 1.4;
        body.angular_damping = 1.4;
        let modules = (0..translations.len())
            .map(|_| SwerveModuleSimulation::new(config.module))
            .collect::<Vec<_>>();
        let gravity_force_on_each_module = config.robot_mass * GRAVITY / modules.len() as f64;
        let mut sim = Self {
            kinematics: SwerveDriveKinematics::new(translations),
            config,
            body,
            modules,
            gyro: GyroSimulation::pigeon2(),
            gravity_force_on_each_module,
            previous_module_speeds_field_relative: Translation2d::new(0.0, 0.0),
        };
        sim.set_simulation_world_pose(initial_pose_on_field);
        sim
    }

    /// Teleports the body and stops its linear motion, but not its spin.
    pub fn set_simulation_world_pose(&mut self, robot_pose: Pose2d) {
        self.body.transform = Transform::new(
            V2::new(robot_pose.x(), robot_pose.y()),
            robot_pose.rotation.radians(),
        );
        self.body.linear_velocity = V2::ZERO;
    }

    pub fn simulated_pose(&self) -> Pose2d {
        let t = &self.body.transform;
        // dyn4j reads the angle back with acos and the sign of the sine
        let acos = t.cost.acos();
        let angle = if t.sint >= 0.0 { acos } else { -acos };
        Pose2d::from_xy(t.translation.x, t.translation.y, Rotation2d::new(angle))
    }

    pub fn chassis_speeds_field_relative(&self) -> ChassisSpeeds {
        ChassisSpeeds::new(
            self.body.linear_velocity.x,
            self.body.linear_velocity.y,
            self.body.angular_velocity,
        )
    }

    pub fn chassis_speeds_robot_relative(&self) -> ChassisSpeeds {
        ChassisSpeeds::from_field_relative(
            self.chassis_speeds_field_relative(),
            self.simulated_pose().rotation,
        )
    }

    pub fn modules(&self) -> &[SwerveModuleSimulation] {
        &self.modules
    }

    pub fn modules_mut(&mut self) -> &mut [SwerveModuleSimulation] {
        &mut self.modules
    }

    pub fn gyro(&self) -> &GyroSimulation {
        &self.gyro
    }

    pub(crate) fn body_mut(&mut self) -> &mut RigidBody {
        &mut self.body
    }

    /// Total supply current of the drive and steer motors, in maple-sim's order.
    pub fn total_supply_current(&self, battery_voltage: f64) -> f64 {
        self.modules
            .iter()
            .map(|m| {
                m.drive_motor_supply_current(battery_voltage)
                    + m.steer_motor_supply_current(battery_voltage)
            })
            .sum()
    }

    pub fn simulation_sub_tick(&mut self, dt: f64, battery: &SimulatedBattery) {
        self.simulate_chassis_friction_force(dt);
        self.simulate_chassis_friction_torque();
        self.simulate_module_propelling_forces(dt, battery);
        self.gyro
            .update_simulation_sub_tick(self.body.angular_velocity, dt);
    }

    fn simulate_chassis_friction_force(&mut self, dt: f64) {
        let module_speeds = self.module_speeds();
        let rotation = self.simulated_pose().rotation;

        // Friction pulls the chassis from its floor speeds toward the module speeds
        let difference = module_speeds - self.chassis_speeds_robot_relative();
        let floor_and_module_speeds_diff_field_relative =
            Translation2d::new(difference.vx, difference.vy).rotate_by(rotation);
        let friction_force_gain = 3.0;
        let total_gripping_force = self.modules[0]
            .config
            .gripping_force_newtons(self.gravity_force_on_each_module)
            * self.modules.len() as f64;
        let speeds_difference_friction_force = V2::polar(
            (friction_force_gain
                * total_gripping_force
                * floor_and_module_speeds_diff_field_relative.norm())
            .min(total_gripping_force),
            angle_of(floor_and_module_speeds_diff_field_relative).radians(),
        );

        // Centripetal friction while the chassis changes its direction of travel
        let module_speeds_field_relative =
            ChassisSpeeds::from_robot_relative(module_speeds, rotation);
        let module_translation_field_relative = Translation2d::new(
            module_speeds_field_relative.vx,
            module_speeds_field_relative.vy,
        );
        let d_theta = angle_of(module_translation_field_relative)
            .rotate_by(-angle_of(self.previous_module_speeds_field_relative));
        let orbital_angular_velocity = d_theta.radians() / dt;
        let centripetal_force_direction = angle_of(self.previous_module_speeds_field_relative)
            .rotate_by(Rotation2d::from_degrees(90.0));
        let centripetal_friction_force = V2::polar(
            self.previous_module_speeds_field_relative.norm()
                * orbital_angular_velocity
                * self.config.robot_mass,
            centripetal_force_direction.radians(),
        );
        self.previous_module_speeds_field_relative = module_translation_field_relative;

        let total_unlimited = centripetal_friction_force.plus(speeds_difference_friction_force);
        self.body.apply_force(V2::polar(
            total_gripping_force.min(total_unlimited.magnitude()),
            total_unlimited.direction(),
        ));
    }

    fn simulate_chassis_friction_torque(&mut self) {
        let max_angular_velocity = self.max_angular_velocity();
        let desired_rotational_motion_percent =
            (self.desired_speed().omega / max_angular_velocity).abs();
        let angular_velocity = self.body.angular_velocity;
        let actual_rotational_motion_percent = (angular_velocity / max_angular_velocity).abs();
        let difference_between_floor_speed_and_module_speed =
            self.module_speeds().omega - angular_velocity;
        let gripping_torque_magnitude = self.modules[0]
            .config
            .gripping_force_newtons(self.gravity_force_on_each_module)
            * self.config.module_translations[0].norm()
            * self.modules.len() as f64;
        let friction_torque_gain = 1.0;

        if actual_rotational_motion_percent < 0.01 && desired_rotational_motion_percent < 0.02 {
            self.body.angular_velocity = 0.0;
        } else {
            self.body.apply_torque(
                (friction_torque_gain
                    * gripping_torque_magnitude
                    * difference_between_floor_speed_and_module_speed.abs())
                .min(gripping_torque_magnitude)
                .copysign(difference_between_floor_speed_and_module_speed),
            );
        }
    }

    fn simulate_module_propelling_forces(&mut self, dt: f64, battery: &SimulatedBattery) {
        let rotation = self.simulated_pose().rotation.radians();
        for i in 0..self.modules.len() {
            let t = self.config.module_translations[i];
            let module_world_position = self.body.transform.transformed(V2::new(t.x, t.y));
            let module_force = self.modules[i].update_simulation_sub_tick_get_module_force(
                self.body.linear_velocity_at(module_world_position),
                rotation,
                self.gravity_force_on_each_module,
                dt,
                battery,
            );
            self.body
                .apply_force_at(module_force, module_world_position);
        }
    }

    /// The chassis speeds the modules are trying to reach.
    fn desired_speed(&self) -> ChassisSpeeds {
        let states: [SwerveModuleState; 4] =
            std::array::from_fn(|i| self.modules[i].free_spin_state());
        self.kinematics.to_chassis_speeds(&states)
    }

    /// The chassis speeds of the module states, which differ from the floor speeds while skidding.
    fn module_speeds(&self) -> ChassisSpeeds {
        let states: [SwerveModuleState; 4] =
            std::array::from_fn(|i| self.modules[i].current_state());
        self.kinematics.to_chassis_speeds(&states)
    }

    pub fn max_linear_velocity(&self) -> f64 {
        self.modules[0].config.maximum_ground_speed()
    }

    pub fn max_angular_velocity(&self) -> f64 {
        self.max_linear_velocity() / self.config.drive_base_radius()
    }
}

fn border(x1: f64, y1: f64, x2: f64, y2: f64) -> Obstacle {
    obstacle(
        Shape::Segment {
            a: V2::new(x1, y1),
            b: V2::new(x2, y2),
        },
        Transform::new(V2::ZERO, 0.0),
    )
}

fn rectangle(width: f64, height: f64, x: f64, y: f64) -> Obstacle {
    obstacle(
        Shape::Rectangle { width, height },
        Transform::new(V2::new(x, y), 0.0),
    )
}

fn obstacle(shape: Shape, transform: Transform) -> Obstacle {
    Obstacle::new(shape, transform, OBSTACLE_FRICTION, OBSTACLE_RESTITUTION)
}

/// `Arena2026Rebuilt.RebuiltFieldObstaclesMap`.
fn rebuilt_field_obstacles(add_ramp_collider: bool) -> Vec<Obstacle> {
    const FIELD_LENGTH: f64 = 16.540988;
    const FIELD_WIDTH: f64 = 8.052;
    const BLUE_HUB: V2 = V2::new(4.5974, 4.034536);
    const RED_HUB: V2 = V2::new(11.938, 4.034536);

    let mut obstacles = vec![
        border(0.0, 0.0, 0.0, FIELD_WIDTH),
        border(FIELD_LENGTH, 0.0, FIELD_LENGTH, FIELD_WIDTH),
        border(FIELD_LENGTH, FIELD_WIDTH, 0.0, FIELD_WIDTH),
        border(0.0, 0.0, FIELD_LENGTH, 0.0),
    ];

    let trench_wall_dist_x = 120.0 * INCHES + 47.0 / 2.0 * INCHES;
    let trench_wall_dist_y = 73.0 * INCHES + 47.0 / 2.0 * INCHES + 6.0 * INCHES;
    let (trench_width, trench_height) = (53.0 * INCHES, 12.0 * INCHES);
    obstacles.extend([
        rectangle(
            trench_width,
            trench_height,
            8.27 - trench_wall_dist_x,
            4.035 - trench_wall_dist_y,
        ),
        rectangle(
            trench_width,
            trench_height,
            8.27 + trench_wall_dist_x,
            4.035 - trench_wall_dist_y,
        ),
        rectangle(
            trench_width,
            trench_height,
            8.27 - trench_wall_dist_x,
            4.035 + trench_wall_dist_y,
        ),
        // maple-sim: the fourth wall repeats the first, so one trench corner has no wall
        rectangle(
            trench_width,
            trench_height,
            8.27 - trench_wall_dist_x,
            4.035 - trench_wall_dist_y,
        ),
    ]);

    obstacles.extend([
        rectangle(2.0 * INCHES, 47.0 * INCHES, 42.0 * INCHES, 159.0 * INCHES),
        rectangle(
            2.0 * INCHES,
            47.0 * INCHES,
            (651.0 - 42.0) * INCHES,
            170.0 * INCHES,
        ),
    ]);

    let hub_depth = (if add_ramp_collider { 217.0 } else { 47.0 }) * INCHES;
    for hub in [BLUE_HUB, RED_HUB] {
        obstacles.push(rectangle(47.0 * INCHES, hub_depth, hub.x, hub.y));
    }
    obstacles
}

/// `SimulatedArena` as `Arena2026Rebuilt` uses it for drivetrains: one physics world, the
/// battery, and the drivetrain simulations, stepped in that order each sub tick.
pub struct SimulatedArena {
    world: World,
    battery: SimulatedBattery,
    drive_train_simulations: Vec<SwerveDriveSimulation>,
    simulation_dt: f64,
    sub_ticks_in_1_period: usize,
}

impl SimulatedArena {
    /// `new Arena2026Rebuilt(addRampCollider)` after `overrideSimulationTimings`.
    pub fn rebuilt_2026(
        add_ramp_collider: bool,
        battery: SimulatedBattery,
        robot_period: f64,
        sub_ticks_in_1_period: usize,
    ) -> Self {
        Self {
            world: World::new(rebuilt_field_obstacles(add_ramp_collider)),
            battery,
            drive_train_simulations: Vec::new(),
            simulation_dt: robot_period / sub_ticks_in_1_period as f64,
            sub_ticks_in_1_period,
        }
    }

    pub fn add_drive_train_simulation(&mut self, simulation: SwerveDriveSimulation) -> usize {
        self.drive_train_simulations.push(simulation);
        self.drive_train_simulations.len() - 1
    }

    pub fn drive_train_simulation(&self, index: usize) -> &SwerveDriveSimulation {
        &self.drive_train_simulations[index]
    }

    pub fn drive_train_simulation_mut(&mut self, index: usize) -> &mut SwerveDriveSimulation {
        &mut self.drive_train_simulations[index]
    }

    pub fn battery_voltage(&self) -> f64 {
        self.battery.voltage()
    }

    pub fn obstacle_count(&self) -> usize {
        self.world.obstacles().len()
    }

    pub fn simulation_periodic(&mut self) {
        for _ in 0..self.sub_ticks_in_1_period {
            self.simulation_sub_tick();
        }
    }

    fn simulation_sub_tick(&mut self) {
        let battery_voltage = self.battery.voltage();
        let total_current: f64 = self
            .drive_train_simulations
            .iter()
            .map(|d| d.total_supply_current(battery_voltage))
            .sum();
        self.battery.simulation_sub_tick(total_current);

        for drive in &mut self.drive_train_simulations {
            drive.simulation_sub_tick(self.simulation_dt, &self.battery);
        }

        let mut bodies: Vec<&mut RigidBody> = self
            .drive_train_simulations
            .iter_mut()
            .map(SwerveDriveSimulation::body_mut)
            .collect();
        self.world.step(&mut bodies, self.simulation_dt);
    }
}
