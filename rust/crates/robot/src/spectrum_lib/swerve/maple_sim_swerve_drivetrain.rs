//! Port of `frc.spectrumLib.swerve.MapleSimSwerveDrivetrain`: injects the maple-sim drivetrain
//! into the Phoenix devices of the swerve modules.
//!
//! Java source: maple-sim's CTRE swerve template (MIT, Copyright 2021-2025 Iron Maple 5516).
//! `regulateModuleConstantsForSimulation` lives in `robot::subsystems::swerve::swerve`, next to
//! its only caller.

use super::maple_sim::{
    DriveTrainSimulationConfig, RoboRioSim, SimulatedArena, SimulatedBattery,
    SimulatedMotorController, SwerveDriveSimulation, SwerveModuleSimulationConfig,
};
use crate::rebuilt::robot_bump_sim::RobotBumpSim;
use crate::robot::subsystems::swerve::swerve::SwerveSimBackend;
use phoenix6::Pigeon2;
use phoenix6::sim::{CANcoderSimState, Pigeon2SimState, TalonFXSimState};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use swerve::{ModuleConstants, SwerveModule};
use wpilib::notifier::Notifier;
use wpilib::shutdown;
use wpimath::geometry::{Pose2d, Pose3d, Translation2d};
use wpimath::kinematics::ChassisSpeeds;
use wpimath::system::DcMotor;
use wpimath::units::radians_to_rotations;

/// Where the drivetrain writes the yaw and yaw rate of the Pigeon2 (`Pigeon2SimState`).
pub trait PigeonSim: Send {
    fn set_yaw_and_rate(&self, raw_yaw_degrees: f64, angular_velocity_z_dps: f64);
}

impl PigeonSim for Pigeon2SimState {
    fn set_yaw_and_rate(&self, raw_yaw_degrees: f64, angular_velocity_z_dps: f64) {
        self.set_raw_yaw(raw_yaw_degrees);
        self.set_angular_velocity_z(angular_velocity_z_dps);
    }
}

wpilib::native_lib! {
    struct HalRoboRio = "wpiHal" {
        fn HALSIM_SetRoboRioVInVoltage(v_in_voltage: f64);
        fn HALSIM_GetRoboRioBrownoutVoltage() -> f64;
    }
}

/// `RoboRioSim` and `SmartDashboard` through the HAL simulation.
struct HalRoboRioSim(HalRoboRio);

impl HalRoboRioSim {
    fn load() -> Self {
        // SAFETY: signatures from the 2026.2.1 HAL sim headers (`RoboRioData.h`).
        let lib = unsafe { HalRoboRio::load() }.unwrap_or_else(|e| {
            panic!(
                "{}",
                wpilib::native::load_failure(&HalRoboRio::file(), &e, "WPILib")
            )
        });
        Self(lib)
    }
}

impl RoboRioSim for HalRoboRioSim {
    fn set_vin_voltage(&mut self, volts: f64) {
        // SAFETY: plain value argument.
        unsafe { (self.0.HALSIM_SetRoboRioVInVoltage)(volts) };
    }

    fn brownout_voltage(&self) -> f64 {
        // SAFETY: no arguments.
        unsafe { (self.0.HALSIM_GetRoboRioBrownoutVoltage)() }
    }

    fn publish(&mut self, total_current_amps: f64, battery_volts: f64) {
        wpilib::dashboard::smart_dashboard::put_number(
            "BatterySim/TotalCurrent (Amps)",
            total_current_amps,
        );
        wpilib::dashboard::smart_dashboard::put_number(
            "BatterySim/BatteryVoltage (Volts)",
            battery_volts,
        );
    }
}

/// Injects maple-sim data into the Pigeon2 and the module devices of a CTRE swerve drivetrain.
pub struct MapleSimSwerveDrivetrain {
    pigeon_sim: Box<dyn PigeonSim>,
    arena: SimulatedArena,
}

impl MapleSimSwerveDrivetrain {
    /// Builds the sim starting at pose zero, with its own Rebuilt arena. `module_locations` are
    /// in the order FL, FR, BL, BR. Java reads the shared module constants from `[0]`, so the
    /// gear ratios, friction voltages, wheel radius and steer inertia are the same for all four.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        sim_period: f64,
        robot_mass_with_bumpers: f64,
        bumper_length_x: f64,
        bumper_width_y: f64,
        drive_motor_model: DcMotor,
        steer_motor_model: DcMotor,
        wheel_cof: f64,
        module_locations: &[Translation2d; 4],
        pigeon: &Pigeon2,
        modules: &[SwerveModule; 4],
        module_constants: &[ModuleConstants; 4],
    ) -> Self {
        let mut drivetrain = Self::with_parts(
            sim_period,
            robot_mass_with_bumpers,
            bumper_length_x,
            bumper_width_y,
            drive_motor_model,
            steer_motor_model,
            wheel_cof,
            module_locations,
            &module_constants[0],
            Box::new(pigeon.sim_state()),
            Box::new(HalRoboRioSim::load()),
        );
        let drive = drivetrain.maple_sim_drive_mut();
        for (module_simulation, module) in drive.modules_mut().iter_mut().zip(modules) {
            module_simulation.use_drive_motor_controller(Box::new(TalonFxMotorControllerSim::new(
                module.drive_motor().sim_state(),
                module.drive_motor().device_id(),
            )));
            module_simulation.use_steer_motor_controller(Box::new(
                TalonFxMotorControllerWithRemoteCanCoderSim::new(
                    module.steer_motor().sim_state(),
                    module.steer_motor().device_id(),
                    module.encoder().sim_state(),
                    module.encoder().device_id(),
                ),
            ));
        }
        drivetrain
    }

    /// The constructor of Java without the Phoenix wiring: the modules keep the generic motor
    /// controllers maple-sim gives them.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn with_parts(
        sim_period: f64,
        robot_mass_with_bumpers: f64,
        bumper_length_x: f64,
        bumper_width_y: f64,
        drive_motor_model: DcMotor,
        steer_motor_model: DcMotor,
        wheel_cof: f64,
        module_locations: &[Translation2d; 4],
        module_constants: &ModuleConstants,
        pigeon_sim: Box<dyn PigeonSim>,
        roborio: Box<dyn RoboRioSim>,
    ) -> Self {
        let module_config = SwerveModuleSimulationConfig::new(
            drive_motor_model,
            steer_motor_model,
            module_constants.drive_motor_gear_ratio,
            module_constants.steer_motor_gear_ratio,
            module_constants.drive_friction_voltage,
            module_constants.steer_friction_voltage,
            module_constants.wheel_radius_m,
            module_constants.steer_inertia,
            wheel_cof,
        );
        let simulation_config = DriveTrainSimulationConfig::new(
            robot_mass_with_bumpers,
            bumper_length_x,
            bumper_width_y,
            module_locations.to_vec(),
            module_config,
        );
        let drive = SwerveDriveSimulation::new(simulation_config, Pose2d::ZERO);

        // overrideSimulationTimings(simPeriod, 1), then a fresh arena as the instance
        let mut arena =
            SimulatedArena::rebuilt_2026(false, SimulatedBattery::new(roborio), sim_period, 1);
        arena.add_drive_train_simulation(drive);
        Self { pigeon_sim, arena }
    }

    /// `mapleSimDrive`.
    pub fn maple_sim_drive(&self) -> &SwerveDriveSimulation {
        self.arena.drive_train_simulation(0)
    }

    pub fn maple_sim_drive_mut(&mut self) -> &mut SwerveDriveSimulation {
        self.arena.drive_train_simulation_mut(0)
    }

    pub fn battery_voltage(&self) -> f64 {
        self.arena.battery_voltage()
    }

    /// Steps the arena and copies the resulting yaw and yaw rate into the Pigeon2's sim state.
    pub fn update(&mut self) {
        self.arena.simulation_periodic();
        let drive = self.maple_sim_drive();
        let yaw_degrees = drive.simulated_pose().rotation.degrees();
        let angular_velocity_dps = drive.chassis_speeds_robot_relative().omega.to_degrees();
        self.pigeon_sim
            .set_yaw_and_rate(yaw_degrees, angular_velocity_dps);
    }
}

/// `TalonFXMotorControllerSim`: feeds the rotor state of maple-sim into a TalonFX sim state and
/// returns the voltage the motor asks for.
pub struct TalonFxMotorControllerSim {
    pub id: i32,
    talon_fx_sim_state: TalonFXSimState,
}

impl TalonFxMotorControllerSim {
    pub fn new(talon_fx_sim_state: TalonFXSimState, id: i32) -> Self {
        Self {
            id,
            talon_fx_sim_state,
        }
    }
}

impl SimulatedMotorController for TalonFxMotorControllerSim {
    /// The mechanism side arguments are ignored: this path only writes the rotor side.
    fn update_control_signal(
        &mut self,
        _mechanism_angle: f64,
        _mechanism_velocity: f64,
        encoder_angle: f64,
        encoder_velocity: f64,
        battery_voltage: f64,
    ) -> f64 {
        self.talon_fx_sim_state
            .set_raw_rotor_position(radians_to_rotations(encoder_angle));
        self.talon_fx_sim_state
            .set_rotor_velocity(radians_to_rotations(encoder_velocity));
        self.talon_fx_sim_state.set_supply_voltage(battery_voltage);
        self.talon_fx_sim_state.motor_voltage()
    }
}

/// `TalonFXMotorControllerWithRemoteCanCoderSim`: the steer adapter that also drives the remote
/// CANcoder's sim state.
pub struct TalonFxMotorControllerWithRemoteCanCoderSim {
    base: TalonFxMotorControllerSim,
    pub encoder_id: i32,
    remote_cancoder_sim_state: CANcoderSimState,
}

impl TalonFxMotorControllerWithRemoteCanCoderSim {
    pub fn new(
        talon_fx_sim_state: TalonFXSimState,
        talon_fx_id: i32,
        remote_cancoder_sim_state: CANcoderSimState,
        encoder_id: i32,
    ) -> Self {
        Self {
            base: TalonFxMotorControllerSim::new(talon_fx_sim_state, talon_fx_id),
            encoder_id,
            remote_cancoder_sim_state,
        }
    }
}

impl SimulatedMotorController for TalonFxMotorControllerWithRemoteCanCoderSim {
    /// Writes the mechanism side state to the CANcoder, which sits on that side, then lets the
    /// base handle the rotor side and the returned voltage.
    fn update_control_signal(
        &mut self,
        mechanism_angle: f64,
        mechanism_velocity: f64,
        encoder_angle: f64,
        encoder_velocity: f64,
        battery_voltage: f64,
    ) -> f64 {
        self.remote_cancoder_sim_state
            .set_supply_voltage(battery_voltage);
        self.remote_cancoder_sim_state
            .set_raw_position(radians_to_rotations(mechanism_angle));
        self.remote_cancoder_sim_state
            .set_velocity(radians_to_rotations(mechanism_velocity));
        self.base.update_control_signal(
            mechanism_angle,
            mechanism_velocity,
            encoder_angle,
            encoder_velocity,
            battery_voltage,
        )
    }
}

/// What Java's `simLock` protects: the drivetrain sim and the bump sim.
struct SimWorld {
    drivetrain: MapleSimSwerveDrivetrain,
    robot_bump_sim: RobotBumpSim,
}

fn lock(world: &Mutex<SimWorld>) -> MutexGuard<'_, SimWorld> {
    world.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Stops the sim thread and releases its HAL notifier. Safe to call twice.
fn stop_notifier(slot: &Mutex<Option<Notifier>>) {
    let notifier = slot.lock().unwrap_or_else(PoisonError::into_inner).take();
    drop(notifier);
}

/// The simulated drivetrain behind `Swerve`: `MapleSimSwerveDrivetrain` and `RobotBumpSim`
/// under one lock, stepped by a notifier thread.
pub struct MapleSimBackend {
    world: Arc<Mutex<SimWorld>>,
    notifier: Arc<Mutex<Option<Notifier>>>,
}

impl MapleSimBackend {
    pub fn new(drivetrain: MapleSimSwerveDrivetrain, robot_bump_sim: RobotBumpSim) -> Self {
        Self {
            world: Arc::new(Mutex::new(SimWorld {
                drivetrain,
                robot_bump_sim,
            })),
            notifier: Arc::new(Mutex::new(None)),
        }
    }

    /// Java's `simNotifier.startPeriodic(config.getSimLoopPeriod())`.
    ///
    /// # Panics
    /// If the HAL cannot create the notifier, where Java throws.
    pub fn start_notifier(&self, period_seconds: f64) {
        let world = Arc::clone(&self.world);
        let notifier = Notifier::new(move || lock(&world).drivetrain.update())
            .unwrap_or_else(|e| panic!("Could not create the swerve sim notifier: {e}"));
        notifier.start_periodic(period_seconds);
        let previous = self
            .notifier
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .replace(notifier);
        drop(previous);
        let weak = Arc::downgrade(&self.notifier);
        shutdown::register(move || {
            if let Some(slot) = weak.upgrade() {
                stop_notifier(&slot);
            }
        });
    }

    /// One `mapleSimSwerveDrivetrain.update()` under the lock, as the notifier does.
    pub fn update(&self) {
        lock(&self.world).drivetrain.update();
    }
}

impl SwerveSimBackend for MapleSimBackend {
    fn simulated_pose(&self) -> Pose2d {
        lock(&self.world)
            .drivetrain
            .maple_sim_drive()
            .simulated_pose()
    }

    fn set_simulation_world_pose(&self, pose: Pose2d) {
        lock(&self.world)
            .drivetrain
            .maple_sim_drive_mut()
            .set_simulation_world_pose(pose);
    }

    fn update_bump_sim(&self) -> Option<Pose3d> {
        let mut world = lock(&self.world);
        let sim_pose = world.drivetrain.maple_sim_drive().simulated_pose();
        let robot_rel_speeds = world
            .drivetrain
            .maple_sim_drive()
            .chassis_speeds_robot_relative();
        let field_rel_speeds: ChassisSpeeds =
            ChassisSpeeds::from_robot_relative(robot_rel_speeds, sim_pose.rotation);
        // 5 subticks gives 4 ms sub-steps, close to MapleSim's 5 ms period
        let pose3d = world.robot_bump_sim.update(sim_pose, field_rel_speeds, 5);
        if world.robot_bump_sim.is_on_ramp() {
            let world_pose = world.robot_bump_sim.get_sim_world_pose(sim_pose);
            world
                .drivetrain
                .maple_sim_drive_mut()
                .set_simulation_world_pose(world_pose);
        }
        Some(pose3d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use golden::close;
    use wpimath::units::{inches_to_meters, lbs_to_kilograms};

    #[test]
    fn stopping_an_empty_notifier_slot_twice_is_fine() {
        let slot = Mutex::new(None);
        stop_notifier(&slot);
        stop_notifier(&slot);
        assert!(slot.lock().unwrap().is_none());
    }

    const PERIOD: f64 = 0.005;
    const TRACK: f64 = 0.5334;

    struct FixedRoboRio;

    impl RoboRioSim for FixedRoboRio {
        fn set_vin_voltage(&mut self, _volts: f64) {}

        fn brownout_voltage(&self) -> f64 {
            6.75
        }

        fn publish(&mut self, _total_current_amps: f64, _battery_volts: f64) {}
    }

    #[derive(Clone, Default)]
    struct RecordedPigeon(Arc<Mutex<Vec<(f64, f64)>>>);

    impl PigeonSim for RecordedPigeon {
        fn set_yaw_and_rate(&self, raw_yaw_degrees: f64, angular_velocity_z_dps: f64) {
            self.0
                .lock()
                .unwrap()
                .push((raw_yaw_degrees, angular_velocity_z_dps));
        }
    }

    fn module_constants() -> ModuleConstants {
        ModuleConstants {
            drive_motor_gear_ratio: 6.03,
            steer_motor_gear_ratio: 21.428571428571427,
            wheel_radius_m: 0.0499,
            drive_friction_voltage: 0.1,
            steer_friction_voltage: 0.05,
            steer_inertia: 0.05,
            ..ModuleConstants::default()
        }
    }

    fn locations() -> [Translation2d; 4] {
        [
            Translation2d::new(TRACK / 2.0, TRACK / 2.0),
            Translation2d::new(TRACK / 2.0, -TRACK / 2.0),
            Translation2d::new(-TRACK / 2.0, TRACK / 2.0),
            Translation2d::new(-TRACK / 2.0, -TRACK / 2.0),
        ]
    }

    fn drivetrain(pigeon: RecordedPigeon) -> MapleSimSwerveDrivetrain {
        MapleSimSwerveDrivetrain::with_parts(
            PERIOD,
            lbs_to_kilograms(115.0),
            inches_to_meters(30.0),
            inches_to_meters(30.0),
            DcMotor::kraken_x60_foc(1),
            DcMotor::kraken_x60_foc(1),
            1.2,
            &locations(),
            &module_constants(),
            Box::new(pigeon),
            Box::new(FixedRoboRio),
        )
    }

    fn request(drivetrain: &mut MapleSimSwerveDrivetrain, drive: [f64; 4], steer: [f64; 4]) {
        for (i, module) in drivetrain
            .maple_sim_drive_mut()
            .modules_mut()
            .iter_mut()
            .enumerate()
        {
            module
                .drive_motor_controller_mut()
                .request_voltage(drive[i]);
            module
                .steer_motor_controller_mut()
                .request_voltage(steer[i]);
        }
    }

    fn bump_sim() -> RobotBumpSim {
        RobotBumpSim::new(locations())
    }

    #[test]
    fn driving_forward_for_two_seconds_moves_the_pose_along_x() {
        let backend = MapleSimBackend::new(drivetrain(RecordedPigeon::default()), bump_sim());
        backend.set_simulation_world_pose(Pose2d::from_xy(
            1.0,
            2.0,
            wpimath::geometry::Rotation2d::new(0.0),
        ));
        {
            let mut world = lock(&backend.world);
            request(&mut world.drivetrain, [12.0; 4], [0.0; 4]);
        }
        for _ in 0..400 {
            backend.update();
        }
        let pose = backend.simulated_pose();
        // Free speed is 5.0 m/s, and the battery sags under the load
        assert!(pose.x() - 1.0 > 3.0 && pose.x() - 1.0 < 10.0, "{pose:?}");
        assert!((pose.y() - 2.0).abs() < 1e-6, "{pose:?}");
        assert!(pose.rotation.radians().abs() < 1e-6, "{pose:?}");
    }

    #[test]
    fn update_copies_yaw_and_rate_to_the_pigeon() {
        let pigeon = RecordedPigeon::default();
        let mut sim = drivetrain(pigeon.clone());
        sim.maple_sim_drive_mut()
            .set_simulation_world_pose(Pose2d::from_xy(
                2.0,
                6.0,
                wpimath::geometry::Rotation2d::new(0.4),
            ));
        sim.update();
        let recorded = pigeon.0.lock().unwrap();
        let (yaw, rate) = recorded[0];
        assert!((yaw - 0.4_f64.to_degrees()).abs() < 1e-6, "{yaw}");
        assert!(rate.abs() < 1e-6, "{rate}");
    }

    #[test]
    fn bump_sim_stays_off_the_ramp_on_open_floor() {
        let backend = MapleSimBackend::new(drivetrain(RecordedPigeon::default()), bump_sim());
        backend.set_simulation_world_pose(Pose2d::from_xy(
            2.0,
            2.0,
            wpimath::geometry::Rotation2d::new(0.0),
        ));
        let pose3d = backend.update_bump_sim().unwrap();
        assert!((pose3d.x() - 2.0).abs() < 1e-6);
        assert!((pose3d.y() - 2.0).abs() < 1e-6);
        assert!(!lock(&backend.world).robot_bump_sim.is_on_ramp());
    }

    #[test]
    fn rebuilt_arena_has_the_java_obstacles() {
        let sim = drivetrain(RecordedPigeon::default());
        // 4 borders, 4 trench walls, 2 tower poles, 2 hubs
        assert_eq!(sim.arena.obstacle_count(), 12);
    }

    /// Replays the scenarios of `MapleSimFixtures.java`. Free driving matches Java closely.
    /// Collisions differ by dyn4j's GJK, EPA and constraint order, so they get a looser bound.
    fn run_scenario(input: &[f64]) -> Vec<f64> {
        let mut sim = drivetrain(RecordedPigeon::default());
        sim.maple_sim_drive_mut()
            .set_simulation_world_pose(Pose2d::from_xy(
                input[0],
                input[1],
                wpimath::geometry::Rotation2d::new(input[2]),
            ));
        let phases = input[3] as usize;
        let mut out = Vec::new();
        let mut step = 0;
        for p in 0..phases {
            let phase = &input[4 + p * 9..4 + (p + 1) * 9];
            request(
                &mut sim,
                [phase[1], phase[2], phase[3], phase[4]],
                [phase[5], phase[6], phase[7], phase[8]],
            );
            for _ in 0..phase[0] as usize {
                sim.update();
                step += 1;
                if step % 10 == 0 {
                    let drive = sim.maple_sim_drive();
                    let pose = drive.simulated_pose();
                    let speeds = drive.chassis_speeds_field_relative();
                    out.extend([
                        pose.x(),
                        pose.y(),
                        pose.rotation.radians(),
                        speeds.vx,
                        speeds.vy,
                        speeds.omega,
                    ]);
                    out.extend(drive.modules().iter().map(|m| m.current_state().speed));
                    out.extend(drive.modules().iter().map(|m| m.steer_absolute_angle()));
                    out.push(sim.battery_voltage());
                }
            }
        }
        out
    }

    fn compare_scenarios(ops: &[&str], tolerance: f64) {
        for case in golden::load("maple_swerve") {
            if !ops.contains(&case.op.as_str()) {
                continue;
            }
            let got = run_scenario(&case.input);
            assert_eq!(got.len(), case.out.len(), "{}", case.op);
            for (i, (g, w)) in got.iter().zip(&case.out).enumerate() {
                assert!(
                    close(*g, *w, tolerance),
                    "{} sample {} value {}: java {w}, rust {g}",
                    case.op,
                    i / 15,
                    i % 15
                );
            }
        }
    }

    #[test]
    fn matches_java_for_free_driving_and_a_trench_wall_hit() {
        compare_scenarios(
            &["straight", "spin", "arc", "wall_back", "hub_oblique"],
            1e-9,
        );
    }

    /// Contacts whose depth lands on zero keep or drop a point by the rounding of that depth, so
    /// Rust and Java agree to 1e-6 and 1e-2 on these two and then part by a few millimeters.
    #[test]
    fn matches_java_through_corner_and_head_on_hub_hits() {
        compare_scenarios(&["corner"], 1e-5);
        compare_scenarios(&["hub_head_on"], 2e-2);
    }

    /// The pole hit matches to 1e-9 for 85 steps. At step 86 the one contact left has depth
    /// 0.0 in Rust and a hair below in Java, and the robot ends 3.5 mm and 0.004 rad apart.
    #[test]
    fn matches_java_until_the_pole_contact_ends_then_stays_close() {
        let case = golden::load("maple_swerve")
            .into_iter()
            .find(|c| c.op == "pole")
            .unwrap();
        let got = run_scenario(&case.input);
        assert_eq!(got.len(), case.out.len());
        for (i, (g, w)) in got.iter().zip(&case.out).enumerate().take(8 * 15) {
            assert!(
                close(*g, *w, 1e-9),
                "sample {} value {}: java {w}, rust {g}",
                i / 15,
                i % 15
            );
        }
        let last = got.len() - 15;
        assert!((got[last] - case.out[last]).abs() < 0.01);
        assert!((got[last + 1] - case.out[last + 1]).abs() < 0.01);
        assert!((got[last + 2] - case.out[last + 2]).abs() < 0.01);
    }
}
