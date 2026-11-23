use crate::context;
use crate::rebuilt::shift_helpers;
use crate::rebuilt::shot_calculator::{self, ShotCalculator};
use crate::robot::auton::auton::{self, Auton};
use crate::robot::build_constants;
use crate::robot::configs::{fm2026::fm2026, photon2026::photon2026, pm2026::pm2026};
use crate::robot::operator::operator::{Operator, OperatorButtons, OperatorConfig};
use crate::robot::pilot::pilot::{Pilot, PilotButtons, PilotConfig};
use crate::robot::robot_sim::{self, RobotSim};
use crate::robot::subsystems::fuel_intake::{FuelIntake, FuelIntakeConfig};
use crate::robot::subsystems::hood::{Hood, HoodConfig};
use crate::robot::subsystems::indexer_bed::{IndexerBed, IndexerBedConfig};
use crate::robot::subsystems::indexer_tower::{IndexerTower, IndexerTowerConfig};
use crate::robot::subsystems::intake_extension::{IntakeExtension, IntakeExtensionConfig};
use crate::robot::subsystems::launcher::{Launcher, LauncherConfig};
use crate::robot::subsystems::super_structure::{SuperStructure, WantedSuperState};
use crate::robot::subsystems::swerve::swerve::{PilotDriveInputs, Swerve};
use crate::robot::subsystems::swerve::swerve_config::SwerveConfig;
use crate::robot::subsystems::vision::{Vision, VisionConfig};
use crate::spectrum_lib::framework::robot_loop;
use crate::spectrum_lib::framework::spectrum_robot::timed_robot;
use crate::spectrum_lib::hardware::rio::Rio;
use crate::spectrum_lib::sim::sim_loop;
use crate::spectrum_lib::util::crash_tracker;
use crate::spectrum_lib::util::util;
use ::telemetry::{BatteryLogger, SystemLoadMonitor};
use commands::{Command, CommandScheduler, Commands, RobotState, Subsystem, SubsystemId};
use pathplanner::{
    AutoBuilder, FollowPathCommand, PathPlannerAuto, PathPlannerPath, PathfindingCommand,
};
use phoenix6::{CanBus, utils};
use std::cell::Cell;
use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::rc::Rc;
use telemetry::{PrintPriority, Telemetry, telemetry};
use wpilib::dashboard::{self, Field2d, smart_dashboard};
use wpilib::hooks::{self, RobotMode};
use wpilib::robot::{Robot as WpilibRobot, TimedRobot};
use wpilib::web_server;
use wpilib::{driver_station, filesystem, nt, robot_controller, timer};
use wpimath::geometry::{Pose2d, Rotation2d};

#[derive(Default)]
pub struct Config {
    pub swerve: SwerveConfig,
    pub pilot: PilotConfig,
    pub operator: OperatorConfig,
    pub fuel_intake: FuelIntakeConfig,
    pub intake_extension: IntakeExtensionConfig,
    pub indexer_tower: IndexerTowerConfig,
    pub indexer_bed: IndexerBedConfig,
    pub launcher: LauncherConfig,
    pub hood: HoodConfig,
    pub vision: VisionConfig,
}

/// The config for the roboRIO this runs on.
pub fn config_for(rio: Rio) -> Config {
    match rio {
        Rio::Photon2026 => photon2026(),
        Rio::Pm2026 => pm2026(),
        _ => fm2026(), // SIM and UNKNOWN
    }
}

/// What the pilot and operator bindings start. `Robot` backs it with the real subsystems,
/// and tests back it with recorders.
pub trait BindingTargets {
    fn set_state(&self, state: WantedSuperState) -> Command;
    fn reorient_forward(&self) -> Command;
    fn reorient_left(&self) -> Command;
    fn reorient_back(&self) -> Command;
    fn reorient_right(&self) -> Command;
    fn coast_mechanisms(&self) -> Command;
    fn brake_mechanisms(&self) -> Command;
    fn reset_intake_extension_to_max(&self) -> Command;
    fn operator_rumble(&self) -> Command;
}

struct RobotTargets {
    super_structure: Rc<SuperStructure>,
    swerve: Rc<Swerve>,
    operator: Rc<Operator>,
}

impl BindingTargets for RobotTargets {
    fn set_state(&self, state: WantedSuperState) -> Command {
        self.super_structure.set_state_command(state)
    }

    fn reorient_forward(&self) -> Command {
        self.swerve.reorient_forward()
    }

    fn reorient_left(&self) -> Command {
        self.swerve.reorient_left()
    }

    fn reorient_back(&self) -> Command {
        self.swerve.reorient_back()
    }

    fn reorient_right(&self) -> Command {
        self.swerve.reorient_right()
    }

    fn coast_mechanisms(&self) -> Command {
        self.super_structure.coast_mechanisms()
    }

    fn brake_mechanisms(&self) -> Command {
        self.super_structure.brake_mechanisms()
    }

    fn reset_intake_extension_to_max(&self) -> Command {
        self.super_structure
            .intake_extension
            .reset_current_position_to_max_command()
    }

    fn operator_rumble(&self) -> Command {
        self.operator.rumble_command_uniform(1.0, 0.5)
    }
}

/// Java's `Telemetry` is a subsystem. The scheduler calls its `periodic` once per loop.
struct TelemetrySubsystem {
    id: SubsystemId,
    telemetry: Telemetry,
}

impl Subsystem for TelemetrySubsystem {
    fn id(&self) -> SubsystemId {
        self.id
    }

    fn name(&self) -> String {
        "Telemetry".to_owned()
    }

    fn periodic(&self) {
        self.telemetry.periodic();
    }
}

/// The robot mode as the driver station reports it, for the `commands` crate.
struct DriverStationState;

impl RobotState for DriverStationState {
    fn is_disabled(&self) -> bool {
        hooks::robot_mode() == RobotMode::Disabled
    }

    fn is_autonomous_enabled(&self) -> bool {
        hooks::robot_mode() == RobotMode::Autonomous
    }

    fn is_teleop_enabled(&self) -> bool {
        hooks::robot_mode() == RobotMode::Teleop
    }

    fn is_test_enabled(&self) -> bool {
        hooks::robot_mode() == RobotMode::Test
    }

    fn is_simulation(&self) -> bool {
        wpilib::is_simulation()
    }
}

/// Robot entry point. Picks the config for the Rio serial it is running on, then builds the
/// gamepads and mechanisms.
pub struct Robot {
    robot_sim: Option<RobotSim>,
    field2d: Field2d,
    auton_warmed_up: Rc<Cell<bool>>,

    swerve: Rc<Swerve>,
    operator: Rc<Operator>,
    pilot: Rc<Pilot>,
    vision: Rc<Vision>,
    auton: Auton,
    super_structure: Rc<SuperStructure>,
    main_can_bus: CanBus,
    system_load: SystemLoadMonitor,

    auto_name: String,
}

/// Builds the robot the way Java's `RobotBase.startRobot(Robot::new)` does: the commands
/// crate gets WPILib's clock and robot mode, the NetworkTables server starts, `Robot::new`
/// runs, and the dashboard updates join the loop. Call on the thread that runs the loop.
pub fn new_timed_robot() -> TimedRobot<Robot> {
    commands::env::set_clock(hooks::time_source);
    commands::env::set_robot_state(DriverStationState);
    nt::start_default_server();
    // Java's SpectrumRobot constructor does this before the Robot constructor body, and
    // binding a trigger reads its axis.
    driver_station::silence_joystick_connection_warning(true);

    let mut timed = timed_robot(Robot::new());
    dashboard::install(&mut timed);
    timed
}

/// Logs a panic like Java's `CrashTracker.logThrowableCrash` and lets it continue.
fn guarded<T>(body: impl FnOnce() -> T) -> T {
    match catch_unwind(AssertUnwindSafe(body)) {
        Ok(value) => value,
        Err(payload) => {
            let message = payload
                .downcast_ref::<&str>()
                .map(|s| (*s).to_owned())
                .or_else(|| payload.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "unknown panic".to_owned());
            crash_tracker::log_throwable_crash(&message);
            resume_unwind(payload)
        }
    }
}

impl Robot {
    pub fn new() -> Robot {
        let telemetry_subsystem = Rc::new(TelemetrySubsystem {
            id: SubsystemId::new(),
            telemetry: Telemetry::new(),
        });
        CommandScheduler::get_instance().register_subsystem(telemetry_subsystem);
        let field2d = Field2d::new();
        let system_load = SystemLoadMonitor::new();
        context::set_field2d(field2d.clone());
        context::set_left_view(robot_sim::new_left_view());
        context::set_auton_pose_update(auton::auton_pose_update());
        context::set_auton_launching((*auton::auton_launching()).clone());

        telemetry::start(
            wpilib::is_simulation(),
            true,
            false,
            true,
            false,
            true,
            PrintPriority::Normal,
        );

        let robot = guarded(|| Self::init(field2d, system_load));

        if let Err(error) = robot_controller::set_brownout_voltage(4.6) {
            error.report();
        }

        telemetry::log_dash_always("BuildConstants/ProjectName", build_constants::MAVEN_NAME);
        telemetry::log_dash_always("BuildConstants/BuildDate", build_constants::BUILD_DATE);
        telemetry::log_dash_always("BuildConstants/GitSHA", build_constants::GIT_SHA);
        telemetry::log_dash_always("BuildConstants/GitDate", build_constants::GIT_DATE);
        telemetry::log_dash_always("BuildConstants/GitBranch", build_constants::GIT_BRANCH);
        telemetry::log_dash_always(
            "BuildConstants/GitDirty",
            build_constants::dirty_description(),
        );
        robot
    }

    /// The `try` block of Java's constructor.
    fn init(field2d: Field2d, system_load: SystemLoadMonitor) -> Robot {
        telemetry::print("--- Robot Init Starting ---");

        let mut config = config_for(Rio::id());

        let can_init_delay = 0.1; // separates the mechanisms' CAN config writes
        let main_can_bus = CanBus::new(Rio::CANIVORE); // "*" takes the first CANivore found

        let pilot = Pilot::new(&mut config.pilot, &config.swerve);
        context::set_pilot(pilot.clone() as Rc<dyn PilotDriveInputs>);
        let operator = Operator::new(&config.operator);

        let swerve = Swerve::new(config.swerve.clone());
        context::set_swerve(swerve.clone());
        timer::delay(can_init_delay);

        let intake_extension = IntakeExtension::new(config.intake_extension.clone());
        context::set_intake_extension(intake_extension.clone());
        timer::delay(can_init_delay);

        let fuel_intake = FuelIntake::new(config.fuel_intake.clone());
        timer::delay(can_init_delay);

        let hood = Hood::new(config.hood.clone());
        context::set_hood(hood.clone());
        timer::delay(can_init_delay);

        let launcher = Launcher::new(config.launcher.clone());
        timer::delay(can_init_delay);

        let indexer_tower = IndexerTower::new(config.indexer_tower.clone());
        timer::delay(can_init_delay);

        let indexer_bed = IndexerBed::new(config.indexer_bed.clone());
        timer::delay(can_init_delay);

        let super_structure = SuperStructure::new(
            swerve.clone(),
            fuel_intake,
            intake_extension,
            indexer_tower,
            indexer_bed,
            launcher,
            hood,
        );
        context::set_super_structure(super_structure.clone());

        let auton = Auton::new(super_structure.clone());
        let vision = Vision::new(config.vision.clone());
        let battery_logger = Rc::new(BatteryLogger::new());
        context::set_battery_logger(battery_logger.clone());

        let mut robot = Robot {
            robot_sim: None,
            field2d,
            auton_warmed_up: Rc::new(Cell::new(false)),
            swerve,
            operator,
            pilot,
            vision,
            auton,
            super_structure,
            main_can_bus,
            system_load,
            auto_name: String::new(),
        };

        if utils::is_simulation() {
            robot.robot_sim = Some(RobotSim::new(robot.super_structure.clone()));
            robot.configure_sim_bindings();
        }

        robot.configure_bindings();

        battery_logger.set_enabled(true);

        telemetry::print("--- Robot Init Complete ---");
        robot
    }

    pub fn configure_bindings(&self) {
        bind_controls(
            &self.pilot.buttons,
            &self.operator.buttons,
            &RobotTargets {
                super_structure: self.super_structure.clone(),
                swerve: self.swerve.clone(),
                operator: self.operator.clone(),
            },
        );
    }

    pub fn configure_sim_bindings(&self) {
        if let Some(robot_sim) = &self.robot_sim {
            robot_sim::sim_launching().while_true(robot_sim.ball_sim_launch_fuel());
        }
    }

    pub fn setup_smart_dashboard_data(&self) {
        smart_dashboard::put_data("Field2d", &self.field2d);
    }
}

impl Default for Robot {
    fn default() -> Self {
        Self::new()
    }
}

/// `Robot.configureBindings`.
pub fn bind_controls(
    pilot: &PilotButtons,
    operator: &OperatorButtons,
    targets: &dyn BindingTargets,
) {
    use WantedSuperState as State;

    // LT alone intakes. RT wins, so the guard yields when RT is held.
    pilot.lt.on_true(Commands::either(
        targets.set_state(State::IntakeFuel),
        Commands::none(),
        pilot.rt.negate(),
    ));

    // RT alone launches with a squeeze. LT yields, so RT+LT falls through to the binding below.
    pilot.rt.on_true(Commands::either(
        targets.set_state(State::LaunchWithSqueeze),
        Commands::none(),
        pilot.lt.negate(),
    ));

    // RT+LT together launches with the intake held out.
    pilot
        .rt
        .and(&pilot.lt)
        .on_true(targets.set_state(State::LaunchWithoutSqueeze));

    // LT up while RT is held launches with no squeeze delay.
    pilot.lt.on_false(Commands::either(
        targets.set_state(State::LaunchWithSqueezeWithNoDelay),
        Commands::none(),
        &pilot.rt,
    ));

    // RT up while LT is held resumes the intake.
    pilot.rt.on_false(Commands::either(
        targets.set_state(State::IntakeFuel),
        Commands::none(),
        &pilot.lt,
    ));

    // Both triggers up goes idle.
    pilot
        .rt
        .or(&pilot.lt)
        .on_false(targets.set_state(State::Idle));

    pilot
        .lt
        .and(&pilot.lb)
        .on_true(targets.set_state(State::Eject));
    pilot
        .lt
        .and(&pilot.lb)
        .on_false(targets.set_state(State::Idle));

    pilot
        .x_button
        .on_true(targets.set_state(State::TrackTarget));
    pilot.x_button.on_false(targets.set_state(State::Idle));

    pilot.a_button.on_true(targets.set_state(State::Unjam));
    pilot.a_button.on_false(targets.set_state(State::Idle));

    pilot
        .select_button
        .on_true(targets.set_state(State::ForceHome));
    pilot.select_button.on_false(targets.set_state(State::Idle));

    pilot
        .d_pad_up
        .and(&pilot.lb)
        .on_true(targets.reorient_forward());
    pilot
        .d_pad_left
        .and(&pilot.lb)
        .on_true(targets.reorient_left());
    pilot
        .d_pad_down
        .and(&pilot.lb)
        .on_true(targets.reorient_back());
    pilot
        .d_pad_right
        .and(&pilot.lb)
        .on_true(targets.reorient_right());

    util::disabled()
        .and(&pilot.a_button)
        .on_true(targets.coast_mechanisms());
    util::disabled()
        .and(&pilot.b_button)
        .on_true(targets.brake_mechanisms());

    util::disabled()
        .and(&operator.a_button)
        .on_true(targets.coast_mechanisms());
    util::disabled()
        .and(&operator.b_button)
        .on_true(targets.brake_mechanisms());

    operator.lb.and(&operator.y_button).on_true(
        Commands::parallel([
            targets.reset_intake_extension_to_max(),
            targets.operator_rumble(),
        ])
        .ignoring_disable(true),
    );

    operator
        .select_button
        .on_true(targets.set_state(State::ForceHome));
    operator
        .select_button
        .on_false(targets.set_state(State::Idle));

    operator
        .d_pad_down
        .on_true(shot_calculator::decrease_hood_angle_offset());
    operator
        .d_pad_up
        .on_true(shot_calculator::increase_hood_angle_offset());
    operator
        .d_pad_right
        .on_true(shot_calculator::decrease_drive_angle_offset());
    operator
        .d_pad_left
        .on_true(shot_calculator::increase_drive_angle_offset());

    // Reset the shift timer whenever the robot enables.
    util::teleop().on_true(Commands::run_once(shift_helpers::initialize, &[]));
    util::auto_mode().on_true(Commands::run_once(shift_helpers::initialize, &[]));
    util::disabled()
        .on_true(Commands::run_once(shift_helpers::initialize, &[]).ignoring_disable(true));

    auton::auton_intake().on_true(targets.set_state(State::AutonIntakeFuel));
    auton::auton_shot_prep().on_true(targets.set_state(State::AutonTrackTarget));
    auton::auton_unjam().on_true(Commands::sequence([
        targets.set_state(State::Unjam),
        Commands::wait_seconds(1.0),
        targets.set_state(State::LaunchWithSqueeze),
    ]));
    auton::auton_clear_state().on_true(targets.set_state(State::Idle));
}

impl WpilibRobot for Robot {
    fn robot_init(&mut self) {
        self.setup_smart_dashboard_data();

        // Build the ShotCalculator now so its Hub Model Chooser is on the dashboard before
        // enabling.
        context::set_shot_calculator(Rc::new(ShotCalculator::new()));

        web_server::start(5800, filesystem::deploy_directory());
    }

    /// Runs the command scheduler. WPILib calls this every 20 ms, and nothing in the command
    /// framework moves without it.
    fn robot_periodic(&mut self) {
        robot_loop::next();
        self.system_load.periodic();
        guarded(|| {
            telemetry::time("Scheduler/robotPeriodic");
            CommandScheduler::get_instance().run();

            telemetry::log_dash_with_unit(
                "Match Data/MatchTime",
                driver_station::match_time(),
                "seconds",
            );
            telemetry::log_dash(
                "Match Data/InShift",
                shift_helpers::get_official_shift_info().active,
            );
            telemetry::log_dash_with_unit(
                "Match Data/TimeLeftInShift",
                shift_helpers::get_official_shift_info().remaining_time,
                "seconds",
            );

            let battery_logger = context::battery_logger();
            battery_logger.set_battery_voltage(robot_controller::battery_voltage());
            battery_logger.set_rio_current(robot_controller::input_current());
            battery_logger.log_power();

            let can_info = self.main_can_bus.status();
            telemetry::log_with_unit(
                "CANivore/BusUtilization",
                f64::from(can_info.bus_utilization * 100.0),
                "%",
            );
            telemetry::log("CANivore/BusOffCount", can_info.bus_off_count);
            telemetry::log("CANivore/TxFullCount", can_info.tx_full_count);
            telemetry::log("CANivore/ReceiveErrorCounter", can_info.rec);
            telemetry::log("CANivore/TransmitErrorCounter", can_info.tec);

            self.field2d.set_robot_pose(self.swerve.get_robot_pose());

            context::shot_calculator().clear_shooting_parameters();
            telemetry::time_end("Scheduler/robotPeriodic");
        });
    }

    fn disabled_init(&mut self) {
        telemetry::print("### Disabled Init Starting ### ");

        if !self.auton_warmed_up.get() {
            let warmed_up = self.auton_warmed_up.clone();
            let auton_start_command = Commands::sequence([
                FollowPathCommand::warmup_command(),
                PathfindingCommand::warmup_command(),
                Commands::run_once(
                    move || {
                        telemetry::log_dash_always("Initialized", true);
                        warmed_up.set(true);
                    },
                    &[],
                ),
            ])
            .ignoring_disable(true);
            CommandScheduler::get_instance().schedule(&auton_start_command);
        }

        telemetry::print("### Disabled Init Complete ### ");
    }

    fn disabled_periodic(&mut self) {
        let full_auto_name = self.auton.get_autonomous_command().get_name();
        let left_start = !full_auto_name.ends_with(" - Right");
        let mut path_planner_paths: Vec<Rc<PathPlannerPath>> = Vec::new();

        if full_auto_name == "Do Nothing" {
            self.field2d.get_object("Auto Routine").set_poses(&[]);
            self.auto_name = full_auto_name;
            return;
        }

        // The field visualizer keys off the side suffix, so strip it to get the base path name
        let mut base_auto_name = full_auto_name.clone();
        if (base_auto_name.ends_with(" - Left") || base_auto_name.ends_with(" - Right"))
            && let Some(split) = base_auto_name.rfind(" - ")
        {
            base_auto_name.truncate(split);
        }

        // Reload on any name change, whether the auto or the side switched.
        if self.auto_name != full_auto_name {
            self.auto_name = full_auto_name;
            telemetry::log("Auton Warmed Up", false);

            if AutoBuilder::get_all_auto_names().contains(&base_auto_name) {
                match PathPlannerAuto::get_path_group_from_auto_file(&base_auto_name) {
                    Ok(paths) => path_planner_paths = paths,
                    Err(_) => telemetry::print("Could not load path planner paths"),
                }

                if driver_station::alliance() == Some(driver_station::Alliance::Red) {
                    path_planner_paths = path_planner_paths
                        .iter()
                        .map(|path| Rc::new(path.flip_path()))
                        .collect();
                }

                if !left_start {
                    path_planner_paths = path_planner_paths
                        .iter()
                        .map(|path| Rc::new(path.mirror_path()))
                        .collect();
                }

                if let Some(first_path) = path_planner_paths.first() {
                    self.swerve.reset_pose(
                        first_path
                            .get_starting_holonomic_pose()
                            .unwrap_or(Pose2d::ZERO),
                    );

                    let warm_up_path = Commands::sequence([
                        AutoBuilder::follow_path(first_path.clone()).with_timeout(0.5),
                        Commands::run_once(
                            || {
                                telemetry::print_with("Auton Warmed Up", PrintPriority::High);
                                telemetry::log("Auton Warmed Up", true);
                            },
                            &[],
                        ),
                    ])
                    .ignoring_disable(true);
                    CommandScheduler::get_instance().schedule(&warm_up_path);
                } else {
                    telemetry::print(&format!(
                        "Warning: No paths loaded for auto: {base_auto_name}"
                    ));
                }

                let mut poses: Vec<Pose2d> = Vec::new();
                for path in &path_planner_paths {
                    poses.extend(path.get_all_path_points().iter().map(|point| {
                        Pose2d::from_xy(point.position.x, point.position.y, Rotation2d::ZERO)
                    }));
                }
                self.field2d.get_object("Auto Routine").set_poses(&poses);
            } else {
                self.field2d.get_object("Auto Routine").set_poses(&[]);
            }
        }
    }

    fn disabled_exit(&mut self) {
        telemetry::print("### Disabled Exit### ");
    }

    fn autonomous_init(&mut self) {
        telemetry::print("@@@ Auton Init @@@ ");
        if utils::is_simulation()
            && let Some(robot_sim) = &self.robot_sim
        {
            let mut ball_sim = robot_sim.get_ball_sim().borrow_mut();
            ball_sim.clear_balls();
            ball_sim.place_field_balls();
        }
        guarded(|| self.auton.init());
    }

    fn autonomous_periodic(&mut self) {}

    fn autonomous_exit(&mut self) {
        self.auton.exit();
        telemetry::print("@@@ Auton Exit @@@ ");
    }

    fn teleop_init(&mut self) {
        guarded(|| {
            telemetry::print("!!! Teleop Init Starting !!! ");

            self.super_structure
                .set_wanted_super_state(WantedSuperState::Idle);
            self.field2d.get_object("Auto Routine").set_poses(&[]); // clears the visualizer

            telemetry::print("!!! Teleop Init Complete !!! ");
        });
    }

    fn teleop_periodic(&mut self) {}

    fn teleop_exit(&mut self) {
        if driver_station::is_fms_attached() {
            self.vision.trigger_rewind_capture_for_all_cameras();
        }
        telemetry::print("!!! Teleop Exit !!! ");
    }

    fn test_init(&mut self) {
        guarded(|| {
            telemetry::print("~~~ Test Init Starting ~~~ ");

            telemetry::print("~~~ Test Init Complete ~~~ ");
        });
    }

    fn test_periodic(&mut self) {}

    fn test_exit(&mut self) {
        telemetry::print("~~~ Test Exit ~~~ ");
    }

    fn simulation_init(&mut self) {
        telemetry::print("$$$ Simulation Init Starting $$$ ");
        telemetry::print("$$$ Simulation Init Complete $$$ ");
    }

    fn simulation_periodic(&mut self) {
        if let Some(robot_sim) = &self.robot_sim {
            let mut ball_sim = robot_sim.get_ball_sim().borrow_mut();
            ball_sim.tick(); // ticks the physics and publishes ball positions to NT
            robot_sim.update_articulated_mechanisms();
            telemetry::log("Sim/Fuel", ball_sim.get_total_intaked());
        }
        sim_loop::draw_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use commands::Trigger;
    use commands::testing::{FakeClock, FakeRobotState, Mode};
    use std::cell::RefCell;
    use std::collections::HashSet;

    const PHOTON_SERIAL: &str = "032B4BB3";
    const PM_SERIAL: &str = "0329AD07";

    fn config_for_serial(serial: &str) -> Config {
        config_for(Rio::from_serial(serial).unwrap_or(Rio::Unknown))
    }

    fn encoder_offsets(config: &Config) -> [f64; 4] {
        let swerve = &config.swerve;
        [
            swerve.front_left_encoder_offset_rot,
            swerve.front_right_encoder_offset_rot,
            swerve.back_left_encoder_offset_rot,
            swerve.back_right_encoder_offset_rot,
        ]
    }

    #[test]
    fn photon_serial_selects_photon() {
        let config = config_for_serial(PHOTON_SERIAL);
        assert_eq!(
            encoder_offsets(&config),
            [0.127197265625, -0.260009765625, 0.1171875, -0.3427734375]
        );
        assert!(config.indexer_bed.base.attached);
        assert!(config.hood.base.attached);
    }

    #[test]
    fn pm_serial_selects_pm() {
        let config = config_for_serial(PM_SERIAL);
        assert_eq!(
            encoder_offsets(&config),
            [
                -0.312744140625 + 0.5,
                -0.032470703125 + 0.5,
                0.3544921875 - 0.5,
                -0.4765625 + 0.5
            ]
        );
    }

    #[test]
    fn the_simulator_and_unknown_rios_select_fm() {
        let fm = [-0.163818359375, 0.24902, 0.2724609375, -0.31005859375];
        assert_eq!(encoder_offsets(&config_for_serial("")), fm);
        assert_eq!(encoder_offsets(&config_for_serial("DEADBEEF")), fm);
        assert_eq!(encoder_offsets(&config_for(Rio::Fm2025)), fm);
    }

    #[test]
    fn mechanism_attach_flags_follow_each_robot() {
        let fm = config_for(Rio::Sim);
        assert!(fm.fuel_intake.base.attached && fm.hood.base.attached);
        assert!(fm.pilot.base.attached && fm.operator.base.attached);

        let am = crate::robot::configs::am2026::am2026();
        assert!(!am.fuel_intake.base.attached && !am.launcher.base.attached);
        assert!(!am.intake_extension.base.attached && !am.indexer_tower.base.attached);

        let xm = crate::robot::configs::xm2026::xm2026();
        assert!(xm.fuel_intake.base.attached && !xm.intake_extension.base.attached);
    }

    /// Records the label of every command the bindings start.
    struct Recorder {
        log: Rc<RefCell<Vec<String>>>,
    }

    impl Recorder {
        fn command(&self, label: &str, runs_when_disabled: bool) -> Command {
            let (log, label) = (self.log.clone(), label.to_owned());
            Commands::run_once(move || log.borrow_mut().push(label.clone()), &[])
                .ignoring_disable(runs_when_disabled)
        }
    }

    impl BindingTargets for Recorder {
        fn set_state(&self, state: WantedSuperState) -> Command {
            self.command(&format!("{state:?}"), false)
        }

        fn reorient_forward(&self) -> Command {
            self.command("reorient forward", false)
        }

        fn reorient_left(&self) -> Command {
            self.command("reorient left", false)
        }

        fn reorient_back(&self) -> Command {
            self.command("reorient back", false)
        }

        fn reorient_right(&self) -> Command {
            self.command("reorient right", false)
        }

        fn coast_mechanisms(&self) -> Command {
            self.command("coast", true)
        }

        fn brake_mechanisms(&self) -> Command {
            self.command("brake", true)
        }

        fn reset_intake_extension_to_max(&self) -> Command {
            self.command("reset intake extension", true)
        }

        fn operator_rumble(&self) -> Command {
            self.command("operator rumble", true)
        }
    }

    /// Pilot and operator buttons that the test presses by hand.
    struct Harness {
        pressed: Rc<RefCell<HashSet<&'static str>>>,
        log: Rc<RefCell<Vec<String>>>,
        _state: FakeRobotState,
        _clock: FakeClock,
    }

    const PILOT_BUTTONS: [&str; 11] = [
        "lt", "rt", "lb", "a", "b", "x", "select", "up", "down", "left", "right",
    ];
    const OPERATOR_BUTTONS: [&str; 9] = [
        "op lb",
        "op y",
        "op a",
        "op b",
        "op select",
        "op up",
        "op down",
        "op left",
        "op right",
    ];

    impl Harness {
        fn new(mode: Mode) -> Harness {
            let clock = FakeClock::install();
            let state = FakeRobotState::install();
            state.set_mode(mode);
            let pressed = Rc::new(RefCell::new(HashSet::new()));
            let log = Rc::new(RefCell::new(Vec::new()));

            let button = |name: &'static str| {
                let pressed = pressed.clone();
                Trigger::new(move || pressed.borrow().contains(name))
            };
            let off = Trigger::new(|| false);
            let pilot = PilotButtons {
                lt: button("lt"),
                rt: button("rt"),
                lb: button("lb"),
                a_button: button("a"),
                b_button: button("b"),
                x_button: button("x"),
                select_button: button("select"),
                d_pad_up: button("up"),
                d_pad_down: button("down"),
                d_pad_left: button("left"),
                d_pad_right: button("right"),
                ..PilotButtons::filled_with(&off)
            };
            let operator = OperatorButtons {
                lb: button("op lb"),
                y_button: button("op y"),
                a_button: button("op a"),
                b_button: button("op b"),
                select_button: button("op select"),
                d_pad_up: button("op up"),
                d_pad_down: button("op down"),
                d_pad_left: button("op left"),
                d_pad_right: button("op right"),
                ..OperatorButtons::filled_with(&off)
            };
            bind_controls(&pilot, &operator, &Recorder { log: log.clone() });
            Harness {
                pressed,
                log,
                _state: state,
                _clock: clock,
            }
        }

        /// Sets which buttons are held, runs one scheduler pass, and returns what started.
        fn hold(&self, names: &[&'static str]) -> Vec<String> {
            *self.pressed.borrow_mut() = names.iter().copied().collect();
            CommandScheduler::get_instance().run();
            self.log.borrow_mut().drain(..).collect()
        }
    }

    fn teleop() -> Harness {
        Harness::new(Mode::Teleop)
    }

    #[test]
    fn nothing_starts_until_a_button_moves() {
        let harness = teleop();
        assert!(harness.hold(&[]).is_empty());
        assert!(harness.hold(&[]).is_empty());
        assert_eq!(PILOT_BUTTONS.len() + OPERATOR_BUTTONS.len(), 20);
    }

    #[test]
    fn lt_alone_intakes_and_releasing_goes_idle() {
        let harness = teleop();
        assert_eq!(harness.hold(&["lt"]), ["IntakeFuel"]);
        assert_eq!(harness.hold(&[]), ["Idle"]);
    }

    #[test]
    fn rt_alone_launches_with_squeeze_and_releasing_goes_idle() {
        let harness = teleop();
        assert_eq!(harness.hold(&["rt"]), ["LaunchWithSqueeze"]);
        assert_eq!(harness.hold(&[]), ["Idle"]);
    }

    #[test]
    fn rt_then_lt_launches_without_squeeze() {
        let harness = teleop();
        harness.hold(&["rt"]);
        assert_eq!(harness.hold(&["rt", "lt"]), ["LaunchWithoutSqueeze"]);
    }

    #[test]
    fn lt_then_rt_launches_without_squeeze() {
        let harness = teleop();
        harness.hold(&["lt"]);
        assert_eq!(harness.hold(&["lt", "rt"]), ["LaunchWithoutSqueeze"]);
    }

    #[test]
    fn both_triggers_in_one_loop_launch_without_squeeze() {
        let harness = teleop();
        assert_eq!(harness.hold(&["lt", "rt"]), ["LaunchWithoutSqueeze"]);
    }

    #[test]
    fn lt_up_while_rt_is_held_launches_with_no_delay() {
        let harness = teleop();
        harness.hold(&["lt", "rt"]);
        assert_eq!(harness.hold(&["rt"]), ["LaunchWithSqueezeWithNoDelay"]);
    }

    #[test]
    fn rt_up_while_lt_is_held_resumes_the_intake() {
        let harness = teleop();
        harness.hold(&["lt", "rt"]);
        assert_eq!(harness.hold(&["lt"]), ["IntakeFuel"]);
        assert_eq!(harness.hold(&[]), ["Idle"]);
    }

    #[test]
    fn lt_with_lb_ejects_and_both_released_goes_idle_twice() {
        let harness = teleop();
        harness.hold(&["lb"]);
        assert_eq!(harness.hold(&["lb", "lt"]), ["IntakeFuel", "Eject"]);
        // Java: the triggers-up binding and the eject binding each send Idle.
        assert_eq!(harness.hold(&["lb"]), ["Idle", "Idle"]);
    }

    #[test]
    fn face_buttons_and_select_map_to_states() {
        let harness = teleop();
        assert_eq!(harness.hold(&["x"]), ["TrackTarget"]);
        assert_eq!(harness.hold(&[]), ["Idle"]);
        assert_eq!(harness.hold(&["a"]), ["Unjam"]);
        assert_eq!(harness.hold(&[]), ["Idle"]);
        assert_eq!(harness.hold(&["select"]), ["ForceHome"]);
        assert_eq!(harness.hold(&[]), ["Idle"]);
        assert!(harness.hold(&["b"]).is_empty());
    }

    #[test]
    fn lb_with_the_dpad_reorients() {
        let harness = teleop();
        harness.hold(&["lb"]);
        assert_eq!(harness.hold(&["lb", "up"]), ["reorient forward"]);
        harness.hold(&["lb"]);
        assert_eq!(harness.hold(&["lb", "left"]), ["reorient left"]);
        harness.hold(&["lb"]);
        assert_eq!(harness.hold(&["lb", "down"]), ["reorient back"]);
        harness.hold(&["lb"]);
        assert_eq!(harness.hold(&["lb", "right"]), ["reorient right"]);
        harness.hold(&[]);
        assert!(harness.hold(&["up"]).is_empty());
    }

    #[test]
    fn operator_select_force_homes() {
        let harness = teleop();
        assert_eq!(harness.hold(&["op select"]), ["ForceHome"]);
        assert_eq!(harness.hold(&[]), ["Idle"]);
    }

    #[test]
    fn operator_lb_with_y_resets_the_intake_extension_and_rumbles() {
        let harness = teleop();
        harness.hold(&["op lb"]);
        assert_eq!(
            harness.hold(&["op lb", "op y"]),
            ["reset intake extension", "operator rumble"]
        );
    }

    #[test]
    fn operator_dpad_trims_the_shot() {
        let harness = teleop();
        let (hood, drive) = (
            shot_calculator::hood_angle_offset(),
            shot_calculator::drive_angle_offset(),
        );
        harness.hold(&["op up"]);
        harness.hold(&[]);
        assert!((shot_calculator::hood_angle_offset() - (hood + 0.1)).abs() < 1e-9);
        harness.hold(&["op down"]);
        harness.hold(&["op down"]);
        harness.hold(&[]);
        harness.hold(&["op down"]);
        assert!((shot_calculator::hood_angle_offset() - (hood - 0.1)).abs() < 1e-9);
        harness.hold(&["op left"]);
        assert_eq!(shot_calculator::drive_angle_offset(), drive + 1.0);
        harness.hold(&[]);
        harness.hold(&["op right"]);
        assert_eq!(shot_calculator::drive_angle_offset(), drive);
    }

    #[test]
    fn disabled_a_and_b_coast_and_brake_for_both_gamepads() {
        let harness = Harness::new(Mode::Disabled);
        assert_eq!(harness.hold(&["a"]), ["coast"]);
        assert_eq!(harness.hold(&["b"]), ["brake"]);
        assert_eq!(harness.hold(&["op a"]), ["coast"]);
        assert_eq!(harness.hold(&["op b"]), ["brake"]);
        // The state commands do not run while disabled.
        assert!(harness.hold(&["x"]).is_empty());
    }

    #[test]
    fn enabled_a_and_b_do_not_coast_or_brake() {
        let harness = teleop();
        assert_eq!(harness.hold(&["a"]), ["Unjam"]);
        assert_eq!(harness.hold(&["b"]), ["Idle"]);
        assert!(harness.hold(&["op a", "op b"]).is_empty());
    }
}
