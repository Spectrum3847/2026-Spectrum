use crate::auto::set_current_path_name;
use crate::config::{ModuleConfig, PidConstants, RobotConfig, kraken_x60};
use crate::controller::{PpHolonomicDriveController, SharedController};
use crate::events::EventScheduler;
use crate::feedforwards::DriveFeedforwards;
use crate::path::{GoalEndState, IdealStartingState, PathConstraints, PathPlannerPath};
use crate::telemetry::{PathPlannerLogging, PpLibTelemetry};
use crate::trajectory::PathPlannerTrajectory;
use crate::util::Timer;
use commands::{Command, CommandBody, Commands, SubsystemId};
use std::cell::RefCell;
use std::rc::Rc;
use wpimath::geometry::{Pose2d, Rotation2d};
use wpimath::kinematics::ChassisSpeeds;

/// The robot pose, field relative.
pub type PoseSupplier = Rc<dyn Fn() -> Pose2d>;
/// The robot-relative chassis speeds.
pub type SpeedsSupplier = Rc<dyn Fn() -> ChassisSpeeds>;
/// Takes robot-relative speeds and the feedforwards for each drive module.
pub type OutputConsumer = Rc<dyn Fn(ChassisSpeeds, &DriveFeedforwards)>;
/// True when paths should be flipped to the other side of the field.
pub type FlipSupplier = Rc<dyn Fn() -> bool>;

/// The command that follows a path.
pub struct FollowPathCommand;

struct FollowPath {
    timer: Timer,
    original_path: Rc<PathPlannerPath>,
    pose_supplier: PoseSupplier,
    speeds_supplier: SpeedsSupplier,
    output: OutputConsumer,
    controller: SharedController,
    robot_config: Rc<RobotConfig>,
    should_flip_path: FlipSupplier,
    event_scheduler: EventScheduler,
    path: Rc<PathPlannerPath>,
    trajectory: Option<Rc<PathPlannerTrajectory>>,
}

impl FollowPathCommand {
    /// Follows `path` with `controller`, sending the result to `output`.
    ///
    /// `requirements` are usually just the drive subsystem. The command also requires what the
    /// path's event marker commands require.
    ///
    /// # Panics
    /// When an event marker command requires one of `requirements`.
    #[allow(clippy::too_many_arguments, clippy::new_ret_no_self)]
    pub fn new(
        path: Rc<PathPlannerPath>,
        pose_supplier: PoseSupplier,
        speeds_supplier: SpeedsSupplier,
        output: OutputConsumer,
        controller: SharedController,
        robot_config: Rc<RobotConfig>,
        should_flip_path: FlipSupplier,
        requirements: &[SubsystemId],
    ) -> Command {
        let event_requirements = EventScheduler::get_scheduler_requirements(&path);
        assert!(
            !requirements.iter().any(|r| event_requirements.contains(r)),
            "Events that are triggered during path following cannot require the drive subsystem"
        );

        // Generate the ideal trajectory now so starting the path does not have to.
        let trajectory = path.get_ideal_trajectory(&robot_config);

        let body = FollowPath {
            timer: Timer::new(),
            original_path: path.clone(),
            pose_supplier,
            speeds_supplier,
            output,
            controller,
            robot_config,
            should_flip_path,
            event_scheduler: EventScheduler::new(),
            path,
            trajectory,
        };
        Command::builder("FollowPathCommand", body)
            .requires(requirements)
            .requires(&event_requirements)
            .build()
    }

    /// Runs a short path with a differential config, as PathPlannerLib does to warm up its
    /// code. Runs when disabled.
    pub fn warmup_command() -> Command {
        let waypoints = PathPlannerPath::waypoints_from_poses(&[
            Pose2d::from_xy(0.0, 0.0, Rotation2d::ZERO),
            Pose2d::from_xy(6.0, 6.0, Rotation2d::ZERO),
        ]);
        let path = PathPlannerPath::simple(
            waypoints,
            PathConstraints::new(4.0, 4.0, 4.0, 4.0),
            Some(IdealStartingState::new(0.0, Rotation2d::ZERO)),
            GoalEndState::new(0.0, Rotation2d::CW_PI_2),
        );

        Self::new(
            Rc::new(path),
            Rc::new(|| Pose2d::ZERO),
            Rc::new(ChassisSpeeds::default),
            Rc::new(|_, _| {}),
            Rc::new(RefCell::new(PpHolonomicDriveController::new(
                PidConstants::new(5.0, 0.0, 0.0),
                PidConstants::new(5.0, 0.0, 0.0),
            ))),
            Rc::new(warmup_robot_config()),
            Rc::new(|| true),
            &[],
        )
        .and_then(Commands::print(
            "[PathPlanner] FollowPathCommand finished warmup",
        ))
        .ignoring_disable(true)
    }
}

/// The config both warmup commands use. It is a differential drive, as in PathPlannerLib.
pub(crate) fn warmup_robot_config() -> RobotConfig {
    RobotConfig::new_differential(
        75.0,
        6.8,
        ModuleConfig::new(0.048, 5.0, 1.2, kraken_x60(1).with_reduction(6.14), 60.0, 1),
        0.55,
    )
}

impl FollowPath {
    fn trajectory(&self) -> &PathPlannerTrajectory {
        self.trajectory
            .as_deref()
            .expect("initialize runs before the trajectory is used")
    }
}

impl CommandBody for FollowPath {
    fn initialize(&mut self) {
        self.path = if (self.should_flip_path)() && !self.original_path.prevent_flipping {
            Rc::new(self.original_path.flip_path())
        } else {
            self.original_path.clone()
        };

        let current_pose = (self.pose_supplier)();
        let current_speeds = (self.speeds_supplier)();

        self.controller
            .borrow_mut()
            .reset(current_pose, current_speeds);

        let linear_vel = current_speeds.vx.hypot(current_speeds.vy);

        let config = &self.robot_config;
        let regenerate = || {
            Rc::new(
                self.path
                    .generate_trajectory(current_speeds, current_pose.rotation, config),
            )
        };
        self.trajectory = Some(match self.path.ideal_starting_state() {
            Some(ideal) => {
                // The ideal trajectory is good when the robot starts near the ideal state.
                let ideal_velocity = (linear_vel - ideal.velocity_mps).abs() <= 0.25;
                let ideal_rotation = !config.is_holonomic
                    || (current_pose.rotation - ideal.rotation).degrees().abs() <= 30.0;
                if ideal_velocity && ideal_rotation {
                    self.path
                        .get_ideal_trajectory(config)
                        .expect("a path with an ideal starting state has an ideal trajectory")
                } else {
                    regenerate()
                }
            }
            None => regenerate(),
        });

        set_current_path_name(&self.original_path.name);

        PathPlannerLogging::log_active_path(Some(&self.path));
        PpLibTelemetry::set_current_path(&self.path);

        let trajectory = self.trajectory.clone().expect("set above");
        self.event_scheduler.initialize(&trajectory);

        self.timer.reset();
        self.timer.start();
    }

    fn execute(&mut self) {
        let current_time = self.timer.get();
        let target_state = self.trajectory().sample(current_time);

        let current_pose = (self.pose_supplier)();
        let current_speeds = (self.speeds_supplier)();

        let target_speeds = self
            .controller
            .borrow_mut()
            .calculate_robot_relative_speeds(current_pose, &target_state);

        let current_vel = current_speeds.vx.hypot(current_speeds.vy);

        PpLibTelemetry::set_current_pose(current_pose);
        PathPlannerLogging::log_current_pose(current_pose);

        PpLibTelemetry::set_target_pose(target_state.pose);
        PathPlannerLogging::log_target_pose(target_state.pose);

        PpLibTelemetry::set_velocities(
            current_vel,
            target_state.linear_velocity,
            current_speeds.omega,
            target_speeds.omega,
        );

        (self.output)(target_speeds, &target_state.feedforwards);

        self.event_scheduler.execute(current_time);
    }

    fn is_finished(&mut self) -> bool {
        let total_time = self.trajectory().total_time_seconds();
        self.timer.has_elapsed(total_time) || !total_time.is_finite()
    }

    fn end(&mut self, interrupted: bool) {
        self.timer.stop();
        set_current_path_name("");

        // Only stop the robot at the end of a path that is meant to stop. Interrupting a path
        // then hands over smoothly to something like an auto-alignment routine.
        if !interrupted && self.path.goal_end_state().velocity_mps < 0.1 {
            (self.output)(
                ChassisSpeeds::default(),
                &DriveFeedforwards::zeros(self.robot_config.num_modules),
            );
        }

        PathPlannerLogging::log_active_path(None);

        self.event_scheduler.end();
    }
}
