use crate::config::{PidConstants, RobotConfig};
use crate::controller::{PpHolonomicDriveController, SharedController};
use crate::feedforwards::DriveFeedforwards;
use crate::follow_path::{OutputConsumer, PoseSupplier, SpeedsSupplier, warmup_robot_config};
use crate::path::{GoalEndState, PathConstraints};
use crate::pathfinding::Pathfinding;
use crate::telemetry::{PathPlannerLogging, PpLibTelemetry};
use crate::trajectory::PathPlannerTrajectory;
use crate::util::Timer;
use commands::{Command, CommandBody, Commands, SubsystemId};
use std::cell::RefCell;
use std::rc::Rc;
use wpimath::geometry::{Pose2d, Rotation2d};
use wpimath::kinematics::ChassisSpeeds;
use wpimath::math_util;

/// The command that pathfinds to a pose.
pub struct PathfindingCommand;

struct Pathfind {
    timer: Timer,
    target_pose: Pose2d,
    goal_end_state: GoalEndState,
    constraints: PathConstraints,
    pose_supplier: PoseSupplier,
    speeds_supplier: SpeedsSupplier,
    output: OutputConsumer,
    controller: SharedController,
    robot_config: Rc<RobotConfig>,
    current_trajectory: Option<PathPlannerTrajectory>,
    time_offset: f64,
    finish: bool,
}

impl PathfindingCommand {
    /// Pathfinds to `target_pose`, then ends moving at `goal_end_vel` m/s. Starts the default
    /// pathfinder if none is set. The pose rotation is the goal heading.
    #[allow(clippy::too_many_arguments, clippy::new_ret_no_self)]
    pub fn new(
        target_pose: Pose2d,
        constraints: PathConstraints,
        goal_end_vel: f64,
        pose_supplier: PoseSupplier,
        speeds_supplier: SpeedsSupplier,
        output: OutputConsumer,
        controller: SharedController,
        robot_config: Rc<RobotConfig>,
        requirements: &[SubsystemId],
    ) -> Command {
        Pathfinding::ensure_initialized();

        let body = Pathfind {
            timer: Timer::new(),
            target_pose,
            goal_end_state: GoalEndState::new(goal_end_vel, target_pose.rotation),
            constraints,
            pose_supplier,
            speeds_supplier,
            output,
            controller,
            robot_config,
            current_trajectory: None,
            time_offset: 0.0,
            finish: false,
        };
        Command::builder("PathfindingCommand", body)
            .requires(requirements)
            .build()
    }

    /// Pathfinds across the field with a differential config, as PathPlannerLib does to warm up
    /// the pathfinder. Runs when disabled.
    pub fn warmup_command() -> Command {
        Self::new(
            Pose2d::from_xy(15.0, 4.0, Rotation2d::PI),
            PathConstraints::new(4.0, 3.0, 4.0, 4.0),
            0.0,
            Rc::new(|| Pose2d::from_xy(1.5, 4.0, Rotation2d::ZERO)),
            Rc::new(ChassisSpeeds::default),
            Rc::new(|_, _| {}),
            Rc::new(RefCell::new(PpHolonomicDriveController::new(
                PidConstants::new(5.0, 0.0, 0.0),
                PidConstants::new(5.0, 0.0, 0.0),
            ))),
            Rc::new(warmup_robot_config()),
            &[],
        )
        .and_then(Commands::print(
            "[PathPlanner] PathfindingCommand finished warmup",
        ))
        .ignoring_disable(true)
    }
}

impl Pathfind {
    fn stop(&self) {
        (self.output)(
            ChassisSpeeds::default(),
            &DriveFeedforwards::zeros(self.robot_config.num_modules),
        );
    }
}

impl CommandBody for Pathfind {
    fn initialize(&mut self) {
        self.current_trajectory = None;
        self.time_offset = 0.0;
        self.finish = false;

        let current_pose = (self.pose_supplier)();

        self.controller
            .borrow_mut()
            .reset(current_pose, (self.speeds_supplier)());

        if current_pose
            .translation
            .distance(self.target_pose.translation)
            < 0.5
        {
            self.stop();
            self.finish = true;
        } else {
            Pathfinding::set_start_position(current_pose.translation);
            Pathfinding::set_goal_position(self.target_pose.translation);
        }
    }

    fn execute(&mut self) {
        if self.finish {
            return;
        }

        let current_pose = (self.pose_supplier)();
        let current_speeds = (self.speeds_supplier)();

        PathPlannerLogging::log_current_pose(current_pose);
        PpLibTelemetry::set_current_pose(current_pose);

        // Skip new paths when close to the end.
        let skip_updates = self.current_trajectory.as_ref().is_some_and(|t| {
            current_pose
                .translation
                .distance(t.end_state().pose.translation)
                < 2.0
        });

        if !skip_updates && Pathfinding::is_new_path_available() {
            let current_path = Pathfinding::get_current_path(self.constraints, self.goal_end_state);

            if let Some(path) = current_path {
                let trajectory = PathPlannerTrajectory::generate(
                    &path,
                    current_speeds,
                    current_pose.rotation,
                    &self.robot_config,
                );
                let finite = trajectory.total_time_seconds().is_finite();
                let trajectory = self.current_trajectory.insert(trajectory);
                if !finite {
                    self.finish = true;
                    return;
                }

                // Find the two closest states, in front of and behind the robot.
                let states = trajectory.states();
                let mut closest_state1_idx = 0;
                let mut closest_state2_idx = 1;
                while closest_state2_idx < states.len() - 1 {
                    let closest2_dist = states[closest_state2_idx]
                        .pose
                        .translation
                        .distance(current_pose.translation);
                    let next_dist = states[closest_state2_idx + 1]
                        .pose
                        .translation
                        .distance(current_pose.translation);
                    if next_dist < closest2_dist {
                        closest_state1_idx += 1;
                        closest_state2_idx += 1;
                    } else {
                        break;
                    }
                }

                // Interpolate the time offset from those two states. It accounts for the delay
                // in pathfinding.
                let closest_state1 = &states[closest_state1_idx];
                let closest_state2 = &states[closest_state2_idx];

                let d = closest_state1
                    .pose
                    .translation
                    .distance(closest_state2.pose.translation);
                let t = current_pose
                    .translation
                    .distance(closest_state1.pose.translation)
                    / d;
                let t = math_util::clamp(t, 0.0, 1.0);

                self.time_offset = math_util::interpolate(
                    closest_state1.time_seconds,
                    closest_state2.time_seconds,
                    t,
                );

                // A robot that is stationary at the start moves the offset to the next loop. That
                // stops it staying still when new paths arrive every loop.
                if self.time_offset <= 0.02 && current_speeds.vx.hypot(current_speeds.vy) < 0.1 {
                    self.time_offset = 0.02;
                }

                PathPlannerLogging::log_active_path(Some(&path));
                PpLibTelemetry::set_current_path(&path);
            }

            self.timer.reset();
            self.timer.start();
        }

        if let Some(trajectory) = &self.current_trajectory {
            let target_state = trajectory.sample(self.timer.get() + self.time_offset);

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
        }
    }

    fn is_finished(&mut self) -> bool {
        if self.finish {
            return true;
        }

        match &self.current_trajectory {
            Some(trajectory) => self
                .timer
                .has_elapsed(trajectory.total_time_seconds() - self.time_offset),
            None => false,
        }
    }

    fn end(&mut self, interrupted: bool) {
        self.timer.stop();

        // Only stop at the end of a path that is meant to stop. Interrupting it then hands over
        // smoothly to something like an auto-alignment routine.
        if !interrupted && self.goal_end_state.velocity_mps < 0.1 {
            self.stop();
        }

        PathPlannerLogging::log_active_path(None);
    }
}
