use crate::config::RobotConfig;
use crate::controller::SharedController;
use crate::flipping::FlippingUtil;
use crate::follow_path::{
    FlipSupplier, FollowPathCommand, OutputConsumer, PoseSupplier, SpeedsSupplier,
};
use crate::path::{PathConstraints, PathPlannerPath};
use crate::pathfinding_command::PathfindingCommand;
use crate::util::deploy_directory;
use commands::{Command, Commands, SubsystemId};
use std::cell::RefCell;
use std::rc::Rc;
use wpimath::geometry::Pose2d;

type PathCommandBuilder = Rc<dyn Fn(Rc<PathPlannerPath>) -> Command>;
type PathfindCommandBuilder = Rc<dyn Fn(Pose2d, PathConstraints, f64) -> Command>;

#[derive(Default)]
struct Globals {
    configured: bool,
    pose_supplier: Option<PoseSupplier>,
    reset_pose: Option<Rc<dyn Fn(Pose2d)>>,
    should_flip_path: Option<FlipSupplier>,
    is_holonomic: bool,
    path_following_command_builder: Option<PathCommandBuilder>,
    pathfinding_configured: bool,
    pathfind_to_pose_command_builder: Option<PathfindCommandBuilder>,
}

thread_local! {
    static GLOBALS: RefCell<Globals> = RefCell::new(Globals::default());
}

fn globals<R>(f: impl FnOnce(&Globals) -> R) -> R {
    GLOBALS.with(|g| f(&g.borrow()))
}

/// Builds the commands that follow paths and run autos. Configure it once at startup.
///
/// Its methods panic when called before [`configure`](Self::configure), where Java throws an
/// `AutoBuilderException`.
pub struct AutoBuilder;

impl AutoBuilder {
    /// Sets up PathPlanner's built-in commands.
    ///
    /// `output` takes robot-relative speeds and the feedforwards for each drive module. The
    /// feedforwards assume unoptimized module states. When you optimize your states, reverse the
    /// feedforwards of the modules you flipped. `should_flip_path` says whether paths flip to the
    /// other side of the field, keeping a global blue origin.
    #[allow(clippy::too_many_arguments)]
    pub fn configure(
        pose_supplier: PoseSupplier,
        reset_pose: Rc<dyn Fn(Pose2d)>,
        robot_relative_speeds_supplier: SpeedsSupplier,
        output: OutputConsumer,
        controller: SharedController,
        robot_config: RobotConfig,
        should_flip_path: FlipSupplier,
        drive_requirements: &[SubsystemId],
    ) {
        if Self::is_configured() {
            wpilib::driver_station::report_error(
                "Auto builder has already been configured. This is likely in error.",
            );
        }

        let robot_config = Rc::new(robot_config);
        let requirements = drive_requirements.to_vec();

        let follow = {
            let (pose, speeds, output, controller, config, flip, requirements) = (
                pose_supplier.clone(),
                robot_relative_speeds_supplier.clone(),
                output.clone(),
                controller.clone(),
                robot_config.clone(),
                should_flip_path.clone(),
                requirements.clone(),
            );
            Rc::new(move |path: Rc<PathPlannerPath>| {
                FollowPathCommand::new(
                    path,
                    pose.clone(),
                    speeds.clone(),
                    output.clone(),
                    controller.clone(),
                    config.clone(),
                    flip.clone(),
                    &requirements,
                )
            })
        };

        let pathfind = {
            let (pose, speeds, output, controller, config, requirements) = (
                pose_supplier.clone(),
                robot_relative_speeds_supplier,
                output,
                controller,
                robot_config.clone(),
                requirements,
            );
            Rc::new(
                move |target: Pose2d, constraints: PathConstraints, goal_end_vel: f64| {
                    PathfindingCommand::new(
                        target,
                        constraints,
                        goal_end_vel,
                        pose.clone(),
                        speeds.clone(),
                        output.clone(),
                        controller.clone(),
                        config.clone(),
                        &requirements,
                    )
                },
            )
        };

        GLOBALS.with(|g| {
            *g.borrow_mut() = Globals {
                configured: true,
                pose_supplier: Some(pose_supplier),
                reset_pose: Some(reset_pose),
                should_flip_path: Some(should_flip_path),
                is_holonomic: robot_config.is_holonomic,
                path_following_command_builder: Some(follow),
                pathfinding_configured: true,
                pathfind_to_pose_command_builder: Some(pathfind),
            };
        });
    }

    pub fn is_configured() -> bool {
        globals(|g| g.configured)
    }

    pub fn is_pathfinding_configured() -> bool {
        globals(|g| g.pathfinding_configured)
    }

    /// Clears everything `configure` set.
    pub fn reset_for_testing() {
        GLOBALS.with(|g| *g.borrow_mut() = Globals::default());
    }

    pub fn get_current_pose() -> Pose2d {
        let supplier = globals(|g| g.pose_supplier.clone()).expect("AutoBuilder is configured");
        supplier()
    }

    /// Whether paths and field positions should currently be flipped.
    pub fn should_flip() -> bool {
        let supplier = globals(|g| g.should_flip_path.clone()).expect("AutoBuilder is configured");
        supplier()
    }

    /// Whether the robot config is a holonomic drive.
    pub fn is_holonomic() -> bool {
        assert!(
            Self::is_configured(),
            "AutoBuilder was not configured before use"
        );
        globals(|g| g.is_holonomic)
    }

    /// A command that follows `path` and fires its event markers.
    pub fn follow_path(path: Rc<PathPlannerPath>) -> Command {
        let builder = globals(|g| g.path_following_command_builder.clone()).expect(
            "Auto builder was used to build a path following command before being configured",
        );
        builder(path)
    }

    /// A command that pathfinds to a pose. The pose rotation is the goal heading.
    pub fn pathfind_to_pose(
        pose: Pose2d,
        constraints: PathConstraints,
        goal_end_velocity: f64,
    ) -> Command {
        let builder = globals(|g| g.pathfind_to_pose_command_builder.clone())
            .expect("Auto builder was used to build a pathfinding command before being configured");
        builder(pose, constraints, goal_end_velocity)
    }

    /// Like [`pathfind_to_pose`](Self::pathfind_to_pose), flipping the pose when the flip supplier
    /// says so as the command starts.
    pub fn pathfind_to_pose_flipped(
        pose: Pose2d,
        constraints: PathConstraints,
        goal_end_velocity: f64,
    ) -> Command {
        let flip = globals(|g| g.should_flip_path.clone())
            .expect("Auto builder was used to build a pathfinding command before being configured");
        Commands::either(
            Self::pathfind_to_pose(
                FlippingUtil::flip_field_pose(pose),
                constraints,
                goal_end_velocity,
            ),
            Self::pathfind_to_pose(pose, constraints, goal_end_velocity),
            flip,
        )
    }

    /// The names of the autos in the deploy folder, sorted.
    pub fn get_all_auto_names() -> Vec<String> {
        let Ok(entries) = std::fs::read_dir(deploy_directory().join("pathplanner/autos")) else {
            return Vec::new();
        };
        let mut names: Vec<String> = entries
            .filter_map(Result::ok)
            .filter(|e| e.path().is_file())
            .filter_map(|e| e.file_name().into_string().ok())
            .filter_map(|name| name.strip_suffix(".auto").map(str::to_string))
            .collect();
        names.sort();
        names
    }

    /// A command that resets the robot pose to a blue-origin pose, flipped when needed.
    pub fn reset_odom(blue_pose: Pose2d) -> Command {
        assert!(
            Self::is_configured(),
            "AutoBuilder was not configured before use"
        );
        Commands::run_once(
            move || {
                let flip = Self::should_flip();
                let reset = globals(|g| g.reset_pose.clone()).expect("AutoBuilder is configured");
                if flip {
                    reset(FlippingUtil::flip_field_pose(blue_pose));
                } else {
                    reset(blue_pose);
                }
            },
            &[],
        )
    }
}
