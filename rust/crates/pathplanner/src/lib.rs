//! PathPlanner path loading, trajectory generation and path following.
//!
//! Ported from PathPlannerLib v2026.1.2 (MIT, see `THIRD_PARTY.md`). Java names map to
//! snake_case, and the `PP` prefix becomes `Pp`.
//!
//! Not ported, because the robot does not use it: Choreo trajectories, `PathfindThenFollowPath`,
//! dynamic obstacles, hot reload, the LTV controller, the setpoint generator, auto choosers,
//! the auto triggers (`isRunning`, `timeElapsed`, `event`, `beforeEvent` and the rest) and
//! `configureCustom`. Differential drive exists only where the warmup commands need it.

mod auto;
mod auto_builder;
mod command_util;
mod config;
mod controller;
mod error;
mod events;
mod feedforwards;
mod flipping;
mod follow_path;
mod geometry_util;
mod path;
mod pathfinding;
mod pathfinding_command;
mod telemetry;
mod trajectory;
mod util;

pub use auto::{PathPlannerAuto, current_path_name};
pub use auto_builder::AutoBuilder;
pub use command_util::{NamedCommands, command_from_json, wrapped_event_command};
pub use config::{ModuleConfig, PidConstants, RobotConfig, kraken_x60};
pub use controller::{PathFollowingController, PpHolonomicDriveController, SharedController};
pub use error::{Error, Result};
pub use events::{Event, EventScheduler, EventTrigger, PointTowardsZoneTrigger};
pub use feedforwards::{DriveFeedforwards, MAX_MODULES, PerModule};
pub use flipping::{FieldSymmetry, FlippingUtil};
pub use follow_path::{
    FlipSupplier, FollowPathCommand, OutputConsumer, PoseSupplier, SpeedsSupplier,
};
pub use geometry_util::{calculate_radius, cubic_lerp, quadratic_lerp};
pub use path::{
    ConstraintsZone, EventMarker, GoalEndState, IdealStartingState, PathConstraints,
    PathPlannerPath, PathPoint, PointTowardsZone, RotationTarget, Waypoint,
};
pub use pathfinding::{
    AdStarPlanner, Grid, GridPosition, LocalAdStar, OutOfGrid, Pathfinder, Pathfinding, Plan,
    PlanRequest,
};
pub use pathfinding_command::PathfindingCommand;
pub use telemetry::{PathPlannerLogging, PpLibTelemetry};
pub use trajectory::{PathPlannerTrajectory, PathPlannerTrajectoryState};
pub use util::set_deploy_directory;
