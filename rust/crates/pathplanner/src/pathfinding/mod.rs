mod ad_star;
mod java_map;

pub use ad_star::{AdStarPlanner, Grid, GridPosition, LocalAdStar, OutOfGrid, Plan, PlanRequest};

use crate::path::{GoalEndState, PathConstraints, PathPlannerPath};
use std::sync::{Arc, Mutex, MutexGuard};
use wpimath::geometry::Translation2d;

/// A pathfinder the pathfinding commands can use.
pub trait Pathfinder: Send + Sync {
    /// Whether a path has been calculated since the last time one was retrieved.
    fn is_new_path_available(&self) -> bool;

    /// The most recent path, made from the points the pathfinder calculated. `None` when it has
    /// not found one.
    fn get_current_path(
        &self,
        constraints: PathConstraints,
        goal_end_state: GoalEndState,
    ) -> Option<PathPlannerPath>;

    /// A start inside an obstacle moves to the nearest free node.
    fn set_start_position(&self, start_position: Translation2d);

    /// A goal inside an obstacle moves to the nearest free node.
    fn set_goal_position(&self, goal_position: Translation2d);
}

static PATHFINDER: Mutex<Option<Arc<dyn Pathfinder>>> = Mutex::new(None);

fn pathfinder() -> MutexGuard<'static, Option<Arc<dyn Pathfinder>>> {
    PATHFINDER.lock().unwrap_or_else(|e| e.into_inner())
}

fn current() -> Arc<dyn Pathfinder> {
    pathfinder()
        .clone()
        .expect("Pathfinding::ensure_initialized runs before the pathfinder is used")
}

/// The pathfinder the pathfinding commands share.
pub struct Pathfinding;

impl Pathfinding {
    pub fn set_pathfinder(new_pathfinder: Arc<dyn Pathfinder>) {
        *pathfinder() = Some(new_pathfinder);
    }

    /// Chooses the default pathfinder, [`LocalAdStar`], when none is set yet.
    pub fn ensure_initialized() {
        let mut guard = pathfinder();
        if guard.is_none() {
            *guard = Some(Arc::new(LocalAdStar::new()));
        }
    }

    pub fn is_new_path_available() -> bool {
        current().is_new_path_available()
    }

    pub fn get_current_path(
        constraints: PathConstraints,
        goal_end_state: GoalEndState,
    ) -> Option<PathPlannerPath> {
        current().get_current_path(constraints, goal_end_state)
    }

    pub fn set_start_position(start_position: Translation2d) {
        current().set_start_position(start_position);
    }

    pub fn set_goal_position(goal_position: Translation2d) {
        current().set_goal_position(goal_position);
    }
}
