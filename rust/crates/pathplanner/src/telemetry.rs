use crate::path::PathPlannerPath;
use std::cell::RefCell;
use wpilib::nt::{DoubleArray, Instance, Options, Publisher, StructArrayKind, StructKind};
use wpimath::geometry::Pose2d;

struct Publishers {
    vel: Publisher<DoubleArray>,
    current_pose: Publisher<StructKind<Pose2d>>,
    active_path: Publisher<StructArrayKind<Pose2d>>,
    target_pose: Publisher<StructKind<Pose2d>>,
}

impl Publishers {
    fn new() -> Self {
        let inst = Instance::default_instance();
        let options = Options::default();
        Self {
            vel: inst
                .topic::<DoubleArray>("/PathPlanner/vel")
                .publish(&options),
            current_pose: inst
                .topic::<StructKind<Pose2d>>("/PathPlanner/currentPose")
                .publish(&options),
            active_path: inst
                .topic::<StructArrayKind<Pose2d>>("/PathPlanner/activePath")
                .publish(&options),
            target_pose: inst
                .topic::<StructKind<Pose2d>>("/PathPlanner/targetPose")
                .publish(&options),
        }
    }
}

#[derive(Default)]
struct State {
    comp_mode: bool,
    publishers: Option<Publishers>,
}

thread_local! {
    static STATE: RefCell<State> = RefCell::new(State::default());
}

/// Sends data to the PathPlanner app and dashboards over NetworkTables, under `/PathPlanner`.
///
/// It publishes only once the HAL is loaded, so code that runs without natives (unit tests)
/// does nothing here.
pub struct PpLibTelemetry;

impl PpLibTelemetry {
    /// Stops all publishing.
    pub fn enable_competition_mode() {
        STATE.with(|s| s.borrow_mut().comp_mode = true);
    }

    fn publish(f: impl FnOnce(&Publishers)) {
        if wpilib::hal::try_hal().is_none() {
            return;
        }
        STATE.with(|s| {
            let mut state = s.borrow_mut();
            if state.comp_mode {
                return;
            }
            f(state.publishers.get_or_insert_with(Publishers::new));
        });
    }

    /// Actual and commanded velocity in m/s, then actual and commanded angular velocity in rad/s.
    pub fn set_velocities(
        actual_vel: f64,
        commanded_vel: f64,
        actual_ang_vel: f64,
        commanded_ang_vel: f64,
    ) {
        Self::publish(|p| {
            p.vel
                .set(&[actual_vel, commanded_vel, actual_ang_vel, commanded_ang_vel]);
        });
    }

    pub fn set_current_pose(pose: Pose2d) {
        Self::publish(|p| {
            p.current_pose.set(&pose);
        });
    }

    pub fn set_current_path(path: &PathPlannerPath) {
        Self::publish(|p| {
            p.active_path.set(&path.get_path_poses());
        });
    }

    pub fn set_target_pose(target_pose: Pose2d) {
        Self::publish(|p| {
            p.target_pose.set(&target_pose);
        });
    }
}

type PoseCallback = Box<dyn Fn(Pose2d)>;
type PathCallback = Box<dyn Fn(Vec<Pose2d>)>;

#[derive(Default)]
struct Callbacks {
    current_pose: Option<PoseCallback>,
    target_pose: Option<PoseCallback>,
    active_path: Option<PathCallback>,
}

thread_local! {
    static CALLBACKS: RefCell<Callbacks> = RefCell::new(Callbacks::default());
}

/// Callbacks for logging what path following does. Each can be cleared with `None`.
pub struct PathPlannerLogging;

impl PathPlannerLogging {
    pub fn set_log_current_pose_callback(callback: Option<PoseCallback>) {
        CALLBACKS.with(|c| c.borrow_mut().current_pose = callback);
    }

    pub fn set_log_target_pose_callback(callback: Option<PoseCallback>) {
        CALLBACKS.with(|c| c.borrow_mut().target_pose = callback);
    }

    pub fn set_log_active_path_callback(callback: Option<PathCallback>) {
        CALLBACKS.with(|c| c.borrow_mut().active_path = callback);
    }

    pub fn clear_logging_callbacks() {
        CALLBACKS.with(|c| *c.borrow_mut() = Callbacks::default());
    }

    pub(crate) fn log_current_pose(pose: Pose2d) {
        CALLBACKS.with(|c| {
            if let Some(callback) = &c.borrow().current_pose {
                callback(pose);
            }
        });
    }

    pub(crate) fn log_target_pose(pose: Pose2d) {
        CALLBACKS.with(|c| {
            if let Some(callback) = &c.borrow().target_pose {
                callback(pose);
            }
        });
    }

    /// `None` logs an empty path.
    pub(crate) fn log_active_path(path: Option<&PathPlannerPath>) {
        CALLBACKS.with(|c| {
            if let Some(callback) = &c.borrow().active_path {
                callback(
                    path.map(PathPlannerPath::get_path_poses)
                        .unwrap_or_default(),
                );
            }
        });
    }
}
