//! Publishes the PathPlanner telemetry to the default NetworkTables instance and reads it back.
//!
//! Run from `rust/` with the natives on the library path:
//! `cargo xtask sim pathplanner_telemetry -p pathplanner`

use pathplanner::{PathPlannerPath, PpLibTelemetry, set_deploy_directory};
use std::path::PathBuf;
use wpilib::nt::{DoubleArray, Instance, Options, StructArrayKind, StructKind};
use wpimath::geometry::{Pose2d, Rotation2d};

fn main() {
    set_deploy_directory(Some(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../src/main/deploy"),
    ));
    // Telemetry stays quiet until something has loaded the HAL.
    wpilib::hal::initialize();

    let path = PathPlannerPath::from_path_file("TBTT 1").unwrap();
    let pose = Pose2d::from_xy(1.5, 2.5, Rotation2d::from_degrees(90.0));
    PpLibTelemetry::set_velocities(1.0, 2.0, 3.0, 4.0);
    PpLibTelemetry::set_current_pose(pose);
    PpLibTelemetry::set_target_pose(Pose2d::from_xy(3.0, 4.0, Rotation2d::ZERO));
    PpLibTelemetry::set_current_path(&path);

    let inst = Instance::default_instance();
    let options = Options::default();

    let vel = inst
        .topic::<DoubleArray>("/PathPlanner/vel")
        .subscribe(vec![], &options)
        .get();
    println!("vel: {vel:?}");
    assert_eq!(vel, [1.0, 2.0, 3.0, 4.0]);

    let current = inst
        .topic::<StructKind<Pose2d>>("/PathPlanner/currentPose")
        .subscribe(Pose2d::ZERO, &options)
        .get();
    println!("currentPose: {current:?}");
    assert_eq!(current, pose);

    let target = inst
        .topic::<StructKind<Pose2d>>("/PathPlanner/targetPose")
        .subscribe(Pose2d::ZERO, &options)
        .get();
    assert_eq!(target.x(), 3.0);

    let active = inst
        .topic::<StructArrayKind<Pose2d>>("/PathPlanner/activePath")
        .subscribe(vec![], &options)
        .get();
    println!("activePath: {} poses", active.len());
    assert_eq!(active.len(), path.num_points());

    PpLibTelemetry::enable_competition_mode();
    PpLibTelemetry::set_velocities(9.0, 9.0, 9.0, 9.0);
    let after = inst
        .topic::<DoubleArray>("/PathPlanner/vel")
        .subscribe(vec![], &options)
        .get();
    assert_eq!(
        after,
        [1.0, 2.0, 3.0, 4.0],
        "competition mode stops publishing"
    );
    println!("ok");
}
