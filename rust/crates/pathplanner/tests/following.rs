//! Runs the commands against a simulated robot that follows its commanded speeds exactly.

use commands::testing::FakeClock;
use commands::{Command, CommandScheduler, Commands};
use pathplanner::*;
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::path::PathBuf;
use std::rc::Rc;
use std::thread::sleep;
use std::time::{Duration, Instant};
use wpimath::geometry::{Pose2d, Rotation2d};
use wpimath::kinematics::ChassisSpeeds;

const DT: f64 = 0.02;

fn use_deploy_folder() {
    set_deploy_directory(Some(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../src/main/deploy"),
    ));
}

/// A robot that drives at whatever speeds it was last commanded.
#[derive(Clone)]
struct SimRobot {
    pose: Rc<Cell<Pose2d>>,
    speeds: Rc<Cell<ChassisSpeeds>>,
    outputs: Rc<RefCell<Vec<ChassisSpeeds>>>,
    flip: Rc<Cell<bool>>,
}

impl SimRobot {
    fn new(pose: Pose2d) -> Self {
        Self {
            pose: Rc::new(Cell::new(pose)),
            speeds: Rc::new(Cell::new(ChassisSpeeds::default())),
            outputs: Rc::default(),
            flip: Rc::default(),
        }
    }

    fn configure(&self) {
        let (pose, speeds, outputs, flip) = (
            self.pose.clone(),
            self.speeds.clone(),
            self.outputs.clone(),
            self.flip.clone(),
        );
        let (reset_pose, output_speeds) = (self.pose.clone(), self.speeds.clone());
        AutoBuilder::reset_for_testing();
        AutoBuilder::configure(
            Rc::new(move || pose.get()),
            Rc::new(move |p| reset_pose.set(p)),
            Rc::new(move || speeds.get()),
            Rc::new(move |s, _| {
                output_speeds.set(s);
                outputs.borrow_mut().push(s);
            }),
            Rc::new(RefCell::new(PpHolonomicDriveController::new(
                PidConstants::new(4.0, 0.0, 0.0),
                PidConstants::new(3.0, 0.0, 0.0),
            ))),
            RobotConfig::from_gui_settings().unwrap(),
            Rc::new(move || flip.get()),
            &[],
        );
    }

    /// Moves the robot for one loop.
    fn step(&self) {
        let pose = self.pose.get();
        let speeds = self.speeds.get();
        let field = ChassisSpeeds::from_robot_relative(speeds, pose.rotation);
        self.pose.set(Pose2d::from_xy(
            pose.x() + field.vx * DT,
            pose.y() + field.vy * DT,
            Rotation2d::new(pose.rotation.radians() + speeds.omega * DT),
        ));
    }
}

/// Runs the scheduler until the command ends. Returns the seconds it took.
fn run_until_done(
    clock: &FakeClock,
    robot: &SimRobot,
    command: &Command,
    real_time_wait: bool,
) -> f64 {
    let scheduler = CommandScheduler::get_instance();
    scheduler.schedule(command);
    let start = Instant::now();
    let mut loops = 0;
    while command.is_scheduled() {
        clock.advance(DT);
        scheduler.run();
        robot.step();
        loops += 1;
        assert!(loops < 5000, "the command did not finish");
        if real_time_wait {
            assert!(
                start.elapsed() < Duration::from_secs(60),
                "the command did not finish"
            );
            sleep(Duration::from_millis(2));
        }
    }
    f64::from(loops) * DT
}

fn distance(a: Pose2d, b: Pose2d) -> f64 {
    a.translation.distance(b.translation)
}

fn run_path(name: &str, flip: bool) -> (Pose2d, f64, HashMap<&'static str, u32>, SimRobot) {
    use_deploy_folder();
    let clock = FakeClock::install();
    let path = PathPlannerPath::from_path_file(name).unwrap();
    let mut start = path.get_starting_holonomic_pose().unwrap();
    if flip {
        start = FlippingUtil::flip_field_pose(start);
    }
    let robot = SimRobot::new(start);
    robot.flip.set(flip);
    robot.configure();

    let counts: Rc<RefCell<HashMap<&'static str, u32>>> = Rc::default();
    for event in ["intake", "shotPrep", "poseUpdate", "clearState"] {
        let counts = counts.clone();
        EventTrigger::new(event).on_true(Commands::run_once(
            move || *counts.borrow_mut().entry(event).or_default() += 1,
            &[],
        ));
    }

    let command = AutoBuilder::follow_path(path);
    let seconds = run_until_done(&clock, &robot, &command, false);
    let counts = counts.borrow().clone();
    (robot.pose.get(), seconds, counts, robot)
}

#[test]
fn follows_a_path_and_fires_its_events() {
    let (end, seconds, counts, robot) = run_path("TBTT 1", false);

    let goal = Pose2d::from_xy(2.26, 5.8, Rotation2d::from_degrees(135.0));
    assert!(distance(end, goal) < 0.1, "ended at {end:?}");
    assert!((end.rotation - goal.rotation).degrees().abs() < 5.0);
    assert!((seconds - 5.32).abs() < 0.1, "took {seconds} s");

    // One marker for each of intake and shotPrep, and two poseUpdate zones.
    assert_eq!(counts.get("intake"), Some(&1));
    assert_eq!(counts.get("shotPrep"), Some(&1));
    assert_eq!(counts.get("poseUpdate"), Some(&2));
    assert_eq!(counts.get("clearState"), None);

    // The path stops, so the last output is zero speeds.
    let last = *robot.outputs.borrow().last().unwrap();
    assert_eq!(last, ChassisSpeeds::default());
    assert_eq!(current_path_name(), "");
}

#[test]
fn flips_the_path_for_the_other_alliance() {
    let (end, _, counts, _) = run_path("TBTT 1", true);
    let goal = Pose2d::from_xy(
        16.54 - 2.26,
        8.07 - 5.8,
        Rotation2d::from_degrees(135.0 - 180.0),
    );
    assert!(distance(end, goal) < 0.1, "ended at {end:?}");
    assert_eq!(counts.get("intake"), Some(&1));
}

#[test]
fn reports_the_path_name_while_running() {
    use_deploy_folder();
    let clock = FakeClock::install();
    let path = PathPlannerPath::from_path_file("1 Meter").unwrap();
    let robot = SimRobot::new(path.get_starting_holonomic_pose().unwrap());
    robot.configure();

    let log: Rc<RefCell<Vec<String>>> = Rc::default();
    let (current_log, target_log, path_log) = (log.clone(), log.clone(), log.clone());
    PathPlannerLogging::set_log_current_pose_callback(Some(Box::new(move |_| {
        current_log.borrow_mut().push("current".into());
    })));
    PathPlannerLogging::set_log_target_pose_callback(Some(Box::new(move |_| {
        target_log.borrow_mut().push("target".into());
    })));
    PathPlannerLogging::set_log_active_path_callback(Some(Box::new(move |poses| {
        path_log.borrow_mut().push(format!("path {}", poses.len()));
    })));

    let command = AutoBuilder::follow_path(path.clone());
    let scheduler = CommandScheduler::get_instance();
    scheduler.schedule(&command);
    clock.advance(DT);
    scheduler.run();
    assert_eq!(current_path_name(), "1 Meter");
    while command.is_scheduled() {
        clock.advance(DT);
        scheduler.run();
        robot.step();
    }
    assert_eq!(current_path_name(), "");

    let log = log.borrow();
    assert_eq!(
        log.first().map(String::as_str),
        Some(&*format!("path {}", path.num_points()))
    );
    assert_eq!(log.last().map(String::as_str), Some("path 0"));
    assert!(log.iter().filter(|e| *e == "current").count() > 10);
    assert!(log.iter().filter(|e| *e == "target").count() > 10);
    PathPlannerLogging::clear_logging_callbacks();
}

#[test]
fn interrupting_a_path_does_not_stop_the_robot() {
    use_deploy_folder();
    let clock = FakeClock::install();
    let path = PathPlannerPath::from_path_file("5 Meter").unwrap();
    let robot = SimRobot::new(path.get_starting_holonomic_pose().unwrap());
    robot.configure();

    let command = AutoBuilder::follow_path(path);
    let scheduler = CommandScheduler::get_instance();
    scheduler.schedule(&command);
    for _ in 0..60 {
        clock.advance(DT);
        scheduler.run();
        robot.step();
    }
    command.cancel();
    let last = *robot.outputs.borrow().last().unwrap();
    assert!(last.vx.abs() > 0.5, "the robot kept its speed: {last:?}");
}

#[test]
fn an_auto_runs_its_paths_in_order_and_resets_odometry() {
    use_deploy_folder();
    let clock = FakeClock::install();
    let robot = SimRobot::new(Pose2d::from_xy(9.0, 1.0, Rotation2d::from_degrees(77.0)));
    robot.configure();

    let auto = PathPlannerAuto::new("TBTB Full", false);
    let start = auto.starting_pose().unwrap();
    let command = auto.into_command();
    assert_eq!(command.get_name(), "TBTB Full");

    let scheduler = CommandScheduler::get_instance();
    scheduler.schedule(&command);
    clock.advance(DT);
    scheduler.run();
    // The first loop already reset the pose and took a step along the first path.
    assert!(
        distance(robot.pose.get(), start) < 0.1,
        "{:?}",
        robot.pose.get()
    );

    let mut loops = 0;
    while command.is_scheduled() {
        clock.advance(DT);
        scheduler.run();
        robot.step();
        loops += 1;
        assert!(loops < 5000);
    }
    let last_path = PathPlannerPath::from_path_file("1st-TBTB 3").unwrap();
    let goal = last_path.get_all_path_points().last().unwrap().position;
    assert!(robot.pose.get().translation.distance(goal) < 0.15);
}

#[test]
fn a_missing_auto_does_nothing() {
    use_deploy_folder();
    let robot = SimRobot::new(Pose2d::ZERO);
    robot.configure();
    let auto = PathPlannerAuto::new("no such auto", false);
    assert!(auto.starting_pose().is_none());
    assert_eq!(auto.command().get_name(), "no such auto");
    assert!(auto.command().get_requirements().is_empty());
}

#[test]
#[should_panic(expected = "AutoBuilder was not configured")]
fn an_auto_needs_the_auto_builder() {
    use_deploy_folder();
    AutoBuilder::reset_for_testing();
    PathPlannerAuto::new("TBTB 1", false);
}

#[test]
fn reset_odom_flips_for_red() {
    use_deploy_folder();
    let clock = FakeClock::install();
    let robot = SimRobot::new(Pose2d::ZERO);
    robot.configure();
    let blue = Pose2d::from_xy(3.0, 2.0, Rotation2d::from_degrees(20.0));

    let scheduler = CommandScheduler::get_instance();
    let blue_command = AutoBuilder::reset_odom(blue);
    scheduler.schedule(&blue_command);
    clock.advance(DT);
    scheduler.run();
    assert_eq!(robot.pose.get(), blue);

    robot.flip.set(true);
    let red_command = AutoBuilder::reset_odom(blue);
    scheduler.schedule(&red_command);
    clock.advance(DT);
    scheduler.run();
    assert_eq!(robot.pose.get(), FlippingUtil::flip_field_pose(blue));
}

#[test]
fn lists_the_autos_in_the_deploy_folder() {
    use_deploy_folder();
    let names = AutoBuilder::get_all_auto_names();
    assert_eq!(names.len(), 32);
    assert!(names.contains(&"TBTB Full".to_string()));
    assert!(names.windows(2).all(|w| w[0] <= w[1]));
}

#[test]
fn pathfinding_drives_to_the_pose_and_the_warmups_finish() {
    use_deploy_folder();
    let clock = FakeClock::install();
    let robot = SimRobot::new(Pose2d::from_xy(2.0, 2.0, Rotation2d::ZERO));
    robot.configure();

    let target = Pose2d::from_xy(14.0, 6.0, Rotation2d::from_degrees(180.0));
    let constraints =
        PathConstraints::new(3.0, 3.0, 540.0_f64.to_radians(), 720.0_f64.to_radians());
    let command = AutoBuilder::pathfind_to_pose_flipped(target, constraints, 0.0);
    run_until_done(&clock, &robot, &command, true);
    let end = robot.pose.get();
    assert!(distance(end, target) < 0.6, "ended at {end:?}");
    assert_eq!(
        *robot.outputs.borrow().last().unwrap(),
        ChassisSpeeds::default()
    );

    // The red alliance flips the target.
    robot.pose.set(Pose2d::from_xy(14.0, 6.0, Rotation2d::ZERO));
    robot.flip.set(true);
    let command = AutoBuilder::pathfind_to_pose_flipped(target, constraints, 0.0);
    run_until_done(&clock, &robot, &command, true);
    let flipped = FlippingUtil::flip_field_pose(target);
    assert!(
        distance(robot.pose.get(), flipped) < 0.6,
        "ended at {:?}",
        robot.pose.get()
    );

    run_until_done(&clock, &robot, &FollowPathCommand::warmup_command(), false);
    run_until_done(&clock, &robot, &PathfindingCommand::warmup_command(), true);
}

#[test]
fn pathfinding_does_nothing_when_already_at_the_target() {
    use_deploy_folder();
    let clock = FakeClock::install();
    let robot = SimRobot::new(Pose2d::from_xy(5.0, 4.0, Rotation2d::ZERO));
    robot.configure();
    let command = AutoBuilder::pathfind_to_pose(
        Pose2d::from_xy(5.2, 4.1, Rotation2d::ZERO),
        PathConstraints::new(3.0, 3.0, 4.0, 4.0),
        0.0,
    );
    let seconds = run_until_done(&clock, &robot, &command, false);
    assert!(seconds <= 2.0 * DT);
    // Zero speeds when it starts, and again when it ends.
    let outputs = robot.outputs.borrow();
    assert_eq!(outputs.len(), 2);
    assert!(outputs.iter().all(|s| *s == ChassisSpeeds::default()));
}
