//! The state that the odometry thread keeps: pose estimate, speeds, module states, loop timing
//! and DAQ counts. Pure logic. The thread in `drivetrain` feeds it already-read signal values.
//!
//! CTRE compiles the thread body. This follows what the headers declare (`SwerveDriveState`,
//! `ControlParameters`, the DAQ counters, `IsOdometryValid`) and the pre-2025 Java source of the
//! same thread.

use crate::request::NUM_MODULES;
use wpimath::filter::LinearFilter;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::kinematics::{
    ChassisSpeeds, SwerveDriveKinematics, SwerveModulePosition, SwerveModuleState,
};

pub use wpimath::estimator::SwerveDrivePoseEstimator;

/// A snapshot of the drivetrain (`SwerveDriveState`).
#[derive(Clone, Debug, Default)]
pub struct SwerveDriveState {
    pub pose: Pose2d,
    pub speeds: ChassisSpeeds,
    pub module_states: [SwerveModuleState; NUM_MODULES],
    pub module_targets: [SwerveModuleState; NUM_MODULES],
    pub module_positions: [SwerveModulePosition; NUM_MODULES],
    pub raw_heading: Rotation2d,
    /// Phoenix clock, seconds.
    pub timestamp: f64,
    /// Smoothed loop period, seconds.
    pub odometry_period: f64,
    pub successful_daqs: i32,
    pub failed_daqs: i32,
}

/// What one odometry cycle read.
#[derive(Clone, Debug)]
pub struct OdometryInput {
    /// Phoenix clock, seconds.
    pub time: f64,
    /// Latency-compensated Pigeon 2 yaw.
    pub yaw: Rotation2d,
    pub positions: [SwerveModulePosition; NUM_MODULES],
    pub states: [SwerveModuleState; NUM_MODULES],
    pub targets: [SwerveModuleState; NUM_MODULES],
    /// False when the wait for the signals failed or timed out.
    pub daq_ok: bool,
}

/// Median of the last three samples (WPILib `MedianFilter`), to drop single long cycles.
#[derive(Default)]
struct Median3 {
    samples: Vec<f64>,
}

impl Median3 {
    fn calculate(&mut self, value: f64) -> f64 {
        self.samples.push(value);
        if self.samples.len() > 3 {
            self.samples.remove(0);
        }
        let mut sorted = self.samples.clone();
        sorted.sort_by(f64::total_cmp);
        let mid = sorted.len() / 2;
        if sorted.len() % 2 == 1 {
            sorted[mid]
        } else {
            (sorted[mid - 1] + sorted[mid]) / 2.0
        }
    }
}

pub struct OdometryCore {
    kinematics: SwerveDriveKinematics<NUM_MODULES>,
    estimator: SwerveDrivePoseEstimator<NUM_MODULES>,
    peak_remover: Median3,
    low_pass: LinearFilter,
    last_time: f64,
    state: SwerveDriveState,
    operator_forward_direction: Rotation2d,
    latest_yaw: Rotation2d,
    latest_positions: [SwerveModulePosition; NUM_MODULES],
}

/// What a request needs, taken at the end of an update.
#[derive(Clone, Copy, Debug)]
pub struct ControlSnapshot {
    pub operator_forward_direction: Rotation2d,
    pub current_chassis_speed: ChassisSpeeds,
    pub current_pose: Pose2d,
    pub timestamp: f64,
    pub update_period: f64,
}

impl OdometryCore {
    /// `start_time` is the Phoenix clock now. `nominal_period` stands in for the loop period
    /// until a second sample exists.
    pub fn new(
        module_locations: [Translation2d; NUM_MODULES],
        initial_yaw: Rotation2d,
        initial_positions: [SwerveModulePosition; NUM_MODULES],
        odometry_std_devs: [f64; 3],
        vision_std_devs: [f64; 3],
        start_time: f64,
        nominal_period: f64,
    ) -> Self {
        let kinematics = SwerveDriveKinematics::new(module_locations);
        let estimator = SwerveDrivePoseEstimator::new(
            kinematics.clone(),
            initial_yaw,
            initial_positions,
            Pose2d::ZERO,
            odometry_std_devs,
            vision_std_devs,
        );
        Self {
            kinematics,
            estimator,
            peak_remover: Median3::default(),
            low_pass: LinearFilter::moving_average(50),
            last_time: start_time - nominal_period,
            state: SwerveDriveState::default(),
            operator_forward_direction: Rotation2d::ZERO,
            latest_yaw: initial_yaw,
            latest_positions: initial_positions,
        }
    }

    pub fn kinematics(&self) -> &SwerveDriveKinematics<NUM_MODULES> {
        &self.kinematics
    }

    pub fn state(&self) -> &SwerveDriveState {
        &self.state
    }

    pub fn is_odometry_valid(&self) -> bool {
        self.state.successful_daqs > 2
    }

    /// Runs the estimator on one cycle of readings and refreshes the cached state.
    pub fn update(&mut self, input: &OdometryInput) -> ControlSnapshot {
        let period = input.time - self.last_time;
        self.last_time = input.time;
        let averaged = self.low_pass.calculate(self.peak_remover.calculate(period));

        if input.daq_ok {
            self.state.successful_daqs += 1;
        } else {
            self.state.failed_daqs += 1;
        }

        let pose = self
            .estimator
            .update_with_time(input.time, input.yaw, input.positions);
        let speeds = self.kinematics.to_chassis_speeds(&input.states);

        self.latest_yaw = input.yaw;
        self.latest_positions = input.positions;
        self.state.pose = pose;
        self.state.speeds = speeds;
        self.state.module_states = input.states;
        self.state.module_targets = input.targets;
        self.state.module_positions = input.positions;
        self.state.raw_heading = input.yaw;
        self.state.timestamp = input.time;
        self.state.odometry_period = averaged;

        ControlSnapshot {
            operator_forward_direction: self.operator_forward_direction,
            current_chassis_speed: speeds,
            current_pose: pose,
            timestamp: input.time,
            update_period: period,
        }
    }

    fn refresh_cached_pose(&mut self) {
        self.state.pose = self.estimator.estimated_position();
    }

    /// Zeroes the pose with the given module positions, as `TareEverything` does.
    pub fn tare(&mut self, positions: [SwerveModulePosition; NUM_MODULES]) {
        self.latest_positions = positions;
        self.estimator
            .reset_position(self.latest_yaw, positions, Pose2d::ZERO);
        self.refresh_cached_pose();
    }

    pub fn reset_pose(&mut self, pose: Pose2d) {
        self.estimator.reset_pose(pose);
        self.refresh_cached_pose();
    }

    pub fn reset_translation(&mut self, translation: Translation2d) {
        self.estimator.reset_translation(translation);
        self.refresh_cached_pose();
    }

    pub fn reset_rotation(&mut self, rotation: Rotation2d) {
        self.estimator.reset_rotation(rotation);
        self.refresh_cached_pose();
    }

    /// Makes `rotation` the operator-forward direction of the current heading.
    pub fn seed_field_centric(&mut self, rotation: Rotation2d) {
        self.reset_rotation(rotation + self.operator_forward_direction);
    }

    pub fn set_operator_perspective_forward(&mut self, direction: Rotation2d) {
        self.operator_forward_direction = direction;
    }

    pub fn operator_forward_direction(&self) -> Rotation2d {
        self.operator_forward_direction
    }

    pub fn add_vision_measurement(&mut self, pose: Pose2d, timestamp: f64) {
        self.estimator.add_vision_measurement(pose, timestamp);
    }

    pub fn add_vision_measurement_with_std_devs(
        &mut self,
        pose: Pose2d,
        timestamp: f64,
        std_devs: [f64; 3],
    ) {
        self.estimator
            .add_vision_measurement_with_std_devs(pose, timestamp, std_devs);
    }

    pub fn set_vision_measurement_std_devs(&mut self, std_devs: [f64; 3]) {
        self.estimator.set_vision_measurement_std_devs(std_devs);
    }

    pub fn sample_pose_at(&self, timestamp: f64) -> Option<Pose2d> {
        self.estimator.sample_at(timestamp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HALF: f64 = 0.276;

    fn locations() -> [Translation2d; 4] {
        [
            Translation2d::new(HALF, HALF),
            Translation2d::new(HALF, -HALF),
            Translation2d::new(-HALF, HALF),
            Translation2d::new(-HALF, -HALF),
        ]
    }

    fn core() -> OdometryCore {
        OdometryCore::new(
            locations(),
            Rotation2d::ZERO,
            [SwerveModulePosition::default(); 4],
            [0.1; 3],
            [0.9; 3],
            100.0,
            0.004,
        )
    }

    fn input(time: f64, distance: f64, yaw: Rotation2d, speed: f64) -> OdometryInput {
        OdometryInput {
            time,
            yaw,
            positions: [SwerveModulePosition::new(distance, Rotation2d::ZERO); 4],
            states: [SwerveModuleState::new(speed, Rotation2d::ZERO); 4],
            targets: [SwerveModuleState::new(speed, Rotation2d::ZERO); 4],
            daq_ok: true,
        }
    }

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn driving_straight_advances_the_pose() {
        let mut core = core();
        for i in 1..=250 {
            let t = 100.0 + f64::from(i) * 0.004;
            core.update(&input(t, 2.0 * (t - 100.0), Rotation2d::ZERO, 2.0));
        }
        let state = core.state();
        assert!(near(state.pose.x(), 2.0));
        assert!(near(state.pose.y(), 0.0));
        assert!(near(state.speeds.vx, 2.0));
        assert!(near(state.timestamp, 101.0));
        assert_eq!(state.successful_daqs, 250);
        assert_eq!(state.failed_daqs, 0);
    }

    #[test]
    fn heading_comes_from_the_gyro() {
        let mut core = core();
        core.update(&input(100.004, 0.0, Rotation2d::from_degrees(90.0), 0.0));
        let state = core.state();
        assert!(near(state.pose.rotation.degrees(), 90.0));
        assert!(near(state.raw_heading.degrees(), 90.0));
        // Driving "forward" now moves along +Y.
        core.update(&input(100.008, 1.0, Rotation2d::from_degrees(90.0), 1.0));
        assert!(near(core.state().pose.y(), 1.0));
        assert!(near(core.state().pose.x(), 0.0));
    }

    #[test]
    fn failed_daqs_are_counted_and_validity_needs_three_good_ones() {
        let mut core = core();
        let mut bad = input(100.004, 0.0, Rotation2d::ZERO, 0.0);
        bad.daq_ok = false;
        core.update(&bad);
        assert_eq!(core.state().failed_daqs, 1);
        for i in 2..=3 {
            core.update(&input(
                100.0 + f64::from(i) * 0.004,
                0.0,
                Rotation2d::ZERO,
                0.0,
            ));
        }
        assert!(!core.is_odometry_valid());
        core.update(&input(100.016, 0.0, Rotation2d::ZERO, 0.0));
        assert!(core.is_odometry_valid());
    }

    #[test]
    fn odometry_period_is_a_smoothed_loop_time() {
        let mut core = core();
        for i in 1..=100 {
            core.update(&input(
                100.0 + f64::from(i) * 0.004,
                0.0,
                Rotation2d::ZERO,
                0.0,
            ));
        }
        assert!((core.state().odometry_period - 0.004).abs() < 1e-9);
        // One long cycle is dropped by the median.
        core.update(&input(100.4 + 0.05, 0.0, Rotation2d::ZERO, 0.0));
        assert!((core.state().odometry_period - 0.004).abs() < 1e-3);
    }

    #[test]
    fn snapshot_carries_the_cycle_period_and_pose() {
        let mut core = core();
        let snapshot = core.update(&input(100.01, 0.0, Rotation2d::ZERO, 0.0));
        assert!(near(snapshot.update_period, 0.014));
        assert_eq!(snapshot.timestamp, 100.01);
        let snapshot = core.update(&input(100.012, 0.0, Rotation2d::ZERO, 0.0));
        assert!(near(snapshot.update_period, 0.002));
    }

    #[test]
    fn reset_pose_updates_the_cached_state_at_once() {
        let mut core = core();
        core.update(&input(100.004, 0.0, Rotation2d::ZERO, 0.0));
        core.reset_pose(Pose2d::from_xy(3.0, 4.0, Rotation2d::from_degrees(30.0)));
        assert!(near(core.state().pose.x(), 3.0));
        assert!(near(core.state().pose.rotation.degrees(), 30.0));
        core.reset_translation(Translation2d::new(1.0, 2.0));
        assert!(near(core.state().pose.y(), 2.0));
        core.reset_rotation(Rotation2d::from_degrees(-45.0));
        assert!(near(core.state().pose.rotation.degrees(), -45.0));
    }

    #[test]
    fn seed_field_centric_adds_the_operator_direction() {
        let mut core = core();
        core.update(&input(100.004, 0.0, Rotation2d::ZERO, 0.0));
        core.set_operator_perspective_forward(Rotation2d::PI);
        core.seed_field_centric(Rotation2d::ZERO);
        assert!(near(
            core.state().pose.rotation.radians().abs(),
            std::f64::consts::PI
        ));
        assert_eq!(core.operator_forward_direction(), Rotation2d::PI);
    }

    #[test]
    fn tare_zeroes_the_pose_with_the_current_gyro() {
        let mut core = core();
        for i in 1..=10 {
            let t = 100.0 + f64::from(i) * 0.004;
            core.update(&input(t, f64::from(i), Rotation2d::from_degrees(20.0), 1.0));
        }
        assert!(core.state().pose.x() > 1.0);
        core.tare([SwerveModulePosition::new(0.0, Rotation2d::ZERO); 4]);
        assert!(near(core.state().pose.x(), 0.0));
        assert!(near(core.state().pose.rotation.degrees(), 0.0));
    }

    #[test]
    fn vision_pulls_the_estimate_and_sampling_returns_history() {
        let mut core = core();
        for i in 1..=100 {
            let t = 100.0 + f64::from(i) * 0.004;
            core.update(&input(t, 0.0, Rotation2d::ZERO, 0.0));
        }
        let before = core.sample_pose_at(100.2).expect("sample");
        assert!(near(before.x(), 0.0));
        core.add_vision_measurement_with_std_devs(
            Pose2d::from_xy(1.0, 0.0, Rotation2d::ZERO),
            100.3,
            [0.01, 0.01, 0.01],
        );
        // Very trusted vision: the next update shows the estimate almost all the way there.
        core.update(&input(100.404, 0.0, Rotation2d::ZERO, 0.0));
        let x = core.state().pose.x();
        assert!(x > 0.9, "x = {x}");
        // Before the vision update the history is untouched.
        assert!(near(core.sample_pose_at(100.1).expect("sample").x(), 0.0));
        assert!(core.sample_pose_at(100.4).expect("sample").x() > 0.9);
    }

    #[test]
    fn median_drops_single_outliers() {
        let mut m = Median3::default();
        assert_eq!(m.calculate(1.0), 1.0);
        assert_eq!(m.calculate(100.0), 50.5);
        assert_eq!(m.calculate(2.0), 2.0);
        assert_eq!(m.calculate(3.0), 3.0);
    }
}
