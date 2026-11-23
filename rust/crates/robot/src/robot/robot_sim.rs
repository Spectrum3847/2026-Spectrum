use crate::context;
use crate::rebuilt::fuel_physics_sim::FuelPhysicsSim;
use crate::robot::subsystems::super_structure::{CurrentSuperState, SuperStructure};
use commands::{Command, Commands, Trigger};
use phoenix6::utils;
use std::cell::RefCell;
use std::collections::hash_map::RandomState;
use std::hash::{BuildHasher, Hasher};
use std::rc::Rc;
use telemetry::telemetry;
use wpilib::dashboard::{Color8Bit, Ligament, Mechanism2d, smart_dashboard};
use wpimath::geometry::{
    Pose3d, Rotation2d, Rotation3d, Transform2d, Transform3d, Translation2d, Translation3d,
};
use wpimath::units::inches_to_meters;

const LEFT_VIEW_HEIGHT: f64 = 75.0;
const LEFT_VIEW_WIDTH: f64 = 75.0;

fn sim_robot_width() -> f64 {
    inches_to_meters(33.0)
}

fn sim_robot_length() -> f64 {
    inches_to_meters(32.75)
}

const SINGLE_LANE_BPS: i32 = 8;
const TIME_BETWEEN_BALL_LAUNCHES: f64 = 1.0 / SINGLE_LANE_BPS as f64;
const LAUNCHER_WIDTH: f64 = 24.0;
const NUM_OF_LANES: i32 = 4;
const LANE_WIDTH: f64 = LAUNCHER_WIDTH / NUM_OF_LANES as f64;
const LANE_1: f64 = -2.0 * LANE_WIDTH / 2.0;
const LANE_2: f64 = -LANE_WIDTH / 2.0;
const LANE_3: f64 = LANE_WIDTH / 2.0;
const LANE_4: f64 = 2.0 * LANE_WIDTH / 2.0;

/// The canvas the subsystem sims draw on, which Java keeps in `RobotSim.leftView`. Robot init
/// stores it in the context before the subsystems are built, on the real robot too.
pub fn new_left_view() -> Mechanism2d {
    Mechanism2d::new(
        inches_to_meters(LEFT_VIEW_WIDTH),
        inches_to_meters(LEFT_VIEW_HEIGHT),
    )
}

pub fn sim_launching() -> Trigger {
    Trigger::new(|| {
        let state = context::super_structure().get_current_super_state();
        utils::is_simulation()
            && (state == CurrentSuperState::LaunchWithSqueeze
                || state == CurrentSuperState::LaunchWithoutSqueeze
                || state == CurrentSuperState::LaunchWithSqueezeWithNoDelay)
    })
}

/// Hood pivot in the robot frame, in meters.
fn shooter_hood_pivot_point() -> Translation3d {
    Translation3d::new(-inches_to_meters(11.00), 0.0, inches_to_meters(19.237))
}

/// Draws the robot in the sim, top down and from the left.
pub struct RobotSim {
    ball_sim: Rc<RefCell<FuelPhysicsSim>>,
    robot_super_structure: Rc<SuperStructure>,
}

impl RobotSim {
    pub fn new(super_structure: Rc<SuperStructure>) -> RobotSim {
        let left_view = context::left_view();
        smart_dashboard::put_data("Sim/LeftView", &left_view);
        left_view.set_background_color(Color8Bit::LIGHT_GRAY);

        let mut ball_sim = FuelPhysicsSim::new("Sim/Fuel");
        ball_sim.enable();
        ball_sim.place_field_balls();
        config_ball_sim_robot(&mut ball_sim);
        let robot_sim = RobotSim {
            ball_sim: Rc::new(RefCell::new(ball_sim)),
            robot_super_structure: super_structure,
        };
        robot_sim.draw_robot();
        robot_sim
    }

    pub fn get_ball_sim(&self) -> &RefCell<FuelPhysicsSim> {
        &self.ball_sim
    }

    pub fn update_articulated_mechanisms(&self) {
        let intake_extension_pose = inches_to_meters(12.0)
            * self
                .robot_super_structure
                .intake_extension
                .get_position_percentage()
            / 100.0;
        let intake_pose3d = Pose3d::ZERO.transform_by(Transform3d::new(
            Translation3d::new(intake_extension_pose, 0.0, 0.0),
            Rotation3d::ZERO,
        ));

        let hood_angle_degrees = self.robot_super_structure.hood.get_position_degrees();
        let hood_pose3d = Pose3d::ZERO.rotate_around(
            shooter_hood_pivot_point(),
            Rotation3d::from_euler(0.0, -(hood_angle_degrees - 9.0).to_radians(), 0.0),
        );

        let mechanism_poses = [intake_pose3d, hood_pose3d];

        telemetry::log_struct_array("Sim/Components", &mechanism_poses);
    }

    pub fn draw_robot(&self) {
        self.draw_side_robot();
    }

    pub fn draw_side_robot(&self) {
        let robot_root = context::left_view().get_root(
            "Top Robot Root",
            inches_to_meters(LEFT_VIEW_WIDTH / 2.0 + 25.0),
            inches_to_meters(LEFT_VIEW_HEIGHT / 2.0 - 12.5),
        );

        let rect_width_in = 25.0;
        let rect_height_in = 40.0;
        let rect_width_m = inches_to_meters(rect_width_in);
        let rect_height_m = inches_to_meters(rect_height_in);

        let tr = robot_root.append(Ligament::new("TopEdge", rect_height_m, 180.0));
        let br = tr.append(Ligament::new("RightEdge", rect_width_m, 270.0));
        let bl = br.append(Ligament::new("BottomEdge", rect_height_m, 270.0));
        let ll = bl.append(Ligament::new("LeftEdge", rect_width_m, 270.0));

        let edge_color = Color8Bit::PURPLE;
        tr.set_color(edge_color);
        br.set_color(edge_color);
        bl.set_color(edge_color);
        ll.set_color(edge_color);
    }

    pub fn ball_sim_launch_fuel(&self) -> Command {
        if !utils::is_simulation() {
            return Commands::none();
        }
        let ball_sim = self.ball_sim.clone();
        Commands::defer(
            move || {
                let fuel_count = ball_sim.borrow().get_total_intaked();
                let num_to_launch_per_lane = fuel_count / NUM_OF_LANES;
                let lane = |offset: f64| {
                    let mut group = vec![Commands::wait_seconds(random() * 0.3)];
                    for _ in 0..num_to_launch_per_lane {
                        group.push(create_sim_ball_launch(ball_sim.clone(), offset));
                        group.push(Commands::wait_seconds(TIME_BETWEEN_BALL_LAUNCHES));
                    }
                    Commands::sequence(group)
                };
                let group1 = lane(LANE_1);
                let group2 = lane(LANE_2);
                let group3 = lane(LANE_3);
                let group4 = lane(LANE_4);
                Commands::parallel([group1, group2, group3, group4])
                    .with_name("RobotSim.ballSimLaunchFuel")
            },
            &[], // no subsystem requirements
        )
    }
}

fn create_sim_ball_launch(ball_sim: Rc<RefCell<FuelPhysicsSim>>, lane_offset: f64) -> Command {
    Commands::run_once(
        move || {
            let params = context::shot_calculator().get_parameters();
            let launch_speed = params.exit_speed_ms;
            let launch_angle = (90.0 - params.hood_angle).to_radians();
            let robot_pos = context::swerve().get_robot_pose();
            let launch_yaw = robot_pos.rotation.radians() + 180.0_f64.to_radians();
            let launch_rotation = Rotation3d::from_euler(0.0, -launch_angle, launch_yaw);
            let launch_velocity = Translation3d::from_polar(launch_speed, launch_rotation);

            let robot_to_launcher = Transform2d::new(
                Translation2d::new(inches_to_meters(-5.5), inches_to_meters(lane_offset)),
                Rotation2d::PI,
            );
            let launcher_xy = robot_pos.transform_by(robot_to_launcher).translation;
            let launcher_pose =
                Translation3d::new(launcher_xy.x, launcher_xy.y, inches_to_meters(17.5));
            ball_sim
                .borrow_mut()
                .launch_ball(launcher_pose, launch_velocity, 500.0);
        },
        &[],
    )
}

fn config_ball_sim_robot(ball_sim: &mut FuelPhysicsSim) {
    let bumper_height = inches_to_meters(4.56);
    let intake_width = inches_to_meters(10.0);
    let intake_length = inches_to_meters(28.5);
    let intake_x_min = inches_to_meters(0.0);
    let intake_x_max = sim_robot_width() / 2.0 + intake_width;
    let intake_y_min = -intake_length / 2.0;
    let intake_y_max = intake_length / 2.0;
    ball_sim.configure_robot(
        sim_robot_width(),
        sim_robot_length(),
        bumper_height,
        50,
        || context::swerve().get_robot_pose(),
        || context::swerve().get_current_robot_chassis_speeds(),
    );
    ball_sim.add_intake_zone(
        intake_x_min,
        intake_x_max,
        intake_y_min,
        intake_y_max,
        || context::super_structure().get_current_super_state() == CurrentSuperState::IntakeFuel,
    );
}

/// Java's `Math.random()`: uniform in [0, 1).
fn random() -> f64 {
    let bits = RandomState::new().build_hasher().finish() >> 11;
    bits as f64 / (1u64 << 53) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_size_constants_are_the_java_inch_values() {
        assert_eq!(sim_robot_width(), inches_to_meters(33.0));
        assert_eq!(sim_robot_length(), inches_to_meters(32.75));
    }

    #[test]
    fn lanes_split_the_launcher_width_evenly() {
        assert_eq!([LANE_1, LANE_2, LANE_3, LANE_4], [-6.0, -3.0, 3.0, 6.0]);
        assert_eq!(TIME_BETWEEN_BALL_LAUNCHES, 0.125);
    }

    #[test]
    fn random_is_in_the_unit_interval() {
        for _ in 0..1000 {
            let value = random();
            assert!((0.0..1.0).contains(&value));
        }
    }
}
