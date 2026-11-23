//! Simulation of a linear (elevator-style) mechanism driven by one or more Kraken X60
//! motors. Advances the TalonFX sim state on the shared `SimLoop` thread and animates a
//! static backing ligament plus a moving stage ligament in a `Mechanism2d` canvas. Implements
//! [`Mount`] so other mechanisms can attach to the moving stage, and [`Mountable`] so this
//! stage can itself attach to a parent mount.

use super::linear_config::LinearConfig;
use super::mount::{Mount, MountType};
use super::mountable::Mountable;
use super::sim_loop;
use super::sim_motor::{self, MotorSim};
use phoenix6::TalonFX;
use std::f64::consts::PI;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use wpilib::dashboard::{Color8Bit, Ligament, Mechanism2d, MechanismLigament2d, MechanismRoot2d};
use wpimath::sim::ElevatorSim;
use wpimath::system::DcMotor;

struct State {
    config: LinearConfig,
    elevator_sim: ElevatorSim,
    /// Stage angle in degrees and the moving root position, set by each update.
    angle_degrees: f64,
    root_x: f64,
    root_y: f64,
    updated: bool,
}

/// The physics half of a [`LinearSim`], shared with the `SimLoop` thread and with child
/// mounts.
pub struct LinearCore {
    state: Mutex<State>,
    motor: Box<dyn MotorSim>,
}

/// Converts carriage travel (metres, or metres/second) to motor rotations (or rotations/s).
fn drum_rotations(config: &LinearConfig, meters: f64) -> f64 {
    (meters / (2.0 * PI * config.drum_radius)) * config.elevator_gearing
}

/// The stage angle in degrees, following the parent mount when mounted.
fn stage_angle_degrees(config: &LinearConfig) -> f64 {
    let Some(mount) = config.mount.as_ref() else {
        return config.angle;
    };
    match mount.mount_type() {
        MountType::Arm => config.angle + mount.angle().to_degrees(),
        MountType::Linear => config.angle + (mount.angle() - config.init_mount_angle).to_degrees(),
    }
}

fn angle_of(state: &State) -> f64 {
    match state.config.mount.as_ref() {
        Some(mount) => mount.angle() + state.config.angle.to_radians(),
        None => state.config.angle.to_radians(),
    }
}

impl LinearCore {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// One sim tick: steps the elevator and feeds the rotor state back.
    fn update(&self, dt: f64) {
        let mut state = self.lock();
        state.elevator_sim.set_input(self.motor.motor_voltage());
        state.elevator_sim.update(dt);

        let velocity = state.elevator_sim.velocity_meters_per_second();
        let position = state.elevator_sim.position_meters();
        self.motor
            .set_rotor_velocity(drum_rotations(&state.config, velocity));
        self.motor
            .set_raw_rotor_position(drum_rotations(&state.config, position));

        let displacement = position;
        let angle = stage_angle_degrees(&state.config);
        if state.config.is_mounted() {
            state.config.static_root_x = self.updated_x_for(&state.config);
            state.config.static_root_y = self.updated_y_for(&state.config);
        }
        state.angle_degrees = angle;
        // Only a mounted stage moves its static root, so unmounted this stays put.
        let radians = angle.to_radians();
        state.root_x = state.config.static_root_x + displacement * radians.cos();
        state.root_y = state.config.static_root_y + displacement * radians.sin();
        state.updated = true;
    }
}

impl Mountable for LinearCore {}

impl Mount for LinearCore {
    /// Always `Linear`, so children apply linear placement.
    fn mount_type(&self) -> MountType {
        MountType::Linear
    }

    /// Horizontal distance the carriage has travelled, projected along the stage angle, in
    /// metres.
    fn displacement_x(&self) -> f64 {
        let state = self.lock();
        let angle = stage_angle_degrees(&state.config);
        state.elevator_sim.position_meters() * angle.to_radians().cos()
            + (state.config.static_root_x - state.config.initial_x)
    }

    /// Vertical distance the carriage has travelled, projected along the stage angle, in
    /// metres.
    fn displacement_y(&self) -> f64 {
        let state = self.lock();
        let angle = stage_angle_degrees(&state.config);
        state.elevator_sim.position_meters() * angle.to_radians().sin()
            + (state.config.static_root_y - state.config.initial_y)
    }

    /// Stage angle in radians, plus the parent mount's angle when mounted.
    fn angle(&self) -> f64 {
        angle_of(&self.lock())
    }

    /// The X coordinate of this stage's static root, which a child attaches to, in metres.
    fn mount_x(&self) -> f64 {
        self.lock().config.static_root_x
    }

    /// The Y coordinate of this stage's static root, which a child attaches to, in metres.
    fn mount_y(&self) -> f64 {
        self.lock().config.static_root_y
    }
}

/// Canvas handles, used on the robot loop thread only.
#[derive(Clone)]
struct Viz {
    static_root: MechanismRoot2d,
    root: MechanismRoot2d,
    static_mech2d: MechanismLigament2d,
    elevator_mech2d: MechanismLigament2d,
}

#[derive(Clone)]
pub struct LinearSim {
    core: Arc<LinearCore>,
    viz: Viz,
}

impl LinearSim {
    /// `name` prefixes this sim's Mechanism2d element labels.
    pub fn new(config: LinearConfig, mech: &Mechanism2d, motor: &TalonFX, name: &str) -> Self {
        let motor = sim_motor::sim_state(motor, config.reversed_linkage);
        Self::with_motor_sim(config, mech, Box::new(motor), name, true)
    }

    /// [`new`](Self::new) with any [`MotorSim`]. `register` false skips the `SimLoop`, for
    /// tests that step the sim by hand.
    pub fn with_motor_sim(
        config: LinearConfig,
        mech: &Mechanism2d,
        motor: Box<dyn MotorSim>,
        name: &str,
        register: bool,
    ) -> Self {
        let elevator_sim = ElevatorSim::new(
            DcMotor::kraken_x60_foc(config.num_motors),
            config.elevator_gearing,
            config.carriage_mass_kg,
            config.drum_radius,
            config.min_height,
            config.max_height,
            true,
            0.0,
        );

        let static_root = mech.get_root(
            &format!("{name} 1StaticRoot"),
            config.initial_x,
            config.initial_y,
        );
        let static_mech2d = static_root.append(
            Ligament::new(
                &format!("{name} 1Static"),
                config.static_length,
                config.angle,
            )
            .with_weight(config.line_width)
            .with_color(Color8Bit::ORANGE),
        );

        let root = mech.get_root(&format!("{name} Root"), config.initial_x, config.initial_y);
        let elevator_mech2d = root.append(
            Ligament::new(name, config.moving_length, config.angle)
                .with_weight(config.line_width)
                .with_color(Color8Bit::BLACK),
        );

        let core = Arc::new(LinearCore {
            state: Mutex::new(State {
                config,
                elevator_sim,
                angle_degrees: 0.0,
                root_x: 0.0,
                root_y: 0.0,
                updated: false,
            }),
            motor,
        });
        let sim = Self {
            core,
            viz: Viz {
                static_root,
                root,
                static_mech2d,
                elevator_mech2d,
            },
        };
        if register {
            let core = sim.core.clone();
            sim_loop::register(move |dt| core.update(dt));
        }
        let drawn = sim.clone();
        sim_loop::register_draw(move || drawn.draw());
        sim
    }

    /// One sim tick: steps the elevator and feeds the rotor state back.
    pub fn update(&self, dt: f64) {
        self.core.update(dt);
    }

    /// Moves the canvas roots and ligaments to the latest physics state.
    pub fn draw(&self) {
        let (mounted, angle_degrees, static_x, static_y, root_x, root_y) = {
            let state = self.core.lock();
            if !state.updated {
                return;
            }
            (
                state.config.is_mounted(),
                state.angle_degrees,
                state.config.static_root_x,
                state.config.static_root_y,
                state.root_x,
                state.root_y,
            )
        };
        if mounted {
            self.viz.static_root.set_position(static_x, static_y);
            self.viz.static_mech2d.set_angle(angle_degrees);
            self.viz.elevator_mech2d.set_angle(angle_degrees);
        }
        self.viz.root.set_position(root_x, root_y);
    }

    /// A copy of the config with the live static root position.
    pub fn config(&self) -> LinearConfig {
        self.core.lock().config.clone()
    }

    /// The handle a child config attaches to.
    pub fn mount(&self) -> Arc<dyn Mount> {
        self.core.clone()
    }

    pub fn displacement_x(&self) -> f64 {
        self.core.displacement_x()
    }

    pub fn displacement_y(&self) -> f64 {
        self.core.displacement_y()
    }

    /// Stage angle in radians, plus the parent mount's angle when mounted.
    pub fn angle(&self) -> f64 {
        self.core.angle()
    }

    pub fn mount_x(&self) -> f64 {
        self.core.mount_x()
    }

    pub fn mount_y(&self) -> f64 {
        self.core.mount_y()
    }
}

impl Mountable for LinearSim {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectrum_lib::sim::arm_config::ArmConfig;
    use crate::spectrum_lib::sim::arm_sim::ArmSim;
    use crate::spectrum_lib::sim::sim_motor::test_support::FakeMotor;

    const DT: f64 = 0.005;

    fn config() -> LinearConfig {
        LinearConfig::new(0.5, 0.1, 5.0, 0.0243)
            .set_max_height(30.0)
            .set_carriage_mass(4.0)
    }

    fn build(config: LinearConfig) -> (LinearSim, FakeMotor) {
        let motor = FakeMotor::default();
        let mech = Mechanism2d::new(2.0, 2.0);
        let sim = LinearSim::with_motor_sim(config, &mech, Box::new(motor.clone()), "Lift", false);
        (sim, motor)
    }

    fn reference(config: &LinearConfig) -> ElevatorSim {
        ElevatorSim::new(
            DcMotor::kraken_x60_foc(config.num_motors),
            config.elevator_gearing,
            config.carriage_mass_kg,
            config.drum_radius,
            config.min_height,
            config.max_height,
            true,
            0.0,
        )
    }

    #[test]
    fn step_matches_the_wpimath_elevator_sim_and_feeds_the_rotor_back() {
        let config = config();
        let mut expected = reference(&config);
        let (sim, motor) = build(config.clone());
        for i in 0..200 {
            let volts = if i < 120 { 9.0 } else { -2.0 };
            motor.set_voltage(volts);
            expected.set_input(volts);
            expected.update(DT);
            sim.update(DT);

            let state = motor.state();
            let to_rotations =
                |meters: f64| meters / (2.0 * PI * config.drum_radius) * config.elevator_gearing;
            assert_eq!(state.position, to_rotations(expected.position_meters()));
            assert_eq!(
                state.velocity,
                to_rotations(expected.velocity_meters_per_second())
            );
        }
        assert!(motor.state().position > 0.0);
    }

    #[test]
    fn unmounted_stage_projects_travel_along_its_angle() {
        let config = config().set_angle(60.0);
        let (sim, motor) = build(config);
        motor.set_voltage(9.0);
        for _ in 0..100 {
            sim.update(DT);
        }
        let travel = sim.core.lock().elevator_sim.position_meters();
        assert!(travel > 0.0);
        let radians = 60.0_f64.to_radians();
        assert!((sim.displacement_x() - travel * radians.cos()).abs() < 1e-12);
        assert!((sim.displacement_y() - travel * radians.sin()).abs() < 1e-12);
        assert_eq!(sim.angle(), radians);
        assert_eq!((sim.mount_x(), sim.mount_y()), (0.5, 0.1));
        assert_eq!(sim.core.mount_type(), MountType::Linear);
    }

    #[test]
    fn draw_places_the_moving_root_along_the_stage() {
        let (sim, motor) = build(config());
        sim.draw();
        motor.set_voltage(9.0);
        for _ in 0..100 {
            sim.update(DT);
        }
        let travel = sim.core.lock().elevator_sim.position_meters();
        let (x, y) = {
            let state = sim.core.lock();
            (state.root_x, state.root_y)
        };
        let radians = 90.0_f64.to_radians();
        assert!((x - (0.5 + travel * radians.cos())).abs() < 1e-12);
        assert!((y - (0.1 + travel * radians.sin())).abs() < 1e-12);
        sim.draw();
    }

    #[test]
    fn stage_mounted_on_an_arm_follows_its_pivot_and_angle() {
        let arm_motor = FakeMotor::default();
        let mech = Mechanism2d::new(2.0, 2.0);
        let arm = ArmSim::with_motor_sim(
            ArmConfig::new(0.3, 0.2, 100.0, 0.6, -10.0, 100.0, 0.0),
            &mech,
            Box::new(arm_motor.clone()),
            "Arm",
            false,
        );
        let mounted = config().set_mount_arm(Some(&arm));
        assert_eq!(mounted.init_mount_x, 0.3);
        assert_eq!(mounted.init_mount_angle, 0.0);
        let (child, _) = build(mounted.clone());

        arm_motor.set_voltage(6.0);
        for _ in 0..100 {
            arm.update(DT);
            child.update(DT);
        }
        assert!(arm.angle() > 0.0);
        let state = child.core.lock();
        assert!((state.config.static_root_x - child.updated_x_for(&mounted)).abs() < 1e-12);
        assert!((state.config.static_root_y - child.updated_y_for(&mounted)).abs() < 1e-12);
        let expected_degrees = 90.0 + arm.angle().to_degrees();
        assert!((state.angle_degrees - expected_degrees).abs() < 1e-9);
    }

    #[test]
    fn stage_mounted_on_a_stage_subtracts_the_initial_mount_angle() {
        let (parent, parent_motor) = build(config().set_angle(30.0));
        let mounted = config().set_mount_linear(Some(&parent));
        assert!((mounted.init_mount_angle - 30.0_f64.to_radians()).abs() < 1e-15);
        let (child, _) = build(mounted);
        parent_motor.set_voltage(9.0);
        for _ in 0..50 {
            parent.update(DT);
            child.update(DT);
        }
        // Linear mounts add the change from the initial mount angle, which is zero here.
        assert!((child.core.lock().angle_degrees - 90.0).abs() < 1e-9);
        // getAngle() adds the whole mount angle, so the two differ.
        let expected = 30.0_f64.to_radians() + 90.0_f64.to_radians();
        assert!((child.angle() - expected).abs() < 1e-12);
    }

    #[test]
    fn config_setters_convert_inches_and_keep_java_defaults() {
        let config = LinearConfig::new(0.1, 0.2, 7.0, 0.03)
            .set_static_length(10.0)
            .set_moving_length(20.0)
            .set_max_height(40.0)
            .set_num_motors(2)
            .set_line_width(3.0);
        assert!((config.static_length - 0.254).abs() < 1e-12);
        assert!((config.moving_length - 0.508).abs() < 1e-12);
        assert!((config.max_height - 1.016).abs() < 1e-12);
        assert_eq!((config.num_motors, config.line_width), (2, 3.0));
        let defaults = LinearConfig::new(0.1, 0.2, 7.0, 0.03);
        assert_eq!(
            (defaults.static_length, defaults.moving_length),
            (20.0, 20.0)
        );
        assert_eq!(
            (defaults.angle, defaults.min_height, defaults.max_height),
            (90.0, 0.0, 10000.0)
        );
        assert_eq!((defaults.static_root_x, defaults.static_root_y), (0.1, 0.2));
        assert_eq!(defaults.color, Color8Bit::PURPLE);
        assert!(!defaults.set_mount_linear(None).is_mounted());
    }
}
