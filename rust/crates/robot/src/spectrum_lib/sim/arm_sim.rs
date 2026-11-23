//! Simulation of a single-jointed arm driven by one or more Kraken X60 motors. Advances the
//! TalonFX sim state on the shared `SimLoop` thread and animates the arm in a `Mechanism2d`
//! canvas. Implements [`Mount`] so other mechanisms can attach to the arm tip, and
//! [`Mountable`] so this arm can itself attach to a parent mount.

use super::arm_config::ArmConfig;
use super::mount::{Mount, MountType};
use super::mountable::Mountable;
use super::sim_loop;
use super::sim_motor::{self, MotorSim};
use phoenix6::TalonFX;
use std::f64::consts::TAU;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use wpilib::dashboard::{Ligament, Mechanism2d, MechanismLigament2d, MechanismRoot2d};
use wpimath::sim::SingleJointedArmSim;
use wpimath::system::DcMotor;

struct State {
    config: ArmConfig,
    arm_sim: SingleJointedArmSim,
    /// Arm ligament angle in degrees, set by each update.
    angle_degrees: f64,
    updated: bool,
}

/// The physics half of an [`ArmSim`], shared with the `SimLoop` thread and with child mounts.
pub struct ArmCore {
    state: Mutex<State>,
    motor: Box<dyn MotorSim>,
}

fn angle_of(state: &State) -> f64 {
    let angle_rads = state.arm_sim.angle_rads();
    if state.config.is_mounted() && !state.config.abs_angle {
        angle_rads + state.config.mount.as_ref().expect("mounted").angle()
    } else {
        angle_rads
    }
}

impl ArmCore {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// One sim tick: steps the arm physics and feeds the rotor state back.
    fn update(&self, dt: f64) {
        let mut state = self.lock();
        state.arm_sim.set_input(self.motor.motor_voltage());
        state.arm_sim.update(dt);

        let ratio = state.config.ratio;
        let rotations = (state.arm_sim.angle_rads() - state.config.starting_angle) / TAU;
        self.motor.set_raw_rotor_position(rotations * ratio);
        self.motor
            .set_rotor_velocity(state.arm_sim.velocity_rad_per_sec() / TAU * ratio);

        if state.config.is_mounted() {
            state.config.pivot_x = self.updated_x_for(&state.config);
            state.config.pivot_y = self.updated_y_for(&state.config);
        }
        state.angle_degrees = angle_of(&state).to_degrees();
        state.updated = true;
    }
}

impl Mountable for ArmCore {}

impl Mount for ArmCore {
    /// Always `Arm`, so children apply arm placement.
    fn mount_type(&self) -> MountType {
        MountType::Arm
    }

    /// How far the pivot has moved horizontally from its initial position, in metres.
    fn displacement_x(&self) -> f64 {
        let state = self.lock();
        state.config.pivot_x - state.config.initial_x
    }

    /// How far the pivot has moved vertically from its initial position, in metres.
    fn displacement_y(&self) -> f64 {
        let state = self.lock();
        state.config.pivot_y - state.config.initial_y
    }

    /// The arm's own angle, plus the parent mount's angle unless the config asked for an
    /// absolute angle, in radians.
    fn angle(&self) -> f64 {
        angle_of(&self.lock())
    }

    /// The X coordinate of the arm pivot, which a child attaches to, in metres.
    fn mount_x(&self) -> f64 {
        self.lock().config.pivot_x
    }

    /// The Y coordinate of the arm pivot, which a child attaches to, in metres.
    fn mount_y(&self) -> f64 {
        self.lock().config.pivot_y
    }
}

/// Canvas handles, used on the robot loop thread only.
#[derive(Clone)]
struct Viz {
    arm_pivot: MechanismRoot2d,
    arm_mech2d: MechanismLigament2d,
}

#[derive(Clone)]
pub struct ArmSim {
    core: Arc<ArmCore>,
    viz: Viz,
}

impl ArmSim {
    /// `name` prefixes this sim's Mechanism2d element labels.
    pub fn new(config: ArmConfig, mech: &Mechanism2d, motor: &TalonFX, name: &str) -> Self {
        let motor = sim_motor::sim_state(motor, config.reversed_linkage);
        Self::with_motor_sim(config, mech, Box::new(motor), name, true)
    }

    /// [`new`](Self::new) with any [`MotorSim`]. `register` false skips the `SimLoop`, for
    /// tests that step the sim by hand.
    pub fn with_motor_sim(
        config: ArmConfig,
        mech: &Mechanism2d,
        motor: Box<dyn MotorSim>,
        name: &str,
        register: bool,
    ) -> Self {
        let arm_sim = SingleJointedArmSim::new(
            DcMotor::kraken_x60_foc(config.num_motors),
            config.ratio,
            config.sim_moi,
            config.sim_cg_length,
            config.min_angle,
            config.max_angle,
            config.simulate_gravity,
            config.starting_angle,
        );

        let arm_pivot = mech.get_root(&format!("{name} Arm Pivot"), config.pivot_x, config.pivot_y);
        // Java: the starting ligament angle is min_angle in radians where degrees are
        // expected. The first update overwrites it.
        let arm_mech2d = arm_pivot.append(
            Ligament::new(&format!("{name} Arm"), config.length, config.min_angle)
                .with_weight(5.0)
                .with_color(config.color),
        );

        let core = Arc::new(ArmCore {
            state: Mutex::new(State {
                config,
                arm_sim,
                angle_degrees: 0.0,
                updated: false,
            }),
            motor,
        });
        let sim = Self {
            core,
            viz: Viz {
                arm_pivot,
                arm_mech2d,
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

    /// One sim tick: steps the arm physics and feeds the rotor state back.
    pub fn update(&self, dt: f64) {
        self.core.update(dt);
    }

    /// Moves the canvas ligament and pivot to the latest physics state.
    pub fn draw(&self) {
        let (angle_degrees, pivot_x, pivot_y) = {
            let state = self.core.lock();
            if !state.updated {
                return;
            }
            (
                state.angle_degrees,
                state.config.pivot_x,
                state.config.pivot_y,
            )
        };
        self.viz.arm_mech2d.set_angle(angle_degrees);
        self.viz.arm_pivot.set_position(pivot_x, pivot_y);
    }

    /// A copy of the config with the live pivot position.
    pub fn config(&self) -> ArmConfig {
        self.core.lock().config.clone()
    }

    /// The handle a child config attaches to.
    pub fn mount(&self) -> Arc<dyn Mount> {
        self.core.clone()
    }

    pub fn angle_rads(&self) -> f64 {
        self.core.lock().arm_sim.angle_rads()
    }

    pub fn displacement_x(&self) -> f64 {
        self.core.displacement_x()
    }

    pub fn displacement_y(&self) -> f64 {
        self.core.displacement_y()
    }

    /// The arm's own angle, plus the parent mount's angle unless the config asked for an
    /// absolute angle, in radians.
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

impl Mountable for ArmSim {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectrum_lib::sim::linear_config::LinearConfig;
    use crate::spectrum_lib::sim::linear_sim::LinearSim;
    use crate::spectrum_lib::sim::sim_motor::test_support::FakeMotor;
    use std::f64::consts::TAU;

    const DT: f64 = 0.005;

    fn config() -> ArmConfig {
        ArmConfig::new(0.5, 0.25, 100.0, 0.6, -10.0, 100.0, 0.0)
    }

    fn build(config: ArmConfig) -> (ArmSim, FakeMotor) {
        let motor = FakeMotor::default();
        let mech = Mechanism2d::new(2.0, 2.0);
        let sim = ArmSim::with_motor_sim(config, &mech, Box::new(motor.clone()), "Arm", false);
        (sim, motor)
    }

    fn reference(config: &ArmConfig) -> SingleJointedArmSim {
        SingleJointedArmSim::new(
            DcMotor::kraken_x60_foc(config.num_motors),
            config.ratio,
            config.sim_moi,
            config.sim_cg_length,
            config.min_angle,
            config.max_angle,
            config.simulate_gravity,
            config.starting_angle,
        )
    }

    #[test]
    fn step_matches_the_wpimath_arm_sim_and_feeds_the_rotor_back() {
        let config = config();
        let mut expected = reference(&config);
        let (sim, motor) = build(config.clone());
        for i in 0..200 {
            // Change the voltage mid-run: the sim reads it on every tick.
            let volts = if i < 100 { 6.0 } else { -3.0 };
            motor.set_voltage(volts);
            expected.set_input(volts);
            expected.update(DT);
            sim.update(DT);

            assert_eq!(sim.angle_rads(), expected.angle_rads());
            let state = motor.state();
            let ratio = config.ratio;
            assert_eq!(
                state.position,
                (expected.angle_rads() - config.starting_angle) / TAU * ratio
            );
            assert_eq!(
                state.velocity,
                expected.velocity_rad_per_sec() / TAU * ratio
            );
        }
        assert!(sim.angle_rads() > 0.0);
    }

    #[test]
    fn gravity_pulls_an_unpowered_arm_to_its_lower_limit() {
        let config = ArmConfig::new(0.0, 0.0, 100.0, 0.6, -10.0, 100.0, 45.0);
        let (sim, _) = build(config.clone());
        for _ in 0..4000 {
            sim.update(DT);
        }
        assert_eq!(sim.angle_rads(), config.min_angle);
    }

    #[test]
    fn unmounted_arm_keeps_its_pivot_and_reports_its_own_angle() {
        let (sim, motor) = build(config());
        motor.set_voltage(4.0);
        sim.update(DT);
        assert_eq!(sim.mount_x(), 0.5);
        assert_eq!(sim.mount_y(), 0.25);
        assert_eq!(sim.displacement_x(), 0.0);
        assert_eq!(sim.displacement_y(), 0.0);
        assert_eq!(sim.angle(), sim.angle_rads());
        assert_eq!(sim.core.mount_type(), MountType::Arm);
    }

    fn linear_parent() -> (LinearSim, FakeMotor) {
        let motor = FakeMotor::default();
        let mech = Mechanism2d::new(2.0, 2.0);
        let config = LinearConfig::new(0.2, 0.1, 5.0, 0.0243);
        let sim = LinearSim::with_motor_sim(config, &mech, Box::new(motor.clone()), "Lift", false);
        (sim, motor)
    }

    #[test]
    fn mounted_arm_follows_the_linear_stage() {
        let (parent, parent_motor) = linear_parent();
        let mounted = ArmConfig::new(0.5, 0.25, 100.0, 0.6, -10.0, 100.0, 0.0)
            .set_mount_linear(Some(&parent), true);
        assert!(mounted.is_mounted());
        assert_eq!(mounted.init_mount_x, 0.2);
        assert_eq!(mounted.init_mount_y, 0.1);
        assert!((mounted.init_mount_angle - 90.0_f64.to_radians()).abs() < 1e-15);
        let (child, _) = build(mounted.clone());

        parent_motor.set_voltage(8.0);
        for _ in 0..100 {
            parent.update(DT);
            child.update(DT);
        }
        let expected_x = child.updated_x_for(&mounted);
        let expected_y = child.updated_y_for(&mounted);
        assert!((child.mount_x() - expected_x).abs() < 1e-12);
        assert!((child.mount_y() - expected_y).abs() < 1e-12);
        assert!(child.mount_y() > 0.25, "pivot rose with the stage");
        assert!((child.displacement_y() - (child.mount_y() - 0.25)).abs() < 1e-12);
    }

    #[test]
    fn angle_adds_the_mount_angle_unless_absolute() {
        let (parent, _) = linear_parent();
        let base = ArmConfig::new(0.5, 0.25, 100.0, 0.6, -10.0, 100.0, 0.0);
        let (relative, _) = build(base.clone().set_mount_linear(Some(&parent), false));
        let (absolute, _) = build(base.set_mount_linear(Some(&parent), true));
        let stage = 90.0_f64.to_radians();
        assert!((relative.angle() - (relative.angle_rads() + stage)).abs() < 1e-12);
        assert_eq!(absolute.angle(), absolute.angle_rads());
    }

    #[test]
    fn mounting_to_an_arm_uses_its_starting_angle_and_none_leaves_it_unmounted() {
        let (parent, _) = build(ArmConfig::new(0.1, 0.2, 50.0, 0.4, 0.0, 90.0, 30.0));
        let config = config().set_mount_arm(Some(&parent), false);
        assert!(config.is_mounted());
        assert_eq!(config.init_mount_x, 0.1);
        assert_eq!(config.init_mount_y, 0.2);
        assert!((config.init_mount_angle - 30.0_f64.to_radians()).abs() < 1e-15);
        assert!(!config.abs_angle);
        assert!(!self::config().set_mount_arm(None, true).is_mounted());
    }

    #[test]
    fn config_converts_degrees_to_radians_and_keeps_java_defaults() {
        let config = ArmConfig::new(1.0, 2.0, 10.0, 0.5, -90.0, 180.0, 45.0);
        assert!((config.min_angle + std::f64::consts::FRAC_PI_2).abs() < 1e-15);
        assert!((config.max_angle - std::f64::consts::PI).abs() < 1e-15);
        assert!((config.starting_angle - std::f64::consts::FRAC_PI_4).abs() < 1e-15);
        assert_eq!((config.pivot_x, config.pivot_y), (1.0, 2.0));
        assert_eq!((config.sim_moi, config.sim_cg_length), (1.2, 0.2));
        assert_eq!(config.num_motors, 1);
        assert!(config.simulate_gravity && !config.reversed_linkage);
        assert_eq!(config.color, wpilib::dashboard::Color8Bit::BLUE);
    }

    #[test]
    fn draw_waits_for_the_first_update_then_follows_the_physics() {
        let (sim, motor) = build(config());
        // Java: the ligament starts at min_angle in radians read as degrees.
        let min_angle = sim.config().min_angle;
        sim.draw();
        assert_eq!(sim.viz.arm_mech2d.angle(), min_angle);

        motor.set_voltage(6.0);
        for _ in 0..50 {
            sim.update(DT);
        }
        sim.draw();
        assert_eq!(sim.viz.arm_mech2d.angle(), sim.angle().to_degrees());
    }
}
