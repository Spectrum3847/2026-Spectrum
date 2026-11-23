//! Simulation of a roller (flywheel) mechanism driven by a single Kraken X60 motor. Advances
//! the TalonFX sim state on the shared `SimLoop` thread and animates the roller in a
//! `Mechanism2d` canvas, coloring by spin direction. Implements [`Mountable`] so the roller
//! axle can follow a parent [`Mount`](super::mount::Mount).

use super::circle::Circle;
use super::mountable::Mountable;
use super::roller_config::RollerConfig;
use super::sim_loop;
use super::sim_motor::{self, MotorSim};
use phoenix6::TalonFX;
use std::f64::consts::TAU;
use std::rc::Rc;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use wpilib::dashboard::{Color8Bit, Ligament, Mechanism2d, MechanismLigament2d, MechanismRoot2d};
use wpimath::sim::FlywheelSim;
use wpimath::system::{DcMotor, create_flywheel_system};
use wpimath::units::inches_to_meters;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Spin {
    Reverse,
    Forward,
    Stopped,
}

struct State {
    config: RollerConfig,
    roller_sim: FlywheelSim,
    /// Axle position, viz angle in degrees and spin color, set by each update.
    axle_x: f64,
    axle_y: f64,
    viz_angle_degrees: f64,
    spin: Spin,
    updated: bool,
}

/// The physics half of a [`RollerSim`], shared with the `SimLoop` thread.
pub struct RollerCore {
    state: Mutex<State>,
    motor: Box<dyn MotorSim>,
}

impl Mountable for RollerCore {}

impl RollerCore {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// One sim tick: steps the flywheel and feeds the rotor state back.
    fn update(&self, dt: f64) {
        let mut state = self.lock();
        state.roller_sim.set_input(self.motor.motor_voltage());
        state.roller_sim.update(dt);

        // FlywheelSim reports mechanism-side velocity; the rotor turns gear_ratio times
        // faster.
        let rotor_rotations_per_second =
            state.roller_sim.angular_velocity_rad_per_sec() / TAU * state.config.gear_ratio;
        self.motor.set_rotor_velocity(rotor_rotations_per_second);
        self.motor
            .add_rotor_position(rotor_rotations_per_second * dt);

        if state.config.is_mounted() {
            state.axle_x = self.updated_x_for(&state.config);
            state.axle_y = self.updated_y_for(&state.config);
        } else {
            state.axle_x = state.config.initial_x;
            state.axle_y = state.config.initial_y;
        }

        // Scale the drawn angle down so a spinning roller reads as a blur rather than a
        // strobe. Java: the rpm is passed to toDegrees as if it were radians.
        let rpm = state.roller_sim.angular_velocity_rpm() / 2.0;
        state.viz_angle_degrees += rpm.to_degrees() * dt * 0.1;

        let velocity = state.roller_sim.angular_velocity_rad_per_sec();
        state.spin = if velocity < -1.0 {
            Spin::Reverse
        } else if velocity > 1.0 {
            Spin::Forward
        } else {
            Spin::Stopped
        };
        state.updated = true;
    }
}

/// Canvas handles, used on the robot loop thread only.
struct Viz {
    roller_axle: MechanismRoot2d,
    roller_viz: MechanismLigament2d,
    roller: Circle,
}

#[derive(Clone)]
pub struct RollerSim {
    core: Arc<RollerCore>,
    viz: Rc<Viz>,
}

impl RollerSim {
    /// `name` prefixes this sim's Mechanism2d element labels.
    pub fn new(config: RollerConfig, mech: &Mechanism2d, motor: &TalonFX, name: &str) -> Self {
        let motor = sim_motor::sim_state(motor, config.reversed_linkage);
        Self::with_motor_sim(config, mech, Box::new(motor), name, true)
    }

    /// [`new`](Self::new) with any [`MotorSim`]. `register` false skips the `SimLoop`, for
    /// tests that step the sim by hand.
    pub fn with_motor_sim(
        config: RollerConfig,
        mech: &Mechanism2d,
        motor: Box<dyn MotorSim>,
        name: &str,
        register: bool,
    ) -> Self {
        let kraken = DcMotor::kraken_x60_foc(1);
        let flywheel_system = create_flywheel_system(kraken, config.sim_moi, config.gear_ratio);
        let roller_sim = FlywheelSim::new(flywheel_system, kraken);

        let roller_axle = mech.get_root(&format!("{name} Axle"), 0.0, 0.0);

        let roller_viz = roller_axle.append(
            Ligament::new(
                &format!("{name} Roller"),
                inches_to_meters(config.roller_diameter_inches) / 2.0,
                0.0,
            )
            .with_weight(5.0)
            .with_color(Color8Bit::WHITE),
        );

        let roller = Circle::new(
            config.background_lines,
            config.roller_diameter_inches,
            name,
            &roller_axle,
            mech,
        );

        let core = Arc::new(RollerCore {
            state: Mutex::new(State {
                config,
                roller_sim,
                axle_x: 0.0,
                axle_y: 0.0,
                viz_angle_degrees: 0.0,
                spin: Spin::Stopped,
                updated: false,
            }),
            motor,
        });
        let sim = Self {
            core,
            viz: Rc::new(Viz {
                roller_axle,
                roller_viz,
                roller,
            }),
        };
        if register {
            let core = sim.core.clone();
            sim_loop::register(move |dt| core.update(dt));
        }
        let drawn = sim.clone();
        sim_loop::register_draw(move || drawn.draw());
        sim
    }

    /// One sim tick: steps the flywheel and feeds the rotor state back.
    pub fn update(&self, dt: f64) {
        self.core.update(dt);
    }

    /// Moves the axle, spins the marker and colors the ring from the latest physics state.
    pub fn draw(&self) {
        let (axle_x, axle_y, viz_angle, spin, rev, off, fwd) = {
            let state = self.core.lock();
            if !state.updated {
                return;
            }
            (
                state.axle_x,
                state.axle_y,
                state.viz_angle_degrees,
                state.spin,
                state.config.rev_color,
                state.config.off_color,
                state.config.fwd_color,
            )
        };
        let viz = &self.viz;
        viz.roller_axle.set_position(axle_x, axle_y);
        viz.roller_viz.set_angle(viz_angle);
        match spin {
            Spin::Reverse => viz.roller.set_half_background(rev, off),
            Spin::Forward => viz.roller.set_background_color(fwd),
            Spin::Stopped => viz.roller.set_background_color(off),
        }
    }
}

impl Mountable for RollerSim {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spectrum_lib::sim::linear_config::LinearConfig;
    use crate::spectrum_lib::sim::linear_sim::LinearSim;
    use crate::spectrum_lib::sim::sim_motor::test_support::FakeMotor;

    const DT: f64 = 0.005;

    fn build(config: RollerConfig) -> (RollerSim, FakeMotor) {
        let motor = FakeMotor::default();
        let mech = Mechanism2d::new(2.0, 2.0);
        let sim =
            RollerSim::with_motor_sim(config, &mech, Box::new(motor.clone()), "Roller", false);
        (sim, motor)
    }

    fn reference(config: &RollerConfig) -> FlywheelSim {
        let kraken = DcMotor::kraken_x60_foc(1);
        FlywheelSim::new(
            create_flywheel_system(kraken, config.sim_moi, config.gear_ratio),
            kraken,
        )
    }

    fn ring_colors(sim: &RollerSim) -> Vec<String> {
        sim.viz
            .roller
            .circle_background()
            .iter()
            .map(|l| l.color())
            .collect()
    }

    #[test]
    fn step_matches_the_wpimath_flywheel_sim_and_accumulates_rotor_position() {
        let config = RollerConfig::new(2.0)
            .set_gear_ratio(4.0)
            .set_sim_moi(0.005);
        let mut expected = reference(&config);
        let (sim, motor) = build(config.clone());
        let mut position = 0.0;
        for i in 0..200 {
            let volts = if i < 100 { 8.0 } else { -5.0 };
            motor.set_voltage(volts);
            expected.set_input(volts);
            expected.update(DT);
            sim.update(DT);

            let rps = expected.angular_velocity_rad_per_sec() / TAU * config.gear_ratio;
            position += rps * DT;
            let state = motor.state();
            assert_eq!(state.velocity, rps);
            assert!((state.position - position).abs() < 1e-12);
            assert_eq!(
                state.position_sets, 0,
                "roller adds position, never sets it"
            );
        }
    }

    #[test]
    fn spin_state_selects_the_ring_colors() {
        let config = RollerConfig::new(2.0);
        let (fwd, off, rev) = (
            config.fwd_color.hex(),
            config.off_color.hex(),
            config.rev_color.hex(),
        );
        let (sim, motor) = build(config);
        let lines = sim.viz.roller.circle_background().len();

        sim.update(DT);
        sim.draw();
        assert_eq!(ring_colors(&sim), vec![off.clone(); lines]);

        motor.set_voltage(10.0);
        for _ in 0..100 {
            sim.update(DT);
        }
        sim.draw();
        assert_eq!(ring_colors(&sim), vec![fwd; lines]);

        motor.set_voltage(-10.0);
        for _ in 0..400 {
            sim.update(DT);
        }
        sim.draw();
        let colors = ring_colors(&sim);
        for (i, color) in colors.iter().enumerate() {
            assert_eq!(*color, if i % 2 == 0 { &rev } else { &off }.as_str());
        }
    }

    #[test]
    fn viz_angle_accumulates_with_the_scaled_rpm_and_axle_stays_put_unmounted() {
        let config = RollerConfig::new(2.0).set_position(0.4, 0.6);
        let (sim, motor) = build(config);
        motor.set_voltage(6.0);
        let mut angle = 0.0;
        let mut expected = reference(&RollerConfig::new(2.0));
        expected.set_input(6.0);
        for _ in 0..50 {
            expected.update(DT);
            angle += (expected.angular_velocity_rpm() / 2.0).to_degrees() * DT * 0.1;
            sim.update(DT);
        }
        sim.draw();
        assert!((sim.viz.roller_viz.angle() - angle).abs() < 1e-9);
        let state = sim.core.lock();
        assert_eq!((state.axle_x, state.axle_y), (0.4, 0.6));
    }

    #[test]
    fn axle_follows_a_mounted_stage() {
        let motor = FakeMotor::default();
        let mech = Mechanism2d::new(2.0, 2.0);
        let stage = LinearSim::with_motor_sim(
            LinearConfig::new(0.5, 0.1, 5.0, 0.0243).set_max_height(30.0),
            &mech,
            Box::new(motor.clone()),
            "Lift",
            false,
        );
        let mounted = RollerConfig::new(2.0)
            .set_position(0.6, 0.4)
            .set_mount_linear(Some(&stage));
        assert_eq!(mounted.init_mount_x, 0.5);
        let (roller, _) = build(mounted.clone());
        motor.set_voltage(9.0);
        for _ in 0..100 {
            stage.update(DT);
            roller.update(DT);
        }
        let state = roller.core.lock();
        assert!((state.axle_x - roller.updated_x_for(&mounted)).abs() < 1e-12);
        assert!((state.axle_y - roller.updated_y_for(&mounted)).abs() < 1e-12);
        assert!(state.axle_y > 0.4, "axle rose with the stage");
    }

    #[test]
    fn config_keeps_java_defaults() {
        let config = RollerConfig::new(3.0);
        assert_eq!(config.roller_diameter_inches, 3.0);
        assert_eq!(
            (config.background_lines, config.gear_ratio, config.sim_moi),
            (36, 5.0, 0.01)
        );
        assert_eq!(config.fwd_color, Color8Bit::GREEN);
        assert_eq!(config.rev_color, Color8Bit::RED);
        assert!(!config.set_mount_arm(None).is_mounted());
    }
}
