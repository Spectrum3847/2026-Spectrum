//! Physical properties, display settings, and the optional parent mount for an `ArmSim`.

use super::arm_sim::ArmSim;
use super::linear_sim::LinearSim;
use super::mount::Mount;
use super::mountable::MountedConfig;
use std::sync::Arc;
use wpilib::dashboard::Color8Bit;

#[derive(Clone)]
pub struct ArmConfig {
    /// Number of Kraken X60 motors driving the arm.
    pub num_motors: i32,
    /// Initial pivot X on the Mechanism2d canvas, in metres.
    pub initial_x: f64,
    /// Initial pivot Y on the Mechanism2d canvas, in metres.
    pub initial_y: f64,
    /// Current pivot X in metres, moved every tick while mounted.
    pub pivot_x: f64,
    /// Current pivot Y in metres, moved every tick while mounted.
    pub pivot_y: f64,
    /// Motor rotations required for one full revolution of the arm mechanism.
    pub ratio: f64,
    /// Visual length of the arm ligament, in metres.
    pub length: f64,
    /// Moment of inertia used by the physics simulation, in kg*m^2.
    pub sim_moi: f64,
    /// Distance from the pivot to the arm's centre of gravity, in metres.
    pub sim_cg_length: f64,
    /// Minimum allowable arm angle, in radians.
    pub min_angle: f64,
    /// Maximum allowable arm angle, in radians.
    pub max_angle: f64,
    /// Arm angle at the start of the simulation, in radians.
    pub starting_angle: f64,
    pub simulate_gravity: bool,
    /// The parent mount this arm is attached to.
    pub mount: Option<Arc<dyn Mount>>,
    /// X position of the mount at simulation start, in metres.
    pub init_mount_x: f64,
    /// Y position of the mount at simulation start, in metres.
    pub init_mount_y: f64,
    /// Angle of the mount at simulation start, in radians.
    pub init_mount_angle: f64,
    /// True keeps the arm's own angle absolute in the robot frame. False adds the parent
    /// mount's angle to it, in radians.
    pub abs_angle: bool,
    /// Color used to draw the arm ligament in the Mechanism2d canvas.
    pub color: Color8Bit,
    /// True when the sim geometry travels opposite the motor's positive direction.
    pub reversed_linkage: bool,
}

impl ArmConfig {
    /// Angle arguments arrive in degrees and are stored in radians. `ratio` is motor
    /// rotations per one full arm revolution.
    pub fn new(
        initial_x: f64,
        initial_y: f64,
        ratio: f64,
        length: f64,
        min_angle_degrees: f64,
        max_angle_degrees: f64,
        starting_angle_degrees: f64,
    ) -> Self {
        Self {
            num_motors: 1,
            initial_x,
            initial_y,
            pivot_x: initial_x,
            pivot_y: initial_y,
            ratio,
            length,
            sim_moi: 1.2,
            sim_cg_length: 0.2,
            min_angle: min_angle_degrees.to_radians(),
            max_angle: max_angle_degrees.to_radians(),
            starting_angle: starting_angle_degrees.to_radians(),
            simulate_gravity: true,
            mount: None,
            init_mount_x: 0.0,
            init_mount_y: 0.0,
            init_mount_angle: 0.0,
            abs_angle: false,
            color: Color8Bit::BLUE,
            reversed_linkage: false,
        }
    }

    pub fn is_mounted(&self) -> bool {
        self.mount.is_some()
    }

    /// Mirrors the simulated motor so the sim's travel matches the motor's positive
    /// direction. See [`sim_state`](super::sim_motor::sim_state).
    pub fn set_reversed_linkage(mut self, reversed_linkage: bool) -> Self {
        self.reversed_linkage = reversed_linkage;
        self
    }

    pub fn set_color(mut self, color: Color8Bit) -> Self {
        self.color = color;
        self
    }

    pub fn set_simulated_gravity(mut self, simulate_gravity: bool) -> Self {
        self.simulate_gravity = simulate_gravity;
        self
    }

    /// Attaches this arm to a `LinearSim` so its pivot follows that stage. `None` leaves
    /// the arm unmounted. `fixed_angle` true treats the arm angle as absolute, false adds
    /// the mount's current angle.
    pub fn set_mount_linear(mut self, sim: Option<&LinearSim>, fixed_angle: bool) -> Self {
        if let Some(sim) = sim {
            let config = sim.config();
            self.mount = Some(sim.mount());
            self.init_mount_x = config.initial_x;
            self.init_mount_y = config.initial_y;
            self.init_mount_angle = config.angle.to_radians();
            self.abs_angle = fixed_angle;
        }
        self
    }

    /// Attaches this arm to a parent `ArmSim` so its pivot follows that arm's tip. `None`
    /// leaves the arm unmounted. `abs_angle` true treats the arm angle as absolute, false
    /// adds the parent's current angle.
    pub fn set_mount_arm(mut self, sim: Option<&ArmSim>, abs_angle: bool) -> Self {
        if let Some(sim) = sim {
            let config = sim.config();
            self.mount = Some(sim.mount());
            self.init_mount_x = config.initial_x;
            self.init_mount_y = config.initial_y;
            self.init_mount_angle = config.starting_angle;
            self.abs_angle = abs_angle;
        }
        self
    }
}

impl MountedConfig for ArmConfig {
    fn mount(&self) -> &dyn Mount {
        self.mount.as_deref().expect("config is mounted")
    }

    fn initial_x(&self) -> f64 {
        self.initial_x
    }

    fn initial_y(&self) -> f64 {
        self.initial_y
    }

    fn init_mount_x(&self) -> f64 {
        self.init_mount_x
    }

    fn init_mount_y(&self) -> f64 {
        self.init_mount_y
    }

    fn init_mount_angle(&self) -> f64 {
        self.init_mount_angle
    }
}
