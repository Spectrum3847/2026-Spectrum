//! Physical properties, display colors, canvas position, and the optional parent mount for a
//! `RollerSim`.

use super::arm_sim::ArmSim;
use super::linear_sim::LinearSim;
use super::mount::Mount;
use super::mountable::MountedConfig;
use std::sync::Arc;
use wpilib::dashboard::Color8Bit;

#[derive(Clone)]
pub struct RollerConfig {
    /// Roller diameter in inches, used for both the physics model and the drawing.
    pub roller_diameter_inches: f64,
    pub background_lines: i32,
    /// Motor rotations per roller revolution.
    pub gear_ratio: f64,
    /// Roller moment of inertia in kg*m^2, fed to the flywheel sim.
    pub sim_moi: f64,
    /// Color below the spin threshold the sim treats as stationary.
    pub off_color: Color8Bit,
    /// Color while the roller spins in the forward direction.
    pub fwd_color: Color8Bit,
    /// Color while the roller spins in the reverse direction.
    pub rev_color: Color8Bit,
    /// Initial axle X on the Mechanism2d canvas, in metres.
    pub initial_x: f64,
    /// Initial axle Y on the Mechanism2d canvas, in metres.
    pub initial_y: f64,
    /// The parent mount this roller is attached to.
    pub mount: Option<Arc<dyn Mount>>,
    /// X position of the mount at simulation start, in metres.
    pub init_mount_x: f64,
    /// Y position of the mount at simulation start, in metres.
    pub init_mount_y: f64,
    /// Angle of the mount at simulation start, in radians.
    pub init_mount_angle: f64,
    /// True when the sim geometry travels opposite the motor's positive direction.
    pub reversed_linkage: bool,
}

impl RollerConfig {
    pub fn new(diameter_inches: f64) -> Self {
        Self {
            roller_diameter_inches: diameter_inches,
            background_lines: 36,
            gear_ratio: 5.0,
            sim_moi: 0.01,
            off_color: Color8Bit::BLACK,
            fwd_color: Color8Bit::GREEN,
            rev_color: Color8Bit::RED,
            initial_x: 0.0,
            initial_y: 0.0,
            mount: None,
            init_mount_x: 0.0,
            init_mount_y: 0.0,
            init_mount_angle: 0.0,
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

    pub fn set_gear_ratio(mut self, ratio: f64) -> Self {
        self.gear_ratio = ratio;
        self
    }

    pub fn set_sim_moi(mut self, moi: f64) -> Self {
        self.sim_moi = moi;
        self
    }

    pub fn set_position(mut self, x: f64, y: f64) -> Self {
        self.initial_x = x;
        self.initial_y = y;
        self
    }

    /// Attaches this roller to a `LinearSim` so its axle follows that stage. `None` leaves
    /// this roller unmounted.
    pub fn set_mount_linear(mut self, sim: Option<&LinearSim>) -> Self {
        if let Some(sim) = sim {
            let config = sim.config();
            self.mount = Some(sim.mount());
            self.init_mount_x = config.initial_x;
            self.init_mount_y = config.initial_y;
            self.init_mount_angle = config.angle.to_radians();
        }
        self
    }

    /// Attaches this roller to an `ArmSim` so its axle follows the arm tip. `None` leaves
    /// this roller unmounted.
    pub fn set_mount_arm(mut self, sim: Option<&ArmSim>) -> Self {
        if let Some(sim) = sim {
            let config = sim.config();
            self.mount = Some(sim.mount());
            self.init_mount_x = config.initial_x;
            self.init_mount_y = config.initial_y;
            self.init_mount_angle = config.starting_angle;
        }
        self
    }
}

impl MountedConfig for RollerConfig {
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
