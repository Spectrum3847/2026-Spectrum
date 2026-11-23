//! Physical properties, Mechanism2d display settings, and the optional parent mount for a
//! `LinearSim`.

use super::arm_sim::ArmSim;
use super::linear_sim::LinearSim;
use super::mount::Mount;
use super::mountable::MountedConfig;
use std::sync::Arc;
use wpilib::dashboard::Color8Bit;
use wpimath::units::inches_to_meters;

#[derive(Clone)]
pub struct LinearConfig {
    /// Number of Kraken X60 motors driving the linear stage.
    pub num_motors: i32,
    /// Gear ratio between the motor and the elevator drum.
    pub elevator_gearing: f64,
    /// Mass of the moving carriage in kilograms, fed to the physics sim.
    pub carriage_mass_kg: f64,
    /// Drum radius in metres, which converts motor rotations into carriage travel.
    pub drum_radius: f64,
    /// Minimum travel height of the mechanism in metres.
    pub min_height: f64,
    /// Maximum travel height of the mechanism in metres.
    pub max_height: f64,
    /// Angle of the linear stage on the Mechanism2d canvas, in degrees, where 0 is
    /// horizontal, 90 is vertical, and positive is counter-clockwise.
    pub angle: f64,
    pub color: Color8Bit,
    /// Stroke width of the stage ligaments, in pixels.
    pub line_width: f64,
    /// Initial X of the static root on the canvas, in metres.
    pub initial_x: f64,
    /// Initial Y of the static root on the canvas, in metres.
    pub initial_y: f64,
    /// Current X of the static root in metres, moved every tick while mounted.
    pub static_root_x: f64,
    /// Current Y of the static root in metres, moved every tick while mounted.
    pub static_root_y: f64,
    /// Visual length of the non-moving stage ligament, in metres.
    // Java: the default of 20 is in metres, the setter stores metres from inches.
    pub static_length: f64,
    /// Visual length of the moving stage ligament, in metres.
    pub moving_length: f64,
    /// The parent mount this linear stage is attached to.
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

impl LinearConfig {
    pub fn new(x: f64, y: f64, gearing: f64, drum_radius: f64) -> Self {
        Self {
            num_motors: 1,
            elevator_gearing: gearing,
            carriage_mass_kg: 1.0,
            drum_radius,
            min_height: 0.0,
            max_height: 10000.0,
            angle: 90.0,
            color: Color8Bit::PURPLE,
            line_width: 10.0,
            initial_x: x,
            initial_y: y,
            static_root_x: x,
            static_root_y: y,
            static_length: 20.0,
            moving_length: 20.0,
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

    pub fn set_num_motors(mut self, num_motors: i32) -> Self {
        self.num_motors = num_motors;
        self
    }

    pub fn set_carriage_mass(mut self, carriage_mass_kg: f64) -> Self {
        self.carriage_mass_kg = carriage_mass_kg;
        self
    }

    pub fn set_angle(mut self, angle: f64) -> Self {
        self.angle = angle;
        self
    }

    pub fn set_color(mut self, color: Color8Bit) -> Self {
        self.color = color;
        self
    }

    pub fn set_line_width(mut self, line_width: f64) -> Self {
        self.line_width = line_width;
        self
    }

    /// Sets the static ligament's visual length. Takes inches and stores metres.
    pub fn set_static_length(mut self, length_inches: f64) -> Self {
        self.static_length = inches_to_meters(length_inches);
        self
    }

    /// Sets the moving ligament's visual length. Takes inches and stores metres.
    pub fn set_moving_length(mut self, length_inches: f64) -> Self {
        self.moving_length = inches_to_meters(length_inches);
        self
    }

    /// Sets the top of the travel range. Takes inches and stores metres.
    pub fn set_max_height(mut self, length_inches: f64) -> Self {
        self.max_height = inches_to_meters(length_inches);
        self
    }

    /// Attaches this stage to a parent `LinearSim` so its root follows that stage. `None`
    /// leaves this one unmounted.
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

    /// Attaches this stage to a parent `ArmSim` so its root follows the arm tip. `None`
    /// leaves this stage unmounted.
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

impl MountedConfig for LinearConfig {
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
