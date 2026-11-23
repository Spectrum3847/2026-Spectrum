use super::linear_system_sim::LinearSystemSim;
use super::signum;
use crate::math_util;
use crate::system::{DcMotor, LinearSystem, create_elevator_system, rkdp};
use nalgebra::SVector;

#[derive(Clone, Debug)]
pub struct ElevatorSim {
    sim: LinearSystemSim<2, 1, 2>,
    gearbox: DcMotor,
    min_height: f64,
    max_height: f64,
    simulate_gravity: bool,
}

impl ElevatorSim {
    #[allow(clippy::too_many_arguments)] // mirrors the WPILib constructor
    pub fn new(
        gearbox: DcMotor,
        gearing: f64,
        carriage_mass_kg: f64,
        drum_radius_meters: f64,
        min_height_meters: f64,
        max_height_meters: f64,
        simulate_gravity: bool,
        starting_height_meters: f64,
    ) -> Self {
        let plant: LinearSystem<2, 1, 2> =
            create_elevator_system(gearbox, carriage_mass_kg, drum_radius_meters, gearing);
        let mut elevator = Self {
            sim: LinearSystemSim::new(plant),
            gearbox,
            min_height: min_height_meters,
            max_height: max_height_meters,
            simulate_gravity,
        };
        elevator.set_state(starting_height_meters, 0.0);
        elevator
    }

    pub fn set_input(&mut self, volts: f64) {
        self.sim.u = SVector::<f64, 1>::new(volts);
    }

    /// Position is clamped to the travel limits.
    pub fn set_state(&mut self, position_meters: f64, velocity_meters_per_second: f64) {
        self.sim.set_state(SVector::<f64, 2>::new(
            math_util::clamp(position_meters, self.min_height, self.max_height),
            velocity_meters_per_second,
        ));
    }

    pub fn would_hit_lower_limit(&self, elevator_height_meters: f64) -> bool {
        elevator_height_meters <= self.min_height
    }

    pub fn would_hit_upper_limit(&self, elevator_height_meters: f64) -> bool {
        elevator_height_meters >= self.max_height
    }

    pub fn has_hit_lower_limit(&self) -> bool {
        self.would_hit_lower_limit(self.position_meters())
    }

    pub fn has_hit_upper_limit(&self) -> bool {
        self.would_hit_upper_limit(self.position_meters())
    }

    pub fn position_meters(&self) -> f64 {
        self.sim.y[0]
    }

    pub fn velocity_meters_per_second(&self) -> f64 {
        self.sim.y[1]
    }

    pub fn current_draw_amps(&self) -> f64 {
        let k_a = 1.0 / self.sim.plant.b()[(1, 0)];
        let k_v = -self.sim.plant.a()[(1, 1)] * k_a;
        let motor_velocity = self.sim.x[1] * k_v * self.gearbox.kv_rad_per_sec_per_volt;
        let applied = self.sim.u[0];
        self.gearbox.current(motor_velocity, applied) * signum(applied)
    }

    pub fn update(&mut self, dt_seconds: f64) {
        let a = *self.sim.plant.a();
        let b = *self.sim.plant.b();
        let gravity = self.simulate_gravity;
        let updated = rkdp(
            |x, u| {
                let xdot = a * x + b * u;
                if gravity {
                    xdot + SVector::<f64, 2>::new(0.0, -9.8)
                } else {
                    xdot
                }
            },
            self.sim.x,
            self.sim.u,
            dt_seconds,
        );
        let next = if self.would_hit_lower_limit(updated[0]) {
            SVector::<f64, 2>::new(self.min_height, 0.0)
        } else if self.would_hit_upper_limit(updated[0]) {
            SVector::<f64, 2>::new(self.max_height, 0.0)
        } else {
            updated
        };
        self.sim.set_state(next);
    }
}
