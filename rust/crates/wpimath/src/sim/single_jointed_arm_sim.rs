use super::linear_system_sim::LinearSystemSim;
use super::signum;
use crate::math_util;
use crate::system::{DcMotor, LinearSystem, create_single_jointed_arm_system, rkdp};
use nalgebra::SVector;

#[derive(Clone, Debug)]
pub struct SingleJointedArmSim {
    sim: LinearSystemSim<2, 1, 2>,
    gearbox: DcMotor,
    gearing: f64,
    arm_len_meters: f64,
    min_angle: f64,
    max_angle: f64,
    simulate_gravity: bool,
}

impl SingleJointedArmSim {
    #[allow(clippy::too_many_arguments)] // mirrors the WPILib constructor
    pub fn new(
        gearbox: DcMotor,
        gearing: f64,
        j_kg_meters_squared: f64,
        arm_length_meters: f64,
        min_angle_rads: f64,
        max_angle_rads: f64,
        simulate_gravity: bool,
        starting_angle_rads: f64,
    ) -> Self {
        let plant: LinearSystem<2, 1, 2> =
            create_single_jointed_arm_system(gearbox, j_kg_meters_squared, gearing);
        let mut arm = Self {
            sim: LinearSystemSim::new(plant),
            gearbox,
            gearing,
            arm_len_meters: arm_length_meters,
            min_angle: min_angle_rads,
            max_angle: max_angle_rads,
            simulate_gravity,
        };
        arm.set_state(starting_angle_rads, 0.0);
        arm
    }

    pub fn set_input(&mut self, volts: f64) {
        self.sim.u = SVector::<f64, 1>::new(volts);
    }

    /// Angle is clamped to the travel limits.
    pub fn set_state(&mut self, angle_radians: f64, velocity_rad_per_sec: f64) {
        self.sim.set_state(SVector::<f64, 2>::new(
            math_util::clamp(angle_radians, self.min_angle, self.max_angle),
            velocity_rad_per_sec,
        ));
    }

    pub fn would_hit_lower_limit(&self, current_angle_rads: f64) -> bool {
        current_angle_rads <= self.min_angle
    }

    pub fn would_hit_upper_limit(&self, current_angle_rads: f64) -> bool {
        current_angle_rads >= self.max_angle
    }

    pub fn has_hit_lower_limit(&self) -> bool {
        self.would_hit_lower_limit(self.angle_rads())
    }

    pub fn has_hit_upper_limit(&self) -> bool {
        self.would_hit_upper_limit(self.angle_rads())
    }

    pub fn angle_rads(&self) -> f64 {
        self.sim.y[0]
    }

    pub fn velocity_rad_per_sec(&self) -> f64 {
        self.sim.y[1]
    }

    pub fn current_draw_amps(&self) -> f64 {
        let motor_velocity = self.sim.x[1] * self.gearing;
        let u = self.sim.u[0];
        self.gearbox.current(motor_velocity, u) * signum(u)
    }

    pub fn update(&mut self, dt_seconds: f64) {
        let a = *self.sim.plant.a();
        let b = *self.sim.plant.b();
        let gravity = self.simulate_gravity;
        let arm_len = self.arm_len_meters;
        let updated = rkdp(
            |x, u| {
                let xdot = a * x + b * u;
                if gravity {
                    let alpha_grav = 3.0 / 2.0 * -9.8 * x[0].cos() / arm_len;
                    xdot + SVector::<f64, 2>::new(0.0, alpha_grav)
                } else {
                    xdot
                }
            },
            self.sim.x,
            self.sim.u,
            dt_seconds,
        );
        let next = if self.would_hit_lower_limit(updated[0]) {
            SVector::<f64, 2>::new(self.min_angle, 0.0)
        } else if self.would_hit_upper_limit(updated[0]) {
            SVector::<f64, 2>::new(self.max_angle, 0.0)
        } else {
            updated
        };
        self.sim.set_state(next);
    }
}
