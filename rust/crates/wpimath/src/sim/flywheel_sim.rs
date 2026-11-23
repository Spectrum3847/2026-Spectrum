use super::linear_system_sim::LinearSystemSim;
use super::signum;
use crate::system::{DcMotor, LinearSystem};
use crate::units;
use nalgebra::SVector;

#[derive(Clone, Debug)]
pub struct FlywheelSim {
    sim: LinearSystemSim<1, 1, 1>,
    gearbox: DcMotor,
    gearing: f64,
    j_kg_meters_squared: f64,
}

impl FlywheelSim {
    pub fn new(plant: LinearSystem<1, 1, 1>, gearbox: DcMotor) -> Self {
        let a = plant.a()[(0, 0)];
        let b = plant.b()[(0, 0)];
        let gearing = -gearbox.kv_rad_per_sec_per_volt * a / b;
        let j_kg_meters_squared = gearing * gearbox.kt_nm_per_amp / (gearbox.r_ohms * b);
        Self {
            sim: LinearSystemSim::new(plant),
            gearbox,
            gearing,
            j_kg_meters_squared,
        }
    }

    /// Sets the applied voltage. Not clamped to the battery voltage.
    pub fn set_input(&mut self, volts: f64) {
        self.sim.u = SVector::<f64, 1>::new(volts);
    }

    pub fn update(&mut self, dt_seconds: f64) {
        self.sim.update_exact(dt_seconds);
    }

    pub fn set_angular_velocity(&mut self, velocity_rad_per_sec: f64) {
        self.sim
            .set_state(SVector::<f64, 1>::new(velocity_rad_per_sec));
    }

    pub fn gearing(&self) -> f64 {
        self.gearing
    }

    pub fn j_kg_meters_squared(&self) -> f64 {
        self.j_kg_meters_squared
    }

    pub fn angular_velocity_rad_per_sec(&self) -> f64 {
        self.sim.y[0]
    }

    pub fn angular_velocity_rpm(&self) -> f64 {
        units::radians_per_second_to_rotations_per_minute(self.angular_velocity_rad_per_sec())
    }

    pub fn angular_acceleration_rad_per_sec_sq(&self) -> f64 {
        (self.sim.plant.a() * self.sim.x + self.sim.plant.b() * self.sim.u)[0]
    }

    pub fn torque_newton_meters(&self) -> f64 {
        self.angular_acceleration_rad_per_sec_sq() * self.j_kg_meters_squared
    }

    pub fn current_draw_amps(&self) -> f64 {
        let u = self.sim.u[0];
        self.gearbox.current(self.sim.x[0] * self.gearing, u) * signum(u)
    }
}
