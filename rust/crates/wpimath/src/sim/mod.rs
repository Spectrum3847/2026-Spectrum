//! Mechanism physics sims from WPILib. None add measurement noise: the robot never passes
//! measurement standard deviations.

mod elevator_sim;
mod flywheel_sim;
mod linear_system_sim;
mod single_jointed_arm_sim;

pub use elevator_sim::ElevatorSim;
pub use flywheel_sim::FlywheelSim;
pub use single_jointed_arm_sim::SingleJointedArmSim;

/// Java `Math.signum`: zero stays zero.
fn signum(v: f64) -> f64 {
    if v == 0.0 || v.is_nan() {
        v
    } else {
        v.signum()
    }
}
