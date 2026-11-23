//! State-space models: motors, plants, discretization and integration.

mod dc_motor;
mod discretization;
mod linear_system;
mod linear_system_id;
mod numerical_integration;

pub use dc_motor::DcMotor;
pub use discretization::{discretize_ab, matrix_exp};
pub use linear_system::LinearSystem;
pub use linear_system_id::{
    create_elevator_system, create_flywheel_system, create_single_jointed_arm_system,
};
pub use numerical_integration::{rkdp, rkdp_with_error};
