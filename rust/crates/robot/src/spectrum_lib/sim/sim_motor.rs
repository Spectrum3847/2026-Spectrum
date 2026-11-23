//! Shared wiring between a `TalonFX` and the WPILib physics sims in this package.

use phoenix6::TalonFX;
use phoenix6::sim::{ChassisReference, TalonFXSimState};

/// What the physics sims need from a simulated motor. `TalonFXSimState` is the real one.
/// The sims step on the `SimLoop` thread, so it must be `Send + Sync`.
pub trait MotorSim: Send + Sync {
    fn motor_voltage(&self) -> f64;
    fn set_raw_rotor_position(&self, rotations: f64);
    fn set_rotor_velocity(&self, rps: f64);
    fn add_rotor_position(&self, delta_rotations: f64);
}

impl MotorSim for TalonFXSimState {
    fn motor_voltage(&self) -> f64 {
        TalonFXSimState::motor_voltage(self)
    }

    fn set_raw_rotor_position(&self, rotations: f64) {
        TalonFXSimState::set_raw_rotor_position(self, rotations);
    }

    fn set_rotor_velocity(&self, rps: f64) {
        TalonFXSimState::set_rotor_velocity(self, rps);
    }

    fn add_rotor_position(&self, delta_rotations: f64) {
        TalonFXSimState::add_rotor_position(self, delta_rotations);
    }
}

/// The motor's sim state, with `orientation` flipped when the sim's travel runs opposite the
/// motor's positive direction.
///
/// The sims read `motor_voltage` and feed position back through `set_raw_rotor_position`.
/// Orientation negates both, so flipping it mirrors the whole loop. It is not derived from
/// the motor's invert: CTRE defines it as the mechanical linkage, and the invert alone does
/// not fix the sign. What matters is the direction each sim's geometry produces:
///
/// - `ElevatorSim` travels `[min_height, max]` with min normally 0, so its rotor term is
///   always positive.
/// - `SingleJointedArmSim` feeds back `angle - starting_angle`, which is negative whenever
///   the arm starts at its maximum.
///
/// An inverted motor is fine on a sim whose geometry already runs negative, and broken on
/// one that runs positive: the model clamps at its limit and the mechanism never moves.
/// Only the mechanism author knows which case applies, hence the explicit flag.
pub fn sim_state(motor: &TalonFX, reversed_linkage: bool) -> TalonFXSimState {
    let mut state = motor.sim_state();
    state.orientation = if reversed_linkage {
        ChassisReference::Clockwise_Positive
    } else {
        ChassisReference::CounterClockwise_Positive
    };
    state
}

#[cfg(test)]
pub(crate) mod test_support {
    use super::MotorSim;
    use std::sync::{Arc, Mutex};

    #[derive(Default, Debug, Clone, Copy, PartialEq)]
    pub struct MotorState {
        pub voltage: f64,
        pub position: f64,
        pub velocity: f64,
        pub position_sets: u32,
    }

    /// A motor that records what the sims write. Clones share state.
    #[derive(Default, Clone)]
    pub struct FakeMotor(pub Arc<Mutex<MotorState>>);

    impl FakeMotor {
        pub fn set_voltage(&self, volts: f64) {
            self.0.lock().unwrap().voltage = volts;
        }

        pub fn state(&self) -> MotorState {
            *self.0.lock().unwrap()
        }
    }

    impl MotorSim for FakeMotor {
        fn motor_voltage(&self) -> f64 {
            self.0.lock().unwrap().voltage
        }

        fn set_raw_rotor_position(&self, rotations: f64) {
            let mut s = self.0.lock().unwrap();
            s.position = rotations;
            s.position_sets += 1;
        }

        fn set_rotor_velocity(&self, rps: f64) {
            self.0.lock().unwrap().velocity = rps;
        }

        fn add_rotor_position(&self, delta_rotations: f64) {
            self.0.lock().unwrap().position += delta_rotations;
        }
    }
}
