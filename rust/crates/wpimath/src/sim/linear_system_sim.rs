use crate::system::LinearSystem;
use nalgebra::{SMatrix, SVector};

/// Shared state of the plant sims: state, input, output and a cached discretization.
#[derive(Clone, Debug)]
pub(super) struct LinearSystemSim<const S: usize, const I: usize, const O: usize> {
    pub plant: LinearSystem<S, I, O>,
    pub x: SVector<f64, S>,
    pub u: SVector<f64, I>,
    pub y: SVector<f64, O>,
    discretized: Option<(f64, SMatrix<f64, S, S>, SMatrix<f64, S, I>)>,
}

impl<const S: usize, const I: usize, const O: usize> LinearSystemSim<S, I, O> {
    pub fn new(plant: LinearSystem<S, I, O>) -> Self {
        Self {
            plant,
            x: SVector::zeros(),
            u: SVector::zeros(),
            y: SVector::zeros(),
            discretized: None,
        }
    }

    pub fn set_state(&mut self, x: SVector<f64, S>) {
        self.x = x;
        self.y = self.plant.calculate_y(&self.x, &self.u);
    }

    /// Exact discrete step; WPILib's default `updateX`.
    pub fn update_exact(&mut self, dt_seconds: f64) {
        let (disc_a, disc_b) = match self.discretized {
            Some((dt, a, b)) if dt == dt_seconds => (a, b),
            _ => {
                let (a, b) = self.plant.discretize(dt_seconds);
                self.discretized = Some((dt_seconds, a, b));
                (a, b)
            }
        };
        self.set_state(disc_a * self.x + disc_b * self.u);
    }
}
