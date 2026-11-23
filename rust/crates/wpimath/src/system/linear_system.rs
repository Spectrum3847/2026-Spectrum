use super::discretize_ab;
use nalgebra::{SMatrix, SVector};

/// Continuous state-space model `x' = A x + B u`, `y = C x + D u`.
#[derive(Clone, Debug)]
pub struct LinearSystem<const S: usize, const I: usize, const O: usize> {
    a: SMatrix<f64, S, S>,
    b: SMatrix<f64, S, I>,
    c: SMatrix<f64, O, S>,
    d: SMatrix<f64, O, I>,
}

impl<const S: usize, const I: usize, const O: usize> LinearSystem<S, I, O> {
    /// # Panics
    /// If any matrix element is not finite.
    pub fn new(
        a: SMatrix<f64, S, S>,
        b: SMatrix<f64, S, I>,
        c: SMatrix<f64, O, S>,
        d: SMatrix<f64, O, I>,
    ) -> Self {
        assert!(
            a.iter().all(|v| v.is_finite()),
            "Elements of A aren't finite. This is usually due to model implementation errors."
        );
        assert!(
            b.iter().all(|v| v.is_finite()),
            "Elements of B aren't finite. This is usually due to model implementation errors."
        );
        assert!(
            c.iter().all(|v| v.is_finite()),
            "Elements of C aren't finite. This is usually due to model implementation errors."
        );
        assert!(
            d.iter().all(|v| v.is_finite()),
            "Elements of D aren't finite. This is usually due to model implementation errors."
        );
        Self { a, b, c, d }
    }

    pub fn a(&self) -> &SMatrix<f64, S, S> {
        &self.a
    }

    pub fn b(&self) -> &SMatrix<f64, S, I> {
        &self.b
    }

    pub fn c(&self) -> &SMatrix<f64, O, S> {
        &self.c
    }

    pub fn d(&self) -> &SMatrix<f64, O, I> {
        &self.d
    }

    /// Discrete `(A, B)` for a step of `dt_seconds`.
    pub fn discretize(&self, dt_seconds: f64) -> (SMatrix<f64, S, S>, SMatrix<f64, S, I>) {
        discretize_ab(&self.a, &self.b, dt_seconds)
    }

    /// State after `dt_seconds` with input `u` held constant.
    pub fn calculate_x(
        &self,
        x: &SVector<f64, S>,
        u: &SVector<f64, I>,
        dt_seconds: f64,
    ) -> SVector<f64, S> {
        let (disc_a, disc_b) = self.discretize(dt_seconds);
        disc_a * x + disc_b * u
    }

    pub fn calculate_y(&self, x: &SVector<f64, S>, u: &SVector<f64, I>) -> SVector<f64, O> {
        self.c * x + self.d * u
    }
}
