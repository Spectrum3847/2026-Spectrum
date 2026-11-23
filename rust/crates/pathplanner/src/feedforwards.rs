use crate::flipping::FlippingUtil;
use std::ops::Deref;
use wpimath::math_util;

/// The most drive modules a robot config can have. Swerve has 4.
pub const MAX_MODULES: usize = 4;

/// One value per drive module, stored inline so trajectory states need no allocation.
/// Swerve order is FL, FR, BL, BR.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PerModule {
    values: [f64; MAX_MODULES],
    len: usize,
}

impl PerModule {
    pub fn zeros(len: usize) -> Self {
        assert!(len <= MAX_MODULES, "at most {MAX_MODULES} modules");
        Self {
            values: [0.0; MAX_MODULES],
            len,
        }
    }

    pub fn from_slice(values: &[f64]) -> Self {
        let mut out = Self::zeros(values.len());
        out.values[..values.len()].copy_from_slice(values);
        out
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.values[..self.len]
    }

    pub(crate) fn set(&mut self, index: usize, value: f64) {
        assert!(index < self.len, "module index out of range");
        self.values[index] = value;
    }

    /// Left and right modules swapped, which is what mirroring does. Other counts are unchanged.
    pub(crate) fn swapped_sides(&self) -> Self {
        match self.len {
            4 => Self::from_slice(&[self[1], self[0], self[3], self[2]]),
            2 => Self::from_slice(&[self[1], self[0]]),
            _ => *self,
        }
    }

    pub(crate) fn negated(&self) -> Self {
        let mut out = *self;
        for v in &mut out.values[..out.len] {
            *v = -*v;
        }
        out
    }

    fn interpolate(&self, end: &Self, t: f64) -> Self {
        let mut out = Self::zeros(self.len);
        for i in 0..self.len {
            out.values[i] = math_util::interpolate(self.values[i], end.values[i], t);
        }
        out
    }
}

impl Deref for PerModule {
    type Target = [f64];

    fn deref(&self) -> &[f64] {
        self.as_slice()
    }
}

/// Feedforward values for each drive module, in FL, FR, BL, BR order for swerve.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DriveFeedforwards {
    /// Linear acceleration at the wheels in m/s^2.
    pub accelerations_mps_sq: PerModule,
    /// Linear force applied by the motors at the wheels in newtons.
    pub linear_forces_newtons: PerModule,
    /// Torque current of the drive motors in amps.
    pub torque_currents_amps: PerModule,
    /// X components of the robot-relative wheel force vectors in newtons. Larger than the linear
    /// forces because of friction.
    pub robot_relative_forces_x_newtons: PerModule,
    /// Y components of the robot-relative wheel force vectors in newtons.
    pub robot_relative_forces_y_newtons: PerModule,
}

impl DriveFeedforwards {
    pub fn zeros(num_modules: usize) -> Self {
        let zeros = PerModule::zeros(num_modules);
        Self {
            accelerations_mps_sq: zeros,
            linear_forces_newtons: zeros,
            torque_currents_amps: zeros,
            robot_relative_forces_x_newtons: zeros,
            robot_relative_forces_y_newtons: zeros,
        }
    }

    pub fn interpolate(&self, end: &Self, t: f64) -> Self {
        Self {
            accelerations_mps_sq: self
                .accelerations_mps_sq
                .interpolate(&end.accelerations_mps_sq, t),
            linear_forces_newtons: self
                .linear_forces_newtons
                .interpolate(&end.linear_forces_newtons, t),
            torque_currents_amps: self
                .torque_currents_amps
                .interpolate(&end.torque_currents_amps, t),
            robot_relative_forces_x_newtons: self
                .robot_relative_forces_x_newtons
                .interpolate(&end.robot_relative_forces_x_newtons, t),
            robot_relative_forces_y_newtons: self
                .robot_relative_forces_y_newtons
                .interpolate(&end.robot_relative_forces_y_newtons, t),
        }
    }

    /// For the other side of the field. Only changes anything under mirrored symmetry.
    pub fn flip(&self) -> Self {
        Self {
            accelerations_mps_sq: FlippingUtil::flip_feedforwards(&self.accelerations_mps_sq),
            linear_forces_newtons: FlippingUtil::flip_feedforwards(&self.linear_forces_newtons),
            torque_currents_amps: FlippingUtil::flip_feedforwards(&self.torque_currents_amps),
            robot_relative_forces_x_newtons: FlippingUtil::flip_feedforward_xs(
                &self.robot_relative_forces_x_newtons,
            ),
            robot_relative_forces_y_newtons: FlippingUtil::flip_feedforward_ys(
                &self.robot_relative_forces_y_newtons,
            ),
        }
    }

    /// Left and right swapped for a mirrored path.
    pub(crate) fn mirror(&self) -> Self {
        Self {
            accelerations_mps_sq: self.accelerations_mps_sq.swapped_sides(),
            linear_forces_newtons: self.linear_forces_newtons.swapped_sides(),
            torque_currents_amps: self.torque_currents_amps.swapped_sides(),
            robot_relative_forces_x_newtons: self.robot_relative_forces_x_newtons.swapped_sides(),
            robot_relative_forces_y_newtons: self.robot_relative_forces_y_newtons.swapped_sides(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn per_module_values_behave_like_a_slice() {
        let v = PerModule::from_slice(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(v.len(), 4);
        assert_eq!(v[2], 3.0);
        assert_eq!(v.swapped_sides().as_slice(), [2.0, 1.0, 4.0, 3.0]);
        assert_eq!(v.negated().as_slice(), [-1.0, -2.0, -3.0, -4.0]);
        let two = PerModule::from_slice(&[5.0, 6.0]);
        assert_eq!(two.swapped_sides().as_slice(), [6.0, 5.0]);
        let three = PerModule::from_slice(&[1.0, 2.0, 3.0]);
        assert_eq!(three.swapped_sides(), three);
    }

    #[test]
    #[should_panic(expected = "at most 4 modules")]
    fn there_are_at_most_four_modules() {
        PerModule::zeros(5);
    }

    #[test]
    fn feedforwards_interpolate_every_array() {
        let mut a = DriveFeedforwards::zeros(4);
        let mut b = DriveFeedforwards::zeros(4);
        a.accelerations_mps_sq = PerModule::from_slice(&[0.0, 1.0, 2.0, 3.0]);
        b.accelerations_mps_sq = PerModule::from_slice(&[10.0, 11.0, 12.0, 13.0]);
        b.robot_relative_forces_y_newtons = PerModule::from_slice(&[4.0; 4]);
        let mid = a.interpolate(&b, 0.5);
        assert_eq!(mid.accelerations_mps_sq.as_slice(), [5.0, 6.0, 7.0, 8.0]);
        assert_eq!(mid.robot_relative_forces_y_newtons.as_slice(), [2.0; 4]);
        // The factor is clamped.
        assert_eq!(a.interpolate(&b, 2.0), b);
    }

    #[test]
    fn flipping_changes_nothing_for_rotational_symmetry() {
        let mut ff = DriveFeedforwards::zeros(4);
        ff.linear_forces_newtons = PerModule::from_slice(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(ff.flip(), ff);
    }

    #[test]
    fn mirroring_swaps_left_and_right() {
        let mut ff = DriveFeedforwards::zeros(4);
        ff.linear_forces_newtons = PerModule::from_slice(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(
            ff.mirror().linear_forces_newtons.as_slice(),
            [2.0, 1.0, 4.0, 3.0]
        );
    }
}
