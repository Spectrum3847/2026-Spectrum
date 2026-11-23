use super::{ChassisSpeeds, SwerveModulePosition, SwerveModuleState};
use crate::geometry::{Rotation2d, Translation2d, Twist2d};
use nalgebra::Matrix3;

/// Swerve kinematics for `N` modules.
#[derive(Clone, Debug)]
pub struct SwerveDriveKinematics<const N: usize> {
    modules: [Translation2d; N],
    // Columns of the 3 x 2N pseudo-inverse: x and y wheel components of each module.
    forward_x: [[f64; 3]; N],
    forward_y: [[f64; 3]; N],
    module_headings: [Rotation2d; N],
}

impl<const N: usize> SwerveDriveKinematics<N> {
    /// # Panics
    /// If `N < 2`.
    pub fn new(modules: [Translation2d; N]) -> Self {
        assert!(N >= 2, "A swerve drive requires at least two modules");
        // Pseudo-inverse (AtA)^-1 At of the 2N x 3 inverse kinematics matrix A. nalgebra's SVD
        // only reaches ~1e-10 on this shape, which breaks parity with WPILib.
        let a = |row: usize, col: usize| {
            let m = modules[row / 2];
            match (row % 2, col) {
                (0, 0) | (1, 1) => 1.0,
                (0, 2) => -m.y,
                (1, 2) => m.x,
                _ => 0.0,
            }
        };
        let ata = Matrix3::from_fn(|r, c| (0..2 * N).map(|k| a(k, r) * a(k, c)).sum::<f64>());
        let ata_inverse = ata
            .try_inverse()
            .expect("swerve modules must not all be at the same position");
        let forward = |row: usize| -> [f64; 3] {
            std::array::from_fn(|r| (0..3).map(|k| ata_inverse[(r, k)] * a(row, k)).sum::<f64>())
        };
        let forward_x = std::array::from_fn(|i| forward(2 * i));
        let forward_y = std::array::from_fn(|i| forward(2 * i + 1));
        Self {
            modules,
            forward_x,
            forward_y,
            module_headings: [Rotation2d::ZERO; N],
        }
    }

    pub fn modules(&self) -> &[Translation2d; N] {
        &self.modules
    }

    pub fn reset_headings(&mut self, headings: [Rotation2d; N]) {
        self.module_headings = headings;
    }

    /// Module states for `speeds`, rotating about `center_of_rotation`. Modules that would
    /// barely move keep their last heading.
    pub fn to_swerve_module_states(
        &mut self,
        speeds: ChassisSpeeds,
        center_of_rotation: Translation2d,
    ) -> [SwerveModuleState; N] {
        if speeds.vx == 0.0 && speeds.vy == 0.0 && speeds.omega == 0.0 {
            return std::array::from_fn(|i| SwerveModuleState::new(0.0, self.module_headings[i]));
        }
        std::array::from_fn(|i| {
            let m = self.modules[i];
            let x = speeds.vx + (-m.y + center_of_rotation.y) * speeds.omega;
            let y = speeds.vy + (m.x - center_of_rotation.x) * speeds.omega;
            let speed = x.hypot(y);
            let angle = if speed > 1e-6 {
                Rotation2d::from_xy(x, y)
            } else {
                self.module_headings[i]
            };
            self.module_headings[i] = angle;
            SwerveModuleState::new(speed, angle)
        })
    }

    pub fn to_chassis_speeds(&self, states: &[SwerveModuleState; N]) -> ChassisSpeeds {
        let wheel = |i: usize| {
            (
                states[i].speed * states[i].angle.cos(),
                states[i].speed * states[i].angle.sin(),
            )
        };
        let v = self.multiply_forward(wheel);
        ChassisSpeeds::new(v[0], v[1], v[2])
    }

    /// Robot displacement between two sets of module positions, using the end angles.
    pub fn to_twist2d(
        &self,
        start: &[SwerveModulePosition; N],
        end: &[SwerveModulePosition; N],
    ) -> Twist2d {
        let wheel = |i: usize| {
            let distance = end[i].distance - start[i].distance;
            (distance * end[i].angle.cos(), distance * end[i].angle.sin())
        };
        let v = self.multiply_forward(wheel);
        Twist2d::new(v[0], v[1], v[2])
    }

    fn multiply_forward(&self, wheel: impl Fn(usize) -> (f64, f64)) -> [f64; 3] {
        let mut out = [0.0; 3];
        for i in 0..N {
            let (x, y) = wheel(i);
            for (r, o) in out.iter_mut().enumerate() {
                *o += self.forward_x[i][r] * x;
                *o += self.forward_y[i][r] * y;
            }
        }
        out
    }

    /// Scales all speeds down so none exceeds `attainable_max`.
    pub fn desaturate_wheel_speeds(states: &mut [SwerveModuleState], attainable_max: f64) {
        let real_max = states.iter().fold(0.0_f64, |m, s| m.max(s.speed.abs()));
        if real_max > attainable_max {
            for s in states {
                s.speed = s.speed / real_max * attainable_max;
            }
        }
    }

    /// Scales speeds by how much of the translation and rotation limits the command uses.
    pub fn desaturate_wheel_speeds_for_chassis(
        states: &mut [SwerveModuleState],
        desired: ChassisSpeeds,
        attainable_max_module_speed: f64,
        attainable_max_translational_speed: f64,
        attainable_max_rotational_velocity: f64,
    ) {
        let real_max = states.iter().fold(0.0_f64, |m, s| m.max(s.speed.abs()));
        if attainable_max_translational_speed == 0.0
            || attainable_max_rotational_velocity == 0.0
            || real_max == 0.0
        {
            return;
        }
        let translational_k = desired.vx.hypot(desired.vy) / attainable_max_translational_speed;
        let rotational_k = desired.omega.abs() / attainable_max_rotational_velocity;
        let k = translational_k.max(rotational_k);
        let scale = (k * attainable_max_module_speed / real_max).min(1.0);
        for s in states {
            s.speed *= scale;
        }
    }
}
