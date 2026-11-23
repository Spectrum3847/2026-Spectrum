//! Placement of a component on the canvas from its parent mount's type, position and angle.
//! All lengths are in metres and angles in radians.

use super::mount::{Mount, MountType};

/// The mount and starting geometry a mounted sim config carries.
pub trait MountedConfig {
    /// The parent mount. Panics when the config is not mounted.
    fn mount(&self) -> &dyn Mount;

    /// Component's initial canvas X.
    fn initial_x(&self) -> f64;

    /// Component's initial canvas Y.
    fn initial_y(&self) -> f64;

    /// Mount's X at simulation start.
    fn init_mount_x(&self) -> f64;

    /// Mount's Y at simulation start.
    fn init_mount_y(&self) -> f64;

    /// Mount's angle at simulation start.
    fn init_mount_angle(&self) -> f64;
}

/// Marks a sim that attaches to a [`Mount`]. The placement helpers are default methods.
pub trait Mountable {
    /// Full-geometry placement. Returns the updated X of the component on the canvas.
    #[allow(clippy::too_many_arguments)] // mirrors the Java signature
    fn updated_x(
        &self,
        mount_type: MountType,
        initial_x: f64,
        initial_y: f64,
        init_mount_x: f64,
        init_mount_y: f64,
        init_mount_angle: f64,
        mount_x: f64,
        _mount_y: f64,
        displacement_x: f64,
        _displacement_y: f64,
        mount_angle: f64,
    ) -> f64 {
        let radius = distance(initial_x, initial_y, init_mount_x, init_mount_y);
        let angle = mount_angle
            + angle_offset(
                initial_x,
                initial_y,
                init_mount_x,
                init_mount_y,
                init_mount_angle,
            );
        // A linear mount carries the component by its displacement; an arm pivots it about
        // the mount's current position.
        let base = if mount_type == MountType::Linear {
            init_mount_x + displacement_x
        } else {
            mount_x
        };
        x_with_angle(radius, angle, base)
    }

    /// Full-geometry placement. Returns the updated Y of the component on the canvas.
    #[allow(clippy::too_many_arguments)] // mirrors the Java signature
    fn updated_y(
        &self,
        mount_type: MountType,
        initial_x: f64,
        initial_y: f64,
        init_mount_x: f64,
        init_mount_y: f64,
        init_mount_angle: f64,
        _mount_x: f64,
        mount_y: f64,
        _displacement_x: f64,
        displacement_y: f64,
        mount_angle: f64,
    ) -> f64 {
        let radius = distance(initial_x, initial_y, init_mount_x, init_mount_y);
        let angle = mount_angle
            + angle_offset(
                initial_x,
                initial_y,
                init_mount_x,
                init_mount_y,
                init_mount_angle,
            );
        let base = if mount_type == MountType::Linear {
            init_mount_y + displacement_y
        } else {
            mount_y
        };
        y_with_angle(radius, angle, base)
    }

    /// Placement using the geometry a mounted sim config carries.
    fn updated_x_for(&self, config: &dyn MountedConfig) -> f64 {
        let mount = config.mount();
        self.updated_x(
            mount.mount_type(),
            config.initial_x(),
            config.initial_y(),
            config.init_mount_x(),
            config.init_mount_y(),
            config.init_mount_angle(),
            mount.mount_x(),
            mount.mount_y(),
            mount.displacement_x(),
            mount.displacement_y(),
            mount.angle(),
        )
    }

    /// Placement using the geometry a mounted sim config carries.
    fn updated_y_for(&self, config: &dyn MountedConfig) -> f64 {
        let mount = config.mount();
        self.updated_y(
            mount.mount_type(),
            config.initial_x(),
            config.initial_y(),
            config.init_mount_x(),
            config.init_mount_y(),
            config.init_mount_angle(),
            mount.mount_x(),
            mount.mount_y(),
            mount.displacement_x(),
            mount.displacement_y(),
            mount.angle(),
        )
    }
}

/// Angle of a component's initial position relative to its mount, measured off the mount's
/// starting angle. A component left of the mount takes the mirrored branch.
pub fn angle_offset(
    initial_x: f64,
    initial_y: f64,
    mount_x: f64,
    mount_y: f64,
    starting_angle: f64,
) -> f64 {
    let hypotenuse = distance(initial_x, initial_y, mount_x, mount_y);
    if initial_x >= mount_x {
        ((initial_y - mount_y) / hypotenuse).asin() - starting_angle
    } else {
        std::f64::consts::PI - ((initial_y - mount_y) / hypotenuse).asin() - starting_angle
    }
}

/// Straight-line distance between two canvas points.
pub fn distance(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
}

/// Cartesian X of a point `radius` from the reference point, `angle` radians off the Y axis.
pub fn x_with_angle(radius: f64, angle: f64, displacement_x: f64) -> f64 {
    radius * angle.cos() + displacement_x
}

/// Cartesian Y of a point `radius` from the reference point, `angle` radians off the X axis.
pub fn y_with_angle(radius: f64, angle: f64, displacement_y: f64) -> f64 {
    radius * angle.sin() + displacement_y
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    struct Placer;
    impl Mountable for Placer {}

    const EPS: f64 = 1e-12;

    #[test]
    fn distance_is_euclidean() {
        assert_eq!(distance(0.0, 0.0, 3.0, 4.0), 5.0);
        assert_eq!(distance(1.0, 1.0, 1.0, 1.0), 0.0);
    }

    #[test]
    fn angle_offset_takes_the_mirrored_branch_left_of_the_mount() {
        // Right of the mount: asin(dy / r) - start.
        let right = angle_offset(1.0, 1.0, 0.0, 0.0, 0.25);
        assert!((right - (FRAC_PI_4 - 0.25)).abs() < EPS);
        // Left of the mount: pi - asin(dy / r) - start.
        let left = angle_offset(-1.0, 1.0, 0.0, 0.0, 0.25);
        assert!((left - (PI - FRAC_PI_4 - 0.25)).abs() < EPS);
        // Directly above takes the right branch (initial_x >= mount_x).
        let above = angle_offset(0.0, 2.0, 0.0, 0.0, 0.0);
        assert!((above - FRAC_PI_2).abs() < EPS);
    }

    #[test]
    fn angle_helpers_project_onto_the_axes() {
        assert!((x_with_angle(2.0, 0.0, 1.0) - 3.0).abs() < EPS);
        assert!((y_with_angle(2.0, FRAC_PI_2, 1.0) - 3.0).abs() < EPS);
    }

    #[test]
    fn arm_mount_pivots_about_the_mount_position() {
        // Component at (1, 1) around a mount that started at the origin, mount turned 45 deg.
        let args = |t| {
            (
                t, 1.0, 1.0, 0.0, 0.0, 0.0, // type, initial, init mount, init angle
                2.0, 3.0, 9.0, 9.0, FRAC_PI_4, // mount x/y, displacement, mount angle
            )
        };
        let (t, ix, iy, imx, imy, ia, mx, my, dx, dy, ma) = args(MountType::Arm);
        let x = Placer.updated_x(t, ix, iy, imx, imy, ia, mx, my, dx, dy, ma);
        let y = Placer.updated_y(t, ix, iy, imx, imy, ia, mx, my, dx, dy, ma);
        // radius sqrt(2), angle 45 + 45 = 90 degrees.
        assert!((x - 2.0).abs() < EPS);
        assert!((y - (2.0_f64.sqrt() + 3.0)).abs() < EPS);
    }

    #[test]
    fn linear_mount_carries_the_component_by_displacement() {
        let (ix, iy, imx, imy) = (1.0, 1.0, 0.0, 0.0);
        let x = Placer.updated_x(
            MountType::Linear,
            ix,
            iy,
            imx,
            imy,
            0.0,
            50.0,
            50.0,
            0.5,
            0.25,
            0.0,
        );
        let y = Placer.updated_y(
            MountType::Linear,
            ix,
            iy,
            imx,
            imy,
            0.0,
            50.0,
            50.0,
            0.5,
            0.25,
            0.0,
        );
        // Mount angle 0: radius sqrt(2) at 45 degrees gives (1, 1) plus the displacement.
        assert!((x - 1.5).abs() < EPS);
        assert!((y - 1.25).abs() < EPS);
    }

    struct FixedMount;
    impl Mount for FixedMount {
        fn mount_type(&self) -> MountType {
            MountType::Arm
        }
        fn displacement_x(&self) -> f64 {
            0.0
        }
        fn displacement_y(&self) -> f64 {
            0.0
        }
        fn angle(&self) -> f64 {
            FRAC_PI_4
        }
        fn mount_x(&self) -> f64 {
            2.0
        }
        fn mount_y(&self) -> f64 {
            3.0
        }
    }

    struct FixedConfig(FixedMount);
    impl MountedConfig for FixedConfig {
        fn mount(&self) -> &dyn Mount {
            &self.0
        }
        fn initial_x(&self) -> f64 {
            1.0
        }
        fn initial_y(&self) -> f64 {
            1.0
        }
        fn init_mount_x(&self) -> f64 {
            0.0
        }
        fn init_mount_y(&self) -> f64 {
            0.0
        }
        fn init_mount_angle(&self) -> f64 {
            0.0
        }
    }

    #[test]
    fn config_overloads_pass_the_config_values_through() {
        let config = FixedConfig(FixedMount);
        assert!((Placer.updated_x_for(&config) - 2.0).abs() < EPS);
        assert!((Placer.updated_y_for(&config) - (2.0_f64.sqrt() + 3.0)).abs() < EPS);
    }

    #[test]
    fn placement_matches_java() {
        golden::check("sim_mountable", 1e-9, |op, x| {
            if op != "updated_xy" {
                return None;
            }
            let mount_type = if x[0] == 0.0 {
                MountType::Linear
            } else {
                MountType::Arm
            };
            let args = (
                mount_type, x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10],
            );
            let (t, a, b, c, d, e, f, g, h, i, j) = args;
            Some(vec![
                Placer.updated_x(t, a, b, c, d, e, f, g, h, i, j),
                Placer.updated_y(t, a, b, c, d, e, f, g, h, i, j),
            ])
        });
    }
}
