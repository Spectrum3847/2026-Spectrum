use crate::feedforwards::PerModule;
use std::cell::Cell;
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d};
use wpimath::kinematics::ChassisSpeeds;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldSymmetry {
    /// The red side is the blue side rotated by 180 degrees.
    Rotational,
    /// The field is mirrored over its vertical center line.
    Mirrored,
}

#[derive(Clone, Copy)]
struct Field {
    symmetry: FieldSymmetry,
    size_x: f64,
    size_y: f64,
}

thread_local! {
    // Java keeps these as mutable statics. The framework is single-threaded, so per thread
    // is the same thing and keeps parallel tests apart.
    static FIELD: Cell<Field> = const {
        Cell::new(Field {
            symmetry: FieldSymmetry::Rotational,
            size_x: 16.54,
            size_y: 8.07,
        })
    };
}

/// Flips positions and rotations to the other side of the field, keeping a blue origin.
pub struct FlippingUtil;

impl FlippingUtil {
    pub fn symmetry_type() -> FieldSymmetry {
        FIELD.get().symmetry
    }

    pub fn set_symmetry_type(symmetry: FieldSymmetry) {
        FIELD.set(Field {
            symmetry,
            ..FIELD.get()
        });
    }

    pub fn field_size_x() -> f64 {
        FIELD.get().size_x
    }

    pub fn field_size_y() -> f64 {
        FIELD.get().size_y
    }

    pub fn set_field_size(size_x: f64, size_y: f64) {
        FIELD.set(Field {
            size_x,
            size_y,
            ..FIELD.get()
        });
    }

    pub fn flip_field_position(pos: Translation2d) -> Translation2d {
        let field = FIELD.get();
        match field.symmetry {
            FieldSymmetry::Mirrored => Translation2d::new(field.size_x - pos.x, pos.y),
            FieldSymmetry::Rotational => {
                Translation2d::new(field.size_x - pos.x, field.size_y - pos.y)
            }
        }
    }

    pub fn flip_field_rotation(rotation: Rotation2d) -> Rotation2d {
        match FIELD.get().symmetry {
            FieldSymmetry::Mirrored => Rotation2d::PI - rotation,
            FieldSymmetry::Rotational => rotation - Rotation2d::PI,
        }
    }

    pub fn flip_field_pose(pose: Pose2d) -> Pose2d {
        Pose2d::new(
            Self::flip_field_position(pose.translation),
            Self::flip_field_rotation(pose.rotation),
        )
    }

    pub fn flip_field_speeds(speeds: ChassisSpeeds) -> ChassisSpeeds {
        match FIELD.get().symmetry {
            FieldSymmetry::Mirrored => ChassisSpeeds::new(-speeds.vx, speeds.vy, -speeds.omega),
            FieldSymmetry::Rotational => ChassisSpeeds::new(-speeds.vx, -speeds.vy, speeds.omega),
        }
    }

    /// Swaps left and right modules under mirrored symmetry, otherwise returns the input.
    pub fn flip_feedforwards(values: &PerModule) -> PerModule {
        match FIELD.get().symmetry {
            FieldSymmetry::Mirrored => values.swapped_sides(),
            FieldSymmetry::Rotational => *values,
        }
    }

    pub fn flip_feedforward_xs(values: &PerModule) -> PerModule {
        Self::flip_feedforwards(values)
    }

    /// Y components also change sign under mirrored symmetry.
    pub fn flip_feedforward_ys(values: &PerModule) -> PerModule {
        let flipped = Self::flip_feedforwards(values);
        match FIELD.get().symmetry {
            FieldSymmetry::Mirrored => flipped.negated(),
            FieldSymmetry::Rotational => flipped,
        }
    }
}
