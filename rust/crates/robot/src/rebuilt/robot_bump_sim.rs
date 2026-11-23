// Port of frc.rebuilt.RobotBumpSim.
//
// Copyright (c) 2025-2026 KAISER 6989
// https://github.com/haar09/FRC-Rebuilt-BumpSim
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

//! Bump physics for the 2026 REBUILT field, to run alongside the swerve sim. Tracks the
//! robot's Z height, pitch and roll over the raised bumps, and takes over field X while the
//! robot is on a ramp so it slides back rather than ghosting through.
//!
//! Cresting a bump needs 1.8 m/s, from sqrt(2 * g * 0.165).
//!
//! While [`RobotBumpSim::is_on_ramp`] is true the caller must push
//! [`RobotBumpSim::get_sim_world_pose`] into the swerve sim. Positions are in meters, with
//! the origin at the blue alliance driver-station corner.

use super::field::{FIELD_LENGTH, FIELD_WIDTH};
use wpimath::geometry::{Pose2d, Pose3d, Rotation3d, Translation2d, Translation3d};
use wpimath::kinematics::ChassisSpeeds;

#[cfg(test)]
#[path = "robot_bump_sim/tests.rs"]
mod tests;

/// Robot control-loop period in seconds, matching the WPILib default.
const PERIOD: f64 = 0.02;

/// Z component of gravitational acceleration, m/s^2.
const GRAVITY_Z: f64 = -9.81;

/// Start points of the eight bump XZ line segments, as (fieldX, yMin, fieldZ): where the face
/// starts in X, the lowest field-Y it spans, and the ramp height there. Indices 0 and 1 are
/// the Blue ascending faces, 2 and 3 the Blue descending, 4 and 5 the Red ascending, 6 and 7
/// the Red descending. [`BUMP_LINE_ENDS`] holds the matching end points.
pub(crate) const BUMP_LINE_STARTS: [Translation3d; 8] = [
    Translation3d::new(3.96, 1.57, 0.0),
    Translation3d::new(3.96, FIELD_WIDTH / 2.0 + 0.60, 0.0),
    Translation3d::new(4.61, 1.57, 0.165),
    Translation3d::new(4.61, FIELD_WIDTH / 2.0 + 0.60, 0.165),
    Translation3d::new(FIELD_LENGTH - 5.18, 1.57, 0.0),
    Translation3d::new(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2.0 + 0.60, 0.0),
    Translation3d::new(FIELD_LENGTH - 4.61, 1.57, 0.165),
    Translation3d::new(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2.0 + 0.60, 0.165),
];

/// End points of the eight bump XZ line segments, as (fieldX, yMax, fieldZ). The index
/// layout matches [`BUMP_LINE_STARTS`].
pub(crate) const BUMP_LINE_ENDS: [Translation3d; 8] = [
    Translation3d::new(4.61, FIELD_WIDTH / 2.0 - 0.60, 0.165),
    Translation3d::new(4.61, FIELD_WIDTH - 1.57, 0.165),
    Translation3d::new(5.18, FIELD_WIDTH / 2.0 - 0.60, 0.0),
    Translation3d::new(5.18, FIELD_WIDTH - 1.57, 0.0),
    Translation3d::new(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2.0 - 0.60, 0.165),
    Translation3d::new(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
    Translation3d::new(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2.0 - 0.60, 0.0),
    Translation3d::new(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0.0),
];

/// Effective wheel contact radius against the bump surface, in meters.
const WHEEL_RADIUS: f64 = 0.048;

/// Height offset from the average module-contact Z to the robot-body origin, in meters.
const CHASSIS_HEIGHT: f64 = 0.0;

/// Coefficient of restitution for vertical collisions with the bump. 0 is fully inelastic,
/// 1 is fully elastic.
const BUMP_COR: f64 = 0.15;

/// Tolerance so rounding at a segment endpoint does not read as a miss.
const SEGMENT_PROJECTION_TOLERANCE: f64 = 1e-6;

pub struct RobotBumpSim {
    /// Robot-relative module positions, order: FL, FR, BL, BR.
    module_offsets: [Translation2d; 4],
    /// Absolute Z position of each module contact point, meters above the floor.
    module_z_pos: [f64; 4],
    /// Z velocity of each module contact point, m/s.
    module_z_vel: [f64; 4],
    /// Distance between front and back module pairs along the robot X axis, in meters.
    front_back_dist: f64,
    /// Distance between left and right module pairs along the robot Y axis, in meters.
    left_right_dist: f64,
    /// True while this sim owns the robot's field-X position.
    on_ramp: bool,
    /// Field-X position owned by the ramp model, in meters. Ignored while `on_ramp` is false.
    sim_x_pos: f64,
    /// Field-X velocity owned by the ramp model, m/s, seeded from the robot's field-Vx at
    /// first contact and decelerated by gravity along the ramp each subtick. Ignored while
    /// `on_ramp` is false.
    sim_x_vel: f64,
}

impl RobotBumpSim {
    /// `module_offsets` are robot-relative module positions in FL, FR, BL, BR order, meters.
    pub fn new(module_offsets: [Translation2d; 4]) -> Self {
        let front_x = (module_offsets[0].x + module_offsets[1].x) / 2.0;
        let back_x = (module_offsets[2].x + module_offsets[3].x) / 2.0;
        let front_back_dist = (front_x - back_x).abs().max(1e-3);

        let left_y = (module_offsets[0].y + module_offsets[2].y) / 2.0;
        let right_y = (module_offsets[1].y + module_offsets[3].y) / 2.0;
        let left_right_dist = (left_y - right_y).abs().max(1e-3);

        Self {
            module_offsets,
            module_z_pos: [0.0; 4],
            module_z_vel: [0.0; 4],
            front_back_dist,
            left_right_dist,
            on_ramp: false,
            sim_x_pos: 0.0,
            sim_x_vel: 0.0,
        }
    }

    /// While this returns true the caller must push [`Self::get_sim_world_pose`] into the
    /// swerve sim, so the robot slides back instead of only appearing to.
    pub fn is_on_ramp(&self) -> bool {
        self.on_ramp
    }

    /// X comes from the ramp model. Y and rotation pass through from `latest_maple_pose` so
    /// the swerve sim keeps owning lateral motion.
    pub fn get_sim_world_pose(&self, latest_maple_pose: Pose2d) -> Pose2d {
        Pose2d::from_xy(
            self.sim_x_pos,
            latest_maple_pose.y(),
            latest_maple_pose.rotation,
        )
    }

    /// Advances the sim by one control period and returns the robot's 3D pose. X is the ramp
    /// model's position while on the ramp, so the caller must also push
    /// [`Self::get_sim_world_pose`] into the swerve sim.
    ///
    /// `subticks` is the number of physics sub-steps per period, which must match the value
    /// a companion ball sim uses. 5 gives 4 ms sub-steps in a 20 ms loop.
    pub fn update(
        &mut self,
        robot_pose2d: Pose2d,
        field_relative_speeds: ChassisSpeeds,
        subticks: i32,
    ) -> Pose3d {
        let vx = field_relative_speeds.vx;
        let dt = PERIOD / f64::from(subticks);

        // A diagonal approach eases the climb: the factor is abs(cos(2 * heading)), so it is
        // 1.0 straight across the bump, 0 at 45 degrees to it, and back to 1.0 at 90 degrees.
        let contact_factor = (2.0 * robot_pose2d.rotation.radians()).cos().abs();

        // Module Y is owned by the swerve sim and fixed for the whole period
        let mut world_y = [0.0; 4];
        for (y, offset) in world_y.iter_mut().zip(&self.module_offsets) {
            let wo = offset.rotate_by(robot_pose2d.rotation);
            *y = robot_pose2d.y() + wo.y;
        }

        for _ in 0..subticks {
            let current_robot_x = if self.on_ramp {
                self.sim_x_pos
            } else {
                robot_pose2d.x()
            };

            let mut grav_accel_x_sum = 0.0;
            let mut contact_count = 0;

            for (i, &y) in world_y.iter().enumerate() {
                let wo = self.module_offsets[i].rotate_by(robot_pose2d.rotation);
                let wx = current_robot_x + wo.x;

                self.module_z_vel[i] += GRAVITY_Z * dt;
                self.module_z_pos[i] += self.module_z_vel[i] * dt;

                let x_vel = if self.on_ramp { self.sim_x_vel } else { vx };
                for line_idx in 0..BUMP_LINE_STARTS.len() {
                    if let Some(gax) = self.handle_module_bump_collision(i, wx, y, x_vel, line_idx)
                    {
                        grav_accel_x_sum += gax;
                        contact_count += 1;
                    }
                }

                if self.module_z_pos[i] < 0.0 {
                    self.module_z_pos[i] = 0.0;
                    if self.module_z_vel[i] < 0.0 {
                        self.module_z_vel[i] = -self.module_z_vel[i] * BUMP_COR;
                    }
                }
            }

            if contact_count > 0 {
                if !self.on_ramp {
                    self.on_ramp = true;
                    self.sim_x_pos = robot_pose2d.x();
                    self.sim_x_vel = vx;
                }
                let avg_grav_accel_x =
                    (grav_accel_x_sum / f64::from(contact_count)) * contact_factor;
                self.sim_x_vel += avg_grav_accel_x * dt;
                self.sim_x_pos += self.sim_x_vel * dt;
            } else if self.on_ramp {
                let all_flat = !self.module_z_pos.iter().any(|z| *z > 0.01);
                if all_flat {
                    // Backed out or crossed, so the swerve sim reclaims X
                    self.on_ramp = false;
                } else {
                    // Airborne over the peak; keep coasting on sim_x_vel
                    self.sim_x_pos += self.sim_x_vel * dt;
                }
            }
        }

        self.compute_pose3d(robot_pose2d)
    }

    /// Resolves module `module_idx` against bump segment `line_idx` in the XZ plane,
    /// correcting Z position and applying a Z impulse. Returns the gravity-along-ramp X
    /// acceleration in m/s^2, or `None` with no contact. Gravity opposes travel up an
    /// ascending face and adds to it on a descending one. The surface is frictionless, so
    /// the drivetrain contributes no force here.
    fn handle_module_bump_collision(
        &mut self,
        module_idx: usize,
        world_x: f64,
        world_y: f64,
        current_x_vel: f64,
        line_idx: usize,
    ) -> Option<f64> {
        let line_start = BUMP_LINE_STARTS[line_idx];
        let line_end = BUMP_LINE_ENDS[line_idx];

        if world_y < line_start.y || world_y > line_end.y {
            return None;
        }

        let start2d = Translation2d::new(line_start.x, line_start.z);
        let end2d = Translation2d::new(line_end.x, line_end.z);
        let pos2d = Translation2d::new(world_x, self.module_z_pos[module_idx]);
        let line_vec = end2d - start2d;

        let to_module = pos2d - start2d;
        let projection_t = to_module.dot(line_vec) / line_vec.squared_norm();
        let projected = start2d + line_vec * projection_t;

        if projected.distance(start2d) + projected.distance(end2d)
            > line_vec.norm() + SEGMENT_PROJECTION_TOLERANCE
        {
            return None; // off segment
        }

        let dist = pos2d.distance(projected);
        if dist > WHEEL_RADIUS {
            return None;
        }

        let normal_x = -line_vec.y / line_vec.norm();
        let normal_z = line_vec.x / line_vec.norm();

        self.module_z_pos[module_idx] += normal_z * (WHEEL_RADIUS - dist);

        // Z velocity impulse. The X model is unaffected.
        let vel_dot_normal = current_x_vel * normal_x + self.module_z_vel[module_idx] * normal_z;
        if vel_dot_normal < 0.0 {
            self.module_z_vel[module_idx] += normal_z * (-(1.0 + BUMP_COR) * vel_dot_normal);
        }

        Some(-GRAVITY_Z * normal_x * normal_z)
    }

    fn compute_pose3d(&self, robot_pose2d: Pose2d) -> Pose3d {
        let z = &self.module_z_pos;
        let front_z = (z[0] + z[1]) / 2.0;
        let back_z = (z[2] + z[3]) / 2.0;
        let left_z = (z[0] + z[2]) / 2.0;
        let right_z = (z[1] + z[3]) / 2.0;
        let center_z = (front_z + back_z) / 2.0 + CHASSIS_HEIGHT;

        // Pitch is negated: WPILib takes positive Rotation3d pitch as nose-down
        let pitch = -(front_z - back_z).atan2(self.front_back_dist);
        // Roll: positive when the left side is higher
        let roll = (left_z - right_z).atan2(self.left_right_dist);

        let visual_x = if self.on_ramp {
            self.sim_x_pos
        } else {
            robot_pose2d.x()
        };

        Pose3d::new(
            Translation3d::new(visual_x, robot_pose2d.y(), center_z),
            Rotation3d::from_euler(roll, pitch, robot_pose2d.rotation.radians()),
        )
    }
}
