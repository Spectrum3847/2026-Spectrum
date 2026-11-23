use crate::config::PidConstants;
use crate::trajectory::PathPlannerTrajectoryState;
use std::cell::RefCell;
use std::f64::consts::PI;
use std::rc::Rc;
use wpimath::controller::PidController;
use wpimath::geometry::Pose2d;
use wpimath::kinematics::ChassisSpeeds;

/// Turns a pose and a target trajectory state into robot-relative speeds.
pub trait PathFollowingController {
    fn calculate_robot_relative_speeds(
        &mut self,
        current_pose: Pose2d,
        target_state: &PathPlannerTrajectoryState,
    ) -> ChassisSpeeds;

    fn reset(&mut self, current_pose: Pose2d, current_speeds: ChassisSpeeds);

    fn is_holonomic(&self) -> bool;
}

/// One controller shared by every path following command, as in Java.
pub type SharedController = Rc<RefCell<dyn PathFollowingController>>;

/// Path following controller for holonomic drivetrains.
pub struct PpHolonomicDriveController {
    x_controller: PidController,
    y_controller: PidController,
    rotation_controller: PidController,
    is_enabled: bool,
}

impl PpHolonomicDriveController {
    pub fn new(translation: PidConstants, rotation: PidConstants) -> Self {
        Self::with_period(translation, rotation, 0.02)
    }

    pub fn with_period(translation: PidConstants, rotation: PidConstants, period: f64) -> Self {
        let pid = |c: PidConstants| {
            let mut pid = PidController::with_period(c.kp, c.ki, c.kd, period);
            pid.set_integrator_range(-c.i_zone, c.i_zone);
            pid
        };
        let mut rotation_controller = pid(rotation);
        rotation_controller.enable_continuous_input(-PI, PI);
        Self {
            x_controller: pid(translation),
            y_controller: pid(translation),
            rotation_controller,
            is_enabled: true,
        }
    }

    /// A disabled controller returns only the feedforward speeds, for troubleshooting.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.is_enabled = enabled;
    }
}

impl PathFollowingController for PpHolonomicDriveController {
    fn calculate_robot_relative_speeds(
        &mut self,
        current_pose: Pose2d,
        target_state: &PathPlannerTrajectoryState,
    ) -> ChassisSpeeds {
        let x_ff = target_state.field_speeds.vx;
        let y_ff = target_state.field_speeds.vy;

        if !self.is_enabled {
            return ChassisSpeeds::from_field_relative(
                ChassisSpeeds::new(x_ff, y_ff, 0.0),
                current_pose.rotation,
            );
        }

        let x_feedback = self
            .x_controller
            .calculate(current_pose.x(), target_state.pose.x());
        let y_feedback = self
            .y_controller
            .calculate(current_pose.y(), target_state.pose.y());

        let rotation_feedback = self.rotation_controller.calculate(
            current_pose.rotation.radians(),
            target_state.pose.rotation.radians(),
        );
        let rotation_ff = target_state.field_speeds.omega;

        ChassisSpeeds::from_field_relative(
            ChassisSpeeds::new(
                x_ff + x_feedback,
                y_ff + y_feedback,
                rotation_ff + rotation_feedback,
            ),
            current_pose.rotation,
        )
    }

    fn reset(&mut self, _current_pose: Pose2d, _current_speeds: ChassisSpeeds) {
        self.x_controller.reset();
        self.y_controller.reset();
        self.rotation_controller.reset();
    }

    fn is_holonomic(&self) -> bool {
        true
    }
}
