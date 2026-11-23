//! Logs a [`Limelight`] camera to the wpilog under the `Vision/<name>/` namespace.
//!
//! Each method reads the value from the camera, forwards it to the telemetry log, and
//! returns it so callers do not have to query the camera twice.
//!
//! Log keys are built once in the constructor, so per-loop logging allocates no strings.

use super::limelight::Limelight;
use std::rc::Rc;
use telemetry::telemetry as log;
use wpimath::geometry::Pose2d;

pub struct VisionLogger {
    limelight: Rc<Limelight>,

    /// Namespace prefix used in all telemetry keys (`Vision/<name>/...`).
    pub name: String,

    connection_key: String,
    integrating_key: String,
    log_status_key: String,
    tag_status_key: String,
    mt1_pose_key: String,
    mt2_pose_key: String,
    tag_count_key: String,
    target_size_key: String,
    estimate_age_key: String,
    integrated_key: String,
    mount_tilt_key: String,
    mount_pitch_key: String,
    mount_roll_key: String,
    mount_height_key: String,
}

impl VisionLogger {
    pub fn new(name: &str, limelight: Rc<Limelight>) -> Self {
        let prefix = format!("Vision/{name}/");
        Self {
            limelight,
            name: name.to_owned(),
            connection_key: format!("{prefix}ConnectionStatus"),
            integrating_key: format!("{prefix}IntegratingStatus"),
            log_status_key: format!("{prefix}LogStatus"),
            tag_status_key: format!("{prefix}TagStatus"),
            mt1_pose_key: format!("{prefix}MT1Pose"),
            mt2_pose_key: format!("{prefix}MT2Pose"),
            tag_count_key: format!("{prefix}TagCount"),
            target_size_key: format!("{prefix}TargetSize"),
            estimate_age_key: format!("{prefix}EstimateAgeSeconds"),
            integrated_key: format!("{prefix}IntegratedThisLoop"),
            mount_tilt_key: format!("{prefix}MountCheck/TiltDeg"),
            mount_pitch_key: format!("{prefix}MountCheck/PitchDeg"),
            mount_roll_key: format!("{prefix}MountCheck/RollDeg"),
            mount_height_key: format!("{prefix}MountCheck/HeightMeters"),
        }
    }

    /// Age of this loop's estimate in seconds, or NaN if the camera produced none.
    pub fn get_estimate_age(&self) -> f64 {
        let age = self.limelight.get_last_estimate_age_seconds();
        // Elastic watches the turret camera's age on the dashboard.
        log::log_dash_with_unit(&self.estimate_age_key, age, "seconds");
        age
    }

    /// Whether an estimate was actually fused into the pose estimator this loop, as opposed
    /// to merely passing the acceptance tiers.
    pub fn get_integrated_this_loop(&self) -> bool {
        let integrated = self.limelight.is_integrated_this_loop();
        log::log(&self.integrated_key, integrated);
        integrated
    }

    pub fn get_camera_connection(&self) -> bool {
        let connected = self.limelight.is_camera_connected();
        log::log_dash(&self.connection_key, connected);
        connected
    }

    pub fn get_integrating_status(&self) -> bool {
        let integrating = self.limelight.is_integrating();
        log::log(&self.integrating_key, integrating);
        integrating
    }

    pub fn get_log_status(&self) -> String {
        let status = self.limelight.get_log_status();
        log::log(&self.log_status_key, &status);
        status
    }

    pub fn get_tag_status(&self) -> String {
        let status = self.limelight.get_tag_status();
        log::log(&self.tag_status_key, &status);
        status
    }

    /// Robot pose from the MegaTag1 estimate, projected to 2-D in the WPILib Blue origin
    /// frame.
    pub fn get_pose(&self) -> Pose2d {
        let pose = self.limelight.get_mega_tag1_pose3d().to_pose2d();
        log::log_struct(&self.mt1_pose_key, &pose);
        pose
    }

    /// Robot pose from the heading-fused MegaTag2 estimate, in the WPILib Blue origin frame.
    pub fn get_mega_pose(&self) -> Pose2d {
        let pose = self.limelight.get_mega_tag2_pose2d();
        log::log_struct(&self.mt2_pose_key, &pose);
        pose
    }

    /// Logs the out-of-plane part of the MegaTag1 estimate so the camera's mount angles can
    /// be checked against the values entered in its web UI.
    ///
    /// MegaTag1 reports the robot's full 3-D pose. It solves the camera's pose from the tag,
    /// then applies the camera-to-robot transform from the camera's entered offsets. With
    /// the robot sitting flat on the carpet the truth is height 0, pitch 0, roll 0, so
    /// whatever these read is the error in the entered mount rotation. `TiltDeg` is the
    /// total angle between the reported robot Z axis and field up, so it equals the
    /// magnitude of that error whatever the camera's yaw. A camera entered at 60 deg but
    /// really mounted at 55 shows about 5 deg of tilt.
    ///
    /// `PitchDeg` and `RollDeg` say which way the error points, but in the robot frame
    /// rather than the camera's: a pitch error of d on a camera yawed at psi lands as roll
    /// -d*sin(psi) and pitch d*cos(psi). On the rear cameras at yaw +/-135 that splits
    /// evenly between the two, and on the turret camera the split rotates with the turret,
    /// which is the other reason to read `TiltDeg`.
    ///
    /// Read these with the robot stationary on a flat floor and at least one tag in view.
    /// Real chassis tilt, from an obstacle or from weight transfer under acceleration, shows
    /// up here too. Nothing is logged when no tag is in view, so the values do not fall to
    /// zero between sightings. The MegaTag1 sample is read once per loop and cached, so this
    /// costs nothing beyond [`get_pose`](Self::get_pose).
    pub fn log_mount_check(&self) {
        if self.limelight.get_tag_count_in_view() < 1.0 {
            return;
        }
        let pose = self.limelight.get_mega_tag1_pose3d();
        let pitch = pose.rotation.y();
        let roll = pose.rotation.x();
        log::log_with_unit(&self.mount_tilt_key, pitch.hypot(roll).to_degrees(), "deg");
        log::log_with_unit(&self.mount_pitch_key, pitch.to_degrees(), "deg");
        log::log_with_unit(&self.mount_roll_key, roll.to_degrees(), "deg");
        log::log_with_unit(&self.mount_height_key, pose.z(), "meters");
    }

    pub fn get_tag_count(&self) -> f64 {
        let count = self.limelight.get_tag_count_in_view();
        log::log(&self.tag_count_key, count);
        count
    }

    /// Area of the primary target as a percentage of the camera image (0-100).
    pub fn get_target_size(&self) -> f64 {
        let size = self.limelight.get_target_size();
        log::log(&self.target_size_key, size);
        size
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_live_under_the_vision_namespace() {
        let limelight = Rc::new(Limelight::with_attached("limelight-back", false));
        let logger = VisionLogger::new("Back", limelight);
        assert_eq!(logger.name, "Back");
        assert_eq!(logger.connection_key, "Vision/Back/ConnectionStatus");
        assert_eq!(logger.integrating_key, "Vision/Back/IntegratingStatus");
        assert_eq!(logger.log_status_key, "Vision/Back/LogStatus");
        assert_eq!(logger.tag_status_key, "Vision/Back/TagStatus");
        assert_eq!(logger.mt1_pose_key, "Vision/Back/MT1Pose");
        assert_eq!(logger.mt2_pose_key, "Vision/Back/MT2Pose");
        assert_eq!(logger.tag_count_key, "Vision/Back/TagCount");
        assert_eq!(logger.target_size_key, "Vision/Back/TargetSize");
        assert_eq!(logger.estimate_age_key, "Vision/Back/EstimateAgeSeconds");
        assert_eq!(logger.integrated_key, "Vision/Back/IntegratedThisLoop");
        assert_eq!(logger.mount_tilt_key, "Vision/Back/MountCheck/TiltDeg");
        assert_eq!(logger.mount_pitch_key, "Vision/Back/MountCheck/PitchDeg");
        assert_eq!(logger.mount_roll_key, "Vision/Back/MountCheck/RollDeg");
        assert_eq!(
            logger.mount_height_key,
            "Vision/Back/MountCheck/HeightMeters"
        );
    }
}
