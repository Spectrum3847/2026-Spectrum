package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Logs telemetry data from a {@link Limelight} camera to the robot's data-logging system (DogLog)
 * under the {@code Vision/<name>/} namespace.
 *
 * <p>Each method reads the corresponding value from the camera, forwards it to {@link
 * frc.spectrumLib.telemetry.Telemetry#log}, and also returns the value so callers can use it
 * directly without a second camera query.
 */
public class VisionLogger {
    /** The Limelight camera whose data is being logged. */
    private final Limelight limelight;

    /** Namespace prefix used in all telemetry keys ({@code Vision/<name>/...}). */
    @Getter private String name;

    /**
     * Constructs a logger for the given Limelight camera.
     *
     * @param name the namespace prefix used in telemetry keys
     * @param limelight the camera to read from
     */
    public VisionLogger(String name, Limelight limelight) {
        this.limelight = limelight;
        this.name = name;
    }

    /**
     * Logs and returns whether the camera is currently connected.
     *
     * @return {@code true} if the camera is reachable over the network
     */
    public boolean getCameraConnection() {
        boolean connected = limelight.isCameraConnected();
        Telemetry.log("Vision/" + name + "/ConnectionStatus", connected);
        return connected;
    }

    /**
     * Logs and returns whether pose measurements are currently being fused into the estimator.
     *
     * @return {@code true} if the camera is actively integrating
     */
    public boolean getIntegratingStatus() {
        boolean integrating = limelight.isIntegrating();
        Telemetry.log("Vision/" + name + "/IntegratingStatus", integrating);
        return integrating;
    }

    /**
     * Logs and returns the camera's human-readable integration status message.
     *
     * @return the current log status string from the camera
     */
    public String getLogStatus() {
        String status = limelight.getLogStatus();
        Telemetry.log("Vision/" + name + "/LogStatus", status);
        return status;
    }

    /**
     * Logs and returns the camera's human-readable tag-detection status message.
     *
     * @return the current tag status string from the camera
     */
    public String getTagStatus() {
        String status = limelight.getTagStatus();
        Telemetry.log("Vision/" + name + "/TagStatus", status);
        return status;
    }

    /**
     * Logs and returns the robot's 2-D pose derived from the MegaTag1 estimate.
     *
     * @return the MegaTag1 pose projected to 2-D in the WPILib Blue origin frame
     */
    public Pose2d getPose() {
        Pose2d pose = limelight.getMegaTag1_Pose3d().toPose2d();
        Telemetry.log("Vision/" + name + "/MT1Pose", pose);
        return pose;
    }

    /**
     * Logs and returns the robot's 2-D pose from the MegaTag2 (heading-fused) estimate.
     *
     * @return the MegaTag2 {@link Pose2d} in the WPILib Blue origin frame
     */
    public Pose2d getMegaPose() {
        Pose2d pose = limelight.getMegaTag2_Pose2d();
        Telemetry.log("Vision/" + name + "/MT2Pose", pose);
        return pose;
    }

    /**
     * Logs and returns the number of AprilTags contributing to the current pose estimate.
     *
     * @return the tag count from the MegaTag1 estimate
     */
    public double getTagCount() {
        double count = limelight.getTagCountInView();
        Telemetry.log("Vision/" + name + "/TagCount", count);
        return count;
    }

    /**
     * Logs and returns the area of the primary target as a percentage of the camera image.
     *
     * @return target area (0–100 %)
     */
    public double getTargetSize() {
        double size = limelight.getTargetSize();
        Telemetry.log("Vision/" + name + "/TargetSize", size);
        return size;
    }
}
