package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Logs telemetry data from a {@link Limelight} camera to the robot's data-logging system (DogLog)
 * under the {@code Vision/<name>/} namespace.
 *
 * <p>Every method logs the value it reads from the camera and returns it, so callers never query
 * the camera twice.
 */
public class VisionLogger {
    private final Limelight limelight;

    @Getter private String name;

    private final String connectionStatusKey;
    private final String integratingStatusKey;
    private final String logStatusKey;
    private final String tagStatusKey;
    private final String mt1PoseKey;
    private final String mt2PoseKey;
    private final String tagCountKey;
    private final String targetSizeKey;

    public VisionLogger(String name, Limelight limelight) {
        this.limelight = limelight;
        this.name = name;
        this.connectionStatusKey = "Vision/" + name + "/ConnectionStatus";
        this.integratingStatusKey = "Vision/" + name + "/IntegratingStatus";
        this.logStatusKey = "Vision/" + name + "/LogStatus";
        this.tagStatusKey = "Vision/" + name + "/TagStatus";
        this.mt1PoseKey = "Vision/" + name + "/MT1Pose";
        this.mt2PoseKey = "Vision/" + name + "/MT2Pose";
        this.tagCountKey = "Vision/" + name + "/TagCount";
        this.targetSizeKey = "Vision/" + name + "/TargetSize";
    }

    public boolean getCameraConnection() {
        boolean connected = limelight.isCameraConnected();
        Telemetry.log(connectionStatusKey, connected);
        return connected;
    }

    public boolean getIntegratingStatus() {
        boolean integrating = limelight.isIntegrating();
        Telemetry.log(integratingStatusKey, integrating);
        return integrating;
    }

    /** Why the camera last accepted or rejected integration, from the send status calls. */
    public String getLogStatus() {
        String status = limelight.getLogStatus();
        Telemetry.log(logStatusKey, status);
        return status;
    }

    /** What the camera last saw, from setTagStatus. */
    public String getTagStatus() {
        String status = limelight.getTagStatus();
        Telemetry.log(tagStatusKey, status);
        return status;
    }

    /** MegaTag1 pose projected to 2-D. */
    public Pose2d getPose() {
        Pose2d pose = limelight.getMegaTag1_Pose3d().toPose2d();
        Telemetry.log(mt1PoseKey, pose);
        return pose;
    }

    /** MegaTag2 pose, already 2-D. */
    public Pose2d getMegaPose() {
        Pose2d pose = limelight.getMegaTag2_Pose2d();
        Telemetry.log(mt2PoseKey, pose);
        return pose;
    }

    /** Tags in the current MegaTag1 estimate. */
    public double getTagCount() {
        double count = limelight.getTagCountInView();
        Telemetry.log(tagCountKey, count);
        return count;
    }

    /** Primary target area as a percentage of the image, 0 to 100. */
    public double getTargetSize() {
        double size = limelight.getTargetSize();
        Telemetry.log(targetSizeKey, size);
        return size;
    }
}
