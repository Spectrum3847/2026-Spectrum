package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

public class VisionLogger {
    /** Tracks position with Limelight using current logger (DogLog) to record data */
    private final Limelight limelight;

    @Getter private String name;

    public VisionLogger(String name, Limelight limelight) {
        this.limelight = limelight;
        this.name = name;
    }

    public boolean getCameraConnection() {
        boolean connected = limelight.isCameraConnected();
        Telemetry.log("Vision/" + name + "/ConnectionStatus", connected);
        return connected;
    }

    public boolean getIntegratingStatus() {
        boolean integrating = limelight.isIntegrating();
        Telemetry.log("Vision/" + name + "/IntegratingStatus", integrating);
        return integrating;
    }

    public String getLogStatus() {
        String status = limelight.getLogStatus();
        Telemetry.log("Vision/" + name + "/LogStatus", status);
        return status;
    }

    public String getTagStatus() {
        String status = limelight.getTagStatus();
        Telemetry.log("Vision/" + name + "/TagStatus", status);
        return status;
    }

    public Pose2d getPose() {
        Pose2d pose = limelight.getMegaTag1_Pose3d().toPose2d();
        Telemetry.log("Vision/" + name + "/MT1Pose", pose);
        return pose;
    }

    public Pose2d getMegaPose() {
        Pose2d pose = limelight.getMegaTag2_Pose2d();
        Telemetry.log("Vision/" + name + "/MT2Pose", pose);
        return pose;
    }

    public double getTagCount() {
        double count = limelight.getTagCountInView();
        Telemetry.log("Vision/" + name + "/TagCount", count);
        return count;
    }

    public double getTargetSize() {
        double size = limelight.getTargetSize();
        Telemetry.log("Vision/" + name + "/TargetSize", size);
        return size;
    }
}
