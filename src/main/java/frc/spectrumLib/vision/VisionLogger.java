package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Logs a {@link Limelight} camera to the wpilog under the {@code Vision/<name>/} namespace.
 *
 * <p>Each method reads the value from the camera, forwards it to {@link
 * frc.spectrumLib.telemetry.Telemetry#log}, and returns it so callers do not have to query the
 * camera twice.
 *
 * <p>Log keys are built once in the constructor, so per-loop logging allocates no strings.
 */
public class VisionLogger {
    private final Limelight limelight;

    /** Namespace prefix used in all telemetry keys ({@code Vision/<name>/...}). */
    @Getter private final String name;

    private final String connectionKey;
    private final String integratingKey;
    private final String logStatusKey;
    private final String tagStatusKey;
    private final String mt1PoseKey;
    private final String mt2PoseKey;
    private final String tagCountKey;
    private final String targetSizeKey;
    private final String estimateAgeKey;
    private final String integratedKey;
    private final String mountTiltKey;
    private final String mountPitchKey;
    private final String mountRollKey;
    private final String mountHeightKey;

    public VisionLogger(String name, Limelight limelight) {
        this.limelight = limelight;
        this.name = name;
        String prefix = "Vision/" + name + "/";
        connectionKey = prefix + "ConnectionStatus";
        integratingKey = prefix + "IntegratingStatus";
        logStatusKey = prefix + "LogStatus";
        tagStatusKey = prefix + "TagStatus";
        mt1PoseKey = prefix + "MT1Pose";
        mt2PoseKey = prefix + "MT2Pose";
        tagCountKey = prefix + "TagCount";
        targetSizeKey = prefix + "TargetSize";
        estimateAgeKey = prefix + "EstimateAgeSeconds";
        integratedKey = prefix + "IntegratedThisLoop";
        mountTiltKey = prefix + "MountCheck/TiltDeg";
        mountPitchKey = prefix + "MountCheck/PitchDeg";
        mountRollKey = prefix + "MountCheck/RollDeg";
        mountHeightKey = prefix + "MountCheck/HeightMeters";
    }

    /** Age of this loop's estimate in seconds, or NaN if the camera produced none. */
    public double getEstimateAge() {
        double age = limelight.getLastEstimateAgeSeconds();
        // Elastic watches the turret camera's age on the dashboard.
        Telemetry.logDash(estimateAgeKey, age, "seconds");
        return age;
    }

    /**
     * Whether an estimate was actually fused into the pose estimator this loop, as opposed to
     * merely passing the acceptance tiers.
     */
    public boolean getIntegratedThisLoop() {
        boolean integrated = limelight.isIntegratedThisLoop();
        Telemetry.log(integratedKey, integrated);
        return integrated;
    }

    public boolean getCameraConnection() {
        boolean connected = limelight.isCameraConnected();
        Telemetry.logDash(connectionKey, connected);
        return connected;
    }

    public boolean getIntegratingStatus() {
        boolean integrating = limelight.isIntegrating();
        Telemetry.log(integratingKey, integrating);
        return integrating;
    }

    public String getLogStatus() {
        String status = limelight.getLogStatus();
        Telemetry.log(logStatusKey, status);
        return status;
    }

    public String getTagStatus() {
        String status = limelight.getTagStatus();
        Telemetry.log(tagStatusKey, status);
        return status;
    }

    /** Robot pose from the MegaTag1 estimate, projected to 2-D in the WPILib Blue origin frame. */
    public Pose2d getPose() {
        Pose2d pose = limelight.getMegaTag1_Pose3d().toPose2d();
        Telemetry.log(mt1PoseKey, pose);
        return pose;
    }

    /** Robot pose from the heading-fused MegaTag2 estimate, in the WPILib Blue origin frame. */
    public Pose2d getMegaPose() {
        Pose2d pose = limelight.getMegaTag2_Pose2d();
        Telemetry.log(mt2PoseKey, pose);
        return pose;
    }

    /**
     * Logs the out-of-plane part of the MegaTag1 estimate so the camera's mount angles can be
     * checked against the values entered in its web UI.
     *
     * <p>MegaTag1 reports the robot's full 3-D pose. It solves the camera's pose from the tag, then
     * applies the camera-to-robot transform from the camera's entered offsets. With the robot
     * sitting flat on the carpet the truth is height 0, pitch 0, roll 0, so whatever these read is
     * the error in the entered mount rotation. TiltDeg is the total angle between the reported
     * robot Z axis and field up, so it equals the magnitude of that error whatever the camera's
     * yaw. A camera entered at 60 deg but really mounted at 55 shows about 5 deg of tilt.
     *
     * <p>PitchDeg and RollDeg say which way the error points, but in the robot frame rather than
     * the camera's: a pitch error of d on a camera yawed at psi lands as roll -d*sin(psi) and pitch
     * d*cos(psi). On the rear cameras at yaw +/-135 that splits evenly between the two, and on the
     * turret camera the split rotates with the turret, which is the other reason to read TiltDeg.
     *
     * <p>Read these with the robot stationary on a flat floor and at least one tag in view. Real
     * chassis tilt, from an obstacle or from weight transfer under acceleration, shows up here too.
     * Nothing is logged when no tag is in view, so the values do not fall to zero between
     * sightings. The MegaTag1 sample is read once per loop and cached, so this costs nothing beyond
     * {@link #getPose()}.
     */
    public void logMountCheck() {
        if (limelight.getTagCountInView() < 1) {
            return;
        }
        Pose3d pose = limelight.getMegaTag1_Pose3d();
        double pitch = pose.getRotation().getY();
        double roll = pose.getRotation().getX();
        Telemetry.log(mountTiltKey, Units.radiansToDegrees(Math.hypot(pitch, roll)), "deg");
        Telemetry.log(mountPitchKey, Units.radiansToDegrees(pitch), "deg");
        Telemetry.log(mountRollKey, Units.radiansToDegrees(roll), "deg");
        Telemetry.log(mountHeightKey, pose.getZ(), "meters");
    }

    public double getTagCount() {
        double count = limelight.getTagCountInView();
        Telemetry.log(tagCountKey, count);
        return count;
    }

    /** Area of the primary target as a percentage of the camera image (0-100). */
    public double getTargetSize() {
        double size = limelight.getTargetSize();
        Telemetry.log(targetSizeKey, size);
        return size;
    }
}
