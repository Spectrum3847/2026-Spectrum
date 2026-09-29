package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.spectrumLib.vision.LimelightHelpers.LimelightResults;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * Wraps one Limelight camera and returns its AprilTag pose estimates and target offsets.
 *
 * <p>Getters return zero, false, or an empty value instead of throwing when the camera is not
 * attached, so callers do not need to check {@link #isAttached()} first. The exception is {@link
 * #getTagTx()}, {@link #getTagTA()} and {@link #getTagRotationDegrees()}, which return -99999 when
 * the camera is unattached or no target is in view, so a missing target must never be read as a
 * measurement.
 */
public class Limelight {

    /**
     * Network-table name and robot-frame mount pose for one camera. Translation is in metres and
     * rotation in degrees. {@code @Accessors} makes the setters chainable.
     */
    @Accessors(chain = true)
    public static class LimelightConfig {
        /** Must match the name given in the LL dashboard */
        @Getter @Setter private String name;

        @Getter @Setter private boolean attached = true;

        @Getter @Setter private boolean isIntegrating;

        /** Positive is toward the front of the robot. */
        @Getter private double forward, right, up; // meters

        @Getter private double roll, pitch, yaw; // degrees

        public LimelightConfig(String name) {
            this.name = name;
        }

        public LimelightConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /**
         * Positive roll rotates the camera to the right, positive pitch tilts it up, positive yaw
         * rotates it to the left.
         */
        public LimelightConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }
    }

    private final DecimalFormat df = new DecimalFormat();

    private LimelightConfig config;

    @Getter @Setter private boolean isIntegrating = false;

    @Getter private String cameraName = "default";

    @Getter @Setter private String logStatus = "";

    @Getter @Setter private String tagStatus = "";

    public Limelight(LimelightConfig config) {
        this.config = config;
        cameraName = config.getName();
    }

    public Limelight(String name) {
        cameraName = name;
        config = new LimelightConfig(name);
    }

    public Limelight(String name, boolean attached) {
        cameraName = name;
        config = new LimelightConfig(name).setAttached(attached);
    }

    public Limelight(String name, int pipeline) {
        this(name);
        cameraName = name;
        setLimelightPipeline(pipeline);
    }

    public Limelight(String name, int pipeline, LimelightConfig config) {
        this(name);
        cameraName = name;
        this.config = config;
        setLimelightPipeline(pipeline);
    }

    public String getName() {
        return config.getName();
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * Horizontal offset from the crosshair to the target, in degrees. LL1 spans -27 to 27, LL2
     * -29.8 to 29.8.
     */
    public double getHorizontalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTX(config.getName());
    }

    /**
     * Vertical offset from the crosshair to the target, in degrees. LL1 spans -20.5 to 20.5, LL2
     * -24.85 to 24.85.
     */
    public double getVerticalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTY(config.getName());
    }

    public boolean targetInView() {
        if (!isAttached()) {
            return false;
        }
        return LimelightHelpers.getTV(config.getName());
    }

    public boolean multipleTagsInView() {
        if (!isAttached()) {
            return false;
        }
        return getTagCountInView() > 1;
    }

    /** Tag count in the current MegaTag1 estimate. */
    public double getTagCountInView() {
        if (!isAttached()) {
            return 0;
        }
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName());
        if (est == null) {
            return 0;
        }
        return est.tagCount;
    }

    /** ID of the most centered AprilTag, following the primary target rule in the LL dashboard. */
    public double getClosestTagID() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getFiducialID(config.getName());
    }

    /** Primary target area as a percentage of the image, 0 to 100. */
    public double getTargetSize() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTA(config.getName());
    }

    /**
     * Robot pose in the WPILib Blue origin frame, which the Limelight picks from the DriverStation
     * alliance. Returns {@link Pose3d#kZero} when no estimate is available.
     */
    public Pose3d getMegaTag1_Pose3d() {
        if (!isAttached()) {
            return Pose3d.kZero;
        }
        Pose3d pose3d = LimelightHelpers.getBotPose3d_wpiBlue(config.name);
        if (pose3d == null) {
            return Pose3d.kZero;
        }
        return pose3d;
    }

    /**
     * Robot pose in the WPILib Blue origin frame. Returns {@link Pose2d#kZero} when no estimate is
     * available.
     */
    public Pose2d getMegaTag2_Pose2d() {
        if (!isAttached()) {
            return Pose2d.kZero;
        }
        PoseEstimate poseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
        if (poseEstimate == null) {
            return Pose2d.kZero;
        }
        return poseEstimate.pose;
    }

    /** MegaTag1 estimate in the WPILib Blue origin frame; empty when no data is available. */
    public PoseEstimate getMegaTag1_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }

        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
        if (poseEstimate == null) {
            return new PoseEstimate();
        }
        return poseEstimate;
    }

    /**
     * Heading-fused MegaTag2 estimate in the WPILib Blue origin frame; empty when no data is
     * available.
     */
    public PoseEstimate getMegaTag2_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }

        PoseEstimate poseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
        if (poseEstimate == null) {
            return new PoseEstimate();
        }
        return poseEstimate;
    }

    public boolean hasAccuratePose() {
        if (!isAttached()) {
            return false;
        }
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    /** Horizontal distance from the camera to the closest AprilTag, in metres. */
    public double getDistanceToTagFromCamera() {
        if (!isAttached()) {
            return 0;
        }
        double x = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getX();
        double y = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getZ();
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /** Raw AprilTag detections from the MegaTag1 estimate; empty when no data is available. */
    public RawFiducial[] getRawFiducial() {
        if (!isAttached()) {
            return new RawFiducial[0];
        }
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
        if (est == null || est.rawFiducials == null) {
            return new RawFiducial[0];
        }
        return est.rawFiducials;
    }

    /** Capture time of the current MegaTag1 estimate, in seconds. */
    public double getMegaTag1PoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName());
        if (est == null) {
            return 0;
        }
        return est.timestampSeconds;
    }

    /** Capture time of the current MegaTag2 estimate, in seconds. */
    public double getMegaTag2PoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.getName());
        if (est == null) {
            return 0;
        }
        return est.timestampSeconds;
    }

    /** Latency of the pose estimate, in seconds. */
    @Deprecated(forRemoval = true)
    public double getPoseLatency() {
        if (!isAttached()) {
            return 0;
        }
        return Units.millisecondsToSeconds(
                LimelightHelpers.getBotPose_wpiBlue(config.getName())[6]);
    }

    /**
     * Horizontal distance to a target of known height, from the camera's own height and tilt.
     *
     * @param targetHeight height of the target center above the floor, in metres
     * @return distance to the target in metres
     */
    public double getDistanceToTarget(double targetHeight) {
        if (!isAttached()) {
            return 0;
        }
        return (targetHeight - config.up)
                / Math.tan(Units.degreesToRadians(config.pitch + getVerticalOffset()));
    }

    /** Marks the camera as integrating and stores {@code message} as its log status. */
    public void sendValidStatus(String message) {
        config.isIntegrating = true;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    /** Marks the camera as not integrating and stores {@code message} as its log status. */
    public void sendInvalidStatus(String message) {
        config.isIntegrating = false;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    /** Despite the name, this returns parsed results rather than raw JSON. */
    @SuppressWarnings("unused")
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(config.name);
    }

    public void setLimelightPipeline(int pipelineIndex) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setPipelineIndex(config.name, pipelineIndex);
    }

    /** Heading in degrees, used by MegaTag2 heading fusion. */
    public void setRobotOrientation(double degrees) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, 0, 0, 0, 0, 0);
    }

    /**
     * Feeds the heading and yaw rate that the camera fuses against its own IMU.
     *
     * @param degrees robot heading in degrees, positive counter-clockwise
     * @param angularRate current yaw rate in degrees per second
     */
    public void setRobotOrientation(double degrees, double angularRate) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, angularRate, 0, 0, 0, 0);
    }

    /**
     * Picks which IMU the camera fuses against.
     *
     * @param mode mode index from the Limelight documentation
     */
    public void setIMUmode(int mode) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetIMUMode(config.name, mode);
    }

    /**
     * Target offset along the robot's left/right axis, in metres. Returns -99999 when no target is
     * in view.
     */
    public double getTagTx() {
        if (!isAttached()) {
            return -99999;
        }

        if (!targetInView()) {
            return -99999;
        }

        double tx = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getX();

        return tx;
    }

    /**
     * Primary target area as a percentage of the image. Returns -99999 when no target is in view.
     */
    public double getTagTA() {
        if (!isAttached()) {
            return -99999;
        }
        if (!targetInView()) {
            return -99999;
        }

        double ta = LimelightHelpers.getTA(cameraName);

        return ta;
    }

    /** Target yaw in the robot frame, in degrees. Returns -99999 when no target is in view. */
    public double getTagRotationDegrees() {
        if (!isAttached()) {
            return -99999;
        }
        if (!targetInView()) {
            return -99999;
        }

        double rotationRadians =
                LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getRotation().getZ();

        return Math.toDegrees(rotationRadians);
    }

    public void setLEDMode(boolean enabled) {
        if (!isAttached()) {
            return;
        }
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(config.getName());
        } else {
            LimelightHelpers.setLEDMode_ForceOff(config.getName());
        }
    }

    public void blinkLEDs() {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setLEDMode_ForceBlink(config.getName());
    }

    /** Treats a missing or truncated botpose entry from the camera as disconnected. */
    public boolean isCameraConnected() {
        if (!isAttached()) {
            return false;
        }
        try {
            var rawPoseArray =
                    LimelightHelpers.getLimelightNTTableEntry(config.getName(), "botpose_wpiblue")
                            .getDoubleArray(new double[0]);
            if (rawPoseArray.length < 6) {
                return false;
            }
            return true;
        } catch (Exception e) {
            System.err.println("Avoided crashing statement in Limelight.java: isCameraConnected()");
            return false;
        }
    }

    public void printDebug() {
        if (!isAttached()) {
            return;
        }
        Pose3d botPose3d = getMegaTag1_Pose3d();
        SmartDashboard.putString("LimelightX", df.format(botPose3d.getTranslation().getX()));
        SmartDashboard.putString("LimelightY", df.format(botPose3d.getTranslation().getY()));
        SmartDashboard.putString("LimelightZ", df.format(botPose3d.getTranslation().getZ()));
        SmartDashboard.putString(
                "LimelightRoll", df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
        SmartDashboard.putString(
                "LimelightPitch",
                df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
        SmartDashboard.putString(
                "LimelightYaw", df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
    }
}
