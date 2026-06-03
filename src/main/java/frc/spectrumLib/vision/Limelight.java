package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.Vision.VisionConfig;
import frc.spectrumLib.vision.LimelightHelpers.LimelightResults;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * Provides a high-level interface to a single Limelight camera for AprilTag-based pose estimation
 * and basic targeting.
 *
 * <p>All methods are safe to call when the camera is not attached ({@link #isAttached()} returns
 * {@code false}); they return zero / false / empty values in that case.
 *
 * <p>Two pose estimation flavors are supported:
 *
 * <ul>
 *   <li><b>MegaTag1</b> — full 3-D pose estimation using one or more AprilTags.
 *   <li><b>MegaTag2</b> — fused estimate that incorporates the robot heading supplied via {@link
 *       #setRobotOrientation(double)}.
 * </ul>
 */
public class Limelight {

    /* Limelight Configuration */

    /**
     * Configuration for a single Limelight camera, including its network-table name and physical
     * mounting position on the robot.
     *
     * <p>The Lombok {@code @Accessors(chain = true)} annotation allows fluent setter calls: {@code
     * config.setName("limelight").setAttached(true)}.
     */
    @Accessors(chain = true)
    public static class LimelightConfig {
        /** Must match to the name given in LL dashboard */
        @Getter @Setter private String name;

        /** Whether this camera is physically connected to the robot. */
        @Getter @Setter private boolean attached = true;

        /**
         * Whether pose measurements from this camera are currently being fused into the estimator.
         */
        @Getter @Setter private boolean isIntegrating;

        /** Physical Config */
        /**
         * Forward offset of the camera from the robot center in meters (positive = toward front).
         */
        @Getter private double forward, right, up; // meters

        /** Orientation of the camera in degrees (roll/pitch/yaw relative to robot frame). */
        @Getter private double roll, pitch, yaw; // degrees

        /**
         * Creates a configuration for the named Limelight camera.
         *
         * @param name the network-table name assigned to the camera in the LL dashboard
         */
        public LimelightConfig(String name) {
            this.name = name;
        }

        /**
         * Sets the position of the robot in meters
         *
         * @param forward (meters) forward from center of robot
         * @param right (meters) right from center of robot
         * @param up (meters) up from center of robot
         * @return the LimelightConfig object
         */
        public LimelightConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /**
         * Sets the rotation of the limelight in degrees
         *
         * @param roll (degrees) roll of limelight || positive is rotated right
         * @param pitch (degrees) pitch of limelight || positive is camera tilted up
         * @param yaw (yaw) yaw of limelight || positive is rotated left
         * @return the LimelightConfig object
         */
        public LimelightConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }
    }

    /* Debug */
    /** Formatter used for printing pose coordinates to SmartDashboard. */
    private final DecimalFormat df = new DecimalFormat();

    /** Active configuration for this Limelight instance. */
    private LimelightConfig config;

    /** Whether pose measurements from this camera are currently being integrated. */
    @Getter @Setter private boolean isIntegrating = false;

    /** Network-table name of this camera (mirrors {@link LimelightConfig#getName()}). */
    @Getter private String cameraName = "default";

    /** Human-readable string describing the current integration status, logged for diagnostics. */
    @Getter @Setter private String logStatus = "";

    /** Human-readable string describing the currently visible tag(s), logged for diagnostics. */
    @Getter @Setter private String tagStatus = "";

    /**
     * Constructs a Limelight wrapper from a fully populated {@link LimelightConfig}.
     *
     * @param config the camera configuration
     */
    public Limelight(LimelightConfig config) {
        this.config = config;
    }

    /**
     * Constructs a Limelight wrapper with a default configuration for the given camera name.
     *
     * @param name the network-table name of the camera
     */
    public Limelight(String name) {
        cameraName = name;
        config = new LimelightConfig(name);
    }

    /**
     * Constructs a Limelight wrapper, explicitly setting whether the camera is attached.
     *
     * @param name the network-table name of the camera
     * @param attached {@code true} if the camera is physically present on the robot
     */
    public Limelight(String name, boolean attached) {
        cameraName = name;
        config = new LimelightConfig(name).setAttached(attached);
    }

    /**
     * Constructs a Limelight wrapper and immediately sets its active pipeline.
     *
     * @param name the network-table name of the camera
     * @param pipeline the pipeline index to activate (see {@link
     *     frc.robot.subsystems.vision.Vision.VisionConfig})
     */
    public Limelight(String name, int pipeline) {
        this(name);
        cameraName = name;
        setLimelightPipeline(pipeline);
    }

    /**
     * Constructs a Limelight wrapper with an explicit configuration and immediately sets its active
     * pipeline.
     *
     * @param name the network-table name of the camera
     * @param pipeline the pipeline index to activate
     * @param config the fully populated {@link LimelightConfig} to use
     */
    public Limelight(String name, int pipeline, LimelightConfig config) {
        this(name);
        cameraName = name;
        this.config = config;
        setLimelightPipeline(pipeline);
    }

    /**
     * Returns the network-table name of this camera.
     *
     * @return the camera name as configured in the LL dashboard
     */
    public String getName() {
        return config.getName();
    }

    /**
     * Returns whether this camera is physically connected to the robot.
     *
     * @return {@code true} if attached
     */
    public boolean isAttached() {
        return config.isAttached();
    }

    /* ::: Basic Information Retrieval ::: */
    /**
     * Get the horizontal offset from crosshair to target
     *
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2:
     *     -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTX(config.getName());
    }

    /**
     * Get the vertical offset from crosshair to target
     *
     * @return Vertical Offset From Crosshair To Target in degrees (LL1: -20.5 degrees to 20.5
     *     degrees / LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTY(config.getName());
    }

    /**
     * Returns whether the LL has any valid targets (April tags or other vision targets)
     *
     * @return Whether the LL has any valid targets (April tags or other vision targets)
     */
    public boolean targetInView() {
        if (!isAttached()) {
            return false;
        }
        return LimelightHelpers.getTV(config.getName());
    }

    /**
     * Checks if the LL sees multiple tags
     *
     * @return whether the LL sees multiple tags or not
     */
    public boolean multipleTagsInView() {
        if (!isAttached()) {
            return false;
        }
        return getTagCountInView() > 1;
    }

    /**
     * Returns the number of AprilTags included in the current MegaTag1 pose estimate.
     *
     * @return number of tags contributing to the current pose estimate, or {@code 0} if not
     *     attached or no estimate is available
     */
    public double getTagCountInView() {
        if (!isAttached()) {
            return 0;
        }
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.getName());
        if (est == null) {
            return 0;
        }
        return est.tagCount;

        // if (retrieveJSON() == null) return 0;

        // return retrieveJSON().targetingResults.targets_Fiducials.length;
    }

    /**
     * Gets the ID of the apriltag most centered in the LL's view (or based on different
     *
     * @return the tag ID of the apriltag most centered in the LL's view (or based on different
     *     criteria set in LL dasbhoard)
     */
    public double getClosestTagID() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getFiducialID(config.getName());
    }

    /**
     * Returns the area of the primary target as a percentage of the camera image (0–100).
     *
     * @return target area percentage, or {@code 0} if not attached
     */
    public double getTargetSize() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getTA(config.getName());
    }

    /* ::: Pose Retrieval ::: */

    /**
     * Get the corresponding LL Pose3d (MEGATAG1) for the alliance in DriverStation.java
     *
     * @return the corresponding LL Pose3d (MEGATAG1) for the alliance in DriverStation.java
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
     * Get the corresponding LL Pose2d (MEGATAG1) for the alliance in DriverStation.java
     *
     * @return the corresponding LL Pose3d (MEGATAG2) for the alliance in DriverStation.java
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

    /**
     * Returns the full MegaTag1 {@link PoseEstimate}, including timestamp, tag count, and raw
     * fiducials. Returns an empty estimate when not attached or when no estimate is available.
     *
     * @return the MegaTag1 pose estimate in the WPILib Blue origin frame
     */
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
     * Returns the full MegaTag2 {@link PoseEstimate} (heading-fused), including timestamp and tag
     * count. Returns an empty estimate when not attached or when no estimate is available.
     *
     * @return the MegaTag2 pose estimate in the WPILib Blue origin frame
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

    /**
     * Returns {@code true} when the pose estimate is considered accurate — i.e., multiple tags are
     * visible and the combined target area exceeds a minimum threshold.
     *
     * @return {@code true} if the pose estimate meets the accuracy criteria
     */
    public boolean hasAccuratePose() {
        if (!isAttached()) {
            return false;
        }
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    /**
     * Get the distance of the 2d vector from the camera to closest apriltag
     *
     * @return the distance of the 2d vector from the camera to closest apriltag
     */
    public double getDistanceToTagFromCamera() {
        if (!isAttached()) {
            return 0;
        }
        double x = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getX();
        double y = LimelightHelpers.getCameraPose3d_TargetSpace(config.name).getZ();
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /**
     * Returns the raw fiducial data for all currently detected AprilTags.
     *
     * @return array of {@link RawFiducial} entries from the MegaTag1 estimate; empty array if not
     *     attached or no estimate is available
     */
    public RawFiducial[] getRawFiducial() {
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
        if (est == null) {
            return new RawFiducial[0];
        }
        return est.rawFiducials;
    }

    /**
     * Returns the timestamp of the MEGATAG1 pose estimation from the Limelight camera.
     *
     * @return The timestamp of the pose estimation in seconds.
     */
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

    /**
     * Returns the timestamp of the MEGATAG2 pose estimation from the Limelight camera.
     *
     * @return The timestamp of the pose estimation in seconds.
     */
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

    /**
     * Returns the latency of the pose estimation from the Limelight camera.
     *
     * @return The latency of the pose estimation in seconds.
     */
    @Deprecated(forRemoval = true)
    public double getPoseLatency() {
        if (!isAttached()) {
            return 0;
        }
        return Units.millisecondsToSeconds(
                LimelightHelpers.getBotPose_wpiBlue(config.getName())[6]);
    }

    /*
     * Custom Helpers
     */

    /**
     * get distance in meters to a target
     *
     * @param targetHeight meters
     * @return distance in meters to a target
     */
    public double getDistanceToTarget(double targetHeight) {
        if (!isAttached()) {
            return 0;
        }
        return (targetHeight - config.up)
                / Math.tan(Units.degreesToRadians(config.roll + getVerticalOffset()));
    }

    /**
     * Marks this camera as actively integrating and updates the log status message.
     *
     * @param message a human-readable description of why integration is valid
     */
    public void sendValidStatus(String message) {
        config.isIntegrating = true;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    /**
     * Marks this camera as not integrating and updates the log status message.
     *
     * @param message a human-readable description of why integration is invalid
     */
    public void sendInvalidStatus(String message) {
        config.isIntegrating = false;
        this.isIntegrating = config.isIntegrating;
        logStatus = message;
    }

    /*
     * Utility Wrappers
     */

    /**
     * @return The latest LL results as a LimelightResults object.
     */
    @SuppressWarnings("unused")
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(config.name);
    }

    /**
     * Sets the LL pipeline to the given index.
     *
     * @param pipelineIndex use pipeline indexes in {@link VisionConfig}
     */
    public void setLimelightPipeline(int pipelineIndex) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setPipelineIndex(config.name, pipelineIndex);
    }

    /** Sets the robot orientation in degrees for the Limelight's internal IMU. */
    public void setRobotOrientation(double degrees) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, 0, 0, 0, 0, 0);
    }

    /**
     * Sets the robot orientation and yaw rate for the Limelight's internal IMU fusion (MegaTag2).
     *
     * @param degrees robot heading in degrees (positive counter-clockwise)
     * @param angularRate current yaw rate in degrees per second
     */
    public void setRobotOrientation(double degrees, double angularRate) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation(config.name, degrees, angularRate, 0, 0, 0, 0);
    }

    /**
     * Sets the IMU mode on the Limelight.
     *
     * @param mode the IMU mode index (refer to the LL documentation for valid values)
     */
    public void setIMUmode(int mode) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetIMUMode(config.name, mode);
    }

    /**
     * Returns the X offset of the primary target in robot space (meters along the robot's
     * left/right axis).
     *
     * @return target X translation in meters, or {@code -99999} if not attached or no target in
     *     view
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
     * Returns the area of the primary target as a percentage of the camera image.
     *
     * @return target area (0–100 %), or {@code -99999} if not attached or no target in view
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

    /**
     * Returns the Z-axis rotation of the primary target in robot space (yaw in radians, converted
     * to degrees).
     *
     * @return target yaw rotation in degrees, or {@code -99999} if not attached or no target in
     *     view
     */
    public double getTagRotationDegrees() {
        if (!isAttached()) {
            return -99999;
        }
        if (!targetInView()) {
            return -99999;
        }

        double rotation =
                LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getRotation().getZ();

        return rotation;
    }

    /**
     * Sets the LED mode of the LL.
     *
     * @param enabled true to enable the LED mode, false to disable it
     */
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

    /** Set LL LED's to blink */
    public void blinkLEDs() {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setLEDMode_ForceBlink(config.getName());
    }

    /** Checks if the camera is connected by looking for an empty botpose array from camera. */
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

    /** Prints the vision, estimated, and odometry pose to SmartDashboard */
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
