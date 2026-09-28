package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.spectrumLib.vision.LimelightHelpers.LimelightResults;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import java.text.DecimalFormat;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

/**
 * Wraps one Limelight camera for AprilTag pose estimation and basic targeting.
 *
 * <p>Every getter is safe on a detached camera ({@link #isAttached()} is false) and returns zero,
 * false, or an empty value.
 *
 * <p>Getters share one NetworkTables sample per robot loop. The first getter to need a topic reads
 * it, and every later getter in that loop reuses the sample, so pose, tag count, fiducials, and
 * timestamp all describe the same camera frame. The owning subsystem calls {@link #invalidate()}
 * once per loop before any getter. Returned {@link PoseEstimate} and {@link RawFiducial} objects
 * belong to that sample and must not be mutated.
 *
 * <p>MegaTag1 solves pose from the tags alone. MegaTag2 fuses in the heading the owning subsystem
 * passes to {@link #setRobotOrientation(double)}.
 */
public class Limelight {

    /**
     * Network-table name and mounting position for one camera.
     *
     * <p>Lombok chains the setters, so config calls read as one expression: {@code
     * config.setName("limelight").setAttached(true)}.
     */
    @Accessors(chain = true)
    public static class LimelightConfig {
        /** Must match the name given in the LL dashboard. */
        @Getter @Setter private String name;

        /** Whether this camera is physically connected to the robot. */
        @Getter @Setter private boolean attached = true;

        /** Mount offsets from the robot center; forward is positive toward the front. */
        @Getter private double forward, right, up; // meters

        /** Camera orientation as a Rotation3d in the robot frame. */
        @Getter private double roll, pitch, yaw; // degrees

        public LimelightConfig(String name) {
            this.name = name;
        }

        /**
         * Sets the camera's mounting offsets.
         *
         * @param forward metres ahead of the robot center
         * @param right metres right of the robot center
         * @param up metres above the robot center
         */
        public LimelightConfig withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            return this;
        }

        /** Sets the camera's orientation in degrees, expressed in the robot frame. */
        public LimelightConfig withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            return this;
        }
    }

    /** Formats the pose values {@link #printDebug()} puts on the dashboard. */
    private final DecimalFormat df = new DecimalFormat();

    private LimelightConfig config;

    /** Whether the pose estimator is currently fusing a measurement from this camera. */
    @Getter @Setter private boolean isIntegrating = false;

    @Getter @Setter private String logStatus = "";

    @Getter @Setter private String tagStatus = "";

    // Per-loop NetworkTables snapshot, filled lazily on first use and cleared by invalidate().
    // Main robot thread only, no synchronization.

    /** Raw botpose_wpiblue sample (value + server timestamp); null until first MT1 read. */
    private TimestampedDoubleArray mt1Sample;

    private PoseEstimate mt1Estimate;

    private Pose3d mt1Pose3d;

    /** MegaTag2 estimate from botpose_orb_wpiblue; null until first MT2 read. */
    private PoseEstimate mt2Estimate;

    private boolean tvCached;
    private boolean tvValue;
    private boolean taCached;
    private double taValue;

    /** Written by the owning subsystem as it accepts or rejects each estimate. */
    @Getter @Setter private boolean integratedThisLoop;

    /**
     * Seconds from the last estimate's capture time to now, or NaN if this camera produced none
     * this loop. A large value means the camera's timestamps are not trustworthy.
     */
    @Getter @Setter private double lastEstimateAgeSeconds = Double.NaN;

    /**
     * Drops the per-loop snapshot so the next getter re-reads the camera.
     *
     * <p>The owning subsystem calls this once per loop before any getter (see {@code
     * Vision.periodic()}). Safe on a detached camera.
     */
    public void invalidate() {
        mt1Sample = null;
        mt1Estimate = null;
        mt1Pose3d = null;
        mt2Estimate = null;
        tvCached = false;
        taCached = false;
        integratedThisLoop = false;
        lastEstimateAgeSeconds = Double.NaN;
    }

    /** Raw MegaTag1 sample for this loop. Caller must have checked {@link #isAttached()}. */
    private TimestampedDoubleArray mt1Sample() {
        if (mt1Sample == null) {
            mt1Sample =
                    LimelightHelpers.getLimelightDoubleArrayEntry(
                                    config.getName(), "botpose_wpiblue")
                            .getAtomic();
        }
        return mt1Sample;
    }

    /** MegaTag1 estimate for this loop. Caller must have checked {@link #isAttached()}. */
    private PoseEstimate mt1Estimate() {
        if (mt1Estimate == null) {
            TimestampedDoubleArray sample = mt1Sample();
            mt1Estimate = LimelightHelpers.parsePoseEstimate(sample.value, sample.timestamp, false);
        }
        return mt1Estimate;
    }

    /** MegaTag1 Pose3d for this loop. Caller must have checked {@link #isAttached()}. */
    private Pose3d mt1Pose3d() {
        if (mt1Pose3d == null) {
            mt1Pose3d = LimelightHelpers.toPose3D(mt1Sample().value);
        }
        return mt1Pose3d;
    }

    /** MegaTag2 estimate for this loop. Caller must have checked {@link #isAttached()}. */
    private PoseEstimate mt2Estimate() {
        if (mt2Estimate == null) {
            mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.getName());
        }
        return mt2Estimate;
    }

    public Limelight(LimelightConfig config) {
        this.config = config;
    }

    public Limelight(String name) {
        config = new LimelightConfig(name);
    }

    public Limelight(String name, boolean attached) {
        config = new LimelightConfig(name).setAttached(attached);
    }

    /**
     * @param pipeline index to activate; the indexes come from the robot's vision configuration
     */
    public Limelight(String name, int pipeline) {
        this(name);
        setLimelightPipeline(pipeline);
    }

    /**
     * @param pipeline index to activate; the indexes come from the robot's vision configuration
     */
    public Limelight(String name, int pipeline, LimelightConfig config) {
        this(config);
        setLimelightPipeline(pipeline);
    }

    public String getName() {
        return config.getName();
    }

    /** Same as {@link #getName()}. */
    public String getCameraName() {
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
        if (!tvCached) {
            tvValue = LimelightHelpers.getTV(config.getName());
            tvCached = true;
        }
        return tvValue;
    }

    public boolean multipleTagsInView() {
        return getTagCountInView() > 1;
    }

    public double getTagCountInView() {
        if (!isAttached()) {
            return 0;
        }
        return mt1Estimate().tagCount;
    }

    public double getClosestTagID() {
        if (!isAttached()) {
            return 0;
        }
        return LimelightHelpers.getFiducialID(config.getName());
    }

    /** Area of the primary target as a percentage of the camera image (0-100). */
    public double getTargetSize() {
        if (!isAttached()) {
            return 0;
        }
        if (!taCached) {
            taValue = LimelightHelpers.getTA(config.getName());
            taCached = true;
        }
        return taValue;
    }

    public Pose3d getMegaTag1_Pose3d() {
        if (!isAttached()) {
            return Pose3d.kZero;
        }
        return mt1Pose3d();
    }

    public Pose2d getMegaTag2_Pose2d() {
        if (!isAttached()) {
            return Pose2d.kZero;
        }
        return mt2Estimate().pose;
    }

    /**
     * Full MegaTag1 estimate, including timestamp, tag count, and raw fiducials, in the WPILib Blue
     * origin frame.
     */
    public PoseEstimate getMegaTag1_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }
        return mt1Estimate();
    }

    /**
     * Full heading-fused MegaTag2 estimate, including timestamp and tag count, in the WPILib Blue
     * origin frame.
     */
    public PoseEstimate getMegaTag2_PoseEstimate() {
        if (!isAttached()) {
            return new PoseEstimate();
        }
        return mt2Estimate();
    }

    /** Needs more than one tag and a target area above 0.1 percent of the frame. */
    public boolean hasAccuratePose() {
        return multipleTagsInView() && getTargetSize() > 0.1;
    }

    /** Distance to the closest tag, from the X and Z axes of the camera-in-tag-space pose. */
    public double getDistanceToTagFromCamera() {
        if (!isAttached()) {
            return 0;
        }
        Pose3d cameraInTargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(config.name);
        return Math.hypot(cameraInTargetSpace.getX(), cameraInTargetSpace.getZ());
    }

    /** Raw fiducials from the MegaTag1 estimate, or an empty array when there is no estimate. */
    public RawFiducial[] getRawFiducial() {
        if (!isAttached()) {
            return new RawFiducial[0];
        }
        // Shares the per-loop MT1 sample, so this costs no extra NetworkTables read.
        RawFiducial[] fiducials = mt1Estimate().rawFiducials;
        return fiducials == null ? new RawFiducial[0] : fiducials;
    }

    /** Timestamp of the MegaTag1 pose estimate, in seconds. */
    public double getMegaTag1PoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        return mt1Estimate().timestampSeconds;
    }

    /** Timestamp of the MegaTag2 pose estimate, in seconds. */
    public double getMegaTag2PoseTimestamp() {
        if (!isAttached()) {
            return 0;
        }
        return mt2Estimate().timestampSeconds;
    }

    /**
     * Distance to a target at a known height, from the camera's height above the floor and the
     * camera's pitch plus the reported vertical offset.
     *
     * @param targetHeight height of the target above the floor, in metres
     */
    public double getDistanceToTarget(double targetHeight) {
        if (!isAttached()) {
            return 0;
        }
        return (targetHeight - config.up)
                / Math.tan(Units.degreesToRadians(config.pitch + getVerticalOffset()));
    }

    /**
     * Marks this camera as integrating and records why.
     *
     * @param message status text the caller wants on the dashboard
     */
    public void sendValidStatus(String message) {
        isIntegrating = true;
        logStatus = message;
    }

    /**
     * Marks this camera as not integrating and records why.
     *
     * @param message status text the caller wants on the dashboard
     */
    public void sendInvalidStatus(String message) {
        isIntegrating = false;
        logStatus = message;
    }

    @SuppressWarnings("unused")
    private LimelightResults retrieveJSON() {
        return LimelightHelpers.getLatestResults(config.name);
    }

    /** Activates a pipeline by index; the indexes come from the robot's vision configuration. */
    public void setLimelightPipeline(int pipelineIndex) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setPipelineIndex(config.name, pipelineIndex);
    }

    /**
     * Sets the robot heading the camera's internal IMU fuses, in degrees.
     *
     * <p>Does not flush NetworkTables. The owning subsystem flushes once after all per-loop writes.
     */
    public void setRobotOrientation(double degrees) {
        setRobotOrientation(degrees, 0);
    }

    public void updateCameraPose(Pose3d pose) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setCameraPose_RobotSpace(
                config.name,
                pose.getX(),
                -pose.getY(),
                pose.getZ(),
                Units.radiansToDegrees(pose.getRotation().getX()),
                Units.radiansToDegrees(pose.getRotation().getY()),
                Units.radiansToDegrees(pose.getRotation().getZ()));
    }

    /**
     * Writes the configured mount offsets ({@link LimelightConfig#withTranslation} and {@link
     * LimelightConfig#withRotation}) to the camera, so the code is the source of truth instead of
     * the values typed into the web UI. The six values cross in the order and sign the camera
     * wants, so a config value can be read straight against the matching web UI box.
     *
     * <p>A camera keeps its own copy in flash and falls back to it on boot, so this has to run
     * again periodically: one that reboots mid-match reverts to whatever was last typed into it.
     * See the resend loop in {@code Vision.periodic()}. Does not flush NetworkTables.
     */
    public void pushConfiguredCameraPose() {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.setCameraPose_RobotSpace(
                config.name,
                config.forward,
                config.right,
                config.up,
                config.roll,
                config.pitch,
                config.yaw);
    }

    /**
     * Sets the heading and yaw rate the camera's internal IMU fuses, for MegaTag2.
     *
     * <p>Does not flush NetworkTables. The owning subsystem flushes once after all per-loop writes.
     *
     * @param degrees robot heading in degrees, positive counter-clockwise
     * @param angularRate current yaw rate in degrees per second
     */
    public void setRobotOrientation(double degrees, double angularRate) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetRobotOrientation_NoFlush(config.name, degrees, angularRate, 0, 0, 0, 0);
    }

    /**
     * @param mode IMU mode index; valid values are in the Limelight documentation
     */
    public void setIMUmode(int mode) {
        if (!isAttached()) {
            return;
        }
        LimelightHelpers.SetIMUMode(config.name, mode);
    }

    /**
     * X offset of the primary target in robot space, in metres along the robot's left/right axis.
     * Returns -99999 when no target is in view.
     */
    public double getTagTx() {
        if (!targetInView()) {
            return -99999;
        }
        return LimelightHelpers.getTargetPose3d_RobotSpace(config.getName()).getX();
    }

    /**
     * Area of the primary target as a percentage of the camera image. Returns -99999 when no target
     * is in view.
     */
    public double getTagTA() {
        if (!targetInView()) {
            return -99999;
        }
        return getTargetSize();
    }

    /**
     * Z-axis rotation of the primary target in robot space, in degrees. Returns -99999 when no
     * target is in view.
     */
    public double getTagRotationDegrees() {
        if (!targetInView()) {
            return -99999;
        }

        double rotationRadians =
                LimelightHelpers.getTargetPose3d_RobotSpace(config.getName()).getRotation().getZ();

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

    /** True when the raw MT1 sample is long enough to hold a pose. */
    public boolean isCameraConnected() {
        if (!isAttached()) {
            return false;
        }
        try {
            return mt1Sample().value.length >= 6;
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
