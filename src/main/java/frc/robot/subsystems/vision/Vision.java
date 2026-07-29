package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.rebuilt.FieldHelpers;
import frc.robot.Robot;
import frc.robot.auton.Auton;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import frc.spectrumLib.vision.VisionLogger;
import java.util.Arrays;
import java.util.IdentityHashMap;
import java.util.function.DoubleSupplier;
import lombok.Getter;

/**
 * Vision subsystem that manages the chassis-mounted Limelights (back, left, right) plus the
 * turret-mounted Limelight, and fuses their pose estimates into the swerve odometry via WPILib's
 * {@code SwerveDrivePoseEstimator}.
 *
 * <p>Each robot loop iteration the subsystem:
 *
 * <ol>
 *   <li>Pushes the robot's current heading to all Limelights so MegaTag2 IMU fusion remains
 *       accurate (the turret camera receives robot heading + turret angle, since its own frame
 *       rotates with the turret).
 *   <li>Selects the chassis Limelight with the best view ({@link #getBestLimelight()}).
 *   <li>Runs the MT1 (and, while disabled, MT2) rejection pipeline and adds accepted estimates to
 *       the pose estimator with appropriate std-dev vectors.
 *   <li>Runs the turret MT1 pipeline, which additionally de-rotates the reported pose by the
 *       current turret angle ({@link #turretCameraPoseToRobotPose(Pose2d)}).
 *   <li>Logs per-camera status and pose data via {@link VisionLogger}.
 * </ol>
 *
 * <p>Individual cameras can be enabled/disabled with {@link LimelightConfig#setAttached(boolean)};
 * a detached camera is never read from and contributes no estimates.
 */
public class Vision implements Subsystem {

    // =========================================================================
    // Configuration
    // =========================================================================

    /**
     * Static configuration for the Vision subsystem, including Limelight identifiers,
     * camera-to-robot transforms, pipeline indices, and pose estimation covariance parameters.
     */
    public static class VisionConfig {

        @Getter final String name = "Vision";

        // -- Back Limelight ---------------------------------------------------

        /** NetworkTables hostname for the rear-facing Limelight. */
        @Getter final String backLL = "limelight-back";

        /**
         * Robot-relative pose of the rear Limelight. Translation in metres (x, y, z); rotation in
         * degrees (roll, pitch, yaw).
         *
         * <p>Detached — this camera is not mounted on the current robot.
         */
        @Getter
        final LimelightConfig backConfig =
                new LimelightConfig(backLL)
                        .withTranslation(-0.3084987734, 0.2134100126, 0.6502249886)
                        .withRotation(0, 0, 180)
                        .setAttached(false);

        // -- Left Limelight ---------------------------------------------------

        /** NetworkTables hostname for the left-facing Limelight. */
        @Getter final String leftLL = "limelight-left";

        /**
         * Robot-relative pose of the left Limelight. Translation in metres (x, y, z); rotation in
         * degrees (roll, pitch, yaw).
         *
         * <p>Detached — this camera is not mounted on the current robot.
         */
        @Getter
        final LimelightConfig leftConfig =
                new LimelightConfig(leftLL)
                        .withTranslation(0, 0.215, 0.188)
                        .withRotation(0, 0, 90)
                        .setAttached(false);

        // -- Right Limelight --------------------------------------------------

        /** NetworkTables hostname for the right-facing Limelight. */
        @Getter final String rightLL = "limelight-right";

        /**
         * Robot-relative pose of the right Limelight. Translation in metres (x, y, z); rotation in
         * degrees (roll, pitch, yaw).
         *
         * <p>Detached — this camera is not mounted on the current robot.
         */
        @Getter
        final LimelightConfig rightConfig =
                new LimelightConfig(rightLL)
                        .withTranslation(-0.04445, 0.3027487722, 0.7137249886)
                        .withRotation(0, 0, -90)
                        .setAttached(false);

        // -- Turret Limelight -------------------------------------------------

        /** NetworkTables hostname for the turret-mounted Limelight. */
        @Getter final String turretLL = "limelight-turret";

        /**
         * Robot-relative pose of the turret Limelight <b>with the turret at zero</b>. Translation
         * in metres (x, y, z); rotation in degrees (roll, pitch, yaw).
         *
         * <p>IMPORTANT — Limelight GUI setup for this camera:
         *
         * <ul>
         *   <li>Enter the <b>rotation</b> offsets (roll/pitch/yaw) as usual, so the reported pose
         *       is level and its yaw is the turret frame's yaw.
         *   <li><b>Zero the translation</b> offsets, so "botpose" is the camera's own field
         *       position instead of a robot position computed from a turret-at-zero transform.
         * </ul>
         *
         * <p>The chassis pose is then recovered in code by {@link
         * #turretCameraPoseToRobotPose(Pose2d)} using the live turret angle. The translation values
         * here are used only by {@link Limelight#getDistanceToTarget(double)}.
         */
        @Getter
        final LimelightConfig turretConfig =
                new LimelightConfig(turretLL)
                        .withTranslation(
                                Units.inchesToMeters(5.360),
                                Units.inchesToMeters(0.0),
                                Units.inchesToMeters(21.212))
                        .withRotation(0, 30, 0);

        // -- Turret geometry --------------------------------------------------

        /** Robot-centre to turret pivot offset (metres). */
        @Getter final Translation2d robotToTurretCenter = Translation2d.kZero;

        /** Turret pivot to camera offset (metres), measured with the turret at zero. */
        @Getter
        final Translation2d turretCenterToCamera =
                new Translation2d(Units.inchesToMeters(5.360), 0);

        // -- Pipeline indices -------------------------------------------------

        @Getter final int backTagPipeline = 0;
        @Getter final int leftTagPipeline = 0;
        @Getter final int rightTagPipeline = 0;
        @Getter final int turretTagPipeline = 0;

        // -- Pose estimation covariance ---------------------------------------

        /**
         * Default translational standard deviation for vision pose measurements (metres). Lower
         * values trust vision more; higher values trust odometry more.
         */
        @Getter double visionStdDevX = 0.5;

        /**
         * @see #visionStdDevX
         */
        @Getter double visionStdDevY = 0.5;

        /**
         * Default rotational standard deviation for vision pose measurements (radians). Only used
         * for MT1; MT2 heading is always discarded ({@link #kLargeVariance}).
         */
        @Getter double visionStdDevTheta = 0.2;

        /**
         * Variance used to effectively ignore a measurement dimension (e.g., heading from MT2 or
         * single-tag MT1).
         */
        @Getter final double kLargeVariance = 999999.0;

        /** Measurements older than this many seconds are not fused. */
        @Getter final double kMaxTimeDeltaSeconds = 0.1;

        /** Pre-built std-dev matrix using the default X/Y/theta values. */
        @Getter
        final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta);
    }

    // =========================================================================
    // Fields
    // =========================================================================

    /** Rear-facing Limelight instance. */
    @Getter public final Limelight backLL;

    /** Left-facing Limelight instance. */
    @Getter public final Limelight leftLL;

    /** Right-facing Limelight instance. */
    @Getter public final Limelight rightLL;

    /** Turret-mounted Limelight instance; its frame rotates with the turret. */
    @Getter public final Limelight turretLL;

    /** Chassis-mounted (non-turret) Limelights, whose reported pose is already the robot pose. */
    public final Limelight[] swerveLimelights;

    /** All Limelights in one array for bulk operations. */
    public final Limelight[] allLimelights;

    /* Vision loggers — one per Limelight */
    private final VisionLogger backLogger;
    private final VisionLogger leftLogger;
    private final VisionLogger rightLogger;
    private final VisionLogger turretLogger;

    /** All loggers in one array for bulk telemetry loops. */
    private final VisionLogger[] allLoggers;

    /** Live turret angle in degrees, positive counter-clockwise, zero pointing robot-forward. */
    @Getter
    private final DoubleSupplier turretRotationSupplier =
            () -> Robot.getTurret().getPositionDegrees();

    /** AprilTag IDs that are valid scoring targets for the blue alliance. */
    private final int[] blueTags = {18, 19, 20, 21, 24, 25, 26, 27};

    /** AprilTag IDs that are valid scoring targets for the red alliance. */
    private final int[] redTags = {2, 3, 4, 5, 8, 9, 10, 11, 12};

    /** Field layout loaded once at construction and shared across the robot. */
    @Getter private static AprilTagFieldLayout tagLayout;

    private final VisionConfig config;

    /**
     * Tracks the last IMU mode written to each Limelight so we only issue a NetworkTables write
     * when the desired mode actually changes.
     */
    private final IdentityHashMap<Limelight, Integer> lastImuModeByLL = new IdentityHashMap<>();

    // =========================================================================
    // Construction
    // =========================================================================

    /**
     * Creates the Vision subsystem.
     *
     * <p>Instantiates all Limelights, registers their loggers, applies initial camera settings
     * (LEDs off; IMU mode 1 for the chassis cameras and mode 2 for the turret camera, whose frame
     * rotates relative to the robot), loads the AprilTag field layout, and registers this subsystem
     * with the WPILib scheduler.
     *
     * @param config the static configuration object
     */
    public Vision(VisionConfig config) {
        this.config = config;

        backLL = new Limelight(config.backLL, config.backTagPipeline, config.backConfig);
        leftLL = new Limelight(config.leftLL, config.leftTagPipeline, config.leftConfig);
        rightLL = new Limelight(config.rightLL, config.rightTagPipeline, config.rightConfig);
        turretLL = new Limelight(config.turretLL, config.turretTagPipeline, config.turretConfig);

        swerveLimelights = new Limelight[] {backLL, leftLL, rightLL};
        allLimelights = new Limelight[] {backLL, leftLL, rightLL, turretLL};

        backLogger = new VisionLogger("BackLL", backLL);
        leftLogger = new VisionLogger("LeftLL", leftLL);
        rightLogger = new VisionLogger("RightLL", rightLL);
        turretLogger = new VisionLogger("TurretLL", turretLL);
        allLoggers = new VisionLogger[] {backLogger, leftLogger, rightLogger, turretLogger};

        for (Limelight limelight : swerveLimelights) {
            limelight.setLEDMode(false);
            setImuModeIfChanged(limelight, 1);
        }

        // The turret camera's yaw is robot yaw + turret angle, so it runs on its internal IMU
        // rather than the robot heading we push to the chassis cameras.
        turretLL.setLEDMode(false);
        setImuModeIfChanged(turretLL, 2);

        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        this.register();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    /**
     * @return the subsystem name defined in {@link VisionConfig}.
     */
    @Override
    public String getName() {
        return config.getName();
    }

    // =========================================================================
    // Subsystem Periodic
    // =========================================================================

    /**
     * Called every robot loop iteration by the WPILib scheduler.
     *
     * <ol>
     *   <li>Pushes current heading to all Limelights (required for MegaTag2).
     *   <li>Runs pose-estimation updates appropriate to the current robot mode.
     *   <li>Logs telemetry for all cameras.
     * </ol>
     */
    @Override
    public void periodic() {
        setLimeLightOrientation();
        disabledLimelightUpdates();
        enabledLimelightUpdates();
        logTelemetry();
    }

    /**
     * Logs connection status, integration status, tag status, MT1 poses, tag count, and target size
     * for each Limelight via their {@link VisionLogger}. MT2 poses are only logged while disabled
     * (they are unreliable when moving). Also updates the {@code Field2d} widget with the MT1 pose
     * from each camera.
     */
    public void logTelemetry() {
        for (VisionLogger logger : allLoggers) {
            logger.getCameraConnection();
            logger.getIntegratingStatus();
            logger.getLogStatus();
            logger.getTagStatus();
            logger.getPose();
            logger.getTagCount();
            logger.getTargetSize();
        }

        // MT2 poses are only reliable when the robot is stationary
        if (Util.disabled.getAsBoolean()) {
            backLogger.getMegaPose();
            leftLogger.getMegaPose();
            rightLogger.getMegaPose();
        }

        // Update Field2d visualization (null-safe; returns Pose2d.kZero when no data)
        Robot.getField2d().getObject(backLL.getCameraName()).setPose(getBackMegaTag1Pose());
        Robot.getField2d().getObject(leftLL.getCameraName()).setPose(getLeftMegaTag1Pose());
        Robot.getField2d().getObject(rightLL.getCameraName()).setPose(getRightMegaTag1Pose());
        // The turret camera reports its own field pose; show the turret-corrected robot pose
        Robot.getField2d().getObject(turretLL.getCameraName()).setPose(getTurretRobotPose());

        if (turretLL.isAttached()) {
            Telemetry.log(
                    "Vision/TurretLL/TurretAngle", turretRotationSupplier.getAsDouble(), "deg");
        }
    }

    // =========================================================================
    // Pose Estimation — Private Pipeline
    // =========================================================================

    /**
     * Pushes the robot's current heading (from swerve odometry) to all Limelights each loop so
     * MegaTag2 IMU fusion uses an up-to-date yaw.
     *
     * <p>The turret camera's own frame rotates with the turret, so it receives robot heading plus
     * turret angle instead of the raw robot heading.
     */
    private void setLimeLightOrientation() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        for (Limelight limelight : swerveLimelights) {
            limelight.setRobotOrientation(yaw);
        }
        turretLL.setRobotOrientation(yaw + turretRotationSupplier.getAsDouble());
    }

    /**
     * While the robot is disabled, integrates both MegaTag1 and MegaTag2 estimates from the best
     * chassis Limelight, plus the turret MegaTag1 estimate, to pre-seed the pose estimator before
     * enable.
     */
    private void disabledLimelightUpdates() {
        if (Util.disabled.getAsBoolean()) {
            Limelight bestLimelight = getBestLimelight();
            integrateSingleEstimate(getMT1VisionEstimate(bestLimelight, true));
            integrateSingleEstimate(getMT2VisionEstimate(bestLimelight));

            if (turretEstimatesAvailable()) {
                integrateSingleEstimate(getMT1TurretEstimate(turretLL, true));
            }
        }
    }

    /**
     * While the robot is enabled (teleop, auto-update state, or auto-launching), integrates the
     * MegaTag1 estimate from the best chassis Limelight and from the turret Limelight.
     */
    private void enabledLimelightUpdates() {
        if (Util.teleop.getAsBoolean()
                || Auton.autonPoseUpdate.getAsBoolean()
                || Auton.autonLaunching.getAsBoolean()) {
            Limelight bestLimelight = getBestLimelight();
            integrateSingleEstimate(getMT1VisionEstimate(bestLimelight, false));

            if (turretEstimatesAvailable()) {
                integrateSingleEstimate(getMT1TurretEstimate(turretLL, false));
            }
        }
    }

    /**
     * Returns {@code true} when the turret camera can produce a usable estimate — both the camera
     * and the turret mechanism must be attached, since the correction math needs a live turret
     * angle.
     */
    private boolean turretEstimatesAvailable() {
        return turretLL.isAttached() && Robot.getTurret().isAttached();
    }

    /**
     * Builds a MegaTag1 (multi-tag, heading-fused) pose estimate for the given Limelight and
     * decides whether it is trustworthy enough to add to the pose estimator.
     *
     * <p>Rejection criteria (any one triggers rejection):
     *
     * <ul>
     *   <li>No targets in view.
     *   <li>Any tag ambiguity &gt; 0.9 (pose flip risk).
     *   <li>Pose outside the field boundary.
     *   <li>Robot spin rate &ge; 1.6 rad/s.
     *   <li>Target too small (&le; 0.025 %).
     *   <li>Roll or pitch &gt; 5° (camera physically disturbed).
     * </ul>
     *
     * <p>Accepted estimates are assigned std-dev vectors based on how many tags are visible and how
     * large the target appears. {@code forceIntegrateXY} overrides the std-devs to near-zero, used
     * during disabled pre-seeding.
     *
     * @param ll the Limelight to query
     * @param forceIntegrateXY if {@code true}, bypass std-dev selection and use very tight
     *     covariance (disabled pre-seeding)
     * @return a {@link VisionFieldPoseEstimate} ready to pass to the pose estimator, or {@code
     *     null} if rejected
     */
    private VisionFieldPoseEstimate getMT1VisionEstimate(Limelight ll, boolean forceIntegrateXY) {
        return getMT1Estimate(ll, forceIntegrateXY, false);
    }

    /**
     * Builds a MegaTag1 pose estimate for the turret-mounted Limelight.
     *
     * <p>Runs the same rejection/covariance pipeline as {@link #getMT1VisionEstimate(Limelight,
     * boolean)}, but first converts the camera's reported field pose into a robot pose with {@link
     * #turretCameraPoseToRobotPose(Pose2d)}. Because the resulting heading is only as good as the
     * turret encoder, the heading std-dev is widened more aggressively than for a chassis camera.
     *
     * @param ll the turret Limelight to query
     * @param forceIntegrateXY if {@code true}, bypass std-dev selection and use very tight
     *     covariance (disabled pre-seeding)
     * @return a {@link VisionFieldPoseEstimate} ready to pass to the pose estimator, or {@code
     *     null} if rejected
     */
    private VisionFieldPoseEstimate getMT1TurretEstimate(Limelight ll, boolean forceIntegrateXY) {
        return getMT1Estimate(ll, forceIntegrateXY, true);
    }

    /**
     * Shared MegaTag1 pipeline for both chassis-mounted and turret-mounted cameras.
     *
     * @param ll the Limelight to query
     * @param forceIntegrateXY if {@code true}, bypass std-dev selection and use very tight
     *     covariance
     * @param turretMounted if {@code true}, de-rotate the reported pose by the live turret angle
     *     before validating and integrating it
     * @return a {@link VisionFieldPoseEstimate}, or {@code null} if rejected
     */
    private VisionFieldPoseEstimate getMT1Estimate(
            Limelight ll, boolean forceIntegrateXY, boolean turretMounted) {
        if (!ll.targetInView()) {
            ll.setTagStatus("No Targets in View");
            ll.sendInvalidStatus("No Targets in View Rejection");
            return null;
        }

        boolean multiTags = ll.multipleTagsInView();
        double targetSize = ll.getTargetSize();
        Pose3d megaTag1Pose3d = ll.getMegaTag1_Pose3d();
        Pose2d reportedPose2d = megaTag1Pose3d.toPose2d();

        // A turret camera reports its own field pose, not the robot's — undo the turret geometry
        // before any pose-based rejection or covariance decision is made.
        Pose2d megaTag1Pose2d =
                turretMounted ? turretCameraPoseToRobotPose(reportedPose2d) : reportedPose2d;

        RawFiducial[] tags = ll.getRawFiducial();
        double highestAmbiguity = -1;
        ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        double robotLinearSpeed =
                Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

        // Distance from current odometry pose to the MT1 estimate
        double mt1PoseDifference =
                Robot.getSwerve()
                        .getRobotPose()
                        .getTranslation()
                        .getDistance(megaTag1Pose2d.getTranslation());

        // Ambiguity scan — reject immediately if any tag exceeds 0.9
        ll.setTagStatus("");
        if (tags != null) {
            for (RawFiducial tag : tags) {
                if (highestAmbiguity < 0 || tag.ambiguity > highestAmbiguity) {
                    highestAmbiguity = tag.ambiguity;
                }
                if (tag.ambiguity > 0.9) {
                    ll.sendInvalidStatus("High Ambiguity Rejection");
                    return null;
                }
            }
        }

        // Field boundary, spin rate, and target-size rejections
        if (rejectionCheck(ll, megaTag1Pose2d, targetSize)) {
            return null;
        }

        // Reject if the camera pose shows significant roll or pitch (> 5°)
        if (Math.abs(megaTag1Pose3d.getRotation().getX()) > Math.toRadians(5)
                || Math.abs(megaTag1Pose3d.getRotation().getY()) > Math.toRadians(5)) {
            ll.sendInvalidStatus("Roll/Pitch Rejection");
            return null;
        }

        // Select std-devs based on confidence tier
        double xyStds;
        double degStds;

        if (robotLinearSpeed <= 0.2 && targetSize > 4) {
            ll.sendValidStatus("Stationary close integration");
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 2) {
            ll.sendValidStatus("Strong Multi integration");
            xyStds = 0.1;
            degStds = 0.1;
        } else if (multiTags && targetSize > 0.2) {
            ll.sendValidStatus("Multi integration");
            xyStds = 0.25;
            degStds = 8;
        } else if (targetSize > 2 && mt1PoseDifference < 0.5) {
            ll.sendValidStatus("Close integration");
            xyStds = 0.5;
            degStds = config.getKLargeVariance();
        } else if (targetSize > 1 && mt1PoseDifference < 0.25) {
            ll.sendValidStatus("Proximity integration");
            xyStds = 1.0;
            degStds = config.getKLargeVariance();
        } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
            ll.sendValidStatus("Stable integration");
            xyStds = 1.5;
            degStds = config.getKLargeVariance();
        } else {
            ll.sendInvalidStatus("Integration Criteria not Met");
            return null;
        }

        // Widen heading std-dev when ambiguity is moderate. The turret heading also carries turret
        // encoder error, so it is trusted less than a chassis camera's heading.
        if (highestAmbiguity > 0.5) {
            degStds = Math.max(degStds, turretMounted ? 50 : 15);
        }

        // Discard heading during fast rotation (MegaTag1 heading unreliable while spinning)
        if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 0.5) {
            degStds = Math.max(degStds, turretMounted ? 75 : 50);
        }

        // Override covariance for disabled pre-seeding
        if (forceIntegrateXY) {
            xyStds = 0.01;
            degStds = 0.01;
        }

        Pose2d integratedPose =
                new Pose2d(megaTag1Pose2d.getTranslation(), megaTag1Pose2d.getRotation());
        double timestamp = Utils.fpgaToCurrentTime(ll.getMegaTag1PoseTimestamp());
        // The pose estimator expects the heading std-dev in radians; degStds is in degrees.
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
        int numTags = tags == null ? 1 : tags.length;

        return new VisionFieldPoseEstimate(integratedPose, timestamp, stdDevs, numTags);
    }

    /**
     * Builds a MegaTag2 (IMU-fused, translation-only) pose estimate for the given Limelight.
     *
     * <p>MegaTag2 heading is always discarded ({@link VisionConfig#kLargeVariance}) because it
     * relies on the IMU rather than tag geometry. This method is only called while the robot is
     * disabled.
     *
     * <p>Rejection criteria:
     *
     * <ul>
     *   <li>No targets in view.
     *   <li>Pose outside field boundary.
     *   <li>Robot spin rate &ge; 1.6 rad/s.
     *   <li>Target too small (&le; 0.025 %).
     * </ul>
     *
     * @param ll the Limelight to query
     * @return a {@link VisionFieldPoseEstimate} ready to pass to the pose estimator, or {@code
     *     null} if rejected
     */
    private VisionFieldPoseEstimate getMT2VisionEstimate(Limelight ll) {
        if (!ll.targetInView()) {
            ll.setTagStatus("No Targets in View");
            ll.sendInvalidStatus("No Targets in View Rejection");
            return null;
        }

        boolean multiTags = ll.multipleTagsInView();
        double targetSize = ll.getTargetSize();
        Pose2d megaTag2Pose2d = ll.getMegaTag2_Pose2d();
        ChassisSpeeds robotSpeed = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        double robotLinearSpeed =
                Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

        double mt2PoseDifference =
                Robot.getSwerve()
                        .getRobotPose()
                        .getTranslation()
                        .getDistance(megaTag2Pose2d.getTranslation());

        if (rejectionCheck(ll, megaTag2Pose2d, targetSize)) {
            return null;
        }

        // Select translational std-devs (heading is always discarded for MT2)
        double xyStds;

        if (robotLinearSpeed <= 0.2 && targetSize > 4) {
            ll.sendValidStatus("Stationary close integration");
            xyStds = 0.1;
        } else if (multiTags && targetSize > 2) {
            ll.sendValidStatus("Strong Multi integration");
            xyStds = 0.1;
        } else if (multiTags && targetSize > 0.2) {
            ll.sendValidStatus("Multi integration");
            xyStds = 0.25;
        } else if (targetSize > 2 && (mt2PoseDifference < 0.5 || DriverStation.isDisabled())) {
            ll.sendValidStatus("Close integration");
            xyStds = 0.5;
        } else if (targetSize > 1 && (mt2PoseDifference < 0.25 || DriverStation.isDisabled())) {
            ll.sendValidStatus("Proximity integration");
            xyStds = 1.0;
        } else if (targetSize >= 0.03) {
            ll.sendValidStatus("Stable integration");
            xyStds = 1.5;
        } else {
            ll.sendInvalidStatus("Integration Criteria not Met");
            return null;
        }

        double degStds = config.getKLargeVariance();

        Pose2d integratedPose =
                new Pose2d(megaTag2Pose2d.getTranslation(), megaTag2Pose2d.getRotation());

        return new VisionFieldPoseEstimate(
                integratedPose,
                Utils.fpgaToCurrentTime(ll.getMegaTag2PoseTimestamp()),
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)),
                (int) ll.getTagCountInView());
    }

    /**
     * Adds a vision measurement to the swerve pose estimator if the estimate is non-null.
     *
     * @param estimate the estimate to integrate, or {@code null} to skip
     */
    private void integrateSingleEstimate(VisionFieldPoseEstimate estimate) {
        if (estimate != null) {
            Robot.getSwerve()
                    .addVisionMeasurement(
                            estimate.getVisionRobotPoseMeters(),
                            estimate.getTimestampSeconds(),
                            estimate.getVisionMeasurementStdDevs());
        }
    }

    /**
     * Common rejection gate shared by both MT1 and MT2 pipelines.
     *
     * <p>Rejects on:
     *
     * <ul>
     *   <li>Pose outside field boundary.
     *   <li>Robot spin rate &ge; 1.6 rad/s.
     *   <li>Target size &le; 0.025 % (too far / too small to trust).
     * </ul>
     *
     * @param ll the Limelight (used for status reporting)
     * @param pose the candidate pose to validate
     * @param targetSize the Limelight target-size percentage
     * @return {@code true} if the measurement should be rejected
     */
    private boolean rejectionCheck(Limelight ll, Pose2d pose, double targetSize) {
        if (FieldHelpers.poseOutOfField(pose)) {
            ll.sendInvalidStatus("Out of Field Rejection");
            return true;
        }

        if (Math.abs(Robot.getSwerve().getCurrentRobotChassisSpeeds().omegaRadiansPerSecond)
                >= 1.6) {
            ll.sendInvalidStatus("Rotation Speed Rejection");
            return true;
        }

        if (targetSize <= 0.025) {
            ll.sendInvalidStatus("Target Size Rejection");
            return true;
        }

        return false;
    }

    /**
     * Updates the IMU mode on a Limelight only when the desired mode differs from the last written
     * value, avoiding redundant NetworkTables writes.
     *
     * @param limelight the Limelight to configure
     * @param desiredMode the IMU mode to apply (0 = external, 1 = internal, etc.)
     */
    private void setImuModeIfChanged(Limelight limelight, int desiredMode) {
        Integer lastMode = lastImuModeByLL.get(limelight);
        if (lastMode == null || lastMode.intValue() != desiredMode) {
            limelight.setIMUmode(desiredMode);
            lastImuModeByLL.put(limelight, desiredMode);
        }
    }

    // =========================================================================
    // Turret Camera Geometry
    // =========================================================================

    /**
     * Converts a field pose reported by the turret-mounted camera into a field pose of the robot
     * chassis.
     *
     * <p>The turret camera's mounting offsets are zeroed in the Limelight GUI, so its "botpose" is
     * really the camera's own field pose, expressed in a frame that rotates with the turret. Two
     * corrections are needed:
     *
     * <ol>
     *   <li><b>Rotation:</b> the reported yaw is {@code robotYaw + turretAngle}, so the robot
     *       heading is the reported yaw minus the turret angle.
     *   <li><b>Translation:</b> the camera sits at {@code robotToTurretCenter +
     *       turretCenterToCamera} rotated by the turret angle (robot frame). Rotating that vector
     *       into the field frame and subtracting it from the reported translation yields the
     *       chassis translation.
     * </ol>
     *
     * <p>The robot-frame → field-frame rotation uses the odometry heading rather than the freshly
     * derived vision heading: the offset is only ~0.3 m long, and gyro yaw is more stable than a
     * single-tag MegaTag1 yaw (which, in the low-confidence tiers, is not fused at all).
     *
     * @param cameraFieldPose the pose the turret camera reported, in the WPILib Blue origin frame
     * @return the corresponding chassis pose in the WPILib Blue origin frame
     */
    public Pose2d turretCameraPoseToRobotPose(Pose2d cameraFieldPose) {
        Rotation2d turretRotation = Rotation2d.fromDegrees(turretRotationSupplier.getAsDouble());

        // Robot-frame vector from robot centre to the camera at the current turret angle
        Translation2d robotToCamera =
                config.getRobotToTurretCenter()
                        .plus(config.getTurretCenterToCamera().rotateBy(turretRotation));

        // Same vector expressed in the field frame
        Translation2d robotToCameraField =
                robotToCamera.rotateBy(Robot.getSwerve().getRobotPose().getRotation());

        return new Pose2d(
                cameraFieldPose.getTranslation().minus(robotToCameraField),
                cameraFieldPose.getRotation().minus(turretRotation));
    }

    // =========================================================================
    // Pose Access & Queries
    // =========================================================================

    /**
     * Returns the MegaTag1 (MT1) pose from the back Limelight, or {@link Pose2d#kZero} if no
     * estimate is available.
     */
    public Pose2d getBackMegaTag1Pose() {
        Pose2d pose = backLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the MegaTag1 (MT1) pose from the left Limelight, or {@link Pose2d#kZero} if no
     * estimate is available.
     */
    public Pose2d getLeftMegaTag1Pose() {
        Pose2d pose = leftLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the MegaTag1 (MT1) pose from the right Limelight, or {@link Pose2d#kZero} if no
     * estimate is available.
     */
    public Pose2d getRightMegaTag1Pose() {
        Pose2d pose = rightLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the raw MegaTag1 (MT1) pose reported by the turret Limelight — i.e., the
     * <b>camera's</b> field pose, uncorrected for turret geometry. Use {@link
     * #getTurretRobotPose()} for a robot pose.
     */
    public Pose2d getTurretMegaTag1Pose() {
        Pose2d pose = turretLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the turret Limelight's MegaTag1 estimate converted to a chassis pose, or {@link
     * Pose2d#kZero} if the camera has no estimate.
     *
     * @see #turretCameraPoseToRobotPose(Pose2d)
     */
    public Pose2d getTurretRobotPose() {
        if (!turretLL.isAttached() || !turretLL.targetInView()) {
            return Pose2d.kZero;
        }
        return turretCameraPoseToRobotPose(getTurretMegaTag1Pose());
    }

    /**
     * Returns {@code true} when the turret camera has a reasonably large AprilTag in view (target
     * area &ge; 2 %).
     */
    public boolean isTurretSeeingTag() {
        return turretLL.targetInView() && turretLL.getTagTA() >= 2;
    }

    /**
     * Returns the MegaTag2 (MT2) pose from the back Limelight, or {@link Pose2d#kZero} if no
     * estimate is available.
     */
    public Pose2d getBackMegaTag2Pose() {
        Pose2d pose = backLL.getMegaTag2_Pose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the MegaTag2 (MT2) pose from the left Limelight, or {@link Pose2d#kZero} if no
     * estimate is available.
     */
    public Pose2d getLeftMegaTag2Pose() {
        Pose2d pose = leftLL.getMegaTag2_Pose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the MegaTag2 (MT2) pose from the right Limelight, or {@link Pose2d#kZero} if no
     * estimate is available.
     */
    public Pose2d getRightMegaTag2Pose() {
        Pose2d pose = rightLL.getMegaTag2_Pose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Selects and returns the chassis Limelight with the highest combined score of visible tag
     * count and target size. Ties fall back to {@link #backLL}.
     *
     * <p>The turret camera is excluded because its reported pose needs the turret correction
     * applied by {@link #getMT1TurretEstimate(Limelight, boolean)} and cannot go through the
     * chassis pipelines.
     *
     * @return the chassis Limelight currently offering the best view of AprilTags
     */
    public Limelight getBestLimelight() {
        Limelight bestLimelight = backLL;
        double bestScore = 0;
        for (Limelight limelight : swerveLimelights) {
            double score = limelight.getTagCountInView() + limelight.getTargetSize();
            if (score > bestScore) {
                bestScore = score;
                bestLimelight = limelight;
            }
        }
        return bestLimelight;
    }

    /**
     * Returns {@code true} if at least one Limelight reports an accurate pose (via {@link
     * Limelight#hasAccuratePose()}).
     */
    public boolean hasAccuratePose() {
        for (Limelight limelight : allLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /**
     * Returns {@code true} if any Limelight currently sees an AprilTag belonging to the current
     * alliance's set of scoring targets.
     *
     * <p>Uses {@link DriverStation#getAlliance()} and falls back to Red if the alliance is unknown.
     */
    public boolean tagsInView() {
        DriverStation.Alliance alliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        int[] allianceTags = (alliance == DriverStation.Alliance.Blue) ? blueTags : redTags;
        return Arrays.stream(allLimelights)
                .mapToInt(ll -> (int) ll.getClosestTagID())
                .anyMatch(id -> Arrays.stream(allianceTags).anyMatch(tag -> tag == id));
    }

    /**
     * Triggers a rewind-capture snapshot on all Limelights (captures 165 seconds of history for
     * post-match review).
     */
    public void triggerRewindCaptureForAllCameras() {
        for (Limelight limelight : allLimelights) {
            LimelightHelpers.triggerRewindCapture(limelight.getName(), 165);
        }
    }

    // =========================================================================
    // Pose Reset
    // =========================================================================

    /**
     * Resets the robot pose to the best available vision pose, preferring a chassis Limelight and
     * falling back to the turret camera. Delegates to {@link #resetPoseToVision(boolean, Pose3d,
     * Pose2d, double)} / {@link #resetPoseToTurretVision()}.
     */
    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        if (ll.targetInView()) {
            resetPoseToVision(
                    true,
                    ll.getMegaTag1_Pose3d(),
                    ll.getMegaTag2_Pose2d(),
                    ll.getMegaTag1PoseTimestamp());
            return;
        }
        resetPoseToTurretVision();
    }

    /**
     * Resets the robot pose using the turret camera's MegaTag1 estimate, corrected for the current
     * turret angle.
     *
     * <p>Unlike the chassis path this does not blend in MegaTag2: the turret camera runs on its own
     * internal IMU in a frame that rotates with the turret, so its MT2 estimate is not a robot
     * pose.
     *
     * @return {@code true} if the pose was accepted and the reset was applied
     */
    public boolean resetPoseToTurretVision() {
        if (!turretEstimatesAvailable() || !turretLL.targetInView()) {
            return false;
        }
        Pose3d cameraPose3d = turretLL.getMegaTag1_Pose3d();
        return applyPoseReset(
                cameraPose3d,
                turretCameraPoseToRobotPose(cameraPose3d.toPose2d()),
                turretLL.getMegaTag1PoseTimestamp());
    }

    /**
     * Resets the robot pose to a specific vision pose if the data passes sanity checks.
     *
     * <p>Uses a very tight covariance ({@code 0.00001}) so the pose estimator snaps immediately to
     * the vision measurement. The MT1 heading is preserved while MT2 provides the translation.
     *
     * <p>Rejection criteria:
     *
     * <ul>
     *   <li>No target in view.
     *   <li>Pose outside field boundary.
     *   <li>Camera height {@code |z| > 0.25 m} (robot floating / bad solve).
     *   <li>Roll or pitch &gt; 5° (camera physically disturbed).
     * </ul>
     *
     * @param targetInView whether the Limelight has a target
     * @param botpose3D the MT1 Pose3d from the Limelight
     * @param megaPose the MT2 Pose2d (provides translation for the reset)
     * @param poseTimestamp the FPGA timestamp of the pose estimate
     * @return {@code true} if the pose was accepted and the reset was applied
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {

        if (!targetInView) return false;

        // Use MT2 translation + MT1 heading for best combined accuracy
        Pose2d resetPose =
                new Pose2d(megaPose.getTranslation(), botpose3D.toPose2d().getRotation());
        return applyPoseReset(botpose3D, resetPose, poseTimestamp);
    }

    /**
     * Sanity-checks a candidate reset pose and, if it passes, snaps the pose estimator to it.
     *
     * @param sanityPose3d the raw camera 3-D solve, used for the height and tilt checks
     * @param resetPose the 2-D robot pose to snap to (already corrected for camera geometry)
     * @param poseTimestamp the FPGA timestamp of the pose estimate
     * @return {@code true} if the pose was accepted and the reset was applied
     */
    private boolean applyPoseReset(Pose3d sanityPose3d, Pose2d resetPose, double poseTimestamp) {
        if (FieldHelpers.poseOutOfField(resetPose)) {
            Telemetry.log("Vision/PoseReset/Rejection", "Out of field");
            return false;
        }
        if (Math.abs(sanityPose3d.getZ()) > 0.25) {
            Telemetry.log("Vision/PoseReset/Rejection", "Pose in air");
            return false;
        }
        if (Math.abs(sanityPose3d.getRotation().getX()) > Math.toRadians(5)
                || Math.abs(sanityPose3d.getRotation().getY()) > Math.toRadians(5)) {
            Telemetry.log("Vision/PoseReset/Rejection", "Pose tilted");
            return false;
        }

        double[] before = {
            resetPose.getX(), resetPose.getY(), resetPose.getRotation().getDegrees()
        };
        Telemetry.log("Vision/PoseReset/Before", before);

        Robot.getSwerve()
                .addVisionMeasurement(
                        resetPose, poseTimestamp, VecBuilder.fill(0.00001, 0.00001, 0.00001));

        Pose2d updated = Robot.getSwerve().getRobotPose();
        double[] after = {updated.getX(), updated.getY(), updated.getRotation().getDegrees()};
        Telemetry.log("Vision/PoseReset/After", after);

        return true;
    }

    // =========================================================================
    // Camera Control
    // =========================================================================

    /**
     * Sets all Limelights to the given pipeline index.
     *
     * @param pipeline the zero-indexed pipeline number to activate
     */
    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /**
     * Returns a command that blinks all Limelight LEDs while active and turns them off when the
     * command ends.
     *
     * @return the blink command
     */
    public Command blinkLimelights() {
        Telemetry.print("Vision.blinkLimelights", PrintPriority.HIGH);
        return startEnd(
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.blinkLEDs();
                            }
                        },
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.setLEDMode(false);
                            }
                        })
                .withName("Vision.blinkLimelights");
    }

    /**
     * Returns a command that holds all Limelight LEDs solid-on while active and turns them off when
     * the command ends.
     *
     * @return the solid-LED command
     */
    public Command solidLimelight() {
        return startEnd(
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.setLEDMode(true);
                            }
                        },
                        () -> {
                            for (Limelight limelight : allLimelights) {
                                limelight.setLEDMode(false);
                            }
                        })
                .withName("Vision.solidLimelight");
    }

    // =========================================================================
    // Inner Classes
    // =========================================================================

    /**
     * Immutable data class that bundles a vision-derived field pose with its FPGA timestamp and
     * covariance matrix, ready for use with {@code
     * SwerveDrivePoseEstimator.addVisionMeasurement()}.
     */
    @Getter
    public class VisionFieldPoseEstimate {

        /** The estimated field-relative robot pose (metres, radians). */
        private final Pose2d visionRobotPoseMeters;

        /** The FPGA-converted timestamp of this measurement (seconds). */
        private final double timestampSeconds;

        /**
         * The 3×1 standard-deviation vector {@code [x, y, theta]} passed to the pose estimator.
         * Larger values indicate less trust in that dimension.
         */
        private final Matrix<N3, N1> visionMeasurementStdDevs;

        /** Number of AprilTags that contributed to this estimate. */
        private final int numTags;

        /**
         * @param visionRobotPoseMeters field-relative robot pose
         * @param timestampSeconds FPGA-converted capture timestamp
         * @param visionMeasurementStdDevs 3×1 std-dev vector [x, y, theta]
         * @param numTags number of tags used in the solve
         */
        public VisionFieldPoseEstimate(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs,
                int numTags) {
            this.visionRobotPoseMeters = visionRobotPoseMeters;
            this.timestampSeconds = timestampSeconds;
            this.visionMeasurementStdDevs = visionMeasurementStdDevs;
            this.numTags = numTags;
        }
    }
}
