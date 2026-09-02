package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.rebuilt.Field;
import frc.rebuilt.FieldHelpers;
import frc.robot.Robot;
import frc.robot.auton.Auton;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.util.Util;
import frc.spectrumLib.vision.Limelight;
import frc.spectrumLib.vision.Limelight.LimelightConfig;
import frc.spectrumLib.vision.LimelightHelpers;
import frc.spectrumLib.vision.LimelightHelpers.RawFiducial;
import frc.spectrumLib.vision.VisionLogger;
import java.util.IdentityHashMap;
import java.util.function.DoubleSupplier;
import lombok.Getter;

/**
 * Vision subsystem that manages the turret-mounted Limelight and fuses its pose estimates into the
 * swerve odometry via WPILib's {@code SwerveDrivePoseEstimator}.
 *
 * <p>Supports optional chassis-mounted cameras (back/left/right, detached by default) plus a
 * turret-mounted camera whose frame rotates with the turret. For the turret, the live
 * robot-to-camera transform (including the turret's yaw) is published every loop so the camera
 * solves for a robot pose directly; chassis cameras use their fixed configured mount.
 *
 * <p>Each robot loop iteration the subsystem:
 *
 * <ol>
 *   <li>Publishes the turret-rotated camera transform ({@link #updateTurretCameraPose()}) and the
 *       robot heading to every camera, then flushes NetworkTables so they solve this frame.
 *   <li>Picks the best chassis camera ({@link #getBestLimelight()}) and integrates its MT1
 *       (enabled) or MT1+MT2 (disabled). The turret camera always integrates via MT2 ({@link
 *       #getMT2VisionEstimate(Limelight)}), trusting the pushed robot heading.
 *   <li>Logs camera status and pose data via {@link VisionLogger}.
 * </ol>
 *
 * <p>The camera can be enabled/disabled with {@link LimelightConfig#setAttached(boolean)}; a
 * detached camera is never read from and contributes no estimates.
 */
public class Vision implements Subsystem {

    // =========================================================================
    // Configuration
    // =========================================================================

    public static class VisionConfig {

        @Getter final String name = "Vision";

        // -- Back-Left Limelight ----------------------------------------------

        /** NetworkTables hostname for the rear-left static Limelight. */
        @Getter final String backLeftLL = "limelight-back-left";

        /**
         * Robot-relative pose of the rear-left Limelight. Translation in metres (x, y, z); rotation
         * in degrees (roll, pitch, yaw). Detached by default; enable per-robot in the config
         * package. TODO: measure real mount offsets before enabling.
         */
        @Getter
        final LimelightConfig backLeftConfig =
                new LimelightConfig(backLeftLL)
                        .withTranslation(0, 0, 0)
                        .withRotation(0, 0, 0)
                        .setAttached(false);

        // -- Back-Right Limelight ---------------------------------------------

        /** NetworkTables hostname for the rear-right static Limelight. */
        @Getter final String backRightLL = "limelight-back-right";

        /**
         * Robot-relative pose of the rear-right Limelight. Detached by default; enable per-robot in
         * the config package. TODO: measure real mount offsets before enabling.
         */
        @Getter
        final LimelightConfig backRightConfig =
                new LimelightConfig(backRightLL)
                        .withTranslation(-0.3084987734, 0, 0)
                        .withRotation(0, 0, 0)
                        .setAttached(false);

        // -- Turret Limelight -------------------------------------------------

        /** NetworkTables hostname for the turret-mounted Limelight. */
        @Getter final String turretLL = "limelight-turret";

        /**
         * Robot-relative pose of the turret Limelight <b>with the turret at zero</b>. Translation
         * in metres (x, y, z); rotation in degrees (roll, pitch, yaw).
         *
         * <p>The offsets entered in the Limelight GUI are irrelevant for pose solving: {@link
         * #updateTurretCameraPose()} overwrites all six values over NetworkTables every loop with
         * the live turret-rotated transform, so the camera reports a robot pose directly. The
         * values here supply the parts that do not move with the turret (height, roll, pitch) and
         * are also used by {@link Limelight#getDistanceToTarget(double)}.
         */
        @Getter
        final LimelightConfig turretConfig =
                new LimelightConfig(turretLL)
                        .withTranslation(
                                Units.inchesToMeters(5.375), // ahead (unused)
                                Units.inchesToMeters(0.0), // left (unused)
                                Units.inchesToMeters(18.632)) // up
                        .withRotation(0, 60, 0); // yaw unused; live turret angle is used

        // -- Turret geometry --------------------------------------------------

        /** Robot-centre to turret pivot offset (metres). */
        @Getter final Translation2d robotToTurretCenter = Translation2d.kZero;

        /** Turret pivot to camera offset (metres), measured with the turret at zero. */
        @Getter
        final Translation2d turretCenterToCamera =
                new Translation2d(Units.inchesToMeters(5.375), 0);

        // -- Pipeline indices -------------------------------------------------

        @Getter final int backLeftTagPipeline = 0;
        @Getter final int backRightTagPipeline = 0;
        @Getter final int turretTagPipeline = 0;

        // -- Pose estimation covariance ---------------------------------------

        /**
         * Variance used to effectively ignore a measurement dimension — here, the heading of a
         * single-tag or low-confidence estimate. The X/Y/heading std-devs that are actually fused
         * are chosen per-estimate in {@link Vision#getMT1Estimate(Limelight, boolean)}.
         */
        @Getter final double kLargeVariance = 999999.0;
    }

    // =========================================================================
    // Fields
    // =========================================================================

    /** Rear-left static Limelight instance. */
    @Getter public final Limelight backLeftLL;

    /** Rear-right static Limelight instance. */
    @Getter public final Limelight backRightLL;

    /** Turret-mounted Limelight instance; its frame rotates with the turret. */
    public final Limelight turretLL;

    /** Chassis-mounted (non-turret) Limelights, whose reported pose is already the robot pose. */
    public final Limelight[] swerveLimelights;

    /** All Limelights in one array for bulk operations. */
    public final Limelight[] allLimelights;

    /* Vision loggers — one per Limelight */
    private final VisionLogger backLeftLogger;
    private final VisionLogger backRightLogger;
    private final VisionLogger turretLogger;

    /** All loggers in one array for bulk telemetry loops. */
    private final VisionLogger[] allLoggers;

    /** Live turret angle in degrees, positive counter-clockwise, zero pointing robot-forward. */
    private final DoubleSupplier turretRotationSupplier =
            () -> Robot.getTurret().getPositionDegrees();

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
     * <p>Instantiates the Limelight and its logger, sets IMU mode 0 (external heading only, correct
     * for a mount whose frame rotates relative to the robot), and registers this subsystem with the
     * WPILib scheduler.
     *
     * @param config the static configuration object
     */
    public Vision(VisionConfig config) {
        this.config = config;

        backLeftLL =
                new Limelight(config.backLeftLL, config.backLeftTagPipeline, config.backLeftConfig);
        backRightLL =
                new Limelight(
                        config.backRightLL, config.backRightTagPipeline, config.backRightConfig);
        turretLL = new Limelight(config.turretLL, config.turretTagPipeline, config.turretConfig);

        swerveLimelights = new Limelight[] {backLeftLL, backRightLL};
        allLimelights = new Limelight[] {backLeftLL, backRightLL, turretLL};

        int[] validIds =
                Field.AprilTagLayoutType.OFFICIAL.getLayout().getTags().stream()
                        .mapToInt(tag -> tag.ID)
                        .toArray();
        for (Limelight limelight : allLimelights) {
            LimelightHelpers.SetFiducialIDFiltersOverride(limelight.getName(), validIds);
        }

        backLeftLogger = new VisionLogger("BackLeftLL", backLeftLL);
        backRightLogger = new VisionLogger("BackRightLL", backRightLL);
        turretLogger = new VisionLogger("TurretLL", turretLL);
        allLoggers = new VisionLogger[] {backLeftLogger, backRightLogger, turretLogger};

        // Chassis cams use the pushed robot heading (mode 1); the turret runs external-only
        // (mode 0) since its frame yaws with the turret.
        for (Limelight limelight : swerveLimelights) {
            limelight.setLEDMode(false);
            setImuModeIfChanged(limelight, 1);
        }
        turretLL.setLEDMode(false);
        setImuModeIfChanged(turretLL, 0);

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
     *   <li>Publishes the live camera mount transform and the robot heading, then flushes NT.
     *   <li>Runs pose-estimation updates appropriate to the current robot mode.
     *   <li>Logs camera telemetry.
     * </ol>
     */
    @Override
    public void periodic() {
        updateTurretCameraPose();
        setLimeLightOrientation();

        NetworkTableInstance.getDefault().flush();

        disabledLimelightUpdates();
        enabledLimelightUpdates();
        logTelemetry();
    }

    /**
     * Logs connection status, integration status, tag status, pose, tag count, and target size for
     * the camera via its {@link VisionLogger}, and updates the {@code Field2d} widget with the
     * reported robot pose.
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

        // Null-safe; returns Pose2d.kZero when no data
        Robot.getField2d().getObject(backLeftLL.getCameraName()).setPose(getBackLeftMegaTag1Pose());
        Robot.getField2d()
                .getObject(backRightLL.getCameraName())
                .setPose(getBackRightMegaTag1Pose());
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
     * Pushes the robot's current heading (from swerve odometry) to the camera each loop.
     *
     * <p>This is the <b>robot</b> heading, not the camera's: {@link #updateTurretCameraPose()}
     * tells the camera where it is mounted, including the turret's yaw, so the camera composes the
     * two itself.
     */
    private void setLimeLightOrientation() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        for (Limelight limelight : swerveLimelights) {
            limelight.setRobotOrientation(yaw);
        }
        turretLL.setRobotOrientation(yaw);
    }

    /**
     * Publishes the live robot-to-camera transform to the turret Limelight, overriding the offsets
     * configured in its GUI.
     *
     * <p>The camera sits at {@code robotToTurretCenter + turretCenterToCamera} rotated by the
     * current turret angle, and its yaw in the robot frame <i>is</i> the turret angle. Height,
     * roll, and pitch do not move with the turret and come from {@link
     * VisionConfig#getTurretConfig()}. Publishing this every loop means the camera's reported
     * botpose is already a robot pose, so no de-rotation is needed downstream.
     */
    private void updateTurretCameraPose() {
        Rotation2d turretRotation =
                Rotation2d.fromDegrees(turretRotationSupplier.getAsDouble())
                        .plus(Rotation2d.k180deg);
        Translation2d robotToCamera =
                config.getRobotToTurretCenter()
                        .plus(config.getTurretCenterToCamera().rotateBy(turretRotation));
        LimelightConfig cam = config.getTurretConfig();

        Pose3d cameraPose =
                new Pose3d(
                        new Translation3d(robotToCamera.getX(), robotToCamera.getY(), cam.getUp()),
                        new Rotation3d(
                                Math.toRadians(cam.getRoll()),
                                Math.toRadians(cam.getPitch()),
                                turretRotation.getRadians()));

        turretLL.updateCameraPose(cameraPose);
        Telemetry.log("Vision/TurretCameraPose", cameraPose.toString());
    }

    /**
     * While the robot is disabled, seeds the pose estimator from MegaTag2 translation and MegaTag1
     * heading, so the pose is correct before the match starts.
     */
    private void disabledLimelightUpdates() {
        if (Util.disabled.getAsBoolean()) {
            Limelight best = getBestLimelight();
            markUnselectedLimelights(best);
            integrateSingleEstimate(getMT1Estimate(best, true));
            integrateSingleEstimate(getMT2VisionEstimate(best));

            if (turretEstimatesAvailable()) {
                integrateSingleEstimate(getMT2VisionEstimate(turretLL));
            }
        }
    }

    /**
     * While the robot is enabled (teleop or auto pose-update), integrates the best chassis camera's
     * MT1 estimate and the turret camera's MT2 estimate.
     */
    private void enabledLimelightUpdates() {
        if (Util.teleop.getAsBoolean() || Auton.autonPoseUpdate.getAsBoolean()) {
            Limelight best = getBestLimelight();
            markUnselectedLimelights(best);
            integrateSingleEstimate(getMT1Estimate(best, false));

            if (turretEstimatesAvailable()) {
                integrateSingleEstimate(getMT2VisionEstimate(turretLL));
            }
        }
    }

    /**
     * Returns {@code true} when the turret camera can produce a usable estimate — both the camera
     * and the turret mechanism must be attached, since the published mount transform needs a live
     * turret angle.
     */
    private boolean turretEstimatesAvailable() {
        return turretLL.isAttached() && Robot.getTurret().isAttached();
    }

    /**
     * Builds a MegaTag1 (multi-tag, heading-fused) pose estimate for a chassis Limelight and
     * decides whether to add it. The camera's botpose is already a robot pose via its configured
     * mount.
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
    private VisionFieldPoseEstimate getMT1Estimate(Limelight ll, boolean forceIntegrateXY) {
        if (!ll.targetInView()) {
            ll.setTagStatus("No Targets in View");
            ll.sendInvalidStatus("No Targets in View Rejection");
            return null;
        }

        boolean multiTags = ll.multipleTagsInView();
        double targetSize = ll.getTargetSize();
        Pose3d megaTag1Pose3d = ll.getMegaTag1_Pose3d();
        Pose2d megaTag1Pose2d = megaTag1Pose3d.toPose2d();

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

        // Widen heading std-dev when ambiguity is moderate.
        if (highestAmbiguity > 0.5) {
            degStds = Math.max(degStds, 15);
        }

        // Discard heading during fast rotation (MegaTag1 heading unreliable while spinning)
        if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 0.5) {
            degStds = Math.max(degStds, 50);
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
     * Builds a MegaTag2 (IMU-fused) pose estimate for a chassis Limelight. MT2 heading is always
     * discarded (set to {@link VisionConfig#getKLargeVariance()}); only its translation is fused,
     * which is why it is used mainly while stationary. Not used for the turret camera, whose MT2
     * heading would be the turret-frame prior we push in.
     *
     * @param ll the chassis Limelight to query
     * @return a {@link VisionFieldPoseEstimate}, or {@code null} if rejected
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
     * Selects the chassis Limelight with the best current view, scored by visible tag count plus
     * target size. Detached cameras score 0, so on a turret-only robot this returns {@code
     * backLeftLL} with score 0 and its estimate is rejected downstream (no target in view).
     *
     * @return the chassis Limelight currently offering the best view of AprilTags
     */
    public Limelight getBestLimelight() {
        Limelight bestLimelight = backLeftLL;
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
     * Marks every chassis Limelight except {@code bestLimelight} as not integrating, so their
     * telemetry status reflects that only the selected camera fed the estimator this loop.
     *
     * @param bestLimelight the camera being integrated this loop, left untouched
     */
    private void markUnselectedLimelights(Limelight bestLimelight) {
        for (Limelight limelight : swerveLimelights) {
            if (limelight != bestLimelight) {
                limelight.sendInvalidStatus("Not best Limelight");
            }
        }
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

        // Only the turret camera moves with the turret; reject its estimate while it slews (smear
        // and mount-transform lag).
        if (ll == turretLL && Math.abs(Robot.getTurret().getMechOmegaRotPerSec()) >= 0.75) {
            ll.sendInvalidStatus("Turret Speed Rejection");
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
    // Pose Access & Queries
    // =========================================================================

    /** MegaTag1 robot pose from the back-left Limelight, or {@link Pose2d#kZero} if unavailable. */
    public Pose2d getBackLeftMegaTag1Pose() {
        Pose2d pose = backLeftLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /** MegaTag1 robot pose from the back-right Limelight, or {@link Pose2d#kZero} if none. */
    public Pose2d getBackRightMegaTag1Pose() {
        Pose2d pose = backRightLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the MegaTag1 (MT1) robot pose reported by the turret Limelight. It is already a robot
     * pose rather than a camera pose, because {@link #updateTurretCameraPose()} publishes the live
     * mount transform.
     */
    public Pose2d getTurretMegaTag1Pose() {
        Pose2d pose = turretLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Returns the turret Limelight's MegaTag1 robot pose, or {@link Pose2d#kZero} if the camera has
     * no estimate.
     */
    public Pose2d getTurretRobotPose() {
        if (!turretLL.isAttached() || !turretLL.targetInView()) {
            return Pose2d.kZero;
        }
        return getTurretMegaTag1Pose();
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
     * Resets the robot pose using the turret camera's MegaTag1 estimate.
     *
     * <p>MegaTag1 rather than MegaTag2, because this is the operator's explicit "fix my pose"
     * action and heading is the part most worth fixing. Under IMU mode 0 an MT2 heading is only the
     * prior we pushed in, so it could never correct one; MT1 solves heading from tag geometry.
     *
     * @return {@code true} if the pose was accepted and the reset was applied
     */
    public boolean resetPoseToTurretVision() {
        if (!turretEstimatesAvailable() || !turretLL.targetInView()) {
            return false;
        }
        Pose3d robotPose3d = turretLL.getMegaTag1_Pose3d();
        return applyPoseReset(
                robotPose3d, robotPose3d.toPose2d(), turretLL.getMegaTag1PoseTimestamp());
    }

    /**
     * Sanity-checks a candidate reset pose and, if it passes, snaps the pose estimator to it.
     *
     * @param sanityPose3d the raw camera 3-D solve, used for the height and tilt checks
     * @param resetPose the 2-D robot pose to snap to
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
    // Commands
    // =========================================================================

    /**
     * Returns a command that seeds the robot pose from the current AprilTag estimate. Runs while
     * disabled, since seeding the pose before a match starts is the point of it.
     *
     * @return the pose reset command
     */
    public Command resetVisionPoseCommand() {
        return runOnce(this::resetPoseToTurretVision)
                .ignoringDisable(true)
                .withName("Vision.resetPoseToTurretVision");
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
