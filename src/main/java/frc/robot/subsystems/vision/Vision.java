package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
import lombok.Getter;

/**
 * Fuses the three Limelights (back, left, right) into the swerve pose estimator.
 *
 * <p>Only the camera with the best view feeds the estimator. While disabled both MegaTag1 and
 * MegaTag2 estimates are fused to pre-seed the pose; while enabled only MegaTag1 is.
 */
public class Vision implements Subsystem {

    /**
     * Camera identities, mount poses (translation in metres, rotation in degrees), and fusion
     * covariance.
     */
    public static class VisionConfig {

        @Getter final String name = "Vision";

        @Getter final String backLL = "limelight-back";

        @Getter
        final LimelightConfig backConfig =
                new LimelightConfig(backLL)
                        .withTranslation(-0.3084987734, 0.2134100126, 0.6502249886)
                        .withRotation(0, 0, 180);

        @Getter final String leftLL = "limelight-left";

        @Getter
        final LimelightConfig leftConfig =
                new LimelightConfig(leftLL).withTranslation(0, 0.215, 0.188).withRotation(0, 0, 90);

        @Getter final String rightLL = "limelight-right";

        @Getter
        final LimelightConfig rightConfig =
                new LimelightConfig(rightLL)
                        .withTranslation(-0.04445, 0.3027487722, 0.7137249886)
                        .withRotation(0, 0, -90);

        /** Robot centre to turret pivot offset, in metres. */
        @Getter
        final Translation2d robotToTurretCenter =
                new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(4.7));

        /** Turret pivot to camera offset, in metres. */
        @Getter
        final Translation2d turretCenterToCamera =
                new Translation2d(Units.inchesToMeters(-5.641455), 0);

        @Getter final int backTagPipeline = 0;
        @Getter final int leftTagPipeline = 0;
        @Getter final int rightTagPipeline = 0;

        /**
         * Default translational standard deviation for a vision measurement, in metres. Lower
         * values trust vision more, higher values trust odometry more.
         */
        @Getter double visionStdDevX = 0.5;

        /**
         * @see #visionStdDevX
         */
        @Getter double visionStdDevY = 0.5;

        /**
         * Default rotational standard deviation for a vision measurement, in radians. Only the MT1
         * path reads it; MT2 heading is always discarded.
         */
        @Getter double visionStdDevTheta = 0.2;

        /** Weight large enough to ignore a measurement dimension, such as MT2 heading. */
        @Getter final double kLargeVariance = 999999.0;

        /** Measurements older than this many seconds are not fused. */
        @Getter final double kMaxTimeDeltaSeconds = 0.1;

        @Getter
        final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta);
    }

    @Getter public final Limelight backLL;

    @Getter public final Limelight leftLL;

    @Getter public final Limelight rightLL;

    public final Limelight[] allLimelights;

    private final VisionLogger backLogger;
    private final VisionLogger leftLogger;
    private final VisionLogger rightLogger;

    private final VisionLogger[] allLoggers;

    /** Field tags that score for the blue alliance. */
    private final int[] blueTags = {18, 19, 20, 21, 24, 25, 26, 27};

    /** Field tags that score for the red alliance. */
    private final int[] redTags = {2, 3, 4, 5, 8, 9, 10, 11, 12};

    @Getter private static AprilTagFieldLayout tagLayout;

    private final VisionConfig config;

    private final IdentityHashMap<Limelight, Integer> lastImuModeByLL = new IdentityHashMap<>();

    public Vision(VisionConfig config) {
        this.config = config;

        backLL = new Limelight(config.backLL, config.backTagPipeline, config.backConfig);
        leftLL = new Limelight(config.leftLL, config.leftTagPipeline, config.leftConfig);
        rightLL = new Limelight(config.rightLL, config.rightTagPipeline, config.rightConfig);

        allLimelights = new Limelight[] {backLL, leftLL, rightLL};

        backLogger = new VisionLogger("BackLL", backLL);
        leftLogger = new VisionLogger("LeftLL", leftLL);
        rightLogger = new VisionLogger("RightLL", rightLL);
        allLoggers = new VisionLogger[] {backLogger, leftLogger, rightLogger};

        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
            setImuModeIfChanged(limelight, 1);
        }

        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        this.register();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public String getName() {
        return config.getName();
    }

    /** Called by the scheduler every loop, so everything here has to stay cheap. */
    @Override
    public void periodic() {
        for (Limelight limelight : allLimelights) {
            limelight.invalidate();
        }
        setLimeLightOrientation();
        // setRobotOrientation does not flush; one flush sends every camera's heading now.
        NetworkTableInstance.getDefault().flush();
        disabledLimelightUpdates();
        enabledLimelightUpdates();
        logTelemetry();
    }

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

        // MegaTag2 fuses the IMU, so its estimate drifts as soon as the robot moves
        if (Util.disabled.getAsBoolean()) {
            backLogger.getMegaPose();
            leftLogger.getMegaPose();
            rightLogger.getMegaPose();
        }

        Robot.getField2d().getObject(backLL.getCameraName()).setPose(getBackMegaTag1Pose());
        Robot.getField2d().getObject(leftLL.getCameraName()).setPose(getLeftMegaTag1Pose());
        Robot.getField2d().getObject(rightLL.getCameraName()).setPose(getRightMegaTag1Pose());
    }

    /** Pushes the current swerve heading to every camera so MegaTag2 fuses the right yaw. */
    private void setLimeLightOrientation() {
        double yaw = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        for (Limelight limelight : allLimelights) {
            limelight.setRobotOrientation(yaw);
        }
    }

    /**
     * Pre-seeds the pose estimator from the best camera while the robot is disabled, so the first
     * enabled loop starts from a vision pose instead of a stale one.
     */
    private void disabledLimelightUpdates() {
        if (Util.disabled.getAsBoolean()) {
            Limelight bestLimelight = getBestLimelight();
            integrateSingleEstimate(getMT1VisionEstimate(bestLimelight, true));
            integrateSingleEstimate(getMT2VisionEstimate(bestLimelight));
        }
    }

    /** Fuses the best camera's MegaTag1 estimate in teleop and the two auton states. */
    private void enabledLimelightUpdates() {
        if (Util.teleop.getAsBoolean()
                || Auton.autonPoseUpdate.getAsBoolean()
                || Auton.autonLaunching.getAsBoolean()) {
            Limelight bestLimelight = getBestLimelight();
            integrateSingleEstimate(getMT1VisionEstimate(bestLimelight, false));
        }
    }

    /**
     * Builds a MegaTag1 estimate for the given camera and decides whether it is trustworthy enough
     * to fuse.
     *
     * <p>Rejects on any of:
     *
     * <ul>
     *   <li>No targets in view.
     *   <li>Any tag ambiguity &gt; 0.9, which risks a pose flip.
     *   <li>Pose outside the field, or a capture older than {@link
     *       VisionConfig#getKMaxTimeDeltaSeconds()}.
     *   <li>Spin rate &ge; 1.6 rad/s.
     *   <li>Target area &le; 0.025 %.
     *   <li>Roll or pitch &gt; 5&deg;, which means the camera was bumped.
     * </ul>
     *
     * <p>Accepted estimates get std-devs from a confidence tier based on tag count, target area,
     * and distance from the current odometry pose. {@code forceIntegrateXY} clamps those to near
     * zero so pre-seeding snaps the estimator to vision.
     *
     * @return an estimate to fuse, or {@code null} if it was rejected
     */
    private VisionFieldPoseEstimate getMT1VisionEstimate(Limelight ll, boolean forceIntegrateXY) {
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

        double mt1PoseDifference =
                Robot.getSwerve()
                        .getRobotPose()
                        .getTranslation()
                        .getDistance(megaTag1Pose2d.getTranslation());

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

        if (rejectionCheck(ll, megaTag1Pose2d, targetSize)) {
            return null;
        }

        if (Math.abs(megaTag1Pose3d.getRotation().getX()) > Math.toRadians(5)
                || Math.abs(megaTag1Pose3d.getRotation().getY()) > Math.toRadians(5)) {
            ll.sendInvalidStatus("Roll/Pitch Rejection");
            return null;
        }

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

        if (highestAmbiguity > 0.5) {
            degStds = Math.max(degStds, 15);
        }

        if (Math.abs(robotSpeed.omegaRadiansPerSecond) >= 0.5) {
            degStds = Math.max(degStds, 50);
        }

        if (forceIntegrateXY) {
            xyStds = 0.01;
            degStds = 0.01;
        }

        Pose2d integratedPose =
                new Pose2d(megaTag1Pose2d.getTranslation(), megaTag1Pose2d.getRotation());
        double timestamp = Utils.fpgaToCurrentTime(ll.getMegaTag1PoseTimestamp());
        // degStds is in degrees while the estimator wants radians
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));

        @SuppressWarnings("null")
        int numTags = tags.length;

        return new VisionFieldPoseEstimate(integratedPose, timestamp, stdDevs, numTags);
    }

    /**
     * Builds a MegaTag2 estimate for the given camera.
     *
     * <p>MegaTag2 heading is discarded ({@link VisionConfig#kLargeVariance}) because it comes from
     * the IMU rather than tag geometry. Only the disabled path calls this, which means the {@code
     * DriverStation.isDisabled()} terms in the distance tiers always pass and pose distance never
     * rejects here.
     *
     * <p>Rejects on no targets in view, plus everything {@link #rejectionCheck} covers.
     *
     * @return an estimate to fuse, or {@code null} if it was rejected
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

    /** Fuses the estimate, ignoring the null that a rejected candidate returns. */
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
     * Rejection gate shared by the MT1 and MT2 paths. Rejects on a pose outside the field, a
     * capture older than {@link VisionConfig#getKMaxTimeDeltaSeconds()}, a spin rate &ge; 1.6
     * rad/s, and a target area &le; 0.025 %. The staleness check reads the MT1 timestamp, so the
     * MT2 path is gated on the MT1 capture age.
     *
     * @return {@code true} to reject the measurement
     */
    private boolean rejectionCheck(Limelight ll, Pose2d pose, double targetSize) {
        if (FieldHelpers.poseOutOfField(pose)) {
            ll.sendInvalidStatus("Out of Field Rejection");
            return true;
        }

        double timestamp = Utils.fpgaToCurrentTime(ll.getMegaTag1PoseTimestamp());
        double currentTime = Utils.fpgaToCurrentTime(Timer.getFPGATimestamp());
        if (currentTime - timestamp > config.getKMaxTimeDeltaSeconds()) {
            ll.sendInvalidStatus("Stale Timestamp Rejection");
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
     * Writes the mode only when it differs from the last one written, to spare NetworkTables.
     *
     * @param desiredMode 0 takes the heading from the DriverStation, 1 from the camera's own IMU, 2
     *     fuses the two
     */
    private void setImuModeIfChanged(Limelight limelight, int desiredMode) {
        Integer lastMode = lastImuModeByLL.get(limelight);
        if (lastMode == null || lastMode.intValue() != desiredMode) {
            limelight.setIMUmode(desiredMode);
            lastImuModeByLL.put(limelight, desiredMode);
        }
    }

    /** MegaTag1 pose, or {@link Pose2d#kZero} without a usable estimate. */
    public Pose2d getBackMegaTag1Pose() {
        Pose2d pose = backLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /** MegaTag1 pose, or {@link Pose2d#kZero} without a usable estimate. */
    public Pose2d getLeftMegaTag1Pose() {
        Pose2d pose = leftLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /** MegaTag1 pose, or {@link Pose2d#kZero} without a usable estimate. */
    public Pose2d getRightMegaTag1Pose() {
        Pose2d pose = rightLL.getMegaTag1_Pose3d().toPose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /** MegaTag2 pose, or {@link Pose2d#kZero} without a usable estimate. */
    public Pose2d getBackMegaTag2Pose() {
        Pose2d pose = backLL.getMegaTag2_Pose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /** MegaTag2 pose, or {@link Pose2d#kZero} without a usable estimate. */
    public Pose2d getLeftMegaTag2Pose() {
        Pose2d pose = leftLL.getMegaTag2_Pose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /** MegaTag2 pose, or {@link Pose2d#kZero} without a usable estimate. */
    public Pose2d getRightMegaTag2Pose() {
        Pose2d pose = rightLL.getMegaTag2_Pose2d();
        return pose != null ? pose : Pose2d.kZero;
    }

    /**
     * Camera with the highest tag count plus target size, or the back one when nothing is in view.
     */
    public Limelight getBestLimelight() {
        Limelight bestLimelight = backLL;
        double bestScore = 0;
        for (Limelight limelight : allLimelights) {
            double score = limelight.getTagCountInView() + limelight.getTargetSize();
            if (score > bestScore) {
                bestScore = score;
                bestLimelight = limelight;
            }
        }
        return bestLimelight;
    }

    public boolean hasAccuratePose() {
        for (Limelight limelight : allLimelights) {
            if (limelight.hasAccuratePose()) return true;
        }
        return false;
    }

    /**
     * True if any camera's primary target is a scoring tag for the current alliance, treating an
     * unknown alliance as blue.
     */
    public boolean tagsInView() {
        DriverStation.Alliance alliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        int[] allianceTags = (alliance == DriverStation.Alliance.Blue) ? blueTags : redTags;
        return Arrays.stream(allLimelights)
                .mapToInt(ll -> (int) ll.getClosestTagID())
                .anyMatch(id -> Arrays.stream(allianceTags).anyMatch(tag -> tag == id));
    }

    /** Asks every camera to rewind-capture 165 s, the longest buffer the Limelight keeps. */
    public void triggerRewindCaptureForAllCameras() {
        for (Limelight limelight : allLimelights) {
            LimelightHelpers.triggerRewindCapture(limelight.getName(), 165);
        }
    }

    /**
     * Resets the pose from the best camera, taking its MT2 translation and MT1 heading. A rejected
     * pose leaves the estimator untouched, and this overload gives the caller no way to tell.
     */
    public void resetPoseToVision() {
        Limelight ll = getBestLimelight();
        resetPoseToVision(
                ll.targetInView(),
                ll.getMegaTag1_Pose3d(),
                ll.getMegaTag2_Pose2d(),
                ll.getMegaTag1PoseTimestamp());
    }

    /**
     * Snaps the pose estimator onto a specific vision pose. The covariance is tiny, so the
     * estimator jumps to vision instead of blending toward it.
     *
     * <p>Rejects on:
     *
     * <ul>
     *   <li>No target in view.
     *   <li>Either pose outside the field.
     *   <li>Camera height {@code |z| > 0.25 m}, which means a floating robot or a bad solve.
     *   <li>Roll or pitch &gt; 5&deg;, which means the camera was bumped.
     * </ul>
     *
     * @param botpose3D the MT1 pose, which supplies the heading
     * @param megaPose the MT2 pose, which supplies the translation
     * @return {@code true} if the reset was applied
     */
    public boolean resetPoseToVision(
            boolean targetInView, Pose3d botpose3D, Pose2d megaPose, double poseTimestamp) {

        if (!targetInView) return false;

        Pose2d botpose = botpose3D.toPose2d();

        if (FieldHelpers.poseOutOfField(botpose3D)) {
            Telemetry.log("Vision/PoseReset/Rejection", "Out of field");
            return false;
        }
        if (Math.abs(botpose3D.getZ()) > 0.25) {
            Telemetry.log("Vision/PoseReset/Rejection", "Pose in air");
            return false;
        }
        if (Math.abs(botpose3D.getRotation().getX()) > Math.toRadians(5)
                || Math.abs(botpose3D.getRotation().getY()) > Math.toRadians(5)) {
            Telemetry.log("Vision/PoseReset/Rejection", "Pose tilted");
            return false;
        }

        double[] before = {botpose.getX(), botpose.getY(), botpose.getRotation().getDegrees()};
        Telemetry.log("Vision/PoseReset/Before", before);

        if (FieldHelpers.poseOutOfField(megaPose)) {
            Telemetry.log("Vision/PoseReset/Rejection", "MegaPose out of field");
            return false;
        }

        Pose2d integratedPose = new Pose2d(megaPose.getTranslation(), botpose.getRotation());
        Robot.getSwerve()
                .addVisionMeasurement(
                        integratedPose,
                        Utils.fpgaToCurrentTime(poseTimestamp),
                        VecBuilder.fill(0.00001, 0.00001, 0.00001));

        Pose2d updated = Robot.getSwerve().getRobotPose();
        double[] after = {updated.getX(), updated.getY(), updated.getRotation().getDegrees()};
        Telemetry.log("Vision/PoseReset/After", after);

        return true;
    }

    public void setLimelightPipelines(int pipeline) {
        for (Limelight limelight : allLimelights) {
            limelight.setLimelightPipeline(pipeline);
        }
    }

    /** Blinks every camera's LEDs while the command runs, then turns them off. */
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

    /** Holds every camera's LEDs on while the command runs, then turns them off. */
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

    /**
     * A pose estimate bundled with the timestamp and std-devs that {@code
     * SwerveDrivePoseEstimator.addVisionMeasurement()} expects.
     */
    @Getter
    public class VisionFieldPoseEstimate {

        private final Pose2d visionRobotPoseMeters;

        /** Capture time already converted out of the FPGA clock, in seconds. */
        private final double timestampSeconds;

        /** Std-devs for {@code [x, y, theta]}. Larger values mean less trust in that dimension. */
        private final Matrix<N3, N1> visionMeasurementStdDevs;

        private final int numTags;

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
