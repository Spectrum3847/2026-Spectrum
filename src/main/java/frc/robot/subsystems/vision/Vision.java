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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
 *   <li>Picks the best chassis camera ({@link #getBestLimelight()}). While disabled it seeds the
 *       pose (translation and heading) from that camera's MT1 only. While enabled it fuses that
 *       camera's MT1 translation and the turret camera's MT2 translation ({@link
 *       #getMT2VisionEstimate(Limelight)}), never heading, unless the gross-heading safety net
 *       fires ({@link #checkGrossHeadingError(Limelight)}).
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
        @Getter final String backLeftLL = "limelight-left"; // must match the camera's hostname

        /**
         * Robot-relative pose of the rear-left Limelight. Translation in metres (forward, right,
         * up); rotation in degrees (roll, pitch, yaw). Translation measured in CAD from the robot
         * origin (robot centre, on the carpet) to the camera location: 11.103 in behind centre,
         * 12.490 in left of centre, 17.058 in up.
         *
         * <p>Rotation: the camera is mounted upside down (roll 180) on the angled rear-left corner
         * panel, looking out over that corner (yaw +135, i.e. rear-left) and 60 deg above
         * horizontal. If the Limelight web UI image orientation is set to flip the image 180 deg,
         * enter roll 0 there instead of 180. These match the values entered in the camera's web UI
         * on 2026-09-04.
         *
         * <p>Documentation only for pose solving: chassis cameras use the offsets entered in the
         * Limelight web UI, so keep the two in sync.
         */
        @Getter
        final LimelightConfig backLeftConfig =
                new LimelightConfig(backLeftLL)
                        .withTranslation(
                                Units.inchesToMeters(-11.103), // forward (behind centre)
                                Units.inchesToMeters(-12.490), // right (left of centre)
                                Units.inchesToMeters(17.058)) // up
                        .withRotation(180, 60, 135) // upside down, 60 deg up, facing rear-left
                        .setAttached(true);

        // -- Back-Right Limelight ---------------------------------------------

        /** NetworkTables hostname for the rear-right static Limelight. */
        @Getter final String backRightLL = "limelight-right"; // must match the camera's hostname

        /**
         * Robot-relative pose of the rear-right Limelight. Translation measured in CAD from the
         * robot origin (robot centre, on the carpet) to the camera location: 10.064 in behind
         * centre, 13.315 in right of centre, 17.458 in up.
         *
         * <p>Rotation: the camera is mounted upside down (roll 180) on the angled rear-right corner
         * panel, looking out over that corner (yaw -135, i.e. rear-right) and 60 deg above
         * horizontal. If the Limelight web UI image orientation is set to flip the image 180 deg,
         * enter roll 0 there instead of 180. These match the values entered in the camera's web UI
         * on 2026-09-04.
         *
         * <p>Documentation only for pose solving: chassis cameras use the offsets entered in the
         * Limelight web UI, so keep the two in sync.
         */
        @Getter
        final LimelightConfig backRightConfig =
                new LimelightConfig(backRightLL)
                        .withTranslation(
                                Units.inchesToMeters(-10.064), // forward (behind centre)
                                Units.inchesToMeters(13.315), // right
                                Units.inchesToMeters(17.458)) // up
                        .withRotation(180, 60, -135) // upside down, 60 deg up, facing rear-right
                        .setAttached(true);

        // -- Turret Limelight -------------------------------------------------

        /** NetworkTables hostname for the turret-mounted Limelight. */
        @Getter final String turretLL = "limelight-turret";

        /**
         * Robot-relative pose of the turret Limelight <b>with the turret at zero</b>, facing the
         * robot rear. Translation in metres (forward, right, up); rotation in degrees (roll, pitch,
         * yaw). Measured in CAD from the robot origin (robot centre, on the carpet) to the camera
         * location: on the centreline, 0.138 m behind centre. Height is NOT from CAD (the model is
         * known to be wrong there); 18.632 in is the value measured on the robot.
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
                                -0.138, // forward at turret zero (unused; see turretCenterToCamera)
                                0.0, // right (unused)
                                Units.inchesToMeters(18.632)) // up (measured on robot, not CAD)
                        .withRotation(0, 60, 0); // yaw unused; live turret angle is used

        // -- Turret geometry --------------------------------------------------

        /** Robot-centre to turret pivot offset (metres). */
        @Getter final Translation2d robotToTurretCenter = Translation2d.kZero;

        /**
         * Turret pivot to camera offset (metres) along the camera look direction, measured with the
         * turret at zero. From CAD the camera sits 0.138 m behind the robot centre at zero, which
         * equals the pivot arm as long as robotToTurretCenter really is zero.
         */
        @Getter final Translation2d turretCenterToCamera = new Translation2d(0.138, 0);

        // -- Pipeline indices -------------------------------------------------

        @Getter final int backLeftTagPipeline = 0;
        @Getter final int backRightTagPipeline = 0;
        @Getter final int turretTagPipeline = 0;

        // -- Pose estimation covariance ---------------------------------------

        /**
         * Variance used to effectively ignore a measurement dimension — here, the heading of every
         * estimate fused while enabled, so the gyro owns heading during a match. The X/Y std-devs
         * that are actually fused are chosen per-estimate in {@link
         * Vision#getMT1Estimate(Limelight, boolean)}.
         */
        @Getter final double kLargeVariance = 999999.0;

        // -- Estimate sanity gates --------------------------------------------

        /**
         * Estimates older than this (seconds, capture time to now) are rejected. The pose estimator
         * silently drops anything older than its 1.5 s buffer, so without this gate a camera with a
         * bad clock looks "integrating" while never moving the pose.
         */
        @Getter final double maxEstimateAgeSeconds = 1.0;

        /**
         * Turret camera gate: if the turret camera's MegaTag1 heading disagrees with the gyro by
         * more than this (degrees), its estimates are rejected. The turret camera's mount transform
         * is built from the turret encoder, so a disagreement here means the turret zero is off by
         * that amount and every pose the camera reports is displaced by range times that angle.
         */
        @Getter final double turretHeadingMismatchDeg = 5.0;

        // -- Gross heading correction while enabled ---------------------------
        //
        // Heading is normally gyro-only while enabled. This is the safety net for enabling before
        // the cameras have seeded the pose: a stationary robot whose two-tag MegaTag1 heading
        // disagrees with the gyro by a lot, for a full second, gets its heading reset once.

        /**
         * Heading error that arms the correction.
         *
         * <p>Was 10.0, which is above the error this actually has to catch. In the 2026-09-05 18:00
         * log the correction computed a median error of -6.7 deg for the whole enabled window and
         * never armed, because 6.7 is not 10. That 6.7 deg is about a foot and a half of miss at
         * three metres, so "not gross enough to bother with" was the wrong call.
         *
         * <p>It still takes a stationary robot and a full second of agreement from a fresh
         * multi-tag estimate before it moves anything, which is what keeps vision noise from
         * fighting the gyro.
         */
        @Getter final double grossHeadingErrorDeg = 3.0;

        @Getter final double grossHeadingHoldSeconds = 1.0;
        @Getter final double grossHeadingMaxLinearSpeed = 0.2; // m/s
        @Getter final double grossHeadingMaxOmega = 0.1; // rad/s

        // -- Turret zero auto-correction --------------------------------------
        //
        // The turret has no absolute reference, so its zero is wherever it pointed at motor
        // power-on. The turret camera measures that error directly: its mount transform is built
        // from the turret encoder, so a steady disagreement between its MegaTag1 heading and the
        // (gyro-and-swerve-camera) pose heading is the encoder error itself. On 2026-09-05 that
        // sat at a rock-steady -9.6 deg for a hundred seconds while the swerve camera agreed with
        // the pose heading to 0.00 deg, and the shots missed by feet.

        /** Below this the error is noise, not slip; do not spend a CAN write on it. */
        @Getter final double turretZeroDeadbandDeg = 0.3;

        /** Refuse anything wilder than this; that size wants a human, not a servo. */
        @Getter final double turretZeroMaxErrorDeg = 45.0;

        /**
         * Fastest the trim may walk the encoder, in degrees per second.
         *
         * <p>The slip measured on 2026-09-05 ran about 0.15 deg/s while the turret was slewing
         * hard, so this has roughly twenty times the authority it needs to keep up. It is
         * deliberately not faster: this moves a turret that may be aimed at something.
         */
        @Getter final double turretZeroMaxTrimDegPerSec = 3.0;

        /**
         * Seconds between encoder writes.
         *
         * <p>Each correction is a {@code setPosition} call, and the CANivore already sits at 61 to
         * 80 percent utilisation. Five writes a second is enough to chase slip an order of
         * magnitude slower than the trim rate, and cheap enough not to matter.
         */
        @Getter final double turretZeroApplyPeriodSeconds = 0.2;

        /** Low-pass on the measurement, per sample. Slip is slow; single frames are not trusted. */
        @Getter final double turretZeroFilterAlpha = 0.1;

        /** Turret slew above which the mount transform lags enough to spoil the reading. */
        @Getter final double turretZeroMaxTurretOmega = 0.25; // rot/s

        /**
         * Cumulative correction per minute above which the mechanism, not the zero, is the fault.
         */
        @Getter final double turretSlipAlertDegPerMinute = 5.0;

        /**
         * Outstanding error that, if the trim cannot clear it, means the trim is making it worse.
         *
         * <p>This is the guard on the sign. Correcting the right way, a 25 deg error clears in
         * about 8 seconds of trim at 3 deg/s, so it cannot sit above this for {@link
         * #getTurretZeroDivergenceHoldSeconds()}. Correcting the wrong way it grows and stays,
         * which matters more than the aim: soft limits are enforced against the reported position,
         * so driving reported away from reality is exactly how the turret gets past a soft stop and
         * into the cable chain -- and at 3 deg/s it would do that twenty times faster than the slip
         * ever did.
         */
        @Getter final double turretZeroDivergenceDeg = 25.0;

        /** How long the error must stay unclearable before the trim gives up on itself. */
        @Getter final double turretZeroDivergenceHoldSeconds = 10.0;
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

    /**
     * Live turret angle in degrees, positive counter-clockwise, zero pointing robot-forward. Reads
     * the motor directly rather than the turret's per-loop cache: Vision runs before {@code
     * CommandScheduler.run()}, so the cache still holds last loop's value at that point.
     */
    private final DoubleSupplier turretRotationSupplier =
            () -> Robot.getTurret().getPositionDegreesUncached();

    private final VisionConfig config;

    /**
     * How often the IMU mode is re-sent to every camera. A Limelight that boots after the robot, or
     * reboots mid-session, comes up in mode 0 with no robot heading and produces garbage MegaTag2
     * poses; a one-time write at startup cannot recover from that, so the mode is republished on
     * this period. It is a single double write per camera.
     */
    private static final double IMU_MODE_RESEND_PERIOD_SECS = 2.0;

    private double lastImuModeSendFpgaSeconds = Double.NEGATIVE_INFINITY;

    // =========================================================================
    // Construction
    // =========================================================================

    /**
     * Creates the Vision subsystem.
     *
     * <p>Instantiates the Limelight and its logger and sets IMU mode 0 (external heading only,
     * correct for a mount whose frame rotates relative to the robot).
     *
     * <p>Deliberately NOT registered with the scheduler: {@link frc.robot.Robot#robotPeriodic()}
     * calls {@link #periodic()} explicitly before {@code CommandScheduler.run()} so this loop's
     * vision correction is in the pose before any mechanism computes a shot. Registering it as well
     * would run it twice per loop.
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

        for (Limelight limelight : allLimelights) {
            limelight.setLEDMode(false);
        }
        sendImuModes();

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
     * Called once per robot loop by {@link frc.robot.Robot#robotPeriodic()}, before {@code
     * CommandScheduler.run()}, so the mechanisms see this loop's vision-corrected pose.
     *
     * <ol>
     *   <li>Clears every camera's per-loop NetworkTables snapshot so this loop reads fresh data.
     *   <li>Publishes the live camera mount transform and the robot heading, then performs the
     *       loop's single NetworkTables flush (the orientation writes themselves do not flush).
     *   <li>Runs pose-estimation updates appropriate to the current robot mode.
     *   <li>Logs camera telemetry.
     * </ol>
     */
    @Override
    public void periodic() {
        for (Limelight limelight : allLimelights) {
            limelight.invalidate();
        }

        updateTurretCameraPose();
        setLimeLightOrientation();
        if (Timer.getFPGATimestamp() - lastImuModeSendFpgaSeconds >= IMU_MODE_RESEND_PERIOD_SECS) {
            sendImuModes();
        }

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
            logger.getMegaPose();
            logger.getTagCount();
            logger.getTargetSize();
            logger.getEstimateAge();
            logger.getIntegratedThisLoop();
        }
        Telemetry.log("Vision/TurretLL/HeadingErrorDeg", turretCameraHeadingErrorDeg(), "deg");
        correctTurretZero();

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
        Telemetry.log(
                "Vision/SecondsSinceAcceptedEstimate",
                secondsSinceLastAcceptedEstimate(),
                "seconds");
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
        // These writes do not flush; periodic() flushes once after all per-loop writes.
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
        Telemetry.log("Vision/TurretCameraPose", cameraPose);
    }

    /**
     * While the robot is disabled, seeds the pose estimator (translation and heading) from the best
     * chassis camera's MegaTag1 only, so the pose is correct before the match starts.
     *
     * <p>MegaTag2 is deliberately not used here. It depends on the heading we push to the camera,
     * which is the very thing seeding is trying to fix, and the turret camera's MT2 additionally
     * depends on the turret zero. Fusing them alongside a tight MT1 seed made the pose flip a metre
     * every loop when the turret zero was off (seen in the 2026-09-04 bench logs).
     */
    /**
     * True once a chassis camera has actually seeded the pose while disabled.
     *
     * <p>Heading is gyro-only while enabled, and this seeding is the only thing that ever sets it
     * from vision. Enable before it has happened and the robot's idea of which way it is facing is
     * just however it was sitting at power-on, for the whole enabled period -- and the turret aims
     * off by exactly that much.
     *
     * <p>Which is what kept happening. In all three 2026-09-05 test logs the robot was enabled
     * before any camera had produced a pose: by 4 s, by 13 s, and once by 74 s. The cameras were
     * still booting. Nothing said so, because a pose seeded from a bad heading looks exactly like
     * one seeded from a good heading.
     */
    @Getter private boolean poseHeadingSeeded = false;

    private final Alert notSeededAlert =
            new Alert(
                    "Pose heading has not been vision-seeded yet - wait for a limelight to see tags"
                            + " before enabling, or the turret will aim off by the robot's"
                            + " power-on heading error",
                    AlertType.kWarning);

    private void disabledLimelightUpdates() {
        if (Util.disabled.getAsBoolean()) {
            Limelight best = getBestLimelight();
            markUnselectedLimelights(best);
            integrateSingleEstimate(best, getMT1Estimate(best, true));
            if (best.isIntegratedThisLoop()) {
                poseHeadingSeeded = true;
            }
        }

        // Warn only while disabled: once the match is running, saying so does not help anyone and
        // the gross heading correction is the thing that has to save it.
        notSeededAlert.set(!poseHeadingSeeded && Util.disabled.getAsBoolean());
        Telemetry.log("Vision/PoseHeadingSeeded", poseHeadingSeeded);
    }

    /**
     * While the robot is enabled (teleop or auto pose-update), fuses the best chassis camera's MT1
     * translation and the turret camera's MT2 translation, then runs the gross-heading safety net.
     */
    private void enabledLimelightUpdates() {
        if (Util.teleop.getAsBoolean() || Auton.autonPoseUpdate.getAsBoolean()) {
            Limelight best = getBestLimelight();
            markUnselectedLimelights(best);
            integrateSingleEstimate(best, getMT1Estimate(best, false));

            if (turretEstimatesAvailable()) {
                integrateSingleEstimate(turretLL, getMT2VisionEstimate(turretLL));
            }

            checkGrossHeadingError(best);
        }
    }

    /** FPGA time at which the gross heading error was first seen; NaN when not armed. */
    private double grossHeadingErrorStartFpgaSeconds = Double.NaN;

    /**
     * Safety net for enabling before the cameras seeded the pose. If the robot is nearly stationary
     * and the best chassis camera sees two or more tags whose MegaTag1 heading disagrees with the
     * gyro by more than {@link VisionConfig#getGrossHeadingErrorDeg()} continuously for {@link
     * VisionConfig#getGrossHeadingHoldSeconds()}, the heading is reset to the camera's once.
     * Frame-to-frame MegaTag1 heading noise of a degree or two never trips this; a 90 or 180 degree
     * boot-heading error does, within about a second of stopping in front of tags.
     */
    private void checkGrossHeadingError(Limelight best) {
        double now = Timer.getFPGATimestamp();
        Pose2d robotPose = Robot.getSwerve().getRobotPose();
        ChassisSpeeds speeds = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        boolean stationary =
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                                <= config.getGrossHeadingMaxLinearSpeed()
                        && Math.abs(speeds.omegaRadiansPerSecond)
                                <= config.getGrossHeadingMaxOmega();

        double age = best.getLastEstimateAgeSeconds();
        boolean freshMultiTag =
                best.targetInView()
                        && best.multipleTagsInView()
                        && !Double.isNaN(age)
                        && age <= config.getMaxEstimateAgeSeconds();

        double errorDeg = Double.NaN;
        Pose2d mt1Pose = null;
        if (freshMultiTag) {
            mt1Pose = best.getMegaTag1_Pose3d().toPose2d();
            if (!FieldHelpers.poseOutOfField(mt1Pose)) {
                errorDeg = mt1Pose.getRotation().minus(robotPose.getRotation()).getDegrees();
            }
        }

        boolean gross =
                !Double.isNaN(errorDeg)
                        && stationary
                        && Math.abs(errorDeg) >= config.getGrossHeadingErrorDeg();
        boolean applied = false;
        if (!gross) {
            grossHeadingErrorStartFpgaSeconds = Double.NaN;
        } else if (Double.isNaN(grossHeadingErrorStartFpgaSeconds)) {
            grossHeadingErrorStartFpgaSeconds = now;
        } else if (now - grossHeadingErrorStartFpgaSeconds >= config.getGrossHeadingHoldSeconds()) {
            Robot.getSwerve()
                    .addVisionMeasurement(
                            mt1Pose,
                            Utils.fpgaToCurrentTime(best.getMegaTag1PoseTimestamp()),
                            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01)));
            grossHeadingErrorStartFpgaSeconds = Double.NaN;
            applied = true;
            Telemetry.print(
                    String.format(
                            "Vision: gross heading error of %.1f deg corrected from %s",
                            errorDeg, best.getName()));
        }

        Telemetry.log("Vision/HeadingCorrection/ErrorDeg", errorDeg, "deg");
        Telemetry.log("Vision/HeadingCorrection/Armed", gross);
        Telemetry.log("Vision/HeadingCorrection/Applied", applied);
    }

    private double turretZeroFilteredErrorDeg = Double.NaN;
    private double turretZeroLastApplySeconds = Double.NEGATIVE_INFINITY;
    private double turretZeroRateWindowStartSeconds = Double.NaN;
    private double turretZeroRateWindowDeg = 0;
    private double turretZeroRateWindowStartTravelDeg = 0;
    private double turretSlipDegPerMinute = 0;
    private double turretSlipDegPerKiloDegTravel = 0;
    private double turretZeroDivergenceStartSeconds = Double.NaN;
    private boolean turretZeroDiverged = false;

    private final Alert turretZeroDivergedAlert =
            new Alert(
                    "Turret zero trim disabled: correcting is not reducing the error, so the"
                            + " correction sign is likely wrong. Turret soft limits may no longer"
                            + " reflect where the turret actually is.",
                    AlertType.kError);

    private final Alert turretSlipAlert =
            new Alert(
                    "Turret is slipping: vision is having to re-zero it continuously. Check belt"
                            + " tension and the turret pinion.",
                    AlertType.kWarning);

    /**
     * Continuously walks the turret encoder toward what the turret camera says, correcting both a
     * bad power-on zero and ongoing mechanical slip.
     *
     * <p>{@link #turretCameraHeadingErrorDeg()} is actual turret angle minus reported. The camera's
     * mount transform is built from this very encoder, so a non-zero reading is the encoder being
     * wrong and nothing else -- confirmed on 2026-09-05, where the swerve cameras agreed with the
     * pose heading to within a couple of degrees all run while this read -37 deg.
     *
     * <p>This started life as a one-shot: measure, correct once, and give up if the error came
     * back, on the theory that an error which survives correction means the sign is wrong. That was
     * wrong for the actual fault. The turret slips, roughly 0.4 percent of every degree it travels,
     * so the error <em>always</em> comes back and the only useful response is to keep correcting.
     * Hence a rate-limited servo rather than a one-shot, and no give-up.
     *
     * <p>What still stops it: a launch, because moving the turret with fuel in the air is worse
     * than the miss it would fix; a slewing turret or a moving robot, because the reading is not
     * trustworthy then; and anything past {@link VisionConfig#getTurretZeroMaxErrorDeg()}, which is
     * likelier a bad frame than a real error.
     *
     * <p>The correction is bounded by {@link VisionConfig#getTurretZeroMaxTrimDegPerSec()}, so a
     * sign error walks the turret at a known, slow, visible rate instead of jumping it. The slip
     * rate is published, and past {@link VisionConfig#getTurretSlipAlertDegPerMinute()} it raises
     * an alert -- vision papering over a mechanical fault should be loud about it, not silent.
     */
    private void correctTurretZero() {
        double now = Timer.getFPGATimestamp();
        double errorDeg = turretCameraHeadingErrorDeg();

        ChassisSpeeds speeds = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        boolean stationary =
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                                <= config.getGrossHeadingMaxLinearSpeed()
                        && Math.abs(speeds.omegaRadiansPerSecond)
                                <= config.getGrossHeadingMaxOmega();

        boolean measurable =
                !Double.isNaN(errorDeg)
                        && Math.abs(errorDeg) <= config.getTurretZeroMaxErrorDeg()
                        && stationary
                        && Math.abs(Robot.getTurret().getMechOmegaRotPerSec())
                                <= config.getTurretZeroMaxTurretOmega();

        if (!measurable) {
            // Drop the filter rather than let it coast on stale samples into the next window.
            turretZeroFilteredErrorDeg = Double.NaN;
        } else if (Double.isNaN(turretZeroFilteredErrorDeg)) {
            turretZeroFilteredErrorDeg = errorDeg;
        } else {
            turretZeroFilteredErrorDeg +=
                    config.getTurretZeroFilterAlpha() * (errorDeg - turretZeroFilteredErrorDeg);
        }

        updateTurretSlipRate(now);

        boolean applicable =
                !turretZeroDiverged
                        && !Double.isNaN(turretZeroFilteredErrorDeg)
                        && Math.abs(turretZeroFilteredErrorDeg) >= config.getTurretZeroDeadbandDeg()
                        && !Robot.getSuperStructure().currentStateIsLaunching()
                        && now - turretZeroLastApplySeconds
                                >= config.getTurretZeroApplyPeriodSeconds();

        if (applicable) {
            double elapsed = Math.min(now - turretZeroLastApplySeconds, 1.0);
            double limit = config.getTurretZeroMaxTrimDegPerSec() * elapsed;
            double step = Math.max(-limit, Math.min(limit, turretZeroFilteredErrorDeg));

            Robot.getTurret().applyZeroCorrectionDegrees(step);
            turretZeroLastApplySeconds = now;
            turretZeroRateWindowDeg += Math.abs(step);

            // The encoder just moved by step, so the outstanding error did too. Without this the
            // filter would re-apply the same correction until fresh frames caught up.
            turretZeroFilteredErrorDeg -= step;
            Telemetry.log("Vision/TurretZero/LastStepDeg", step, "deg");

            // Trimming should be shrinking this. If it is not, the sign is wrong, and continuing
            // walks the reported position away from the real one -- which is what the soft limits
            // are checked against.
            if (Math.abs(turretZeroFilteredErrorDeg) >= config.getTurretZeroDivergenceDeg()) {
                if (Double.isNaN(turretZeroDivergenceStartSeconds)) {
                    turretZeroDivergenceStartSeconds = now;
                } else if (now - turretZeroDivergenceStartSeconds
                        >= config.getTurretZeroDivergenceHoldSeconds()) {
                    turretZeroDiverged = true;
                    turretZeroDivergedAlert.set(true);
                    Telemetry.print(
                            String.format(
                                    "Vision: turret zero trim disabled, %.1f deg of error would not"
                                            + " clear. Check the correction sign.",
                                    turretZeroFilteredErrorDeg));
                }
            } else {
                turretZeroDivergenceStartSeconds = Double.NaN;
            }
        }

        Telemetry.log("Vision/TurretZero/Measurable", measurable);
        Telemetry.log("Vision/TurretZero/FilteredErrorDeg", turretZeroFilteredErrorDeg, "deg");
        Telemetry.log("Vision/TurretZero/SlipDegPerMinute", turretSlipDegPerMinute, "deg");
        Telemetry.log(
                "Vision/TurretZero/SlipDegPerKiloDegTravel", turretSlipDegPerKiloDegTravel, "deg");
        Telemetry.log(
                "Vision/TurretZero/CorrectedTotalDeg",
                Robot.getTurret().getZeroCorrectionTotalDegrees(),
                "deg");
    }

    /**
     * Tracks how much correction the turret is absorbing per minute.
     *
     * <p>A healthy turret needs one correction after power-on and nothing more. A steady demand for
     * correction is the mechanism giving way, and the number is the useful part: it says how fast,
     * which says whether it is worth stopping for now or after the match.
     *
     * @param now current FPGA time in seconds
     */
    private void updateTurretSlipRate(double now) {
        double travel = Robot.getTurret().getTravelTotalDegrees();
        if (Double.isNaN(turretZeroRateWindowStartSeconds)) {
            turretZeroRateWindowStartSeconds = now;
            turretZeroRateWindowStartTravelDeg = travel;
            return;
        }

        double elapsed = now - turretZeroRateWindowStartSeconds;
        double travelled = travel - turretZeroRateWindowStartTravelDeg;

        // Published continuously once there is enough of a window to mean anything, rather than
        // once a minute. A match is barely two of those, and the first would read zero throughout
        // the part of it anyone is watching.
        if (elapsed >= SLIP_RATE_MIN_WINDOW_SECONDS) {
            turretSlipDegPerMinute = turretZeroRateWindowDeg * 60.0 / elapsed;
            turretSlipDegPerKiloDegTravel =
                    travelled > 1.0 ? turretZeroRateWindowDeg * 1000.0 / travelled : 0;
            turretSlipAlert.set(turretSlipDegPerMinute >= config.getTurretSlipAlertDegPerMinute());
        }

        if (elapsed >= SLIP_RATE_WINDOW_SECONDS) {
            turretZeroRateWindowStartSeconds = now;
            turretZeroRateWindowStartTravelDeg = travel;
            turretZeroRateWindowDeg = 0;
        }
    }

    /** Shortest window that gives a slip rate worth publishing. */
    private static final double SLIP_RATE_MIN_WINDOW_SECONDS = 5.0;

    /** How much history each slip rate covers before the window rolls. */
    private static final double SLIP_RATE_WINDOW_SECONDS = 60.0;

    /**
     * Turret camera MegaTag1 heading minus the pose heading, wrapped to [-180, 180) degrees, or NaN
     * when the turret camera has no target. Because the turret camera's mount transform comes from
     * the turret encoder, a steady non-zero value here is the turret zero error.
     */
    private double turretCameraHeadingErrorDeg() {
        if (!turretEstimatesAvailable() || !turretLL.targetInView()) {
            return Double.NaN;
        }
        Rotation2d cameraHeading = turretLL.getMegaTag1_Pose3d().toPose2d().getRotation();
        return cameraHeading.minus(Robot.getSwerve().getRobotPose().getRotation()).getDegrees();
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
     * <p>Accepted estimates are assigned a translation std-dev based on how many tags are visible
     * and how large the target appears; heading is always given a huge std-dev so the gyro owns
     * heading while enabled. {@code forceIntegrateXY} overrides both to near-zero, used during
     * disabled pre-seeding, which is where the field heading is established.
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

        // Select the translation std-dev based on confidence tier.
        //
        // Heading is never fused while enabled. The Pigeon drifts a small fraction of a degree
        // over a match, while MegaTag1 yaw jitters by a degree or more frame to frame. Fusing it
        // (the 2025 code used 0.1 deg here, which applies ~98% of the camera's heading every
        // frame) put that jitter straight into the turret setpoint, since turret angle is the
        // field bearing minus the robot heading. Heading is corrected only by the disabled
        // pre-seeding below and by the operator's manual pose reset.
        double xyStds;
        double degStds = config.getKLargeVariance();

        if (robotLinearSpeed <= 0.2 && targetSize > 4) {
            ll.sendValidStatus("Stationary close integration");
            xyStds = 0.1;
        } else if (multiTags && targetSize > 2) {
            ll.sendValidStatus("Strong Multi integration");
            xyStds = 0.1;
        } else if (multiTags && targetSize > 0.2) {
            ll.sendValidStatus("Multi integration");
            xyStds = 0.25;
        } else if (targetSize > 2 && mt1PoseDifference < 0.5) {
            ll.sendValidStatus("Close integration");
            xyStds = 0.5;
        } else if (targetSize > 1 && mt1PoseDifference < 0.25) {
            ll.sendValidStatus("Proximity integration");
            xyStds = 1.0;
        } else if (highestAmbiguity < 0.25 && targetSize >= 0.03) {
            ll.sendValidStatus("Stable integration");
            xyStds = 1.5;
        } else {
            ll.sendInvalidStatus("Integration Criteria not Met");
            return null;
        }

        // Override covariance for disabled pre-seeding: this is the one place vision sets heading,
        // so the gyro's arbitrary boot heading gets aligned to the field before the match.
        if (forceIntegrateXY) {
            xyStds = 0.01;
            degStds = 0.01;
        }

        Pose2d integratedPose =
                new Pose2d(megaTag1Pose2d.getTranslation(), megaTag1Pose2d.getRotation());
        double timestamp = Utils.fpgaToCurrentTime(ll.getMegaTag1PoseTimestamp());
        if (isStale(ll, timestamp)) {
            return null;
        }
        // The pose estimator expects the heading std-dev in radians; degStds is in degrees.
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
        int numTags = tags == null ? 1 : tags.length;

        return new VisionFieldPoseEstimate(integratedPose, timestamp, stdDevs, numTags);
    }

    /**
     * Records the estimate's age on the camera for telemetry and rejects it if it is older than
     * {@link VisionConfig#getMaxEstimateAgeSeconds()}. The pose estimator would drop such a
     * measurement silently; rejecting it here makes a camera with a bad clock visible in the logs.
     *
     * @param ll the camera the estimate came from
     * @param timestampCurrentTime the estimate's capture time in the pose estimator's time base
     * @return {@code true} if the estimate is too old to use
     */
    private boolean isStale(Limelight ll, double timestampCurrentTime) {
        double age = Utils.getCurrentTimeSeconds() - timestampCurrentTime;
        ll.setLastEstimateAgeSeconds(age);
        if (age > config.getMaxEstimateAgeSeconds()) {
            ll.sendInvalidStatus("Stale Estimate Rejection");
            return true;
        }
        return false;
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

        // Tag count intentionally comes from the MT1 estimate so the tiering matches MT1 exactly.
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
        double timestamp = Utils.fpgaToCurrentTime(ll.getMegaTag2PoseTimestamp());
        if (isStale(ll, timestamp)) {
            return null;
        }

        return new VisionFieldPoseEstimate(
                integratedPose,
                timestamp,
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
     * Adds a vision measurement to the swerve pose estimator if the estimate is non-null, and marks
     * the source camera as having been integrated this loop for telemetry.
     *
     * @param ll the camera the estimate came from
     * @param estimate the estimate to integrate, or {@code null} to skip
     */
    private void integrateSingleEstimate(Limelight ll, VisionFieldPoseEstimate estimate) {
        if (estimate != null) {
            Robot.getSwerve()
                    .addVisionMeasurement(
                            estimate.getVisionRobotPoseMeters(),
                            estimate.getTimestampSeconds(),
                            estimate.getVisionMeasurementStdDevs());
            lastAcceptedEstimateFpgaSeconds = Timer.getFPGATimestamp();
            ll.setIntegratedThisLoop(true);
        }
    }

    /** FPGA time of the most recent estimate fused into the estimator; NaN until the first. */
    private double lastAcceptedEstimateFpgaSeconds = Double.NaN;

    /**
     * Seconds since any camera's estimate was last fused into the pose estimator, or {@code
     * Double.POSITIVE_INFINITY} if none has been. A shot-readiness input: a large value means the
     * pose is running on odometry alone.
     */
    public double secondsSinceLastAcceptedEstimate() {
        if (Double.isNaN(lastAcceptedEstimateFpgaSeconds)) {
            return Double.POSITIVE_INFINITY;
        }
        return Timer.getFPGATimestamp() - lastAcceptedEstimateFpgaSeconds;
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

        // The turret camera's mount transform is built from the turret encoder. If its MegaTag1
        // heading disagrees with the gyro, the turret zero is off by that much and every pose it
        // reports is displaced by range times that angle, so do not fuse it. A 69 deg zero error
        // on the bench put its estimates a metre off while every status read "Multi integration".
        if (ll == turretLL) {
            double headingErrorDeg = turretCameraHeadingErrorDeg();
            if (!Double.isNaN(headingErrorDeg)
                    && Math.abs(headingErrorDeg) > config.getTurretHeadingMismatchDeg()) {
                ll.sendInvalidStatus("Turret Heading Mismatch Rejection");
                return true;
            }
        }

        return false;
    }

    /**
     * Publishes the IMU mode to every camera. Chassis cameras use mode 1 (internal IMU seeded by
     * the pushed robot heading); the turret camera uses mode 0 (external heading only) because its
     * frame yaws with the turret. Called at construction and then every {@link
     * #IMU_MODE_RESEND_PERIOD_SECS} from {@link #periodic()} so a camera that reboots picks the
     * mode back up instead of staying in its default mode 0.
     */
    private void sendImuModes() {
        for (Limelight limelight : swerveLimelights) {
            limelight.setIMUmode(1);
        }
        turretLL.setIMUmode(0);
        lastImuModeSendFpgaSeconds = Timer.getFPGATimestamp();
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
