package frc.rebuilt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.rebuilt.targetFactories.HubTargetFactory;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStructure;
import frc.spectrumLib.telemetry.Telemetry;
import java.text.DecimalFormat;

public class ShotCalculator {
    private static ShotCalculator instance;
    private final SuperStructure superStructure = Robot.getSuperStructure();

    // Offset from robot center to turret center (leave zero if turret is centered)
    private static final Transform2d robotToTurret =
            new Transform2d(
                    new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(5.0)),
                    new Rotation2d());

    // Height of the turret's rotation axis above the point the chassis actually tips about
    // (roughly bumper/floor contact height, NOT necessarily the robot's CG). The taller this
    // is, the more a given pitch/roll angle shifts the turret horizontally. Measure this on
    // the real robot and tune it - it matters a lot more than it looks like it should.
    private static final double turretPivotHeightMeters = Units.inchesToMeters(24.0);

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            boolean isValid,
            Rotation2d turretAngle,
            double turretAngularVelocityRotPerSec,
            double flywheelSpeed) {}

    private ShootingParameters latestParameters = null;

    private static final DecimalFormat df = new DecimalFormat("0.00");

    public static final double STARTING_FLYWHEEL_SPEED_OFFSET = 0; // percent
    public static double FLYWHEEL_SPEED_OFFSET = STARTING_FLYWHEEL_SPEED_OFFSET;

    public static final double STARTING_TURRET_ANGLE_OFFSET_DEGREES = 0;
    public static double TURRET_ANGLE_OFFSET_DEGREES = STARTING_TURRET_ANGLE_OFFSET_DEGREES;

    // RPM subtracted per m/s of closing speed toward the target (moving in needs less RPM, moving
    // out needs more). Start at 0 for distance-only behavior and tune on-robot.
    public static final double RADIAL_RPM_PER_MPS = 0.0;

    public static void increaseFlywheelSpeedOffset() {
        FLYWHEEL_SPEED_OFFSET += 1;
    }

    public static void decreaseFlywheelSpeedOffset() {
        FLYWHEEL_SPEED_OFFSET -= 1;
    }

    public static void increaseTurretAngleOffsetDegrees() {
        TURRET_ANGLE_OFFSET_DEGREES += 1;
    }

    public static void decreaseTurretAngleOffsetDegrees() {
        TURRET_ANGLE_OFFSET_DEGREES -= 1;
    }

    // ===== Config / maps =====
    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;

    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
            new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
            new InterpolatingDoubleTreeMap();

    // ===== Turret angular velocity calculation =====
    // If you have a known loop period constant, swap it in here.
    // WPILib TimedRobot default is 0.02s, but use your actual period.
    private static final double loopPeriodSecs = 0.02;

    private final LinearFilter turretOmegaFilter =
            LinearFilter.movingAverage((int) (0.1 / loopPeriodSecs)); // ~100ms window

    private Rotation2d lastTurretAngle = null;

    static {
        minDistance = 1.34;
        maxDistance = 5.60;

        phaseDelay = 0.03;

        // Flywheel map
        shotFlywheelSpeedMap.put(1.50, 2250.0 + 100);
        shotFlywheelSpeedMap.put(1.78, 2300.0 + 100);
        shotFlywheelSpeedMap.put(2.00, 2450.0 + 100);
        shotFlywheelSpeedMap.put(2.35, 2600.0 + 100);
        shotFlywheelSpeedMap.put(2.56, 2650.0 + 100);
        shotFlywheelSpeedMap.put(2.96, 2750.0 + 100);
        shotFlywheelSpeedMap.put(3.16, 2900.0 + 100);
        shotFlywheelSpeedMap.put(3.50, 3200.0 + 100);
        shotFlywheelSpeedMap.put(4.00, 3300.0 + 100);
        shotFlywheelSpeedMap.put(4.20, 3650.0 + 100);
        shotFlywheelSpeedMap.put(5.00, 4000.0 + 100);

        // TOF map
        timeOfFlightMap.put(3.41, 1.10);
        timeOfFlightMap.put(3.08, 1.07);
        timeOfFlightMap.put(2.75, 1.05);
        timeOfFlightMap.put(2.33, 0.95);
        timeOfFlightMap.put(2.03, 0.85);
        timeOfFlightMap.put(1.68, 0.76);
    }

    /**
     * Rotates the robot-frame turret offset (dx forward, dy left, dz up) by the full 3D orientation
     * of the chassis (roll about X, then pitch about Y, then yaw about Z - standard body-to-world
     * Tait-Bryan order) and returns the resulting horizontal (field X/Y) shift of the turret from
     * the robot's origin.
     *
     * <p>This is what actually matters when the robot is tilted (stuck on a bump / game piece): a
     * turret offset from center does NOT stay where the flat-ground math assumes it does once the
     * chassis is tipped - it swings out in 3D, and only its horizontal projection is relevant for
     * aiming.
     */
    private static Translation2d computeTiltCompensatedTurretOffset(
            Rotation2d yaw, Rotation2d pitch, Rotation2d roll) {
        double dx = robotToTurret.getTranslation().getX();
        double dy = robotToTurret.getTranslation().getY();
        double dz = turretPivotHeightMeters;

        // Roll about robot X axis (forward axis) - side-to-side tip
        double cr = roll.getCos();
        double sr = roll.getSin();
        double x1 = dx;
        double y1 = dy * cr - dz * sr;
        double z1 = dy * sr + dz * cr;

        // Pitch about robot Y axis (left axis) - front-to-back tip
        double cp = pitch.getCos();
        double sp = pitch.getSin();
        double x2 = x1 * cp + z1 * sp;
        double y2 = y1;
        // double z2 = -x1 * sp + z1 * cp; // resulting height offset - not used for aiming yet,
        // but here if you later want to correct flywheel/ToF for launch-height changes too.

        // Yaw about field Z axis - normal heading rotation
        double cy = yaw.getCos();
        double sy = yaw.getSin();
        double fieldDx = x2 * cy - y2 * sy;
        double fieldDy = x2 * sy + y2 * cy;

        return new Translation2d(fieldDx, fieldDy);
    }

    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        // Target selection: use feed target in the feed zone, but only when not actively launching
        SuperStructure.CurrentSuperState state = Robot.getSuperStructure().getCurrentSuperState();
        boolean feed =
                superStructure.isRobotInFeedZone()
                        && state != SuperStructure.CurrentSuperState.LAUNCH_WITH_SQUEEZE
                        && state != SuperStructure.CurrentSuperState.LAUNCH_WITHOUT_SQUEEZE
                        && state != SuperStructure.CurrentSuperState.LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY;
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();

        // Estimated pose w/ phase delay
        Pose2d estimatedPose = Robot.getSwerve().getRobotPose();
        ChassisSpeeds robotRelativeVelocity = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        estimatedPose =
                estimatedPose.exp(
                        new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Chassis tilt (pitch about Y, roll about X) from the gyro. These MUST come from your
        // real IMU (Pigeon2 / NavX) - wire getPitch()/getRoll() on your swerve subsystem if they
        // don't exist yet. If your gyro reports degrees, wrap with Rotation2d.fromDegrees(...).
        // TODO: test
        Rotation2d robotPitch = Robot.getSwerve().getPitch();
        Rotation2d robotRoll = Robot.getSwerve().getRoll();

        // Turret pose + base distance, now tilt-compensated
        Translation2d tiltCompensatedTurretOffset =
                computeTiltCompensatedTurretOffset(
                        estimatedPose.getRotation(), robotPitch, robotRoll);
        Translation2d turretFieldTranslation =
                estimatedPose.getTranslation().plus(tiltCompensatedTurretOffset);
        Pose2d turretPose = new Pose2d(turretFieldTranslation, estimatedPose.getRotation());
        double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

        // Field-relative velocity of robot
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        robotRelativeVelocity, estimatedPose.getRotation());

        // Turret tangential velocity due to robot rotation about robot center
        // (Left as a flat-ground approximation - the extra 3D component from
        // pitch/roll rate is a fast transient and negligible for a robot that's
        // stuck/tilted, as opposed to actively tipping.)
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX =
                fieldVelocity.vxMetersPerSecond
                        + fieldVelocity.omegaRadiansPerSecond
                                * (robotToTurret.getY() * Math.cos(robotAngle)
                                        - robotToTurret.getX() * Math.sin(robotAngle));
        double turretVelocityY = 
                fieldVelocity.vyMetersPerSecond
                        + fieldVelocity.omegaRadiansPerSecond
                                * (robotToTurret.getX() * Math.cos(robotAngle)
                                        - robotToTurret.getY() * Math.sin(robotAngle));

        // Lookahead iteration: converge distance
        double lookaheadDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            double tof = timeOfFlightMap.get(lookaheadDistance);
            double offsetX = turretVelocityX * tof;
            double offsetY = turretVelocityY * tof;

            Translation2d lookaheadTurretTranslation =
                    turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY));

            double newDistance = target.getDistance(lookaheadTurretTranslation);
            if (Math.abs(newDistance - lookaheadDistance) < 0.01) {
                lookaheadDistance = newDistance;
                break;
            }
            lookaheadDistance = newDistance;
        }

        // Final compensated turret translation using final TOF
        double tofFinal = timeOfFlightMap.get(lookaheadDistance);
        Translation2d compensatedTurretTranslation =
                turretPose
                        .getTranslation()
                        .plus(
                                new Translation2d(
                                        turretVelocityX * tofFinal, turretVelocityY * tofFinal));

        // Commanded turret angle (with preference offset)
        Rotation2d turretAngle = target.minus(compensatedTurretTranslation).getAngle();
        turretAngle = turretAngle.plus(Rotation2d.fromDegrees(TURRET_ANGLE_OFFSET_DEGREES));

        // Turret angular velocity (rot/s) for your position controller feedforward
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        double deltaRot =
                MathUtil.inputModulus(turretAngle.minus(lastTurretAngle).getRotations(), -0.5, 0.5);

        double rawOmega = deltaRot / loopPeriodSecs;
        double turretAngularVelocityRotPerSec = turretOmegaFilter.calculate(rawOmega);
        lastTurretAngle = turretAngle;

        // Closing speed of the turret toward the target (positive = approaching).
        double toTargetX = target.getX() - compensatedTurretTranslation.getX();
        double toTargetY = target.getY() - compensatedTurretTranslation.getY();
        double toTargetNorm = Math.hypot(toTargetX, toTargetY);
        double radialClosingSpeed = 0.0;
        if (toTargetNorm > 1e-6) {
            radialClosingSpeed =
                    (turretVelocityX * toTargetX + turretVelocityY * toTargetY) / toTargetNorm;
        }

        // Flywheel from map + preference offset (%) + radial-velocity compensation
        double flywheelSpeed = shotFlywheelSpeedMap.get(lookaheadDistance);
        flywheelSpeed += flywheelSpeed * (FLYWHEEL_SPEED_OFFSET / 100.0);
        flywheelSpeed -= RADIAL_RPM_PER_MPS * radialClosingSpeed;

        boolean isValid = lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance;

        latestParameters =
                new ShootingParameters(
                        isValid, turretAngle, turretAngularVelocityRotPerSec, flywheelSpeed);

        Telemetry.log("ShotCalc/IsValid", isValid);
        Telemetry.log("ShotCalc/DistanceMeters", df.format(lookaheadDistance));
        Telemetry.log("ShotCalc/TurretAngleDeg", df.format(turretAngle.getDegrees()));
        Telemetry.log("ShotCalc/TurretOmegaRadPerSec", df.format(turretAngularVelocityRotPerSec));
        Telemetry.log("ShotCalc/FlywheelSpeedRPM", df.format(flywheelSpeed));
        Telemetry.log("ShotCalc/RadialClosingSpeedMPS", df.format(radialClosingSpeed));
        Telemetry.log("ShotCalc/TurretPose", turretPose);
        Telemetry.log("ShotCalc/LookaheadPose", compensatedTurretTranslation);
        Telemetry.log(
                "ShotCalc/TargetPose", new Pose2d(target.getX(), target.getY(), new Rotation2d()));
        Telemetry.log("ShotCalc/FlywheelSpeedOffset", FLYWHEEL_SPEED_OFFSET);
        Telemetry.log("ShotCalc/TurretAngleOffsetDegrees", TURRET_ANGLE_OFFSET_DEGREES);
        Telemetry.log("ShotCalc/RobotPitchDeg", df.format(robotPitch.getDegrees()));
        Telemetry.log("ShotCalc/RobotRollDeg", df.format(robotRoll.getDegrees()));
        Telemetry.log(
                "ShotCalc/TiltTurretOffsetMeters",
                df.format(tiltCompensatedTurretOffset.getNorm()));

        return latestParameters;
    }

    /** Returns the last computed parameters without recalculating. */
    public ShootingParameters getLatestParameters() {
        return latestParameters;
    }

    /** Clears the cached parameters so the next getParameters() call recomputes. */
    public void clearShootingParameters() {
        latestParameters = null;
    }
}
