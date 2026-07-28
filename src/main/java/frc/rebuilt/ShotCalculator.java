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
import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.rebuilt.targetFactories.HubTargetFactory;
import frc.robot.Robot;
import frc.spectrumLib.telemetry.*;
import java.text.DecimalFormat;

public class ShotCalculator {
    private static ShotCalculator instance;

    // Offset from robot center to turret center (leave zero if turret is centered)
    private static final Transform2d robotToTurret = Transform2d.kZero;

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            boolean isValid,
            Rotation2d turretAngle,
            double turretAngularVelocity,
            double hoodAngle,
            double hoodVelocity,
            double launcherSpeed) {}

    private ShootingParameters latestParameters = null;

    private static final DecimalFormat df = new DecimalFormat("0.00");

    public static final double STARTING_LAUNCHER_SPEED_OFFSET = 0; // percent
    public static double LAUNCHER_SPEED_OFFSET = STARTING_LAUNCHER_SPEED_OFFSET;

    public static final double STARTING_TURRET_ANGLE_OFFSET = 0;
    public static double TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;

    public static final double STARTING_HOOD_ANGLE_OFFSET = 0;
    public static double HOOD_ANGLE_OFFSET = STARTING_HOOD_ANGLE_OFFSET;

    public static void increaseHoodAngleOffset() {
        HOOD_ANGLE_OFFSET += 1;
    }

    public static void decreaseHoodAngleOffset() {
        HOOD_ANGLE_OFFSET -= 1;
    }

    public static void increaseTurretAngleOffset() {
        TURRET_ANGLE_OFFSET += 1;
    }

    public static void decreaseTurretAngleOffset() {
        TURRET_ANGLE_OFFSET -= 1;
    }

    // ===== Config / maps =====
    private static double minDistance;
    private static double maxDistance;
    private static double PHASE_DELAY_SECS;

    private static final InterpolatingDoubleTreeMap shotLauncherSpeedMap =
            new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
            new InterpolatingDoubleTreeMap();

    // ===== Turret angular velocity calculation =====
    // If you have a known loop period constant, swap it in here.
    // WPILib TimedRobot default is 0.02s, but use your actual period.
    private static final double LOOP_PERIOD_SECS = 0.02;

    private final LinearFilter turretOmegaFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100ms window

    private final LinearFilter hoodAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100 ms window

    private Rotation2d lastTurretAngle = null;
    private double lastHoodAngle = Double.NaN;

    static {
        minDistance = 1.34;
        maxDistance = 5.60;

        PHASE_DELAY_SECS = 0.03;

        // TODO: tune
        // Launcher map
        shotLauncherSpeedMap.put(1.50, 2250.0 + 100);
        shotLauncherSpeedMap.put(1.78, 2300.0 + 100);
        shotLauncherSpeedMap.put(2.00, 2450.0 + 100);
        shotLauncherSpeedMap.put(2.35, 2600.0 + 100);
        shotLauncherSpeedMap.put(2.56, 2650.0 + 100);
        shotLauncherSpeedMap.put(2.96, 2750.0 + 100);
        shotLauncherSpeedMap.put(3.16, 2900.0 + 100);
        shotLauncherSpeedMap.put(3.50, 3200.0 + 100);
        shotLauncherSpeedMap.put(4.00, 3300.0 + 100);
        shotLauncherSpeedMap.put(4.20, 3650.0 + 100);
        shotLauncherSpeedMap.put(5.00, 4000.0 + 100);

        hoodAngleMap.put(1.50, 20.0);

        // TOF map
        timeOfFlightMap.put(3.41, 1.10);
        timeOfFlightMap.put(3.08, 1.07);
        timeOfFlightMap.put(2.75, 1.05);
        timeOfFlightMap.put(2.33, 0.95);
        timeOfFlightMap.put(2.03, 0.85);
        timeOfFlightMap.put(1.68, 0.76);
    }

    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        // Target selection
        boolean feed = Robot.getSuperStructure().isRobotInFeedZone();
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();

        // Estimated pose w/ phase delay
        Pose2d estimatedPose = Robot.getSwerve().getRobotPose();
        ChassisSpeeds robotRelativeVelocity = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        estimatedPose =
                estimatedPose.exp(
                        new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY_SECS,
                                robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY_SECS,
                                robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY_SECS));

        // Turret pose + base distance
        Pose2d turretPose = estimatedPose.transformBy(robotToTurret);
        double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

        // Field-relative velocity of robot
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        robotRelativeVelocity, estimatedPose.getRotation());

        // Turret and hood tangential velocity due to robot rotation about robot center
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
        turretAngle = turretAngle.plus(Rotation2d.fromDegrees(TURRET_ANGLE_OFFSET));

        // Turret angular velocity (rot/s) for your position controller feedforward
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        double deltaRotTurret =
                MathUtil.inputModulus(turretAngle.minus(lastTurretAngle).getRotations(), -0.5, 0.5);

        double rawOmegaTurret = deltaRotTurret / LOOP_PERIOD_SECS;
        double turretAngularVelocity = turretOmegaFilter.calculate(rawOmegaTurret);
        lastTurretAngle = turretAngle;

        double rawHoodAngle = hoodAngleMap.get(lookaheadDistance);

        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = rawHoodAngle;
        double hoodVelocity =
                hoodAngleFilter.calculate((rawHoodAngle - lastHoodAngle) / LOOP_PERIOD_SECS);
        lastHoodAngle = rawHoodAngle;
        double hoodAngle = Math.max(rawHoodAngle + HOOD_ANGLE_OFFSET, 9);

        // Launcher from map + preference offset (%)
        double launcherSpeed = shotLauncherSpeedMap.get(lookaheadDistance);
        launcherSpeed += launcherSpeed * (LAUNCHER_SPEED_OFFSET / 100.0);

        boolean isValid = lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance;

        latestParameters =
                new ShootingParameters(
                        isValid,
                        turretAngle,
                        turretAngularVelocity,
                        hoodAngle,
                        hoodVelocity,
                        launcherSpeed);

        Telemetry.log("ShotCalc/IsValid", isValid);
        Telemetry.log("ShotCalc/DistanceMeters", df.format(lookaheadDistance));
        Telemetry.log("ShotCalc/TurretAngleDeg", df.format(turretAngle.getDegrees()));
        Telemetry.log("ShotCalc/TurretOmegaRotPerSec", df.format(turretAngularVelocity));
        Telemetry.log("ShotCalc/LauncherSpeedRPM", df.format(launcherSpeed));
        Telemetry.log("ShotCalc/TurretPose", turretPose);
        Telemetry.log("ShotCalc/LookaheadPose", compensatedTurretTranslation);
        Telemetry.log(
                "ShotCalc/TargetPose", new Pose2d(target.getX(), target.getY(), new Rotation2d()));
        Telemetry.log("ShotCalc/LauncherSpeedOffset", LAUNCHER_SPEED_OFFSET);
        Telemetry.log("ShotCalc/TurretAngleOffsetDegrees", TURRET_ANGLE_OFFSET);
        Telemetry.log("ShotCalc/HoodAngleOffset", HOOD_ANGLE_OFFSET);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
