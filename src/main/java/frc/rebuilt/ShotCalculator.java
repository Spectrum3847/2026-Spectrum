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
import frc.robot.RobotStates;
import frc.spectrumLib.Telemetry;

public class ShotCalculator {
    private static ShotCalculator instance;

    // Offset from robot center to turret center (leave zero if turret is centered)
    private static final Transform2d robotToTurret =
            new Transform2d(
                    new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
                    new Rotation2d());

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            boolean isValid,
            Rotation2d fieldAngle,
            double turretAngularVelocityRotPerSec,
            double flywheelSpeed) {}

    private ShootingParameters latestParameters = null;

    public static final double STARTING_FLYWHEEL_SPEED_OFFSET = 0; // percent
    public static double FLYWHEEL_SPEED_OFFSET = STARTING_FLYWHEEL_SPEED_OFFSET;

    public static final double STARTING_FIELD_ANGLE_OFFSET_DEGREES = 0;
    public static double FIELD_ANGLE_OFFSET_DEGREES = STARTING_FIELD_ANGLE_OFFSET_DEGREES;

    public static void increaseFlywheelSpeedOffset() {
        FLYWHEEL_SPEED_OFFSET += 1;
    }

    public static void decreaseFlywheelSpeedOffset() {
        FLYWHEEL_SPEED_OFFSET -= 1;
    }

    public static void increaseTurretAngleOffsetDegrees() {
        FIELD_ANGLE_OFFSET_DEGREES += 1;
    }

    public static void decreaseTurretAngleOffsetDegrees() {
        FIELD_ANGLE_OFFSET_DEGREES -= 1;
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
        shotFlywheelSpeedMap.put(2.00, 1700.0);
        shotFlywheelSpeedMap.put(2.35, 1825.0);
        shotFlywheelSpeedMap.put(2.65, 1850.0);
        shotFlywheelSpeedMap.put(2.96, 1910.0);
        shotFlywheelSpeedMap.put(3.23, 1950.0);
        shotFlywheelSpeedMap.put(3.65, 2000.0);
        shotFlywheelSpeedMap.put(4.00, 2100.0);
        shotFlywheelSpeedMap.put(4.20, 2175.0);
        shotFlywheelSpeedMap.put(4.50, 2300.0);

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
        boolean feed =
                RobotStates.robotInFeedZone.getAsBoolean()
                        && !RobotStates.forceScore.getAsBoolean();
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

        // Turret pose + base distance
        Pose2d turretPose = estimatedPose.transformBy(robotToTurret);
        double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

        // Field-relative velocity of robot
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        robotRelativeVelocity, estimatedPose.getRotation());

        // Turret tangential velocity due to robot rotation about robot center
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
        Rotation2d fieldAngle = target.minus(compensatedTurretTranslation).getAngle();
        fieldAngle = fieldAngle.plus(Rotation2d.fromDegrees(FIELD_ANGLE_OFFSET_DEGREES));

        // Turret angular velocity (rot/s) for your position controller feedforward
        if (lastTurretAngle == null) lastTurretAngle = fieldAngle;
        double deltaRot =
                MathUtil.inputModulus(fieldAngle.minus(lastTurretAngle).getRotations(), -0.5, 0.5);

        double rawOmega = deltaRot / loopPeriodSecs;
        double turretAngularVelocityRotPerSec = turretOmegaFilter.calculate(rawOmega);
        lastTurretAngle = fieldAngle;

        // Flywheel from map + preference offset (%)
        double flywheelSpeed = shotFlywheelSpeedMap.get(lookaheadDistance);
        flywheelSpeed += flywheelSpeed * (FLYWHEEL_SPEED_OFFSET / 100.0);

        boolean isValid = lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance;

        latestParameters =
                new ShootingParameters(
                        isValid, fieldAngle, turretAngularVelocityRotPerSec, flywheelSpeed);

        Telemetry.log("ShotCalc/DistanceMeters", lookaheadDistance);
        Telemetry.log("ShotCalc/FieldAngleDeg", fieldAngle.getDegrees());
        Telemetry.log("ShotCalc/FlywheelSpeedRPM", flywheelSpeed);
        Telemetry.log("ShotCalc/FlywheelSpeedOffset", FLYWHEEL_SPEED_OFFSET);
        Telemetry.log("ShotCalc/TurretAngleOffsetDegrees", FIELD_ANGLE_OFFSET_DEGREES);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
