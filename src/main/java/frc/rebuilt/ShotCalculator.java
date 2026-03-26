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

    // Offset from robot center to launcher center (leave zero if launcher is centered)
    private static final Transform2d robotToLauncher =
            new Transform2d(
                    new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
                    new Rotation2d());

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            boolean isValid,
            Rotation2d driveAngle,
            double driveAngularVelocity,
            double hoodAngle,
            double hoodVelocity,
            double flywheelSpeed,
            double distance,
            double distanceNoLookahead,
            double timeOfFlight) {}

    private ShootingParameters latestParameters = null;

    public static final double STARTING_HOOD_ANGLE_OFFSET = 0; // degrees
    public static double HOOD_ANGLE_OFFSET = STARTING_HOOD_ANGLE_OFFSET;

    public static final double STARTING_DRIVE_ANGLE_OFFSET = 0; // degrees
    public static double DRIVE_ANGLE_OFFSET = STARTING_DRIVE_ANGLE_OFFSET;

    public static void increaseHoodAngleOffset() {
        HOOD_ANGLE_OFFSET += 1;
    }

    public static void decreaseHoodAngleOffset() {
        HOOD_ANGLE_OFFSET -= 1;
    }

    public static void increaseDriveAngleOffset() {
        DRIVE_ANGLE_OFFSET += 1;
    }

    public static void decreaseDriveAngleOffset() {
        DRIVE_ANGLE_OFFSET -= 1;
    }

    // ===== Config / maps =====
    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;

    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
            new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
            new InterpolatingDoubleTreeMap();

    // ===== Velocity and angle calculation =====
    private static final double loopPeriodSecs = 0.02;

    private final LinearFilter hoodAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / loopPeriodSecs)); // ~100ms window

    private final LinearFilter driveAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / loopPeriodSecs)); // ~100ms window

    private double lastHoodAngle = Double.NaN;
    private Rotation2d lastDriveAngle = null;

    static {
        minDistance = 1.34;
        maxDistance = 5.60;
        phaseDelay = 0.03;

        // Hood angle map (in degrees)
        hoodAngleMap.put(1.34, 15.0);
        hoodAngleMap.put(2.00, 18.0);
        hoodAngleMap.put(2.35, 20.0);
        hoodAngleMap.put(2.65, 22.0);
        hoodAngleMap.put(2.96, 24.0);
        hoodAngleMap.put(3.23, 26.0);
        hoodAngleMap.put(3.65, 28.0);
        hoodAngleMap.put(4.00, 30.0);
        hoodAngleMap.put(4.20, 32.0);
        hoodAngleMap.put(4.50, 34.0);
        hoodAngleMap.put(5.60, 36.0);

        // Flywheel map (in RPM)

        // Into hub shots
        shotFlywheelSpeedMap.put(0.00, 1800.0);
        shotFlywheelSpeedMap.put(4.50, 1800.0);

        // Feeding shots
        shotFlywheelSpeedMap.put(4.51, 2300.0);
        shotFlywheelSpeedMap.put(6.00, 2400.0);

        // TOF map (in seconds)
        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);
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

        // Launcher pose + base distance
        Pose2d launcherPose = estimatedPose.transformBy(robotToLauncher);
        double launcherToTargetDistance = target.getDistance(launcherPose.getTranslation());
        double distanceNoLookahead = launcherToTargetDistance;

        // Field-relative velocity of robot
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        robotRelativeVelocity, estimatedPose.getRotation());

        // Launcher tangential velocity due to robot rotation about robot center
        double robotAngle = estimatedPose.getRotation().getRadians();
        double launcherVelocityX =
                fieldVelocity.vxMetersPerSecond
                        + fieldVelocity.omegaRadiansPerSecond
                                * (robotToLauncher.getY() * Math.cos(robotAngle)
                                        - robotToLauncher.getX() * Math.sin(robotAngle));
        double launcherVelocityY =
                fieldVelocity.vyMetersPerSecond
                        + fieldVelocity.omegaRadiansPerSecond
                                * (robotToLauncher.getX() * Math.cos(robotAngle)
                                        - robotToLauncher.getY() * Math.sin(robotAngle));

        // Lookahead iteration: converge distance
        double lookaheadDistance = launcherToTargetDistance;
        for (int i = 0; i < 20; i++) {
            double tof = timeOfFlightMap.get(lookaheadDistance);
            double offsetX = launcherVelocityX * tof;
            double offsetY = launcherVelocityY * tof;

            Translation2d lookaheadLauncherTranslation =
                    launcherPose.getTranslation().plus(new Translation2d(offsetX, offsetY));

            double newDistance = target.getDistance(lookaheadLauncherTranslation);
            if (Math.abs(newDistance - lookaheadDistance) < 0.01) {
                lookaheadDistance = newDistance;
                break;
            }
            lookaheadDistance = newDistance;
        }

        // Final compensated launcher translation using final TOF
        double tofFinal = timeOfFlightMap.get(lookaheadDistance);
        Translation2d compensatedLauncherTranslation =
                launcherPose
                        .getTranslation()
                        .plus(
                                new Translation2d(
                                        launcherVelocityX * tofFinal,
                                        launcherVelocityY * tofFinal));

        // Commanded drive angle (robot angle to aim launcher at target)
        Rotation2d driveAngle = target.minus(compensatedLauncherTranslation).getAngle();
        driveAngle = driveAngle.plus(Rotation2d.fromDegrees(DRIVE_ANGLE_OFFSET));

        // Drive angular velocity (rad/s) for your position controller feedforward
        if (lastDriveAngle == null) lastDriveAngle = driveAngle;
        double deltaRot =
                MathUtil.inputModulus(driveAngle.minus(lastDriveAngle).getRotations(), -0.5, 0.5);

        double rawDriveOmega = deltaRot / loopPeriodSecs;
        double driveAngularVelocity = driveAngleFilter.calculate(rawDriveOmega);
        lastDriveAngle = driveAngle;

        // Hood angle from map with offset
        double hoodAngle = hoodAngleMap.get(lookaheadDistance);
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
        double hoodVelocity =
                hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / loopPeriodSecs);
        lastHoodAngle = hoodAngle;

        // Apply hood angle offset
        hoodAngle += HOOD_ANGLE_OFFSET;

        // Flywheel from map + preference offset (%)
        double flywheelSpeed = shotFlywheelSpeedMap.get(lookaheadDistance);

        boolean isValid = lookaheadDistance >= minDistance && lookaheadDistance <= maxDistance;

        latestParameters =
                new ShootingParameters(
                        isValid,
                        driveAngle,
                        driveAngularVelocity,
                        hoodAngle,
                        hoodVelocity,
                        flywheelSpeed,
                        lookaheadDistance,
                        distanceNoLookahead,
                        tofFinal);

        Telemetry.log("ShotCalc/DistanceMeters", lookaheadDistance);
        Telemetry.log("ShotCalc/DriveAngleDeg", driveAngle.getDegrees());
        Telemetry.log("ShotCalc/HoodAngleDeg", hoodAngle);
        Telemetry.log("ShotCalc/FlywheelSpeedRPM", flywheelSpeed);
        Telemetry.log("ShotCalc/DriveAngleOffsetDegrees", DRIVE_ANGLE_OFFSET);
        Telemetry.log("ShotCalc/HoodAngleOffsetDegrees", HOOD_ANGLE_OFFSET);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
