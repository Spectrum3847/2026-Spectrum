package frc.rebuilt;

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
import frc.spectrumLib.util.Conversions;
import java.text.DecimalFormat;

public class ShotCalculator {
    private static ShotCalculator instance;

    // Offset from robot center to turret center (leave zero if turret is centered)
    private static final Transform2d robotToTurret =
            new Transform2d(
                    new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(5.0)),
                    new Rotation2d());

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public record ShootingParameters(
            Rotation2d turretAngle, double visionTurretOffset, double flywheelSpeed) {}

    private ShootingParameters latestParameters = null;

    private static final DecimalFormat df = new DecimalFormat("0.00");

    private static double phaseDelay;

    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
            new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap timeOfFlightMap =
            new InterpolatingDoubleTreeMap();

    public static final double STARTING_FLYWHEEL_SPEED_OFFSET = 0;
    public static double FLYWHEEL_SPEED_OFFSET = STARTING_FLYWHEEL_SPEED_OFFSET;

    public static final double STARTING_TURRET_ANGLE_OFFSET_DEGREES = 0;
    public static double TURRET_ANGLE_OFFSET_DEGREES = STARTING_TURRET_ANGLE_OFFSET_DEGREES;

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

    static {
        phaseDelay = 0.03;

        shotFlywheelSpeedMap.put(1.5, Conversions.RPStoRPM(30.5 + 5));
        shotFlywheelSpeedMap.put(1.78, Conversions.RPStoRPM(31.0 + 5));
        shotFlywheelSpeedMap.put(2.00, Conversions.RPStoRPM(33.0 + 5));
        shotFlywheelSpeedMap.put(2.35, Conversions.RPStoRPM(34.5 + 5));
        shotFlywheelSpeedMap.put(2.56, Conversions.RPStoRPM(35.5 + 5));
        shotFlywheelSpeedMap.put(2.96, Conversions.RPStoRPM(36.0 + 5));

        timeOfFlightMap.put(5.68, 1.16);
        timeOfFlightMap.put(4.55, 1.12);
        timeOfFlightMap.put(3.15, 1.11);
        timeOfFlightMap.put(1.88, 1.09);
        timeOfFlightMap.put(1.38, 0.90);
    }

    public ShootingParameters getParameters() {
        if (latestParameters != null) {
            return latestParameters;
        }

        // Target location on the field
        boolean feed =
                RobotStates.robotInFeedZone.getAsBoolean()
                        && !RobotStates.forceScore.getAsBoolean();
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();

        // Calculate estimated pose while accounting for phase delay
        Pose2d robotPose = Robot.getSwerve().getRobotPose();
        ChassisSpeeds robotRelativeVelocity = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        robotPose =
                robotPose.exp(
                        new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        Pose2d turretPosition = robotPose.transformBy(robotToTurret);
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        robotRelativeVelocity, Robot.getSwerve().getRobotPose().getRotation());
        double robotAngle = robotPose.getRotation().getRadians();
        double turretVelocityX =
                robotVelocity.vxMetersPerSecond
                        + robotVelocity.omegaRadiansPerSecond
                                * (robotToTurret.getY() * Math.cos(robotAngle)
                                        - robotToTurret.getX() * Math.sin(robotAngle));
        double turretVelocityY =
                robotVelocity.vyMetersPerSecond
                        + robotVelocity.omegaRadiansPerSecond
                                * (robotToTurret.getX() * Math.cos(robotAngle)
                                        - robotToTurret.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (turret) to offset
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        double lastDistance;
        for (int i = 0; i < 5; i++) {
            lastDistance = lookaheadTurretToTargetDistance;

            double tof = timeOfFlightMap.get(lastDistance);
            double offsetX = turretVelocityX * tof;
            double offsetY = turretVelocityY * tof;

            lookaheadTurretToTargetDistance =
                    target.getDistance(
                            turretPosition
                                    .getTranslation()
                                    .plus(new Translation2d(offsetX, offsetY)));

            if (Math.abs(lookaheadTurretToTargetDistance - lastDistance) < 0.01) break;
        }
        // Calculate parameters accounted for imparted velocity
        Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        double flywheelSpeed = shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance);
        turretAngle = turretAngle.plus(Rotation2d.fromDegrees(TURRET_ANGLE_OFFSET_DEGREES));
        flywheelSpeed += flywheelSpeed * (FLYWHEEL_SPEED_OFFSET / 100.0);

        double visionTurretOffset = Robot.getVision().getTurretLL().getHorizontalOffset();

        latestParameters = new ShootingParameters(turretAngle, visionTurretOffset, flywheelSpeed);

        Telemetry.log("ShotCalc/DistanceMeters", df.format(lookaheadTurretToTargetDistance));
        Telemetry.log("ShotCalc/TurretAngleDeg", df.format(turretAngle.getDegrees()));
        Telemetry.log("ShotCalc/FlywheelSpeedRPM", df.format(flywheelSpeed));
        Telemetry.log("ShotCalc/TurretPose", turretPosition);
        Telemetry.log("ShotCalc/LookaheadPose", lookaheadPose);
        Telemetry.log(
                "ShotCalc/TargetPose", new Pose2d(target.getX(), target.getY(), new Rotation2d()));
        Telemetry.log("ShotCalc/FlywheelSpeedOffset", FLYWHEEL_SPEED_OFFSET);
        Telemetry.log("ShotCalc/TurretAngleOffsetDegrees", TURRET_ANGLE_OFFSET_DEGREES);
        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }
}
