package frc.rebuilt;

import dev.doglog.DogLog;
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
import frc.robot.RobotStates;
import frc.spectrumLib.vision.LimelightHelpers;

public class ShotCalculator {
  private static ShotCalculator instance;

  // Offset from robot center to turret center (leave zero if turret is centered)
  private static final Transform2d robotToTurret = new Transform2d();

  public static ShotCalculator getInstance() {
    if (instance == null)
      instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      Rotation2d turretAngle,
      double visionTurretOffset,
      double flywheelSpeed) {
  }

  private ShootingParameters latestParameters = null;

  private static double phaseDelay;

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    phaseDelay = 0.03;

    shotFlywheelSpeedMap.put(1.5, 30.5);
    shotFlywheelSpeedMap.put(1.78, 31.0);
    shotFlywheelSpeedMap.put(2.00, 33.5);
    shotFlywheelSpeedMap.put(2.35, 35.0);

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
    boolean feed = RobotStates.robotInFeedZone.getAsBoolean();
    Translation2d target = feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();

    // Calculate estimated pose while accounting for phase delay
    Pose2d robotPose = Robot.getSwerve().getRobotPose();
    ChassisSpeeds robotRelativeVelocity = Robot.getSwerve().getCurrentRobotChassisSpeeds();
    robotPose = robotPose.exp(
        new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
            robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
            robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Pose2d turretPosition = robotPose.transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity,
        Robot.getSwerve().getRobotPose().getRotation());
    double robotAngle = robotPose.getRotation().getRadians();
    double turretVelocityX = robotVelocity.vxMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (robotToTurret.getY() * Math.cos(robotAngle)
                - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY = robotVelocity.vyMetersPerSecond
        + robotVelocity.omegaRadiansPerSecond
            * (robotToTurret.getX() * Math.cos(robotAngle)
                - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose = new Pose2d(
          turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
          turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }
    // Calculate parameters accounted for imparted velocity
    Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    double flywheelSpeed = shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance);

    double visionTurretOffset = Robot.getVision().getTurretLL().getHorizontalOffset();

    latestParameters = new ShootingParameters(
        turretAngle,
        visionTurretOffset,
        flywheelSpeed);

    DogLog.log("ShotCalc/DistanceMeters", Double.toString(lookaheadTurretToTargetDistance));
    DogLog.log("ShotCalc/LookaheadPose", lookaheadPose);
    DogLog.log("ShotCalc/TurretAngleDeg", Double.toString(turretAngle.getDegrees()));
    DogLog.log("ShotCalc/FlywheelSpeed", Double.toString(flywheelSpeed));
    DogLog.log("ShotCalc/TargetPose", new Pose2d(target.getX(), target.getY(), new Rotation2d()));
    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
