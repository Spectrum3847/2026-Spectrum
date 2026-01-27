package frc.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.Robot;

public class ShotCalculator {
  private static ShotCalculator instance;

  // Offset from robot center to turret center (leave zero if turret is centered)
  private static final Transform2d robotToTurret = new Transform2d();

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      Rotation2d turretAngle,
      double hoodAngle,
      double flywheelSpeed) {}

  private ShootingParameters latestParameters = null;

  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    shotHoodAngleMap.put(1.8122, Rotation2d.fromDegrees(20.0));
    shotHoodAngleMap.put(2.612079, Rotation2d.fromDegrees(25.0));
    shotHoodAngleMap.put(3.75661, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.96786, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.8122, 200.0);
    shotFlywheelSpeedMap.put(2.612079, 210.0);
    shotFlywheelSpeedMap.put(3.75661, 230.0);
    shotFlywheelSpeedMap.put(4.96786, 260.0);
  }

  public ShootingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Target location on the field
    Translation2d target = HubTargetFactory.generate().toTranslation2d();

    // Turret position on the field
    Pose2d turretPose =
        Robot.getSwerve().getRobotPose().transformBy(robotToTurret);

    // Distance to target
    double distance =
        target.getDistance(turretPose.getTranslation());

    // Field-relative angle from turret to target
    Rotation2d turretAngle =
        target.minus(turretPose.getTranslation()).getAngle();

    double hoodAngle =
        shotHoodAngleMap.get(distance).getRadians();

    double flywheelSpeed =
        shotFlywheelSpeedMap.get(distance);

    latestParameters =
        new ShootingParameters(
            turretAngle,
            hoodAngle,
            flywheelSpeed);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
