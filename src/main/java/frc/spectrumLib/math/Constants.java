package frc.spectrumLib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
  public static final class Turret {
    public static final double robotToTurretBaseX = 0.0; // meters
    public static final double robotToTurretBaseY = 0.0; // meters
    public static final double robotToTurretBaseZ = 0.0; // meters
    public static final Transform2d robotToTurretBaseT2d =
        new Transform2d(new Translation2d(robotToTurretBaseX, robotToTurretBaseY), Rotation2d.fromDegrees(0.0));
    public static final Translation3d robotToTurretBaseT =
        new Translation3d(robotToTurretBaseX, robotToTurretBaseY, robotToTurretBaseZ);

    public static final double feedingAngle = Math.toRadians(45.0); // radians
    public static final double startingHoodAngle = Math.toRadians(30.0); // radians
  }
}
