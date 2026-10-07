package frc.quixlib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.Constants;
import frc.robot.Fiducials;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class LaunchCalculator {
  public static final boolean kLaserTestMode = false;

  public static final double kSystemLatencySec = 0.05; // seconds

  public static final String kDISTANCE_COLUMN_LABEL = "distance_m";
  public static final String kVELOCITY_COLUMN_STRING = "optimal_v_mps";
  public static final String kANGLE_COLUMN_LABEL = "optimal_angle_deg";

  private final InterpolatingTreeMap<Double, LaunchParameter> m_launchTable =
      new InterpolatingTreeMap<Double, LaunchParameter>(
          InverseInterpolator.forDouble(), LaunchParameter::interpolate);
  private final Path m_launchTablePath;
  private double m_minKey = Double.MAX_VALUE;
  private double m_maxKey = Double.MIN_VALUE;

  public LaunchCalculator(String csvPath) {
    m_launchTablePath = resolveCsvPath(csvPath);
    reloadLaunchTable();
  }

  public LaunchParameter interpolateLaunchEntry(double distanceM) {
    return m_launchTable.get(distanceM).updateInTableRange(inTableRange(distanceM));
  }

  public boolean inTableRange(double distanceM) {
    return !(distanceM < m_minKey || distanceM > m_maxKey);
  }

  public void reloadLaunchTable() {
    final List<String> lines;
    try {
      lines = Files.readAllLines(m_launchTablePath, StandardCharsets.US_ASCII);
    } catch (IOException e) {
      throw new IllegalStateException(
          "Failed to read launch calculator CSV: " + m_launchTablePath, e);
    }

    if (lines.size() <= 1) {
      throw new IllegalStateException(
          "Launch calculator CSV has no data rows: " + m_launchTablePath);
    }

    final String[] headers = lines.get(0).trim().split(",", -1);
    int distanceIndex = -1;
    int velocityIndex = -1;
    int angleIndex = -1;
    for (int i = 0; i < headers.length; i++) {
      if (headers[i].equals(kDISTANCE_COLUMN_LABEL)) {
        distanceIndex = i;
      }
      if (headers[i].equals(kVELOCITY_COLUMN_STRING)) {
        velocityIndex = i;
      }
      if (headers[i].equals(kANGLE_COLUMN_LABEL)) {
        angleIndex = i;
      }
    }

    if (distanceIndex < 0 || velocityIndex < 0 || angleIndex < 0) {
      throw new IllegalStateException(
          "Launch calculator CSV is missing required columns: " + m_launchTablePath);
    }

    // Read in actual data rows, register in LaunchTable
    m_launchTable.clear();
    m_minKey = Double.MAX_VALUE;
    m_maxKey = Double.MIN_VALUE;

    final int maximalIndex = Math.max(distanceIndex, Math.max(velocityIndex, angleIndex));

    // iterate through data rows insert into launch table, assume that rows are pre-sorted by
    // distance
    for (int lineIndex = 0; lineIndex < lines.size() - 1; lineIndex++) {
      final String[] tokens = lines.get(lineIndex + 1).trim().split(",", maximalIndex + 2);
      if (tokens.length <= maximalIndex) {
        throw new IllegalStateException(
            "Line " + lineIndex + " in launch calculator CSV has fewer columns than expected");
      }

      final double distance = Double.parseDouble(tokens[distanceIndex].trim());
      m_minKey = Math.min(m_minKey, distance);
      m_maxKey = Math.max(m_maxKey, distance);

      m_launchTable.put(
          distance,
          new LaunchParameter(
              Double.parseDouble(tokens[velocityIndex].trim()),
              Double.parseDouble(tokens[angleIndex].trim())));
    }
  }

  private static Path resolveCsvPath(String csvPath) {
    if (csvPath == null || csvPath.isBlank() || !csvPath.endsWith(".csv")) {
      throw new IllegalArgumentException("Launch calculator CSV path is required");
    }
    final Path result = Paths.get(csvPath).toAbsolutePath().normalize();
    if (Files.exists(result) && Files.isRegularFile(result)) {
      return result;
    }

    throw new IllegalStateException("Launch calculator CSV path not found: " + result);
  }

  public static class LaunchInfo {
    public final double yaw;
    public double yawVelocity;
    public double yawAcceleration;
    public final double elevation;
    public final double launchVelocity;

    public LaunchInfo(
        double yaw,
        double yawVelocity,
        double yawAcceleration,
        double elevation,
        double launchVelocity) {
      this.yaw = yaw;
      this.yawVelocity = yawVelocity;
      this.yawAcceleration = yawAcceleration;
      this.elevation = elevation;
      this.launchVelocity = launchVelocity;
    }
  }

  private static LaunchInfo computeStateAtTime(
      final boolean isFeed,
      final double t,
      final Pose2d robotFieldPose,
      final Pose2d targetPose,
      final LaunchCalculator launchCalculator,
      final Twist2d robotVel,
      final Twist2d robotAccel,
      final Twist2d robotFieldVel,
      final Twist2d robotFieldAccel,
      final double imuRoll,
      final double imuPitch) {
    final Twist2d dTwist =
        new Twist2d(
            robotVel.dx * t + 0.5 * robotAccel.dx * t * t,
            robotVel.dy * t + 0.5 * robotAccel.dy * t * t,
            robotVel.dtheta * t + 0.5 * robotAccel.dtheta * t * t);
    final Pose2d pose = robotFieldPose.exp(dTwist);

    final Twist2d vel =
        new Twist2d(
            robotVel.dx + robotAccel.dx * t,
            robotVel.dy + robotAccel.dy * t,
            robotVel.dtheta + robotAccel.dtheta * t);

    final Twist2d fieldVel =
        new Twist2d(
            robotFieldVel.dx + robotFieldAccel.dx * t,
            robotFieldVel.dy + robotFieldAccel.dy * t,
            robotFieldVel.dtheta + robotFieldAccel.dtheta * t);

    return computeInstantaneousLaunchInfo(
        isFeed,
        pose,
        targetPose,
        launchCalculator,
        vel,
        robotAccel,
        fieldVel,
        robotFieldAccel,
        imuRoll,
        imuPitch);
  }

  public static LaunchInfo computeLaunchInfo(
      final boolean isFeed,
      final Pose2d robotFieldPose,
      final Pose2d targetPose,
      final LaunchCalculator launchCalculator,
      final Twist2d robotVel,
      final Twist2d robotAccel,
      final Twist2d robotFieldVel,
      final Twist2d robotFieldAccel,
      final double imuRoll,
      final double imuPitch) {
    final double dt = 0.005; // seconds

    final LaunchInfo infoMinus =
        computeStateAtTime(
            isFeed,
            kSystemLatencySec - dt,
            robotFieldPose,
            targetPose,
            launchCalculator,
            robotVel,
            robotAccel,
            robotFieldVel,
            robotFieldAccel,
            imuRoll,
            imuPitch);
    final LaunchInfo info0 =
        computeStateAtTime(
            isFeed,
            kSystemLatencySec,
            robotFieldPose,
            targetPose,
            launchCalculator,
            robotVel,
            robotAccel,
            robotFieldVel,
            robotFieldAccel,
            imuRoll,
            imuPitch);
    final LaunchInfo infoPlus =
        computeStateAtTime(
            isFeed,
            kSystemLatencySec + dt,
            robotFieldPose,
            targetPose,
            launchCalculator,
            robotVel,
            robotAccel,
            robotFieldVel,
            robotFieldAccel,
            imuRoll,
            imuPitch);

    info0.yawVelocity =
        MathUtils.constrainAngleNegPiToPi(infoPlus.yaw - infoMinus.yaw) / (2.0 * dt);
    info0.yawAcceleration =
        (MathUtils.constrainAngleNegPiToPi(infoPlus.yaw - info0.yaw)
                - MathUtils.constrainAngleNegPiToPi(info0.yaw - infoMinus.yaw))
            / (dt * dt);

    return info0;
  }

  public static LaunchInfo computeInstantaneousLaunchInfo(
      final boolean isFeed,
      final Pose2d robotFieldPose,
      final Pose2d targetPose,
      final LaunchCalculator launchCalculator,
      final Twist2d robotVel,
      final Twist2d robotAccel,
      final Twist2d robotFieldVel,
      final Twist2d robotFieldAccel,
      final double imuRoll,
      final double imuPitch) {
    // Transform robot pose to turret base pose
    final Pose2d turretFieldPose =
        robotFieldPose.transformBy(Constants.Turret.robotToTurretBaseT2d);
    final Twist2d turretVelocity =
        MathUtils.getVelocityAtOffset(
            robotFieldPose, Constants.Turret.robotToTurretBaseT2d, robotFieldVel);

    // Calculate distance from turret to target
    final Translation2d turretToTarget =
        (kLaserTestMode ? Fiducials.aprilTagFiducials[25].getPose().toPose2d() : targetPose)
            .getTranslation()
            .minus(turretFieldPose.getTranslation());
    final double distanceToTarget = turretToTarget.getNorm();
    Logger.recordOutput("LaunchCalculator/distanceToTarget", distanceToTarget);

    final double v0;
    final double phi0;
    final double theta0 = Math.atan2(turretToTarget.getY(), turretToTarget.getX());
    if (isFeed) {
      // Solve for launch velocity using ballistic trajectory equations
      // Using: range = v0 * cos(theta) * (v0 * sin(theta) + sqrt((v0 * sin(theta))^2 + 2*g*h)) / g
      phi0 = Constants.Turret.feedingAngle;
      v0 =
          MathUtils.solveBallisticVelocity(
                  distanceToTarget, -Constants.Turret.robotToTurretBaseT.getZ(), phi0)
              .orElse(0.0);
    } else {
      // Get optimal launch parameters from calculator
      final var fieldRelativeLaunchParams =
          kLaserTestMode
              ? new LaunchParameter(Double.MAX_VALUE, Constants.Turret.startingHoodAngle)
              : launchCalculator.interpolateLaunchEntry(distanceToTarget);
      v0 = fieldRelativeLaunchParams.launchVelocityMps();
      phi0 = Math.toRadians(fieldRelativeLaunchParams.launchAngleDeg());
    }

    // Desired field-relative launch velocity components
    final double vx_target = v0 * Math.cos(phi0) * Math.cos(theta0);
    final double vy_target = v0 * Math.cos(phi0) * Math.sin(theta0);
    final double vz_target = v0 * Math.sin(phi0);

    // Required field-relative launch velocity components (subtract turret motion)
    final double vx_field = vx_target - turretVelocity.dx;
    final double vy_field = vy_target - turretVelocity.dy;
    final double vz_field = vz_target;

    // Rotate into the robot's tilted body frame, accounting for heading, roll, and pitch.
    final double heading = turretFieldPose.getRotation().getRadians();
    final double[] vRel =
        fieldToRobotVelocity(vx_field, vy_field, vz_field, heading, imuRoll, imuPitch);
    final double vx_rel = vRel[0];
    final double vy_rel = vRel[1];
    final double vz_rel = vRel[2];

    // Convert back to robot-relative launch parameters
    final double v_rel = Math.sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
    final double phi_rel = Math.asin(vz_rel / v_rel);
    final double yaw_rel = Math.atan2(vy_rel, vx_rel) - turretFieldPose.getRotation().getRadians();

    return new LaunchInfo(yaw_rel, 0, 0, phi_rel, v_rel);
  }

  /**
   * Rotates a field-relative velocity vector into the robot's tilted body frame, accounting for the
   * robot's heading, roll, and pitch. The result is re-projected back to field heading coordinates
   * so that downstream yaw computation (atan2) still works correctly.
   *
   * <p>Steps: un-rotate by heading → apply Ry(-pitch) → apply Rx(-roll) → re-rotate by heading.
   *
   * @param vxField field-relative X velocity (m/s)
   * @param vyField field-relative Y velocity (m/s)
   * @param vzField field-relative Z (upward) velocity (m/s)
   * @param heading robot/turret field heading (rad, CCW positive)
   * @param roll IMU roll (rad, rotation around +X axis, right-positive)
   * @param pitch IMU pitch (rad, rotation around +Y axis, down-positive)
   * @return {vx, vy, vz} in the robot's tilted frame, re-rotated to field heading
   */
  public static double[] fieldToRobotVelocity(
      final double vxField,
      final double vyField,
      final double vzField,
      final double heading,
      final double roll,
      final double pitch) {
    final double cosH = Math.cos(heading);
    final double sinH = Math.sin(heading);
    // Un-rotate yaw: align field X/Y with robot forward/left
    final double vfwd = cosH * vxField + sinH * vyField;
    final double vlat = -sinH * vxField + cosH * vyField;
    final double vup = vzField;

    // Apply inverse body tilt: Ry(-pitch) then Rx(-roll)
    final double cosR = Math.cos(roll);
    final double sinR = Math.sin(roll);
    final double cosP = Math.cos(pitch);
    final double sinP = Math.sin(pitch);
    // Ry(-pitch)
    final double vfwd2 = cosP * vfwd - sinP * vup;
    final double vlat2 = vlat;
    final double vup2 = sinP * vfwd + cosP * vup;
    // Rx(-roll)
    final double vfwd3 = vfwd2;
    final double vlat3 = cosR * vlat2 + sinR * vup2;
    final double vup3 = -sinR * vlat2 + cosR * vup2;

    // Re-rotate back to field heading so downstream yaw computation (atan2) still works correctly
    return new double[] {cosH * vfwd3 - sinH * vlat3, sinH * vfwd3 + cosH * vlat3, vup3};
  }

  // Yaw utils
  public static double getTargetYaw(final Pose2d robotPose, final Translation2d target) {
    final double dx = target.getX() - robotPose.getX();
    final double dy = target.getY() - robotPose.getY();
    return Math.atan2(dy, dx) - robotPose.getRotation().getRadians();
  }

  public static double getTargetYawVelocity(
      final Pose2d robotPose, final Twist2d robotVelocity, final Translation2d target) {
    final double x = target.getX() - robotPose.getX();
    final double y = target.getY() - robotPose.getY();
    final double xVelocity = robotVelocity.dx;
    final double yVelocity = robotVelocity.dy;
    final double rotationVelocity = robotVelocity.dtheta;
    return ((y * xVelocity) - (x * yVelocity)) / (x * x + y * y) - rotationVelocity;
  }

  public static double getTargetYawAcceleration(
      final Pose2d robotPose,
      final Twist2d robotVelocity,
      final Twist2d robotAcceleration,
      final Translation2d target) {
    final double x = target.getX() - robotPose.getX();
    final double y = target.getY() - robotPose.getY();
    final double xVelocity = robotVelocity.dx;
    final double yVelocity = robotVelocity.dy;
    final double xAcceleration = robotAcceleration.dx;
    final double yAcceleration = robotAcceleration.dy;
    final double rotationAcceleration = robotAcceleration.dtheta;
    final double r2 = x * x + y * y;
    return (((y * xAcceleration) - (x * yAcceleration)) * r2
                - 2 * ((y * xVelocity) - (x * yVelocity)) * ((x * xVelocity) + (y * yVelocity)))
            / (r2 * r2)
        - rotationAcceleration;
  }
}
