package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Planar geometry of a camera mounted on the turret: where the camera is for a given robot pose and
 * turret angle, and the inverse, which robot pose a camera solve implies.
 *
 * <p>Pure functions of the three numbers the turret mount is described by, with no robot state, so
 * the forward and inverse models can be checked against each other in a unit test. The forward
 * model is what the robot used to push to the camera every loop; the inverse is what it now applies
 * on the roboRIO to a camera solve, using the turret angle from the frame's own timestamp rather
 * than whatever the turret was doing when the solve arrived.
 *
 * <p>Conventions: robot frame is x forward, y left, angles counter-clockwise positive. Turret angle
 * zero points the turret robot-forward. The camera's yaw in the robot frame is the turret angle
 * plus the constructor's {@code cameraYawAtTurretZero}, and its planar position is the turret pivot
 * plus the pivot arm rotated by that yaw.
 */
public final class TurretCameraGeometry {

    private final Translation2d robotToTurretCenter;
    private final Translation2d turretCenterToCamera;
    private final Rotation2d cameraYawAtTurretZero;

    /**
     * @param robotToTurretCenter robot centre to turret pivot, metres, robot frame
     * @param turretCenterToCamera turret pivot to camera along the camera's look direction, metres,
     *     expressed in the camera's yaw frame (so a camera on the pivot arm behind the pivot is
     *     {@code (+arm, 0)} together with a 180 deg {@code cameraYawAtTurretZero})
     * @param cameraYawAtTurretZero the camera's yaw in the robot frame with the turret at zero
     */
    public TurretCameraGeometry(
            Translation2d robotToTurretCenter,
            Translation2d turretCenterToCamera,
            Rotation2d cameraYawAtTurretZero) {
        this.robotToTurretCenter = robotToTurretCenter;
        this.turretCenterToCamera = turretCenterToCamera;
        this.cameraYawAtTurretZero = cameraYawAtTurretZero;
    }

    /** The camera's yaw in the robot frame at this turret angle. */
    public Rotation2d cameraYawInRobot(Rotation2d turretAngle) {
        return turretAngle.plus(cameraYawAtTurretZero);
    }

    /** The camera's planar position in the robot frame at this turret angle. */
    public Translation2d cameraInRobot(Rotation2d turretAngle) {
        return robotToTurretCenter.plus(
                turretCenterToCamera.rotateBy(cameraYawInRobot(turretAngle)));
    }

    /** Robot-to-camera planar transform at this turret angle. */
    public Transform2d robotToCamera(Rotation2d turretAngle) {
        return new Transform2d(cameraInRobot(turretAngle), cameraYawInRobot(turretAngle));
    }

    /**
     * Forward model: the camera's floor-projected field pose (its planar position and heading) for
     * a robot pose and turret angle.
     */
    public Pose2d cameraFloorPose(Pose2d robotPose, Rotation2d turretAngle) {
        return robotPose.transformBy(robotToCamera(turretAngle));
    }

    /**
     * The robot heading a camera heading implies at this turret angle. A steady difference between
     * this and the gyro heading is the turret zero error (or a pose heading error; the two cannot
     * be told apart from the turret camera alone).
     */
    public Rotation2d robotHeadingFromCamera(Rotation2d cameraHeading, Rotation2d turretAngle) {
        return cameraHeading.minus(cameraYawInRobot(turretAngle));
    }

    /**
     * Inverse model: the robot pose implied by a camera floor pose.
     *
     * <p>The lever arm from camera back to robot centre is rotated by {@code robotHeading}, which
     * the caller chooses. Passing the gyro heading at the frame time makes the recovered
     * translation independent of the camera's own heading solve, which for one or two tags is the
     * noisiest thing it reports; passing {@link #robotHeadingFromCamera} instead makes this the
     * exact inverse of {@link #cameraFloorPose}. The returned heading is always the camera-implied
     * one, so the caller can compare it with the gyro.
     *
     * @param cameraFloorPose the camera's planar field position and heading
     * @param turretAngle the turret angle at the moment the frame was captured
     * @param robotHeading the heading used to un-rotate the lever arm
     */
    public Pose2d robotPose(
            Pose2d cameraFloorPose, Rotation2d turretAngle, Rotation2d robotHeading) {
        Translation2d robot =
                cameraFloorPose
                        .getTranslation()
                        .minus(cameraInRobot(turretAngle).rotateBy(robotHeading));
        return new Pose2d(
                robot, robotHeadingFromCamera(cameraFloorPose.getRotation(), turretAngle));
    }
}
