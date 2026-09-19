package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Random;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * The inverse model in {@link TurretCameraGeometry} must undo the forward model exactly, for every
 * robot pose and turret angle, including across the 180 deg wrap, and a turret zero error must show
 * up in the recovered heading with the sign the zero trim expects.
 */
public class TurretCameraGeometryTest {

    private static final double EPS = 1e-9;

    /**
     * Seeded so a failure reproduces; a field rather than a local to keep SpotBugs' DMI rule quiet.
     */
    private final Random random = new Random(3847);

    /** The real robot: pivot on centre, camera 0.138 m behind the pivot, facing rear at zero. */
    private static final TurretCameraGeometry SPECTRUM =
            new TurretCameraGeometry(
                    Translation2d.kZero, new Translation2d(0.138, 0), Rotation2d.k180deg);

    /** A less symmetric mount, so a sign error in either offset cannot cancel out. */
    private static final TurretCameraGeometry OFFSET =
            new TurretCameraGeometry(
                    new Translation2d(0.12, -0.05),
                    new Translation2d(0.2, 0.03),
                    Rotation2d.fromDegrees(180));

    private static void assertPoseEquals(Pose2d expected, Pose2d actual) {
        assertEquals(expected.getX(), actual.getX(), EPS, "x");
        assertEquals(expected.getY(), actual.getY(), EPS, "y");
        assertEquals(
                0.0,
                expected.getRotation().minus(actual.getRotation()).getDegrees(),
                1e-7,
                "heading");
    }

    @Test
    @DisplayName("At turret zero the camera sits 0.138 m behind centre, facing rearward")
    void cameraAtTurretZero() {
        Pose2d robot = new Pose2d(3, 4, Rotation2d.fromDegrees(30));
        Pose2d camera = SPECTRUM.cameraFloorPose(robot, Rotation2d.kZero);
        Translation2d expected =
                robot.getTranslation()
                        .plus(new Translation2d(-0.138, 0).rotateBy(robot.getRotation()));
        assertEquals(expected.getX(), camera.getX(), EPS);
        assertEquals(expected.getY(), camera.getY(), EPS);
        assertEquals(
                0.0, camera.getRotation().minus(Rotation2d.fromDegrees(210)).getDegrees(), 1e-9);
    }

    @Test
    @DisplayName("Inverse undoes forward for random poses and turret angles")
    void roundTrip() {
        for (TurretCameraGeometry geometry : new TurretCameraGeometry[] {SPECTRUM, OFFSET}) {
            for (int i = 0; i < 2000; i++) {
                Pose2d robot =
                        new Pose2d(
                                random.nextDouble() * 16,
                                random.nextDouble() * 8,
                                Rotation2d.fromDegrees(random.nextDouble() * 720 - 360));
                Rotation2d turret = Rotation2d.fromDegrees(random.nextDouble() * 720 - 360);

                Pose2d camera = geometry.cameraFloorPose(robot, turret);
                Rotation2d impliedHeading =
                        geometry.robotHeadingFromCamera(camera.getRotation(), turret);
                assertEquals(0.0, impliedHeading.minus(robot.getRotation()).getDegrees(), 1e-7);

                // Exact inverse when the camera heading drives the lever arm.
                assertPoseEquals(robot, geometry.robotPose(camera, turret, impliedHeading));
                // And when the gyro heading does, which is what the robot passes.
                assertPoseEquals(robot, geometry.robotPose(camera, turret, robot.getRotation()));
            }
        }
    }

    @Test
    @DisplayName(
            "A turret zero error appears as camera-implied heading minus gyro, actual minus reported")
    void zeroErrorSign() {
        Pose2d robot = new Pose2d(5, 2, Rotation2d.fromDegrees(-140));
        Rotation2d actualTurret = Rotation2d.fromDegrees(40);
        double zeroErrorDeg = 9.6;
        Rotation2d reportedTurret = actualTurret.minus(Rotation2d.fromDegrees(zeroErrorDeg));

        // The camera really is where the actual turret angle puts it.
        Pose2d camera = SPECTRUM.cameraFloorPose(robot, actualTurret);

        // The robot believes the encoder.
        Rotation2d implied = SPECTRUM.robotHeadingFromCamera(camera.getRotation(), reportedTurret);
        double errorDeg = implied.minus(robot.getRotation()).getDegrees();

        // Turret.applyZeroCorrectionDegrees expects actual minus reported.
        assertEquals(zeroErrorDeg, errorDeg, 1e-9);
    }

    @Test
    @DisplayName(
            "A wrong turret angle displaces the recovered translation by arm times angle, not more")
    void translationErrorIsBoundedByArm() {
        Pose2d robot = new Pose2d(5, 2, Rotation2d.fromDegrees(10));
        Rotation2d actualTurret = Rotation2d.fromDegrees(-70);
        Rotation2d wrongTurret = actualTurret.plus(Rotation2d.fromDegrees(5));

        Pose2d camera = SPECTRUM.cameraFloorPose(robot, actualTurret);
        Pose2d recovered = SPECTRUM.robotPose(camera, wrongTurret, robot.getRotation());

        double displacement = recovered.getTranslation().getDistance(robot.getTranslation());
        // Chord of a 5 deg arc on a 0.138 m arm: about 12 mm. This is the whole point of solving
        // on the roboRIO: a 5 deg transform lag used to move the pose by range times the angle.
        double chord = 2 * 0.138 * Math.sin(Math.toRadians(2.5));
        assertEquals(chord, displacement, 1e-9);
    }

    @Test
    @DisplayName("Headings wrap cleanly across +/-180")
    void wrap() {
        Pose2d robot = new Pose2d(1, 1, Rotation2d.fromDegrees(179));
        Rotation2d turret = Rotation2d.fromDegrees(3);
        Pose2d camera = SPECTRUM.cameraFloorPose(robot, turret);
        // 179 + 3 + 180 = 362 -> 2 deg
        assertEquals(2.0, camera.getRotation().getDegrees(), 1e-9);
        assertPoseEquals(robot, SPECTRUM.robotPose(camera, turret, robot.getRotation()));
    }
}
