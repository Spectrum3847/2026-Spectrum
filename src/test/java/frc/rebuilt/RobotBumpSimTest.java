package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class RobotBumpSimTest {

    @Test
    @DisplayName("Test RobotBumpSim initialization and flat ground update")
    void testFlatGroundUpdate() {
        Translation2d[] moduleLocations =
                new Translation2d[] {
                    new Translation2d(0.3, 0.3), // FL
                    new Translation2d(0.3, -0.3), // FR
                    new Translation2d(-0.3, 0.3), // BL
                    new Translation2d(-0.3, -0.3) // BR
                };

        RobotBumpSim bumpSim = new RobotBumpSim(moduleLocations);

        assertFalse(bumpSim.isOnRamp());

        Pose2d startPose = new Pose2d(1.0, 1.0, Rotation2d.kZero);
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0);

        Pose3d pose3d = bumpSim.update(startPose, speeds, 5);

        assertNotNull(pose3d);
        assertEquals(1.0, pose3d.getX(), 1e-6);
        assertEquals(1.0, pose3d.getY(), 1e-6);
        // On flat ground, center Z should be chassis height (0.0)
        assertEquals(0.0, pose3d.getZ(), 1e-6);
        assertFalse(bumpSim.isOnRamp());
    }

    @Test
    @DisplayName("Test RobotBumpSim getSimWorldPose")
    void testGetSimWorldPose() {
        Translation2d[] moduleLocations =
                new Translation2d[] {
                    new Translation2d(0.3, 0.3),
                    new Translation2d(0.3, -0.3),
                    new Translation2d(-0.3, 0.3),
                    new Translation2d(-0.3, -0.3)
                };

        RobotBumpSim bumpSim = new RobotBumpSim(moduleLocations);

        Pose2d maplePose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90.0));
        Pose2d worldPose = bumpSim.getSimWorldPose(maplePose);

        assertEquals(3.0, worldPose.getY(), 1e-6);
        assertEquals(90.0, worldPose.getRotation().getDegrees(), 1e-6);
    }

    @Test
    @DisplayName("Test getSimWorldPose before any update uses zero simXPos")
    void testGetSimWorldPoseBeforeUpdate() {
        Translation2d[] moduleLocations =
                new Translation2d[] {
                    new Translation2d(0.3, 0.3),
                    new Translation2d(0.3, -0.3),
                    new Translation2d(-0.3, 0.3),
                    new Translation2d(-0.3, -0.3)
                };

        RobotBumpSim bumpSim = new RobotBumpSim(moduleLocations);

        // Before update() is ever called, the frictionless X position defaults to 0.0
        Pose2d maplePose = new Pose2d(7.0, 4.0, Rotation2d.fromDegrees(30.0));
        Pose2d worldPose = bumpSim.getSimWorldPose(maplePose);

        assertEquals(0.0, worldPose.getX(), 1e-6);
        assertEquals(4.0, worldPose.getY(), 1e-6);
        assertEquals(30.0, worldPose.getRotation().getDegrees(), 1e-6);
    }

    @Test
    @DisplayName("Test flat ground update produces zero pitch and roll and preserves yaw")
    void testFlatGroundPitchRollAndYaw() {
        Translation2d[] moduleLocations =
                new Translation2d[] {
                    new Translation2d(0.3, 0.3),
                    new Translation2d(0.3, -0.3),
                    new Translation2d(-0.3, 0.3),
                    new Translation2d(-0.3, -0.3)
                };

        RobotBumpSim bumpSim = new RobotBumpSim(moduleLocations);

        Pose2d startPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(30.0));
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        Pose3d pose3d = bumpSim.update(startPose, speeds, 5);

        // Roll (X) and pitch (Y) should be ~0 on flat, level ground
        assertEquals(0.0, pose3d.getRotation().getX(), 1e-6);
        assertEquals(0.0, pose3d.getRotation().getY(), 1e-6);
        // Yaw (Z) should match the input heading
        assertEquals(Math.toRadians(30.0), pose3d.getRotation().getZ(), 1e-6);
    }

    @Test
    @DisplayName("Test repeated flat-ground updates remain stable off the ramp")
    void testRepeatedFlatGroundUpdatesStayOffRamp() {
        Translation2d[] moduleLocations =
                new Translation2d[] {
                    new Translation2d(0.3, 0.3),
                    new Translation2d(0.3, -0.3),
                    new Translation2d(-0.3, 0.3),
                    new Translation2d(-0.3, -0.3)
                };

        RobotBumpSim bumpSim = new RobotBumpSim(moduleLocations);

        // Far away from any bump geometry, stationary
        Pose2d pose = new Pose2d(1.0, 1.0, Rotation2d.kZero);
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        for (int i = 0; i < 10; i++) {
            Pose3d pose3d = bumpSim.update(pose, speeds, 5);
            assertEquals(0.0, pose3d.getZ(), 1e-6);
            assertFalse(bumpSim.isOnRamp());
        }
    }
}
