package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class RobotBumpSimTest {

    /** Verifies initialization and flat-ground pose output of the bump sim. */
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

    /** Verifies getSimWorldPose substitutes simXPos for X while keeping Y and rotation. */
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

        // Drive into the bump so the frictionless slide sim owns the field-X position
        Pose2d maplePose = new Pose2d(3.8, 3.0, Rotation2d.fromDegrees(90.0));
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        Pose3d simulated = bumpSim.update(maplePose, speeds, 5);

        assertTrue(bumpSim.isOnRamp());

        Pose2d worldPose = bumpSim.getSimWorldPose(maplePose);

        // X comes from the frictionless simXPos advanced by update(), not the MapleSim pose X
        assertEquals(simulated.getX(), worldPose.getX(), 1e-6);
        assertTrue(worldPose.getX() > 0.0);
        assertEquals(3.0, worldPose.getY(), 1e-6);
        assertEquals(90.0, worldPose.getRotation().getDegrees(), 1e-6);
    }
}
