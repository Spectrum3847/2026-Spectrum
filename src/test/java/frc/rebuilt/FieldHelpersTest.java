package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class FieldHelpersTest {
    /** Verifies flip angle. */
    @Test
    @DisplayName("Test flipAngle with degrees and Rotation2d")
    void testFlipAngle() {
        assertEquals(270.0, FieldHelpers.flipAngle(90.0), 1e-6);
        assertEquals(180.0, FieldHelpers.flipAngle(0.0), 1e-6);
        assertEquals(90.0, FieldHelpers.flipAngle(270.0), 1e-6);

        Rotation2d rot = Rotation2d.fromDegrees(45.0);
        Rotation2d flipped = FieldHelpers.flipAngle(rot);
        assertEquals(-135.0, flipped.getDegrees(), 1e-6);
    }
    /** Verifies flip xand y. */
    @Test
    @DisplayName("Test flipX and flipY")
    void testFlipXandY() {
        double expectedFlippedX = Field.fieldLength - 5.0;
        double expectedFlippedY = Field.fieldWidth - 3.0;

        assertEquals(expectedFlippedX, FieldHelpers.flipX(5.0), 1e-6);
        assertEquals(expectedFlippedY, FieldHelpers.flipY(3.0), 1e-6);
    }
    /** Verifies normalize angle. */
    @Test
    @DisplayName("Test normalizeAngle radians")
    void testNormalizeAngle() {
        assertEquals(0.0, FieldHelpers.normalizeAngle(0.0), 1e-6);
        assertEquals(Math.PI / 2.0, FieldHelpers.normalizeAngle(Math.PI / 2.0), 1e-6);
        assertEquals(-Math.PI / 2.0, FieldHelpers.normalizeAngle(3.0 * Math.PI / 2.0), 1e-6);
        assertEquals(0.0, FieldHelpers.normalizeAngle(2 * Math.PI), 1e-6);
    }
    /** Verifies pose out of field. */
    @Test
    @DisplayName("Test poseOutOfField for Pose2d and Pose3d")
    void testPoseOutOfField() {
        // Valid poses inside field
        Pose2d insidePose = new Pose2d(5.0, 3.0, Rotation2d.kZero);
        assertFalse(FieldHelpers.poseOutOfField(insidePose));

        Pose3d insidePose3d = new Pose3d(insidePose);
        assertFalse(FieldHelpers.poseOutOfField(insidePose3d));

        // Out of field poses
        Pose2d negativeX = new Pose2d(-0.1, 3.0, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(negativeX));

        Pose2d tooLargeX = new Pose2d(Field.fieldLength + 0.1, 3.0, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(tooLargeX));

        Pose2d negativeY = new Pose2d(5.0, -0.1, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(negativeY));

        Pose2d tooLargeY = new Pose2d(5.0, Field.fieldWidth + 0.1, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(tooLargeY));

        // Poses exactly on the field boundaries are also out of field
        Pose2d zeroX = new Pose2d(0.0, 3.0, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(zeroX));
        assertTrue(FieldHelpers.poseOutOfField(new Pose3d(zeroX)));

        Pose2d fieldLengthX = new Pose2d(Field.fieldLength, 3.0, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(fieldLengthX));
        assertTrue(FieldHelpers.poseOutOfField(new Pose3d(fieldLengthX)));

        Pose2d zeroY = new Pose2d(5.0, 0.0, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(zeroY));
        assertTrue(FieldHelpers.poseOutOfField(new Pose3d(zeroY)));

        Pose2d fieldWidthY = new Pose2d(5.0, Field.fieldWidth, Rotation2d.kZero);
        assertTrue(FieldHelpers.poseOutOfField(fieldWidthY));
        assertTrue(FieldHelpers.poseOutOfField(new Pose3d(fieldWidthY)));
    }
}
