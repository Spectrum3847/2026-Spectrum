package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class FieldHelpersTest {

    @Test
    void flipAngle_positive() {
        assertEquals(190.0, FieldHelpers.flipAngle(10.0), 1e-9);
    }

    @Test
    void flipAngle_180becomesZero() {
        assertEquals(0.0, FieldHelpers.flipAngle(180.0), 1e-9);
    }

    @Test
    void flipAngle_above360() {
        assertEquals(130.0, FieldHelpers.flipAngle(310.0), 1e-9);
    }

    @Test
    void flipAngle_zero() {
        assertEquals(180.0, FieldHelpers.flipAngle(0.0), 1e-9);
    }

    @Test
    void flipAngle_Rotation2d() {
        Rotation2d result = FieldHelpers.flipAngle(Rotation2d.fromDegrees(45));
        assertEquals(-135.0, result.getDegrees(), 1e-9);
    }

    @Test
    void flipAngle_Rotation2d_zero() {
        Rotation2d result = FieldHelpers.flipAngle(Rotation2d.fromDegrees(0));
        assertEquals(180.0, result.getDegrees(), 1e-9);
    }

    @Test
    void flipAngle_Rotation2d_negative() {
        Rotation2d result = FieldHelpers.flipAngle(Rotation2d.fromDegrees(-90));
        assertEquals(90.0, result.getDegrees(), 1e-9);
    }

    @Test
    void normalizeAngle_alreadyInRange() {
        assertEquals(1.0, FieldHelpers.normalizeAngle(1.0), 1e-9);
    }

    @Test
    void normalizeAngle_abovePi() {
        double result = FieldHelpers.normalizeAngle(Math.PI + 0.5);
        assertTrue(result < Math.PI);
        assertTrue(result > -Math.PI);
    }

    @Test
    void normalizeAngle_belowNegativePi() {
        double result = FieldHelpers.normalizeAngle(-Math.PI - 0.5);
        assertTrue(result < Math.PI);
        assertTrue(result > -Math.PI);
    }

    @Test
    void normalizeAngle_exactPi_wrapsToNegativePi() {
        double result = FieldHelpers.normalizeAngle(Math.PI);
        assertEquals(-Math.PI, result, 1e-9);
    }

    @Test
    void normalizeAngle_multiTurn() {
        double result = FieldHelpers.normalizeAngle(4 * Math.PI + 0.1);
        assertEquals(0.1, result, 1e-9);
    }

    @Test
    void normalizeAngle_zero() {
        assertEquals(0.0, FieldHelpers.normalizeAngle(0.0), 1e-9);
    }

    @Test
    void poseOutOfField_xNegative() {
        assertTrue(
                FieldHelpers.poseOutOfField(
                        new edu.wpi.first.math.geometry.Pose2d(-0.1, 4.0, new Rotation2d())));
    }

    @Test
    void poseOutOfField_yNegative() {
        assertTrue(
                FieldHelpers.poseOutOfField(
                        new edu.wpi.first.math.geometry.Pose2d(8.0, -0.1, new Rotation2d())));
    }

    @Test
    void poseOutOfField_inside() {
        assertFalse(
                FieldHelpers.poseOutOfField(
                        new edu.wpi.first.math.geometry.Pose2d(8.0, 4.0, new Rotation2d())));
    }

    @Test
    void poseOutOfField_atBoundary_true() {
        assertTrue(
                FieldHelpers.poseOutOfField(
                        new edu.wpi.first.math.geometry.Pose2d(0.0, 4.0, new Rotation2d())));
    }
}
