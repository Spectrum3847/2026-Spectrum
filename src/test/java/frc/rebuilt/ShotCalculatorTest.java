package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ShotCalculatorTest {

    private Double originalHoodAngleOffset;
    private Double originalTurretAngleOffset;
    /** Restore static state. */
    @AfterEach
    void restoreStaticState() {
        if (originalHoodAngleOffset != null) {
            ShotCalculator.HOOD_ANGLE_OFFSET = originalHoodAngleOffset;
            originalHoodAngleOffset = null;
        }
        if (originalTurretAngleOffset != null) {
            ShotCalculator.TURRET_ANGLE_OFFSET = originalTurretAngleOffset;
            originalTurretAngleOffset = null;
        }
    }
    /** Verifies shooting parameters record. */
    @Test
    @DisplayName("Test ShootingParameters record properties")
    void testShootingParametersRecord() {
        ShotCalculator.ShootingParameters params =
                new ShotCalculator.ShootingParameters(
                        true,
                        Rotation2d.fromDegrees(45.0),
                        0.5,
                        30.0,
                        2.0,
                        4000.0,
                        15.0,
                        5.0,
                        4.8,
                        1.2);

        assertTrue(params.isValid());
        assertEquals(45.0, params.turretAngle().getDegrees(), 1e-6);
        assertEquals(0.5, params.turretAngularVelocity(), 1e-6);
        assertEquals(30.0, params.hoodAngle(), 1e-6);
        assertEquals(2.0, params.hoodVelocity(), 1e-6);
        assertEquals(4000.0, params.flywheelSpeed(), 1e-6);
        assertEquals(15.0, params.exitSpeedMs(), 1e-6);
        assertEquals(5.0, params.distance(), 1e-6);
        assertEquals(4.8, params.distanceNoLookahead(), 1e-6);
        assertEquals(1.2, params.timeOfFlight(), 1e-6);
    }
    /** Verifies hood angle offset commands. */
    @Test
    @DisplayName("Test Hood angle offset increment and decrement commands")
    void testHoodAngleOffsetCommands() {
        originalHoodAngleOffset = ShotCalculator.HOOD_ANGLE_OFFSET;
        double initialOffset = ShotCalculator.HOOD_ANGLE_OFFSET;

        ShotCalculator.increaseHoodAngleOffset().initialize();
        assertEquals(
                initialOffset + ShotCalculator.HOOD_OFFSET_STEP_DEG,
                ShotCalculator.HOOD_ANGLE_OFFSET,
                1e-6);

        ShotCalculator.decreaseHoodAngleOffset().initialize();
        assertEquals(initialOffset, ShotCalculator.HOOD_ANGLE_OFFSET, 1e-6);
    }
    /** Verifies turret angle offset commands. */
    @Test
    @DisplayName("Test Turret angle offset increment and decrement commands")
    void testTurretAngleOffsetCommands() {
        originalTurretAngleOffset = ShotCalculator.TURRET_ANGLE_OFFSET;
        double initialOffset = ShotCalculator.TURRET_ANGLE_OFFSET;

        ShotCalculator.increaseTurretAngleOffset().initialize();
        assertEquals(initialOffset + 1.0, ShotCalculator.TURRET_ANGLE_OFFSET, 1e-6);

        ShotCalculator.decreaseTurretAngleOffset().initialize();
        assertEquals(initialOffset, ShotCalculator.TURRET_ANGLE_OFFSET, 1e-6);
    }
    /** Verifies singleton. */
    @Test
    @DisplayName("Test ShotCalculator singleton instance")
    void testSingleton() {
        ShotCalculator instance1 = ShotCalculator.getInstance();
        ShotCalculator instance2 = ShotCalculator.getInstance();
        assertSame(instance1, instance2);

        instance1.clearShootingParameters();
    }
}
