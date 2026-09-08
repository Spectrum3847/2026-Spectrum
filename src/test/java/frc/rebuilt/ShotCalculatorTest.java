package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ShotCalculatorTest {

    private Double originalHoodAngleOffset;
    private Double originalTurretAngleOffset;

    /**
     * Restore static state, including the Preferences store.
     *
     * <p>Since 2026-09-08 the trim commands write through to {@link Preferences}, so putting the
     * static fields back is no longer enough: a leftover stored trim would be read by the next test
     * that calls {@code loadPersistedTrims()}.
     */
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
        Preferences.setDouble(ShotCalculator.HOOD_TRIM_PREF_KEY, ShotCalculator.HOOD_ANGLE_OFFSET);
        Preferences.setDouble(
                ShotCalculator.TURRET_TRIM_PREF_KEY, ShotCalculator.TURRET_ANGLE_OFFSET);
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

    /**
     * The whole point of 3.1: a trim dialled in during a session is still there after the code
     * restarts. Zeroing the field and reloading is what a redeploy does to these statics.
     */
    @Test
    @DisplayName("A trim survives the restart that used to zero it")
    void trimSurvivesRestart() {
        originalHoodAngleOffset = ShotCalculator.HOOD_ANGLE_OFFSET;
        originalTurretAngleOffset = ShotCalculator.TURRET_ANGLE_OFFSET;

        ShotCalculator.HOOD_ANGLE_OFFSET = 0;
        ShotCalculator.TURRET_ANGLE_OFFSET = 0;
        ShotCalculator.decreaseHoodAngleOffset().initialize();
        ShotCalculator.decreaseHoodAngleOffset().initialize();
        ShotCalculator.increaseTurretAngleOffset().initialize();

        double hood = ShotCalculator.HOOD_ANGLE_OFFSET;
        double turret = ShotCalculator.TURRET_ANGLE_OFFSET;
        assertEquals(-2 * ShotCalculator.HOOD_OFFSET_STEP_DEG, hood, 1e-9);

        // What a redeploy leaves behind: fresh statics, and whatever is in flash.
        ShotCalculator.HOOD_ANGLE_OFFSET = ShotCalculator.STARTING_HOOD_ANGLE_OFFSET;
        ShotCalculator.TURRET_ANGLE_OFFSET = ShotCalculator.STARTING_TURRET_ANGLE_OFFSET;
        ShotCalculator.loadPersistedTrims();

        assertEquals(hood, ShotCalculator.HOOD_ANGLE_OFFSET, 1e-9);
        assertEquals(turret, ShotCalculator.TURRET_ANGLE_OFFSET, 1e-9);
    }

    /** A D-pad held down cannot walk a trim past the cap. */
    @Test
    @DisplayName("Presses clamp at MAX_TRIM_DEG instead of accumulating")
    void pressesClampAtTheCap() {
        originalHoodAngleOffset = ShotCalculator.HOOD_ANGLE_OFFSET;
        originalTurretAngleOffset = ShotCalculator.TURRET_ANGLE_OFFSET;

        ShotCalculator.HOOD_ANGLE_OFFSET = 0;
        int presses =
                (int) (ShotCalculator.MAX_TRIM_DEG / ShotCalculator.HOOD_OFFSET_STEP_DEG) + 10;
        for (int i = 0; i < presses; i++) {
            ShotCalculator.increaseHoodAngleOffset().initialize();
        }
        assertEquals(ShotCalculator.MAX_TRIM_DEG, ShotCalculator.HOOD_ANGLE_OFFSET, 1e-9);

        ShotCalculator.TURRET_ANGLE_OFFSET = 0;
        for (int i = 0; i < presses; i++) {
            ShotCalculator.decreaseTurretAngleOffset().initialize();
        }
        assertEquals(-ShotCalculator.MAX_TRIM_DEG, ShotCalculator.TURRET_ANGLE_OFFSET, 1e-9);
    }

    /**
     * The case the cap actually exists for: something other than the D-pad wrote the stored value.
     * A hand-edited or corrupt preference must not reach the turret.
     */
    @Test
    @DisplayName("An out-of-range stored trim is clamped on load, not trusted")
    void storedTrimIsClampedOnLoad() {
        originalHoodAngleOffset = ShotCalculator.HOOD_ANGLE_OFFSET;
        originalTurretAngleOffset = ShotCalculator.TURRET_ANGLE_OFFSET;

        Preferences.setDouble(ShotCalculator.HOOD_TRIM_PREF_KEY, 250.0);
        Preferences.setDouble(ShotCalculator.TURRET_TRIM_PREF_KEY, -180.0);
        ShotCalculator.loadPersistedTrims();

        assertEquals(ShotCalculator.MAX_TRIM_DEG, ShotCalculator.HOOD_ANGLE_OFFSET, 1e-9);
        assertEquals(-ShotCalculator.MAX_TRIM_DEG, ShotCalculator.TURRET_ANGLE_OFFSET, 1e-9);

        // And the clamped value is written back, so the bad number is gone rather than waiting.
        assertEquals(
                ShotCalculator.MAX_TRIM_DEG,
                Preferences.getDouble(ShotCalculator.HOOD_TRIM_PREF_KEY, Double.NaN),
                1e-9);
    }

    /**
     * Start+Select has to clear the store as well as the fields, or the next boot brings it back.
     */
    @Test
    @DisplayName("The reset chord clears both trims and the stored copies")
    void resetClearsStoredTrims() {
        originalHoodAngleOffset = ShotCalculator.HOOD_ANGLE_OFFSET;
        originalTurretAngleOffset = ShotCalculator.TURRET_ANGLE_OFFSET;

        ShotCalculator.HOOD_ANGLE_OFFSET = 0;
        ShotCalculator.TURRET_ANGLE_OFFSET = 0;
        ShotCalculator.increaseHoodAngleOffset().initialize();
        ShotCalculator.increaseTurretAngleOffset().initialize();
        assertTrue(ShotCalculator.HOOD_ANGLE_OFFSET != 0);

        ShotCalculator.resetTrimsCommand().initialize();

        assertEquals(0, ShotCalculator.HOOD_ANGLE_OFFSET, 1e-9);
        assertEquals(0, ShotCalculator.TURRET_ANGLE_OFFSET, 1e-9);

        ShotCalculator.HOOD_ANGLE_OFFSET = 99;
        ShotCalculator.TURRET_ANGLE_OFFSET = 99;
        ShotCalculator.loadPersistedTrims();
        assertEquals(0, ShotCalculator.HOOD_ANGLE_OFFSET, 1e-9);
        assertEquals(0, ShotCalculator.TURRET_ANGLE_OFFSET, 1e-9);
    }
}
