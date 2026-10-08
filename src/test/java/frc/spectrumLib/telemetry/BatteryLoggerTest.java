package frc.spectrumLib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Verifies that {@link BatteryLogger} aggregates parent keys exactly as before the key-derivation
 * cache was added, on both the first (uncached) and later (cached) reports of each channel.
 */
public class BatteryLoggerTest {

    private static final double EPS = 1e-9;

    private static void report(BatteryLogger logger) {
        logger.reportCurrentUsage("Mechanisms/Turret", 2.0);
        logger.reportCurrentUsage("Mechanisms/Hood", 1.0);
        logger.reportCurrentUsage("A/B/C", 1.0);
        logger.reportCurrentUsage("X-Y", 0.5);
        logger.reportCurrentUsage("Solo", 3.0);
        logger.reportCurrentUsage("Signed", -1.5, 0.5);
    }

    @Test
    @DisplayName("Parent keys aggregate on first and cached reports")
    void parentAggregation() {
        BatteryLogger logger = new BatteryLogger();
        logger.setEnabled(true);

        report(logger);

        // Leaves hold the latest value; parents sum their children; hyphen splits like a slash.
        assertEquals(2.0, logger.getSubsystemCurrent("Mechanisms/Turret"), EPS);
        assertEquals(1.0, logger.getSubsystemCurrent("Mechanisms/Hood"), EPS);
        assertEquals(3.0, logger.getSubsystemCurrent("Mechanisms"), EPS);
        assertEquals(1.0, logger.getSubsystemCurrent("A/B/C"), EPS);
        assertEquals(1.0, logger.getSubsystemCurrent("A/B"), EPS);
        assertEquals(1.0, logger.getSubsystemCurrent("A"), EPS);
        assertEquals(0.5, logger.getSubsystemCurrent("X-Y"), EPS);
        assertEquals(0.5, logger.getSubsystemCurrent("X"), EPS);
        assertEquals(3.0, logger.getSubsystemCurrent("Solo"), EPS);
        assertEquals(2.0, logger.getSubsystemCurrent("Signed"), EPS);
        assertEquals(9.5, logger.getTotalCurrent(), EPS);

        // Second pass hits the cached parent-key path. Leaves are overwritten (put) while parents
        // keep summing (merge) until logPower() zeroes them, matching the original behavior.
        report(logger);

        assertEquals(2.0, logger.getSubsystemCurrent("Mechanisms/Turret"), EPS);
        assertEquals(6.0, logger.getSubsystemCurrent("Mechanisms"), EPS);
        assertEquals(2.0, logger.getSubsystemCurrent("A/B"), EPS);
        assertEquals(2.0, logger.getSubsystemCurrent("A"), EPS);
        assertEquals(1.0, logger.getSubsystemCurrent("X"), EPS);
        assertEquals(19.0, logger.getTotalCurrent(), EPS);
    }

    @Test
    @DisplayName("Disabled logger records nothing")
    void disabledIsNoOp() {
        BatteryLogger logger = new BatteryLogger();
        report(logger);
        assertEquals(0.0, logger.getTotalCurrent(), EPS);
        assertEquals(0.0, logger.getSubsystemCurrent("Mechanisms"), EPS);
    }
}
