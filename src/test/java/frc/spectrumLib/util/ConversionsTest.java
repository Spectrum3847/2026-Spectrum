package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ConversionsTest {
    /** Verifies rp mto rps double. */
    @Test
    @DisplayName("Test RPM to RPS conversion with double")
    void testRPMtoRPSDouble() {
        assertEquals(0.0, Conversions.RPMtoRPS(0.0), 1e-6);
        assertEquals(1.0, Conversions.RPMtoRPS(60.0), 1e-6);
        assertEquals(100.0, Conversions.RPMtoRPS(6000.0), 1e-6);
        assertEquals(-50.0, Conversions.RPMtoRPS(-3000.0), 1e-6);
    }
    /** Verifies rp mto rps double supplier. */
    @Test
    @DisplayName("Test RPM to RPS conversion with DoubleSupplier")
    void testRPMtoRPSDoubleSupplier() {
        assertEquals(1.0, Conversions.RPMtoRPS(() -> 60.0), 1e-6);
        assertEquals(50.0, Conversions.RPMtoRPS(() -> 3000.0), 1e-6);
    }
    /** Verifies rp sto rpm. */
    @Test
    @DisplayName("Test RPS to RPM conversion")
    void testRPStoRPM() {
        assertEquals(0.0, Conversions.RPStoRPM(0.0), 1e-6);
        assertEquals(60.0, Conversions.RPStoRPM(1.0), 1e-6);
        assertEquals(6000.0, Conversions.RPStoRPM(100.0), 1e-6);
        assertEquals(-3000.0, Conversions.RPStoRPM(-50.0), 1e-6);
    }
}
