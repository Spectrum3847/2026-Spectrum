package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

class ConversionsTest {

    @Test
    void rpmToRps_positive() {
        assertEquals(1.0, Conversions.RPMtoRPS(60.0), 1e-9);
    }

    @Test
    void rpmToRps_zero() {
        assertEquals(0.0, Conversions.RPMtoRPS(0.0), 1e-9);
    }

    @Test
    void rpmToRps_negative() {
        assertEquals(-1.0, Conversions.RPMtoRPS(-60.0), 1e-9);
    }

    @Test
    void rpmToRps_fractional() {
        assertEquals(0.5, Conversions.RPMtoRPS(30.0), 1e-9);
    }

    @Test
    void rpmToRps_fromSupplier() {
        DoubleSupplier supplier = () -> 120.0;
        assertEquals(2.0, Conversions.RPMtoRPS(supplier), 1e-9);
    }

    @Test
    void rpmToRps_fromSupplier_zero() {
        DoubleSupplier supplier = () -> 0.0;
        assertEquals(0.0, Conversions.RPMtoRPS(supplier), 1e-9);
    }

    @Test
    void rpsToRpm_positive() {
        assertEquals(60.0, Conversions.RPStoRPM(1.0), 1e-9);
    }

    @Test
    void rpsToRpm_zero() {
        assertEquals(0.0, Conversions.RPStoRPM(0.0), 1e-9);
    }

    @Test
    void rpsToRpm_negative() {
        assertEquals(-60.0, Conversions.RPStoRPM(-1.0), 1e-9);
    }

    @Test
    void roundTrip_rpmToRpsToRpm() {
        double original = 1234.567;
        assertEquals(original, Conversions.RPStoRPM(Conversions.RPMtoRPS(original)), 1e-9);
    }
}
