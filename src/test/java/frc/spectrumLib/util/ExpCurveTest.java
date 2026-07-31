package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ExpCurveTest {

    @Test
    @DisplayName("Test ExpCurve default constructor values")
    void testDefaultConstructor() {
        ExpCurve expCurve = new ExpCurve();
        assertEquals(1.0, expCurve.getExpVal(), 1e-6);
        assertEquals(0.0, expCurve.getOffset(), 1e-6);
        assertEquals(1.0, expCurve.getScalar(), 1e-6);
        assertEquals(0.0, expCurve.getDeadzone(), 1e-6);
    }

    @Test
    @DisplayName("Test ExpCurve parameterized constructor")
    void testParameterizedConstructor() {
        ExpCurve expCurve = new ExpCurve(2.5, 0.1, 1.2, 0.05);
        assertEquals(2.5, expCurve.getExpVal(), 1e-6);
        assertEquals(0.1, expCurve.getOffset(), 1e-6);
        assertEquals(1.2, expCurve.getScalar(), 1e-6);
        assertEquals(0.05, expCurve.getDeadzone(), 1e-6);
    }

    @Test
    @DisplayName("Test ExpCurve setExpVal non-positive handling")
    void testSetExpValNonPositive() {
        ExpCurve expCurve = new ExpCurve();
        expCurve.setExpVal(0.0);
        assertEquals(1.0, expCurve.getExpVal(), 1e-6);

        expCurve.setExpVal(-5.0);
        assertEquals(1.0, expCurve.getExpVal(), 1e-6);
    }

    @Test
    @DisplayName("Test ExpCurve calculate with linear response (expVal = 1.0)")
    void testCalculateLinear() {
        ExpCurve expCurve = new ExpCurve(1.0, 0.0, 1.0, 0.0);
        assertEquals(0.0, expCurve.calculate(0.0), 1e-6);
        assertEquals(0.5, expCurve.calculate(0.5), 1e-6);
        assertEquals(-0.5, expCurve.calculate(-0.5), 1e-6);
        assertEquals(1.0, expCurve.calculate(1.0), 1e-6);
        assertEquals(-1.0, expCurve.calculate(-1.0), 1e-6);
    }

    @Test
    @DisplayName("Test ExpCurve calculate with non-linear expVal")
    void testCalculateExponential() {
        double expVal = 3.0;
        ExpCurve expCurve = new ExpCurve(expVal, 0.0, 1.0, 0.0);

        // For input = 0.5: (3^0.5 - 1) / (3 - 1) * 1 = (sqrt(3) - 1) / 2 approx 0.366025
        double expected05 = (Math.pow(expVal, 0.5) - 1.0) / (expVal - 1.0);
        assertEquals(expected05, expCurve.calculate(0.5), 1e-6);
        assertEquals(-expected05, expCurve.calculate(-0.5), 1e-6);

        // For input = 1.0: (3^1 - 1) / 2 = 1.0
        assertEquals(1.0, expCurve.calculate(1.0), 1e-6);
        assertEquals(-1.0, expCurve.calculate(-1.0), 1e-6);
    }
}
