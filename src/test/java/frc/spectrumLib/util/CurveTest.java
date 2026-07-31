package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class CurveTest {

    private static class TestCurve extends Curve {
        @Override
        public double calculate(double input) {
            double deadbandVal = calculateDeadzone(input);
            double scaledVal = calculateScalar(deadbandVal);
            return calculateOffset(scaledVal);
        }
    }

    @Test
    @DisplayName("Test Curve getters and setters")
    void testGettersAndSetters() {
        Curve curve = new TestCurve();
        curve.setOffset(2.5);
        assertEquals(2.5, curve.getOffset(), 1e-6);

        curve.setScalar(1.5);
        assertEquals(1.5, curve.getScalar(), 1e-6);

        curve.setDeadzone(0.1);
        assertEquals(0.1, curve.getDeadzone(), 1e-6);

        curve.setDeadzone(-0.2); // Negative deadzone should be converted with Math.abs
        assertEquals(0.2, curve.getDeadzone(), 1e-6);
    }

    @Test
    @DisplayName("Test Curve deadzone calculation")
    void testDeadzoneCalculation() {
        Curve curve = new TestCurve();
        curve.setDeadzone(0.2); // deadRadius = 0.1
        curve.setScalar(1.0);
        curve.setOffset(0.0);

        // Within deadband -> 0.0
        assertEquals(0.0, curve.calculate(0.0), 1e-6);
        assertEquals(0.0, curve.calculate(0.05), 1e-6);
        assertEquals(0.0, curve.calculate(-0.05), 1e-6);

        // At boundary
        assertEquals(0.0, curve.calculate(0.1), 1e-6);
        assertEquals(0.0, curve.calculate(-0.1), 1e-6);

        // Outside deadband (squished range)
        // (1.0 / (1.0 - 0.1)) * (0.55 - 0.1) = (1/0.9) * 0.45 = 0.5
        assertEquals(0.5, curve.calculate(0.55), 1e-6);
        assertEquals(-0.5, curve.calculate(-0.55), 1e-6);
        assertEquals(1.0, curve.calculate(1.0), 1e-6);
        assertEquals(-1.0, curve.calculate(-1.0), 1e-6);
    }

    @Test
    @DisplayName("Test Curve scalar and offset")
    void testScalarAndOffset() {
        Curve curve = new TestCurve();
        curve.setDeadzone(0.0);
        curve.setScalar(2.0);
        curve.setOffset(1.0);

        assertEquals(1.0, curve.calculate(0.0), 1e-6);
        assertEquals(3.0, curve.calculate(1.0), 1e-6);
        assertEquals(-1.0, curve.calculate(-1.0), 1e-6);
    }

    @Test
    @DisplayName("Test getCurvePoints generation")
    void testGetCurvePoints() {
        Curve curve = new TestCurve();
        curve.setDeadzone(0.0);
        curve.setScalar(1.0);
        curve.setOffset(0.0);

        double[][] points = curve.getCurvePoints(5);
        assertEquals(5, points.length);
        assertEquals(-1.0, points[0][0], 1e-6);
        assertEquals(-1.0, points[0][1], 1e-6);

        assertEquals(-0.5, points[1][0], 1e-6);
        assertEquals(-0.5, points[1][1], 1e-6);

        assertEquals(0.0, points[2][0], 1e-6);
        assertEquals(0.0, points[2][1], 1e-6);

        assertEquals(0.5, points[3][0], 1e-6);
        assertEquals(0.5, points[3][1], 1e-6);

        assertEquals(1.0, points[4][0], 1e-6);
        assertEquals(1.0, points[4][1], 1e-6);
    }
}
