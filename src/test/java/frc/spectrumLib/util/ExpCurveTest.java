package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ExpCurveTest {

    @Test
    void defaultConstructor_linearPassThrough() {
        ExpCurve curve = new ExpCurve();
        assertEquals(0.5, curve.calculate(0.5), 1e-9);
        assertEquals(0.0, curve.calculate(0.0), 1e-9);
        assertEquals(-0.5, curve.calculate(-0.5), 1e-9);
    }

    @Test
    void defaultConstructor_inputAtOne() {
        ExpCurve curve = new ExpCurve();
        assertEquals(1.0, curve.calculate(1.0), 1e-9);
    }

    @Test
    void defaultConstructor_inputAtNegativeOne() {
        ExpCurve curve = new ExpCurve();
        assertEquals(-1.0, curve.calculate(-1.0), 1e-9);
    }

    @Test
    void exponentialCurve_reducesOutputs() {
        ExpCurve curve = new ExpCurve(2.0, 0.0, 1.0, 0.0);
        double result = curve.calculate(0.5);
        assertTrue(result < 0.5);
        assertTrue(result > 0.0);
    }

    @Test
    void exponentialCurve_preservesSign() {
        ExpCurve curve = new ExpCurve(2.0, 0.0, 1.0, 0.0);
        assertTrue(curve.calculate(0.5) > 0.0);
        assertTrue(curve.calculate(-0.5) < 0.0);
    }

    @Test
    void scalar_applied() {
        ExpCurve curve = new ExpCurve(1.0, 0.0, 0.5, 0.0);
        assertEquals(0.25, curve.calculate(0.5), 1e-9);
    }

    @Test
    void offset_applied() {
        ExpCurve curve = new ExpCurve(1.0, 0.2, 1.0, 0.0);
        assertEquals(0.7, curve.calculate(0.5), 1e-9);
    }

    @Test
    void deadzone_createsZeroRegion() {
        ExpCurve curve = new ExpCurve(1.0, 0.0, 1.0, 0.2);
        assertEquals(0.0, curve.calculate(0.0), 1e-9);
        assertEquals(0.0, curve.calculate(0.05), 1e-9);
    }

    @Test
    void deadzone_remapsOutside() {
        ExpCurve curve = new ExpCurve(1.0, 0.0, 1.0, 0.5);
        assertEquals(1.0, curve.calculate(1.0), 1e-9);
    }

    @Test
    void setExpVal_zeroOrNegative_clampsToOne() {
        ExpCurve curve = new ExpCurve();
        curve.setExpVal(0.0);
        assertEquals(1.0, curve.getExpVal(), 1e-9);
        curve.setExpVal(-3.0);
        assertEquals(1.0, curve.getExpVal(), 1e-9);
    }

    @Test
    void setExpVal_updatesGetter() {
        ExpCurve curve = new ExpCurve();
        curve.setExpVal(3.0);
        assertEquals(3.0, curve.getExpVal(), 1e-9);
    }

    @Test
    void getCurvePoints_returnsRequestedCount() {
        ExpCurve curve = new ExpCurve();
        double[][] points = curve.getCurvePoints(10);
        assertEquals(10, points.length);
    }

    @Test
    void getCurvePoints_range() {
        ExpCurve curve = new ExpCurve();
        double[][] points = curve.getCurvePoints(3);
        assertEquals(-1.0, points[0][0], 1e-9);
        assertEquals(0.0, points[1][0], 1e-9);
        assertEquals(1.0, points[2][0], 1e-9);
    }

    @Test
    void inherited_setOffset_getOffset() {
        ExpCurve curve = new ExpCurve();
        curve.setOffset(0.75);
        assertEquals(0.75, curve.getOffset(), 1e-9);
    }

    @Test
    void inherited_setScalar_getScalar() {
        ExpCurve curve = new ExpCurve();
        curve.setScalar(0.5);
        assertEquals(0.5, curve.getScalar(), 1e-9);
    }

    @Test
    void inherited_setDeadzone_getDeadzone() {
        ExpCurve curve = new ExpCurve();
        curve.setDeadzone(0.3);
        assertEquals(0.3, curve.getDeadzone(), 1e-9);
    }

    @Test
    void setDeadzone_abs_negativeInput() {
        ExpCurve curve = new ExpCurve();
        curve.setDeadzone(-0.4);
        assertEquals(0.4, curve.getDeadzone(), 1e-9);
    }
}
