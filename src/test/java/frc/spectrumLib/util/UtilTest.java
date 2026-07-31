package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.junit.jupiter.api.Test;

class UtilTest {

    @Test
    void limit_symmetric_withinRange() {
        assertEquals(5.0, Util.limit(5.0, 10.0), 1e-9);
    }

    @Test
    void limit_symmetric_atPositiveBound() {
        assertEquals(10.0, Util.limit(10.0, 10.0), 1e-9);
    }

    @Test
    void limit_symmetric_atNegativeBound() {
        assertEquals(-10.0, Util.limit(-10.0, 10.0), 1e-9);
    }

    @Test
    void limit_symmetric_aboveMax() {
        assertEquals(10.0, Util.limit(15.0, 10.0), 1e-9);
    }

    @Test
    void limit_symmetric_belowMin() {
        assertEquals(-10.0, Util.limit(-15.0, 10.0), 1e-9);
    }

    @Test
    void limit_threeArg_withinRange() {
        assertEquals(3.0, Util.limit(3.0, 1.0, 5.0), 1e-9);
    }

    @Test
    void limit_threeArg_atMax() {
        assertEquals(5.0, Util.limit(10.0, 1.0, 5.0), 1e-9);
    }

    @Test
    void limit_threeArg_atMin() {
        assertEquals(1.0, Util.limit(0.0, 1.0, 5.0), 1e-9);
    }

    @Test
    void inRange_symmetric_inside() {
        assertTrue(Util.inRange(3.0, 10.0));
    }

    @Test
    void inRange_symmetric_atBound_false() {
        assertFalse(Util.inRange(10.0, 10.0));
        assertFalse(Util.inRange(-10.0, 10.0));
    }

    @Test
    void inRange_symmetric_outside() {
        assertFalse(Util.inRange(15.0, 10.0));
        assertFalse(Util.inRange(-15.0, 10.0));
    }

    @Test
    void inRange_threeArg_inside() {
        assertTrue(Util.inRange(3.0, 1.0, 5.0));
    }

    @Test
    void inRange_threeArg_atBound_false() {
        assertFalse(Util.inRange(1.0, 1.0, 5.0));
        assertFalse(Util.inRange(5.0, 1.0, 5.0));
    }

    @Test
    void inRange_threeArg_belowMin() {
        assertFalse(Util.inRange(0.0, 1.0, 5.0));
    }

    @Test
    void inRange_threeArg_aboveMax() {
        assertFalse(Util.inRange(6.0, 1.0, 5.0));
    }

    @Test
    void interpolate_midpoint() {
        assertEquals(5.0, Util.interpolate(0.0, 10.0, 0.5), 1e-9);
    }

    @Test
    void interpolate_start() {
        assertEquals(0.0, Util.interpolate(0.0, 10.0, 0.0), 1e-9);
    }

    @Test
    void interpolate_end() {
        assertEquals(10.0, Util.interpolate(0.0, 10.0, 1.0), 1e-9);
    }

    @Test
    void interpolate_clampBelow() {
        assertEquals(0.0, Util.interpolate(0.0, 10.0, -0.5), 1e-9);
    }

    @Test
    void interpolate_clampAbove() {
        assertEquals(10.0, Util.interpolate(0.0, 10.0, 1.5), 1e-9);
    }

    @Test
    void epsilonEquals_double_default() {
        assertTrue(Util.epsilonEquals(1.0, 1.0));
        assertTrue(Util.epsilonEquals(1.0, 1.0 + 1e-13));
        assertFalse(Util.epsilonEquals(1.0, 1.0 + 1e-11));
    }

    @Test
    void epsilonEquals_double_customEpsilon() {
        assertTrue(Util.epsilonEquals(1.0, 1.1, 0.2));
        assertFalse(Util.epsilonEquals(1.0, 1.3, 0.2));
    }

    @Test
    void epsilonEquals_int_differenceWithinEpsilon() {
        assertTrue(Util.epsilonEquals(5, 7, 3));
    }

    @Test
    void epsilonEquals_int_differenceOutsideEpsilon() {
        assertFalse(Util.epsilonEquals(5, 10, 3));
    }

    @Test
    void epsilonEquals_int_exactMatch() {
        assertTrue(Util.epsilonEquals(5, 5, 0));
    }

    @Test
    void allCloseTo_allWithinEpsilon() {
        List<Double> list = List.of(1.0, 1.0 + 1e-13, 1.0 - 1e-13);
        assertTrue(Util.allCloseTo(list, 1.0, 1e-10));
    }

    @Test
    void allCloseTo_oneOutside() {
        List<Double> list = List.of(1.0, 2.0, 1.0);
        assertFalse(Util.allCloseTo(list, 1.0, 0.5));
    }

    @Test
    void allCloseTo_emptyList_true() {
        List<Double> list = Collections.emptyList();
        assertTrue(Util.allCloseTo(list, 1.0, 1e-10));
    }

    @Test
    void joinStrings_basic() {
        assertEquals("a, b, c", Util.joinStrings(", ", List.of("a", "b", "c")));
    }

    @Test
    void joinStrings_customDelimiter() {
        assertEquals("a--b--c", Util.joinStrings("--", List.of("a", "b", "c")));
    }

    @Test
    void joinStrings_emptyList() {
        assertEquals("", Util.joinStrings(", ", new ArrayList<>()));
    }

    @Test
    void joinStrings_numbers() {
        assertEquals("1|2|3", Util.joinStrings("|", List.of(1, 2, 3)));
    }

    @Test
    void joinStrings_singleElement() {
        assertEquals("hello", Util.joinStrings(", ", List.of("hello")));
    }
}
