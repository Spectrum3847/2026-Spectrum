package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class UtilTest {
    /** Verifies limit. */
    @Test
    @DisplayName("Test limit with max magnitude and min/max bounds")
    void testLimit() {
        assertEquals(5.0, Util.limit(10.0, 5.0), 1e-6);
        assertEquals(-5.0, Util.limit(-10.0, 5.0), 1e-6);
        assertEquals(3.0, Util.limit(3.0, 5.0), 1e-6);

        assertEquals(10.0, Util.limit(15.0, 0.0, 10.0), 1e-6);
        assertEquals(0.0, Util.limit(-5.0, 0.0, 10.0), 1e-6);
        assertEquals(7.0, Util.limit(7.0, 0.0, 10.0), 1e-6);
    }
    /** Verifies in range. */
    @Test
    @DisplayName("Test inRange methods")
    void testInRange() {
        assertTrue(Util.inRange(3.0, 5.0));
        assertFalse(Util.inRange(5.0, 5.0)); // Exclusive bound
        assertFalse(Util.inRange(6.0, 5.0));

        assertTrue(Util.inRange(5.0, 0.0, 10.0));
        assertFalse(Util.inRange(0.0, 0.0, 10.0)); // Exclusive bound
        assertFalse(Util.inRange(10.0, 0.0, 10.0)); // Exclusive bound

        assertTrue(Util.inRange(() -> 5.0, () -> 0.0, () -> 10.0));
        assertFalse(Util.inRange(() -> 0.0, () -> 0.0, () -> 10.0));
    }
    /** Verifies interpolate. */
    @Test
    @DisplayName("Test interpolate lerp method")
    void testInterpolate() {
        assertEquals(10.0, Util.interpolate(10.0, 20.0, 0.0), 1e-6);
        assertEquals(15.0, Util.interpolate(10.0, 20.0, 0.5), 1e-6);
        assertEquals(20.0, Util.interpolate(10.0, 20.0, 1.0), 1e-6);

        // Clamping factor x to [0, 1]
        assertEquals(10.0, Util.interpolate(10.0, 20.0, -0.5), 1e-6);
        assertEquals(20.0, Util.interpolate(10.0, 20.0, 1.5), 1e-6);
    }
    /** Verifies join strings. */
    @Test
    @DisplayName("Test joinStrings method")
    void testJoinStrings() {
        List<String> items = List.of("apple", "banana", "cherry");
        assertEquals("apple, banana, cherry", Util.joinStrings(", ", items));
        assertEquals("apple|banana|cherry", Util.joinStrings("|", items));
    }
    /** Verifies epsilon equals. */
    @Test
    @DisplayName("Test epsilonEquals for double and integer")
    void testEpsilonEquals() {
        assertTrue(Util.epsilonEquals(1.0, 1.0000000000001, 1e-6));
        assertFalse(Util.epsilonEquals(1.0, 1.01, 1e-6));

        assertTrue(Util.epsilonEquals(1.0, 1.0 + Util.EPSILON / 2.0));
        assertFalse(Util.epsilonEquals(1.0, 1.0 + Util.EPSILON * 2.0));

        assertTrue(Util.epsilonEquals(10, 12, 2));
        assertFalse(Util.epsilonEquals(10, 13, 2));
    }
    /** Verifies all close to. */
    @Test
    @DisplayName("Test allCloseTo list method")
    void testAllCloseTo() {
        List<Double> list = List.of(1.0, 1.0001, 0.9999);
        assertTrue(Util.allCloseTo(list, 1.0, 1e-3));
        assertFalse(Util.allCloseTo(list, 1.0, 1e-5));
    }
}
