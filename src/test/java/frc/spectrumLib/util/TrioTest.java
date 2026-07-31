package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class TrioTest {
    /** Verifies trio constructor and getters. */
    @Test
    @DisplayName("Test Trio constructor and getters")
    void testTrioConstructorAndGetters() {
        Trio<String, Integer, Double> trio = new Trio<>("Hello", 123, 4.56);
        assertEquals("Hello", trio.getFirst());
        assertEquals(123, trio.getSecond());
        assertEquals(4.56, trio.getThird(), 1e-6);
    }
    /** Verifies trio of factory. */
    @Test
    @DisplayName("Test Trio.of factory method")
    void testTrioOfFactory() {
        Trio<Integer, Boolean, String> trio = Trio.of(1, true, "World");
        assertEquals(1, trio.getFirst());
        assertEquals(true, trio.getSecond());
        assertEquals("World", trio.getThird());
    }
}
