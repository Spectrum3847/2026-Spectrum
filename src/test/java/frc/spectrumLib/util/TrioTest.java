package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class TrioTest {

    @Test
    @DisplayName("Test Trio constructor and getters")
    void testTrioConstructorAndGetters() {
        Trio<String, Integer, Double> trio = new Trio<>("Hello", 123, 4.56);
        assertEquals("Hello", trio.getFirst());
        assertEquals(123, trio.getSecond());
        assertEquals(4.56, trio.getThird(), 1e-6);
    }

    @Test
    @DisplayName("Test Trio.of factory method")
    void testTrioOfFactory() {
        Trio<Integer, Boolean, String> trio = Trio.of(1, true, "World");
        assertEquals(1, trio.getFirst());
        assertEquals(true, trio.getSecond());
        assertEquals("World", trio.getThird());
    }

    @Test
    @DisplayName("Test Trio allows null values for its components")
    void testTrioWithNullValues() {
        Trio<String, Integer, Double> trio = new Trio<>(null, null, null);
        assertEquals(null, trio.getFirst());
        assertEquals(null, trio.getSecond());
        assertEquals(null, trio.getThird());
    }
}
