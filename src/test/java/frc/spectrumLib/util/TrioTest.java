package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

class TrioTest {

    @Test
    void constructorAndGetters() {
        Trio<String, Integer, Double> trio = new Trio<>("first", 42, 3.14);
        assertEquals("first", trio.getFirst());
        assertEquals(42, trio.getSecond());
        assertEquals(3.14, trio.getThird());
    }

    @Test
    void getFirst_returnsConstructorArg() {
        Trio<Integer, String, Boolean> trio = new Trio<>(100, "mid", true);
        assertEquals(100, trio.getFirst());
    }

    @Test
    void getSecond_returnsConstructorArg() {
        Trio<Integer, String, Boolean> trio = new Trio<>(100, "mid", true);
        assertEquals("mid", trio.getSecond());
    }

    @Test
    void getThird_returnsConstructorArg() {
        Trio<Integer, String, Boolean> trio = new Trio<>(100, "mid", true);
        assertEquals(true, trio.getThird());
    }

    @Test
    void staticOf_createsTrio() {
        Trio<Double, Double, Double> trio = Trio.of(1.0, 2.0, 3.0);
        assertNotNull(trio);
        assertEquals(1.0, trio.getFirst());
        assertEquals(2.0, trio.getSecond());
        assertEquals(3.0, trio.getThird());
    }

    @Test
    void nullValues_preserved() {
        Trio<String, String, String> trio = new Trio<>(null, "b", null);
        assertEquals(null, trio.getFirst());
        assertEquals("b", trio.getSecond());
        assertEquals(null, trio.getThird());
    }

    @Test
    void mixedTypes() {
        Trio<Long, Character, Object> trio = Trio.of(123L, 'x', new Object());
        assertEquals(123L, trio.getFirst());
        assertEquals('x', trio.getSecond());
        assertNotNull(trio.getThird());
    }
}
