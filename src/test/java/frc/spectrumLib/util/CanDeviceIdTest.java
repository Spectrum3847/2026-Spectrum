package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class CanDeviceIdTest {

    @Test
    @DisplayName("Test CanDeviceId constructor and getters")
    void testConstructorAndGetters() {
        CanDeviceId id1 = new CanDeviceId(5);
        assertEquals(5, id1.getDeviceNumber());
        assertEquals("", id1.getBus());

        CanDeviceId id2 = new CanDeviceId(10, "rio");
        assertEquals(10, id2.getDeviceNumber());
        assertEquals("rio", id2.getBus());
    }

    @Test
    @DisplayName("Test CanDeviceId equality and hashCode")
    void testEqualsAndHashCode() {
        CanDeviceId id1 = new CanDeviceId(1, "canivore");
        CanDeviceId id2 = new CanDeviceId(1, "canivore");
        CanDeviceId id3 = new CanDeviceId(1, "rio");
        CanDeviceId id4 = new CanDeviceId(2, "canivore");

        assertEquals(id1, id2);
        assertTrue(id1.equals(id2));
        assertEquals(id1.hashCode(), id2.hashCode());

        assertNotEquals(id1, id3);
        assertNotEquals(id1, id4);
        assertNotEquals(id1, null);
        assertNotEquals(id1, "not a CanDeviceId");
    }
}
