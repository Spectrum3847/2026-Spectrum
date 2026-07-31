package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CanDeviceIdTest {

    @Test
    void constructorWithBus() {
        CanDeviceId id = new CanDeviceId(5, "canivore");
        assertEquals(5, id.getDeviceNumber());
        assertEquals("canivore", id.getBus());
    }

    @Test
    void constructorDefaultBus() {
        CanDeviceId id = new CanDeviceId(3);
        assertEquals(3, id.getDeviceNumber());
        assertEquals("", id.getBus());
    }

    @Test
    void equals_sameDeviceSameBus() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        CanDeviceId b = new CanDeviceId(7, "rio");
        assertTrue(a.equals(b));
    }

    @Test
    void equals_differentDeviceSameBus() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        CanDeviceId b = new CanDeviceId(8, "rio");
        assertFalse(a.equals(b));
    }

    @Test
    void equals_sameDeviceDifferentBus() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        CanDeviceId b = new CanDeviceId(7, "canivore");
        assertFalse(a.equals(b));
    }

    @Test
    void equals_null_false() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        assertFalse(a.equals((Object) null));
    }

    @Test
    void equals_sameInstance_true() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        assertTrue(a.equals(a));
    }

    @Test
    void equals_differentType_false() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        assertFalse(a.equals("not a CanDeviceId"));
    }

    @Test
    void equals_typedOverload() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        CanDeviceId b = new CanDeviceId(7, "rio");
        CanDeviceId c = new CanDeviceId(8, "rio");
        assertTrue(a.equals(b));
        assertFalse(a.equals(c));
    }

    @Test
    void hashCode_equalInstances_equalHash() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        CanDeviceId b = new CanDeviceId(7, "rio");
        assertEquals(a.hashCode(), b.hashCode());
    }

    @Test
    void hashCode_differentBus_differentHash() {
        CanDeviceId a = new CanDeviceId(7, "rio");
        CanDeviceId b = new CanDeviceId(7, "canivore");
        assertNotEquals(a.hashCode(), b.hashCode());
    }

    @Test
    void equals_nullBus_same() {
        CanDeviceId a = new CanDeviceId(5, null);
        CanDeviceId b = new CanDeviceId(5, null);
        assertTrue(a.equals(b));
    }

    @Test
    void equals_nullBus_vs_nonNull() {
        CanDeviceId a = new CanDeviceId(5, null);
        CanDeviceId b = new CanDeviceId(5, "rio");
        assertFalse(a.equals(b));
    }

    @Test
    void equals_nonNullBus_vs_null() {
        CanDeviceId a = new CanDeviceId(5, "rio");
        CanDeviceId b = new CanDeviceId(5, null);
        assertFalse(a.equals(b));
    }
}
