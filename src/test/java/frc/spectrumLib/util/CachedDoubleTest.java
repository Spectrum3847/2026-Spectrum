package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class CachedDoubleTest {

    @Test
    void getAsDouble_returnsSourceValue() {
        CachedDouble cached = new CachedDouble(() -> 42.0);
        assertEquals(42.0, cached.getAsDouble(), 1e-9);
    }

    @Test
    void getAsDouble_cachesUntilPeriodic() {
        AtomicInteger callCount = new AtomicInteger(0);
        CachedDouble cached =
                new CachedDouble(
                        () -> {
                            callCount.incrementAndGet();
                            return 3.14;
                        });

        assertEquals(3.14, cached.getAsDouble(), 1e-9);
        assertEquals(3.14, cached.getAsDouble(), 1e-9);
        assertEquals(3.14, cached.getAsDouble(), 1e-9);
        assertEquals(1, callCount.get(), "source should be called only once before periodic()");
    }

    @Test
    void periodic_invalidatesCache() {
        AtomicInteger callCount = new AtomicInteger(0);
        CachedDouble cached =
                new CachedDouble(
                        () -> {
                            callCount.incrementAndGet();
                            return 1.0;
                        });

        cached.getAsDouble();
        assertEquals(1, callCount.get());

        cached.periodic();
        cached.getAsDouble();
        assertEquals(2, callCount.get(), "periodic should force re-read on next getAsDouble");
    }

    @Test
    void sourceChanges_betweenPeriodics() {
        double[] value = {10.0};
        CachedDouble cached = new CachedDouble(() -> value[0]);

        assertEquals(10.0, cached.getAsDouble(), 1e-9);
        value[0] = 20.0;
        assertEquals(
                10.0, cached.getAsDouble(), 1e-9, "cached value should not reflect source change");

        cached.periodic();
        assertEquals(
                20.0,
                cached.getAsDouble(),
                1e-9,
                "after periodic, should reflect new source value");
    }

    @Test
    void negativeValue_preserved() {
        CachedDouble cached = new CachedDouble(() -> -123.456);
        assertEquals(-123.456, cached.getAsDouble(), 1e-9);
    }

    @Test
    void zeroValue() {
        CachedDouble cached = new CachedDouble(() -> 0.0);
        assertEquals(0.0, cached.getAsDouble(), 1e-9);
    }
}
