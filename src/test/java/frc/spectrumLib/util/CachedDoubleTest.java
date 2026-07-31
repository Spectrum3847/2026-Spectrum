package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class CachedDoubleTest {

    @Test
    @DisplayName("Test CachedDouble caches value within same iteration and invalidates on periodic")
    void testCachingAndInvalidation() {
        AtomicInteger callCounter = new AtomicInteger(0);
        CachedDouble cachedDouble =
                new CachedDouble(
                        () -> {
                            callCounter.incrementAndGet();
                            return 42.0;
                        });

        assertEquals(0, callCounter.get());

        // First call evaluates the supplier
        double val1 = cachedDouble.getAsDouble();
        assertEquals(42.0, val1, 1e-6);
        assertEquals(1, callCounter.get());

        // Subsequent calls return cached value without querying supplier again
        double val2 = cachedDouble.getAsDouble();
        double val3 = cachedDouble.getAsDouble();
        assertEquals(42.0, val2, 1e-6);
        assertEquals(42.0, val3, 1e-6);
        assertEquals(1, callCounter.get());

        // Scheduler periodic call invalidates cache
        cachedDouble.periodic();

        // Next getAsDouble queries the supplier again
        double val4 = cachedDouble.getAsDouble();
        assertEquals(42.0, val4, 1e-6);
        assertEquals(2, callCounter.get());
    }

    @Test
    @DisplayName("Test CachedDouble reflects updated source value only after periodic invalidation")
    void testCachedValueUpdatesAcrossIterations() {
        AtomicInteger sourceValue = new AtomicInteger(1);
        CachedDouble cachedDouble = new CachedDouble(() -> sourceValue.get());

        assertEquals(1.0, cachedDouble.getAsDouble(), 1e-6);

        // Changing the underlying source without calling periodic() should not affect the cache.
        sourceValue.set(2);
        assertEquals(1.0, cachedDouble.getAsDouble(), 1e-6);

        // Once periodic() invalidates the cache, the new source value is picked up.
        cachedDouble.periodic();
        assertEquals(2.0, cachedDouble.getAsDouble(), 1e-6);

        sourceValue.set(3);
        cachedDouble.periodic();
        assertEquals(3.0, cachedDouble.getAsDouble(), 1e-6);
    }
}
