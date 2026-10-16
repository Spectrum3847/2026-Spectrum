package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class CachedDoubleTest {

    private CachedDouble cachedDouble;

    @AfterEach
    void cleanup() {
        if (cachedDouble != null) {
            CommandScheduler.getInstance().unregisterSubsystem(cachedDouble);
            cachedDouble = null;
        }
    }

    @Test
    @DisplayName("Test CachedDouble caches value within same iteration and invalidates on periodic")
    void testCachingAndInvalidation() {
        AtomicInteger callCounter = new AtomicInteger(0);
        cachedDouble =
                new CachedDouble(
                        () -> {
                            callCounter.incrementAndGet();
                            return 42.0;
                        });

        assertEquals(0, callCounter.get());

        double val1 = cachedDouble.getAsDouble();
        assertEquals(42.0, val1, 1e-6);
        assertEquals(1, callCounter.get());

        double val2 = cachedDouble.getAsDouble();
        double val3 = cachedDouble.getAsDouble();
        assertEquals(42.0, val2, 1e-6);
        assertEquals(42.0, val3, 1e-6);
        assertEquals(1, callCounter.get());

        cachedDouble.periodic();

        double val4 = cachedDouble.getAsDouble();
        assertEquals(42.0, val4, 1e-6);
        assertEquals(2, callCounter.get());
    }
}
