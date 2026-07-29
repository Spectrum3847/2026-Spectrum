package frc.spectrumLib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Caches a {@link DoubleSupplier} value so it is computed at most once per loop.
 *
 * <p>Every instance registers itself in a static list. {@link #invalidateAll()} must be called
 * exactly once per loop, at the top of {@code robotPeriodic()} alongside {@code
 * PhoenixUtil.refreshAll()}; without it every instance keeps returning the value it computed on the
 * first read.
 */
public class CachedDouble implements DoubleSupplier {

    /** Every instance created, so a single call can invalidate all of them. */
    private static final List<CachedDouble> instances = new ArrayList<>();

    private boolean cached = false;
    private double value;
    private final DoubleSupplier source;

    /**
     * Creates a CachedDouble wrapping the given supplier.
     *
     * @param source the underlying supplier whose value is cached each loop
     */
    public CachedDouble(DoubleSupplier source) {
        this.source = source;
        instances.add(this);
    }

    /**
     * Invalidates every {@code CachedDouble} so the next read re-queries its source. Call once per
     * loop.
     */
    public static void invalidateAll() {
        // Indexed loop: avoids allocating an iterator on a hot path called every loop.
        for (int i = 0; i < instances.size(); i++) {
            instances.get(i).invalidate();
        }
    }

    /**
     * @return the number of live {@code CachedDouble} instances
     */
    public static int getInstanceCount() {
        return instances.size();
    }

    /** Invalidates this cached value so the next {@link #getAsDouble()} re-queries the source. */
    public void invalidate() {
        cached = false;
    }

    /**
     * Returns the cached value of the source supplier, querying the supplier at most once per loop.
     *
     * @return the supplier's value for the current loop
     */
    @Override
    public double getAsDouble() {
        if (!cached) {
            value = source.getAsDouble();
            cached = true;
        }
        return value;
    }
}
