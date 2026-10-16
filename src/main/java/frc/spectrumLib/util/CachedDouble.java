package frc.spectrumLib.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * Caches a DoubleSupplier so it runs at most once per scheduler iteration. The scheduler polls
 * triggers before it calls subsystem periodic(), which is what makes a per-iteration cache safe.
 */
public class CachedDouble extends SubsystemBase implements DoubleSupplier {
    private boolean cached = false;
    private double value;
    private final DoubleSupplier source;

    public CachedDouble(DoubleSupplier source) {
        this.source = source;
    }

    /**
     * Called by the scheduler each iteration. Clears the cache so the next {@link #getAsDouble()}
     * re-queries the source.
     */
    @Override
    public void periodic() {
        cached = false;
    }

    /** The source's value for the current iteration, read at most once per iteration. */
    @Override
    public double getAsDouble() {
        if (!cached) {
            value = source.getAsDouble();
            cached = true;
        }
        return value;
    }
}
