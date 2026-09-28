package frc.spectrumLib.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * Caches a DoubleSupplier so its value is computed at most once per scheduler iteration.
 * SubsystemBase registers the instance with the scheduler on construction, and that is what calls
 * periodic() to clear the cache.
 */
public class CachedDouble extends SubsystemBase implements DoubleSupplier {
    private boolean cached = false;
    private double value;
    private final DoubleSupplier source;

    public CachedDouble(DoubleSupplier source) {
        this.source = source;
    }

    /** Drops the cached value so the next {@link #getAsDouble()} reads the source again. */
    @Override
    public void periodic() {
        cached = false;
    }

    @Override
    public double getAsDouble() {
        if (!cached) {
            value = source.getAsDouble();
            cached = true;
        }
        return value;
    }
}
