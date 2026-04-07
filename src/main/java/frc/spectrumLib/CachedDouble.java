package frc.spectrumLib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * Caches a DoubleSupplier value so it is computed at most once per scheduler iteration. Note:
 * Subsystem periodic() is typically called after triggers are polled each iteration.
 */
public class CachedDouble extends SubsystemBase implements DoubleSupplier {
    private boolean cached = false;
    private double value;
    private final DoubleSupplier source;

    public CachedDouble(DoubleSupplier source) {
        this.source = source;
    }

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
