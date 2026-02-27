package frc.spectrumLib;

import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;

/**
 * CachedDouble allows for a value to only be checked once per periodic loop if it is called by
 * multiple methods. Periodic is run first, so the value will be updated before it is used in any
 * Triggers or Command
 */
public class CachedDouble implements DoubleSupplier, Subsystem {

    private boolean isCached;
    private double value;
    private DoubleSupplier canCall;

    public CachedDouble(DoubleSupplier canCall) {
        this.canCall = canCall;
        value = canCall.getAsDouble();
        isCached = true;
        this.register();
    }

    @Override
    public void periodic() {
        isCached = false;
    }

    @Override
    public double getAsDouble() {
        if (!isCached) {
            value = canCall.getAsDouble();
            isCached = true;
        }
        return value;
    }
}
