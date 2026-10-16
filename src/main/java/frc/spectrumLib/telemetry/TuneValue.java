package frc.spectrumLib.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import lombok.Getter;

/**
 * A numeric value that is published to SmartDashboard and can be modified at runtime without
 * redeploying code. Useful for tuning PID gains, speeds, and other constants during development.
 * Place a call to {@link #update()} or pass {@link #getSupplier()} inside a command to read the
 * latest value each loop.
 */
public class TuneValue {
    /** Current value, refreshed on each call to {@link #update()}. */
    @Getter private double value;
    /** SmartDashboard key under which this value is published and read. */
    @Getter private String name;

    public TuneValue(String name, double defaultValue) {
        SmartDashboard.putNumber(name, defaultValue);
        value = defaultValue;
        this.name = name;
    }

    /** Reads the current value from SmartDashboard and caches it in {@link #value}. */
    public Double update() {
        value = SmartDashboard.getNumber(name, value);
        return value;
    }

    public DoubleSupplier getSupplier() {
        return this::update;
    }
}
