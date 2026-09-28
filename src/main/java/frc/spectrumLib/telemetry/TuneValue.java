package frc.spectrumLib.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import lombok.Getter;

/**
 * A double published to SmartDashboard that a driver can edit at runtime. Nothing reads it back on
 * its own, so call {@link #update()} or {@link #getSupplier()} to get the current value.
 */
public class TuneValue {
    @Getter private double value;
    /** SmartDashboard key this value is published under. */
    @Getter private String name;

    /** Publishes {@code defaultValue} to SmartDashboard under {@code name}. */
    public TuneValue(String name, double defaultValue) {
        SmartDashboard.putNumber(name, defaultValue);
        value = defaultValue;
        this.name = name;
    }

    public Double update() {
        value = SmartDashboard.getNumber(name, value);
        return value;
    }

    public DoubleSupplier getSupplier() {
        return this::update;
    }
}
