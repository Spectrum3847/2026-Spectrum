package frc.spectrumLib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import lombok.Getter;

// Use this class to create a SmartDashboard tunable value
// You can put this in a Command to get the value from the SmartDashboard
public class TuneValue {
    @Getter private double value;
    @Getter private String name;

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
