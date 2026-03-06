package frc.spectrumLib;

import lombok.Getter;
import lombok.Setter;

public class SpectrumCANcoderConfig {
    @Getter @Setter private int CANcoderID;
    @Getter private double rotorToSensorRatio = 1;
    @Getter private double sensorToMechanismRatio = 1;
    @Getter private double offset = 0;
    @Getter private boolean attached = false;
    @Getter private boolean inverted = false;

    public SpectrumCANcoderConfig(
            double rotorToSensorRatio,
            double sensorToMechanismRatio,
            double offset,
            boolean attached,
            boolean inverted) {
        this.rotorToSensorRatio = rotorToSensorRatio;
        this.sensorToMechanismRatio = sensorToMechanismRatio;
        this.offset = offset;
        this.attached = attached;
        this.inverted = inverted;
    }
}
