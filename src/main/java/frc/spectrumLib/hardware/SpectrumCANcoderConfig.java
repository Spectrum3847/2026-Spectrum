package frc.spectrumLib.hardware;

import lombok.Getter;
import lombok.Setter;

public class SpectrumCANcoderConfig {
    @Getter @Setter private int CANcoderID;
    /** Rotor turns per CANcoder turn. */
    @Getter private double rotorToSensorRatio = 1;
    /** CANcoder turns per mechanism turn. */
    @Getter private double sensorToMechanismRatio = 1;
    /** Offset applied to the reading, in rotations. */
    @Getter private double offset = 0;
    /** {@code false} builds no CANcoder, for sim or a robot without one. */
    @Getter private boolean attached = false;
    /** {@code true} makes clockwise rotation positive. */
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
