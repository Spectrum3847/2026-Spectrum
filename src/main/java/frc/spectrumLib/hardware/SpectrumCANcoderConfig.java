package frc.spectrumLib.hardware;

import lombok.Getter;
import lombok.Setter;

/** Configuration parameters for a {@link SpectrumCANcoder}. */
public class SpectrumCANcoderConfig {
    /** CAN device ID, set separately from this object so a caller can read it back out. */
    @Getter @Setter private int CANcoderID;
    /** Gear ratio between the motor rotor and the CANcoder shaft (rotor turns / sensor turn). */
    @Getter private double rotorToSensorRatio = 1;
    /**
     * Gear ratio between the CANcoder shaft and the mechanism output (sensor turns / mechanism
     * turn).
     */
    @Getter private double sensorToMechanismRatio = 1;
    /** Magnetic offset applied to the CANcoder reading, in rotations. */
    @Getter private double offset = 0;
    /** Whether the CANcoder hardware is physically present on the robot. */
    @Getter private boolean attached = false;
    /** Whether the CANcoder sensor direction is inverted (clockwise positive). */
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
