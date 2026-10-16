package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

/** Shared wiring between a {@link TalonFX} and the WPILib physics sims in this package. */
public final class SimMotor {
    private SimMotor() {}

    /**
     * The motor's sim state, with {@code Orientation} flipped when the sim's travel runs opposite
     * the motor's positive direction.
     *
     * <p>The sims here are driven by {@code getMotorVoltage()} and feed position back through
     * {@code setRawRotorPosition()}. {@code Orientation} negates both as a matched pair, so
     * flipping it mirrors the whole loop and stays self-consistent.
     *
     * <p>Deliberately not derived from the motor's invert. CTRE documents {@code Orientation} as
     * mechanical linkage, and what decides the sign is the direction a given sim's geometry moves.
     * An inverted motor is fine on a sim that already runs negative and broken on one that runs
     * positive: the model clamps at its limit, feeds the same position back, and the mechanism
     * never moves. Only the mechanism's author knows which case applies, hence the explicit flag.
     *
     * @param reversedLinkage true when the sim's travel opposes the motor's positive direction
     * @return the motor's cached sim state, with orientation set
     */
    public static TalonFXSimState simState(TalonFX motor, boolean reversedLinkage) {
        TalonFXSimState simState = motor.getSimState();
        simState.Orientation =
                reversedLinkage
                        ? ChassisReference.Clockwise_Positive
                        : ChassisReference.CounterClockwise_Positive;
        return simState;
    }
}
