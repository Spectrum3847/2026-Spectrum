package frc.spectrumLib.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.spectrumLib.util.CanDeviceId;

/**
 * Builds TalonFX objects and applies the Spectrum defaults. Closed-loop and sensor parameters are
 * left for the application to set.
 */
public class TalonFXFactory {

    private static NeutralModeValue neutralMode = NeutralModeValue.Brake;
    private static InvertedValue invertValue = InvertedValue.CounterClockwise_Positive;
    private static double neutralDeadband = 0.04; // fraction of full output
    private static double supplyCurrentLimit = 40; // amps

    /** Utility class, not instantiable. */
    private TalonFXFactory() {}

    public static TalonFX createDefaultTalon(CanDeviceId id) {
        return createConfigTalon(id, getDefaultConfig());
    }

    public static TalonFX createConfigTalon(CanDeviceId id, TalonFXConfiguration config) {
        var talon = createTalon(id);
        StatusCode result =
                CanConfigBudget.run(
                        "Talon " + id.getDeviceNumber(),
                        timeout -> talon.getConfigurator().apply(config, timeout));
        if (!result.isOK()) {
            DriverStation.reportWarning(
                    "Could not apply config to Talon " + id.getDeviceNumber() + ": " + result,
                    false);
        }
        return talon;
    }

    /**
     * Builds a motor that mirrors the leader's output.
     *
     * @param motorAlignment {@link MotorAlignmentValue#Aligned} for a follower mounted the same way
     *     round as the leader, {@link MotorAlignmentValue#Opposed} for one mounted backwards
     */
    public static TalonFX createPermanentFollowerTalon(
            CanDeviceId followerId, TalonFX leaderTalonFX, MotorAlignmentValue motorAlignment) {
        String leaderCanBus = leaderTalonFX.getNetwork().toString();
        int leaderId = leaderTalonFX.getDeviceID();
        if (!followerId.getBus().equals(leaderCanBus)) {
            throw new IllegalArgumentException(
                    "Leader and Follower Talons must be on the same CAN bus");
        }

        TalonFXConfiguration followerConfig = getDefaultConfig();
        leaderTalonFX.getConfigurator().refresh(followerConfig);
        final TalonFX talon = createConfigTalon(followerId, followerConfig);

        talon.setControl(new Follower(leaderId, motorAlignment));
        return talon;
    }

    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = invertValue;
        config.MotorOutput.DutyCycleNeutralDeadband = neutralDeadband;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.SensorToMechanismRatio = 1;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.Audio.BeepOnBoot = true;
        config.Audio.AllowMusicDurDisable = true;
        config.Audio.BeepOnConfig = true;

        return config;
    }

    /** Creates the talon. */
    private static TalonFX createTalon(CanDeviceId id) {
        TalonFX talon = new TalonFX(id.getDeviceNumber(), new CANBus(id.getBus()));
        // Blocking, and worthless on a bus with nothing on it.
        if (!CanConfigBudget.exhausted()) {
            talon.clearStickyFaults();
        }

        return talon;
    }
}
