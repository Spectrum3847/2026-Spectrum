package frc.spectrumLib.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.spectrumLib.mechanism.Mechanism.Config;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * Wraps a CTRE CANcoder and applies Spectrum configuration. When the config says the encoder is
 * attached, the constructor also points the supplied TalonFX's feedback at it.
 */
public class SpectrumCANcoder {

    @Getter private CANcoder canCoder;

    private SpectrumCANcoderConfig config;

    /** Selects how the TalonFX reads position data from the remote CANcoder. */
    public enum CANCoderFeedbackType {
        /** Position is read remotely; motor encoder is used for velocity. */
        RemoteCANcoder(FeedbackSensorSourceValue.RemoteCANcoder),
        /** CANcoder position is fused with the motor encoder for high-bandwidth feedback. */
        FusedCANcoder(FeedbackSensorSourceValue.FusedCANcoder),
        /** Motor encoder is synchronized to the CANcoder position on enable. */
        SyncCANcoder(FeedbackSensorSourceValue.SyncCANcoder);

        public final FeedbackSensorSourceValue sensorSource;

        CANCoderFeedbackType(FeedbackSensorSourceValue sensorSource) {
            this.sensorSource = sensorSource;
        }
    }

    private CANCoderFeedbackType feedbackSource = CANCoderFeedbackType.FusedCANcoder;

    /**
     * Configures the CANcoder, and when it is attached points the motor's feedback at it. Nothing
     * else happens if the config marks the encoder unattached.
     */
    public SpectrumCANcoder(
            int CANcoderID,
            SpectrumCANcoderConfig config,
            TalonFX motor,
            Config mechConfig,
            CANCoderFeedbackType feedbackSource) {
        this.config = config;
        config.setCANcoderID(CANcoderID);
        this.feedbackSource = feedbackSource;

        if (config.isAttached()) {
            // Fused/Sync feedback requires the CANcoder to be on the same bus as the motor.
            canCoder = new CANcoder(CANcoderID, motor.getNetwork());
            CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
            canCoderConfigs.MagnetSensor.MagnetOffset = config.getOffset();
            canCoderConfigs.MagnetSensor.SensorDirection =
                    config.isInverted()
                            ? SensorDirectionValue.Clockwise_Positive
                            : SensorDirectionValue.CounterClockwise_Positive;
            canCoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            if (canCoderResponseOK(
                    CanConfigBudget.run(
                            "CANcoder " + CANcoderID,
                            timeout ->
                                    canCoder.getConfigurator().apply(canCoderConfigs, timeout)))) {
                modifyMotorConfig(motor, mechConfig);
            }
        }
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * Points the motor's feedback at this CANcoder using the configured source and ratios. Modifies
     * mechConfig's stored TalonFX config in place, then applies it to the motor.
     */
    public SpectrumCANcoder modifyMotorConfig(TalonFX motor, Config mechConfig) {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration talonConfigMod = mechConfig.getTalonConfig();
        talonConfigMod.Feedback.FeedbackRemoteSensorID = config.getCANcoderID();
        talonConfigMod.Feedback.FeedbackSensorSource = feedbackSource.sensorSource;
        talonConfigMod.Feedback.RotorToSensorRatio = config.getRotorToSensorRatio();
        talonConfigMod.Feedback.SensorToMechanismRatio = config.getSensorToMechanismRatio();
        configurator.apply(talonConfigMod);
        return this;
    }

    /**
     * True when the configurator's response came back OK, and prints the failure to {@link
     * Telemetry} when it did not.
     */
    public boolean canCoderResponseOK(StatusCode response) {
        if (!response.isOK()) {
            Telemetry.print(
                    "CANcoder ID "
                            + config.getCANcoderID()
                            + " failed config with error "
                            + response.toString());
            return false;
        }
        return true;
    }
}
