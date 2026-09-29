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
 * Wraps a CTRE CANcoder. Construction applies the magnet settings and points the supplied TalonFX's
 * feedback at the encoder.
 */
public class SpectrumCANcoder {

    @Getter private CANcoder canCoder;

    private SpectrumCANcoderConfig config;

    /** Selects how the TalonFX reads position data from the remote CANcoder. */
    public enum CANCoderFeedbackType {
        /** Position is read remotely; the motor encoder still supplies velocity. */
        RemoteCANcoder,
        /** The CANcoder and the motor encoder are fused into one high-bandwidth signal. */
        FusedCANcoder,
        /** Motor encoder is synchronized to the CANcoder position on enable. */
        SyncCANcoder,
    }

    private CANCoderFeedbackType feedbackSource = CANCoderFeedbackType.FusedCANcoder;

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
            if (canCoderResponseOK(canCoder.getConfigurator().apply(canCoderConfigs))) {
                modifyMotorConfig(motor, mechConfig);
            }
        }
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    /**
     * Points the TalonFX's feedback at this CANcoder, editing {@code mechConfig} in place. Returns
     * this, for chaining.
     */
    public SpectrumCANcoder modifyMotorConfig(TalonFX motor, Config mechConfig) {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration talonConfigMod = mechConfig.getTalonConfig();
        talonConfigMod.Feedback.FeedbackRemoteSensorID = config.getCANcoderID();
        switch (feedbackSource) {
            case RemoteCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                talonConfigMod.Feedback.FeedbackSensorSource =
                        FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        talonConfigMod.Feedback.RotorToSensorRatio = config.getRotorToSensorRatio();
        talonConfigMod.Feedback.SensorToMechanismRatio = config.getSensorToMechanismRatio();
        configurator.apply(talonConfigMod);
        mechConfig.setTalonConfig(talonConfigMod);
        return this;
    }

    /**
     * Reports whether the device accepted a config, and warns through {@link Telemetry} when it did
     * not.
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
