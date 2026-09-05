package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.spectrumLib.telemetry.Telemetry;

/**
 * Publishes the raw per-module CANcoder data that swerve alignment needs.
 *
 * <p>CTRE's swerve telemetry only reports module angles <em>after</em> the magnet offset has been
 * applied, so there is no way to recover the number that belongs in {@code
 * SwerveConfig.configEncoderOffsets(...)} from a log or a dashboard. This class fills that gap: it
 * publishes each module's offset-applied position, the magnet offset actually programmed into the
 * device, and the difference between them, which is the "Absolute Position No Offset" value Phoenix
 * Tuner X shows.
 *
 * <p>The alignment web app in {@code tools/swerve-align} subscribes to these keys over
 * NetworkTables. See {@code docs/tools/swerve-alignment.md}.
 */
public class SwerveAlignment {

    /** NetworkTables/DogLog key prefix for everything this class publishes. */
    public static final String KEY_PREFIX = "Swerve/Align/";

    /** Module names, in the order {@link SwerveConfig#getModules()} returns them. */
    public static final String[] MODULE_NAMES = {
        "FrontLeft", "FrontRight", "BackLeft", "BackRight"
    };

    /** Rate at which the CANcoder signals are refreshed and published, in Hz. */
    private static final double PUBLISH_HZ = 20.0;

    /** Robot loops between publishes. The main loop runs at 50 Hz. */
    private static final int LOOPS_PER_PUBLISH = (int) Math.round(50.0 / PUBLISH_HZ);

    /** One module's encoder signals plus the offsets we want to compare against. */
    private static class ModuleEncoder {
        private final String name;
        private final int encoderId;
        private final StatusSignal<Angle> absolutePosition;
        private final StatusSignal<AngularVelocity> velocity;
        private final double configOffsetRotations;
        private double appliedOffsetRotations;
        private boolean appliedOffsetValid;

        private ModuleEncoder(String name, CANcoder encoder, double configOffsetRotations) {
            this.name = name;
            this.encoderId = encoder.getDeviceID();
            this.absolutePosition = encoder.getAbsolutePosition();
            this.velocity = encoder.getVelocity();
            this.configOffsetRotations = configOffsetRotations;

            // Swerve calls optimizeBusUtilization() before constructing us, which silences every
            // signal it does not use itself. Ask for these two back explicitly.
            BaseStatusSignal.setUpdateFrequencyForAll(PUBLISH_HZ, absolutePosition, velocity);

            readBackAppliedOffset(encoder);
        }

        /**
         * Reads the magnet offset that is actually programmed into the device.
         *
         * <p>This is deliberately a readback rather than a copy of the compiled-in constant. If
         * someone has written a magnet offset into the CANcoder with Tuner X, which {@code
         * docs/tools/phoenix-tuner-x.md} tells us not to do, or if the robot is running an older
         * deploy than the source being edited, the readback and the constant disagree and the
         * alignment app can say so instead of baking the discrepancy into the new offsets.
         *
         * @param encoder the CANcoder to read from
         */
        private void readBackAppliedOffset(CANcoder encoder) {
            CANcoderConfiguration deviceConfig = new CANcoderConfiguration();
            StatusCode status = encoder.getConfigurator().refresh(deviceConfig);
            if (status.isOK()) {
                appliedOffsetRotations = deviceConfig.MagnetSensor.MagnetOffset;
                appliedOffsetValid = true;
            } else {
                appliedOffsetRotations = configOffsetRotations;
                appliedOffsetValid = false;
                Telemetry.print(
                        "SwerveAlignment: could not read the magnet offset from CANcoder "
                                + encoderId
                                + " ("
                                + status
                                + "), falling back to the configured value");
            }
        }
    }

    private final ModuleEncoder[] moduleEncoders;
    private final BaseStatusSignal[] allSignals;
    private int loopCount = 0;

    /**
     * Creates the alignment publisher.
     *
     * <p>Construct this <em>after</em> {@code optimizeBusUtilization()} so the update frequencies
     * it requests are not immediately thrown away.
     *
     * @param modules the drivetrain's modules, in {@link SwerveConfig#getModules()} order
     * @param config the swerve config the modules were built from
     */
    public SwerveAlignment(SwerveModule<?, ?, CANcoder>[] modules, SwerveConfig config) {
        double[] configOffsets = {
            config.getFrontLeftEncoderOffset().in(Rotations),
            config.getFrontRightEncoderOffset().in(Rotations),
            config.getBackLeftEncoderOffset().in(Rotations),
            config.getBackRightEncoderOffset().in(Rotations)
        };

        int count = Math.min(modules.length, MODULE_NAMES.length);
        moduleEncoders = new ModuleEncoder[count];
        allSignals = new BaseStatusSignal[count * 2];
        for (int i = 0; i < count; i++) {
            moduleEncoders[i] =
                    new ModuleEncoder(MODULE_NAMES[i], modules[i].getEncoder(), configOffsets[i]);
            allSignals[i * 2] = moduleEncoders[i].absolutePosition;
            allSignals[i * 2 + 1] = moduleEncoders[i].velocity;
        }
    }

    /** Refreshes and publishes the alignment data. Safe to call every robot loop. */
    public void log() {
        if (loopCount++ % LOOPS_PER_PUBLISH != 0) {
            return;
        }

        BaseStatusSignal.refreshAll(allSignals);

        for (ModuleEncoder module : moduleEncoders) {
            String key = KEY_PREFIX + module.name + "/";
            double absoluteRotations =
                    wrapRotations(module.absolutePosition.getValue().in(Rotations));
            double rawRotations = wrapRotations(absoluteRotations - module.appliedOffsetRotations);

            Telemetry.log(key + "AbsoluteRotations", absoluteRotations);
            Telemetry.log(key + "RawRotations", rawRotations);
            Telemetry.log(key + "AppliedOffset", module.appliedOffsetRotations);
            Telemetry.log(key + "AppliedOffsetValid", module.appliedOffsetValid);
            Telemetry.log(key + "ConfigOffset", module.configOffsetRotations);
            Telemetry.log(key + "SteerVelocity", module.velocity.getValue().in(RotationsPerSecond));
            Telemetry.log(key + "Connected", module.absolutePosition.getStatus().isOK());
            Telemetry.log(key + "EncoderId", (long) module.encoderId);
        }

        // Lets the web app tell "the numbers are not moving" apart from "the robot went away".
        Telemetry.log(KEY_PREFIX + "Timestamp", Timer.getFPGATimestamp());
    }

    /**
     * Wraps a rotation count into {@code [-0.5, 0.5)}, matching how CTRE reports CANcoder absolute
     * position.
     *
     * @param rotations the value to wrap
     * @return the equivalent value in {@code [-0.5, 0.5)}
     */
    public static double wrapRotations(double rotations) {
        return rotations - Math.floor(rotations + 0.5);
    }
}
