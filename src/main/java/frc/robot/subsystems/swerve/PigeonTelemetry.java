package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.spectrumLib.hardware.CanConfigBudget;
import frc.spectrumLib.telemetry.Telemetry;

/**
 * Logs the Pigeon 2 directly, under {@code Swerve/Pigeon/*}, wpilog only.
 *
 * <p>Before this the only heading in the log was the pose's, which vision corrects, so a gyro that
 * reads 355 degrees per turn could not be measured from a log. The set here is chosen to be enough
 * to build on later without another round of logging: raw yaw for the gyro scalar, body rates and
 * tilt for tip and bump detection, and acceleration to set against the wheel speeds in {@code
 * Swerve/State/MeasuredStates} for traction control.
 *
 * <p>Two tiers. The fast signals are logged every loop. The slow ones ride the 10 Hz tick. Yaw and
 * yaw rate already run at the odometry rate and are only read here, never re-rated, since lowering
 * them would slow odometry. Construct this <em>after</em> {@code optimizeBusUtilization()}, which
 * silences every Pigeon signal the drivetrain does not use.
 */
class PigeonTelemetry {
    private static final String PREFIX = "Swerve/Pigeon/";

    /** Above the 50 Hz loop so every loop reads a fresh sample, and the Pigeon's own default. */
    private static final double FAST_HZ = 100;

    /** Matches the slow logging tier. */
    private static final double SLOW_HZ = 10;

    private final BaseStatusSignal yaw;
    private final BaseStatusSignal yawRate;
    private final BaseStatusSignal pitch;
    private final BaseStatusSignal roll;
    private final BaseStatusSignal pitchRate;
    private final BaseStatusSignal rollRate;
    private final BaseStatusSignal accelX;
    private final BaseStatusSignal accelY;
    private final BaseStatusSignal accelZ;
    private final BaseStatusSignal[] fastSignals;

    private final BaseStatusSignal gravityX;
    private final BaseStatusSignal gravityY;
    private final BaseStatusSignal gravityZ;
    private final BaseStatusSignal temperature;
    private final BaseStatusSignal supplyVoltage;
    private final BaseStatusSignal upTime;
    private final BaseStatusSignal faultField;
    private final BaseStatusSignal stickyFaultField;
    private final BaseStatusSignal[] slowSignals;

    PigeonTelemetry(Pigeon2 pigeon) {
        yaw = pigeon.getYaw(false);
        yawRate = pigeon.getAngularVelocityZWorld(false);
        pitch = pigeon.getPitch(false);
        roll = pigeon.getRoll(false);
        pitchRate = pigeon.getAngularVelocityYWorld(false);
        rollRate = pigeon.getAngularVelocityXWorld(false);
        accelX = pigeon.getAccelerationX(false);
        accelY = pigeon.getAccelerationY(false);
        accelZ = pigeon.getAccelerationZ(false);
        fastSignals =
                new BaseStatusSignal[] {
                    yaw, yawRate, pitch, roll, pitchRate, rollRate, accelX, accelY, accelZ
                };

        gravityX = pigeon.getGravityVectorX(false);
        gravityY = pigeon.getGravityVectorY(false);
        gravityZ = pigeon.getGravityVectorZ(false);
        temperature = pigeon.getTemperature(false);
        supplyVoltage = pigeon.getSupplyVoltage(false);
        upTime = pigeon.getUpTime(false);
        faultField = pigeon.getFaultField(false);
        stickyFaultField = pigeon.getStickyFaultField(false);
        slowSignals =
                new BaseStatusSignal[] {
                    gravityX,
                    gravityY,
                    gravityZ,
                    temperature,
                    supplyVoltage,
                    upTime,
                    faultField,
                    stickyFaultField
                };

        // Each call can wait out a CAN timeout, so a dead bus skips them, as it skips
        // optimizeBusUtilization(); the signals then keep their default rates.
        if (!CanConfigBudget.exhausted()) {
            BaseStatusSignal.setUpdateFrequencyForAll(
                    FAST_HZ, pitch, roll, pitchRate, rollRate, accelX, accelY, accelZ);
            BaseStatusSignal.setUpdateFrequencyForAll(SLOW_HZ, slowSignals);
        }
    }

    /** Call once per loop. */
    void log() {
        BaseStatusSignal.refreshAll(fastSignals);
        // Continuous, not wrapped: ten turns read about 3600, which is the gyro scalar test.
        Telemetry.log(PREFIX + "Yaw", yaw.getValueAsDouble(), "degrees");
        Telemetry.log(PREFIX + "YawRate", yawRate.getValueAsDouble(), "degrees per second");
        Telemetry.log(PREFIX + "Pitch", pitch.getValueAsDouble(), "degrees");
        Telemetry.log(PREFIX + "Roll", roll.getValueAsDouble(), "degrees");
        Telemetry.log(PREFIX + "PitchRate", pitchRate.getValueAsDouble(), "degrees per second");
        Telemetry.log(PREFIX + "RollRate", rollRate.getValueAsDouble(), "degrees per second");
        // In g, gravity included, on the Pigeon's own axes until a mount pose is configured.
        Telemetry.log(PREFIX + "AccelX", accelX.getValueAsDouble(), "g");
        Telemetry.log(PREFIX + "AccelY", accelY.getValueAsDouble(), "g");
        Telemetry.log(PREFIX + "AccelZ", accelZ.getValueAsDouble(), "g");
        Telemetry.log(PREFIX + "Connected", yaw.getStatus().isOK());

        if (Telemetry.slowLogThisLoop()) {
            BaseStatusSignal.refreshAll(slowSignals);
            Telemetry.log(PREFIX + "GravityX", gravityX.getValueAsDouble());
            Telemetry.log(PREFIX + "GravityY", gravityY.getValueAsDouble());
            Telemetry.log(PREFIX + "GravityZ", gravityZ.getValueAsDouble());
            Telemetry.log(PREFIX + "Temperature", temperature.getValueAsDouble(), "celsius");
            Telemetry.log(PREFIX + "SupplyVoltage", supplyVoltage.getValueAsDouble(), "volts");
            // Drops back to zero when the Pigeon reboots, say on a brownout.
            Telemetry.log(PREFIX + "UpTime", upTime.getValueAsDouble(), "seconds");
            Telemetry.log(PREFIX + "FaultField", (long) faultField.getValueAsDouble());
            Telemetry.log(PREFIX + "StickyFaultField", (long) stickyFaultField.getValueAsDouble());
        }
    }
}
