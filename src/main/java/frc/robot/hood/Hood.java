package frc.robot.hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.rebuilt.ShotCalculator;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Hood extends Mechanism {

    public static class HoodConfig extends Config {

        /* Hood Voltages and Current */
        @Getter @Setter private double hoodVoltageOut = 6;
        @Getter @Setter private double hoodTorqueCurrent = 30;

        /* Hood config values */
        @Getter private final double currentLimit = 80;
        @Getter private final double torqueCurrentLimit = 180;
        @Getter private final double positionKp = 5;
        @Getter private final double positionKd = 0;
        @Getter private final double positionKv = 0.3;
        @Getter private final double positionKs = 1.5;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 200;
        @Getter private final double mmJerk = 1000;
        @Getter private final double holdMaxSpeedRPM = 18;

        /* Sim Configs */
        @Getter @Setter private double hoodX = Units.inchesToMeters(45);
        @Getter @Setter private double hoodY = Units.inchesToMeters(70);
        @Getter @Setter private double wheelDiameter = 2;

        public HoodConfig() {
            super("Hood", 49, Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }
    }

    private HoodConfig config;

    public Hood(HoodConfig config) {
        super(config);
        this.config = config;

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        logBatteryUsage();
        Telemetry.log("Hood/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Hood/Voltage", getVoltage());
        Telemetry.log("Hood/Current", getStatorCurrent());
        Telemetry.log("Hood/PositionDegrees", getPositionDegrees());
        Telemetry.log("Hood/RPM", getVelocityRPM());
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        HoodStates.setupDefaultCommand();
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    public Command runTorqueFOC(DoubleSupplier torque) {
        return run(() -> setTorqueCurrentFoc(torque));
    }

    public void setVoltageAndCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supply, DoubleSupplier torque) {
        setVoltageOutput(voltage);
        setCurrentLimits(supply, torque);
    }

    public Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return runVoltage(voltage).alongWith(runCurrentLimits(supplyCurrent, torqueCurrent));
    }

    public Command runTCcurrentLimits(DoubleSupplier torqueCurrent, DoubleSupplier supplyCurrent) {
        return runTorqueCurrentFoc(torqueCurrent)
                .alongWith(runCurrentLimits(supplyCurrent, torqueCurrent));
    }

    public Command trackTargetCommand() {
        return run(() -> {
                    var params = ShotCalculator.getInstance().getParameters();
                    setMMPosition(() -> degreesToRotations(() -> params.hoodAngle()));
                })
                .withName("Hood.trackTargetCommand");
    }

    /** Holds the position of the Hood. */
    public Command runHoldHood() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("IntakeExtension.holdPosition");
                addRequirements(Hood.this);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }

            @Override
            public void initialize() {
                holdPosition = getPositionRotations();
                stop();
            }

            @Override
            public void execute() {
                if (Math.abs(getVelocityRPM()) > config.holdMaxSpeedRPM) {
                    stop();
                    holdPosition = getPositionRotations();
                } else {
                    setDynMMPositionFoc(
                            () -> holdPosition,
                            () -> config.getMmCruiseVelocity(),
                            () -> config.getMmAcceleration(),
                            () -> 20);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command stopMotor() {
        return run(() -> stop());
    }
}
