package frc.robot.hood;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.rebuilt.ShotCalculator;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Hood extends Mechanism {

    public static class HoodConfig extends Config {

        @Getter private final double initPosition = 9;

        /* Hood Voltages and Current */
        @Getter @Setter private double hoodVoltageOut = 6;
        @Getter @Setter private double hoodTorqueCurrent = 30;

        @Getter @Setter private double maxRotations = 0.137;
        @Getter @Setter private double minRotations = 0.024;

        @Getter @Setter private double autoTrenchShot = 25.0;

        @Getter
        private final DoubleSubscriber onTheFlyAngle = Telemetry.tunable("Hood/OnTheFlyAngle", 9.0);

        /* Hood config values */
        @Getter private final double currentLimit = 40;
        @Getter private final double torqueCurrentLimit = 60;
        @Getter private final double positionKp = 3000;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 220;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 15;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;

        @Getter private final double gearRatio = 51.667;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 200;
        @Getter private final double mmJerk = 1000;
        @Getter private final double holdMaxSpeedRPM = 18;

        /* Sim Configs */
        @Getter private double hoodX = Units.inchesToMeters(62.5);
        @Getter private double hoodY = Units.inchesToMeters(50);
        @Getter private double simRatio = 5;
        @Getter private double length = Units.inchesToMeters(10);

        public HoodConfig() {
            super("Hood", 15, Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(gearRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
        }
    }

    private HoodConfig config;
    @Getter private HoodSim sim;

    public Hood(HoodConfig config) {
        super(config);
        this.config = config;

        setInitialPosition();

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    private void setInitialPosition() {
        if (isAttached()) {
            motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
        }
    }

    @Override
    public void periodic() {
        logBatteryUsage();
        Telemetry.log("Hood/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Hood/Voltage", getVoltage(), "volts");
        Telemetry.log("Hood/Current", getStatorCurrent(), "amps");
        Telemetry.log("Hood/PositionDegrees", getPositionDegrees(), "degrees");
        Telemetry.log("Hood/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Hood/Temp", getTemp(), "deg_C");
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
                    setMMPositionFoc(() -> degreesToRotations(() -> params.hoodAngle()));
                })
                .withName("Hood.trackTargetCommand");
    }

    public Command moveToDegrees(double degrees) {
        return run(() -> setMMPositionFoc(() -> degreesToRotations(() -> degrees)));
    }

    public Command onTheFlyLaunch() {
        return run(() -> {
                    setMMPositionFoc(() -> config.getOnTheFlyAngle().get());
                })
                .withName("Launcher.onTheFlyLaunch");
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

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new HoodSim(RobotSim.leftView, motor.getSimState());
        }
    }

    // Must be called to enable the simulation
    // if roller position changes configure x and y to set position.
    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class HoodSim extends ArmSim {
        public HoodSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new ArmConfig(
                            config.hoodX,
                            config.hoodY,
                            config.simRatio,
                            config.length,
                            90,
                            180,
                            90),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}
