package frc.robot.intakeExtension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class IntakeExtension extends Mechanism {

    public static class IntakeExtensionConfig extends Config {

        @Getter private final double initPosition = 0;
        @Getter private double triggerTolerance = 5;

        /* Intake Extension config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter @Setter private double maxRotations = 2.779053;
        @Getter @Setter private double minRotations = 0.0;

        /* Positions are in percent of max rotations (0% -> 0 rotations | 100% -> max rotation) */
        @Getter private double home = 0;
        @Getter private double squeeze = 25;
        @Getter private double fullOut = 100;
        @Getter private double atPoseTolerance = 10;

        @Getter
        private final DoubleSubscriber timeUntilIntakeSqueeze =
                Telemetry.tunable("Tunable/TimeUntilIntakeSqueeze", 0.5);

        @Getter private final double currentLimit = 40;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double positionKp = 10;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 0;
        @Getter private final double positionKv = 0.5;
        @Getter private final double positionKs = 1.5;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double gearRatio = 11.25;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 200;
        @Getter private final double mmJerk = 1000;

        @Getter @Setter private double sensorToMechanismRatio = 11.25;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 1.7;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 1;

        @Getter @Setter private double CANcoderOffset = 0;
        @Getter @Setter private boolean CANcoderAttached = false;

        /* Sim Configs */
        @Getter private double intakeX = Units.inchesToMeters(70);
        @Getter private double intakeY = Units.inchesToMeters(23);
        @Getter private double extensionMass = 10.0;
        @Getter private double drumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double extensionGearing = 1.7;
        @Getter private double angle = 180;
        @Getter private double staticLength = 10;
        @Getter private double movingLength = 55;
        @Getter private double lineWidth = 20;
        @Getter private double maxExtensionHeight = 40;

        public IntakeExtensionConfig() {
            super("Intake Extension", 7, Rio.CANIVORE); // Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configGearRatio(gearRatio);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(-1 * torqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }

        public IntakeExtensionConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    @Getter private IntakeExtensionConfig config;
    @Getter private IntakeExtensionSim sim;

    public IntakeExtension(IntakeExtensionConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            setInitialPosition();
        }

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        logBatteryUsage();
        Telemetry.log("IntakeExtension/CurrentCommand", getCurrentCommandName());
        Telemetry.log("IntakeExtension/Voltage", getVoltage(), "volts");
        Telemetry.log("IntakeExtension/Current", getStatorCurrent(), "amps");
        Telemetry.log("IntakeExtension/Position", getPositionRotations(), "rotations");
        Telemetry.log("IntakeExtension/RPM", getVelocityRPM(), "RPM");
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        IntakeExtensionStates.setupDefaultCommand();
    }

    private void setInitialPosition() {
        motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
    }

    public void resetCurrentPositionToMax() {
        motor.setPosition(config.getMaxRotations());
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the Intake Extension. */
    public Command runHoldIntakeExtension() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("IntakeExtension.holdPosition");
                addRequirements(IntakeExtension.this);
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

    public Command move(DoubleSupplier rotations) {
        return run(() -> setVoltageOutput(rotations));
    }

    public Command voltageOutPositive() {
        return run(() -> setVoltageOutput(() -> 8)).withTimeout(2);
    }

    public Command voltageOutNegative() {
        return run(() -> setVoltageOutput(() -> -8)).withTimeout(2);
    }

    public Command motionMagicPercentMove(DoubleSupplier percent) {
        return run(() -> setMMPosition(() -> percentToRotations(percent)));
    }

    public Command slowMoveToPercent(DoubleSupplier percent) {
        return run(
                () ->
                        setDynMMPositionVoltage(
                                () -> percentToRotations(percent), () -> 5, () -> 20, () -> 1000));
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new IntakeExtensionSim(RobotSim.leftView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }

    class IntakeExtensionSim extends LinearSim {
        public IntakeExtensionSim(Mechanism2d mech, TalonFXSimState intakeExtensionMotorSim) {
            super(
                    new LinearConfig(
                                    config.getIntakeX(),
                                    config.getIntakeY(),
                                    config.getExtensionGearing(),
                                    config.getDrumRadiusMeters())
                            .setAngle(config.getAngle())
                            .setMovingLength(config.getMovingLength())
                            .setStaticLength(config.getStaticLength())
                            .setMaxHeight(config.getMaxExtensionHeight())
                            .setLineWidth(config.getLineWidth())
                            .setColor(new Color8Bit(Color.kLightGray)),
                    mech,
                    intakeExtensionMotorSim,
                    config.getName());
        }
    }
}
