package frc.robot.subsystems.intakeExtension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import lombok.Setter;

/** The Intake Extension subsystem. Extends and retracts the fuel intake. */
public class IntakeExtension extends Mechanism {

    public static class IntakeExtensionConfig extends Config {

        @Getter private final double initPosition = 0;
        @Getter private final double triggerTolerance = 5;

        /* Intake Extension config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double maxRotations = 2.779053;
        @Getter private final double minRotations = 0.0;

        /* Positions are in percent of max rotations (0% -> 0 rotations | 100% -> max rotation) */
        @Getter private final double home = 0;
        @Getter private final double squeeze = 25;
        @Getter private final double fullOut = 100;
        @Getter private final double atPoseTolerance = 10;
        @Getter private final double springyPoseTolerance = 20;

        @Getter private final double positiveVoltageOut = 10;
        @Getter private final double negativeVoltageOut = -10;

        @Getter private final double normalCurrentLimit = 40;
        @Getter private final double normalTorqueCurrentLimit = 80;
        @Getter private final double springyModeSupplyCurrentLimit = 5;
        @Getter private final double springyModeStatorCurrentLimit = 20;

        @Getter private final double positionKp = 10;
        @Getter private final double positionKi = 0;
        @Getter private final double positionKd = 0;
        @Getter private final double positionKv = 1.0;
        @Getter private final double positionKs = 2.0;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double gearRatio = 11.25;
        @Getter private final double mmCruiseVelocity = 100;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;

        @Getter private final double sensorToMechanismRatio = 11.25;
        @Getter private final double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter private final double CANcoderRotorToSensorRatio = 1.7;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter private final double CANcoderSensorToMechanismRatio = 1;

        @Getter private final double CANcoderOffset = 0;
        @Getter private final boolean CANcoderAttached = false;

        /* Sim Configs */
        @Getter private final double intakeX = Units.inchesToMeters(70);
        @Getter private final double intakeY = Units.inchesToMeters(23);
        @Getter private final double extensionMass = 10.0;
        @Getter private final double drumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private final double extensionGearing = 11.25;
        @Getter private final double angle = 180;
        @Getter private final double staticLength = 10;
        @Getter private final double movingLength = 55;
        @Getter private final double lineWidth = 20;
        @Getter private final double maxExtensionHeight = 40;

        public IntakeExtensionConfig() {
            super("IntakeExtension", 7, Rio.CANIVORE); // Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, positionKi, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(normalCurrentLimit, true);
            configStatorCurrentLimit(normalTorqueCurrentLimit, true);
            configGearRatio(gearRatio);
            configForwardTorqueCurrentLimit(normalTorqueCurrentLimit);
            configReverseTorqueCurrentLimit(normalTorqueCurrentLimit);
            configForwardSoftLimit(maxRotations, true);
            configReverseSoftLimit(minRotations, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            setFollowerConfigs(
                new FollowerConfig(
                    "IntakeExtension Right", 6, Rio.CANIVORE, MotorAlignmentValue.Opposed)
                );
        }

        public IntakeExtensionConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        STOPPED,
        FULL_EXTEND,
        CONDITIONAL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
    }

    public enum SystemState {
        STOPPED,
        FULL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
    }

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState systemState = SystemState.STOPPED;
    private boolean sentOutByIntakeState = false;

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case STOPPED -> SystemState.STOPPED;
            case FULL_EXTEND -> {
                sentOutByIntakeState = true;
                yield SystemState.FULL_EXTEND;
            }
            case CONDITIONAL_EXTEND -> sentOutByIntakeState
                    ? SystemState.FULL_EXTEND
                    : SystemState.STOPPED;
            case FULL_RETRACT -> {
                sentOutByIntakeState = false;
                yield SystemState.FULL_RETRACT;
            }
            case SLOW_CLOSE -> SystemState.SLOW_CLOSE;
        };
    }

    private void applyStates() {
        boolean slowMove = false;
        double wantedPercent = 0;
        switch (systemState) {
            case FULL_EXTEND:
                wantedPercent = 100;
                break;
            case FULL_RETRACT:
                wantedPercent = 0;
                break;
            case SLOW_CLOSE:
                slowMove = true;
                wantedPercent = 25;
                break;
            case STOPPED:
                stop();
                return;
        }
        final double finalWantedPercent = wantedPercent;
        final double finalRotation = percentToRotations(() -> finalWantedPercent);
        if (slowMove) {
            setDynMMPositionVoltage(() -> finalRotation, () -> 4.0, () -> 20.0, () -> 1000.0);
        } else {
            setMMPosition(() -> finalRotation);
        }
    }

    @Getter private final IntakeExtensionConfig config;
    @Getter private IntakeExtensionSim sim;

    @Getter @Setter private boolean inSpringyMode = false;

    public IntakeExtension(IntakeExtensionConfig config) {
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

    public void resetCurrentPositionToMax() {
        motor.setPosition(config.getMaxRotations());
    }

    public Command resetCurrentPositionToMaxCommand() {
        return run(this::resetCurrentPositionToMax);
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("IntakeExtension/WantedState", wantedState.toString());
        Telemetry.log("IntakeExtension/SystemState", systemState.toString());
        Telemetry.log("IntakeExtension/CurrentCommand", getCurrentCommandName());
        Telemetry.log("IntakeExtension/Voltage", getVoltage(), "volts");
        Telemetry.log("IntakeExtension/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("IntakeExtension/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("IntakeExtension/Position", getPositionRotations(), "rotations");
        Telemetry.log("IntakeExtension/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("IntakeExtension/Temp", getTemp(), "deg_C");
        Telemetry.log("IntakeExtension/InSpringyMode", inSpringyMode);
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
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
