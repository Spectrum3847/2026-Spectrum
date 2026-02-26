package frc.robot.intakeExtension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.SpectrumCANcoderConfig;
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

        @Getter @Setter private double maxRotations = 4.0;
        @Getter @Setter private double minRotations = 0.0;

        @Getter private final double currentLimit = 10;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double positionKp = 250;
        @Getter private final double positionKd = 15;
        @Getter private final double positionKv = 0.15;
        @Getter private final double positionKs = 1.8;
        @Getter private final double positionKa = 2;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;

        @Getter @Setter private double sensorToMechanismRatio = 1.7;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 1.7;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 1;

        @Getter @Setter private double CANcoderOffset = 0;
        @Getter @Setter private boolean CANcoderAttached = false;

        /* Sim Configs */
        @Getter private double intakeX = Units.inchesToMeters(50);
        @Getter private double intakeY = Units.inchesToMeters(40);
        @Getter private double extensionMass = 10.0;
        @Getter private double drumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double extensionGearing = 1.7;
        @Getter private double angle = 180;
        @Getter private double staticLength = 10;
        @Getter private double movingLength = 30;
        @Getter private double lineWidth = 80;
        @Getter private double maxExtensionHeight = 40;

        public IntakeExtensionConfig() {
            super("IntakeExtension", 6, Rio.CANIVORE); // Rio.CANIVORE);
            configMinMaxRotations(minRotations, maxRotations);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
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
    private SpectrumCANcoder canCoder;
    private SpectrumCANcoderConfig canCoderConfig;
    CANcoderSimState canCoderSim;

    public IntakeExtension(IntakeExtensionConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            if (config.isCANcoderAttached() && !Robot.isSimulation()) {
                canCoderConfig =
                        new SpectrumCANcoderConfig(
                                config.getCANcoderRotorToSensorRatio(),
                                config.getCANcoderSensorToMechanismRatio(),
                                config.getCANcoderOffset(),
                                config.isCANcoderAttached());
                canCoder =
                        new SpectrumCANcoder(
                                45,
                                canCoderConfig,
                                motor,
                                config,
                                SpectrumCANcoder.CANCoderFeedbackType.FusedCANcoder);
            }

            setInitialPosition();
        }

        simulationInit();
        telemetryInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {
        IntakeExtensionStates.setupDefaultCommand();
    }

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
            builder.addDoubleProperty("Degrees", this::getPositionDegrees, null);
            builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
            builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
        }
    }

    private void setInitialPosition() {
        if (canCoder != null) {
            if (canCoder.isAttached()
                    && canCoder.canCoderResponseOK(
                            canCoder.getCanCoder().getAbsolutePosition().getStatus())) {
                motor.setPosition(
                        canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                                / config.getCANcoderSensorToMechanismRatio());
            } else {
                motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
            }
        } else {
            motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
        }
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
        return run(() -> setMMPositionFoc(rotations));
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
