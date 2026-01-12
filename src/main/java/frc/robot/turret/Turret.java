package frc.robot.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.SpectrumCANcoderConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import java.util.function.DoubleSupplier;
import lombok.*;

public class Turret extends Mechanism {

    public static class TurretConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        // Positions set as percentage of Turret
        @Getter private final int initializedPosition = 20;

        @Getter private final double initPosition = 0;
        @Getter private double triggerTolerance = 5;
        @Getter private double turretTolerance = 45;

        /* Turret config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 10;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double positionKp = 1000;
        @Getter private final double positionKd = 70;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 1.8;
        @Getter private final double positionKa = 2;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 4.2;
        @Getter private final double mmAcceleration = 32;
        @Getter private final double mmJerk = 0;

        @Getter @Setter private double sensorToMechanismRatio = 22.4;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 22.4;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 1;

        @Getter @Setter private double CANcoderOffset = 0;
        @Getter @Setter private boolean CANcoderAttached = false;

        /* Sim properties */

        public TurretConfig() {
            super("Turret", 44, Rio.CANIVORE); // Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-.25, 0.75);
            configReverseSoftLimit(getMinRotations(), false);
            configForwardSoftLimit(getMaxRotations(), false);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(true);
            configClockwise_Positive();
        }

        public TurretConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    @Getter private TurretConfig config;
    private SpectrumCANcoder canCoder;
    private SpectrumCANcoderConfig canCoderConfig;
    CANcoderSimState canCoderSim;

    public Turret(TurretConfig config) {
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
                                44,
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
        TurretStates.setupDefaultCommand();
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
            // builder.addDoubleProperty("Velocity", this::getVelocityRPM, null);
            builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
            builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
            // builder.addDoubleProperty("Front-TX", Robot.getVision().frontLL::getTagTx, null);
            // builder.addDoubleProperty("Front-TA", Robot.getVision().frontLL::getTagTA, null);
            // builder.addDoubleProperty(
            //        "Front-Rotation", Robot.getVision().frontLL::getTagRotationDegrees, null);
            // builder.addDoubleProperty(
            //        "Front-ClosestTag", Robot.getVision().frontLL::getClosestTagID, null);
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

    /** Holds the position of the Turret. */
    public Command runHoldTurret() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Turret.holdPosition");
                addRequirements(Turret.this);
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

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(degrees).withName(getName() + ".runPoseDegrees");
    }

    private void setDegrees(DoubleSupplier degrees) {
        setMMPositionFoc(() -> degreesToRotations(degrees));
    }

    public Command move(DoubleSupplier targetDegrees, boolean clockwise) {
        return run(
                () -> {
                    double currentDegrees = getPositionDegrees();
                    // Normalize targetDegrees to be within 0 to 360
                    double target = (targetDegrees.getAsDouble() % 360);
                    // Normalize currentDegrees to be within 0 to 360
                    double currentMod = (currentDegrees % 360);
                    if (currentMod < 0) {
                        currentMod += 360;
                    }

                    double output = currentDegrees;

                    if (Math.abs(currentMod - target)
                            < config.getTurretTolerance()) { // check if the difference is within
                        // tolerance
                        if (currentMod - target > 0) {
                            output = currentDegrees - (currentMod - target);
                        } else {
                            output = currentDegrees + (target - currentMod);
                        }
                    } else if (Math.abs(currentMod - target) > 360 - config.getTurretTolerance()
                            && Math.abs(Math.abs(currentMod - target) - 360)
                                    < config.getTurretTolerance()) { // check if the difference is
                        // within tolerance and currentMod or target is near 0 and 360
                        if (currentMod < target) {
                            if (currentMod - (target - 360) > 0) {
                                output = currentDegrees + (currentMod - (target - 360));
                            } else {
                                output = currentDegrees - (currentMod - (target - 360));
                            }
                        } else {
                            if ((currentMod - 360) - target > 0) {
                                output = currentDegrees + ((currentMod - 360) - target);
                            } else {
                                output = currentDegrees - ((currentMod - 360) - target);
                            }
                        }
                    } else {
                        if (clockwise) {
                            // Calculate the closest clockwise position
                            if (currentMod > target) {
                                output = currentDegrees - (currentMod - target);
                            } else if (currentMod < target) {
                                output = currentDegrees - (360 + currentMod - target);
                            }
                        } else {
                            // Calculate the closest counterclockwise position
                            if (currentMod < target) {
                                output = currentDegrees + (target - currentMod);
                            } else if (currentMod > target) {
                                output = currentDegrees + (360 + target - currentMod);
                            }
                        }
                    }

                    final double out = output;
                    setDegrees(() -> out);
                });
    }

    @Override
    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(((getPositionDegrees() % 360) + 360) % 360 - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
        //     sim = new TurretSim(motor.getSimState(), RobotSim.leftView, this);

            // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            // sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        }
    }
}
