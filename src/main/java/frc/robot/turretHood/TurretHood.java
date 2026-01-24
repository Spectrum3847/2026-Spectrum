package frc.robot.turretHood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShotCalculator;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class TurretHood extends Mechanism {

    public static class TurretHoodConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        @Getter private final double initPosition = 0;
        @Getter private final double minRotations = 0;
        @Getter private final double maxRotations = 120;

        /* Turret config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 10;
        @Getter private final double torqueCurrentLimit = 100;
        @Getter private final double positionKp = 1000;
        @Getter private final double positionKd = 175;
        @Getter private final double positionKv = 0.15;
        @Getter private final double positionKs = 1.8;
        @Getter private final double positionKa = 2;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;

        @Getter @Setter private double sensorToMechanismRatio = 1;
        @Getter @Setter private double rotorToSensorRatio = 1;

         /* Sim Configs */
         @Getter private double intakeX = Units.inchesToMeters(10);
         @Getter private double intakeY = Units.inchesToMeters(10);
         @Getter private double simRatio = 1;
         @Getter private double length = 0.4;

        public TurretHoodConfig() {
            super("TurretHood", 46, Rio.CANIVORE); // Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            //configMinMaxRotations(-1, 1);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(false);
            configGravityType(true);
            configCounterClockwise_Positive();
        }

        public TurretHoodConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    private TurretHoodConfig config;
    @Getter private RotationalPivotSim sim;

    public TurretHood(TurretHoodConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
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
        TurretHoodStates.setupDefaultCommand();
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
        motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
    }

    public Command resetToInitialPos() {
        return run(this::setInitialPosition);
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------
    
    /** Holds the position of the Turret. */
    public Command runHoldHood() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("TurretHood.holdPosition");
                addRequirements(TurretHood.this);
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

    public Command trackTargetCommand() {
    return run(() -> {
        var params = ShotCalculator.getInstance().getParameters();
        moveToDegrees(() -> params.hoodAngle());
        });
    }

    @Override
    public Command moveToDegrees(DoubleSupplier degrees) {
        return super.moveToDegrees(degrees).withName(getName() + ".runPoseDegrees");
    }

    @Override
    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(getPositionDegrees() - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new RotationalPivotSim(RobotSim.leftView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
        }
    }
    class RotationalPivotSim extends ArmSim {
        public RotationalPivotSim(Mechanism2d mech, TalonFXSimState turretMotorSim) {
            super(
                    new ArmConfig(
                                    config.intakeX,
                                    config.intakeY,
                                    config.simRatio,
                                    config.length,
                                    0,
                                    120,
                                    0),
                    mech,
                    turretMotorSim,
                    config.getName());
        }
    }
}
