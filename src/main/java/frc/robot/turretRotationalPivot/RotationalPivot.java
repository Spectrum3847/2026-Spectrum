package frc.robot.turretRotationalPivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumCANcoder;
import frc.spectrumLib.SpectrumCANcoderConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import java.util.function.DoubleSupplier;
import lombok.*;

public class RotationalPivot extends Mechanism {

    public static class RotationalPivotConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        @Getter private final double initPosition = 0;
        @Getter private final double presetPosition = 90;
        @Getter private double triggerTolerance = 5;
        @Getter private double unwrapTolerance = 10;

        /* Turret config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 60;
        @Getter private final double positionKp = 850;
        @Getter private final double positionKd = 20;
        @Getter private final double positionKv = 0.2;
        @Getter private final double positionKs = 0.6;
        @Getter private final double positionKa = 2;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;

        @Getter private final double visionTrackingKp = 15;
        @Getter private final double maxTrackingRPS = 0.5;

        @Getter @Setter private double sensorToMechanismRatio = 45;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 5;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 9;

        @Getter @Setter private double CANcoderOffset = -0.196533203125;
        @Getter @Setter private boolean CANcoderAttached = true;

        /* Sim Configs */
        @Getter private double intakeX = Units.inchesToMeters(105); // Vertical Center
        @Getter private double intakeY = Units.inchesToMeters(75); // Horizontal Center
        @Getter private double simRatio = 22.4;
        @Getter private double length = 1;

        public RotationalPivotConfig() {
            super("Turret", 44, Rio.CANIVORE); // Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-0.475, 0.535); // 363.6° range
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(false);
            configClockwise_Positive();
        }

        public RotationalPivotConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    @Getter private RotationalPivotConfig config;
    @Getter private RotationalPivotSim sim;
    private SpectrumCANcoder canCoder;
    private SpectrumCANcoderConfig canCoderConfig;
    CANcoderSimState canCoderSim;

    public RotationalPivot(RotationalPivotConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            setInitialPosition();
            if (config.isCANcoderAttached() && !Robot.isSimulation()) {
                canCoderConfig = new SpectrumCANcoderConfig(
                        config.getCANcoderRotorToSensorRatio(),
                        config.getCANcoderSensorToMechanismRatio(),
                        config.getCANcoderOffset(),
                        config.isCANcoderAttached());
                canCoder = new SpectrumCANcoder(
                        44,
                        canCoderConfig,
                        motor,
                        config,
                        SpectrumCANcoder.CANCoderFeedbackType.FusedCANcoder);
            }
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
        RotationalPivotStates.setupDefaultCommand();
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
    
    // Choose the best equivalent in degrees that lies inside the configured soft-limits.
    // If no equivalent exists in the soft-limit window (soft window < 360°), clamp to nearest endpoint.
    private double wrapDegreesToSoftLimits(double targetDegrees) {

        double minDeg = config.getMinRotations() * 360.0;
        double maxDeg = config.getMaxRotations() * 360.0;
        double currentDeg = getPositionDegrees();

        // Solve for integer n such that minDeg <= targetDegrees + 360*n <= maxDeg
        int nMin = (int) Math.ceil((minDeg - targetDegrees) / 360.0);
        int nMax = (int) Math.floor((maxDeg - targetDegrees) / 360.0);

        if (nMin <= nMax) {
            // At least one equivalent fits in soft limits.
            int nClosest = (int) Math.round((currentDeg - targetDegrees) / 360.0);
            int n = Math.max(nMin, Math.min(nClosest, nMax)); // clamp the closest candidate to allowed range
            return targetDegrees + n * 360.0;
        } else {
            // No equivalent fits in soft limits -> clamp to nearest soft limit endpoint.
            double toMin = Math.abs(currentDeg - minDeg);
            double toMax = Math.abs(currentDeg - maxDeg);
            return (toMin < toMax) ? minDeg : maxDeg;
        }
    }
    
    public void aimFieldRelative(Rotation2d fieldAngle) {
        double robotHeadingDeg = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        double turretDeg = fieldAngle.getDegrees() - robotHeadingDeg;
        final double wrappedTurretDeg = wrapDegreesToSoftLimits(turretDeg);

        setDynMMPositionFoc(
                () -> degreesToRotations(() -> wrappedTurretDeg),
                () -> config.getMmCruiseVelocity(),
                () -> config.getMmAcceleration(),
                () -> config.getMmJerk());
    }

    public Command trackTargetCommand() {
        return run(() -> {
            var params = ShotCalculator.getInstance().getParameters();
            aimFieldRelative(params.turretAngle());
        }).withName("Turret.trackTargetCommand");
    }

    /** Holds the position of the Turret. */
    public Command runHoldTurret() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Turret.holdPosition");
                addRequirements(RotationalPivot.this);
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
        return super.moveToDegrees(() -> wrapDegreesToSoftLimits(degrees.getAsDouble())).withName(getName() + ".runPoseDegrees");
    }

    @Override
    public Trigger atDegrees(DoubleSupplier target, DoubleSupplier tolerance) {
        return new Trigger(
                () ->
                        Math.abs(((getPositionDegrees() % 360) + 360) % 360 - target.getAsDouble())
                                < tolerance.getAsDouble());
    }

    public Trigger aimingAtTarget() {
        return new Trigger(() -> {
            var params = ShotCalculator.getInstance().getParameters();

            Rotation2d targetRotation = params.turretAngle();
            Rotation2d currentRotation = Rotation2d.fromDegrees(getPositionDegrees());

            double errorDeg = currentRotation.minus(targetRotation).getDegrees();

            return Math.abs(errorDeg) < 3;
        });
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new RotationalPivotSim(RobotSim.topView, motor.getSimState());

            // m_CANcoder.setPosition(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
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
                                    -720,
                                    720,
                                    0),
                    mech,
                    turretMotorSim,
                    config.getName());
        }
    }
}

