package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.hardware.SpectrumCANcoder;
import frc.spectrumLib.hardware.SpectrumCANcoderConfig;
import frc.spectrumLib.telemetry.*;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import lombok.*;

public class Turret extends Mechanism {

    public static class TurretConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        @Getter private final double initPosition = 0;
        @Getter private final double presetPosition = 90;
        @Getter private double triggerTolerance = 5;
        @Getter private double unwrapTolerance = 10;

        @Getter private Rotation2d zeroOffsetFromRobotFront = Rotation2d.fromDegrees(180);

        /* Turret config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 60;
        @Getter private final double positionKp = 700;
        @Getter private final double positionKd = 25;
        @Getter private final double positionKv = 0;
        @Getter private final double positionKs = 2;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 50;
        @Getter private final double mmAcceleration = 300;
        @Getter private final double mmJerk = 1000;

        // Trapezoidal profile constraints are in mechanism rotations and mechanism
        // rotations per second
        private final TrapezoidProfile.Constraints turretConstraints;

        @Getter @Setter private double sensorToMechanismRatio = 45;
        @Getter @Setter private double rotorToSensorRatio = 1;

        /* Cancoder config settings */
        @Getter @Setter private double CANcoderRotorToSensorRatio = 5;
        // CANcoderRotorToSensorRatio / sensorToMechanismRatio;

        @Getter @Setter private double CANcoderSensorToMechanismRatio = 9;

        @Getter @Setter private double CANcoderOffset = -0.196533203125;
        @Getter @Setter private boolean CANcoderAttached = true;
        @Getter @Setter private boolean isCANcoderInverted = false;

        /* Sim Configs */
        @Getter private double intakeX = Units.inchesToMeters(105); // Vertical Center
        @Getter private double intakeY = Units.inchesToMeters(75); // Horizontal Center
        @Getter private double simRatio = 22.4;
        @Getter private double length = 1;

        public TurretConfig() {
            super("Turret", 44, Rio.CANIVORE); // Rio.CANIVORE);
            configPIDGains(0, positionKp, 0, positionKd);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(-0.54, 0.50);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(false);
            configCounterClockwise_Positive();

            turretConstraints = new TrapezoidProfile.Constraints(30, 40);
        }

        public TurretConfig modifyMotorConfig(TalonFX motor) {
            TalonFXConfigurator configurator = motor.getConfigurator();
            TalonFXConfiguration talonConfigMod = getTalonConfig();

            configurator.apply(talonConfigMod);
            talonConfig = talonConfigMod;
            return this;
        }
    }

    public enum WantedState {
        OFF,
        HOME,
        AIM_AT_TARGET,
    }

    public enum SystemState {
        OFF,
        HOME,
        AIM_AT_TARGET,
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

        public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    @SuppressWarnings("unused")
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case HOME -> SystemState.HOME;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }

        @SuppressWarnings("unused")
    private void applyStates() {
        double wantedDegrees = 0;
        switch (systemState) {
            case OFF:
                stop();
                return;
            case HOME:
                wantedDegrees = 0;
                break;
            case AIM_AT_TARGET:
                var params = ShotCalculator.getInstance().getParameters(); 
                wantedDegrees = params.turretAngle().getDegrees();
                break;
        }
        final double finalWantedDegrees = wantedDegrees;
        final double finalWantedPosition = degreesToRotations(() -> finalWantedDegrees);
        setMMPositionFoc(() -> finalWantedPosition);
    }

    @SuppressWarnings("unused")
    private TrapezoidProfile profile;
    @SuppressWarnings("unused")
    private TrapezoidProfile.State turretSetpoint;
    @SuppressWarnings("unused")
    private PositionTorqueCurrentFOC turretRequest = new PositionTorqueCurrentFOC(0);

    @Getter private TurretConfig config;
    @Getter private TurretSim sim;
    @SuppressWarnings("unused")
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
                                config.isCANcoderAttached(),
                                config.isCANcoderInverted());
                canCoder =
                        new SpectrumCANcoder(
                                44,
                                canCoderConfig,
                                motor,
                                config,
                                SpectrumCANcoder.CANCoderFeedbackType.FusedCANcoder);
            }
        }

        profile = new TrapezoidProfile(config.turretConstraints);
        turretSetpoint = new TrapezoidProfile.State();

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        Telemetry.log("Turret/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Turret/Voltage", getVoltage());
        Telemetry.log("Turret/Current", getStatorCurrent());
        Telemetry.log("Turret/PositionDegrees", getPositionDegrees());
        Telemetry.log("Turret/PositionRotations", getPositionRotations());
        Telemetry.log("Turret/VelocityRPM", getVelocityRPM());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new TurretSim(RobotSim.leftView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
            // m_CANcoder.getSimState().setRawPosition(sim.getAngleRads() / 0.202);
        }
    }

    class TurretSim extends ArmSim {
        public TurretSim(Mechanism2d mech, TalonFXSimState turretMotorSim) {
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