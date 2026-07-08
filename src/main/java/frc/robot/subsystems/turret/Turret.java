package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.hardware.SpectrumCANcoder;
import frc.spectrumLib.hardware.SpectrumCANcoderConfig;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.telemetry.*;
import lombok.*;

public class Turret extends Mechanism {

    public static class TurretConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        @Getter private final double initPosition = 0;
        @Getter private double triggerTolerance = 5;
        @Getter private double unwrapTolerance = 10;
        @Getter private double unwrapExitMargin = 45;
        @Getter private double shootOnMoveLatencySec = 0.03;
        @Getter private double maxOmegaForShotRotPerSec = 0.75;

        @Getter private Rotation2d zeroOffsetFromRobotFront = Rotation2d.fromDegrees(180);

        /* Turret config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 30;
        @Getter private final double torqueCurrentLimit = 60;
        @Getter private final double positionKp = 700;
        @Getter private final double positionKd = 25;

        // TODO: required for shoot on the move capability
        // additional current output per unit of velocity requested
        // needed because of the velocity setpoint used in the control request
        @Getter private final double positionKv = 0;

        @Getter private final double positionKs = 2;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 2.5;
        @Getter private final double mmAcceleration = 20;
        @Getter private final double mmJerk = 200;

        @Getter private final double sensorToMechanismRatio = 45;
        @Getter private final double rotorToSensorRatio = 1;
        @Getter private final double CANcoderRotorToSensorRatio = 5;
        @Getter private final double CANcoderSensorToMechanismRatio = 9;
        @Getter private final double CANcoderOffset = -0.196533203125;
        @Getter private final boolean CANcoderAttached = true;
        @Getter private final boolean isCANcoderInverted = false;

        /* Sim Configs */
        @Getter private double intakeX = Units.inchesToMeters(105); // Vertical Center
        @Getter private double intakeY = Units.inchesToMeters(75); // Horizontal Center
        @Getter private double simRatio = sensorToMechanismRatio;
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

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case HOME -> SystemState.HOME;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }

    @Getter private boolean unwrapping = false;
    @Getter private int unwrapDir = 0;
    @Getter private double commandedDegrees = 0;
    @Getter private double mechOmegaRotPerSec = 0;

    private void applyStates() {
        switch (systemState) {
            case OFF:
                unwrapping = false;
                stop();
                return;
            case HOME:
                unwrapping = false;
                commandedDegrees = 0;
                mechOmegaRotPerSec = 0;
                setMMPositionFoc(() -> degreesToRotations(() -> 0.0));
                return;
            case AIM_AT_TARGET:
                applyAimAtTarget();
                return;
        }
    }

    @Getter private TurretConfig config;
    @Getter private TurretSim sim;
    @Getter private SpectrumCANcoder canCoder;
    @Getter private SpectrumCANcoderConfig canCoderConfig;

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
            setInitialPosition();
        }

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        logBatteryUsage();
        applyStates();
        Telemetry.log("Turret/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Turret/Voltage", getVoltage());
        Telemetry.log("Turret/Current", getStatorCurrent());
        Telemetry.log("Turret/PositionDegrees", getPositionDegrees());
        Telemetry.log("Turret/PositionRotations", getPositionRotations());
        Telemetry.log("Turret/VelocityRPM", getVelocityRPM());
        Telemetry.log("Turret/CommandedDegrees", commandedDegrees);
        Telemetry.log("Turret/MechOmegaRotPerSec", mechOmegaRotPerSec);
        Telemetry.log("Turret/Unwrapping", unwrapping);
        Telemetry.log("Turret/ReadyToShoot", isReadyToShoot());
    }

    private void setInitialPosition() {
        if (canCoder != null) {
            if (canCoder.isAttached()
                    && canCoder.canCoderResponseOK(
                            canCoder.getCanCoder().getAbsolutePosition().getStatus())) {
                motor.setPosition(
                        (canCoder.getCanCoder().getAbsolutePosition().getValueAsDouble()
                                / config.getCANcoderSensorToMechanismRatio()));
            } else {
                motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
            }
        } else {
            motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
        }
    }

    private void applyAimAtTarget() {
        var params = ShotCalculator.getInstance().getParameters();

        // Convert FIELD-RELATIVE angle to MECHANISM-RELATIVE angle
        double robotHeadingDeg = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        double desiredMechDegrees =
                params.turretAngle().getDegrees()
                        - robotHeadingDeg
                        - config.getZeroOffsetFromRobotFront().getDegrees();

        double commanded = resolveTurretAngle(desiredMechDegrees);
        commandedDegrees = commanded;

        // Counter-rotate for the robot's own spin, so mechOmega = fieldOmega - robotOmega
        ChassisSpeeds robotSpeeds = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        double robotOmegaRotPerSec = robotSpeeds.omegaRadiansPerSecond / (2.0 * Math.PI);
        mechOmegaRotPerSec = params.turretAngularVelocityRotPerSec() - robotOmegaRotPerSec;

        if (unwrapping) {
            // Motion magic for smooth full-turn slew to the opposite winding, so the cable never
            // binds
            final double unwrapRot = degreesToRotations(() -> commandedDegrees);
            setMMPositionFoc(() -> unwrapRot);
            return;
        }

        // Lead the moving target by the actuation latency
        double minDeg = config.getMinRotations() * 360.0;
        double maxDeg = config.getMaxRotations() * 360.0;
        double predictedDegrees =
                MathUtil.clamp(
                        commanded
                                + (mechOmegaRotPerSec * 360.0) * config.getShootOnMoveLatencySec(),
                        minDeg,
                        maxDeg);

        final double posRot = degreesToRotations(() -> predictedDegrees);
        final double ffRps = mechOmegaRotPerSec;
        setPositionFocWithVelocity(() -> posRot, () -> ffRps);
    }

    /**
     * Picks the physically-equivalent turret angle (target direction ± whole turns) that best fits
     * the limited travel range, and drives the proactive cable-unwrap hysteresis.
     */
    private double resolveTurretAngle(double desiredMechDegrees) {
        double minDeg = config.getMinRotations() * 360.0;
        double maxDeg = config.getMaxRotations() * 360.0;
        double currentDeg = getPositionDegrees();

        int nMin = (int) Math.ceil((minDeg - desiredMechDegrees) / 360.0);
        int nMax = (int) Math.floor((maxDeg - desiredMechDegrees) / 360.0);

        if (nMin > nMax) {
            unwrapping = false;
            return (Math.abs(currentDeg - minDeg) < Math.abs(currentDeg - maxDeg))
                    ? minDeg
                    : maxDeg;
        }

        int nClosest = (int) Math.round((currentDeg - desiredMechDegrees) / 360.0);
        int n = Math.max(nMin, Math.min(nClosest, nMax));
        double chosen = desiredMechDegrees + n * 360.0;

        // While unwrapping, hold the committed winding until we physically arrive, so the direction
        // can't flip mid-slew as the current position crosses the halfway point.
        if (unwrapping) {
            int nTarget = (unwrapDir < 0) ? nMin : nMax;
            chosen = desiredMechDegrees + nTarget * 360.0;
            if (nMin == nMax || Math.abs(currentDeg - chosen) <= config.getUnwrapExitMargin()) {
                unwrapping = false;
            }
            return chosen;
        }

        // Proactive unwrap: trigger only if the nearest command is crowding a soft limit and the
        // opposite winding is reachable, then commit to that winding.
        if (nMin != nMax) {
            if (maxDeg - chosen <= config.getUnwrapTolerance() && (n - 1) >= nMin) {
                unwrapping = true;
                unwrapDir = -1;
                chosen = desiredMechDegrees + (n - 1) * 360.0;
            } else if (chosen - minDeg <= config.getUnwrapTolerance() && (n + 1) <= nMax) {
                unwrapping = true;
                unwrapDir = +1;
                chosen = desiredMechDegrees + (n + 1) * 360.0;
            }
        }
        return chosen;
    }

    /**
     * @return true when the turret is aiming, on target within tolerance, slewing slowly enough for
     *     a stable shot, and not mid-unwrap. Use this to gate shooting while on the move.
     */
    public boolean isReadyToShoot() {
        return systemState == SystemState.AIM_AT_TARGET
                && !unwrapping
                && Math.abs(getPositionDegrees() - commandedDegrees) <= config.getTriggerTolerance()
                && Math.abs(mechOmegaRotPerSec) <= config.getMaxOmegaForShotRotPerSec();
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void simulationInit() {
        if (isAttached()) {
            sim = new TurretSim(RobotSim.topView, motor.getSimState());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (isAttached()) {
            sim.simulationPeriodic();
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
                                    0)
                            .setSimulatedGravity(false),
                    mech,
                    turretMotorSim,
                    config.getName());
        }
    }
}
