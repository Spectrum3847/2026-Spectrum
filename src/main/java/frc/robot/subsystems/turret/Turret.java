package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.telemetry.*;
import lombok.*;

public class Turret extends Mechanism {

    public static class TurretConfig extends Config {
        @Getter @Setter private boolean reversed = false;

        @Getter private final double initPosition = 0;
        @Getter private final double triggerTolerance = 5;
        @Getter private final double unwrapTolerance = 10;
        @Getter private final double unwrapExitMargin = 45;
        @Getter private final double shootOnMoveLatencySec = 0.03;
        @Getter private final double maxOmegaForShotRotPerSec = 0.75;

        @Getter private Rotation2d zeroOffsetFromRobotFront = Rotation2d.fromDegrees(0);

        /* Turret config settings */
        @Getter private final double zeroSpeed = -0.1;
        @Getter private final double holdMaxSpeedRPM = 18;

        @Getter private final double currentLimit = 80;
        @Getter private final double supplyCurrentLowerLimit = 40;
        @Getter private final double supplyCurrentLowerTime = 1.0;
        @Getter private final double torqueCurrentLimit = 80;
        @Getter private final double positionKp = 800;
        @Getter private final double positionKi = 100;

        // required for shoot on the move capability update until line 67
        // additional current output per unit of velocity requested
        // needed because of the velocity setpoint used in the control request
        @Getter private final double positionKv = 10;

        @Getter private final double positionKs = 0.6;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 0.25;
        @Getter private final double mmAcceleration = 0.5;
        @Getter private final double mmJerk = 0;
        @Getter private final double peakVoltage = 6;

        @Getter private final double reverseLimitDegrees = -35.0;
        @Getter private final double forwardLimitDegrees = 350.0;

        @Getter private final double sensorToMechanismRatio = 39.78;

        /* Sim Configs */
        @Getter private final double turretX = Units.inchesToMeters(105); // Vertical Center

        @Getter private final double turretY = Units.inchesToMeters(75); // Horizontal Center
        @Getter private final double simRatio = sensorToMechanismRatio;
        @Getter private final double length = 1;

        /** Creates a new TurretConfig instance. */
        public TurretConfig() {
            super("Turret", 14, Rio.CANIVORE); // Rio.CANIVORE);
            configPIDGains(0, positionKp, positionKi, 0);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configForwardVoltageLimit(peakVoltage);
            configReverseVoltageLimit(-peakVoltage);
            configGearRatio(sensorToMechanismRatio);
            configSupplyCurrentLimit(currentLimit, true);
            configLowerSupplyCurrentLimit(supplyCurrentLowerLimit);
            configLowerSupplyCurrentTime(supplyCurrentLowerTime);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configMinMaxRotations(reverseLimitDegrees / 360.0, forwardLimitDegrees / 360.0);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(false);
            configClockwise_Positive();
        }
        /** Modify motor config. */
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
        IDLE,
        AIM_AT_TARGET,
    }

    public enum SystemState {
        OFF,
        IDLE,
        AIM_AT_TARGET,
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;
    /**
     * Sets the wanted state.
     *
     * @param state the wanted state
     */
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }
    /** Handles the state transition. */
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case IDLE -> SystemState.IDLE;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
        };
    }
    // Whether the turret is unwrapping to avoid wire wrap.
    @Getter private boolean unwrapping = false;

    @Getter private int unwrapTargetN = 0;
    @Getter private double commandedDegrees = 0;
    @Getter private double mechOmegaRotPerSec = 0;

    /** Applies the states. */
    private void applyStates() {
        switch (systemState) {
            case OFF:
                unwrapping = false;
                stop();
                return;
            case IDLE:
                unwrapping = false;
                commandedDegrees = 0;
                mechOmegaRotPerSec = 0;
                setPosition(() -> degreesToRotations(() -> 0.0));
                return;
            case AIM_AT_TARGET:
                applyAimAtTarget();
                return;
        }
    }

    @Getter private final TurretConfig config;

    @Getter private TurretSim sim;

    /**
     * Creates a new Turret instance.
     *
     * @param config the config
     */
    public Turret(TurretConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            setInitialPosition();
        }

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }
    /** Runs the periodic update. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        logBatteryUsage();
        applyStates();
        Telemetry.log("Turret/WantedState", wantedState.toString());
        Telemetry.log("Turret/SystemState", systemState.toString());
        Telemetry.log("Turret/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Turret/Voltage", getVoltage(), "volts");
        Telemetry.log("Turret/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Turret/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Turret/Temp", getTemp(), "deg_C");
        Telemetry.log("Turret/CommandedDegrees", commandedDegrees, "deg");
        Telemetry.log("Turret/PositionDegrees", getPositionDegrees(), "deg");
        Telemetry.log("Turret/PositionError", commandedDegrees - getPositionDegrees(), "deg");
        Telemetry.log("Turret/CommandedRotPerSec", mechOmegaRotPerSec, "rot/sec");
        Telemetry.log("Turret/Unwrapping", unwrapping);
        Telemetry.log("Turret/ReadyToShoot", isReadyToShoot());
    }
    /** Sets the initial position. */
    private void setInitialPosition() {
        motor.setPosition(degreesToRotations(() -> config.getInitPosition()));
    }
    /** Applies the aim at target. */
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
        mechOmegaRotPerSec = params.turretAngularVelocity() - robotOmegaRotPerSec;

        if (unwrapping) {
            // Motion magic for smooth full-turn slew to the opposite winding, so the cable never
            // binds
            final double unwrapRot = degreesToRotations(() -> commandedDegrees);
            setMMPosition(() -> unwrapRot);
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
        setPositionWithVelocity(() -> posRot, () -> ffRps);
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
            int nTarget = Math.max(nMin, Math.min(unwrapTargetN, nMax));
            chosen = desiredMechDegrees + nTarget * 360.0;
            if (nMin == nMax || Math.abs(currentDeg - chosen) <= config.getUnwrapExitMargin()) {
                unwrapping = false;
            }
            return chosen;
        }

        // Proactive unwrap: trigger only if the nearest command is crowding a soft limit and the
        // opposite winding is reachable, then commit to that winding. Unwinding by a single turn
        // clears the limit; slewing all the way to nMin/nMax would land on the opposite limit and
        // immediately re-trigger an unwrap back the other way.
        if (nMin != nMax) {
            if (maxDeg - chosen <= config.getUnwrapTolerance() && (n - 1) >= nMin) {
                unwrapping = true;
                unwrapTargetN = n - 1;
                chosen = desiredMechDegrees + unwrapTargetN * 360.0;
            } else if (chosen - minDeg <= config.getUnwrapTolerance() && (n + 1) <= nMax) {
                unwrapping = true;
                unwrapTargetN = n + 1;
                chosen = desiredMechDegrees + unwrapTargetN * 360.0;
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
    /** Simulation init. */
    private void simulationInit() {
        if (isAttached()) {
            sim = new TurretSim(RobotSim.topView, motor);
        }
    }

    class TurretSim extends ArmSim {
        /**
         * Creates a new TurretSim instance.
         *
         * @param mech the mech
         * @param motor the motor
         */
        public TurretSim(Mechanism2d mech, TalonFX motor) {
            super(
                    new ArmConfig(
                                    config.turretX,
                                    config.turretY,
                                    config.simRatio,
                                    config.length,
                                    -360,
                                    360,
                                    0)
                            .setSimulatedGravity(false),
                    mech,
                    motor,
                    config.getName());
        }
    }
}
