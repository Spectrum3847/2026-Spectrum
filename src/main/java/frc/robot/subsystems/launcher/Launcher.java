package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.*;
import lombok.Getter;

public class Launcher extends Mechanism {

    public static class LauncherConfig extends Config {

        @Getter private final double idlingRPM = 700;

        /* Launcher config values */
        /**
         * Was 80, then 65 after the 2026-09-06 21:49 log showed a squeeze launch pulling the
         * battery to 8.3 V with the flywheels at 57 and 45 A supply. At 65 the shots got
         * inconsistent and balls collided in the air: supply headroom is the flywheel's recovery
         * time between balls. 75 keeps most of it.
         */
        @Getter private final double supplyCurrentLimit = 75;

        @Getter private final double statorCurrentLimit = 80;
        @Getter private final double reverseStatorCurrentLimit = -10;
        @Getter private final double lowerSupplyCurrentLimit = 40;
        @Getter private final double timeUntilLowerCurrent = 1;
        @Getter private final double nominalVoltage = 16;

        @Getter private double velocityKp = 0.5;
        @Getter private double velocityKv = 0.1425;
        @Getter private double velocityKs = 0;

        @Getter private double onTargetToleranceRPM = 200;

        @Getter private double gearRatio = 1.38;

        /* Sim Configs */
        @Getter private final double launcherX = Units.inchesToMeters(43);

        @Getter private final double launcherY = Units.inchesToMeters(53);
        @Getter private final double wheelDiameter = 4;

        /** Creates a new LauncherConfig instance. */
        public LauncherConfig() {
            super("Launcher Front Left", 15, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(gearRatio);
            configCurrentLimits(
                    supplyCurrentLimit,
                    statorCurrentLimit,
                    lowerSupplyCurrentLimit,
                    timeUntilLowerCurrent);
            configReverseTorqueCurrentLimit(reverseStatorCurrentLimit);
            configNeutralBrakeMode(false);
            configForwardVoltageLimit(nominalVoltage);
            configReverseVoltageLimit(-nominalVoltage);
            configCounterClockwise_Positive();
            // The flywheel's feedforward is fit from logs, which needs voltage on every sample.
            setFastOutputLogging(true);
            setFollowerConfigs(
                    new FollowerConfig(
                            "Launcher Front Right", 16, Rio.CANIVORE, MotorAlignmentValue.Opposed));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        OFF,
        IDLE_PREP,
        LAUNCH,
        /** Flywheel backwards, to push a ball stuck at the wheels back down during an unjam. */
        REVERSE,
        /** Fixed speed for the pose-independent set shot. */
        SET_SHOT,
    }

    public enum SystemState {
        OFF,
        IDLE_PREP,
        LAUNCH,
        REVERSE,
        SET_SHOT,
    }

    /**
     * Flywheel speed while unjamming. Modest on purpose: the reverse torque limit is only 10 A
     * stator, so this is a nudge, not a launch in the other direction.
     */
    private static final double UNJAM_RPM = -1000;

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
            case IDLE_PREP -> SystemState.IDLE_PREP;
            case LAUNCH -> SystemState.LAUNCH;
            case REVERSE -> SystemState.REVERSE;
            case SET_SHOT -> SystemState.SET_SHOT;
        };
    }

    /** Flywheel speed commanded this loop (RPM); 0 when stopped. */
    @Getter private double commandedRPM = 0;

    /** Applies the states. */
    private void applyStates() {
        double wantedRPM = 0;
        switch (systemState) {
            case OFF:
                commandedRPM = 0;
                stop();
                return;
            case IDLE_PREP:
                wantedRPM = config.getIdlingRPM();
                break;
            case LAUNCH:
                var params = ShotCalculator.getInstance().getParameters();
                wantedRPM = params.flywheelSpeed();
                break;
            case REVERSE:
                wantedRPM = UNJAM_RPM;
                break;
            case SET_SHOT:
                wantedRPM = ShotCalculator.getSetShotFlywheelRPM();
                break;
        }
        commandedRPM = wantedRPM;
        setVelocityRPM(() -> commandedRPM);
    }

    /**
     * Returns {@code true} when the flywheel is in the launch state and its measured speed is
     * within the configured tolerance of the commanded shot speed. Gates feeding into the flywheel.
     */
    public boolean isAtSpeed() {
        return (systemState == SystemState.LAUNCH || systemState == SystemState.SET_SHOT)
                && Math.abs(getVelocityRPM() - commandedRPM) <= config.getOnTargetToleranceRPM();
    }

    /**
     * Returns {@code true} when the flywheel is launching and has not drooped below the given
     * fraction of its commanded speed. Used by the feeder gate to decide whether to <em>keep</em>
     * feeding: each ball loads the flywheel, so a burst that had to re-satisfy {@link #isAtSpeed()}
     * between every ball would feed in stutters. Only droop is checked — running fast is never a
     * reason to stop feeding.
     *
     * @param fraction fraction of commanded RPM the flywheel must still be at (e.g. 0.75)
     * @return true when launching and at or above {@code fraction} of the commanded speed
     */
    public boolean isAboveSpeedFraction(double fraction) {
        return (systemState == SystemState.LAUNCH || systemState == SystemState.SET_SHOT)
                && commandedRPM > 0
                && getVelocityRPM() >= commandedRPM * fraction;
    }

    @Getter private final LauncherConfig config;

    @Getter private LauncherSim sim;

    /**
     * Creates a new Launcher instance.
     *
     * @param config the config
     */
    public Launcher(LauncherConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    /** Runs the periodic update. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        Telemetry.logState("Launcher/WantedState", wantedState);
        Telemetry.logState("Launcher/SystemState", systemState);
        // Flywheel speed stays at loop rate: spin-up and the dip as each ball passes are shot data.
        logStandard("Launcher", true, RpmLog.LOOP_DASH);
        Telemetry.log("Launcher/CommandedRPM", commandedRPM, "RPM");
        Telemetry.logDash("Launcher/AtSpeed", isAtSpeed());
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    /** Simulation init. */
    public void simulationInit() {
        if (isAttached()) {
            sim = new LauncherSim(RobotSim.leftView, motor);
        }
    }

    class LauncherSim extends RollerSim {
        /**
         * Creates a new LauncherSim instance.
         *
         * @param mech the mech
         * @param motor the motor
         */
        public LauncherSim(Mechanism2d mech, TalonFX motor) {
            super(
                    new RollerConfig(config.getWheelDiameter())
                            .setPosition(config.getLauncherX(), config.getLauncherY())
                            .setGearRatio(config.getGearRatio())
                            .setMount(Robot.getHood().getSim()),
                    mech,
                    motor,
                    config.getName());
        }
    }
}
