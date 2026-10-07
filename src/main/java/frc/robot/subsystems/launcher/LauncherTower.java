package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/** The Launcher Tower subsystem. Lifts fuel from the bed up to the launcher. */
public class LauncherTower extends Mechanism {

    public static class LauncherTowerConfig extends Config {
        /* Launcher Tower config values */
        @Getter private final double supplyCurrentLimit = 80;

        @Getter private final double statorCurrentLimit = 80;
        @Getter private final double lowerSupplyCurrentLimit = 40;
        @Getter private final double lowerSupplyCurrentTime = 1;
        @Getter private final double velocityKp = 0.1;
        @Getter private final double velocityKv = 0.0978;
        @Getter private final double velocityKs = 0;

        /** Creates a new LauncherTowerConfig instance. */
        public LauncherTowerConfig() {
            super("LauncherTower Front", 17, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configCurrentLimits(
                    supplyCurrentLimit,
                    statorCurrentLimit,
                    lowerSupplyCurrentLimit,
                    lowerSupplyCurrentTime);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
            // The tower's feedforward is fit from logs, which needs voltage on every sample.
            setFastOutputLogging(true);
            setFollowerConfigs(
                    new FollowerConfig(
                            "LauncherTower Back", 18, Rio.CANIVORE, MotorAlignmentValue.Opposed));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        OFF,
        INDEX_MAX,
        SLOW_INDEX,
        UNJAM,
    }

    public enum SystemState {
        OFF,
        INDEX_MAX,
        SLOW_INDEX,
        UNJAM,
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
            case INDEX_MAX -> SystemState.INDEX_MAX;
            case SLOW_INDEX -> SystemState.SLOW_INDEX;
            case UNJAM -> SystemState.UNJAM;
        };
    }

    /** Applies the states. */
    private void applyStates() {
        double wantedRPM = 0;
        switch (systemState) {
            case OFF:
                commandedRPM = 0;
                stop();
                return;
            case INDEX_MAX:
                wantedRPM = 4000;
                break;
            case SLOW_INDEX:
                wantedRPM = 1000;
                break;
            case UNJAM:
                wantedRPM = -1500;
                break;
        }
        commandedRPM = wantedRPM;
        setVelocityRPM(() -> commandedRPM);
    }

    /** Tower speed commanded this loop (RPM); 0 when stopped. */
    @Getter private double commandedRPM = 0;

    @Getter private final LauncherTowerConfig config;
    /**
     * Creates a new LauncherTower instance.
     *
     * @param config the config
     */
    public LauncherTower(LauncherTowerConfig config) {
        super(config);
        this.config = config;

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    /** Runs the periodic update. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        Telemetry.logState("LauncherTower/WantedState", wantedState);
        Telemetry.logState("LauncherTower/SystemState", systemState);
        // RPM at loop rate, like the launcher's: the tower's feedforward is fit from RPM and
        // CommandedRPM against the voltage that fastOutputLogging keeps at the same rate.
        logStandard("LauncherTower", false, RpmLog.LOOP);
        Telemetry.log("LauncherTower/CommandedRPM", commandedRPM, "RPM");
    }
}
