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

        public LauncherTowerConfig() {
            super("LauncherTower Front", 17, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
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

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case INDEX_MAX -> SystemState.INDEX_MAX;
            case SLOW_INDEX -> SystemState.SLOW_INDEX;
            case UNJAM -> SystemState.UNJAM;
        };
    }

    // TODO: test
    private void applyStates() {
        double wantedRPM = 0;
        switch (systemState) {
            case OFF:
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
        final double finalWantedRPM = wantedRPM;
        setVelocityRPM(() -> finalWantedRPM);
    }

    @Getter private final LauncherTowerConfig config;
    // @Getter private LauncherTowerSim sim;

    public LauncherTower(LauncherTowerConfig config) {
        super(config);
        this.config = config;

        // simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("LauncherTower/WantedState", wantedState.toString());
        Telemetry.log("LauncherTower/SystemState", systemState.toString());
        Telemetry.log("LauncherTower/CurrentCommand", getCurrentCommandName());
        Telemetry.log("LauncherTower/Voltage", getVoltage(), "volts");
        Telemetry.log("LauncherTower/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("LauncherTower/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("LauncherTower/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("LauncherTower/Temp", getTemp(), "deg_C");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // public void simulationInit() {
    //     if (isAttached()) {
    //         // Create a new RollerSim with the left view, the motor's sim state, and a 6 in
    // diameter
    //         sim = new LauncherTowerSim(RobotSim.topView, motor.getSimState());
    //     }
    // }

    // class LauncherTowerSim extends RollerSim {
    //     public LauncherTowerSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
    //         super(
    //                 new RollerConfig(config.getWheelDiameter())
    //                         .setPosition(config.getIntakeX(), config.getIntakeY()),
    //                 mech,
    //                 rollerMotorSim,
    //                 config.getName());
    //     }
    // }
}
