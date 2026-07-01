package frc.robot.subsystems.spindexer;

import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import lombok.Setter;

public class Spindexer extends Mechanism {
    
    public static class SpindexerConfig extends Config {

        @Getter @Setter private double supplyCurrentLimit;
        @Getter @Setter private double statorCurrentLimit;
        @Getter @Setter private double velocityKp;
        @Getter @Setter private double velocityKv;
        @Getter @Setter private double velocityKs;

        public SpindexerConfig() {
            super("Spindexer", 8, Rio.CANIVORE);
            /*
              Must add the following:
              - configPIDGains()
              - configFeedForwardGains()
              - configSupplyCurrentLimits()
              - configStatorCurrentLimit()
              - configClockwise_Positive()
              - follower?
            */
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

    // TODO: get actual values when robot is built
    private void applyStates() {
        double wantedRPM = 0;
        switch (systemState) {
            case OFF:
                stop();
                return;
            case INDEX_MAX:
                wantedRPM = 5000;
                break;
            case SLOW_INDEX:
                wantedRPM = 1000;
                break;
            case UNJAM:
                wantedRPM = -2000;
                break;
        }
        final double finalWantedRPM = wantedRPM;
        setVelocityTCFOCrpm(() -> finalWantedRPM);
    }

    @Getter private final SpindexerConfig config;

    public Spindexer(SpindexerConfig config) {
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
        Telemetry.log("Spindexer/WantedState", wantedState.toString());
        Telemetry.log("Spindexer/SystemState", systemState.toString());
        Telemetry.log("Spindexer/CurrentCommand", getCurrentCommandName());
        Telemetry.log("Spindexer/Voltage", getVoltage(), "volts");
        Telemetry.log("Spindexer/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("Spindexer/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("Spindexer/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("Spindexer/Temp", getTemp(), "deg_C");
    }
}
