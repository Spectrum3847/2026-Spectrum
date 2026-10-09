package frc.robot.subsystems.indexerBed;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/** Moves fuel across the hopper bed toward the indexer tower. */
public class IndexerBed extends Mechanism {

    public static class IndexerBedConfig extends Config {
        @Getter private final double supplyCurrentLimit = 30;
        @Getter private final double statorCurrentLimit = 180;
        @Getter private final double lowerSupplyCurrentLimit = 30;
        @Getter private final double lowerSupplyCurrentTime = 0;
        @Getter private final double velocityKp = 30;
        @Getter private final double velocityKv = 0;
        @Getter private final double velocityKs = 4;

        @Getter private final double intakeX = Units.inchesToMeters(60);
        @Getter private final double intakeY = Units.inchesToMeters(75);
        @Getter private final double wheelDiameter = 12;

        public IndexerBedConfig() {
            super("IndexerBed", 8, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
            configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
            configNeutralBrakeMode(false);
            configClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "IndexerBed Follower 1", 9, Rio.CANIVORE, MotorAlignmentValue.Opposed));
        }
    }

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

    @Getter private final IndexerBedConfig config;

    public IndexerBed(IndexerBedConfig config) {
        super(config);
        this.config = config;

        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.logDash("IndexerBed/WantedState", wantedState.toString());
        Telemetry.logDash("IndexerBed/SystemState", systemState.toString());
        Telemetry.logDash("IndexerBed/CurrentCommand", getCurrentCommandName());
        Telemetry.logDash("IndexerBed/Voltage", getVoltage(), "volts");
        Telemetry.logDash("IndexerBed/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.logDash("IndexerBed/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.logDash("IndexerBed/RPM", getVelocityRPM(), "RPM");
        Telemetry.logDash("IndexerBed/Temp", getTemp(), "deg_C");
    }
}
