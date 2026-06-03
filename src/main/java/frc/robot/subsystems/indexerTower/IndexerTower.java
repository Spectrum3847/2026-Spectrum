package frc.robot.subsystems.indexerTower;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import lombok.Setter;

public class IndexerTower extends Mechanism {

    public static class IndexerTowerConfig extends Config {
        /* Intake config values */
        @Getter @Setter private double currentLimit = 80;
        @Getter @Setter private double torqueCurrentLimit = 140;
        @Getter @Setter private double lowerCurrentLimit = 60;
        @Getter @Setter private double timeUntilLowerCurrent = 1;
        @Getter @Setter private double velocityKp = 50;
        @Getter @Setter private double velocityKv = 0;
        @Getter @Setter private double velocityKs = 40;

        /* Sim Configs */
        @Getter private double intakeX = Units.inchesToMeters(60);
        @Getter private double intakeY = Units.inchesToMeters(75);
        @Getter private double wheelDiameter = 12;

        public IndexerTowerConfig() {
            super("IndexerTower", 51, Rio.CANIVORE);
            configPIDGains(0, velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(1);
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configLowerSupplyCurrentLimit(lowerCurrentLimit);
            configLowerSupplyCurrentTime(timeUntilLowerCurrent);
            configNeutralBrakeMode(true);
            configClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig(
                            "IndexerTower Follower",
                            52,
                            Rio.CANIVORE,
                            MotorAlignmentValue.Aligned));
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
        setVelocityTCFOCrpm(() -> finalWantedRPM);
    }

    // private IndexerTowerConfig config;
    // private IndexerSim sim;

    public IndexerTower(IndexerTowerConfig config) {
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
        Telemetry.log("IndexerTower/WantedState", wantedState.toString());
        Telemetry.log("IndexerTower/SystemState", systemState.toString());
        Telemetry.log("IndexerTower/CurrentCommand", getCurrentCommandName());
        Telemetry.log("IndexerTower/Voltage", getVoltage(), "volts");
        Telemetry.log("IndexerTower/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("IndexerTower/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("IndexerTower/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("IndexerTower/Temp", getTemp(), "deg_C");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // public void simulationInit() {
    //     if (isAttached()) {
    //         // Create a new RollerSim with the left view, the motor's sim state, and a 6 in
    // diameter
    //         sim = new IndexerSim(RobotSim.topView, motor.getSimState());
    //     }
    // }

    // // Must be called to enable the simulation
    // // if roller position changes configure x and y to set position.
    // @Override
    // public void simulationPeriodic() {
    //     if (isAttached()) {
    //         sim.simulationPeriodic();
    //     }
    // }

    // class IndexerSim extends RollerSim {
    //     public IndexerSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
    //         super(
    //                 new RollerConfig(config.getWheelDiameter())
    //                         .setPosition(config.getIntakeX(), config.getIntakeY()),
    //                 mech,
    //                 rollerMotorSim,
    //                 config.getName());
    //     }
    // }
}
