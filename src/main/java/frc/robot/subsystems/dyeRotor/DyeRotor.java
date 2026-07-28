package frc.robot.subsystems.dyeRotor;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.RobotSim;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

public class DyeRotor extends Mechanism {

    public static class DyeRotorConfig extends Config {

        @Getter private final double supplyCurrentLimit = 40;
        @Getter private final double statorCurrentLimit = 80;
        @Getter private final double gearRatio = 36.4;
        // TODO: tune
        @Getter private final double velocityKp = 5;
        @Getter private final double velocityKv = 10;
        @Getter private final double velocityKs = 15;

        /* Sim Configs */
        @Getter private final double rotorX = Units.inchesToMeters(RobotSim.leftViewWidth / 2.0);

        @Getter private final double rotorY = Units.inchesToMeters(RobotSim.leftViewHeight / 2.0);

        @Getter private final double simGearRatio = gearRatio;
        @Getter private final double rotorDiameter = 12;

        public DyeRotorConfig() {
            super("DyeRotor 1", 8, Rio.CANIVORE);
            configPIDGains(velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(gearRatio);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configNeutralBrakeMode(false);
            configCounterClockwise_Positive();
            setFollowerConfigs(
                    new FollowerConfig("DyeRotor 2", 9, Rio.CANIVORE, MotorAlignmentValue.Aligned));
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        OFF,
        INDEX_MAX,
        IDLE_SLOW_INDEX,
        UNJAM,
    }

    public enum SystemState {
        OFF,
        INDEX_MAX,
        IDLE_SLOW_INDEX,
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
            case IDLE_SLOW_INDEX -> SystemState.IDLE_SLOW_INDEX;
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
                wantedRPM = 130;
                break;
            case IDLE_SLOW_INDEX:
                wantedRPM = -20;
                break;
            case UNJAM:
                wantedRPM = 0;
                break;
        }
        final double finalWantedRPM = wantedRPM;
        setVelocityTCFOCrpm(() -> finalWantedRPM);
    }

    @Getter private final DyeRotorConfig config;
    @Getter private DyeRotorSim sim;

    public DyeRotor(DyeRotorConfig config) {
        super(config);
        this.config = config;

        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
        logBatteryUsage();
        Telemetry.log("DyeRotor/WantedState", wantedState.toString());
        Telemetry.log("DyeRotor/SystemState", systemState.toString());
        Telemetry.log("DyeRotor/CurrentCommand", getCurrentCommandName());
        Telemetry.log("DyeRotor/Voltage", getVoltage(), "volts");
        Telemetry.log("DyeRotor/StatorCurrent", getStatorCurrent(), "amps");
        Telemetry.log("DyeRotor/SupplyCurrent", getSupplyCurrent(), "amps");
        Telemetry.log("DyeRotor/RPM", getVelocityRPM(), "RPM");
        Telemetry.log("DyeRotor/Temp", getTemp(), "deg_C");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            sim = new DyeRotorSim(RobotSim.leftView, motor.getSimState());
        }
    }

    class DyeRotorSim extends RollerSim {
        public DyeRotorSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new RollerConfig(config.getRotorDiameter())
                            .setPosition(config.getRotorX(), config.getRotorY())
                            .setGearRatio(config.getSimGearRatio()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}
