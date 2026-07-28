package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.RobotSim;
import frc.robot.subsystems.intakeExtension.IntakeExtension.IntakeExtensionConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension.IntakeExtensionRight.RightConfig;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerFront.FrontConfig;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import lombok.Setter;

public class Spindexer extends Mechanism {

    public static class SpindexerConfig extends Config {

        @Getter @Setter private double supplyCurrentLimit = 40;
        @Getter @Setter private double statorCurrentLimit = 80;

        @Getter @Setter private double gearRatio = 37.5;

        // TODO: tune
        @Getter @Setter private double velocityKp = 5;
        @Getter @Setter private double velocityKv = 10;
        @Getter @Setter private double velocityKs = 15;

        /* Sim Configs */
        @Getter @Setter
        private double spindexerX = Units.inchesToMeters(RobotSim.leftViewWidth / 2.0);

        @Getter @Setter
        private double spindexerY = Units.inchesToMeters(RobotSim.leftViewHeight / 2.0);

        @Getter @Setter private double spindexerDiameter = 12;

        public SpindexerConfig() {
            super("Spindexer Back", 8, Rio.CANIVORE);
            configPIDGains(velocityKp, 0, 0);
            configFeedForwardGains(velocityKs, velocityKv, 0, 0);
            configGearRatio(gearRatio);
            configSupplyCurrentLimit(supplyCurrentLimit, true);
            configStatorCurrentLimit(statorCurrentLimit, true);
            configForwardTorqueCurrentLimit(statorCurrentLimit);
            configReverseTorqueCurrentLimit(statorCurrentLimit);
            configNeutralBrakeMode(false);
            configCounterClockwise_Positive();
        }
    }

    public static class SpindexerFront extends Mechanism {
        public static class FrontConfig extends Config {

            @Getter @Setter private double supplyCurrentLimit = 40;
            @Getter @Setter private double statorCurrentLimit = 80;

            @Getter @Setter private double gearRatio = 1.833;

            // TODO: tune
            @Getter @Setter private double velocityKp = 5;
            @Getter @Setter private double velocityKv = 10;
            @Getter @Setter private double velocityKs = 15;

            public FrontConfig() {
                super("Spindexer Front", 9, Rio.CANIVORE);
                configPIDGains(velocityKp, 0, 0);
                configFeedForwardGains(velocityKs, velocityKv, 0, 0);
                configGearRatio(gearRatio);
                configSupplyCurrentLimit(supplyCurrentLimit, true);
                configStatorCurrentLimit(statorCurrentLimit, true);
                configForwardTorqueCurrentLimit(statorCurrentLimit);
                configReverseTorqueCurrentLimit(statorCurrentLimit);
                configNeutralBrakeMode(false);
                configCounterClockwise_Positive();
            }
        }

        @Getter private final FrontConfig frontConfig;

        public SpindexerFront(SpindexerConfig backConfig) {
            super(new FrontConfig(backConfig));
            this.frontConfig = (FrontConfig) super.config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }

        public void spin(double rpm) {
            setVelocityTCFOCrpm(() -> rpm);
        }

        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("Spindexer/CurrentCommand", getCurrentCommandName());
            Telemetry.log("Spindexer/Voltage", getVoltage(), "volts");
            Telemetry.log("Spindexer/StatorCurrent", getStatorCurrent(), "amps");
            Telemetry.log("Spindexer/SupplyCurrent", getSupplyCurrent(), "amps");
            Telemetry.log("Spindexer/RPM", getVelocityRPM(), "RPM");
            Telemetry.log("Spindexer/Temp", getTemp(), "deg_C");
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
        double wantedRPMSpin = 0;
        double wantedRPMIndex = 0;
        switch (systemState) {
            case OFF:
                stop();
                return;
            case INDEX_MAX:
                wantedRPMSpin = 3000;
                wantedRPMIndex = 3000;
                break;
            case IDLE_SLOW_INDEX:
                wantedRPMSpin = 1000;
                wantedRPMIndex = 1000;
                break;
            case UNJAM:
                wantedRPMSpin = -2000;
                wantedRPMIndex = -2000;
                break;
        }
        final double finalWantedRPMSpin = wantedRPMSpin;
        final double finalWantedRPMIndex = wantedRPMIndex;
        commandBoth(finalWantedRPMSpin, finalWantedRPMIndex);
    }

    @Getter private final SpindexerConfig config;
    @Getter private SpindexerSim sim;
    private final SpindexerFront front;

    // TODO: test
    private void commandBoth(double spinRPM, double indexRPM) {
        setVelocityTCFOCrpm(() -> spinRPM);
        front.spin(indexRPM);
    }

    public Spindexer(SpindexerConfig config) {
        super(config);
        this.config = config;
        this.front = new SpindexerFront(config);

        simulationInit();
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

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    public void simulationInit() {
        if (isAttached()) {
            // Create a new RollerSim with the top view, the motor's sim state, and a 12 in
            // diameter
            sim = new SpindexerSim(RobotSim.leftView, motor.getSimState());
        }
    }

    class SpindexerSim extends RollerSim {
        public SpindexerSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
            super(
                    new RollerConfig(config.getSpindexerDiameter())
                            .setPosition(config.getSpindexerX(), config.getSpindexerY()),
                    mech,
                    rollerMotorSim,
                    config.getName());
        }
    }
}
