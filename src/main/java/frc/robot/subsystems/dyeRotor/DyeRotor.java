package frc.robot.subsystems.dyeRotor;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotSim;
import frc.robot.subsystems.dyeRotor.DyeRotor.Rotor.RotorConfig;
import frc.robot.subsystems.dyeRotor.DyeRotor.Feeder.FeederConfig;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import lombok.Setter;

public class DyeRotor {

    public class Rotor extends Mechanism {

        public static class RotorConfig extends Config {

            @Getter @Setter private double supplyCurrentLimit = 40;
            @Getter @Setter private double statorCurrentLimit = 80;

            @Getter @Setter private double gearRatio = 37.5;

            // TODO: tune
            @Getter @Setter private double velocityKp = 5;
            @Getter @Setter private double velocityKv = 10;
            @Getter @Setter private double velocityKs = 15;

            /* Sim Configs */
            @Getter @Setter
            private double rotorX = Units.inchesToMeters(RobotSim.leftViewWidth / 2.0);

            @Getter @Setter
            private double rotorY = Units.inchesToMeters(RobotSim.leftViewHeight / 2.0);

            @Getter @Setter private double rotorDiameter = 12;

            public RotorConfig() {
                super("Rotor", 8, Rio.CANIVORE);
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

        @Getter private final RotorConfig config;

        public Rotor(RotorConfig config) {
            super(config);
            this.config = config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }

        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("Rotor/CurrentCommand", getCurrentCommandName());
            Telemetry.log("Rotor/Voltage", getVoltage(), "volts");
            Telemetry.log("Rotor/StatorCurrent", getStatorCurrent(), "amps");
            Telemetry.log("Rotor/SupplyCurrent", getSupplyCurrent(), "amps");
            Telemetry.log("Rotor/RPM", getVelocityRPM(), "RPM");
            Telemetry.log("Rotor/Temp", getTemp(), "deg_C");
        }

        public void setRotorRpm(double rpm) {
            setVelocityTCFOCrpm(() -> rpm);
        }

        public void rotorStop() {
            stop();
        }
    }

    public class Feeder extends Mechanism {

        public static class FeederConfig extends Config {

            @Getter @Setter private double supplyCurrentLimit = 40;
            @Getter @Setter private double statorCurrentLimit = 80;

            // TODO: tune
            @Getter @Setter private double velocityKp = 5;
            @Getter @Setter private double velocityKv = 10;
            @Getter @Setter private double velocityKs = 15;

            private final double gearRatio = 1.833;

            public FeederConfig() {
                super("Feeder", 9, Rio.CANIVORE);
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

        @Getter private final FeederConfig config;

        public Feeder(FeederConfig config) {
            super(config);
            this.config = config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }

        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("Feeder/CurrentCommand", getCurrentCommandName());
            Telemetry.log("Feeder/Voltage", getVoltage(), "volts");
            Telemetry.log("Feeder/StatorCurrent", getStatorCurrent(), "amps");
            Telemetry.log("Feeder/SupplyCurrent", getSupplyCurrent(), "amps");
            Telemetry.log("Feeder/RPM", getVelocityRPM(), "RPM");
            Telemetry.log("Feeder/Temp", getTemp(), "deg_C");
        }

        public void setFeederRpm(double rpm) {
            setVelocityTCFOCrpm(() -> rpm);
        }

        public void feederStop() {
            stop();
        }
    }

    public static class DyeRotorConfig {

        @Getter private final RotorConfig rotorConfig;
        @Getter private final FeederConfig feederConfig;

        public DyeRotorConfig(RotorConfig rotorConfig, FeederConfig feederConfig) {
            this.rotorConfig = rotorConfig;
            this.feederConfig = feederConfig;
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
                rotor.rotorStop();
                feeder.feederStop();
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
        rotor.setRotorRpm(finalWantedRPMSpin);
        feeder.setFeederRpm(finalWantedRPMIndex);
    }

    @Getter private Rotor rotor;
    @Getter private Feeder feeder;
    @Getter private DyeRotorConfig config;
    // @Getter private RotorSim sim;

    public DyeRotor(DyeRotorConfig config) {
        this.config = config;

        // simulationInit();
        Telemetry.print("Dye Rotor Subsystem Initialized");
    }

    public void periodic() {
        systemState = handleStateTransition();
        applyStates();
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // public void simulationInit() {
    //     if (rotor.isAttached()) {
    //         // Create a new RollerSim with the top view, the motor's sim state, and a 12 in
    //         // diameter
    //         sim = new RotorSim(RobotSim.leftView, motor.getSimState());
    //     }
    // }

    // class RotorSim extends RollerSim {
    //     public RotorSim(Mechanism2d mech, TalonFXSimState rollerMotorSim) {
    //         super(
    //                 new RollerConfig(rotorConfig.getRotorDiameter())
    //                         .setPosition(rotorConfig.getRotorX(), rotorConfig.getRotorY()),
    //                 mech,
    //                 rollerMotorSim,
    //                 rotorConfig.getName());
    //     }
    // }
}
