package frc.robot.subsystems.dyeRotor;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotSim;
import frc.robot.subsystems.dyeRotor.DyeRotor.Feeder.FeederConfig;
import frc.robot.subsystems.dyeRotor.DyeRotor.Rotor.RotorConfig;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;
import lombok.Setter;

/**
 * The dye rotor: a spinning rotor that agitates fuel plus a feeder that indexes it toward the
 * launcher. Both are driven together by this class's state machine.
 *
 * <p>This is a container for two independent {@link Mechanism}s rather than a {@code Mechanism}
 * itself, so it implements {@link Subsystem} and registers directly. Without that registration the
 * scheduler would never call {@link #periodic()} and the state machine would never run.
 */
public class DyeRotor implements Subsystem {

    public static class Rotor extends Mechanism {

        public static class RotorConfig extends Config {

            @Getter @Setter private double supplyCurrentLimit = 80;
            @Getter @Setter private double supplyCurrentLowerLimit = 40;
            @Getter @Setter private double supplyCurrentLowerTime = 1.0;
            @Getter @Setter private double statorCurrentLimit = 80;

            @Getter private final double gearRatio = 37.5;

            @Getter @Setter private double velocityKp = 5;
            @Getter @Setter private double velocityKv = 4.38;
            @Getter @Setter private double velocityKs = 0;

            /* Sim Configs */
            @Getter
            private final double rotorX = Units.inchesToMeters(RobotSim.leftViewWidth / 2.0);

            @Getter
            private final double rotorY = Units.inchesToMeters(RobotSim.leftViewHeight / 2.0);

            @Getter private final double rotorDiameter = 12;

            /** Creates a new RotorConfig instance. */
            public RotorConfig() {
                super("Rotor", 9, Rio.CANIVORE);
                configPIDGains(velocityKp, 0, 0);
                configFeedForwardGains(velocityKs, velocityKv, 0, 0);
                configGearRatio(gearRatio);
                configSupplyCurrentLimit(supplyCurrentLimit, true);
                configLowerSupplyCurrentLimit(supplyCurrentLowerLimit);
                configLowerSupplyCurrentTime(supplyCurrentLowerTime);
                configStatorCurrentLimit(statorCurrentLimit, true);
                configForwardTorqueCurrentLimit(statorCurrentLimit);
                configReverseTorqueCurrentLimit(statorCurrentLimit);
                configNeutralBrakeMode(false);
                configCounterClockwise_Positive();
            }
        }

        @Getter private final RotorConfig config;

        @Getter private RotorSim sim;

        /**
         * Creates a new Rotor instance.
         *
         * @param config the config
         */
        public Rotor(RotorConfig config) {
            super(config);
            this.config = config;

            simulationInit();
            Telemetry.print(getName() + " Subsystem Initialized");
        }
        /** Runs the periodic update. */
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
        /**
         * Sets the rotor velocity.
         *
         * @param rpm the rotor velocity
         */
        public void setRotorVelocity(double rpm) {
            // setVelocityTCFOCrpm
            setVelocityRPM(() -> rpm);
        }
        /** Rotor stop. */
        public void rotorStop() {
            stop();
        }
        /** Simulation init. */
        public void simulationInit() {
            if (isAttached()) {
                sim = new RotorSim(RobotSim.leftView, motor);
            }
        }

        class RotorSim extends RollerSim {
            /**
             * Creates a new RotorSim instance.
             *
             * @param mech the mech
             * @param motor the motor
             */
            public RotorSim(Mechanism2d mech, TalonFX motor) {
                super(
                        new RollerConfig(config.getRotorDiameter())
                                .setPosition(config.getRotorX(), config.getRotorY())
                                .setGearRatio(config.getGearRatio()),
                        mech,
                        motor,
                        config.getName());
            }
        }
    }

    public static class Feeder extends Mechanism {

        public static class FeederConfig extends Config {

            @Getter @Setter private double supplyCurrentLimit = 80;
            @Getter @Setter private double supplyCurrentLowerLimit = 0;
            @Getter @Setter private double supplyCurrentLowerTime = 1.0;
            @Getter @Setter private double statorCurrentLimit = 120;

            @Getter @Setter private double velocityKp = 0.5;
            @Getter @Setter private double velocityKv = 0.434;
            @Getter @Setter private double velocityKs = 0;

            private final double gearRatio = 3.67;

            /** Creates a new FeederConfig instance. */
            public FeederConfig() {
                super("Feeder", 10, Rio.CANIVORE);
                configPIDGains(velocityKp, 0, 0);
                configFeedForwardGains(velocityKs, velocityKv, 0, 0);
                configGearRatio(gearRatio);
                configSupplyCurrentLimit(supplyCurrentLimit, true);
                configStatorCurrentLimit(statorCurrentLimit, true);
                configLowerSupplyCurrentLimit(supplyCurrentLowerLimit);
                configLowerSupplyCurrentTime(supplyCurrentLowerTime);
                configForwardTorqueCurrentLimit(statorCurrentLimit);
                configReverseTorqueCurrentLimit(statorCurrentLimit);
                configNeutralBrakeMode(false);
                configClockwise_Positive();
            }
        }

        @Getter private final FeederConfig config;

        /**
         * Creates a new Feeder instance.
         *
         * @param config the config
         */
        public Feeder(FeederConfig config) {
            super(config);
            this.config = config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }
        /** Runs the periodic update. */
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
        /**
         * Sets the feeder rpm.
         *
         * @param rpm the feeder rpm
         */
        public void setFeederVelocity(double rpm) {
            // setVelocityTCFOCrpm
            setVelocityRPM(() -> rpm);
        }
        /** Feeder stop. */
        public void feederStop() {
            stop();
        }
    }

    public static class DyeRotorConfig {
        @Getter private final RotorConfig rotorConfig;

        @Getter private final FeederConfig feederConfig;

        /**
         * Creates a new DyeRotorConfig instance.
         *
         * @param rotorConfig the rotorConfig
         * @param feederConfig the feederConfig
         */
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
            case IDLE_SLOW_INDEX -> SystemState.IDLE_SLOW_INDEX;
            case UNJAM -> SystemState.UNJAM;
        };
    }

    // TODO: test
    /** Applies the states. */
    private void applyStates() {
        double wantedRPMSpin = 0;
        double wantedRPMIndex = 0;
        switch (systemState) {
            case OFF:
                rotor.rotorStop();
                feeder.feederStop();
                return;
            case INDEX_MAX:
                wantedRPMSpin = 100;
                wantedRPMIndex = 2500;
                break;
            case IDLE_SLOW_INDEX:
                wantedRPMSpin = -20;
                wantedRPMIndex = 0;
                break;
            case UNJAM:
                wantedRPMSpin = 0;
                wantedRPMIndex = -1000;
                break;
        }
        final double finalWantedRPMSpin = wantedRPMSpin;
        final double finalWantedRPMIndex = wantedRPMIndex;
        rotor.setRotorVelocity(finalWantedRPMSpin);
        feeder.setFeederVelocity(finalWantedRPMIndex);
    }

    @Getter private final Rotor rotor;
    @Getter private final Feeder feeder;
    @Getter private final DyeRotorConfig config;

    /**
     * Creates and registers the dye rotor subsystem with the specified configuration.
     *
     * @param config configuration for the rotor and feeder mechanisms
     */
    public DyeRotor(DyeRotorConfig config) {
        this.config = config;
        this.rotor = new Rotor(config.getRotorConfig());
        this.feeder = new Feeder(config.getFeederConfig());

        this.register();
        Telemetry.print("Dye Rotor Subsystem Initialized");
    }

    /**
     * Updates the internal state, applies the corresponding rotor and feeder commands, and records
     * the requested and active states in telemetry.
     */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("DyeRotor/WantedState", wantedState.toString());
        Telemetry.log("DyeRotor/SystemState", systemState.toString());
    }
}
