package frc.robot.subsystems.fuelIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.subsystems.fuelIntake.FuelIntake.IntakeKicker.IntakeKickerConfig;
import frc.robot.subsystems.fuelIntake.FuelIntake.IntakeRoller.IntakeRollerConfig;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.RollerConfig;
import frc.spectrumLib.sim.RollerSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

/**
 * The Fuel Intake subsystem. Responsible for intake and handling of fuel elements.
 *
 * <p>Made up of the main roller pair that picks fuel off the floor plus a kicker bar that pulls it
 * off the rollers and into the robot. Both are driven together by this class's state machine.
 *
 * <p>This is a container for two independent {@link Mechanism}s rather than a {@code Mechanism}
 * itself, so it implements {@link Subsystem} and registers directly. Without that registration the
 * scheduler would never call {@link #periodic()} and the state machine would never run.
 */
public class FuelIntake implements Subsystem {

    /** The main intake roller pair: a leader plus an opposed follower on the far side. */
    public static class IntakeRoller extends Mechanism {

        public static class IntakeRollerConfig extends Config {

            // Likely keep current limits
            @Getter private final double supplyCurrentLimit = 80;
            @Getter private final double statorCurrentLimit = 80;
            @Getter private final double lowerSupplyCurrentLimit = 40;
            @Getter private final double lowerSupplyCurrentTime = 1;
            // TODO: tune
            @Getter private final double velocityKp = 0.3;
            @Getter private final double velocityKv = 0.23728813559;
            @Getter private final double velocityKs = 0;

            /* kV above was characterized at this ratio; keep the two in sync */
            @Getter private final double gearRatio = 2.33;

            /* Sim Configs */
            @Getter private final double intakeX = Units.inchesToMeters(15);

            @Getter private final double intakeY = Units.inchesToMeters(23);
            @Getter private final double wheelDiameter = 6;

            /** Creates a new IntakeRollerConfig instance. */
            public IntakeRollerConfig() {
                super("Intake Roller Left", 6, Rio.RIO_CANBUS);
                configPIDGains(0, velocityKp, 0, 0);
                configFeedForwardGains(velocityKs, velocityKv, 0, 0);
                configGearRatio(gearRatio);
                configSupplyCurrentLimit(supplyCurrentLimit, true);
                configStatorCurrentLimit(statorCurrentLimit, true);
                configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
                configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
                configForwardTorqueCurrentLimit(statorCurrentLimit);
                configReverseTorqueCurrentLimit(statorCurrentLimit);
                configNeutralBrakeMode(false);
                configCounterClockwise_Positive();
                setFollowerConfigs(
                        new FollowerConfig(
                                "Intake Roller Right",
                                7,
                                Rio.RIO_CANBUS,
                                MotorAlignmentValue.Opposed));
            }
        }

        @Getter private final IntakeRollerConfig config;

        @Getter private IntakeRollerSim sim;

        /**
         * Creates a new IntakeRoller instance.
         *
         * @param config the config
         */
        public IntakeRoller(IntakeRollerConfig config) {
            super(config);
            this.config = config;

            simulationInit();
            Telemetry.print(getName() + " Subsystem Initialized");
        }
        /** Runs the periodic update. */
        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("IntakeRoller/CurrentCommand", getCurrentCommandName());
            Telemetry.log("IntakeRoller/Voltage", getVoltage(), "volts");
            Telemetry.log("IntakeRoller/StatorCurrent", getStatorCurrent(), "amps");
            Telemetry.log("IntakeRoller/SupplyCurrent", getSupplyCurrent(), "amps");
            Telemetry.log("IntakeRoller/RPM", getVelocityRPM(), "RPM");
            Telemetry.log("IntakeRoller/Temp", getTemp(), "deg_C");
        }
        /**
         * Sets the roller voltage.
         *
         * @param volts the roller voltage
         */
        public void setRollerVoltage(double volts) {
            setVoltageOutput(() -> volts);
        }
        /** Roller stop. */
        public void rollerStop() {
            stop();
        }

        // ----------------------------------------------------------------------------
        // Simulation
        // ----------------------------------------------------------------------------
        /** Simulation init. */
        public void simulationInit() {
            if (isAttached()) {
                // Create a new RollerSim with the left view, the motor's sim state, and a 6 in
                // diameter
                sim = new IntakeRollerSim(RobotSim.leftView, motor);
            }
        }

        class IntakeRollerSim extends RollerSim {
            /**
             * Creates a new IntakeRollerSim instance.
             *
             * @param mech the mech
             * @param rollerMotorSim the rollerMotorSim
             */
            public IntakeRollerSim(Mechanism2d mech, TalonFX rollerMotorSim) {
                super(
                        new RollerConfig(config.getWheelDiameter())
                                .setPosition(config.getIntakeX(), config.getIntakeY())
                                .setGearRatio(config.getGearRatio())
                                .setMount(Robot.getIntakeExtension().getSim()),
                        mech,
                        rollerMotorSim,
                        config.getName());
            }
        }
    }

    /** The kicker bar that helps kick fuel through the intake up into the hopper. */
    public static class IntakeKicker extends Mechanism {

        public static class IntakeKickerConfig extends Config {

            // TODO: set real limits once the kicker bar is built
            @Getter private final double supplyCurrentLimit = 40;
            @Getter private final double statorCurrentLimit = 80;
            @Getter private final double lowerSupplyCurrentLimit = 40;
            @Getter private final double lowerSupplyCurrentTime = 1;
            // TODO: tune
            @Getter private final double velocityKp = 0.3;
            @Getter private final double velocityKv = 0.23728813559;
            @Getter private final double velocityKs = 0;

            /* kV above was characterized at this ratio; keep the two in sync */
            @Getter private final double gearRatio = 2.33;

            /** Creates a new IntakeKickerConfig instance. */
            public IntakeKickerConfig() {
                super("Intake Kicker", 8, Rio.CANIVORE);
                configPIDGains(0, velocityKp, 0, 0);
                configFeedForwardGains(velocityKs, velocityKv, 0, 0);
                configGearRatio(gearRatio);
                configSupplyCurrentLimit(supplyCurrentLimit, true);
                configStatorCurrentLimit(statorCurrentLimit, true);
                configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
                configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
                configForwardTorqueCurrentLimit(statorCurrentLimit);
                configReverseTorqueCurrentLimit(statorCurrentLimit);
                configNeutralBrakeMode(false);
                // Counter-rotates against the rollers, so it takes the opposite sense
                configClockwise_Positive();
            }
        }

        @Getter private final IntakeKickerConfig config;

        /**
         * Creates a new IntakeKicker instance.
         *
         * @param config the config
         */
        public IntakeKicker(IntakeKickerConfig config) {
            super(config);
            this.config = config;

            Telemetry.print(getName() + " Subsystem Initialized");
        }
        /** Runs the periodic update. */
        @Override
        public void periodic() {
            logBatteryUsage();
            Telemetry.log("IntakeKicker/CurrentCommand", getCurrentCommandName());
            Telemetry.log("IntakeKicker/Voltage", getVoltage(), "volts");
            Telemetry.log("IntakeKicker/StatorCurrent", getStatorCurrent(), "amps");
            Telemetry.log("IntakeKicker/SupplyCurrent", getSupplyCurrent(), "amps");
            Telemetry.log("IntakeKicker/RPM", getVelocityRPM(), "RPM");
            Telemetry.log("IntakeKicker/Temp", getTemp(), "deg_C");
        }
        /**
         * Sets the kicker voltage.
         *
         * @param volts the kicker voltage
         */
        public void setKickerVoltage(double volts) {
            setVoltageOutput(() -> volts);
        }
        /** Kicker stop. */
        public void kickerStop() {
            stop();
        }
    }

    public static class FuelIntakeConfig {
        @Getter private final IntakeRollerConfig rollerConfig;

        @Getter private final IntakeKickerConfig kickerConfig;

        /**
         * Creates a new FuelIntakeConfig instance.
         *
         * @param rollerConfig the rollerConfig
         * @param kickerConfig the kickerConfig
         */
        public FuelIntakeConfig(IntakeRollerConfig rollerConfig, IntakeKickerConfig kickerConfig) {
            this.rollerConfig = rollerConfig;
            this.kickerConfig = kickerConfig;
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        NEUTRAL,
        OFF,
        INTAKE,
        SLOW_INTAKE,
    }

    public enum SystemState {
        NEUTRAL,
        OFF,
        INTAKE,
        SLOW_INTAKE,
    }

    private WantedState wantedState = WantedState.NEUTRAL;
    private SystemState systemState = SystemState.NEUTRAL;
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
            case NEUTRAL -> SystemState.NEUTRAL;
            case INTAKE -> SystemState.INTAKE;
            case SLOW_INTAKE -> SystemState.SLOW_INTAKE;
            case OFF -> SystemState.OFF;
        };
    }

    // TODO: get actual kicker voltages when the robot is built
    /** Applies the states. */
    private void applyStates() {
        double wantedRollerVoltage = 0;
        double wantedKickerVoltage = 0;
        switch (systemState) {
            case NEUTRAL:
                wantedRollerVoltage = 0;
                wantedKickerVoltage = 0;
                break;
            case INTAKE:
                wantedRollerVoltage = 12;
                wantedKickerVoltage = 12;
                break;
            case SLOW_INTAKE:
                wantedRollerVoltage = 6;
                wantedKickerVoltage = 6;
                break;
            case OFF:
                roller.rollerStop();
                kicker.kickerStop();
                return;
        }
        final double finalRollerVoltage = wantedRollerVoltage;
        final double finalKickerVoltage = wantedKickerVoltage;
        roller.setRollerVoltage(finalRollerVoltage);
        kicker.setKickerVoltage(finalKickerVoltage);
    }

    @Getter private final IntakeRoller roller;

    @Getter private final IntakeKicker kicker;
    @Getter private final FuelIntakeConfig config;

    /**
     * Creates a new FuelIntake instance.
     *
     * @param config the config
     */
    public FuelIntake(FuelIntakeConfig config) {
        this.config = config;
        this.roller = new IntakeRoller(config.getRollerConfig());
        this.kicker = new IntakeKicker(config.getKickerConfig());

        this.register();
        Telemetry.print("Fuel Intake Subsystem Initialized");
    }
    /** Runs the periodic update. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("FuelIntake/WantedState", wantedState.toString());
        Telemetry.log("FuelIntake/SystemState", systemState.toString());
    }
}
