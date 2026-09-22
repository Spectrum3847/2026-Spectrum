package frc.robot.subsystems.fuelIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
            logStandard("IntakeRoller", false, RpmLog.SLOW);
        }

        /**
         * Sets the roller voltage.
         *
         * @param volts the roller voltage
         */
        public void setRollerVoltage(double volts) {
            setVoltageOutput(() -> volts);
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

            @Getter private final double supplyCurrentLimit = 40;
            /**
             * Was 80. The kicker gets pushed into the bumper, which is where its drag comes from
             * and is not an easy mechanical fix, so it ran 90 A at the 99th percentile with 129 A
             * peaks and was the hottest motor on the robot on 2026-09-06 (67 C). It does not need
             * the torque; cap it.
             */
            @Getter private final double statorCurrentLimit = 50;

            @Getter private final double lowerSupplyCurrentLimit = 40;
            @Getter private final double lowerSupplyCurrentTime = 1;

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
            logStandard("IntakeKicker", false, RpmLog.SLOW);
        }

        /**
         * Sets the kicker voltage.
         *
         * @param volts the kicker voltage
         */
        public void setKickerVoltage(double volts) {
            setVoltageOutput(() -> volts);
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        NEUTRAL,
        OFF,
        INTAKE,
        SLOW_INTAKE,
        /** Rollers and kicker backwards, to spit fuel out during an unjam. */
        REVERSE,
        /** Rollers backwards but the kicker still forward, for the kicker unjam. */
        REVERSE_KEEP_KICKER,
    }

    public enum SystemState {
        NEUTRAL,
        OFF,
        INTAKE,
        SLOW_INTAKE,
        REVERSE,
        REVERSE_KEEP_KICKER,
    }

    /** Kicker does not need to spin fast; 12 V just heated it against the bumper. */
    private static final double KICKER_INTAKE_VOLTS = 8;

    /** Kicker reverse during unjam. Full voltage stalled it at the limit for 18 of 19 s. */
    private static final double KICKER_REVERSE_VOLTS = -6;

    /** A reversed kicker this slow, drawing this much, for this long, is jammed: stop it. */
    private static final double KICKER_STALL_RPM = 50;

    private static final double KICKER_STALL_STATOR_AMPS = 35;
    private static final double KICKER_STALL_SECS = 1.0;

    private final Debouncer kickerStallDebouncer =
            new Debouncer(KICKER_STALL_SECS, DebounceType.kRising);
    /** Once set, the kicker stays off until the intake leaves the reverse state. */
    private boolean kickerStallLatched = false;

    /**
     * Reverse voltage for the kicker during unjam, or zero once it has jammed. The latch clears
     * only when the state changes, so a stuck kicker rests until unjam is released and pressed
     * again rather than grinding for as long as the button is held.
     */
    private double kickerReverseVoltsWithStallLatch() {
        if (kickerStallLatched) {
            return 0;
        }
        boolean stalledNow =
                Math.abs(kicker.getVelocityRPM()) < KICKER_STALL_RPM
                        && Math.abs(kicker.getStatorCurrent()) > KICKER_STALL_STATOR_AMPS;
        if (kickerStallDebouncer.calculate(stalledNow)) {
            kickerStallLatched = true;
            return 0;
        }
        return KICKER_REVERSE_VOLTS;
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
            case REVERSE -> SystemState.REVERSE;
            case REVERSE_KEEP_KICKER -> SystemState.REVERSE_KEEP_KICKER;
            case OFF -> SystemState.OFF;
        };
    }

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
                wantedKickerVoltage = KICKER_INTAKE_VOLTS;
                break;
            case SLOW_INTAKE:
                wantedRollerVoltage = 6;
                wantedKickerVoltage = 6;
                break;
            case REVERSE:
                wantedRollerVoltage = -12;
                wantedKickerVoltage = kickerReverseVoltsWithStallLatch();
                break;
            case REVERSE_KEEP_KICKER:
                wantedRollerVoltage = -12;
                wantedKickerVoltage = KICKER_INTAKE_VOLTS;
                break;
            case OFF:
                roller.stop();
                kicker.stop();
                return;
        }
        final double finalRollerVoltage = wantedRollerVoltage;
        final double finalKickerVoltage = wantedKickerVoltage;
        roller.setRollerVoltage(finalRollerVoltage);
        kicker.setKickerVoltage(finalKickerVoltage);
    }

    @Getter private final IntakeRoller roller;

    @Getter private final IntakeKicker kicker;

    /**
     * Creates a new FuelIntake instance.
     *
     * @param rollerConfig the intake roller config
     * @param kickerConfig the intake kicker config
     */
    public FuelIntake(IntakeRollerConfig rollerConfig, IntakeKickerConfig kickerConfig) {
        this.roller = new IntakeRoller(rollerConfig);
        this.kicker = new IntakeKicker(kickerConfig);

        this.register();
        Telemetry.print("Fuel Intake Subsystem Initialized");
    }

    /** Runs the periodic update. */
    @Override
    public void periodic() {
        systemState = handleStateTransition();
        if (systemState != SystemState.REVERSE) {
            kickerStallLatched = false;
            kickerStallDebouncer.calculate(false);
        }
        applyStates();

        Telemetry.logState("FuelIntake/WantedState", wantedState);
        Telemetry.logState("FuelIntake/SystemState", systemState);
        Telemetry.log("FuelIntake/KickerStallLatched", kickerStallLatched);
    }
}
