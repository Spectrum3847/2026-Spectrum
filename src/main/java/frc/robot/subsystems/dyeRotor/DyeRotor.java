package frc.robot.subsystems.dyeRotor;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
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

            @Getter private final double supplyCurrentLimit = 80;
            @Getter private final double supplyCurrentLowerLimit = 40;
            @Getter private final double supplyCurrentLowerTime = 1.0;
            @Getter private final double statorCurrentLimit = 80;

            @Getter private final double gearRatio = 37.5;

            @Getter private final double velocityKp = 270;
            @Getter private final double velocityKv = 0.800000011920929;
            @Getter private final double velocityKs = 2.650390625;

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
                configCurrentLimits(
                        supplyCurrentLimit,
                        statorCurrentLimit,
                        supplyCurrentLowerLimit,
                        supplyCurrentLowerTime);
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
            logStandard("Rotor", false, RpmLog.SLOW_DASH);
        }

        /**
         * Sets the rotor velocity.
         *
         * @param rpm the rotor velocity
         */
        public void setRotorVelocity(double rpm) {
            setVelocityTCFOCrpm(() -> rpm);
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
                                .setGearRatio(config.getGearRatio())
                                .setReversedLinkage(true),
                        mech,
                        motor,
                        config.getName());
            }
        }
    }

    public static class Feeder extends Mechanism {

        public static class FeederConfig extends Config {

            /**
             * Was 80. The feeder was the single biggest draw at the 8.3 V battery minimum in the
             * 2026-09-06 21:49 log, 77 A supply. Trimmed a little; feed rate between balls depends
             * on this.
             */
            @Getter private final double supplyCurrentLimit = 75;

            /**
             * No lower limit. It was 0 with a 1 s lower time, which never engaged because the
             * feeder never held 80 A for a second; a 40 A lower limit under a 65 A cap did engage
             * during long bursts and starved the feed. A lower time of zero disables the lower
             * limit.
             */
            @Getter private final double supplyCurrentLowerLimit = 40;

            @Getter private final double supplyCurrentLowerTime = 0.0;
            @Getter private final double statorCurrentLimit = 120;

            @Getter private final double velocityKp = 0.5;
            @Getter private final double velocityKv = 0.434;
            @Getter private final double velocityKs = 0;

            private final double gearRatio = 3.67;

            /** Creates a new FeederConfig instance. */
            public FeederConfig() {
                super("Feeder", 10, Rio.CANIVORE);
                configPIDGains(velocityKp, 0, 0);
                configFeedForwardGains(velocityKs, velocityKv, 0, 0);
                configGearRatio(gearRatio);
                configCurrentLimits(
                        supplyCurrentLimit,
                        statorCurrentLimit,
                        supplyCurrentLowerLimit,
                        supplyCurrentLowerTime);
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
            logStandard("Feeder", false, RpmLog.SLOW_DASH);
        }

        /**
         * Sets the feeder rpm.
         *
         * @param rpm the feeder rpm
         */
        public void setFeederVelocity(double rpm) {
            setVelocityRPM(() -> rpm);
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

    /**
     * Feeder speed for {@code INDEX_MAX}, in mechanism RPM.
     *
     * <p>Was 2500, which the feeder cannot reach and never has. {@code FeederConfig.velocityKv} is
     * 0.434 V per mechanism rotation per second, so 2500 RPM is 41.7 rot/s and asks 18.1 V of
     * feedforward alone on a 12 V bus. The ceiling at a full 12 V and no load is about 1660 RPM.
     *
     * <p>The logs agree to within a percent. On 2026-09-05 the feeder topped out at 1477 RPM with
     * 11.0 V applied, and 1477 RPM is 24.6 rot/s, wanting 24.6 x 0.434 = 10.7 V. It was not
     * fighting fuel either: the fastest samples drew only 8 to 17 A of stator.
     *
     * <p>The cost of asking the impossible is that the velocity loop saturates at full output the
     * whole time it feeds. It never regulates, so the feed rate is whatever the load allows that
     * moment and the flywheel sees fuel arrive at a rate nobody chose.
     *
     * <p>1300 RPM asks 9.4 V, leaving about 2.6 V of headroom at a healthy bus for load and for the
     * loop to correct. That is close to the 1185 RPM mean the feeder already managed, so the feed
     * rate barely moves; what changes is that it becomes a speed the robot holds rather than one it
     * happens to reach.
     *
     * <p>Tunable so it can be swept live. Above about 1400 it saturates again whenever the battery
     * sags, which during a launch it does.
     */
    private static final DoubleSubscriber indexMaxFeederRPM =
            Telemetry.tunable("DyeRotor/IndexMaxFeederRPM", 1300.0);

    // ---- Idle rotor stall check ----

    /** Slow reverse spin that keeps fuel loose while idling and intaking. */
    private static final double IDLE_ROTOR_RPM = -20;

    /**
     * Below this speed and above this stator current, for this long, the idle rotor is packed
     * against fuel rather than stirring it. In the 2026-09-06 22:25 log it sat that way at the 80 A
     * stator limit for 27 s, 88 percent of it at zero RPM, and went from 26 to 50 C.
     */
    private static final double STALL_RPM = 5;

    private static final double STALL_STATOR_AMPS = 60;
    private static final double STALL_DEBOUNCE_SECS = 0.15;
    /** How long the rotor rests after a stall before trying the slow spin again. */
    private static final double STALL_BACKOFF_SECS = 1.0;

    private final Timer stallTimer = new Timer();
    private boolean stallTiming = false;
    private boolean stallBackoff = false;
    private int stallCount = 0;

    /**
     * Returns the idle spin speed, or zero while backing off from a stall. A stall is the rotor
     * nearly stopped while drawing heavy stator current for the debounce time. This is done in
     * software rather than with a lower current limit because limits are config writes, and one
     * that fails to restore leaves the rotor weak for the rest of the match.
     */
    private double idleRotorRpmWithStallCheck(double rpm) {
        if (stallBackoff) {
            if (stallTimer.hasElapsed(STALL_BACKOFF_SECS)) {
                stallBackoff = false;
                stallTiming = false;
            } else {
                return 0;
            }
        }

        boolean stalledNow =
                Math.abs(rotor.getVelocityRPM()) < STALL_RPM
                        && Math.abs(rotor.getStatorCurrent()) > STALL_STATOR_AMPS;
        if (!stalledNow) {
            stallTiming = false;
            return rpm;
        }
        if (!stallTiming) {
            stallTiming = true;
            stallTimer.restart();
            return rpm;
        }
        if (stallTimer.hasElapsed(STALL_DEBOUNCE_SECS)) {
            stallBackoff = true;
            stallCount++;
            stallTimer.restart();
            return 0;
        }
        return rpm;
    }

    // ---- Feed auto-unjam ----

    /**
     * Rotor stator current, in amps, that means the feed is jammed rather than working. The rotor's
     * velocity loop runs on torque current, so a jam shows up as sustained stator draw well above
     * what stirring loose fuel takes. Tunable so it can be set against the logs.
     */
    private static final DoubleSubscriber autoUnjamAmps =
            Telemetry.tunable("DyeRotor/AutoUnjamAmps", 55.0);

    /**
     * How long after feeding starts, or restarts after an unjam, before the current is watched. The
     * rotor spins up against a packed bed during this time and draws jam-level current
     * legitimately.
     */
    private static final double AUTO_UNJAM_ARM_SECS = 0.4;

    /** How long the current must stay above the threshold, without a break, to count as a jam. */
    private static final double AUTO_UNJAM_DEBOUNCE_SECS = 0.4;

    /** How long the rotor and feeder reverse before feeding resumes. */
    private static final double AUTO_UNJAM_REVERSE_SECS = 0.25;

    /** Runs from the start or restart of feeding; the arm time is measured against it. */
    private final Timer feedTimer = new Timer();
    /** Runs while the current is continuously above the threshold. */
    private final Timer jamTimer = new Timer();
    /** Runs while reversing. */
    private final Timer reverseTimer = new Timer();

    private boolean feeding = false;
    private boolean jamTiming = false;
    private boolean reversing = false;
    private int autoUnjamCount = 0;

    /**
     * Returns the system state while {@code INDEX_MAX} is wanted, reversing briefly when the rotor
     * is jammed. A jam is rotor stator current above the threshold for the debounce time, ignoring
     * the arm time after feeding starts or restarts. After the reverse, feeding resumes and every
     * timer starts over, so a jam that comes straight back earns another reverse only after another
     * full arm and debounce. Follows {@link #idleRotorRpmWithStallCheck} and uses only software so
     * no current-limit config writes are involved.
     */
    private SystemState indexMaxWithAutoUnjam() {
        if (!feeding) {
            feeding = true;
            jamTiming = false;
            reversing = false;
            feedTimer.restart();
            return SystemState.INDEX_MAX;
        }

        if (reversing) {
            if (!reverseTimer.hasElapsed(AUTO_UNJAM_REVERSE_SECS)) {
                return SystemState.UNJAM;
            }
            reversing = false;
            jamTiming = false;
            feedTimer.restart();
            return SystemState.INDEX_MAX;
        }

        if (!feedTimer.hasElapsed(AUTO_UNJAM_ARM_SECS)) {
            jamTiming = false;
            return SystemState.INDEX_MAX;
        }

        boolean jammedNow = Math.abs(rotor.getStatorCurrent()) > autoUnjamAmps.get();
        if (!jammedNow) {
            jamTiming = false;
            return SystemState.INDEX_MAX;
        }
        if (!jamTiming) {
            jamTiming = true;
            jamTimer.restart();
            return SystemState.INDEX_MAX;
        }
        if (jamTimer.hasElapsed(AUTO_UNJAM_DEBOUNCE_SECS)) {
            reversing = true;
            autoUnjamCount++;
            reverseTimer.restart();
            return SystemState.UNJAM;
        }
        return SystemState.INDEX_MAX;
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

    /**
     * Handles the state transition. {@code INDEX_MAX} may briefly resolve to {@code UNJAM} through
     * {@link #indexMaxWithAutoUnjam}; leaving {@code INDEX_MAX} for any reason clears that.
     */
    private SystemState handleStateTransition() {
        if (wantedState != WantedState.INDEX_MAX) {
            feeding = false;
            jamTiming = false;
            reversing = false;
        }
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case INDEX_MAX -> indexMaxWithAutoUnjam();
            case IDLE_SLOW_INDEX -> SystemState.IDLE_SLOW_INDEX;
            case UNJAM -> SystemState.UNJAM;
        };
    }

    /** Applies the states. */
    private void applyStates() {
        double wantedRPMSpin = 0;
        double wantedRPMIndex = 0;
        switch (systemState) {
            case OFF:
                rotor.stop();
                feeder.stop();
                return;
            case INDEX_MAX:
                wantedRPMSpin = 120; // Set to 120 for higher feed rate, 100 was previous
                wantedRPMIndex = indexMaxFeederRPM.get();
                break;
            case IDLE_SLOW_INDEX:
                wantedRPMSpin = idleRotorRpmWithStallCheck(IDLE_ROTOR_RPM);
                break;
            case UNJAM:
                // Both backwards: the rotor unpacks the bed while the feeder pushes fuel back out.
                wantedRPMSpin = -100;
                wantedRPMIndex = -1000;
                break;
        }
        rotor.setRotorVelocity(wantedRPMSpin);
        feeder.setFeederVelocity(wantedRPMIndex);
    }

    @Getter private final Rotor rotor;
    @Getter private final Feeder feeder;

    /**
     * Creates and registers the dye rotor subsystem with the specified configuration.
     *
     * @param rotorConfig the rotor config
     * @param feederConfig the feeder config
     */
    public DyeRotor(RotorConfig rotorConfig, FeederConfig feederConfig) {
        this.rotor = new Rotor(rotorConfig);
        this.feeder = new Feeder(feederConfig);

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

        Telemetry.logState("DyeRotor/WantedState", wantedState);
        Telemetry.logState("DyeRotor/SystemState", systemState);
        Telemetry.log("DyeRotor/RotorStallBackoff", stallBackoff);
        Telemetry.log("DyeRotor/RotorStallCount", stallCount);
        Telemetry.log("DyeRotor/AutoUnjamActive", reversing);
        Telemetry.log("DyeRotor/AutoUnjamCount", autoUnjamCount);
    }
}
