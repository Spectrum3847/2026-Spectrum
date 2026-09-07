package frc.robot.subsystems.intakeExtension;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotSim;
import frc.robot.subsystems.intakeExtension.IntakeExtension.Left.LeftConfig;
import frc.robot.subsystems.intakeExtension.IntakeExtension.Right.RightConfig;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.LinearConfig;
import frc.spectrumLib.sim.LinearSim;
import frc.spectrumLib.telemetry.Telemetry;
import lombok.Getter;

public class IntakeExtension implements Subsystem {

    public static class Left extends Mechanism {

        public static class LeftConfig extends Config {

            @Getter private final double initPosition = 0;
            @Getter private final double triggerTolerance = 0.317637;

            @Getter private final double zeroSpeed = -0.1;
            @Getter private final double holdMaxSpeedRPM = 18;

            @Getter private final double maxRotations = 3.652821;
            @Getter private final double minRotations = 0.0;

            @Getter private final double supplyCurrentLimit = 80;
            @Getter private final double statorCurrentLimit = 80;
            @Getter private final double lowerSupplyCurrentLimit = 40;
            @Getter private final double lowerSupplyCurrentTime = 1;

            @Getter private final double positionKp = 4.2;
            @Getter private final double positionKi = 0;
            @Getter private final double positionKd = 0;
            @Getter private final double positionKv = 0.39;
            @Getter private final double positionKs = 0;
            @Getter private final double positionKa = 0;
            @Getter private final double positionKg = -0.017;
            @Getter private final double gearRatio = 3.5;
            @Getter private final double rampPeriod = 0.02;

            @Getter private final double mmCruiseVelocity = 15.246559;
            @Getter private final double mmAcceleration = 76.232794;
            @Getter private final double mmJerk = 0;
            @Getter private final double slowMmCruiseVelocity = 7.623279;
            @Getter private final double slowMmAcceleration = 38.116397;
            @Getter private final double slowMmJerk = 0;

            /**
             * Full travel of the extension, hard stop to hard stop. {@code maxRotations} is this
             * distance divided by the drum circumference, so inch values convert through it.
             */
            @Getter private final double travelInches = 11.5;

            /*
             * Agitate (after 4414): pull the extension in a short stroke, and if that pull meets
             * fuel resistance (stator current above the loaded threshold) push it back out so the
             * dye rotor can keep turning instead of compressing the bed. Each pull that finishes
             * without meeting resistance means the fuel has drawn down, so the extension keeps
             * going all the way in, still backing off if it hits fuel on the way. Roughly one
             * pull-push cycle per second.
             */
            @Getter private final double agitateStrokeInches = 2.0;
            @Getter private final double agitateHalfPeriodSecs = 0.5;
            @Getter private final double agitateLoadedDebounceSecs = 0.06;
            @Getter private final double agitateSettleToleranceInches = 0.25;
            /** A pull that covers this share of the stroke without loading counts as unloaded. */
            @Getter private final double agitatePullSuccessFraction = 0.5;
            /**
             * Outside a launch, this many stalled pulls in a row means the fuel is already packed
             * and the agitate parks instead of burning current. A launch always agitates.
             */
            @Getter private final int agitateIdleGiveUpPulls = 3;

            /*
             * The two axes are joined through the intake roller, so the roller works badly if one
             * side is further in than the other. Each side runs its own loop, so when they drift
             * apart by more than the max skew the agitate stops moving and brings both to their
             * midpoint until they are within the resume skew of each other.
             */
            @Getter private final double agitateMaxSkewInches = 0.5;
            @Getter private final double agitateResumeSkewInches = 0.2;
            /**
             * A skew hold that has not converged in this long releases anyway, and the hold cannot
             * re-arm for the cooldown. In the 2026-09-06 22:25 log one hold lasted 126 s because
             * the two encoders disagreed by a constant quarter inch, which no amount of commanding
             * the midpoint can remove.
             */
            @Getter private final double agitateSkewHoldTimeoutSecs = 1.0;

            @Getter private final double agitateSkewHoldCooldownSecs = 3.0;
            /**
             * The skew baseline is learned while sitting fully extended: both sides are on the same
             * hard stop, so any encoder difference there is zero offset, not physical skew. The
             * sides must be this slow for this long before the reading is taken.
             */
            @Getter private final double skewBaselineSettleSecs = 0.3;

            @Getter private final double skewBaselineMaxRPM = 18;
            /** Larger differences at the stop are a pushed or slipped side, not a zero offset. */
            @Getter private final double skewBaselineMaxInches = 0.5;

            /**
             * Full extend target. 99 rather than 100 because the extension has to be almost all the
             * way out to intake, but a 100 target sits on the hard stop drawing 20 to 35 A stator
             * for as long as it is out (2026-09-06 22:25 log: 204 s of it).
             */
            @Getter private final double fullExtendPercent = 99;

            /*
             * Full extend drives out, then drops to neutral. The motors are configured to coast, so
             * a collision pushes the intake in instead of breaking it. If it gets pushed in by the
             * re-extend distance and then sits still for the steady time, it drives back out. It
             * also re-extends when intake is pressed again. "At target" is within the tolerance of
             * the full extend target, or stalled short of it for the steady time.
             */
            @Getter private final double extendReextendInches = 2.0;
            @Getter private final double extendSteadySecs = 0.3;
            @Getter private final double extendSteadyMaxRPM = 18;
            @Getter private final double extendAtTargetToleranceInches = 0.25;

            /**
             * Once the intake has deployed it cannot come all the way back in: the kicker bar arms
             * are in the way. Measured 2026-09-06 23:40 with the extension resting against them:
             * 1.715 rot on both sides, 47 percent of travel. No command may ask for less than 50
             * percent once the extension has ever been out past it, so it never touches the arms;
             * only a power-on with the intake stowed (position near zero) clears the latch.
             */
            @Getter private final double deployedRetractFloorPercent = 50;

            /** The deployed floor in drum rotations. */
            public double deployedRetractFloorRotations() {
                return deployedRetractFloorPercent / 100.0 * maxRotations;
            }

            /** Converts a distance along the extension's travel into drum rotations. */
            public double inchesToRotations(double inches) {
                return inches / travelInches * maxRotations;
            }

            @Getter private final double homingVoltage = 6;
            @Getter private final double homingStallRPM = 50.0;
            @Getter private final double homingMinTimeSecs = 0.3;
            @Getter private final double homingStallDebounceSecs = 0.15;
            @Getter private final double homingTimeoutSecs = 3.0;

            @Getter private final double intakeX = Units.inchesToMeters(70);
            @Getter private final double intakeY = Units.inchesToMeters(23);
            @Getter private final double extensionMass = 10.0;
            @Getter private final double drumRadiusMeters = Units.inchesToMeters(0.5010597711);
            @Getter private final double extensionGearing = 3.5;
            @Getter private final double angle = 180;
            @Getter private final double staticLength = 10;
            @Getter private final double movingLength = 55;
            @Getter private final double lineWidth = 20;
            @Getter private final double maxExtensionHeight = 40;

            /**
             * Initializes the left intake extension motor configuration, including motion control,
             * current limits, soft limits, braking, gearing, and positive rotation direction.
             */
            public LeftConfig() {
                super("IntakeExtensionLeft", 4, Rio.CANIVORE);
                configMinMaxRotations(minRotations, maxRotations);
                configPIDGains(0, positionKp, positionKi, positionKd);
                configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
                configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
                configGravityType(false);
                configOpenLoopRamps(rampPeriod);
                configClosedLoopRamps(rampPeriod);
                configSupplyCurrentLimit(supplyCurrentLimit, true);
                configStatorCurrentLimit(statorCurrentLimit, true);
                configLowerSupplyCurrentLimit(lowerSupplyCurrentLimit);
                configLowerSupplyCurrentTime(lowerSupplyCurrentTime);
                configGearRatio(gearRatio);
                configForwardTorqueCurrentLimit(statorCurrentLimit);
                configReverseTorqueCurrentLimit(statorCurrentLimit);
                configForwardSoftLimit(maxRotations, true);
                configReverseSoftLimit(minRotations, true);
                // Always coast: a collision must be able to push the intake in.
                configNeutralBrakeMode(false);
                configClockwise_Positive();
            }
        }

        @Getter private final LeftConfig config;
        @Getter private IntakeExtensionSim sim;

        /**
         * Creates and initializes the left intake extension axis.
         *
         * @param config configuration for the left intake extension axis
         */
        public Left(LeftConfig config) {
            super(config);
            this.config = config;

            simulationInit();
            Telemetry.print(getName() + " Subsystem Initialized");
        }
        /** Runs the periodic update. */
        @Override
        public void periodic() {
            logBatteryUsage();
            if (currentCommandKey == null) {
                // Built once; this used to concatenate eight strings every loop.
                currentCommandKey = getName() + "/CurrentCommand";
                positionKey = getName() + "/Position";
                rpmKey = getName() + "/RPM";
            }
            Telemetry.log(currentCommandKey, getCurrentCommandName());
            logDiagnostics(getName());
            Telemetry.log(positionKey, getPositionRotations(), "rotations");
            if (Telemetry.slowLogThisLoop()) {
                Telemetry.log(rpmKey, getVelocityRPM(), "RPM");
            }
        }

        private String currentCommandKey;
        private String positionKey;
        private String rpmKey;

        /** Closed-loop Motion Magic to an absolute rotation target. */
        public void goToRotations(double rotations) {
            setMMPosition(() -> rotations);
        }

        /**
         * Moves the axis to a rotation target using a dynamic motion profile.
         *
         * @param rotations the target position in rotations
         * @param cruiseVelocity the motion profile's cruise velocity
         * @param acceleration the motion profile's acceleration
         * @param jerk the motion profile's jerk
         */
        public void goToRotationsSlow(
                double rotations, double cruiseVelocity, double acceleration, double jerk) {
            setDynMMPositionVoltage(
                    () -> rotations, () -> cruiseVelocity, () -> acceleration, () -> jerk);
        }

        /** Open-loop voltage that bypasses soft limits. Used to drive into the hard stop. */
        public void driveHomingVoltage(double volts) {
            setVoltageOutputNoSoftLimit(() -> volts);
        }

        /** Seeds this axis's encoder to a known starting position (rotations). */
        public void setInitialPosition(double rotations) {
            if (isAttached()) {
                motor.setPosition(rotations);
            }
        }

        /** Re-zeroes this axis at the fully-extended hard stop. */
        public void zeroAtMax() {
            setMotorPosition(() -> config.getMaxRotations());
        }

        /** Holds the axis (neutral output). */
        public void leftStop() {
            stop();
        }

        /** Initializes the left extension axis simulation when the mechanism is attached. */
        public void simulationInit() {
            if (isAttached()) {
                sim = new IntakeExtensionSim(RobotSim.leftView, motor);
            }
        }

        class IntakeExtensionSim extends LinearSim {
            /**
             * Initializes the intake extension simulation model.
             *
             * @param mech the mechanism visualization to bind to the simulation
             * @param motor the motor simulation state driving the model
             */
            public IntakeExtensionSim(Mechanism2d mech, TalonFX motor) {
                super(
                        new LinearConfig(
                                        config.getIntakeX(),
                                        config.getIntakeY(),
                                        config.getExtensionGearing(),
                                        config.getDrumRadiusMeters())
                                .setAngle(config.getAngle())
                                .setMovingLength(config.getMovingLength())
                                .setStaticLength(config.getStaticLength())
                                .setMaxHeight(config.getMaxExtensionHeight())
                                .setLineWidth(config.getLineWidth())
                                .setColor(new Color8Bit(Color.kLightGray))
                                .setReversedLinkage(true),
                        mech,
                        motor,
                        config.getName());
            }
        }
    }

    public static class Right extends Mechanism {

        public static class RightConfig extends Config {

            @Getter private final double homingStallRPM;
            @Getter private final double homingMinTimeSecs;
            @Getter private final double homingStallDebounceSecs;
            @Getter private final double homingTimeoutSecs;
            @Getter private final double homingVoltage;

            /**
             * Creates a right-axis configuration using the left-axis control and homing settings.
             *
             * @param left the left-axis configuration supplying shared settings
             */
            public RightConfig(LeftConfig left) {
                super("IntakeExtensionRight", 5, Rio.CANIVORE);
                setAttached(left.isAttached());
                configMinMaxRotations(left.getMinRotations(), left.getMaxRotations());
                configPIDGains(0, left.getPositionKp(), left.getPositionKi(), left.getPositionKd());
                configFeedForwardGains(
                        left.getPositionKs(),
                        left.getPositionKv(),
                        left.getPositionKa(),
                        left.getPositionKg());
                configMotionMagic(
                        left.getMmCruiseVelocity(), left.getMmAcceleration(), left.getMmJerk());
                configGravityType(false);
                configOpenLoopRamps(left.getRampPeriod());
                configClosedLoopRamps(left.getRampPeriod());
                configSupplyCurrentLimit(left.getSupplyCurrentLimit(), true);
                configStatorCurrentLimit(left.getStatorCurrentLimit(), true);
                configLowerSupplyCurrentLimit(left.getLowerSupplyCurrentLimit());
                configLowerSupplyCurrentTime(left.getLowerSupplyCurrentTime());
                configGearRatio(left.getGearRatio());
                configForwardTorqueCurrentLimit(left.getStatorCurrentLimit());
                configReverseTorqueCurrentLimit(left.getStatorCurrentLimit());
                configForwardSoftLimit(left.getMaxRotations(), true);
                configReverseSoftLimit(left.getMinRotations(), true);
                // Always coast: a collision must be able to push the intake in.
                configNeutralBrakeMode(false);
                configCounterClockwise_Positive();

                this.homingStallRPM = left.getHomingStallRPM();
                this.homingMinTimeSecs = left.getHomingMinTimeSecs();
                this.homingStallDebounceSecs = left.getHomingStallDebounceSecs();
                this.homingTimeoutSecs = left.getHomingTimeoutSecs();
                this.homingVoltage = left.getHomingVoltage();
            }
        }

        @Getter private final RightConfig config;

        /**
         * Creates a new Right instance.
         *
         * @param config the config
         */
        public Right(RightConfig config) {
            super(config);
            this.config = config;
            Telemetry.print(getName() + " Subsystem Initialized");
        }
        /** Runs the periodic update. */
        @Override
        public void periodic() {
            logBatteryUsage();
            if (currentCommandKey == null) {
                // Built once; this used to concatenate eight strings every loop.
                currentCommandKey = getName() + "/CurrentCommand";
                positionKey = getName() + "/Position";
                rpmKey = getName() + "/RPM";
            }
            Telemetry.log(currentCommandKey, getCurrentCommandName());
            logDiagnostics(getName());
            Telemetry.log(positionKey, getPositionRotations(), "rotations");
            if (Telemetry.slowLogThisLoop()) {
                Telemetry.log(rpmKey, getVelocityRPM(), "RPM");
            }
        }

        private String currentCommandKey;
        private String positionKey;
        private String rpmKey;

        /** Closed-loop Motion Magic to an absolute rotation target. */
        public void goToRotations(double rotations) {
            setMMPosition(() -> rotations);
        }

        /** Slow (dynamic Motion Magic voltage) move to a rotation target. */
        public void goToRotationsSlow(
                double rotations, double cruiseVelocity, double acceleration, double jerk) {
            setDynMMPositionVoltage(
                    () -> rotations, () -> cruiseVelocity, () -> acceleration, () -> jerk);
        }

        /** Open-loop voltage that bypasses soft limits. Used to drive into the hard stop. */
        public void driveHomingVoltage(double volts) {
            setVoltageOutputNoSoftLimit(() -> volts);
        }

        /** Seeds this axis's encoder to a known starting position (rotations). */
        public void setInitialPosition(double rotations) {
            if (isAttached()) {
                motor.setPosition(rotations);
            }
        }

        /** Re-zeroes this axis at the fully-extended hard stop. */
        public void zeroAtMax() {
            setMotorPosition(() -> config.getMaxRotations());
        }

        /** Holds the axis (neutral output). */
        public void rightStop() {
            stop();
        }
    }

    public static class IntakeExtensionConfig {

        @Getter private final LeftConfig leftConfig;
        @Getter private final RightConfig rightConfig;

        /**
         * Creates an intake extension configuration from the configurations for both axes.
         *
         * @param leftConfig the left-axis configuration
         * @param rightConfig the right-axis configuration
         */
        public IntakeExtensionConfig(LeftConfig leftConfig, RightConfig rightConfig) {
            this.leftConfig = leftConfig;
            this.rightConfig = rightConfig;
        }
    }

    // ---- State Machine ----

    public enum WantedState {
        STOPPED,
        FULL_EXTEND,
        CONDITIONAL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
        AGITATE,
        /** Agitate only if intaking sent the extension out; otherwise leave it stopped. */
        CONDITIONAL_AGITATE,
        RESYNC,
    }

    public enum SystemState {
        STOPPED,
        FULL_EXTEND,
        FULL_RETRACT,
        SLOW_CLOSE,
        AGITATE,
        HOMING,
    }

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState systemState = SystemState.STOPPED;
    private SystemState previousSystemState = SystemState.STOPPED;
    private boolean sentOutByIntakeState = false;
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
            case STOPPED -> SystemState.STOPPED;
            case FULL_EXTEND -> {
                sentOutByIntakeState = true;
                yield SystemState.FULL_EXTEND;
            }
            case CONDITIONAL_EXTEND -> sentOutByIntakeState
                    ? SystemState.FULL_EXTEND
                    : SystemState.STOPPED;
            case FULL_RETRACT -> {
                sentOutByIntakeState = false;
                yield SystemState.FULL_RETRACT;
            }
            case SLOW_CLOSE -> SystemState.SLOW_CLOSE;
            case AGITATE -> SystemState.AGITATE;
            case CONDITIONAL_AGITATE -> sentOutByIntakeState
                    ? SystemState.AGITATE
                    : SystemState.STOPPED;
            case RESYNC -> SystemState.HOMING;
        };
    }

    /** Applies the outputs associated with the current system state. */
    private void applyStates() {
        switch (systemState) {
            case FULL_EXTEND:
                applyFullExtend();
                break;
            case FULL_RETRACT:
                applyFullRetract();
                break;
            case SLOW_CLOSE:
                commandBoth(60, true);
                break;
            case AGITATE:
                applyAgitate();
                break;
            case HOMING:
                applyHoming();
                break;
            case STOPPED:
                left.leftStop();
                right.rightStop();
                return;
        }
    }

    /**
     * Commands both intake extension axes to the specified extension percentage.
     *
     * @param percent the target extension percentage
     * @param slow whether to use the slow motion profile
     */
    private void commandBoth(double percent, boolean slow) {
        commandBothRotations(left.percentToRotations(() -> percent), slow);
    }

    /**
     * Commands both intake extension axes to an absolute rotation target.
     *
     * @param rotations the target position in drum rotations
     * @param slow whether to use the slow motion profile
     */
    private void commandBothRotations(double rotations, boolean slow) {
        // Nothing may ask for less than the deployed floor; see retractLimitRotations().
        rotations = Math.max(rotations, retractLimitRotations());
        // Both sides get the same target. An earlier version offset the right target by the
        // learned skew baseline; in the 2026-09-06 23:47 log the baseline swung to -0.96 rot and
        // the right side was told to sit 3 in further out than the left, so it never came in and
        // the two fought through the roller link. The baseline is only used to judge skew now.
        final double rightRotations = rotations;
        if (slow) {
            left.goToRotationsSlow(
                    rotations,
                    config.getLeftConfig().getSlowMmCruiseVelocity(),
                    config.getLeftConfig().getSlowMmAcceleration(),
                    config.getLeftConfig().getSlowMmJerk());
            right.goToRotationsSlow(
                    rightRotations,
                    config.getLeftConfig().getSlowMmCruiseVelocity(),
                    config.getLeftConfig().getSlowMmAcceleration(),
                    config.getLeftConfig().getSlowMmJerk());
        } else {
            left.goToRotations(rotations);
            right.goToRotations(rightRotations);
        }
    }

    // ---- Deployed retract floor ----

    /** Latched once the extension has been out past the floor; cleared only near zero. */
    private boolean deployed = false;

    /**
     * The furthest-in position any command may ask for. Zero until the intake first deploys, then
     * the kicker-bar floor for the rest of the power cycle. Reading near zero again (a power-on
     * with the intake stowed) clears the latch, since that is the only way it can be in there.
     */
    private double retractLimitRotations() {
        return deployed
                ? config.getLeftConfig().deployedRetractFloorRotations()
                : config.getLeftConfig().getMinRotations();
    }

    /** Updates the deployed latch from the measured position. Runs once per loop. */
    private void updateDeployedLatch() {
        LeftConfig cfg = config.getLeftConfig();
        double floor = cfg.deployedRetractFloorRotations();
        double position = (left.getPositionRotations() + right.getPositionRotations()) / 2.0;
        if (position >= floor) {
            deployed = true;
        } else if (position <= cfg.getMinRotations() + cfg.inchesToRotations(0.5)) {
            deployed = false;
        }
    }

    // ---- Agitate ----

    /**
     * Stator current, on either axis, above which a pull is treated as compressing fuel. The
     * threshold starts at {@code Start} when agitate begins and ramps linearly to {@code End} over
     * {@code RampSecs}, so the longer a launch runs the harder the agitate is allowed to push.
     *
     * <p>Sized from the 2026-09-06 22:25 log. At a fixed 40 A, 103 of 156 pulls tripped after a
     * median 0.63 in of a 2 in stroke, but pulls that never met fuel also peaked at a median 39 A
     * and 46 A at the 90th percentile (9 Hz sampling, so true peaks were higher). 40 A was the
     * free-motion noise floor, not a fuel detector. The loaded fraction fell from 65 to 76 percent
     * in the first 4 s of a burst to 44 percent at 6 to 10 s and 0 past 10 s, so the bed does draw
     * down, and the ramp lets the agitate follow it in. Both ends are tunable from NetworkTables.
     */
    private static final DoubleSubscriber agitateLoadedStatorAmpsStart =
            Telemetry.tunable("IntakeExtension/AgitateLoadedStatorAmpsStart", 55.0);

    private static final DoubleSubscriber agitateLoadedStatorAmpsEnd =
            Telemetry.tunable("IntakeExtension/AgitateLoadedStatorAmpsEnd", 75.0);

    private static final DoubleSubscriber agitateLoadedRampSecs =
            Telemetry.tunable("IntakeExtension/AgitateLoadedRampSecs", 8.0);

    /** FPGA time the current agitate began; the loaded threshold ramps from here. */
    private double agitateStartTime = 0;

    /** The loaded threshold in effect this loop, for the ramp. */
    private double agitateLoadedThreshold(double now) {
        double ramp = Math.max(agitateLoadedRampSecs.get(), 0.01);
        double frac = MathUtil.clamp((now - agitateStartTime) / ramp, 0.0, 1.0);
        double start = agitateLoadedStatorAmpsStart.get(), end = agitateLoadedStatorAmpsEnd.get();
        return start + (end - start) * frac;
    }

    private final Timer agitateTimer = new Timer();
    /** True while pushing back out to the outer position, false while pulling in. */
    private boolean agitateOut = false;
    /** Where the current pull started from and where a push-out returns to (rotations). */
    private double agitateOuterRotations = 0;
    /** True once a pull has finished unloaded and the extension is heading all the way in. */
    private boolean agitateFullRetract = false;
    /** True once the extension has reached the retracted stop; it just holds there. */
    private boolean agitateRetracted = false;
    /** FPGA time the stator current was last below the loaded threshold during a pull. */
    private double agitateLastUnloaded = 0;
    /** Whether the current pull has met fuel resistance. */
    private boolean agitateLoaded = false;

    /**
     * Current-aware agitate, after 4414.
     *
     * <p>Pull in one stroke from the outer position. If either axis's stator current stays above
     * the loaded threshold for the debounce time, the pull is compressing fuel: push back out to
     * the outer position so the dye rotor can keep turning, then pull again after half a period. If
     * the pull reaches its target and the half period ends without ever loading, the fuel has drawn
     * down, so keep pulling all the way in. Meeting fuel on the way in pushes out one stroke from
     * wherever it was and resumes the cycle from there. Reaching the retracted stop ends the
     * agitate: the extension holds there and is no longer counted as sent out by intaking.
     *
     * <p>Position decisions use the average of the two encoders. If the sides drift apart past the
     * max skew, the cycle pauses and both are brought to their midpoint first; see {@link
     * #holdForSkew}.
     */
    private void applyAgitate() {
        LeftConfig cfg = config.getLeftConfig();
        final double minRot = retractLimitRotations();
        final double maxRot = cfg.getMaxRotations();
        final double stroke = cfg.inchesToRotations(cfg.getAgitateStrokeInches());
        final double tolerance = cfg.inchesToRotations(cfg.getAgitateSettleToleranceInches());
        final double leftPos = left.getPositionRotations();
        final double rightPos = right.getPositionRotations();
        final double position = (leftPos + rightPos) / 2.0;
        final double now = Timer.getFPGATimestamp();

        if (previousSystemState != SystemState.AGITATE) {
            agitateOuterRotations = MathUtil.clamp(position, minRot, maxRot);
            agitateRetracted = agitateOuterRotations <= minRot + tolerance;
            agitateSkewHold = false;
            agitateStartTime = now;
            agitateStalledPulls = 0;
            agitateIdleParked = false;
            startAgitatePull(now, position);
        }

        // Outside a launch the agitate is only prep. Once a few pulls in a row have stalled the
        // fuel is already packed and more pulling is just current, so park at the outer position
        // until a launch starts (the wanted state becomes plain AGITATE) or the state changes.
        if (wantedState != WantedState.CONDITIONAL_AGITATE) {
            agitateIdleParked = false;
        }
        if (agitateIdleParked) {
            commandBothRotations(agitateOuterRotations, true);
            return;
        }

        if (holdForSkew(leftPos, rightPos, position, now)) {
            return;
        }

        if (agitateRetracted) {
            sentOutByIntakeState = false;
            commandBothRotations(minRot, true);
            return;
        }

        if (agitateOut) {
            // Push-out phase: sit at the outer position for half a period, then pull again.
            if (agitateTimer.hasElapsed(cfg.getAgitateHalfPeriodSecs())) {
                if (wantedState == WantedState.CONDITIONAL_AGITATE
                        && agitateStalledPulls >= cfg.getAgitateIdleGiveUpPulls()) {
                    agitateIdleParked = true;
                    commandBothRotations(agitateOuterRotations, true);
                    return;
                }
                startAgitatePull(now, position);
            } else {
                commandBothRotations(agitateOuterRotations, true);
                return;
            }
        }

        // Pull phase.
        double statorAmps = Math.max(left.getStatorCurrent(), right.getStatorCurrent());
        if (statorAmps < agitateLoadedThreshold(now)) {
            agitateLastUnloaded = now;
        }
        agitateLoaded = (now - agitateLastUnloaded) >= cfg.getAgitateLoadedDebounceSecs();

        double target =
                agitateFullRetract ? minRot : Math.max(agitateOuterRotations - stroke, minRot);

        if (agitateLoaded) {
            // Fuel is resisting: back off one stroke from here so the bed is not compressed.
            if (agitateFullRetract) {
                agitateOuterRotations = MathUtil.clamp(position + stroke, minRot, maxRot);
            }
            agitateFullRetract = false;
            agitateStalledPulls++;
            agitateOut = true;
            agitateTimer.restart();
            commandBothRotations(agitateOuterRotations, true);
            return;
        }

        if (agitateFullRetract) {
            if (position <= minRot + tolerance) {
                agitateRetracted = true;
                sentOutByIntakeState = false;
            }
        } else if (agitateTimer.hasElapsed(cfg.getAgitateHalfPeriodSecs())) {
            // Judge the pull by how far it moved, not by whether it closed on the target. The
            // position loop settles a quarter to a third of an inch short under load (2026-09-06
            // 23:29 log: 80 unloaded pulls travelled a median 1.71 in of the 2 in stroke), and
            // requiring the last quarter inch threw every one of them back out.
            double travelled = agitatePullStartRotations - position;
            if (travelled >= stroke * cfg.getAgitatePullSuccessFraction()) {
                // Most of an unloaded pull: the fuel has drawn down, so keep going all the way in.
                agitateFullRetract = true;
                agitateStalledPulls = 0;
                target = minRot;
            } else {
                // Barely moved but never loaded either. Keep the cadence: push out and retry.
                agitateStalledPulls++;
                agitateOut = true;
                agitateTimer.restart();
                commandBothRotations(agitateOuterRotations, true);
                return;
            }
        }

        commandBothRotations(target, true);
    }

    /** Position when the current pull began; the pull is judged by travel from here. */
    private double agitatePullStartRotations = 0;

    /** Pulls in a row that loaded or failed to move; a pull that gets through resets it. */
    private int agitateStalledPulls = 0;

    /** True when a non-launch agitate has given up and is parked at the outer position. */
    private boolean agitateIdleParked = false;

    /** True while the agitate is paused to bring the two axes back together. */
    private boolean agitateSkewHold = false;

    /**
     * Keeps the two axes roughly together. When they are further apart than the max skew, commands
     * both to their midpoint and holds there until they are within the resume skew, then restarts
     * the current phase so its timer and loaded detector do not count the hold.
     *
     * @return true if the hold is active and the caller should not command anything else
     */
    private boolean holdForSkew(double leftPos, double rightPos, double midpoint, double now) {
        LeftConfig cfg = config.getLeftConfig();
        // Physical skew is the encoder difference minus the zero offset learned at the hard stop.
        double skew = Math.abs(leftPos - rightPos - skewBaselineRotations);

        if (agitateSkewHold) {
            boolean converged = skew <= cfg.inchesToRotations(cfg.getAgitateResumeSkewInches());
            boolean timedOut = now - skewHoldStart >= cfg.getAgitateSkewHoldTimeoutSecs();
            if (converged || timedOut) {
                agitateSkewHold = false;
                if (timedOut) {
                    skewHoldCooldownUntil = now + cfg.getAgitateSkewHoldCooldownSecs();
                    skewHoldTimeouts++;
                }
                // Start the phase over so the hold time is not charged to it.
                agitateTimer.restart();
                agitateLastUnloaded = now;
                return false;
            }
        } else if (skew > cfg.inchesToRotations(cfg.getAgitateMaxSkewInches())
                && now >= skewHoldCooldownUntil) {
            agitateSkewHold = true;
            skewHoldStart = now;
        } else {
            return false;
        }

        double target = MathUtil.clamp(midpoint, retractLimitRotations(), cfg.getMaxRotations());
        commandBothRotations(target, true);
        return true;
    }

    private double skewHoldStart = 0;
    private double skewHoldCooldownUntil = 0;
    private int skewHoldTimeouts = 0;

    // ---- Full extend: drive out, then coast ----

    private enum ExtendPhase {
        DRIVING,
        COASTING,
    }

    private ExtendPhase extendPhase = ExtendPhase.DRIVING;
    /** Set by {@link #requestExtend()}; makes the next full-extend loop drive out again. */
    private boolean extendRequested = false;
    /** Where the extension was when it started coasting; re-extend is measured from here. */
    private double coastStartRotations = 0;
    /** Times a stalled full extend has been taken as the new out point. */
    private int outPointRelearns = 0;

    private final Timer extendSteadyTimer = new Timer();
    private boolean extendSteadyTiming = false;

    /** Asks full extend to drive out again, for when intake is pressed again while coasting. */
    public void requestExtend() {
        extendRequested = true;
    }

    /**
     * Drives to the full extend target, then goes neutral so the coasting motors let a collision
     * push the intake in. While coasting, a push of the re-extend distance that then settles for
     * the steady time, or a new extend request, drives it out again.
     */
    private void applyFullExtend() {
        LeftConfig cfg = config.getLeftConfig();
        final double position = (left.getPositionRotations() + right.getPositionRotations()) / 2.0;
        final double target = left.percentToRotations(cfg::getFullExtendPercent);
        final double tolerance = cfg.inchesToRotations(cfg.getExtendAtTargetToleranceInches());
        final boolean still =
                Math.abs(left.getVelocityRPM()) < cfg.getExtendSteadyMaxRPM()
                        && Math.abs(right.getVelocityRPM()) < cfg.getExtendSteadyMaxRPM();

        if (previousSystemState != SystemState.FULL_EXTEND || extendRequested) {
            extendPhase = ExtendPhase.DRIVING;
            extendRequested = false;
            extendSteadyTiming = false;
        }

        boolean steady = false;
        if (still) {
            if (!extendSteadyTiming) {
                extendSteadyTiming = true;
                extendSteadyTimer.restart();
            }
            steady = extendSteadyTimer.hasElapsed(cfg.getExtendSteadySecs());
        } else {
            extendSteadyTiming = false;
        }

        switch (extendPhase) {
            case DRIVING -> {
                boolean atTarget = position >= target - tolerance;
                if (atTarget || steady) {
                    if (!atTarget && sidesAgree()) {
                        // Stalled short of the target with both sides in the same place: the stop
                        // is here, not where the encoders say. Take this as the new out point so
                        // retract and agitate still work after a motor restart or a power-on with
                        // the intake out. If the sides disagree, one of them is bound up short of
                        // the other (2026-09-06 23:47 log: the right lagged 0.2 to 0.8 rot at each
                        // stall, was zeroed there, then crept to the real stop and read past max)
                        // and zeroing would write that lag into its frame. Leave it alone.
                        left.zeroAtMax();
                        right.zeroAtMax();
                        skewBaselineRotations = 0;
                        outPointRelearns++;
                    }
                    // Either got there or is stalled short of it. Let go.
                    extendPhase = ExtendPhase.COASTING;
                    coastStartRotations = cfg.getMaxRotations();
                    extendSteadyTiming = false;
                    left.leftStop();
                    right.rightStop();
                } else {
                    commandBoth(cfg.getFullExtendPercent(), false);
                }
            }
            case COASTING -> {
                double pushedIn = coastStartRotations - position;
                if (pushedIn >= cfg.inchesToRotations(cfg.getExtendReextendInches()) && steady) {
                    extendPhase = ExtendPhase.DRIVING;
                    extendSteadyTiming = false;
                    commandBoth(cfg.getFullExtendPercent(), false);
                } else {
                    left.leftStop();
                    right.rightStop();
                    if (position >= target - tolerance) {
                        // Resting on the stop: the one place the encoder offset can be learned.
                        updateSkewBaseline();
                        resyncAtStopWhileCoasting(steady);
                    }
                }
            }
        }
    }

    /**
     * While coasting on the extended stop, an encoder that reads past the stop has slipped: in the
     * 2026-09-06 23:47 log the right side drifted from 3.7 to 4.4 rot with no current applied,
     * against a 3.65 max, three times. Both sides are physically on the stop, so once they have
     * been still for the steady time any side reading off the max by more than the tolerance is set
     * back to it. This is the same correction the stall relearn makes, applied at rest.
     */
    private void resyncAtStopWhileCoasting(boolean steady) {
        if (!steady) {
            return;
        }
        LeftConfig cfg = config.getLeftConfig();
        double max = cfg.getMaxRotations();
        double tol = cfg.inchesToRotations(cfg.getExtendAtTargetToleranceInches());
        boolean leftOff = Math.abs(left.getPositionRotations() - max) > tol;
        boolean rightOff = Math.abs(right.getPositionRotations() - max) > tol;
        // Only when the sides read the same place: then both are on the stop and a reading off
        // max is a frame error. If they disagree, one side is short of the stop and its reading
        // is the truth about that, not a zero to correct.
        if ((leftOff || rightOff) && sidesAgree()) {
            left.zeroAtMax();
            right.zeroAtMax();
            skewBaselineRotations = 0;
            restResyncs++;
        }
    }

    /** True when the two encoders read within the skew-baseline tolerance of each other. */
    private boolean sidesAgree() {
        LeftConfig cfg = config.getLeftConfig();
        return Math.abs(left.getPositionRotations() - right.getPositionRotations())
                <= cfg.inchesToRotations(cfg.getSkewBaselineMaxInches());
    }

    private int restResyncs = 0;

    // ---- Full retract: drive in, hold if it stalls short ----

    private final Timer retractSteadyTimer = new Timer();
    private boolean retractSteadyTiming = false;
    private boolean retractHolding = false;
    private double retractHoldRotations = 0;

    /**
     * Drives to fully retracted. If the frame is off (an out point learned against an obstruction
     * puts "zero" past the real retracted stop), the extension stalls short; rather than push into
     * the stop at the current limit, it holds where it stopped. The next clean full extend
     * re-learns the frame at the real stop.
     */
    private void applyFullRetract() {
        LeftConfig cfg = config.getLeftConfig();
        final double position = (left.getPositionRotations() + right.getPositionRotations()) / 2.0;
        final double tolerance = cfg.inchesToRotations(cfg.getExtendAtTargetToleranceInches());
        final boolean still =
                Math.abs(left.getVelocityRPM()) < cfg.getExtendSteadyMaxRPM()
                        && Math.abs(right.getVelocityRPM()) < cfg.getExtendSteadyMaxRPM();

        if (previousSystemState != SystemState.FULL_RETRACT) {
            retractHolding = false;
            retractSteadyTiming = false;
        }

        if (retractHolding) {
            commandBothRotations(retractHoldRotations, false);
            return;
        }

        boolean atTarget = position <= retractLimitRotations() + tolerance;
        if (still && !atTarget) {
            if (!retractSteadyTiming) {
                retractSteadyTiming = true;
                retractSteadyTimer.restart();
            } else if (retractSteadyTimer.hasElapsed(cfg.getExtendSteadySecs())) {
                retractHolding = true;
                retractHoldRotations = position;
                commandBothRotations(retractHoldRotations, false);
                return;
            }
        } else {
            retractSteadyTiming = false;
        }
        commandBothRotations(retractLimitRotations(), false);
    }

    // ---- Skew baseline ----

    /** Left minus right encoder reading when both sides sit on the extended hard stop. */
    private double skewBaselineRotations = 0;

    private final Timer skewBaselineTimer = new Timer();
    private boolean skewBaselineSettling = false;

    /**
     * Learns the encoder zero offset between the two sides. Called while fully extended: once both
     * sides have been nearly stationary for the settle time they are on the hard stop together, so
     * whatever difference the encoders show is offset, and it is recorded as the baseline that the
     * skew hold measures against. A resync zeroes both at that stop, so it resets the baseline.
     */
    private void updateSkewBaseline() {
        LeftConfig cfg = config.getLeftConfig();
        boolean still =
                Math.abs(left.getVelocityRPM()) < cfg.getSkewBaselineMaxRPM()
                        && Math.abs(right.getVelocityRPM()) < cfg.getSkewBaselineMaxRPM();
        if (!still) {
            skewBaselineSettling = false;
            return;
        }
        if (!skewBaselineSettling) {
            skewBaselineSettling = true;
            skewBaselineTimer.restart();
            return;
        }
        if (skewBaselineTimer.hasElapsed(cfg.getSkewBaselineSettleSecs())) {
            double leftPos = left.getPositionRotations();
            double rightPos = right.getPositionRotations();
            double tol = cfg.inchesToRotations(cfg.getExtendAtTargetToleranceInches());
            double target = left.percentToRotations(cfg::getFullExtendPercent);
            double maxOffset = cfg.inchesToRotations(cfg.getSkewBaselineMaxInches());
            double offset = leftPos - rightPos;
            // Only a reading with both sides at the stop and a small difference is a zero offset.
            // Anything else is a side that has been pushed or has slipped, and must not become the
            // reference: in the 2026-09-06 23:47 log the right side read 4.44 rot against a 3.65
            // max and the baseline followed it to -0.96 rot.
            if (leftPos >= target - tol
                    && rightPos >= target - tol
                    && Math.abs(offset) <= maxOffset) {
                skewBaselineRotations = offset;
            }
        }
    }

    /** Begins a pull-in phase: resets the loaded detector and the half-period timer. */
    private void startAgitatePull(double now, double position) {
        agitatePullStartRotations = position;
        agitateOut = false;
        agitateFullRetract = false;
        agitateLoaded = false;
        agitateLastUnloaded = now;
        agitateTimer.restart();
    }

    // ---- Resync / stall homing ----

    private final Timer homingTimer = new Timer();
    private boolean leftHomed = false;
    private boolean rightHomed = false;
    private double leftLastMoving = 0;
    private double rightLastMoving = 0;

    /**
     * Drives both extension axes through the homing process and marks each axis complete when a
     * stall is detected, the homing timeout is reached, or the axis is unattached.
     */
    private void applyHoming() {
        if (previousSystemState != SystemState.HOMING) {
            homingTimer.restart();
            leftHomed = false;
            rightHomed = false;
            leftLastMoving = 0;
            rightLastMoving = 0;
            // Both encoders are about to be zeroed at the same stop, so there is no offset.
            skewBaselineRotations = 0;
        }

        double homingTimeout = config.getLeftConfig().getHomingTimeoutSecs();
        double homingVoltage = config.getLeftConfig().getHomingVoltage();
        boolean timedOut = homingTimer.get() >= homingTimeout;

        if (left.isAttached()) {
            if (!leftHomed) {
                if (detectLeftStall() || timedOut) {
                    if (timedOut) Telemetry.print("IntakeExtension: LEFT resync timed out");
                    left.zeroAtMax();
                    left.leftStop();
                    leftHomed = true;
                } else {
                    left.driveHomingVoltage(homingVoltage);
                }
            } else {
                left.leftStop();
            }
        } else {
            leftHomed = true;
        }

        if (right.isAttached()) {
            if (!rightHomed) {
                if (detectRightStall() || timedOut) {
                    if (timedOut) Telemetry.print("IntakeExtension: RIGHT resync timed out");
                    right.zeroAtMax();
                    right.rightStop();
                    rightHomed = true;
                } else {
                    right.driveHomingVoltage(homingVoltage);
                }
            } else {
                right.rightStop();
            }
        } else {
            rightHomed = true;
        }
    }

    /**
     * Determines whether the left extension is stalled according to its homing thresholds.
     *
     * @return {@code true} if the minimum homing time has elapsed and the axis has remained below
     *     the stall velocity threshold for the debounce period, {@code false} otherwise
     */
    private boolean detectLeftStall() {
        double now = homingTimer.get();
        LeftConfig cfg = config.getLeftConfig();
        if (Math.abs(left.getVelocityRPM()) >= cfg.getHomingStallRPM()) {
            leftLastMoving = now;
        }
        return isStalled(
                now, leftLastMoving, cfg.getHomingMinTimeSecs(), cfg.getHomingStallDebounceSecs());
    }

    /**
     * Determines whether the right extension axis has stalled during homing.
     *
     * @return {@code true} if the axis has remained below the stall velocity threshold for the
     *     required debounce period after the minimum homing time; {@code false} otherwise
     */
    private boolean detectRightStall() {
        double now = homingTimer.get();
        RightConfig cfg = config.getRightConfig();
        if (Math.abs(right.getVelocityRPM()) >= cfg.getHomingStallRPM()) {
            rightLastMoving = now;
        }
        return isStalled(
                now, rightLastMoving, cfg.getHomingMinTimeSecs(), cfg.getHomingStallDebounceSecs());
    }

    /**
     * Determines whether motion has remained below the stall threshold for the required duration.
     *
     * @param now the current homing time in seconds
     * @param lastMoving the most recent time motion was detected in seconds
     * @param minTimeSecs the minimum elapsed time before stall detection begins
     * @param stallDebounceSecs the required duration without motion before reporting a stall
     * @return {@code true} if the minimum time has elapsed and the debounce duration has passed,
     *     {@code false} otherwise
     */
    private boolean isStalled(
            double now, double lastMoving, double minTimeSecs, double stallDebounceSecs) {
        if (now < minTimeSecs) {
            return false;
        }
        return (now - lastMoving) >= stallDebounceSecs;
    }

    /**
     * Determines whether resynchronization has completed for both extension axes.
     *
     * @return {@code true} if the system is homing and both axes are homed, {@code false} otherwise
     */
    public boolean isResyncComplete() {
        return systemState == SystemState.HOMING && leftHomed && rightHomed;
    }

    /**
     * Creates a command that resynchronizes both intake extension axes and stops them when
     * complete.
     *
     * @return the resynchronization command
     */
    public Command resyncCommand() {
        return startEnd(
                        () -> setWantedState(WantedState.RESYNC),
                        () -> setWantedState(WantedState.STOPPED))
                .until(this::isResyncComplete)
                .withName("IntakeExtension.resync");
    }

    // ---- Subsystem plumbing ----

    @Getter private final Left left;
    @Getter private final Right right;
    @Getter private final IntakeExtensionConfig config;

    /**
     * Initializes the intake extension subsystem with its left and right axis configurations.
     *
     * @param config the configuration for both intake extension axes
     */
    public IntakeExtension(IntakeExtensionConfig config) {
        this.config = config;
        this.left = new Left(config.getLeftConfig());
        this.right = new Right(config.getRightConfig());

        // Deliberately no encoder zeroing here. The TalonFX keeps counting across robot-code
        // restarts, so zeroing in the constructor threw the position away on every deploy. On
        // 2026-09-06 at 23:08 the code was deployed with the intake extended: both encoders read
        // zero there, "full extend" was already at the stop, and every agitate pull toward zero
        // pulled toward fully out, so the intake never came in. The zero is therefore wherever the
        // extension was at motor power-on, which should be retracted. If it was not, or a motor
        // restarts mid-match, the first full extend that stalls short of its target takes that
        // stall as the new out point (see applyFullExtend), so the frame fixes itself in use.

        this.register();
        Telemetry.print("Intake Extension Subsystem Initialized");
    }

    /** Sets the initial encoder position for both extension axes from the configured position. */
    private void setInitialPosition() {
        double initialRotations =
                left.degreesToRotations(() -> config.getLeftConfig().getInitPosition());
        left.setInitialPosition(initialRotations);
        right.setInitialPosition(initialRotations);
    }

    /** Resets both extension axes to their maximum positions. */
    public void resetCurrentPositionToMax() {
        left.zeroAtMax();
        right.zeroAtMax();
    }

    /**
     * Creates a command that resets both extension axes to their maximum positions.
     *
     * @return a command that performs the position reset once
     */
    public Command resetCurrentPositionToMaxCommand() {
        return new InstantCommand(this::resetCurrentPositionToMax);
    }

    /**
     * Creates a command that resets both extension axes to their initial positions.
     *
     * @return a command that sets the initial encoder positions
     */
    public Command resetToInitialPos() {
        return new InstantCommand(this::setInitialPosition);
    }

    /**
     * Creates a command that drops both extension axes into coast so the mechanism can be moved by
     * hand. Runs while disabled, which is the only time it is useful.
     *
     * @return the coast-mode command
     */
    public Command coastModeCommand() {
        return new InstantCommand(() -> setBrakeMode(false))
                .ignoringDisable(true)
                .withName("IntakeExtension.coastMode");
    }

    /**
     * Sets the brake mode for both intake extension axes.
     *
     * @param isInBrake whether to enable brake mode
     */
    public void setBrakeMode(boolean isInBrake) {
        left.setBrakeMode(isInBrake);
        right.setBrakeMode(isInBrake);
    }

    /**
     * Provides the simulation model for the left intake extension axis.
     *
     * @return the left intake extension simulation model
     */
    public Left.IntakeExtensionSim getSim() {
        return left.getSim();
    }

    /**
     * Reports the extension position as a percentage of its configured range.
     *
     * @return the current extension position percentage
     */
    public double getPositionPercentage() {
        return left.getPositionPercentage();
    }
    /** Runs the periodic update. */
    @Override
    public void periodic() {
        updateDeployedLatch();
        systemState = handleStateTransition();
        applyStates();

        Telemetry.log("IntakeExtension/WantedState", wantedState.toString());
        Telemetry.log("IntakeExtension/SystemState", systemState.toString());
        // Dashboard: average of both sides as a percentage of travel, so a bad zero is visible.
        Telemetry.logDash(
                "IntakeExtension/Percent",
                (left.getPositionPercentage() + right.getPositionPercentage()) / 2.0,
                "percent");
        Telemetry.log("IntakeExtension/LeftHomed", leftHomed);
        Telemetry.log("IntakeExtension/RightHomed", rightHomed);
        Telemetry.log("IntakeExtension/Agitate/Out", agitateOut);
        Telemetry.log("IntakeExtension/Agitate/Loaded", agitateLoaded);
        Telemetry.log("IntakeExtension/Agitate/FullRetract", agitateFullRetract);
        Telemetry.log("IntakeExtension/Agitate/Retracted", agitateRetracted);
        Telemetry.log("IntakeExtension/Agitate/SkewHold", agitateSkewHold);
        Telemetry.log("IntakeExtension/Agitate/SkewHoldTimeouts", skewHoldTimeouts);
        Telemetry.log("IntakeExtension/ExtendPhase", extendPhase.toString());
        Telemetry.log("IntakeExtension/OutPointRelearns", outPointRelearns);
        Telemetry.log("IntakeExtension/RestResyncs", restResyncs);
        Telemetry.log("IntakeExtension/Deployed", deployed);
        Telemetry.log(
                "IntakeExtension/RetractLimitRotations", retractLimitRotations(), "rotations");
        Telemetry.log("IntakeExtension/Agitate/StalledPulls", agitateStalledPulls);
        Telemetry.log("IntakeExtension/Agitate/IdleParked", agitateIdleParked);
        if (systemState == SystemState.AGITATE) {
            Telemetry.log(
                    "IntakeExtension/Agitate/LoadedThresholdAmps",
                    agitateLoadedThreshold(Timer.getFPGATimestamp()),
                    "amps");
        }
        Telemetry.log("IntakeExtension/SkewBaselineRotations", skewBaselineRotations, "rotations");
        Telemetry.log(
                "IntakeExtension/SkewRotations",
                left.getPositionRotations() - right.getPositionRotations(),
                "rotations");
        Telemetry.log("IntakeExtension/Agitate/OuterRotations", agitateOuterRotations, "rotations");

        previousSystemState = systemState;
    }
}
