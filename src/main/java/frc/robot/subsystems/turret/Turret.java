package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
import frc.robot.subsystems.vision.Vision;
import frc.spectrumLib.framework.RobotLoop;
import frc.spectrumLib.hardware.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import frc.spectrumLib.sim.ArmConfig;
import frc.spectrumLib.sim.ArmSim;
import frc.spectrumLib.telemetry.*;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import lombok.*;

public class Turret extends Mechanism {

    public static class TurretConfig extends Config {
        @Getter private final double initPosition = 0;
        /** Position error (degrees) within which the turret counts as on target for a shot. */
        @Getter private final double triggerTolerance = 2;

        @Getter private final double unwrapTolerance = 10;
        @Getter private final double unwrapExitMargin = 45;
        @Getter private final double shootOnMoveLatencySec = 0.03;

        @Getter private Rotation2d zeroOffsetFromRobotFront = Rotation2d.fromDegrees(180);

        /* Turret config settings */
        @Getter private final double currentLimit = 80;
        @Getter private final double supplyCurrentLowerLimit = 40;
        @Getter private final double supplyCurrentLowerTime = 1.0;
        /**
         * Stator and torque-current ceiling, in amps.
         *
         * <p>Was 80. The 24t-to-30t belt that skipped on 2026-09-05 sits one 4.75:1 reduction below
         * the motor and 6.7:1 above the turret, so it carries motor torque times 4.75 -- about 7.4
         * N-m at 80 A, and the 18:38 log showed 86 to 96 A sustained with a 105 A peak, which is
         * 8.3 to 9.7 N-m. On a 24t HTD-5 pulley that is 435 to 507 N of belt tension.
         *
         * <p>50 A puts it near 290 N. That was the right trade while the belt was skipping and it
         * is the wrong one now: at 50 the turret jams more often than it did at 80, and it costs
         * shots. On the 2026-09-07 03:25 log the turret sat pinned at the limit -- stator p50 50.0
         * A, peak 55.8 -- for a thirteen-second unwrap from -49 to +72 deg, and two launch presses
         * landed inside that sweep with everything else ready. Tracking a target is not a
         * high-torque job, but unwrapping against a stop is, and the limit was set for the first.
         *
         * <p>Back to 80. If the belt starts skipping again, this is the first number to look at --
         * but read {@code Vision/TurretZero/SlipDegPerKiloDegTravel} before touching it, because
         * that is the measurement that says whether the belt is actually the problem.
         */
        @Getter private final double torqueCurrentLimit = 80;

        @Getter private final double positionKp = 800;
        @Getter private final double positionKi = 100;

        /**
         * Feedforward on the velocity setpoint that shoot-on-the-move puts in the Motion Magic
         * request, so the turret leads a moving target instead of lagging it. The turret is driven
         * with {@code MotionMagicVoltage}, so this is volts per rotation per second of turret, not
         * amps. A Kraken through 39.78:1 works out to about 5 V per rot/s; the 10 here overdrives
         * while tracking and is on the list to fit from a log (see the tuning handoff). Never tuned
         * on this robot.
         */
        @Getter private final double positionKv = 10;

        @Getter private final double positionKs = 0.6;
        @Getter private final double positionKa = 0;
        @Getter private final double positionKg = 0;
        @Getter private final double mmCruiseVelocity = 0.25;
        @Getter private final double mmAcceleration = 0.5;
        @Getter private final double mmJerk = 0;
        @Getter private final double peakVoltage = 6;

        @Getter private final double sensorToMechanismRatio = 39.78;

        /* Sim Configs */
        @Getter private final double turretX = Units.inchesToMeters(105); // Vertical Center

        @Getter private final double turretY = Units.inchesToMeters(75); // Horizontal Center
        @Getter private final double length = 1;

        /** Creates a new TurretConfig instance. */
        public TurretConfig() {
            super("Turret", 14, Rio.CANIVORE);
            configPIDGains(0, positionKp, positionKi, 0);
            configFeedForwardGains(positionKs, positionKv, positionKa, positionKg);
            configMotionMagic(mmCruiseVelocity, mmAcceleration, mmJerk);
            configForwardVoltageLimit(peakVoltage);
            configReverseVoltageLimit(-peakVoltage);
            configGearRatio(sensorToMechanismRatio);
            configCurrentLimits(
                    currentLimit,
                    torqueCurrentLimit,
                    supplyCurrentLowerLimit,
                    supplyCurrentLowerTime);
            configMinMaxRotations(-0.6, 0.5);
            configReverseSoftLimit(getMinRotations(), true);
            configForwardSoftLimit(getMaxRotations(), true);
            configNeutralBrakeMode(true);
            configContinuousWrap(false);
            configGravityType(false);
            configCounterClockwise_Positive();
            // The turret's feedforward is fit from logs, which needs voltage on every sample.
            setFastOutputLogging(true);
        }
    }

    public enum WantedState {
        OFF,
        IDLE,
        AIM_AT_TARGET,
        /** Shake side to side about where the turret is, to free fuel tucked against it. */
        UNJAM_SHAKE,
        /** Aim at the target with a slow sweep laid on top, so fuel cannot settle against it. */
        AIM_SWEEP,
        /**
         * Pit check: point at whatever tag the turret camera sees. See {@link #applyTestFollowTag}.
         */
        TEST_FOLLOW_TAG,
        /** Pit check: run limit to limit and back. See {@link #applyTestSweepLimits}. */
        TEST_SWEEP_LIMITS,
        /** Pit check: hold the turret's zero. Same output path as {@link WantedState#IDLE}. */
        TEST_ZERO,
        /**
         * Hold a fixed mechanism angle, set with {@link #setFixedAngleDegrees}. The set shots use
         * it: no pose, no target, the driver has parked the robot and the angle is the parking
         * spot's.
         */
        FIXED_ANGLE,
    }

    public enum SystemState {
        OFF,
        IDLE,
        AIM_AT_TARGET,
        UNJAM_SHAKE,
        AIM_SWEEP,
        TEST_FOLLOW_TAG,
        TEST_SWEEP_LIMITS,
        TEST_ZERO,
        FIXED_ANGLE,
    }

    // ---- Intake sweep ----

    /**
     * While intaking the turret drifts slowly back and forth about its aim, this far each side, one
     * full cycle per period. Slow on purpose: the point is to keep fuel from packing against the
     * turret (2026-09-06 logs: it jammed at the 80 A torque limit around -90 and -30 deg, mostly
     * while intaking), not to shake anything loose.
     */
    private static final double SWEEP_AMPLITUDE_DEG = 20;

    private static final double SWEEP_PERIOD_SECS = 6.0;

    /** Sine sweep offset for this loop, in degrees. */
    private double sweepOffsetDeg() {
        double phase =
                2.0 * Math.PI * (Timer.getFPGATimestamp() % SWEEP_PERIOD_SECS) / SWEEP_PERIOD_SECS;
        return SWEEP_AMPLITUDE_DEG * Math.sin(phase);
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
     * Handles the state transition.
     *
     * <p>No pose gate on the aiming states (removed 2026-09-19): the turret aims from whatever pose
     * the estimator has, seeded or not, rather than holding at zero until vision trusts it.
     */
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case IDLE -> SystemState.IDLE;
            case AIM_AT_TARGET -> SystemState.AIM_AT_TARGET;
            case UNJAM_SHAKE -> SystemState.UNJAM_SHAKE;
            case AIM_SWEEP -> SystemState.AIM_SWEEP;
            case TEST_FOLLOW_TAG -> SystemState.TEST_FOLLOW_TAG;
            case TEST_SWEEP_LIMITS -> SystemState.TEST_SWEEP_LIMITS;
            case TEST_ZERO -> SystemState.TEST_ZERO;
            case FIXED_ANGLE -> SystemState.FIXED_ANGLE;
        };
    }

    private SystemState previousSystemState = SystemState.OFF;

    // ---- Unjam shake ----

    /** Half the shake's swing: the turret goes this far each side of where it started. */
    private static final double SHAKE_AMPLITUDE_DEG = 10;
    /** Time at each side before flipping. A 20 deg step settles in well under this. */
    private static final double SHAKE_HALF_PERIOD_SECS = 0.4;

    private final Timer shakeTimer = new Timer();
    private double shakeCenterDegrees = 0;
    private boolean shakePositive = false;

    /**
     * Steps the turret between two positions ten degrees either side of where it was when the shake
     * began, flipping every half period. The centre is shifted if needed so both sides stay inside
     * the soft limits.
     */
    private void applyUnjamShake() {
        double minDeg = minLimitDegrees();
        double maxDeg = maxLimitDegrees();

        if (previousSystemState != SystemState.UNJAM_SHAKE) {
            shakeCenterDegrees =
                    MathUtil.clamp(
                            getPositionDegrees(),
                            minDeg + SHAKE_AMPLITUDE_DEG,
                            maxDeg - SHAKE_AMPLITUDE_DEG);
            shakePositive = false;
            shakeTimer.restart();
        }

        if (shakeTimer.hasElapsed(SHAKE_HALF_PERIOD_SECS)) {
            shakePositive = !shakePositive;
            shakeTimer.restart();
        }

        holdDegrees(
                shakeCenterDegrees + (shakePositive ? SHAKE_AMPLITUDE_DEG : -SHAKE_AMPLITUDE_DEG));
    }

    // ---- Test-mode pit checks ----
    //
    // Three held-to-run checks bound to the pilot D-pad in test mode (see Robot.configureBindings).
    // They are ordinary turret states, so they run under the same motor config, the same soft
    // limits, the same stall cut-out and the same Turret/* logging as a match: test mode runs
    // robotPeriodic() and the CommandScheduler exactly like teleop, because WPILib only disables
    // the scheduler in test when LiveWindow is enabled there and this robot never enables it.
    //
    // None of them touches the robot pose or ShotCalculator, so they are usable on a cart.
    //
    // Both moving checks drive the motor through holdDegrees -- PositionVoltage, gain slot 0,
    // the same request AIM_AT_TARGET uses (with a zero velocity feedforward, there being no target
    // velocity to feed). NOT Motion Magic. Both were written with setMMPosition first and the
    // difference is not subtle: on FRC_20260919_180624, 163.2 to 177.0 s, the follow check pinned
    // at 89.7 deg/s -- exactly the 0.25 rot/s mmCruiseVelocity -- and never asked for more than
    // 2.51 V of its 6 V ceiling, while AIM_AT_TARGET in the P8 match log runs p90 135 deg/s, p99
    // 385, and uses the full 6.11 V. The profile was throwing away more than half the authority
    // the mechanism had. Mechanism.setPositionWithVelocity's own javadoc says as much: profiling
    // introduces steady-state lag on a moving setpoint, which is what these checks track.
    //
    // The consequence for the sweep is real and deliberate: it now crosses the travel at teleop
    // speed rather than at a profiled 90 deg/s, so it reaches the turnaround fast. That is the
    // point -- it is meant to be the same mechanism behaviour a match sees -- but it is also the
    // one place here where "same as teleop" costs something, because teleop's own full-travel
    // move, the cable unwrap in applyAimAtTarget, is profiled for exactly that reason. If the
    // turnarounds look violent, this is the line to change, not the gains.

    /** Lowest soft limit of the travel, in degrees. */
    private double minLimitDegrees() {
        return config.getMinRotations() * 360.0;
    }

    /** Highest soft limit of the travel, in degrees. */
    private double maxLimitDegrees() {
        return config.getMaxRotations() * 360.0;
    }

    /**
     * Fraction of the measured tag bearing the follow check closes each loop.
     *
     * <p>The command is rebuilt from the <em>measured</em> angle every loop rather than integrated,
     * so this is the outer loop's gain and the turret converges on the tag with no standing error
     * whatever the exact degrees-per-tx scale is. That matters here: the turret camera is pitched
     * about 29 deg up, so a degree of {@code tx} is not quite a degree of turret azimuth. Writing
     * the law this way means the scale only has to be roughly right, not known: convergence needs
     * {@code 0 < TEST_FOLLOW_KP * scale < 2}, and the scale is somewhere near 1.
     *
     * <p>Was 0.5, which halved the commanded step every loop for no reason -- the inner position
     * loop is what should be doing the work. 1.0 means "point where the tag is".
     */
    private static final double TEST_FOLLOW_KP = 1.0;

    /**
     * Points the turret at whatever AprilTag the turret camera currently sees.
     *
     * <p>Deliberately built on the camera's own bearing to the tag ({@code tx}) rather than on a
     * field-relative aim: no pose, no alliance, no tag map, no {@link ShotCalculator}. Put a tag in
     * front of the robot on a cart and the turret should follow it. What it checks is the half of
     * the aiming chain the shot depends on and the pose cannot vouch for -- that the camera, the
     * turret's zero and the gearbox agree on which way is which, and by how much.
     *
     * <p>With no tag in view the turret holds its last command rather than falling back to zero, so
     * walking a tag out of frame parks it where it was instead of sending it across its travel.
     */
    private void applyTestFollowTag() {
        // Entering the state, take over from wherever the turret already is.
        if (previousSystemState != SystemState.TEST_FOLLOW_TAG) {
            commandedDegrees =
                    MathUtil.clamp(getPositionDegrees(), minLimitDegrees(), maxLimitDegrees());
        }

        Vision vision = Robot.getVision();
        boolean tagInView = false;
        if (vision != null && vision.isTurretTagInView()) {
            tagInView = true;
            // Limelight tx is positive with the tag right of the crosshair; the turret is
            // counter-clockwise-positive, so closing that bearing means going negative.
            double txDegrees = vision.getTurretTagBearingDegrees();
            Telemetry.logDash("Turret/Test/FollowTagTxDeg", txDegrees, "deg");
            commandedDegrees =
                    MathUtil.clamp(
                            getPositionDegrees() - TEST_FOLLOW_KP * txDegrees,
                            minLimitDegrees(),
                            maxLimitDegrees());
        }
        Telemetry.log("Turret/Test/FollowTagInView", tagInView);

        holdDegrees(commandedDegrees);
    }

    /**
     * How far inside each soft limit the sweep turns around.
     *
     * <p>Not zero: the Talon's own soft limit zeroes output in that direction, so a command sitting
     * exactly on it leaves a standing position error that never closes and the leg never reads as
     * arrived. A couple of degrees in is reachable and still shows the full travel.
     */
    private static final double TEST_SWEEP_MARGIN_DEG = 2.0;

    /** How close to a leg's target counts as having got there. */
    private static final double TEST_SWEEP_ARRIVAL_DEG = 3.0;

    /**
     * How long one leg may take before the sweep turns around anyway.
     *
     * <p>The full 396 deg of travel is about 4.4 s at the 0.25 rot/s Motion Magic cruise, so this
     * is several times the honest worst case. It exists so a turret that meets an obstacle
     * mid-sweep backs off instead of leaning on it: the check is meant to be run with people near
     * the robot.
     */
    private static final double TEST_SWEEP_LEG_TIMEOUT_SECS = 20.0;

    private final Timer sweepLegTimer = new Timer();
    private boolean sweepingTowardMax = true;

    /**
     * Runs the turret to one soft limit, then to the other, and keeps going while the button is
     * held.
     *
     * <p>This is the travel check: it shows the whole envelope is reachable, that nothing in the
     * cable path binds at either end, and -- read against {@code Turret/TravelTotalDeg} and the
     * reported angle at each end -- whether the belt is giving up teeth. The first leg goes to
     * whichever limit is further away, so the long run happens while somebody is still watching.
     *
     * <p>A leg that stalls or times out turns around rather than pushing. Reversing is also exactly
     * what releases the stall latch, so the sweep recovers itself.
     */
    private void applyTestSweepLimits() {
        double minDeg = minLimitDegrees() + TEST_SWEEP_MARGIN_DEG;
        double maxDeg = maxLimitDegrees() - TEST_SWEEP_MARGIN_DEG;

        if (previousSystemState != SystemState.TEST_SWEEP_LIMITS) {
            double position = getPositionDegrees();
            sweepingTowardMax = Math.abs(maxDeg - position) >= Math.abs(position - minDeg);
            sweepLegTimer.restart();
        }

        double target = sweepingTowardMax ? maxDeg : minDeg;
        boolean arrived = Math.abs(getPositionDegrees() - target) <= TEST_SWEEP_ARRIVAL_DEG;
        boolean gaveUp = stallLatched || sweepLegTimer.hasElapsed(TEST_SWEEP_LEG_TIMEOUT_SECS);

        if (arrived || gaveUp) {
            if (gaveUp && !arrived) {
                Telemetry.print(
                        String.format(
                                "Turret sweep check gave up on the leg to %.1f deg at %.1f deg"
                                        + " (%s) and is turning around. Something is in the way,"
                                        + " or the travel is not what the config says it is.",
                                target,
                                getPositionDegrees(),
                                stallLatched ? "stalled" : "timed out"));
            }
            sweepingTowardMax = !sweepingTowardMax;
            sweepLegTimer.restart();
            target = sweepingTowardMax ? maxDeg : minDeg;
        }

        Telemetry.log("Turret/Test/SweepTowardMax", sweepingTowardMax);
        holdDegrees(target);
    }

    // Whether the turret is unwrapping to avoid wire wrap.
    @Getter private boolean unwrapping = false;

    @Getter private int unwrapTargetN = 0;
    @Getter private double commandedDegrees = 0;
    @Getter private double mechOmegaRotPerSec = 0;

    // -- Turret angle history ---------------------------------------------------------------------

    /** How far back a turret angle can be looked up. Vision frames are at most a few tenths old. */
    private static final double ANGLE_HISTORY_SECONDS = 2.0;

    /**
     * Turret angle against FPGA time, so a vision frame can be paired with the angle the turret had
     * when the shutter opened rather than the angle it has when the solve arrives.
     *
     * <p>Stores the angle <b>net of zero corrections</b>: reported position minus {@link
     * #getZeroCorrectionTotalDegrees()} at the time of the sample. That quantity is continuous
     * through a {@link #applyZeroCorrectionDegrees} step, so when {@link #getAngleAt} adds the
     * current total back, every entry in the history is expressed in the current zero, and a frame
     * captured just before a correction reads the corrected angle rather than the stale one.
     */
    private final TimeInterpolatableBuffer<Rotation2d> angleHistory =
            TimeInterpolatableBuffer.createBuffer(ANGLE_HISTORY_SECONDS);

    private long angleSampleLoop = -1;

    /**
     * Adds this loop's turret angle to the history. Once per loop; later calls in the same loop do
     * nothing, so Vision (which runs before the scheduler and needs the sample to exist before it
     * looks up a frame) and {@link #periodic()} can both call it.
     *
     * <p>The sample is the latency-compensated position stamped with the current FPGA time, which
     * is what the frame timestamps from the cameras are compared against.
     */
    public void recordAngleSample() {
        if (!isAttached()) {
            return;
        }
        if (isPositionSuspect()) {
            // The reading is being held out; the last good sample is still the truth, and a frame
            // paired with a jumped angle aims the next shot at nothing.
            return;
        }
        long loop = RobotLoop.count();
        if (loop == angleSampleLoop) {
            return;
        }
        angleSampleLoop = loop;
        angleHistory.addSample(
                Timer.getFPGATimestamp(),
                Rotation2d.fromDegrees(
                        getLatencyCompensatedPositionDegrees() - zeroCorrectionTotalDegrees));
    }

    /**
     * The turret angle at an FPGA time, interpolated from the history and expressed in the current
     * zero. Empty when the history has nothing (turret detached, or nothing recorded yet).
     * Timestamps newer than the latest sample return the latest sample; older than the oldest
     * return the oldest.
     *
     * @param fpgaSeconds the time to look up, in the FPGA time base
     * @return the turret angle then, positive counter-clockwise, zero robot-forward
     */
    public Optional<Rotation2d> getAngleAt(double fpgaSeconds) {
        return angleHistory
                .getSample(fpgaSeconds)
                .map(angle -> angle.plus(Rotation2d.fromDegrees(zeroCorrectionTotalDegrees)));
    }

    /**
     * How fast the turret is turning relative to the robot, in rotations per second, taking the
     * larger of what was asked for and what the encoder measures.
     *
     * <p>{@link #getMechOmegaRotPerSec()} is the shoot-on-the-move feedforward: a setpoint, and one
     * that {@code IDLE} hard-sets to zero while the turret is still slewing home at cruise. Vision
     * gates its turret-camera estimates and its zero trim on "turret still", and a gate that read
     * zero during a 90 deg/s slew was passing frames whose pushed mount transform lagged the image
     * by several degrees. The measured velocity catches that; the commanded one still catches the
     * first loop of a slew before the encoder shows it.
     *
     * @return the larger magnitude of the commanded and measured mechanism angular velocity
     */
    public double getSlewOmegaRotPerSec() {
        return Math.max(Math.abs(mechOmegaRotPerSec), Math.abs(getVelocityRPM() / 60.0));
    }

    /** Applies the states. */
    private void applyStates() {
        switch (systemState) {
            case OFF:
                unwrapping = false;
                clearStallLatch();
                stop();
                return;
            case IDLE:
                // TEST_ZERO shares this branch on purpose: the pit check for "go back to zero"
                // should exercise the exact output path the robot uses to sit at zero in a match,
                // not a second one that could behave differently.
            case TEST_ZERO:
                holdDegrees(0);
                return;
            case FIXED_ANGLE:
                // Same output path as IDLE, at the set shot's angle instead of zero. A half turn
                // from wherever the turret was aiming runs under PositionVoltage like IDLE's own
                // up-to-216 deg return to zero; the soft limits, current limit and stall cut-out
                // are the guard, as they are there.
                holdDegrees(
                        MathUtil.clamp(fixedAngleDegrees, minLimitDegrees(), maxLimitDegrees()));
                return;
            case AIM_AT_TARGET:
                applyAimAtTarget(0.0);
                return;
            case AIM_SWEEP:
                applyAimAtTarget(sweepOffsetDeg());
                return;
            case UNJAM_SHAKE:
                applyUnjamShake();
                return;
            case TEST_FOLLOW_TAG:
                applyTestFollowTag();
                return;
            case TEST_SWEEP_LIMITS:
                applyTestSweepLimits();
                return;
        }
    }

    @Getter private final TurretConfig config;

    @Getter private TurretSim sim;

    /** Angle {@link WantedState#FIXED_ANGLE} holds, mechanism degrees from zero. */
    @Getter private double fixedAngleDegrees = 0;

    /**
     * Sets the angle {@link WantedState#FIXED_ANGLE} holds. Clamped to the soft limits when
     * applied.
     *
     * @param degrees mechanism angle, degrees from zero
     */
    public void setFixedAngleDegrees(double degrees) {
        fixedAngleDegrees = degrees;
    }

    /**
     * Creates a new Turret instance.
     *
     * @param config the config
     */
    public Turret(TurretConfig config) {
        super(config);
        this.config = config;

        seedFromZeroReference();
        simulationInit();
        Telemetry.print(getName() + " Subsystem Initialized");
    }

    /**
     * Reported mechanism angle, in degrees, when the turret is parked at its true zero.
     *
     * <p>Measured on PM_2026 on 2026-09-19: with the turret squared to its zero, the rotor's
     * absolute position was 0.667480 rotations, which the device reports as 0.0166015625 mechanism
     * rotations. This constant is what makes {@link #seedFromZeroReference()} able to recover the
     * zero without anyone pointing the turret by hand.
     *
     * <p>It is a property of where the motor sits on the belt, so it survives power cycles and code
     * deploys but <b>not</b> a skipped belt tooth, a re-tensioned belt, or a motor swap. If shots
     * start leaving by a constant angle after any of those, this number is the first thing to
     * re-measure: park the turret at zero, read {@code Position} off the Talon in Tuner X, and put
     * that here.
     */
    private static final double ZERO_REFERENCE_DEGREES = 5.9765625;

    /** How long to wait at boot for the first position frame off the CAN bus. */
    private static final double BOOT_SIGNAL_TIMEOUT_SECONDS = 0.25;

    /** The angle the turret came up reading, before the zero reference was applied. */
    @Getter private double bootPositionDegrees = Double.NaN;

    /**
     * Puts the turret's zero back where it belongs at code start.
     *
     * <p>Phoenix 6 seeds the position register from the rotor's absolute position at power-on, not
     * from zero -- "The Talon FX and CANcoder sensors are always initialized to their absolute
     * position in Phoenix 6" (CTRE's Phoenix 5 to 6 migration guide). One rotor turn is 360 / 39.78
     * = 9.05 deg of turret, so the turret comes up reading somewhere in that 9 deg band, set by
     * wherever the rotor magnet happened to stop, and never at 0. That is the whole reason shots
     * used to leave by a constant angle that changed between runs.
     *
     * <p>The rotor's absolute position is repeatable for a given turret angle, so {@link
     * #ZERO_REFERENCE_DEGREES} is all that is needed to undo it: subtract it, and what the turret
     * reads at its parked zero becomes 0. The correction is only unique within one rotor turn, so
     * the difference is wrapped into +/- 4.5 deg -- park the turret within half a rotor turn of
     * zero before a power cycle and it lands right, park it further out and it snaps to the wrong
     * turn.
     *
     * <p>It must therefore run only after a real power cycle. A code restart or a roboRIO reboot
     * leaves the Talon's count intact and correct, and re-seeding then would throw a good zero away
     * (2026-09-19: the seed ran on every code start, and a deploy with the turret at 150 deg would
     * have told it it was at zero). Two signals separate the cases, see {@link #decideBootZero}: a
     * raw reading outside the band a power-on can produce, and a raw reading that matches the
     * position persisted by {@link #persistRawPosition} to the quarter degree.
     */
    private void seedFromZeroReference() {
        if (!isAttached()) {
            return;
        }
        bootPositionDegrees =
                rotationsToDegrees(
                        () ->
                                motor.getPosition()
                                        .waitForUpdate(BOOT_SIGNAL_TIMEOUT_SECONDS)
                                        .getValueAsDouble());

        final double rotorTurnDegrees = 360.0 / config.getSensorToMechanismRatio();
        OptionalDouble persisted = readPersistedRawDegrees();
        bootDecision = decideBootZero(bootPositionDegrees, persisted, rotorTurnDegrees);

        if (!bootDecision.startsWith("seeded")) {
            // The Talon kept its count: it was a code restart, not a power cycle.
            lastPersistedRawDegrees = bootPositionDegrees;
            Telemetry.print(
                    String.format(
                            "Turret zero carried over: reads %.3f deg and the Talon did not lose"
                                    + " its count (%s). No seed applied.",
                            bootPositionDegrees, bootDecision),
                    PrintPriority.HIGH);
            return;
        }

        double corrected = bootPositionDegrees - ZERO_REFERENCE_DEGREES;
        corrected -= rotorTurnDegrees * Math.round(corrected / rotorTurnDegrees);
        final double seeded = corrected;

        motor.setPosition(degreesToRotations(() -> seeded));
        // The file now describes a frame that no longer exists; rewrite it on the first loop.
        lastPersistedRawDegrees = Double.NaN;
        Telemetry.print(
                String.format(
                        "Turret zero seeded (%s): came up reading %.3f deg, zero reference is %.3f"
                                + " deg, so it is now %.3f deg. Re-zero with operator B if the"
                                + " turret was not parked within %.1f deg of zero.",
                        bootDecision,
                        bootPositionDegrees,
                        ZERO_REFERENCE_DEGREES,
                        seeded,
                        rotorTurnDegrees / 2),
                PrintPriority.HIGH);

        double edgeDistance = rotorTurnDegrees / 2 - Math.abs(seeded);
        if (edgeDistance <= WRAP_EDGE_MARGIN_DEG) {
            wrapEdgeAlert.setText(
                    String.format(
                            "Turret booted %.1f deg from zero, only %.1f deg from the rotor-turn"
                                    + " wrap. If it was really parked on the other side it is now"
                                    + " %.1f deg wrong: check the zero mark and re-zero with"
                                    + " operator B if needed.",
                            seeded, edgeDistance, rotorTurnDegrees));
            wrapEdgeAlert.set(true);
        }
    }

    // -- Boot zero: power cycle or code restart? -----------------------------------------------

    /**
     * Where the last known raw Talon position is kept between code starts. On the rio this is
     * {@code /home/lvuser/turret-position.txt}; never touched in simulation.
     */
    private static final File POSITION_FILE =
            new File(Filesystem.getOperatingDirectory(), "turret-position.txt");

    /** Raw reading change that earns a new write. */
    private static final double POSITION_FILE_WRITE_STEP_DEG = 0.5;

    /**
     * A raw reading this close to the persisted one means the Talon never lost its count. The count
     * does not move while the turret sits, so a match is exact apart from the write step and
     * encoder noise; a power cycle whose rotor absolute lands this close to where the turret was
     * left is the one case this cannot tell apart, and parking on zero makes it rare.
     */
    private static final double POSITION_FILE_MATCH_DEG = 0.25;

    /** How close to the wrap edge a seeded reading may land before it is called a coin flip. */
    private static final double WRAP_EDGE_MARGIN_DEG = 1.0;

    private double lastPersistedRawDegrees = Double.NaN;
    private boolean persistWasEnabled = false;

    @Getter private int positionFileWriteFailures = 0;

    /** What {@link #seedFromZeroReference} decided and why, for the log. */
    @Getter private String bootDecision = "not attached";

    private final Alert wrapEdgeAlert = new Alert("", AlertType.kWarning);

    private final ExecutorService positionWriter =
            Executors.newSingleThreadExecutor(
                    r -> {
                        Thread t = new Thread(r, "TurretPositionWriter");
                        t.setDaemon(true);
                        return t;
                    });

    /**
     * Decides whether the raw boot reading is a power cycle to seed from or a count to keep.
     *
     * <p>After a power cycle the Talon's position register is the rotor's absolute angle, which is
     * always inside one rotor turn of zero: 0 to 9.05 deg of turret if Phoenix reports the absolute
     * in [0, 1) rotations, -4.5 to +4.5 if it reports it in [-0.5, 0.5). Both bands are treated as
     * possible. A reading outside them cannot be a power cycle, so the count is kept. A reading
     * inside them that matches the persisted position is a code restart with the turret parked
     * there, so the count is kept. Anything else is seeded.
     *
     * @param rawDegrees the reading at code start
     * @param persisted the last position the previous code wrote, if any
     * @param rotorTurnDegrees one rotor turn in mechanism degrees
     * @return the decision, starting with {@code seeded} or {@code kept}
     */
    static String decideBootZero(
            double rawDegrees, OptionalDouble persisted, double rotorTurnDegrees) {
        double slack = 0.05;
        boolean inPowerOnBand =
                rawDegrees > -rotorTurnDegrees / 2 - slack && rawDegrees < rotorTurnDegrees + slack;
        if (!inPowerOnBand) {
            return String.format(
                    "kept: %.2f deg is outside the %.1f deg band a power-on can produce",
                    rawDegrees, rotorTurnDegrees);
        }
        if (persisted.isPresent()
                && Math.abs(rawDegrees - persisted.getAsDouble()) <= POSITION_FILE_MATCH_DEG) {
            return String.format("kept: matches the persisted %.2f deg", persisted.getAsDouble());
        }
        if (persisted.isPresent()) {
            return String.format(
                    "seeded: in the power-on band and %.2f deg from the persisted %.2f deg",
                    rawDegrees - persisted.getAsDouble(), persisted.getAsDouble());
        }
        return "seeded: in the power-on band and nothing persisted";
    }

    /**
     * Reads the position the previous code start persisted.
     *
     * @return the raw degrees, or empty when there is no file, it is unreadable, or in simulation
     */
    private static OptionalDouble readPersistedRawDegrees() {
        if (RobotBase.isSimulation() || !POSITION_FILE.isFile()) {
            return OptionalDouble.empty();
        }
        try {
            String text = Files.readString(POSITION_FILE.toPath(), StandardCharsets.UTF_8).trim();
            String[] parts = text.split("\\s+");
            if (parts.length == 0 || parts[0].isEmpty()) {
                return OptionalDouble.empty();
            }
            double value = Double.parseDouble(parts[0]);
            return Double.isFinite(value) ? OptionalDouble.of(value) : OptionalDouble.empty();
        } catch (IOException | NumberFormatException e) {
            Telemetry.print(
                    "Turret position file unreadable (" + e.getMessage() + "); treating as absent.",
                    PrintPriority.HIGH);
            return OptionalDouble.empty();
        }
    }

    /**
     * Writes the raw Talon position to {@link #POSITION_FILE} whenever it has moved more than
     * {@link #POSITION_FILE_WRITE_STEP_DEG} since the last write, and once more on every disable so
     * the file is exact while the robot sits. A dozen bytes on a daemon thread; the loop never
     * waits on the disk.
     */
    private void persistRawPosition() {
        if (!isAttached() || RobotBase.isSimulation()) {
            return;
        }
        boolean enabled = DriverStation.isEnabled();
        boolean disableEdge = persistWasEnabled && !enabled;
        persistWasEnabled = enabled;

        double raw = super.getPositionDegrees();
        if (!disableEdge
                && !Double.isNaN(lastPersistedRawDegrees)
                && Math.abs(raw - lastPersistedRawDegrees) < POSITION_FILE_WRITE_STEP_DEG) {
            return;
        }
        lastPersistedRawDegrees = raw;
        final String text = String.format("%.4f %.3f%n", raw, Timer.getFPGATimestamp());
        positionWriter.execute(
                () -> {
                    try {
                        Files.writeString(POSITION_FILE.toPath(), text, StandardCharsets.UTF_8);
                    } catch (IOException e) {
                        positionFileWriteFailures++;
                    }
                });
    }

    /** Runs the periodic update. */
    @Override
    public void periodic() {
        recordAngleSample();
        systemState = handleStateTransition();
        updateStallDetection();
        applyStates();
        previousSystemState = systemState;
        Telemetry.logState("Turret/WantedState", wantedState);
        Telemetry.logState("Turret/SystemState", systemState);
        // The turret is the most-watched mechanism on the dashboard, so its loop-rate values are
        // all logDash; the diagnostics are 10 Hz like every other mechanism. No RPM: the velocity
        // is logged below in rot/sec, next to the commanded rate it is fit against.
        logStandard("Turret", true, RpmLog.NONE);
        Telemetry.logDash("Turret/CommandedDegrees", commandedDegrees, "deg");
        updateTravel();
        persistRawPosition();
        Telemetry.logDash("Turret/PositionDegrees", getPositionDegrees(), "deg");
        Telemetry.log("Turret/TravelTotalDeg", travelTotalDegrees, "deg");
        Telemetry.logDash("Turret/PositionError", commandedDegrees - getPositionDegrees(), "deg");
        Telemetry.log("Turret/CommandedRotPerSec", mechOmegaRotPerSec, "rot/sec");
        // Measured, for fitting the feedforward against; the line above is what was asked for.
        Telemetry.log("Turret/VelocityRotPerSec", getVelocityRPM() / 60.0, "rot/sec");
        Telemetry.log("Turret/Unwrapping", unwrapping);
        Telemetry.logDash("Turret/ReadyToShoot", isReadyToShoot());
        updateEnvelopeAlert();
        Telemetry.log("Turret/BootPositionDegrees", bootPositionDegrees, "deg");
        Telemetry.log("Turret/BootDecision", bootDecision);
        Telemetry.log("Turret/PositionFileWriteFailures", positionFileWriteFailures);
        Telemetry.log("Turret/PositionSuspect", positionSuspect);
        Telemetry.log("Turret/PositionStepBudgetDegrees", positionStepBudgetDegrees, "deg");
        Telemetry.log("Turret/PositionStepsRejected", positionStepsRejected);
        Telemetry.log("Turret/PositionStepsAccepted", positionStepsAccepted);
        Telemetry.log("Turret/StallLatched", stallLatched);
        Telemetry.log("Turret/StallLatchCount", stallLatchCount);
    }

    /**
     * Declares the turret's current physical position to be its zero (facing away from the intake).
     * For use while disabled after a student has pointed the turret at its zero by hand, so a
     * turret that powered on pointing the wrong way can be fixed without a power cycle.
     *
     * @return the zeroing command
     */
    public Command zeroTurretCommand() {
        return new InstantCommand(
                        () -> {
                            if (isAttached()) {
                                motor.setPosition(
                                        degreesToRotations(() -> config.getInitPosition()));
                                clearStallLatch();
                                // The frame changed under the file: force a rewrite next loop
                                // so a code restart does not mistake the new zero for a power
                                // cycle.
                                lastPersistedRawDegrees = Double.NaN;
                                wrapEdgeAlert.set(false);
                            }
                        })
                .ignoringDisable(true)
                .withName("Turret.zeroHere");
    }

    /**
     * Shifts the encoder so the turret's reported angle matches where it is actually pointing.
     *
     * <p>The turret's only absolute reference is the rotor's own, which repeats every 9.05 deg of
     * turret: {@link #seedFromZeroReference()} uses it to recover the zero at boot, but it cannot
     * tell which of the ~40 rotor turns the turret is on, and it knows nothing about belt slip
     * after the fact. Get the zero wrong and every shot leaves by the same angle, in whichever
     * direction it is off, which is why the miss changes sides between runs rather than staying
     * put.
     *
     * <p>{@code errorDegrees} is actual minus reported, which is exactly what the turret camera
     * measures: its mount transform is built from this encoder, so its MegaTag1 heading is wrong by
     * the same amount the encoder is. Adding it makes reported equal actual.
     *
     * <p>The turret will physically move by this much on the next loop, because the position
     * setpoint has not changed but its frame of reference just did. The caller decides when that is
     * acceptable, and is expected to keep each step small: this is called repeatedly to chase
     * mechanical slip, not once with the whole error.
     *
     * @param errorDegrees actual turret angle minus reported turret angle
     */
    public void applyZeroCorrectionDegrees(double errorDegrees) {
        if (!isAttached()) {
            return;
        }
        final double corrected = getPositionDegrees() + errorDegrees;
        motor.setPosition(degreesToRotations(() -> corrected));

        // Logged, not printed. This is called several times a second to chase slip, and a console
        // line per step would bury everything else and go into the log as captured console output.
        zeroCorrectionTotalDegrees += errorDegrees;

        // The reading just jumped without the turret moving; do not bank that as travel.
        if (!Double.isNaN(lastTravelPositionDegrees)) {
            lastTravelPositionDegrees += errorDegrees;
        }
        Telemetry.logDash("Turret/ZeroCorrectionTotalDeg", zeroCorrectionTotalDegrees, "deg");
    }

    /**
     * Running total of every zero correction applied, in degrees.
     *
     * <p>Signed, so cancelling corrections cancel here too. A turret that only needed its power-on
     * zero fixed settles at a constant; one that keeps climbing is slipping, and the slope is how
     * fast.
     */
    @Getter private double zeroCorrectionTotalDegrees = 0;

    /**
     * Every degree this turret has turned, added up regardless of direction.
     *
     * <p>Slip is a function of distance travelled, not of time: the belt gives up a tooth when it
     * is pulled past one, and sitting still costs nothing. So this is the denominator that makes
     * slip comparable between runs. Degrees of slip per minute drops if the drivers simply aim less
     * that match; degrees of slip per thousand degrees travelled does not, which is what makes it
     * worth anything for judging whether a belt or pulley change helped.
     */
    @Getter private double travelTotalDegrees = 0;

    private double lastTravelPositionDegrees = Double.NaN;

    /** Accumulates {@link #travelTotalDegrees}. Call once per loop. */
    private void updateTravel() {
        double position = getPositionDegrees();
        if (!Double.isNaN(lastTravelPositionDegrees)) {
            travelTotalDegrees += Math.abs(position - lastTravelPositionDegrees);
        }
        lastTravelPositionDegrees = position;
    }

    // -- Reading guard ---------------------------------------------------------------------------

    /**
     * Largest believable change in the reported angle between two guarded samples with the turret
     * standing still, in degrees.
     *
     * <p>This is the fixed part of the step budget; {@link #positionStepBudgetDegrees} adds what
     * the velocity signal says the turret actually moved in the time since the last sample. Until
     * 2026-09-19 (Chezy Q50) the 15 deg stood alone and was compared loop to loop as if every loop
     * were 20 ms. It is not: Q50 ran 282 of its 7979 enabled loops over 30 ms, 34 over 45 ms and
     * one at 80 ms mid-auto, and the turret was slewing at 0.8 to 1.27 rot/s (290 to 460 deg/s), so
     * a single slow loop carried 15 to 23 deg of real motion. The guard rejected it, and from then
     * on every fresh reading was even further from the held angle, so it stayed rejected until the
     * 10-loop acceptance "re-framed" the turret by 50 to 108 deg five times in the match (220.6,
     * 220.9, 301.9, 314.1 and 346.9 s), each one a false alarm that also blanked {@code
     * TurretOnTarget} for 12 s of launch time. Q36 had the same three times at 8 V.
     *
     * <p>Fifteen still clears any residual sample jitter with margin and is more than a decade
     * below what this exists to catch: on the 2026-09-19 pit log (FRC_20260919_150646, 162.68 s)
     * the reported angle went from -0.09 to 289.42 deg between two consecutive loops with the
     * turret barely moving, and the controller drove 235 deg of real motion into a hard stop on the
     * strength of that one sample.
     */
    private static final double MAX_POSITION_STEP_DEGREES = 15.0;

    /**
     * Multiplier on the motion the velocity signal accounts for, so the budget survives the turret
     * accelerating between two samples and the position and velocity frames not landing on the same
     * edge. At 1 rot/s and a 40 ms loop the velocity term is 14.4 deg; 1.5 makes it 21.6, on top of
     * the fixed 15. A {@code setPosition} re-frame moves the position register and not the
     * velocity, so at that speed and a nominal loop it still has only 15 + 10.8 deg to hide in, and
     * the 52 deg re-home Q24 saw is caught as before.
     */
    private static final double POSITION_STEP_SLEW_MARGIN = 1.5;

    /**
     * Bounds on the elapsed time the velocity term may claim, in seconds. The floor is one nominal
     * loop so a fast double call cannot shrink the budget below the standing-still case; the
     * ceiling keeps a multi-second stall, or the first sample after enable, from opening the budget
     * to a full turn.
     */
    private static final double POSITION_STEP_MIN_DT_SECONDS = 0.02;

    private static final double POSITION_STEP_MAX_DT_SECONDS = 0.25;

    /**
     * Consecutive loops a stepped reading must repeat before it is believed.
     *
     * <p>A single bad sample is dropped outright and costs nothing. A step still there 10 loops
     * (200 ms) later is not noise: the encoder really has been re-framed, by a device reset or a
     * {@code setPosition}, and the sensor is then the only truth on offer, so it is accepted. What
     * the delay buys is that no output is ever commanded from a one-loop step, and that the
     * acceptance is loud rather than silent.
     */
    private static final int POSITION_STEP_CONFIRM_LOOPS = 10;

    /** How far outside the configured travel the reading may sit before it is called slip. */
    private static final double ENVELOPE_MARGIN_DEGREES = 5.0;

    private double heldPositionDegrees = Double.NaN;
    private double positionStepCandidateDegrees = Double.NaN;
    private int positionStepLoops = 0;
    private long positionGuardLoop = -1;
    private double positionGuardTimestamp = Double.NaN;
    private boolean positionSuspect = false;

    /** This loop's step budget, for the log: what the guard would have let through. */
    @Getter private double positionStepBudgetDegrees = MAX_POSITION_STEP_DEGREES;

    /** How many distinct impossible steps have been held out this power cycle. */
    @Getter private int positionStepsRejected = 0;

    /** How many of those went on to repeat themselves long enough to be believed. */
    @Getter private int positionStepsAccepted = 0;

    private final Alert positionStepAlert =
            new Alert(
                    "Turret angle jumped further in one loop than it could have moved ("
                            + (int) MAX_POSITION_STEP_DEGREES
                            + " deg plus the velocity signal's travel). Holding the last good angle,"
                            + " so the turret will not act on the jump. If this does not clear on"
                            + " its own the encoder"
                            + " has been re-framed: re-zero the turret (operator B, disabled)"
                            + " before trusting a shot.",
                    AlertType.kError);

    private final Alert envelopeAlert = new Alert("", AlertType.kError);

    /**
     * Validates this loop's reported angle, once per loop.
     *
     * <p>Keyed on the loop counter rather than the scheduler, the same way {@link
     * #recordAngleSample()} is, so Vision -- which runs before {@code CommandScheduler.run()} --
     * gets this loop's decision rather than the previous one's.
     */
    private void updatePositionGuard() {
        long loop = RobotLoop.count();
        if (loop == positionGuardLoop) {
            return;
        }
        positionGuardLoop = loop;

        double raw = super.getPositionDegrees();
        double now = Timer.getFPGATimestamp();
        double dt = now - positionGuardTimestamp;
        positionGuardTimestamp = now;
        if (Double.isNaN(heldPositionDegrees)) {
            heldPositionDegrees = raw;
            return;
        }

        // What the turret could honestly have moved since the last sample: the fixed allowance
        // plus the velocity signal's travel over the time that actually elapsed, so a slow loop
        // during a fast slew widens the budget instead of tripping it (Q50, see
        // MAX_POSITION_STEP_DEGREES). Velocity is in the same status frame as position, so a
        // re-framed position register does not bring a matching velocity with it.
        if (Double.isNaN(dt)) {
            dt = POSITION_STEP_MIN_DT_SECONDS;
        }
        dt = MathUtil.clamp(dt, POSITION_STEP_MIN_DT_SECONDS, POSITION_STEP_MAX_DT_SECONDS);
        double velocityDegPerSec = Math.abs(getVelocityRPM()) * 6.0;
        double budget =
                MAX_POSITION_STEP_DEGREES + POSITION_STEP_SLEW_MARGIN * velocityDegPerSec * dt;
        positionStepBudgetDegrees = budget;

        if (Math.abs(raw - heldPositionDegrees) <= budget) {
            heldPositionDegrees = raw;
            positionStepLoops = 0;
            positionSuspect = false;
            positionStepAlert.set(false);
            return;
        }

        // Too big to be motion. Count it only while the reading keeps insisting on the same new
        // value; a reading that wanders is noise starting over, not a re-framed encoder. The same
        // budget applies, so a turret that keeps slewing while held does not reset the count every
        // slow loop (the 349 ms hold at Q50 346.5 s).
        if (positionStepLoops > 0 && Math.abs(raw - positionStepCandidateDegrees) <= budget) {
            positionStepLoops++;
        } else {
            positionStepLoops = 1;
            positionStepsRejected++;
        }
        positionStepCandidateDegrees = raw;
        positionSuspect = true;
        positionStepAlert.set(true);

        if (positionStepLoops >= POSITION_STEP_CONFIRM_LOOPS) {
            Telemetry.print(
                    String.format(
                            "!!! Turret angle re-framed: %.1f deg became %.1f deg and stayed there"
                                    + " for %d loops, so it is now believed. The soft limits and"
                                    + " every aim are in that new frame. Re-zero the turret"
                                    + " (operator B, disabled) before trusting a shot.",
                            heldPositionDegrees, raw, POSITION_STEP_CONFIRM_LOOPS));
            // The reading moved without the turret moving, so it is not travel. Same reasoning as
            // applyZeroCorrectionDegrees: travel is the denominator slip is measured against, and
            // banking a jump into it makes every slip-per-degree number after it a lie.
            if (!Double.isNaN(lastTravelPositionDegrees)) {
                lastTravelPositionDegrees += raw - heldPositionDegrees;
            }
            heldPositionDegrees = raw;
            positionStepLoops = 0;
            positionSuspect = false;
            positionStepsAccepted++;
            positionStepAlert.set(false);
        }
    }

    /**
     * This loop's turret angle, with impossible one-loop steps held out.
     *
     * <p>Overrides the mechanism's raw reading so everything downstream -- the aim, the soft limit
     * arithmetic in {@link #resolveTurretAngle}, the shot gate, travel, and Vision's zero chaser --
     * sees one consistent angle rather than each making its own decision about whether to trust it.
     *
     * @return the guarded turret angle in degrees
     */
    @Override
    public double getPositionDegrees() {
        updatePositionGuard();
        return Double.isNaN(heldPositionDegrees) ? super.getPositionDegrees() : heldPositionDegrees;
    }

    /**
     * Whether the reported angle is currently being held out as an impossible step.
     *
     * @return true while the guard is holding the last good angle
     */
    public boolean isPositionSuspect() {
        updatePositionGuard();
        return positionSuspect;
    }

    /** Raises the envelope alert while the reported angle sits outside the configured travel. */
    private void updateEnvelopeAlert() {
        double minDeg = minLimitDegrees();
        double maxDeg = maxLimitDegrees();
        double position = getPositionDegrees();
        boolean outside =
                position < minDeg - ENVELOPE_MARGIN_DEGREES
                        || position > maxDeg + ENVELOPE_MARGIN_DEGREES;
        if (outside) {
            envelopeAlert.setText(
                    String.format(
                            "Turret reads %.1f deg, outside its %.0f to %.0f deg of travel. The"
                                    + " mechanism cannot be there, so the zero has slipped: the"
                                    + " soft limits are in the wrong frame and every shot leaves"
                                    + " by the same error. Re-zero (operator B, disabled).",
                            position, minDeg, maxDeg));
        }
        envelopeAlert.set(outside);
        Telemetry.log("Turret/AngleOutsideEnvelope", outside);
    }

    // -- Stall protection ------------------------------------------------------------------------

    /**
     * Fraction of {@code torqueCurrentLimit} that counts as pinned against the ceiling.
     *
     * <p>Not the limit itself: a motor held at its limit dithers either side of it. On the
     * 2026-09-19 pit log the turret read 78.5 to 81.4 A against an 80 A ceiling for the whole
     * stall, which is why a detector keyed on the exact number saw 1.8 s of a 6.8 s event.
     */
    private static final double STALL_STATOR_FRACTION = 0.95;

    /** Below this the turret is not turning. Tracking a target never reads this low for long. */
    private static final double STALL_VELOCITY_ROT_PER_SEC = 0.02;

    /** How long pinned-and-stopped must hold before the output is cut. */
    private static final double STALL_SECONDS = 1.0;

    /** How far the other way the turret must be asked to go before the latch releases. */
    private static final double STALL_RECOVERY_MARGIN_DEGREES = 2.0;

    private final Debouncer stallDebouncer = new Debouncer(STALL_SECONDS, DebounceType.kRising);
    private boolean stallLatched = false;

    /** Sign of {@code commanded - measured} when the latch closed: the way it was pushing. */
    private double stallPushSign = 0;

    /** How many times the turret has been latched out this power cycle. */
    @Getter private int stallLatchCount = 0;

    private final Alert stallAlert = new Alert("", AlertType.kError);

    /**
     * Cuts the turret's output once it has been pinned at its current ceiling and not turning for
     * {@link #STALL_SECONDS}.
     *
     * <p>On 2026-09-19 the turret sat at 80 A stator with zero velocity for 6.8 s -- about -2.1 V
     * applied, 14.3 A off the battery -- against a hard stop, and nothing in the code stopped it.
     * That is heat into the belt and the gearbox for as long as the state machine keeps asking, and
     * this belt has skipped teeth at this current before.
     */
    private void updateStallDetection() {
        if (stallLatched) {
            return;
        }
        boolean stalledNow =
                Math.abs(getStatorCurrent())
                                >= STALL_STATOR_FRACTION * config.getTorqueCurrentLimit()
                        && Math.abs(getVelocityRPM() / 60.0) < STALL_VELOCITY_ROT_PER_SEC;
        if (!stallDebouncer.calculate(stalledNow)) {
            return;
        }
        stallLatched = true;
        stallDebouncer.calculate(false);
        stallLatchCount++;
        stallPushSign = Math.signum(commandedDegrees - getPositionDegrees());
        stallAlert.setText(
                String.format(
                        "Turret STALLED at %.1f deg and its output is cut: it was pinned at the"
                                + " %.0f A ceiling, not turning, for %.1f s while being asked for"
                                + " %.1f deg. It drives again as soon as it is asked to go the"
                                + " other way. Check for a jam or a hard stop, and whether the belt"
                                + " skipped.",
                        getPositionDegrees(),
                        config.getTorqueCurrentLimit(),
                        STALL_SECONDS,
                        commandedDegrees));
        stallAlert.set(true);
        Telemetry.print(
                String.format(
                        "!!! Turret stalled at %.1f deg (asked for %.1f) and was cut after %.1f s"
                                + " at the current limit. Asking it the other way releases it.",
                        getPositionDegrees(), commandedDegrees, STALL_SECONDS));
    }

    /**
     * Whether the turret may drive this loop, releasing the latch when the request reverses.
     *
     * <p>A latched turret is not a dead one: the way out of a stop is back the way it came, so a
     * command pointing to the other side of where it sits clears the latch and drives immediately.
     * Anything still pushing into the stop gets nothing.
     *
     * @return true when the motor may be commanded this loop
     */
    private boolean outputAllowed() {
        if (!stallLatched) {
            return true;
        }
        double request = commandedDegrees - getPositionDegrees();
        if (stallPushSign != 0
                && Math.signum(request) == -stallPushSign
                && Math.abs(request) > STALL_RECOVERY_MARGIN_DEGREES) {
            clearStallLatch();
            return true;
        }
        return false;
    }

    /** Releases the stall latch and its alert. */
    private void clearStallLatch() {
        stallLatched = false;
        stallDebouncer.calculate(false);
        stallPushSign = 0;
        stallAlert.set(false);
    }

    /**
     * Holds the turret at an angle with a plain position request: no unwrap, no velocity
     * feedforward. Stops instead while the stall latch is holding the turret out.
     *
     * @param degrees the mechanism angle to hold
     */
    private void holdDegrees(double degrees) {
        unwrapping = false;
        mechOmegaRotPerSec = 0;
        commandedDegrees = degrees;
        if (!outputAllowed()) {
            stop();
            return;
        }
        setPosition(() -> degreesToRotations(() -> degrees));
    }

    /** Applies the aim at target. */
    private void applyAimAtTarget(double offsetDeg) {
        var params = ShotCalculator.getInstance().getParameters();

        // Convert FIELD-RELATIVE angle to MECHANISM-RELATIVE angle
        double robotHeadingDeg = Robot.getSwerve().getRobotPose().getRotation().getDegrees();
        double desiredMechDegrees =
                params.turretAngle().getDegrees()
                        - robotHeadingDeg
                        - config.getZeroOffsetFromRobotFront().getDegrees()
                        + offsetDeg;

        double commanded = resolveTurretAngle(desiredMechDegrees);
        commandedDegrees = commanded;

        // Counter-rotate for the robot's own spin, so mechOmega = fieldOmega - robotOmega
        ChassisSpeeds robotSpeeds = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        double robotOmegaRotPerSec = robotSpeeds.omegaRadiansPerSecond / (2.0 * Math.PI);
        mechOmegaRotPerSec = params.turretAngularVelocity() - robotOmegaRotPerSec;

        if (!outputAllowed()) {
            stop();
            return;
        }

        if (unwrapping) {
            // Motion magic for smooth full-turn slew to the opposite winding, so the cable never
            // binds
            final double unwrapRot = degreesToRotations(() -> commandedDegrees);
            setMMPosition(() -> unwrapRot);
            return;
        }

        // Lead the moving target by the actuation latency
        double minDeg = minLimitDegrees();
        double maxDeg = maxLimitDegrees();
        double predictedDegrees =
                MathUtil.clamp(
                        commanded
                                + (mechOmegaRotPerSec * 360.0) * config.getShootOnMoveLatencySec(),
                        minDeg,
                        maxDeg);

        final double posRot = degreesToRotations(() -> predictedDegrees);
        final double ffRps = mechOmegaRotPerSec;
        setPositionWithVelocity(() -> posRot, () -> ffRps);
    }

    /**
     * Picks the physically-equivalent turret angle (target direction ± whole turns) that best fits
     * the limited travel range, and drives the proactive cable-unwrap hysteresis.
     */
    private double resolveTurretAngle(double desiredMechDegrees) {
        double minDeg = minLimitDegrees();
        double maxDeg = maxLimitDegrees();
        double currentDeg = getPositionDegrees();

        int nMin = (int) Math.ceil((minDeg - desiredMechDegrees) / 360.0);
        int nMax = (int) Math.floor((maxDeg - desiredMechDegrees) / 360.0);

        if (nMin > nMax) {
            unwrapping = false;
            return (Math.abs(currentDeg - minDeg) < Math.abs(currentDeg - maxDeg))
                    ? minDeg
                    : maxDeg;
        }

        int nClosest = (int) Math.round((currentDeg - desiredMechDegrees) / 360.0);
        int n = Math.max(nMin, Math.min(nClosest, nMax));
        double chosen = desiredMechDegrees + n * 360.0;

        // While unwrapping, hold the committed winding until we physically arrive, so the direction
        // can't flip mid-slew as the current position crosses the halfway point.
        if (unwrapping) {
            int nTarget = Math.max(nMin, Math.min(unwrapTargetN, nMax));
            chosen = desiredMechDegrees + nTarget * 360.0;
            if (nMin == nMax || Math.abs(currentDeg - chosen) <= config.getUnwrapExitMargin()) {
                unwrapping = false;
            }
            return chosen;
        }

        // Proactive unwrap: trigger only if the nearest command is crowding a soft limit and the
        // opposite winding is reachable, then commit to that winding. Unwinding by a single turn
        // clears the limit; slewing all the way to nMin/nMax would land on the opposite limit and
        // immediately re-trigger an unwrap back the other way.
        if (nMin != nMax) {
            if (maxDeg - chosen <= config.getUnwrapTolerance() && (n - 1) >= nMin) {
                unwrapping = true;
                unwrapTargetN = n - 1;
                chosen = desiredMechDegrees + unwrapTargetN * 360.0;
            } else if (chosen - minDeg <= config.getUnwrapTolerance() && (n + 1) <= nMax) {
                unwrapping = true;
                unwrapTargetN = n + 1;
                chosen = desiredMechDegrees + unwrapTargetN * 360.0;
            }
        }
        return chosen;
    }

    /**
     * @return true when the turret is aiming, within tolerance of its commanded angle, and not
     *     mid-unwrap. Tracking error is the criterion, not slew rate: while shooting on the move
     *     the turret is legitimately moving, so a velocity clause would only block good shots.
     *     Gates feeding into the flywheel.
     */
    public boolean isReadyToShoot() {
        return isReadyToShoot(config.getTriggerTolerance());
    }

    /**
     * Same check as {@link #isReadyToShoot()} against a caller-supplied tolerance. The feeder gate
     * uses a wider tolerance to decide whether to <em>keep</em> feeding than to start, so normal
     * tracking error mid-burst does not chop the feed on and off.
     *
     * <p>The {@code unwrapping} clause is not relaxed at any tolerance: during an unwrap the turret
     * slews a full turn and fed fuel goes anywhere.
     *
     * @param toleranceDegrees allowed tracking error in degrees
     * @return true when aiming, not mid-unwrap, and within {@code toleranceDegrees}
     */
    public boolean isReadyToShoot(double toleranceDegrees) {
        return (systemState == SystemState.AIM_AT_TARGET || systemState == SystemState.FIXED_ANGLE)
                && !unwrapping
                && Math.abs(getPositionDegrees() - commandedDegrees) <= toleranceDegrees;
    }

    /**
     * Returns the current turret tracking error in degrees, for logging and for setting the gate
     * tolerances from a log.
     *
     * @return commanded minus measured turret angle, in degrees
     */
    public double getTrackingErrorDegrees() {
        return getPositionDegrees() - commandedDegrees;
    }

    /**
     * Creates a command that drops the turret into coast so it can be moved by hand. Runs while
     * disabled, which is the only time it is useful.
     *
     * @return the coast-mode command
     */
    public Command coastModeCommand() {
        return new InstantCommand(() -> setBrakeMode(false))
                .ignoringDisable(true)
                .withName("Turret.coastMode");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    /** Simulation init. */
    private void simulationInit() {
        if (isAttached()) {
            sim = new TurretSim(RobotSim.topView, motor);
        }
    }

    class TurretSim extends ArmSim {
        /**
         * Creates a new TurretSim instance.
         *
         * @param mech the mech
         * @param motor the motor
         */
        public TurretSim(Mechanism2d mech, TalonFX motor) {
            super(
                    new ArmConfig(
                                    config.turretX,
                                    config.turretY,
                                    config.sensorToMechanismRatio,
                                    config.length,
                                    -360,
                                    360,
                                    0)
                            .setSimulatedGravity(false),
                    mech,
                    motor,
                    config.getName());
        }
    }
}
