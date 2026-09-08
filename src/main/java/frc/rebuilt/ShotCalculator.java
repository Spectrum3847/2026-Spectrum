package frc.rebuilt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.rebuilt.targetFactories.HubTargetFactory;
import frc.robot.Robot;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;

@SuppressWarnings("unused")
public class ShotCalculator {

    // =========================================================================
    // Singleton
    // =========================================================================

    private static ShotCalculator instance;

    /** Robot-centre to launcher offset. Zero = launcher is at robot centre. */
    private static final Transform2d robotToLauncher = Transform2d.kZero;
    /**
     * Returns the instance.
     *
     * @return the instance
     */
    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    // =========================================================================
    // Shot Parameters Record
    // =========================================================================

    /**
     * Immutable snapshot of all quantities needed to command the turret, hood, and flywheel
     * subsystems for a single shot.
     */
    public record ShootingParameters(
            /** {@code true} when distance is within the polynomial's fitted range. */
            boolean isValid,
            /** Field-relative heading the robot must face to aim at the goal. */
            Rotation2d turretAngle,
            /** Rate of change of {@code turretAngle} (rotations/s) for heading feedforward. */
            double turretAngularVelocity,
            /** Commanded hood/pivot angle (degrees), including {@link #HOOD_ANGLE_OFFSET}. */
            double hoodAngle,
            /** Rate of change of {@code hoodAngle} (deg/s) for pivot feedforward. */
            double hoodVelocity,
            /** Commanded flywheel speed (RPM). */
            double flywheelSpeed,
            /** Ball exit speed from the polynomial (m/s), before RPM conversion. */
            double exitSpeedMs,
            /** Shoot-on-move compensated distance to goal (metres). */
            double distance,
            /** Raw uncompensated distance to goal (metres). */
            double distanceNoLookahead,
            /** Estimated ball time-of-flight (seconds). */
            double timeOfFlight) {}

    private ShootingParameters latestParameters = null;

    // =========================================================================
    // Runtime-Adjustable Offsets
    // =========================================================================

    public static final double STARTING_HOOD_ANGLE_OFFSET = 0; // degrees
    public static double HOOD_ANGLE_OFFSET = STARTING_HOOD_ANGLE_OFFSET;

    /**
     * Degrees per operator D-pad press.
     *
     * <p>Was 0.1, which is a fortieth of the correction the hub model actually needed -- forty
     * presses to move the shot the distance one afternoon of testing said it was out. A quarter
     * degree is roughly a quarter of a foot of range near where this robot shoots, which is finer
     * than anyone can judge from watching a ball land.
     */
    public static final double HOOD_OFFSET_STEP_DEG = 0.25;

    /** Degrees per operator D-pad press on the turret trim. */
    public static final double TURRET_OFFSET_STEP_DEG = 1.0;

    public static final double STARTING_TURRET_ANGLE_OFFSET = 0; // degrees
    public static double TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;

    /**
     * Largest trim either axis will hold, degrees either side of zero.
     *
     * <p>The hood model moves the shot about a foot per degree near where this robot shoots, so ten
     * degrees is ten feet of range, far past any correction a real fit needs. The cap is not there
     * to stop the operator, who cannot press the D-pad forty times by accident; it is there because
     * the trims now come back off the rio's flash at boot, and a corrupt or hand-edited preference
     * should not be able to command the turret twenty degrees off target before anyone notices. The
     * hood is clamped again downstream against its soft limits; the turret trim is not, which is
     * the axis this actually protects.
     */
    public static final double MAX_TRIM_DEG = 10.0;

    /**
     * Preferences key the hood trim persists under. Flat name, no slash: {@link Preferences} keeps
     * everything in one NetworkTables table, and a slash would nest a sub-table its own {@code
     * getKeys()} does not walk.
     *
     * <p>Public because it is the name of a value that outlives the code that wrote it. Anything
     * reading a trim off a rio -- a test, the robot app, someone in the pit with Elastic's
     * Preferences widget -- needs the same string this class uses, not a copy of it.
     */
    public static final String HOOD_TRIM_PREF_KEY = "ShotHoodTrimDeg";

    /** Preferences key the turret trim persists under. See {@link #HOOD_TRIM_PREF_KEY}. */
    public static final String TURRET_TRIM_PREF_KEY = "ShotTurretTrimDeg";

    /**
     * Reads both trims back off the rio.
     *
     * <p>Call once during robot construction, before any binding can move a trim. Until 2026-09-08
     * these were plain static fields, so every redeploy zeroed whatever the operator had dialled in
     * and the only way to keep a correction was for someone to remember the number and fold it into
     * the model's {@code hoodOffsetDeg} by hand. That is exactly what the -4 and then -5 degree hub
     * trims in the git log are.
     *
     * <p>{@link Preferences} lives in flash, so a trim now survives a power cycle as well as a
     * redeploy. That is the point and it is also the risk: a trim dialled in against a shop ceiling
     * last week silently applies at the next event. The mitigation is this method's boot print and
     * the operator's Start+Select reset, not an expiry. An expiry would zero the trim in the middle
     * of a session the operator thought was still calibrated, which is the worse failure.
     */
    public static void loadPersistedTrims() {
        Preferences.initDouble(HOOD_TRIM_PREF_KEY, STARTING_HOOD_ANGLE_OFFSET);
        Preferences.initDouble(TURRET_TRIM_PREF_KEY, STARTING_TURRET_ANGLE_OFFSET);

        double storedHood = Preferences.getDouble(HOOD_TRIM_PREF_KEY, STARTING_HOOD_ANGLE_OFFSET);
        double storedTurret =
                Preferences.getDouble(TURRET_TRIM_PREF_KEY, STARTING_TURRET_ANGLE_OFFSET);

        HOOD_ANGLE_OFFSET = MathUtil.clamp(storedHood, -MAX_TRIM_DEG, MAX_TRIM_DEG);
        TURRET_ANGLE_OFFSET = MathUtil.clamp(storedTurret, -MAX_TRIM_DEG, MAX_TRIM_DEG);

        if (HOOD_ANGLE_OFFSET != storedHood || TURRET_ANGLE_OFFSET != storedTurret) {
            Telemetry.print(
                    String.format(
                            "!!! Stored shot trims were out of range (hood %.2f, turret %.2f) and"
                                    + " were clamped to +/- %.1f deg. Something other than the"
                                    + " operator D-pad wrote them.",
                            storedHood, storedTurret, MAX_TRIM_DEG),
                    PrintPriority.HIGH);
            writeTrimPreferences();
        }

        if (HOOD_ANGLE_OFFSET != 0 || TURRET_ANGLE_OFFSET != 0) {
            Telemetry.print(
                    String.format(
                            "!!! Persisted shot trims are in effect: hood %+.2f deg, turret %+.2f"
                                    + " deg. These came off the rio, not from this session."
                                    + " Operator Start+Select zeroes both.",
                            HOOD_ANGLE_OFFSET, TURRET_ANGLE_OFFSET),
                    PrintPriority.HIGH);
        } else {
            Telemetry.print("Shot trims loaded from the rio: both zero.", PrintPriority.HIGH);
        }
    }

    private static void writeTrimPreferences() {
        Preferences.setDouble(HOOD_TRIM_PREF_KEY, HOOD_ANGLE_OFFSET);
        Preferences.setDouble(TURRET_TRIM_PREF_KEY, TURRET_ANGLE_OFFSET);
    }

    /**
     * Applies a trim nudge, persists it, and logs it as a shot outcome.
     *
     * <p>Every press is a judgement about the last burst: hood down means it went long, hood up
     * means it fell short, and the turret pair say which side it missed on. That makes the D-pad
     * the outcome signal, so there are no separate short/made/long buttons; a made shot is the
     * absence of a press. See {@code docs/tools/shot-log.md} for how the two record streams pair
     * up.
     *
     * @param hood true for the hood axis, false for the turret axis
     * @param deltaDeg signed nudge in degrees, before clamping
     */
    private static void nudgeTrim(boolean hood, double deltaDeg) {
        double before = hood ? HOOD_ANGLE_OFFSET : TURRET_ANGLE_OFFSET;
        double after = MathUtil.clamp(before + deltaDeg, -MAX_TRIM_DEG, MAX_TRIM_DEG);
        if (hood) {
            HOOD_ANGLE_OFFSET = after;
        } else {
            TURRET_ANGLE_OFFSET = after;
        }
        // Preferences writes go to flash and through NetworkTables. Safe here, in a command that
        // runs once per press; never do this from a periodic.
        writeTrimPreferences();
        logTrimEvent(hood, after - before, after, false);
    }

    /** Increase hood angle offset. The operator's way of saying the last shot fell short. */
    public static Command increaseHoodAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(true, HOOD_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.increaseHoodTrim");
    }

    /** Decrease hood angle offset. The operator's way of saying the last shot went long. */
    public static Command decreaseHoodAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(true, -HOOD_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.decreaseHoodTrim");
    }

    /** Increase turret angle offset. */
    public static Command increaseTurretAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(false, TURRET_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.increaseTurretTrim");
    }

    /** Decrease turret angle offset. */
    public static Command decreaseTurretAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(false, -TURRET_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.decreaseTurretTrim");
    }

    /**
     * Zeroes both trims and clears them from flash.
     *
     * <p>Bound to a two-button chord because it has to be reachable in the pit without being
     * reachable by accident. A persisted trim nobody can clear from the driver station is worse
     * than one that evaporates.
     *
     * @return the reset command
     */
    public static Command resetTrimsCommand() {
        return Commands.runOnce(
                        () -> {
                            double hoodBefore = HOOD_ANGLE_OFFSET;
                            double turretBefore = TURRET_ANGLE_OFFSET;
                            HOOD_ANGLE_OFFSET = STARTING_HOOD_ANGLE_OFFSET;
                            TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;
                            writeTrimPreferences();
                            logTrimEvent(
                                    true, HOOD_ANGLE_OFFSET - hoodBefore, HOOD_ANGLE_OFFSET, true);
                            logTrimEvent(
                                    false,
                                    TURRET_ANGLE_OFFSET - turretBefore,
                                    TURRET_ANGLE_OFFSET,
                                    true);
                            Telemetry.print(
                                    String.format(
                                            "Shot trims reset to zero (were hood %+.2f, turret"
                                                    + " %+.2f).",
                                            hoodBefore, turretBefore),
                                    PrintPriority.HIGH);
                        })
                .ignoringDisable(true)
                .withName("ShotCalculator.resetTrims");
    }

    // =========================================================================
    // Shot Records
    // =========================================================================
    //
    // Two sparse streams, one row per event, both wpilog-only:
    //
    //   ShotCalc/Shot/*  one row when the feed gate opens, saying what was aimed
    //   ShotCalc/Trim/*  one row per operator D-pad press, saying how it went
    //
    // Neither is a loop-rate stream. Balls per burst are counted afterwards from the dips in
    // Launcher/RPM, which is kept at loop rate for exactly that (Launcher.java 203).
    //
    // DogLog skips a record whose value has not changed, so a burst at the same distance with the
    // same model writes Index and TimestampSeconds and little else. Read a row by taking each
    // key's last value at or before that row's timestamp; see docs/tools/shot-log.md.

    /** Bursts since boot. The pairing key between a shot row and the trim row that judges it. */
    private static long shotIndex = 0;

    /** FPGA time of the last burst, or NaN before the first one. */
    private static double lastShotTimestampSeconds = Double.NaN;

    /** Distance of the last burst, so a trim row carries the range it is judging. */
    private static double lastShotDistanceMeters = Double.NaN;

    /** D-pad presses since boot. */
    private static long trimEventIndex = 0;

    // Snapshot of the per-loop values a shot row needs that ShootingParameters does not carry.
    // Written on every getParameters() computation, read on the loop the gate opens.
    private static String activeModelName = "none";
    private static double activeModelHoodOffsetDeg = 0;
    private static double activeRadialVelocityMs = 0;
    private static double activeTangentialVelocityMs = 0;
    private static boolean activeFeedShot = false;

    /**
     * Writes one row describing the burst that is starting.
     *
     * <p>Called on the rising edge of the feed gate, which is the first loop fuel is allowed into
     * the flywheel and so the last loop on which the aim was still a prediction. Everything here is
     * either what the model asked for or what the mechanism actually did, on that loop.
     *
     * <p>Two omissions are deliberate. There is no outcome field, because the outcome arrives later
     * as a trim press. And the vision turret-zero split, {@code
     * Vision/TurretZero/PoseHeadingErrorDeg} and {@code TurretOnlyErrorDeg}, is not copied in: it
     * is already logged at 10 Hz and joins on time, and duplicating it here would let the two drift
     * apart.
     *
     * @param poseTrusted whether vision had accepted an estimate recently enough to believe the
     *     distance, as computed by the feed gate
     */
    public static void recordShot(boolean poseTrusted) {
        ShootingParameters params = getInstance().getParameters();
        double now = Timer.getFPGATimestamp();

        shotIndex++;
        lastShotTimestampSeconds = now;
        lastShotDistanceMeters = params.distanceNoLookahead();

        // The one key on NetworkTables: the operator needs to see bursts counting up to know
        // records are being written at all. One publish per burst is nothing next to the loop-rate
        // traffic the 09-05 tiers exist to control.
        Telemetry.logDashAlways("ShotCalc/Shot/Index", shotIndex);

        Telemetry.log("ShotCalc/Shot/TimestampSeconds", now, "seconds");
        Telemetry.log("ShotCalc/Shot/MatchTimeSeconds", DriverStation.getMatchTime(), "seconds");
        Telemetry.log("ShotCalc/Shot/DistanceMeters", params.distanceNoLookahead(), "meters");
        Telemetry.log("ShotCalc/Shot/LookaheadDistanceMeters", params.distance(), "meters");
        Telemetry.log("ShotCalc/Shot/WantedRPM", params.flywheelSpeed(), "RPM");
        Telemetry.log("ShotCalc/Shot/WantedHoodDeg", params.hoodAngle(), "degrees");
        Telemetry.log("ShotCalc/Shot/ExitSpeedMs", params.exitSpeedMs(), "m/s");
        Telemetry.log("ShotCalc/Shot/TimeOfFlightSeconds", params.timeOfFlight(), "seconds");
        Telemetry.log("ShotCalc/Shot/RadialVelocityMs", activeRadialVelocityMs, "m/s");
        Telemetry.log("ShotCalc/Shot/TangentialVelocityMs", activeTangentialVelocityMs, "m/s");
        Telemetry.log("ShotCalc/Shot/Model", activeModelName);
        Telemetry.log("ShotCalc/Shot/HoodModelOffsetDeg", activeModelHoodOffsetDeg, "degrees");
        Telemetry.log("ShotCalc/Shot/HoodTrimDeg", HOOD_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotCalc/Shot/TurretTrimDeg", TURRET_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotCalc/Shot/FeedShot", activeFeedShot);
        Telemetry.log("ShotCalc/Shot/InRange", params.isValid());
        Telemetry.log("ShotCalc/Shot/PoseTrusted", poseTrusted);

        // Actuals. Null-guarded because a sim or a bench run can call this before every mechanism
        // exists, and a missing number should read as NaN rather than crash the loop.
        Telemetry.log(
                "ShotCalc/Shot/ActualRPM",
                Robot.getLauncher() == null ? Double.NaN : Robot.getLauncher().getVelocityRPM(),
                "RPM");
        Telemetry.log(
                "ShotCalc/Shot/ActualHoodDeg",
                Robot.getHood() == null ? Double.NaN : Robot.getHood().getPositionDegrees(),
                "degrees");
        // Measured minus commanded, matching Turret/TrackingErrorDegrees. Note that
        // Turret/PositionError is logged with the OPPOSITE sign (Turret.java 309); this key follows
        // getTrackingErrorDegrees(). Which way in the world a positive value points is not
        // documented anywhere on the turret, so read it as a magnitude unless you have checked.
        Telemetry.log(
                "ShotCalc/Shot/TurretErrorDeg",
                Robot.getTurret() == null
                        ? Double.NaN
                        : Robot.getTurret().getTrackingErrorDegrees(),
                "degrees");
        if (Robot.getSwerve() != null) {
            Telemetry.log("ShotCalc/Shot/Pose", Robot.getSwerve().getRobotPose());
        }
    }

    /**
     * Writes one row for an operator trim press, and pairs it with the burst it is judging.
     *
     * <p>{@code ShotIndex} and {@code SecondsSinceShot} are the pairing. A press seconds after a
     * burst is a verdict on that burst; a press in the pit with no burst behind it carries index -1
     * and an infinite age, and analysis drops it. Deciding what counts as "seconds after" is the
     * reader's job, not this method's, so the age is logged rather than thresholded here.
     *
     * @param hood true for the hood axis, false for the turret axis
     * @param deltaDeg how far the trim actually moved, after clamping; zero at the limit
     * @param valueDeg the trim's new value
     * @param reset true when this row is the Start+Select reset rather than a judgement
     */
    private static void logTrimEvent(
            boolean hood, double deltaDeg, double valueDeg, boolean reset) {
        double now = Timer.getFPGATimestamp();
        double secondsSinceShot =
                Double.isNaN(lastShotTimestampSeconds)
                        ? Double.POSITIVE_INFINITY
                        : now - lastShotTimestampSeconds;

        String verdict;
        if (reset) {
            verdict = "Reset";
        } else if (deltaDeg == 0) {
            // The trim was already at MAX_TRIM_DEG. The press is still a verdict about the shot,
            // and losing it would bias the dataset towards whichever direction had room left.
            verdict = "AtLimit";
        } else if (hood) {
            // Hood up means the operator is adding range, so the ball fell short.
            verdict = deltaDeg > 0 ? "Short" : "Long";
        } else {
            // A CCW trim correction means the ball landed CW of the target.
            verdict = deltaDeg > 0 ? "MissedCW" : "MissedCCW";
        }

        trimEventIndex++;
        Telemetry.log("ShotCalc/Trim/Index", trimEventIndex);
        Telemetry.log("ShotCalc/Trim/TimestampSeconds", now, "seconds");
        Telemetry.log("ShotCalc/Trim/Axis", hood ? "Hood" : "Turret");
        Telemetry.log("ShotCalc/Trim/DeltaDeg", deltaDeg, "degrees");
        Telemetry.log("ShotCalc/Trim/ValueDeg", valueDeg, "degrees");
        Telemetry.log("ShotCalc/Trim/Verdict", verdict);
        Telemetry.log(
                "ShotCalc/Trim/ShotIndex", Double.isNaN(lastShotTimestampSeconds) ? -1 : shotIndex);
        Telemetry.log("ShotCalc/Trim/SecondsSinceShot", secondsSinceShot, "seconds");
        Telemetry.log("ShotCalc/Trim/ShotDistanceMeters", lastShotDistanceMeters, "meters");
    }

    // =========================================================================
    // Polynomial Model
    // =========================================================================
    // 2D degree-3 polynomial surface:
    //   f(distance_m, radialVel_ms) → { exitSpeed_ms, launchAngle_deg }
    // Monomial basis: 1, d, v, d², d·v, v², d³, d²·v, d·v², v³

    /**
     * Global exit-speed scale factor. Adjust post-characterization to correct for ball compression,
     * wear, or temperature without re-fitting the polynomial. 1.0 = no scaling. Applied to both the
     * hub and feed models.
     */
    private static final double MPS_FACTOR = 1;

    /** Scale factor converting polynomial exit speed (m/s) to flywheel RPM. */
    private static final double RPM_PER_MPS = 365.0;

    /**
     * Per-second rate at which drag bleeds off the chassis velocity the ball inherits. Drives
     * {@link #driftEfficiency(double)}; raise it if shoot-on-move still over-counter-aims, lower it
     * if it under-corrects. Applies on the real robot too, since the drag is real.
     */
    private static final double LEAD_DRAG_BLEED_PER_SEC = 0.085;

    /**
     * A fitted degree-3 polynomial surface plus its input domain and normalisation. Inputs are
     * mapped to zero-mean unit-variance before evaluation, so the coefficients live in normalised
     * space and must not be applied to raw (metres / m/s) inputs directly.
     *
     * @param name descriptive name for telemetry
     * @param distMin fitted distance lower bound (metres); inputs clamped, shots outside flagged
     *     invalid
     * @param distMax fitted distance upper bound (metres)
     * @param rvMin fitted radial-velocity lower bound (m/s)
     * @param rvMax fitted radial-velocity upper bound (m/s)
     * @param dMean distance normalisation mean
     * @param dStd distance normalisation standard deviation
     * @param vMean radial-velocity normalisation mean
     * @param vStd radial-velocity normalisation standard deviation
     * @param speedCoeffs exit-speed coefficients in the monomial basis 1, d, v, d², d·v, v², d³,
     *     d²·v, d·v², v³
     * @param angleCoeffs launch-angle coefficients in the same basis
     * @param tofCoeffs time-of-flight coefficients (seconds) in the same basis; read this instead
     *     of simulating or estimating flight time when solving the virtual target. Null when the
     *     model was fitted without a flight-time output, in which case the solver falls back to a
     *     drag-free kinematic estimate.
     * @param hoodOffsetDeg calibration trim added to this model's hood angle, in degrees. Each fit
     *     is wrong in its own way, so the correction belongs to the model rather than to the robot:
     *     a trim that fixes the full-field fit has no business moving the ceiling fit, which
     *     already scores. The operator's D-pad trim is added on top of this and applies to whatever
     *     model is live.
     */
    private record PolyModel(
            String name,
            double distMin,
            double distMax,
            double rvMin,
            double rvMax,
            double dMean,
            double dStd,
            double vMean,
            double vStd,
            double[] speedCoeffs,
            double[] angleCoeffs,
            double[] tofCoeffs,
            double hoodOffsetDeg) {}

    /** Hub-shot model — used when the robot is in a scoring zone. */
    private static final PolyModel HUB_MODEL =
            new PolyModel(
                    "No Ceiling Hub Model",
                    1.5, // distMin (m)
                    8.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    4.8380417957, // dMean
                    1.9319000086, // dStd
                    -0.0808823529, // vMean
                    1.9705745171, // vStd
                    new double[] {
                        /* 1    */ 9.4913945634e+0,
                        /* d    */ 1.6537445477e+0,
                        /* v    */ -1.7241818573e+0,
                        /* d²   */ 1.3831911088e-1,
                        /* d·v  */ -3.8132583276e-2,
                        /* v²   */ -3.6027845385e-2,
                        /* d³   */ -9.4815161267e-2,
                        /* d²·v */ -9.8905876874e-2,
                        /* d·v² */ 8.4171094158e-2,
                        /* v³   */ 1.5667670376e-1
                    },
                    new double[] {
                        /* 1    */ 6.7148106203e+1,
                        /* d    */ -1.8755189845e+0,
                        /* v    */ 5.5988307381e+0,
                        /* d²   */ 1.5170142533e+0,
                        /* d·v  */ -7.0971996428e-1,
                        /* v²   */ -6.1935424997e-1,
                        /* d³   */ -7.1916259693e-1,
                        /* d²·v */ -3.6499963826e-1,
                        /* d·v² */ 1.0534259064e+0,
                        /* v³   */ 4.8011508185e-1
                    },
                    new double[] {
                        /* 1    */ 1.5339171616e+0,
                        /* d    */ 2.8197518906e-1,
                        /* v    */ -2.4847729715e-1,
                        /* d²   */ 2.4918872691e-2,
                        /* d·v  */ 2.6962100375e-2,
                        /* v²   */ -4.6144611911e-2,
                        /* d³   */ -2.2832527755e-2,
                        /* d²·v */ -3.5839964885e-2,
                        /* d·v² */ 3.5468556379e-2,
                        /* v³   */ 3.7823545109e-2
                    },
                    // Shots landed 3 to 4 feet past the hub centre on 2026-09-05. The model moves
                    // the hood about one degree per foot of range near where this robot shoots, so
                    // four degrees down. Still long on the 20:00 run that evening, so one more.
                    -5.0);

    /** 3 meter ceiling hub model - used when the robot is testing at home */
    private static final PolyModel CEILING_3M_HUB_MODEL =
            new PolyModel(
                    "3 Meter Ceiling Hub Model",
                    1.5, // distMin (m)
                    8.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    4.8380417957, // dMean
                    1.9319000086, // dStd
                    -0.0808823529, // vMean
                    1.9705745171, // vStd
                    new double[] {
                        /* 1    */ 8.7963133789e+0,
                        /* d    */ 1.1951603207e+0,
                        /* v    */ -1.1069879388e+0,
                        /* d²   */ -8.7791981659e-2,
                        /* d·v  */ -3.7029252318e-2,
                        /* v²   */ 3.1929955472e-2,
                        /* d³   */ -3.2965513546e-2,
                        /* d²·v */ -3.0624443563e-2,
                        /* d·v² */ 6.9040866571e-2,
                        /* v³   */ 2.1341472621e-2
                    },
                    new double[] {
                        /* 1    */ 6.0223997922e+1,
                        /* d    */ -8.3303295646e+0,
                        /* v    */ 1.0091287979e+1,
                        /* d²   */ -3.6385265944e-1,
                        /* d·v  */ -1.1510410447e+0,
                        /* v²   */ 9.2100949639e-2,
                        /* d³   */ -1.0315777427e-1,
                        /* d²·v */ -1.0030994978e+0,
                        /* d·v² */ 1.6204499882e+0,
                        /* v³   */ -1.3083914733e-1
                    },
                    new double[] {
                        /* 1    */ 1.2901109428e+0,
                        /* d    */ 7.3269523909e-2,
                        /* v    */ -4.3816902018e-2,
                        /* d²   */ -5.2525932813e-2,
                        /* d·v  */ 3.7797567390e-2,
                        /* v²   */ -2.8695676372e-2,
                        /* d³   */ -5.4873215203e-4,
                        /* d²·v */ -3.0552109674e-2,
                        /* d·v² */ 4.6814046679e-2,
                        /* v³   */ 3.1845113863e-3
                    },
                    // This fit scores as it is.
                    0.0);

    /** Feed-shot model — floor target, optimised for maximum robustness. */
    private static final PolyModel FEED_MODEL =
            new PolyModel(
                    "Feed Shot Model",
                    5.0, // distMin (m)
                    10.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    7.5000000000, // dMean
                    1.5430334996, // dStd
                    0.0000000000, // vMean
                    2.0000000000, // vStd
                    new double[] {
                        /* 1    */ 8.9574914779e+0,
                        /* d    */ 1.2320032575e+0,
                        /* v    */ -1.3282274119e+0,
                        /* d²   */ -9.0580610943e-2,
                        /* d·v  */ -4.1967119907e-2,
                        /* v²   */ 1.9536019536e-1,
                        /* d³   */ -8.4567038602e-2,
                        /* d²·v */ -3.7665953956e-2,
                        /* d·v² */ 4.0977995869e-2,
                        /* v³   */ -7.9772079772e-2
                    },
                    new double[] {
                        /* 1    */ 4.4856254322e+1,
                        /* d    */ 1.6637913478e+0,
                        /* v    */ 3.7355931469e+0,
                        /* d²   */ -5.7584406544e-1,
                        /* d·v  */ -3.3044655869e-1,
                        /* v²   */ 1.4309743590e+0,
                        /* d³   */ -1.0899200445e+0,
                        /* d²·v */ -3.1788374521e-1,
                        /* d·v² */ 3.5247632927e-1,
                        /* v³   */ -7.6581196581e-1
                    },
                    new double[] {
                        /* 1    */ 1.3453977447e+0,
                        /* d    */ 1.7869716357e-1,
                        /* v    */ -1.0562802078e-1,
                        /* d²   */ -2.3670760612e-2,
                        /* d·v  */ -7.3999477975e-3,
                        /* v²   */ 4.6025396825e-2,
                        /* d³   */ -2.6904794466e-2,
                        /* d²·v */ -9.7571644042e-3,
                        /* d·v² */ 1.2178208201e-2,
                        /* v³   */ -2.3703703704e-2
                    },
                    // Never calibrated; feed shots have not been characterised.
                    0.0);

    /**
     * Active hub model. The name is logged to {@code ShotCalc/HubPolyModel} — check it before a
     * match, because the two fits do not shoot the same and nothing else makes the difference
     * obvious.
     *
     * <p>Back on the full-field fit to test the -4 deg hood trim it now carries. It shot 3 to 4
     * feet past the hub centre without it, which its own table says is about four degrees of hood.
     * If that trim does not close the gap, {@link #CEILING_3M_HUB_MODEL} is the known-good fallback
     * — it scores, at the cost of a trajectory shaped to stay under a 3 m roof.
     */
    private static final PolyModel WANTED_HUB_MODEL = HUB_MODEL;

    // =========================================================================
    // State — Velocity Derivative Filters
    // =========================================================================

    private static final double LOOP_PERIOD_SECS = 0.02;

    /**
     * Phase delay applied to the estimated robot pose before computing shot parameters,
     * compensating for sensor and network latency (seconds).
     */
    private static final double PHASE_DELAY_SECS = 0.03;

    private final LinearFilter hoodAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100 ms window

    private final LinearFilter turretAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100 ms window

    private double lastHoodAngle = Double.NaN;
    private Rotation2d lastTurretAngle = null;
    // =========================================================================
    // Main API
    // =========================================================================

    /**
     * Returns the current shooting parameters, computing them from the robot's live pose and
     * velocity if not already cached this loop.
     *
     * <p>Approach:
     *
     * <ol>
     *   <li>Apply a phase delay to the odometry pose to account for sensor latency.
     *   <li>Compute the launcher's field-relative velocity, including the tangential component from
     *       robot rotation about its centre.
     *   <li>Decompose that velocity into radial (toward target) and tangential (perpendicular)
     *       components.
     *   <li>Run the 1690 Orbit iterative virtual-target solver to determine the optimal exit speed,
     *       launch angle, and yaw correction for shoot-on-the-move.
     *   <li>Derive the turret angle, hood angle, and flywheel RPM from the result.
     * </ol>
     *
     * <p>Call {@link #clearShootingParameters()} at the start of each loop to allow re-computation
     * on the next call.
     *
     * @return the latest {@link ShootingParameters}
     */
    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        // ── Target selection ─────────────────────────────────────────────────
        boolean feed = Robot.getSuperStructure().isRobotInFeedZone();
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();
        // Feed and hub shots use separately-fitted polynomial surfaces.
        PolyModel model = feed ? FEED_MODEL : WANTED_HUB_MODEL;

        // ── Phase-delayed pose estimate ──────────────────────────────────────
        Pose2d estimatedPose = Robot.getSwerve().getRobotPose();
        ChassisSpeeds robotRelativeVelocity = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        estimatedPose =
                estimatedPose.exp(
                        new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY_SECS,
                                robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY_SECS,
                                robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY_SECS));

        // ── Launcher pose + static distance ──────────────────────────────────
        Pose2d launcherPose = estimatedPose.transformBy(robotToLauncher);
        Translation2d launcherToTarget = target.minus(launcherPose.getTranslation());
        double distanceNoLookahead = launcherToTarget.getNorm();

        // ── Field-relative launcher velocity (includes rotation arm) ─────────
        ChassisSpeeds fieldVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        robotRelativeVelocity, estimatedPose.getRotation());
        double robotAngle = estimatedPose.getRotation().getRadians();
        double launcherVelocityX =
                fieldVelocity.vxMetersPerSecond
                        - fieldVelocity.omegaRadiansPerSecond
                                * (robotToLauncher.getX() * Math.sin(robotAngle)
                                        + robotToLauncher.getY() * Math.cos(robotAngle));
        double launcherVelocityY =
                fieldVelocity.vyMetersPerSecond
                        + fieldVelocity.omegaRadiansPerSecond
                                * (robotToLauncher.getX() * Math.cos(robotAngle)
                                        - robotToLauncher.getY() * Math.sin(robotAngle));

        // ── Decompose velocity into radial and tangential components ──────────
        // Unit vector from launcher toward target
        double ux = launcherToTarget.getX() / distanceNoLookahead;
        double uy = launcherToTarget.getY() / distanceNoLookahead;
        // Positive radialVelocity = closing on target
        double radialVelocity = launcherVelocityX * ux + launcherVelocityY * uy;
        // Tangential: perpendicular to the radial axis
        double tangentialVelocity = -launcherVelocityX * uy + launcherVelocityY * ux;

        // ── Polynomial + 1690 virtual-target solver ───────────────────────────
        // Returns: { exitSpeed_ms, launchAngle_deg, yawOffset_deg, virtualDist_m, tof_s }
        double[] poly =
                solveVirtualTarget(model, distanceNoLookahead, radialVelocity, tangentialVelocity);
        double exitSpeedMs = poly[0];
        double rawHoodAngle = 90 - poly[1]; // degrees, before HOOD_ANGLE_OFFSET
        double yawOffsetDeg = poly[2];
        double lookaheadDist = poly[3];
        double tofFinal = poly[4];

        // ── Turret angle: static bearing + shoot-on-move yaw + user offset ────
        Rotation2d turretAngle =
                launcherToTarget
                        .getAngle()
                        .plus(Rotation2d.fromDegrees(yawOffsetDeg))
                        .plus(Rotation2d.fromDegrees(TURRET_ANGLE_OFFSET));

        // ── Lookahead pose: estimated launcher position when the ball arrives ────
        // Useful for Field2d visualization and validating shoot-on-move compensation.
        Pose2d lookaheadPose =
                new Pose2d(
                        launcherPose
                                .getTranslation()
                                .plus(
                                        new Translation2d(
                                                launcherVelocityX * tofFinal,
                                                launcherVelocityY * tofFinal)),
                        turretAngle);

        // Turret angular velocity (rotations/s) for heading feedforward
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        double deltaRot =
                MathUtil.inputModulus(turretAngle.minus(lastTurretAngle).getRotations(), -0.5, 0.5);
        double turretAngularVelocity = turretAngleFilter.calculate(deltaRot / LOOP_PERIOD_SECS);
        lastTurretAngle = turretAngle;

        // ── Hood angle + velocity ─────────────────────────────────────────────
        // Compute velocity on the raw (un-offset) angle so HOOD_ANGLE_OFFSET (a
        // near-constant) does not bleed into the derivative.
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = rawHoodAngle;
        double hoodVelocity =
                hoodAngleFilter.calculate((rawHoodAngle - lastHoodAngle) / LOOP_PERIOD_SECS);
        lastHoodAngle = rawHoodAngle;
        double hoodAngle =
                MathUtil.clamp(
                        rawHoodAngle + model.hoodOffsetDeg() + HOOD_ANGLE_OFFSET,
                        Robot.getHood().getConfig().getMinRotations() * 360.0,
                        Robot.getHood().getConfig().getMaxRotations() * 360.0);

        // ── Flywheel speed: exit speed (m/s) → RPM ───────────────────────────
        double flywheelSpeed = exitSpeedMs * RPM_PER_MPS;

        // Snapshot for the shot record: the five values a burst row needs that
        // ShootingParameters does not carry. Kept here rather than widened into the record because
        // nothing in the control path reads them.
        activeModelName = model.name;
        activeModelHoodOffsetDeg = model.hoodOffsetDeg();
        activeRadialVelocityMs = radialVelocity;
        activeTangentialVelocityMs = tangentialVelocity;
        activeFeedShot = feed;

        // ── Validity ──────────────────────────────────────────────────────────
        boolean isValid =
                distanceNoLookahead >= model.distMin() && distanceNoLookahead <= model.distMax();

        latestParameters =
                new ShootingParameters(
                        isValid,
                        turretAngle,
                        turretAngularVelocity,
                        hoodAngle,
                        hoodVelocity,
                        flywheelSpeed,
                        exitSpeedMs,
                        lookaheadDist,
                        distanceNoLookahead,
                        tofFinal);

        // Fifteen keys that move with the pose every loop. Loop rate while a shot is in progress,
        // when they are the record of what was aimed; 10 Hz the rest of the time.
        boolean launching =
                Robot.getSuperStructure() != null
                        && Robot.getSuperStructure().currentStateIsLaunching();
        if (launching || Telemetry.slowLogThisLoop()) {
            Telemetry.log("ShotCalc/LookaheadPose", lookaheadPose);
            Telemetry.logDash("ShotCalc/DistanceMeters", lookaheadDist, "meters");
            Telemetry.log("ShotCalc/DistanceNoLookahead", distanceNoLookahead, "meters");
            Telemetry.logDash("ShotCalc/TurretAngleDeg", turretAngle.getDegrees(), "degrees");
            Telemetry.log("ShotCalc/YawOffsetDeg", yawOffsetDeg, "degrees");
            Telemetry.logDash("ShotCalc/HoodAngleDeg", hoodAngle, "degrees");
            Telemetry.logDash("ShotCalc/FlywheelSpeedRPM", flywheelSpeed, "RPM");
            Telemetry.log("ShotCalc/ExitSpeedMs", exitSpeedMs, "m/s");
            Telemetry.log("ShotCalc/RadialVelocityMs", radialVelocity, "m/s");
            Telemetry.log("ShotCalc/TangentialVelocityMs", tangentialVelocity, "m/s");
            Telemetry.logDash("ShotCalc/TimeOfFlight", tofFinal, "seconds");
            Telemetry.log("ShotCalc/FeedShot", feed);
            Telemetry.logDash("ShotCalc/HubPolyModel", WANTED_HUB_MODEL.name);
            Telemetry.logDash("ShotCalc/TurretAngleOffsetDegrees", TURRET_ANGLE_OFFSET, "degrees");
            Telemetry.logDash("ShotCalc/HoodAngleOffsetDegrees", HOOD_ANGLE_OFFSET, "degrees");
            Telemetry.logDash(
                    "ShotCalc/HoodModelOffsetDegrees", WANTED_HUB_MODEL.hoodOffsetDeg(), "degrees");
            Telemetry.log("ShotCalc/Target", target);
        }

        return latestParameters;
    }

    /**
     * Clears the cached parameters so they are recomputed on the next call to {@link
     * #getParameters()}.
     */
    public void clearShootingParameters() {
        latestParameters = null;
    }

    // =========================================================================
    // Private — Polynomial Solver
    // =========================================================================

    /**
     * 1690 Orbit iterative virtual-target solver.
     *
     * <p>Each pass evaluates the polynomial at the current virtual aim point, reads the fitted
     * time-of-flight, shifts the aim point by how far the launcher moves during that flight, and
     * repeats until TOF converges. Terminates in ≤ 5 iterations (typically 2–3).
     *
     * @param model the polynomial model (hub or feed) to evaluate against
     * @param distance horizontal distance to goal centre (metres)
     * @param radialVelocity launcher velocity toward/away from goal (m/s); positive = closing on
     *     goal
     * @param tangentialVelocity launcher velocity perpendicular to goal line (m/s)
     * @return {@code double[]} with indices:
     *     <ul>
     *       <li>0 — exit speed (m/s), scaled by {@link #MPS_FACTOR}
     *       <li>1 — launch angle (degrees), raw polynomial value
     *       <li>2 — yaw offset (degrees); add to static bearing before firing
     *       <li>3 — converged virtual aim distance (metres)
     *       <li>4 — converged time of flight (seconds)
     *     </ul>
     */
    private static double[] solveVirtualTarget(
            PolyModel model, double distance, double radialVelocity, double tangentialVelocity) {
        double vdx = distance; // virtual aim point — radial component (m)
        double vdz = 0.0; // virtual aim point — lateral component (m)
        double tof = 0.0;
        double lead = 0.0; // effective lead time after drag bleed (s)

        for (int iter = 0; iter < 5; iter++) {
            double vDist = Math.sqrt(vdx * vdx + vdz * vdz);
            if (vDist < 0.1) break;

            // Evaluate polynomial at virtual point with rv = 0 (robot motion is
            // already encoded in the shifted aim point)
            double[] raw = evalPolyRaw(model, vDist, 0.0);
            double prevTof = tof;
            tof = raw[2];
            lead = tof * driftEfficiency(tof);

            // Shift aim point: where the target will be relative to the launcher
            // when the ball arrives
            vdx = distance - radialVelocity * lead;
            vdz = -tangentialVelocity * lead;

            if (iter > 0 && Math.abs(tof - prevTof) < 0.002) break;
        }

        double virtualDist = Math.sqrt(vdx * vdx + vdz * vdz);
        double yawOffsetDeg =
                Math.atan2(-tangentialVelocity * lead, distance - radialVelocity * lead)
                        * (180.0 / Math.PI);

        double[] result = evalPolyRaw(model, virtualDist, 0.0);
        return new double[] {
            result[0] * MPS_FACTOR, // exitSpeed_ms
            result[1], // launchAngle_deg
            yawOffsetDeg, // yaw correction (degrees)
            virtualDist, // converged lookahead distance (m)
            result[2] // time of flight at the converged aim point (s)
        };
    }

    /**
     * Fraction of the chassis velocity the ball inherits that actually survives to the target.
     *
     * <p>Drag bleeds off the inherited velocity during flight, so the ball drifts less than {@code
     * velocity * timeOfFlight}. Leading by the full product over-counter-aims by an amount that
     * grows with both robot speed and flight time. Measured against the ball sim at 0.89 / 0.86 /
     * 0.82 for 4 / 6 / 8 m shots — independent of robot speed, and close enough to linear in flight
     * time to model with a single coefficient.
     *
     * @param tofSeconds ball time of flight (seconds)
     * @return drift efficiency in [0, 1]
     */
    private static double driftEfficiency(double tofSeconds) {
        return MathUtil.clamp(1.0 - LEAD_DRAG_BLEED_PER_SEC * tofSeconds, 0.0, 1.0);
    }

    /**
     * Evaluates the given polynomial surface at (distance, radialVel). Inputs are clamped to the
     * model's fitted data range. Returns raw polynomial output — callers are responsible for
     * applying {@link #MPS_FACTOR} to the exit speed and {@code HOOD_ANGLE_OFFSET} to the launch
     * angle.
     *
     * @param model the polynomial model (hub or feed) to evaluate
     * @param distance horizontal distance to the aim point (metres)
     * @param radialVel radial velocity (m/s)
     * @return double[] { exitSpeed_ms (raw, before MPS_FACTOR), launchAngle_deg, tof_s }
     */
    private static double[] evalPolyRaw(PolyModel model, double distance, double radialVel) {
        double d_raw = Math.max(model.distMin(), Math.min(model.distMax(), distance));
        double v_raw = Math.max(model.rvMin(), Math.min(model.rvMax(), radialVel));
        double d = (d_raw - model.dMean()) / model.dStd();
        double v = (v_raw - model.vMean()) / model.vStd();

        double d2 = d * d;
        double v2 = v * v;
        double d3 = d2 * d;
        double v3 = v2 * v;

        double[] terms = {
            1.0, // 1
            d, // d
            v, // v
            d2, // d²
            d * v, // d·v
            v2, // v²
            d3, // d³
            d2 * v, // d²·v
            d * v2, // d·v²
            v3 // v³
        };

        double[] speedCoeffs = model.speedCoeffs();
        double[] angleCoeffs = model.angleCoeffs();
        double[] tofCoeffs = model.tofCoeffs();
        double exitSpeed = 0.0, launchAngle = 0.0, tof = 0.0;
        for (int i = 0; i < terms.length; i++) {
            exitSpeed += speedCoeffs[i] * terms[i];
            launchAngle += angleCoeffs[i] * terms[i];
            if (tofCoeffs != null) tof += tofCoeffs[i] * terms[i];
        }
        return new double[] {exitSpeed, launchAngle, tof};
    }
}
