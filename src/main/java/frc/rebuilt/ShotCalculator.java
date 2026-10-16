package frc.rebuilt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
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

    private static ShotCalculator instance;

    /** Robot-centre to launcher offset. Zero = launcher is at robot centre. */
    private static final Transform2d robotToLauncher = Transform2d.kZero;

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    /**
     * Immutable snapshot of everything needed to command the turret, hood and flywheel for one
     * shot.
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

    public static final double STARTING_HOOD_ANGLE_OFFSET = 0; // degrees
    public static double HOOD_ANGLE_OFFSET = STARTING_HOOD_ANGLE_OFFSET;

    /**
     * Degrees per operator D-pad press. A quarter of a degree is about a quarter of a foot of range
     * near where this robot shoots, finer than anyone can judge from watching a ball land.
     */
    public static final double HOOD_OFFSET_STEP_DEG = 0.25;

    /** Degrees per operator D-pad press on the turret trim. */
    public static final double TURRET_OFFSET_STEP_DEG = 1.0;

    /**
     * The turret trim is session-only and starts at zero every boot, because it corrects one
     * match's pose error rather than calibrating anything. It never touches {@link Preferences}.
     */
    public static final double STARTING_TURRET_ANGLE_OFFSET = 0; // degrees

    public static double TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;

    public enum TrimAxis {
        HOOD,
        TURRET
    }

    /**
     * Largest trim either axis will hold, degrees either side of zero. Ten degrees is ten feet of
     * hood range near where this robot shoots, far past any real fit. The cap also stops a corrupt
     * or hand-edited preference from commanding the turret off target before anyone notices: the
     * hood is clamped again downstream against its soft limits, but the turret trim is not.
     */
    public static final double MAX_TRIM_DEG = 10.0;

    /**
     * Preferences key the hood trim persists under. Flat name, no slash: {@link Preferences} keeps
     * everything in one NetworkTables table, and a slash would nest a sub-table its own {@code
     * getKeys()} does not walk. Public so a test or the robot app reads trims off the rio with the
     * same string rather than a copy of it.
     */
    public static final String HOOD_TRIM_PREF_KEY = "ShotHoodTrimDeg";

    /**
     * Dead key. No turret trim is written any more, and {@link #loadPersistedTrims()} removes it so
     * a rio that still holds one cannot apply it.
     */
    public static final String TURRET_TRIM_PREF_KEY = "ShotTurretTrimDeg";

    /**
     * Dead key. No flywheel trim exists, and {@link #loadPersistedTrims()} deletes any stored copy
     * so nothing can read it back.
     */
    public static final String FLYWHEEL_TRIM_PREF_KEY = "ShotFlywheelTrimPct";

    /**
     * Reads the hood trim back off the rio. Call once during robot construction, before any binding
     * can move a trim.
     *
     * <p>A trim in flash survives a power cycle as well as a redeploy, so the boot print and the
     * operator's Start+Select reset are the safeguard against a stale correction. An expiry would
     * be the wrong safeguard: it would zero a trim in the middle of a session the operator still
     * thought was calibrated.
     */
    public static void loadPersistedTrims() {
        Preferences.initDouble(HOOD_TRIM_PREF_KEY, STARTING_HOOD_ANGLE_OFFSET);

        // The turret trim is session-only, so drop any copy an older build left in flash.
        if (Preferences.containsKey(TURRET_TRIM_PREF_KEY)) {
            Telemetry.print(
                    String.format(
                            "Removed a stored turret trim of %+.2f deg from the rio; the turret trim"
                                    + " starts at zero every boot now.",
                            Preferences.getDouble(TURRET_TRIM_PREF_KEY, 0)),
                    PrintPriority.HIGH);
            Preferences.remove(TURRET_TRIM_PREF_KEY);
        }
        TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;

        // Nothing writes the flywheel trim, so drop any stored copy.
        if (Preferences.containsKey(FLYWHEEL_TRIM_PREF_KEY)) {
            Preferences.remove(FLYWHEEL_TRIM_PREF_KEY);
        }

        double storedHood = Preferences.getDouble(HOOD_TRIM_PREF_KEY, STARTING_HOOD_ANGLE_OFFSET);
        HOOD_ANGLE_OFFSET = MathUtil.clamp(storedHood, -MAX_TRIM_DEG, MAX_TRIM_DEG);

        if (HOOD_ANGLE_OFFSET != storedHood) {
            Telemetry.print(
                    String.format(
                            "!!! Stored hood trim %.2f deg was out of range and was clamped."
                                    + " Something other than the operator D-pad wrote it.",
                            storedHood),
                    PrintPriority.HIGH);
            writeTrimPreferences();
        }

        if (HOOD_ANGLE_OFFSET != 0) {
            Telemetry.print(
                    String.format(
                            "!!! Persisted hood trim is in effect: %+.2f deg. This came off the"
                                    + " rio, not from this session. Operator Start+Select zeroes"
                                    + " it.",
                            HOOD_ANGLE_OFFSET),
                    PrintPriority.HIGH);
        } else {
            Telemetry.print(
                    "Hood trim loaded from the rio: zero. Turret trim starts at zero every boot.",
                    PrintPriority.HIGH);
        }
    }

    /**
     * Whether each model's {@code hoodOffsetDeg} calibration is applied. Those offsets correct how
     * the real robot's shots land, and the simulated ball flies the fitted model, so in simulation
     * they only make it miss.
     */
    private static boolean applyModelHoodOffsets = true;

    /** The model's hood calibration offset, or zero when model offsets are off. */
    private static double modelHoodOffsetDeg(PolyModel model) {
        return applyModelHoodOffsets ? model.hoodOffsetDeg() : 0;
    }

    /**
     * Zeroes every hood and turret trim for simulation, including the model's {@code hoodOffsetDeg}
     * calibration. Use this instead of {@link #loadPersistedTrims()}, since the sim's Preferences
     * file on the laptop keeps whatever the last session nudged. D-pad nudges still work for the
     * session.
     */
    public static void zeroTrimsForSimulation() {
        HOOD_ANGLE_OFFSET = 0;
        TURRET_ANGLE_OFFSET = 0;
        applyModelHoodOffsets = false;
        Telemetry.print(
                "Simulation: operator trims and model hood offsets are zero.", PrintPriority.HIGH);
    }

    /** Writes the persisted hood trim. The turret trim is session-only and is never written. */
    private static void writeTrimPreferences() {
        Preferences.setDouble(HOOD_TRIM_PREF_KEY, HOOD_ANGLE_OFFSET);
    }

    /**
     * Applies a trim nudge, persists it, and logs it as a shot outcome.
     *
     * <p>Every press is a judgement about the last burst: hood down means it went long, hood up
     * means it fell short, and the turret pair say which side it missed on. That makes the D-pad
     * the outcome signal, so there are no separate short, made and long buttons. See {@code
     * docs/tools/shot-log.md} for how the two record streams pair up.
     *
     * @param delta signed nudge in degrees, before clamping
     */
    private static void nudgeTrim(TrimAxis axis, double delta) {
        double before;
        double after;
        if (axis == TrimAxis.HOOD) {
            before = HOOD_ANGLE_OFFSET;
            after = MathUtil.clamp(before + delta, -MAX_TRIM_DEG, MAX_TRIM_DEG);
            HOOD_ANGLE_OFFSET = after;
            // Preferences writes go to flash and through NetworkTables. Safe here, in a command
            // that runs once per press; never do this from a periodic.
            writeTrimPreferences();
        } else {
            // The turret trim is session-only and never written.
            before = TURRET_ANGLE_OFFSET;
            after = MathUtil.clamp(before + delta, -MAX_TRIM_DEG, MAX_TRIM_DEG);
            TURRET_ANGLE_OFFSET = after;
        }
        logTrimEvent(axis, after - before, after, false);
    }

    /** Increase hood angle offset. The operator's way of saying the last shot fell short. */
    public static Command increaseHoodAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(TrimAxis.HOOD, HOOD_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.increaseHoodTrim");
    }

    /** Decrease hood angle offset. The operator's way of saying the last shot went long. */
    public static Command decreaseHoodAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(TrimAxis.HOOD, -HOOD_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.decreaseHoodTrim");
    }

    public static Command increaseTurretAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(TrimAxis.TURRET, TURRET_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.increaseTurretTrim");
    }

    public static Command decreaseTurretAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(TrimAxis.TURRET, -TURRET_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.decreaseTurretTrim");
    }

    /**
     * Zeroes both trims and clears the stored hood trim from flash.
     *
     * <p>Bound to a two-button chord because it has to be reachable in the pit without being
     * reachable by accident.
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
                                    TrimAxis.HOOD,
                                    HOOD_ANGLE_OFFSET - hoodBefore,
                                    HOOD_ANGLE_OFFSET,
                                    true);
                            logTrimEvent(
                                    TrimAxis.TURRET,
                                    TURRET_ANGLE_OFFSET - turretBefore,
                                    TURRET_ANGLE_OFFSET,
                                    true);
                            Telemetry.print(
                                    String.format(
                                            "Shot trims reset to zero (were hood %+.2f deg, turret"
                                                    + " %+.2f deg).",
                                            hoodBefore, turretBefore),
                                    PrintPriority.HIGH);
                        })
                .ignoringDisable(true)
                .withName("ShotCalculator.resetTrims");
    }

    // Two sparse streams, one row per event, both wpilog only:
    //   ShotCalc/Shot/*  one row when the feed gate opens, saying what was aimed
    //   ShotCalc/Trim/*  one row per operator D-pad press, saying how it went
    // Neither is loop rate; balls per burst are counted afterwards from the dips in Launcher/RPM,
    // which is kept at loop rate for that. DogLog skips a record whose value has not changed, so a
    // burst at the same distance with the same model writes an index and a timestamp and little
    // else. Read a row by taking each key's last value at or before that row's timestamp. See
    // docs/tools/shot-log.md.

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
     * <p>Called on the rising edge of the feed gate, the first loop fuel is allowed into the
     * flywheel and so the last loop the aim was still a prediction.
     *
     * <p>Two omissions are deliberate. There is no outcome field, because the outcome arrives later
     * as a trim press. And the vision turret-zero split is not copied in: it is already logged at
     * 10 Hz and joins on time, and duplicating it here would let the two drift apart.
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

        // Dash only: the operator watches bursts count up to confirm records are being written at
        // all, and one publish per burst is nothing next to the loop-rate traffic.
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

        // Actuals, null-guarded so a sim or a bench run can call this before every mechanism
        // exists and a missing number reads as NaN rather than crashing the loop.
        Telemetry.log(
                "ShotCalc/Shot/ActualRPM",
                Robot.getLauncher() == null ? Double.NaN : Robot.getLauncher().getVelocityRPM(),
                "RPM");
        Telemetry.log(
                "ShotCalc/Shot/ActualHoodDeg",
                Robot.getHood() == null ? Double.NaN : Robot.getHood().getPositionDegrees(),
                "degrees");
        // Measured minus commanded. Note that Turret/PositionError is logged with the opposite
        // sign, and which way a positive value points in the world is not documented on the
        // turret, so read this as a magnitude unless you have checked.
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
     * and an infinite age, and analysis drops it. Deciding what counts as seconds after is the
     * reader's job, so the age is logged rather than thresholded here.
     *
     * @param delta how far the trim actually moved in degrees, after clamping; zero at the limit
     * @param value the trim's new value in degrees
     * @param reset true when this row is the Start+Select reset rather than a judgement
     */
    private static void logTrimEvent(TrimAxis axis, double delta, double value, boolean reset) {
        double now = Timer.getFPGATimestamp();
        double secondsSinceShot =
                Double.isNaN(lastShotTimestampSeconds)
                        ? Double.POSITIVE_INFINITY
                        : now - lastShotTimestampSeconds;

        String verdict;
        if (reset) {
            verdict = "Reset";
        } else if (delta == 0) {
            // The trim was already at its cap. The press is still a verdict about the shot, and
            // losing it would bias the dataset towards whichever direction had room left.
            verdict = "AtLimit";
        } else if (axis == TrimAxis.HOOD) {
            // Hood up means the operator is adding range: the ball fell short.
            verdict = delta > 0 ? "Short" : "Long";
        } else {
            // A CCW trim correction means the ball landed CW of the target.
            verdict = delta > 0 ? "MissedCW" : "MissedCCW";
        }

        trimEventIndex++;
        Telemetry.log("ShotCalc/Trim/Index", trimEventIndex);
        Telemetry.log("ShotCalc/Trim/TimestampSeconds", now, "seconds");
        Telemetry.log("ShotCalc/Trim/Axis", axis == TrimAxis.HOOD ? "Hood" : "Turret");
        Telemetry.log("ShotCalc/Trim/DeltaDeg", delta, "degrees");
        Telemetry.log("ShotCalc/Trim/ValueDeg", value, "degrees");
        Telemetry.log("ShotCalc/Trim/Verdict", verdict);
        Telemetry.log(
                "ShotCalc/Trim/ShotIndex", Double.isNaN(lastShotTimestampSeconds) ? -1 : shotIndex);
        Telemetry.log("ShotCalc/Trim/SecondsSinceShot", secondsSinceShot, "seconds");
        Telemetry.log("ShotCalc/Trim/ShotDistanceMeters", lastShotDistanceMeters, "meters");
    }

    // 2D degree-3 polynomial surface: f(distance_m, radialVel_ms) -> {exitSpeed_ms,
    // launchAngle_deg}
    // Monomial basis: 1, d, v, d², d·v, v², d³, d²·v, d·v², v³

    /**
     * Global exit-speed scale factor. Adjust post-characterization to correct for ball compression,
     * wear or temperature without refitting the polynomial. 1.0 = no scaling. Applied to both the
     * hub and feed models.
     */
    private static final double MPS_FACTOR = 1;

    /**
     * Scale factor converting polynomial exit speed (m/s) to flywheel RPM: what the shot's power
     * transfer costs in RPM per m/s on the ball.
     *
     * <p>The fitted 365 against the 4 in wheel is a coupling ratio of 0.515, the no-slip figure for
     * a single wheel against a fixed hood: the ball rolls, so its centre leaves at half the surface
     * speed and the rest goes into backspin. Grip worse than no-slip (worn wheels, a light squeeze,
     * a cold ball) puts the real ratio below 0.515, and then every shot lands short at the RPM the
     * model asks for. Fudge this rather than refitting. It is the coupling only: raising it
     * commands more RPM for the same wanted exit speed and leaves the ballistics and the sim's ball
     * alone. {@link #MPS_FACTOR} is the other knob and means something different, that the ball
     * really does leave faster than the poly says, so it moves the simulated ball too.
     */
    private static final double RPM_PER_MPS_FITTED = 365.0;

    /**
     * Boot value for the coupling, the fitted figure unchanged.
     *
     * <p>Rule of thumb if it does need to move: a coupling raise shifts every range by {@code dR =
     * 2R * dv/v}, so each 1 % (about 3.7 RPM per m/s) is worth 0.07 m at 3.5 m and more further
     * out. It is the knob for a bias that is the same sign at every distance and biggest far away.
     * It is the wrong knob for a bias at one end of the range, which {@link
     * #nearShotRpmDrop(double)} handles.
     */
    private static final double RPM_PER_MPS_DEFAULT = RPM_PER_MPS_FITTED;

    /** Clamp on the live {@link #RPM_PER_MPS_DEFAULT} fudge, +/-20 % of the boot value. */
    private static final double RPM_PER_MPS_MIN = RPM_PER_MPS_DEFAULT * 0.8;

    private static final double RPM_PER_MPS_MAX = RPM_PER_MPS_DEFAULT * 1.2;

    /**
     * Live handle on the coupling, on the dashboard as {@code ShotCalc/RpmPerMps}.
     *
     * <p>Deliberately a pit knob a person types a number into rather than a gamepad axis or a
     * {@link Preferences} key. It starts at {@link #RPM_PER_MPS_DEFAULT} every boot and is logged
     * every loop, so a log says which number a shot was taken at. The clamp moves with the boot
     * value, so it is 292 to 438 at the current default.
     */
    private static final DoubleSubscriber RPM_PER_MPS_TUNE =
            Telemetry.tunable("ShotCalc/RpmPerMps", RPM_PER_MPS_DEFAULT);

    private static double rpmPerMps() {
        return MathUtil.clamp(
                RPM_PER_MPS_TUNE.get(RPM_PER_MPS_DEFAULT), RPM_PER_MPS_MIN, RPM_PER_MPS_MAX);
    }

    /**
     * Shape of the near-shot correction: fraction of {@link #NEAR_SHOT_RPM_DROP_DEFAULT} to take
     * off the flywheel command, indexed by distance to the hub in metres. 1.0 at 3.0 m and inside,
     * zero from 3.75 m out, and rising below 2.5 m. Endpoints hold outside the table, so the
     * hub-face set shot at 0.98 m gets the 1.5 m value.
     *
     * <p>At the fitted coupling, practice shooting on 2026-09-19 had every hub shot from about
     * tower radius inward landing long and the far shots landing. Taking 1.5 deg of hood out fixed
     * the near shots and dropped the far ones short, so the range comes off exit speed instead,
     * which also keeps the near shot lower. The size comes from converting that 1.5 deg, about 0.25
     * to 0.3 m of range, into the exit-speed change that removes the same range at the model's own
     * hood angle in vacuum: 4.4 % at 3.0 m, 5.2 % at 2.5 m, 6.8 % at 2.0 m, 10.6 % at 1.5 m. Speed
     * is a weak knob at the steep near angles (84 deg launch at 1.5 m), which is why the shape
     * rises so fast inside 2.5 m.
     */
    private static final InterpolatingDoubleTreeMap NEAR_SHOT_DROP_SHAPE =
            new InterpolatingDoubleTreeMap();

    static {
        NEAR_SHOT_DROP_SHAPE.put(1.5, 1.7);
        NEAR_SHOT_DROP_SHAPE.put(2.0, 1.25);
        NEAR_SHOT_DROP_SHAPE.put(2.5, 1.05);
        NEAR_SHOT_DROP_SHAPE.put(3.0, 1.0);
        NEAR_SHOT_DROP_SHAPE.put(3.75, 0.0);
    }

    /**
     * RPM taken off the hub-shot flywheel command at 3.0 m and inside, before the shape scaling.
     * Best guess from the 2026-09-19 practice field, see {@link #NEAR_SHOT_DROP_SHAPE}; expect to
     * move it by 50 to 100 RPM once the near shots have been watched at this value.
     */
    private static final double NEAR_SHOT_RPM_DROP_DEFAULT = 150.0;

    /** Clamp on the live near-shot drop. Zero disables it; 400 RPM is about 15 % at 3 m. */
    private static final double NEAR_SHOT_RPM_DROP_MAX = 400.0;

    /**
     * Live handle on the near-shot drop, on the dashboard as {@code ShotCalc/NearShotRpmDrop}. Same
     * rules as {@link #RPM_PER_MPS_TUNE}: back to {@link #NEAR_SHOT_RPM_DROP_DEFAULT} every boot,
     * never on the gamepad and never persisted, and logged every loop a shot is in progress.
     */
    private static final DoubleSubscriber NEAR_SHOT_RPM_DROP_TUNE =
            Telemetry.tunable("ShotCalc/NearShotRpmDrop", NEAR_SHOT_RPM_DROP_DEFAULT);

    /**
     * @param distanceMeters launcher to hub centre, the distance the model was evaluated at
     * @return a non-negative RPM reduction, zero at and beyond 3.75 m
     */
    private static double nearShotRpmDrop(double distanceMeters) {
        double size =
                MathUtil.clamp(
                        NEAR_SHOT_RPM_DROP_TUNE.get(NEAR_SHOT_RPM_DROP_DEFAULT),
                        0.0,
                        NEAR_SHOT_RPM_DROP_MAX);
        return size * NEAR_SHOT_DROP_SHAPE.get(distanceMeters);
    }

    /**
     * Flywheel speed the launcher can actually hold, measured rather than specified: 416.5 RPM per
     * volt applied, the median of eight Chezy match logs (spread 407 to 430, and the inverse of the
     * fitted {@code velocityKv = 0.1425}). The gearing does not set the ceiling, the battery does.
     * At the fastest moment ever logged, 4194 RPM in Q45, the motor was applying 9.94 V against a
     * 9.95 V bus: saturated, with 5.5 % of that match's launch samples within a volt of the same
     * wall.
     */
    private static final double RPM_PER_VOLT = 416.5;

    /**
     * Bus voltage to size the ceiling against. Not the 12 V of a resting battery: during a launch
     * burst the measured bus sits between 8 and 10.5 V, and Q45 touched 7.35 V at 402 A. 9.85 V is
     * what a healthy pack held at the top of its range.
     */
    private static final double USABLE_BUS_VOLTS = 9.85;

    /**
     * Fastest flywheel speed worth commanding with the current gearing: about 4100 RPM. Nothing in
     * the normal range reaches this, since the model tops out at 3780 RPM at the fitted coupling.
     *
     * <p>Asking for more does not make the ball faster, it makes {@link
     * frc.robot.subsystems.launcher.Launcher#isAtSpeed()} unsatisfiable: that gate is a +/-200 RPM
     * window around the command, so a command the flywheel cannot reach keeps the shot-ready gate
     * shut and no fuel feeds. Clamping here means a far shot fires a little short instead of not
     * firing at all. Raise this when the battery improves, to {@link #RPM_PER_VOLT} times the bus
     * volts a burst actually holds, less a little so the gate's window can close.
     */
    private static final double MAX_FEASIBLE_FLYWHEEL_RPM =
            Math.floor(RPM_PER_VOLT * USABLE_BUS_VOLTS / 50.0) * 50.0;

    /**
     * @param wantedRPM the model's flywheel speed, fudge already applied
     * @return the same speed, or {@link #MAX_FEASIBLE_FLYWHEEL_RPM} when it was over the ceiling
     */
    private static double feasibleFlywheelRPM(double wantedRPM) {
        return Math.min(wantedRPM, MAX_FEASIBLE_FLYWHEEL_RPM);
    }

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
     * @param speedCoeffs exit-speed coefficients in the monomial basis 1, d, v, d², d·v, v², d³,
     *     d²·v, d·v², v³
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
                    // Shots landed 3 to 4 ft past the hub centre on 2026-09-05. The hood is worth
                    // about a degree per foot of range here, so 4 deg down; still long that
                    // evening, so 5.
                    -5.0);

    /** Hub model fitted for a 3 m ceiling, for testing at home. */
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
                    // Scores as it is, so no calibration offset.
                    0.0);

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
     * Active hub model. The name is logged to {@code ShotCalc/HubPolyModel}, so check it before a
     * match: the two fits do not shoot the same and nothing else makes the difference obvious. The
     * ceiling fit is the known-good fallback, at the cost of a trajectory shaped to stay under a 3
     * m roof.
     */
    private static final PolyModel WANTED_HUB_MODEL = HUB_MODEL;

    /**
     * The fixed shots, one per parking spot. Each is a range to the hub centre and a turret angle;
     * the hood and flywheel come off the live hub model at that range, at a standstill, so a set
     * shot follows the shot map without anyone retyping numbers.
     *
     * <p>Ranges are worked from the field geometry, not measured. The robot centre, and therefore
     * the launcher, sits 15 in inside the bumper of a 30 in robot. The hub centre is the midpoint
     * of tags 26 and 20, (4.626, 4.035) m.
     *
     * <p>The turret's zero points away from the intake, so an intake-facing spot means the turret
     * turns a half turn to shoot back over it. That is -180 rather than +180: the travel is -216 to
     * +180 deg, and a command sitting exactly on the forward soft limit has no margin. Every spot
     * except the hub face parks intake-away, turret at zero, which from most tracked turret
     * positions is the short move.
     */
    public enum SetShot {
        /**
         * Parked against the tower's field-facing wall, intake to the wall, turret at zero shooting
         * the hub. The tower front face is 43.51 in on the tower centreline (tag 31, y = 3.746 m),
         * so the robot centre is at (1.486, 3.746) m: 3.15 m, 10.3 ft.
         */
        TOWER("Tower", 3.15, 0.0),
        /**
         * Bumper against the hub's near face, intake to the hub, turret over the intake. Hub half
         * width 23.5 in plus 15 in: 0.98 m. That is below the model's fitted 1.5 m floor, so the
         * model is evaluated at 1.5 m and clamped there. Untested at the time of writing, so treat
         * it as a lob.
         */
        HUB_FACE("HubFace", 0.978, -180.0),
        /**
         * Sitting in the left trench lane with the robot just clear of the trench, intake pointed
         * away from the hub, turret at zero shooting back over the far bumper. The trench opening
         * is 50.34 in wide at the wall, so its centreline is 3.395 m from the field centreline; the
         * robot centre is 23.5 + 15 in along x from the hub centre once it has cleared the 47 in
         * trench: 3.53 m. Sitting inside the trench instead is 3.40 m, 13 cm less, which the model
         * barely notices. The robot is 30 in square, so which end faces the hub does not move its
         * centre, and the range is the same as it was intake-to-hub.
         */
        LEFT_TRENCH("LeftTrench", 3.53, 0.0),
        /** Mirror of {@link #LEFT_TRENCH}. */
        RIGHT_TRENCH("RightTrench", 3.53, 0.0);

        /** Short name, logged to {@code ShotCalc/SetShot}. */
        public final String label;

        /** Launcher to hub centre, metres. */
        public final double distanceMeters;

        /** Turret mechanism angle, degrees from its zero. */
        public final double turretDegrees;

        SetShot(String label, double distanceMeters, double turretDegrees) {
            this.label = label;
            this.distanceMeters = distanceMeters;
            this.turretDegrees = turretDegrees;
        }
    }

    public static final double SET_SHOT_DISTANCE_METERS = SetShot.TOWER.distanceMeters;

    private static volatile SetShot selectedSetShot = SetShot.TOWER;

    /**
     * Picks which fixed shot the SET_SHOT super state runs. Called from the pilot binding before
     * the state is requested; the selection sticks until the next binding changes it.
     */
    public static void selectSetShot(SetShot shot) {
        selectedSetShot = shot;
        Telemetry.logDashAlways("ShotCalc/SetShot", shot.label);
    }

    /** The selected set shot, {@link SetShot#TOWER} until anything picks one. */
    public static SetShot getSelectedSetShot() {
        return selectedSetShot;
    }

    /**
     * Hood angle and flywheel speed for the selected {@link SetShot}, at a standstill.
     *
     * <p>Read off the same fitted surface a tracked shot uses, at a fixed distance with zero
     * velocity, so it moves with the model and with the operator's D-pad hood trim instead of being
     * a pair of magic numbers that go stale on the next refit. It never touches the robot pose,
     * which is the point: this is what gets used when the pose is the thing that has failed.
     *
     * @return {@code { hoodDegrees, flywheelRPM }}
     */
    private static double[] setShotSolution() {
        double[] raw = evalPolyRaw(WANTED_HUB_MODEL, selectedSetShot.distanceMeters, 0.0);
        double hoodDegrees =
                MathUtil.clamp(
                        (90 - raw[1]) + modelHoodOffsetDeg(WANTED_HUB_MODEL) + HOOD_ANGLE_OFFSET,
                        Robot.getHood().getConfig().getMinRotations() * 360.0,
                        Robot.getHood().getConfig().getMaxRotations() * 360.0);
        double rpm =
                raw[0] * MPS_FACTOR * rpmPerMps() - nearShotRpmDrop(selectedSetShot.distanceMeters);
        return new double[] {hoodDegrees, feasibleFlywheelRPM(rpm)};
    }

    public static double getSetShotHoodDegrees() {
        return setShotSolution()[0];
    }

    public static double getSetShotFlywheelRPM() {
        return setShotSolution()[1];
    }

    public static double getSetShotTurretDegrees() {
        return selectedSetShot.turretDegrees;
    }

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

    /**
     * The current shooting parameters, computed from the robot's live pose and velocity if not
     * already cached this loop.
     *
     * <p>Call {@link #clearShootingParameters()} at the start of each loop to allow re-computation
     * on the next call.
     */
    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        boolean feed = Robot.getSuperStructure().isRobotInFeedZone();
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();
        // Feed and hub shots use separately-fitted polynomial surfaces.
        PolyModel model = feed ? FEED_MODEL : WANTED_HUB_MODEL;

        Pose2d estimatedPose = Robot.getSwerve().getRobotPose();
        ChassisSpeeds robotRelativeVelocity = Robot.getSwerve().getCurrentRobotChassisSpeeds();
        estimatedPose =
                estimatedPose.exp(
                        new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY_SECS,
                                robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY_SECS,
                                robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY_SECS));

        Pose2d launcherPose = estimatedPose.transformBy(robotToLauncher);
        Translation2d launcherToTarget = target.minus(launcherPose.getTranslation());
        double distanceNoLookahead = launcherToTarget.getNorm();

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

        // Unit vector from launcher toward target
        double ux = launcherToTarget.getX() / distanceNoLookahead;
        double uy = launcherToTarget.getY() / distanceNoLookahead;
        // Positive radialVelocity = closing on target
        double radialVelocity = launcherVelocityX * ux + launcherVelocityY * uy;
        // Tangential: perpendicular to the radial axis
        double tangentialVelocity = -launcherVelocityX * uy + launcherVelocityY * ux;

        // Returns: { exitSpeed_ms, launchAngle_deg, yawOffset_deg, virtualDist_m, tof_s }
        double[] poly =
                solveVirtualTarget(model, distanceNoLookahead, radialVelocity, tangentialVelocity);
        double exitSpeedMs = poly[0];
        double rawHoodAngle = 90 - poly[1]; // degrees, before HOOD_ANGLE_OFFSET
        double yawOffsetDeg = poly[2];
        double lookaheadDist = poly[3];
        double tofFinal = poly[4];

        Rotation2d turretAngle =
                launcherToTarget
                        .getAngle()
                        .plus(Rotation2d.fromDegrees(yawOffsetDeg))
                        .plus(Rotation2d.fromDegrees(TURRET_ANGLE_OFFSET));

        // Estimated launcher position when the ball arrives, for Field2d and for checking
        // shoot-on-move compensation.
        Pose2d lookaheadPose =
                new Pose2d(
                        launcherPose
                                .getTranslation()
                                .plus(
                                        new Translation2d(
                                                launcherVelocityX * tofFinal,
                                                launcherVelocityY * tofFinal)),
                        turretAngle);

        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        double deltaRot =
                MathUtil.inputModulus(turretAngle.minus(lastTurretAngle).getRotations(), -0.5, 0.5);
        double turretAngularVelocity = turretAngleFilter.calculate(deltaRot / LOOP_PERIOD_SECS);
        lastTurretAngle = turretAngle;

        // Velocity comes off the raw angle so HOOD_ANGLE_OFFSET, a near-constant, does not bleed
        // into the derivative.
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = rawHoodAngle;
        double hoodVelocity =
                hoodAngleFilter.calculate((rawHoodAngle - lastHoodAngle) / LOOP_PERIOD_SECS);
        lastHoodAngle = rawHoodAngle;
        double hoodAngle =
                MathUtil.clamp(
                        rawHoodAngle + modelHoodOffsetDeg(model) + HOOD_ANGLE_OFFSET,
                        Robot.getHood().getConfig().getMinRotations() * 360.0,
                        Robot.getHood().getConfig().getMaxRotations() * 360.0);

        // The near-shot drop is a hub-model correction; feed shots are not characterised.
        double nearShotDrop = feed ? 0.0 : nearShotRpmDrop(lookaheadDist);
        double wantedFlywheelSpeed = exitSpeedMs * rpmPerMps() - nearShotDrop;
        double flywheelSpeed = feasibleFlywheelRPM(wantedFlywheelSpeed);

        // Snapshot for the shot record: the values a burst row needs that ShootingParameters does
        // not carry. Kept here rather than widened into the record because nothing in the control
        // path reads them.
        activeModelName = model.name;
        activeModelHoodOffsetDeg = modelHoodOffsetDeg(model);
        activeRadialVelocityMs = radialVelocity;
        activeTangentialVelocityMs = tangentialVelocity;
        activeFeedShot = feed;

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

        // Keys that move with the pose every loop: loop rate while a shot is in progress, when
        // they are the record of what was aimed, and 10 Hz the rest of the time.
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
            Telemetry.log("ShotCalc/RpmPerMps", rpmPerMps(), "RPM per m/s");
            Telemetry.log("ShotCalc/FlywheelWantedRPM", wantedFlywheelSpeed, "RPM");
            Telemetry.log("ShotCalc/NearShotRpmDropApplied", nearShotDrop, "RPM");
            Telemetry.log(
                    "ShotCalc/FlywheelClamped", wantedFlywheelSpeed > MAX_FEASIBLE_FLYWHEEL_RPM);
            Telemetry.log("ShotCalc/RadialVelocityMs", radialVelocity, "m/s");
            Telemetry.log("ShotCalc/TangentialVelocityMs", tangentialVelocity, "m/s");
            Telemetry.logDash("ShotCalc/TimeOfFlight", tofFinal, "seconds");
            Telemetry.log("ShotCalc/FeedShot", feed);
            Telemetry.logDash("ShotCalc/HubPolyModel", WANTED_HUB_MODEL.name);
            Telemetry.logDash("ShotCalc/TurretAngleOffsetDegrees", TURRET_ANGLE_OFFSET, "degrees");
            Telemetry.logDash("ShotCalc/HoodAngleOffsetDegrees", HOOD_ANGLE_OFFSET, "degrees");
            Telemetry.logDash(
                    "ShotCalc/HoodModelOffsetDegrees",
                    modelHoodOffsetDeg(WANTED_HUB_MODEL),
                    "degrees");
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

    /**
     * 1690 Orbit iterative virtual-target solver.
     *
     * <p>Each pass evaluates the polynomial at the current virtual aim point, reads the fitted
     * time-of-flight, shifts the aim point by how far the launcher moves during that flight, and
     * repeats until the time of flight converges. Terminates in at most 5 iterations, usually 2 to
     * 3.
     *
     * @param distance horizontal distance to goal centre (metres)
     * @param radialVelocity launcher velocity toward or away from goal (m/s); positive = closing
     * @param tangentialVelocity launcher velocity perpendicular to goal line (m/s)
     * @return {@code double[]} with indices:
     *     <ul>
     *       <li>0: exit speed (m/s), scaled by {@link #MPS_FACTOR}
     *       <li>1: launch angle (degrees), raw polynomial value
     *       <li>2: yaw offset (degrees); add to the static bearing before firing
     *       <li>3: converged virtual aim distance (metres)
     *       <li>4: converged time of flight (seconds)
     *     </ul>
     */
    private static double[] solveVirtualTarget(
            PolyModel model, double distance, double radialVelocity, double tangentialVelocity) {
        double vdx = distance; // virtual aim point, radial component (m)
        double vdz = 0.0; // virtual aim point, lateral component (m)
        double tof = 0.0;
        double lead = 0.0; // effective lead time after drag bleed (s)

        for (int iter = 0; iter < 5; iter++) {
            double vDist = Math.sqrt(vdx * vdx + vdz * vdz);
            if (vDist < 0.1) break;

            // Evaluate at the virtual point with rv = 0, since robot motion is already encoded in
            // the shifted aim point
            double[] raw = evalPolyRaw(model, vDist, 0.0);
            double prevTof = tof;
            tof = raw[2];
            lead = tof * driftEfficiency(tof);

            // Where the target will be relative to the launcher when the ball arrives
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
     * Fraction of the chassis velocity the ball inherits that survives to the target.
     *
     * <p>Drag bleeds off the inherited velocity during flight, so the ball drifts less than {@code
     * velocity * timeOfFlight}. Leading by the full product over-counter-aims by an amount that
     * grows with both robot speed and flight time. Measured against the ball sim at 0.89, 0.86 and
     * 0.82 for 4, 6 and 8 m shots: independent of robot speed, and close enough to linear in flight
     * time to model with a single coefficient.
     *
     * @return drift efficiency in [0, 1]
     */
    private static double driftEfficiency(double tofSeconds) {
        return MathUtil.clamp(1.0 - LEAD_DRAG_BLEED_PER_SEC * tofSeconds, 0.0, 1.0);
    }

    /**
     * Evaluates the given polynomial surface at (distance, radialVel), with the inputs clamped to
     * the model's fitted data range. Callers apply {@link #MPS_FACTOR} to the exit speed and {@code
     * HOOD_ANGLE_OFFSET} to the launch angle.
     *
     * @param model the polynomial model (hub or feed) to evaluate
     * @param distance horizontal distance to the aim point, in metres
     * @param radialVel radial velocity, in m/s
     * @return {@code double[]} { exitSpeed_ms (raw, before MPS_FACTOR), launchAngle_deg, tof_s }
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
