package frc.rebuilt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.rebuilt.targetFactories.HubTargetFactory;
import frc.robot.Robot;
import frc.spectrumLib.telemetry.Telemetry;
import frc.spectrumLib.telemetry.Telemetry.PrintPriority;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.OptionalDouble;
import java.util.TreeMap;

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
    // Shot maps — CSV launch tables
    // =========================================================================
    //
    // Every hub and feed shot is read off a distance-indexed CSV in src/main/deploy, linearly
    // interpolated between rows and held at the end rows outside them. Edit the CSV and redeploy to
    // change the shot; no code change needed.
    //
    // Columns, by header name, in any order:
    //   distance_m          launcher to target centre, metres
    //   optimal_v_mps       ball exit speed, m/s (converted to RPM by rpmPerMps())
    //   optimal_angle_deg   launch angle above horizontal, degrees (hood = 90 - this)
    //   tof_s               optional ball time of flight, seconds; drag-free d / (v cos θ) without
    // it

    public static final String kDISTANCE_COLUMN_LABEL = "distance_m";
    public static final String kVELOCITY_COLUMN_STRING = "optimal_v_mps";
    public static final String kANGLE_COLUMN_LABEL = "optimal_angle_deg";
    public static final String kTOF_COLUMN_LABEL = "tof_s";

    /**
     * Hub shot map. Its first twelve rows were sampled from the retired 3 m ceiling polynomial at
     * zero radial velocity, the fit the robot was shooting with, so the switch to the CSV left the
     * shot where it was (2026-10-29).
     */
    public static final String HUB_SHOT_MAP_FILE = "lab_shot_map.csv";

    /** Feed shot map, sampled from the retired feed polynomial the same way. Never calibrated. */
    public static final String FEED_SHOT_MAP_FILE = "feed_shot_map.csv";

    /** Robot pose prediction horizon used by {@link #computeLaunchInfo}. */
    public static final double latencySec = 0.05;

    /** One row of a shot map, or an interpolation between two. */
    public static class LaunchParameter implements Interpolatable<LaunchParameter> {
        private final double m_launchVelocityMps;
        private final double m_launchAngleDeg;
        private final double m_timeOfFlightSec;

        public LaunchParameter(
                double launchVelocityMps, double launchAngleDeg, double timeOfFlightSec) {
            this.m_launchVelocityMps = launchVelocityMps;
            this.m_launchAngleDeg = launchAngleDeg;
            this.m_timeOfFlightSec = timeOfFlightSec;
        }

        public double launchVelocityMps() {
            return m_launchVelocityMps;
        }

        public double launchAngleDeg() {
            return m_launchAngleDeg;
        }

        public double timeOfFlightSec() {
            return m_timeOfFlightSec;
        }

        @Override
        public LaunchParameter interpolate(LaunchParameter endValue, double t) {
            if (t <= 0) {
                return this;
            } else if (t >= 1) {
                return endValue;
            } else {
                return new LaunchParameter(
                        MathUtil.interpolate(m_launchVelocityMps, endValue.m_launchVelocityMps, t),
                        MathUtil.interpolate(m_launchAngleDeg, endValue.m_launchAngleDeg, t),
                        MathUtil.interpolate(m_timeOfFlightSec, endValue.m_timeOfFlightSec, t));
            }
        }
    }

    /** A distance-indexed launch table loaded from a CSV in the deploy directory. */
    public static final class ShotMap {
        private final String name;
        private final String fileName;
        private final double hoodOffsetDeg;
        private final InterpolatingTreeMap<Double, LaunchParameter> table =
                new InterpolatingTreeMap<>(
                        InverseInterpolator.forDouble(), LaunchParameter::interpolate);
        private double minDistance = Double.NaN;
        private double maxDistance = Double.NaN;

        /**
         * Loads a shot map.
         *
         * @param name descriptive name for telemetry
         * @param fileName CSV file name in the deploy directory
         * @param hoodOffsetDeg calibration trim added to this map's hood angle, in degrees. Prefer
         *     fixing the CSV; this is for a quick correction that belongs to one map and not the
         *     other. The operator's D-pad trim is added on top and applies to whichever map is
         *     live.
         */
        ShotMap(String name, String fileName, double hoodOffsetDeg) {
            this.name = name;
            this.fileName = fileName;
            this.hoodOffsetDeg = hoodOffsetDeg;
            reload();
        }

        public String name() {
            return name;
        }

        public double hoodOffsetDeg() {
            return hoodOffsetDeg;
        }

        public double minDistance() {
            return minDistance;
        }

        public double maxDistance() {
            return maxDistance;
        }

        /**
         * Launch parameters at a distance, interpolated between rows and held at the end rows.
         *
         * @param distanceM launcher to target centre, metres
         * @return the launch parameters
         */
        public LaunchParameter get(double distanceM) {
            return table.get(MathUtil.clamp(distanceM, minDistance, maxDistance));
        }

        /**
         * Whether a distance is inside the map's rows rather than held at an end.
         *
         * @param distanceM launcher to target centre, metres
         * @return true between the first and last row, inclusive
         */
        public boolean inRange(double distanceM) {
            return distanceM >= minDistance && distanceM <= maxDistance;
        }

        /** Re-reads the CSV. Throws, leaving the old table in place, if the file is unusable. */
        public void reload() {
            Path path = Filesystem.getDeployDirectory().toPath().resolve(fileName);
            List<String> lines;
            try {
                lines = Files.readAllLines(path, StandardCharsets.US_ASCII);
            } catch (IOException e) {
                throw new IllegalStateException("Failed to read shot map CSV: " + path, e);
            }
            if (lines.isEmpty()) {
                throw new IllegalStateException("Shot map CSV is empty: " + path);
            }

            String[] headers = lines.get(0).trim().split(",", -1);
            int distanceIndex = -1;
            int velocityIndex = -1;
            int angleIndex = -1;
            int tofIndex = -1;
            for (int i = 0; i < headers.length; i++) {
                String header = headers[i].trim();
                if (header.equals(kDISTANCE_COLUMN_LABEL)) distanceIndex = i;
                if (header.equals(kVELOCITY_COLUMN_STRING)) velocityIndex = i;
                if (header.equals(kANGLE_COLUMN_LABEL)) angleIndex = i;
                if (header.equals(kTOF_COLUMN_LABEL)) tofIndex = i;
            }
            if (distanceIndex < 0 || velocityIndex < 0 || angleIndex < 0) {
                throw new IllegalStateException(
                        "Shot map CSV is missing a required column ("
                                + kDISTANCE_COLUMN_LABEL
                                + ", "
                                + kVELOCITY_COLUMN_STRING
                                + ", "
                                + kANGLE_COLUMN_LABEL
                                + "): "
                                + path);
            }
            int maximalIndex = Math.max(distanceIndex, Math.max(velocityIndex, angleIndex));
            maximalIndex = Math.max(maximalIndex, tofIndex);

            // Parsed in full before the live table is touched, so a bad edit leaves the old map.
            TreeMap<Double, LaunchParameter> loaded = new TreeMap<>();
            for (int lineIndex = 1; lineIndex < lines.size(); lineIndex++) {
                String line = lines.get(lineIndex).trim();
                if (line.isEmpty() || line.startsWith("#")) continue;
                String[] tokens = line.split(",", -1);
                if (tokens.length <= maximalIndex) {
                    throw new IllegalStateException(
                            "Line " + (lineIndex + 1) + " of " + path + " is missing columns");
                }
                double distance = Double.parseDouble(tokens[distanceIndex].trim());
                double velocity = Double.parseDouble(tokens[velocityIndex].trim());
                double angleDeg = Double.parseDouble(tokens[angleIndex].trim());
                if (distance <= 0 || velocity <= 0 || angleDeg <= 0 || angleDeg >= 90) {
                    throw new IllegalStateException(
                            "Line " + (lineIndex + 1) + " of " + path + " is out of range");
                }
                double tof =
                        tofIndex >= 0 && !tokens[tofIndex].isBlank()
                                ? Double.parseDouble(tokens[tofIndex].trim())
                                : distance / (velocity * Math.cos(Math.toRadians(angleDeg)));
                loaded.put(distance, new LaunchParameter(velocity, angleDeg, tof));
            }
            if (loaded.isEmpty()) {
                throw new IllegalStateException("Shot map CSV has no data rows: " + path);
            }

            table.clear();
            loaded.forEach(table::put);
            minDistance = loaded.firstKey();
            maxDistance = loaded.lastKey();
        }
    }

    /**
     * Hub shot map. The name is logged to {@code ShotCalc/HubShotMap}; edit {@value
     * #HUB_SHOT_MAP_FILE} to move the shot.
     */
    private final ShotMap hubMap = new ShotMap("Lab Hub Shot Map", HUB_SHOT_MAP_FILE, 0.0);

    /** Feed shot map, floor target. */
    private final ShotMap feedMap = new ShotMap("Feed Shot Map", FEED_SHOT_MAP_FILE, 0.0);

    /**
     * The hub shot map.
     *
     * @return the hub shot map
     */
    public ShotMap getHubMap() {
        return hubMap;
    }

    /**
     * The feed shot map.
     *
     * @return the feed shot map
     */
    public ShotMap getFeedMap() {
        return feedMap;
    }

    /**
     * Hub launch parameters at a distance. Kept for the 604 launch helpers below.
     *
     * @param distanceM launcher to hub centre, metres
     * @return the launch parameters
     */
    public LaunchParameter interpolateLaunchEntry(double distanceM) {
        return hubMap.get(distanceM);
    }

    // =========================================================================
    // Shot Parameters Record
    // =========================================================================

    /**
     * Immutable snapshot of all quantities needed to command the turret, hood, and flywheel
     * subsystems for a single shot.
     */
    public record ShootingParameters(
            /** {@code true} when distance is within the shot map's rows. */
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
            /** Ball exit speed from the shot map (m/s), before RPM conversion. */
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

    /**
     * Session-only since 2026-09-19. Chezy QM4 booted with +10 deg of turret trim in flash, the
     * cap, left there by an operator pressing D-pad right at a turret that was parked for a pose it
     * did not have. A turret trim is a correction for one match's pose error, not a calibration, so
     * it starts at zero every boot and never touches {@link Preferences}.
     */
    public static final double STARTING_TURRET_ANGLE_OFFSET = 0; // degrees

    public static double TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;

    /** The two operator trims. */
    public enum TrimAxis {
        HOOD,
        TURRET
    }

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

    /**
     * Preferences key older builds persisted the turret trim under. Nothing writes it any more;
     * {@link #loadPersistedTrims()} removes it so a rio that still has one cannot apply it.
     */
    public static final String TURRET_TRIM_PREF_KEY = "ShotTurretTrimDeg";

    /**
     * Preferences key a flywheel trim persisted under for one match (Chezy Q11, 2026-09-19).
     * Removed the same day: the operator walked it to the +20 % cap and then the -20 % cap inside
     * one match while the shot map itself was wrong, and a second axis moving with every hood press
     * made the presses impossible to read. RPM comes straight from the model again. {@link
     * #loadPersistedTrims()} deletes any stored copy so nothing can read it back.
     */
    public static final String FLYWHEEL_TRIM_PREF_KEY = "ShotFlywheelTrimPct";

    /**
     * Reads the hood trim back off the rio.
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

        // The turret trim is session-only. Drop any copy an older build left in flash so that
        // nothing, including a hand-edited preference, can read it back.
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

        // The flywheel trim lasted one match. Nothing reads it; drop the stored copy.
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

    /** Writes the persisted hood trim. The turret trim is session-only and is never written. */
    private static void writeTrimPreferences() {
        Preferences.setDouble(HOOD_TRIM_PREF_KEY, HOOD_ANGLE_OFFSET);
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
     * @param axis which trim to move
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

    /** Increase turret angle offset. */
    public static Command increaseTurretAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(TrimAxis.TURRET, TURRET_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.increaseTurretTrim");
    }

    /** Decrease turret angle offset. */
    public static Command decreaseTurretAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(TrimAxis.TURRET, -TURRET_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.decreaseTurretTrim");
    }

    /**
     * Zeroes both trims and clears the stored hood trim from flash.
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
     * @param axis which trim moved
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

    // =========================================================================
    // Exit speed to flywheel RPM
    // =========================================================================

    /**
     * Global exit-speed scale factor. Adjust post-characterization to correct for ball compression,
     * wear, or temperature without editing the shot maps. 1.0 = no scaling. Applied to both the hub
     * and feed maps.
     */
    private static final double MPS_FACTOR = 1;

    /**
     * Scale factor converting shot-map exit speed (m/s) to flywheel RPM: the shot's power transfer,
     * expressed as the RPM it costs to put one m/s on the ball.
     *
     * <p>The fitted 365 RPM per m/s against the 4 in wheel is a coupling ratio of 0.515 (ball speed
     * over wheel surface speed), which is the no-slip figure for a single wheel against a fixed
     * hood: the ball rolls, so its centre leaves at half the surface speed and the rest goes into
     * backspin. Grip that is worse than no-slip -- worn wheels, a light squeeze, a cold ball --
     * moves the real ratio below 0.515, and then every shot lands short at the RPM the model asks
     * for.
     *
     * <p>Fudge it here rather than editing every row of the shot map. This is the coupling only:
     * raising it commands more RPM for the same wanted exit speed and leaves the ballistics, the
     * time of flight, and the sim's ball alone. {@link #MPS_FACTOR} is the other knob and means
     * something different -- that the ball really does leave faster than the map says -- so it
     * moves the simulated ball too.
     */
    private static final double RPM_PER_MPS_FITTED = 365.0;

    /**
     * Boot value: the fitted coupling, unchanged.
     *
     * <p>A 6.8 % raise to 390 was drafted after Chezy Q45 to buy half a metre of range everywhere.
     * It never went on the robot, and the practice-field shooting that followed (2026-09-19,
     * evening, at 365) said the opposite: the far shots were landing, and it was the shots inside
     * tower radius that went long. A coupling raise moves every range by {@code dR = 2R * dv/v}, so
     * it would have made the near shots worse to fix a far problem that was not there. The near
     * shots are handled by {@link #nearShotRpmDrop(double)} instead, and this number stays at the
     * fit.
     *
     * <p>Rule of thumb if it does need to move: each 1 % (about 3.7 RPM per m/s) is worth 0.07 m at
     * 3.5 m, and the effect grows with range, so it is the knob for a bias that is the same sign at
     * every distance and biggest far out. It is the wrong knob for a bias at one end of the range.
     */
    private static final double RPM_PER_MPS_DEFAULT = RPM_PER_MPS_FITTED;

    /** Clamp on the live {@link #RPM_PER_MPS_DEFAULT} fudge, +/-20 % of the boot value. */
    private static final double RPM_PER_MPS_MIN = RPM_PER_MPS_DEFAULT * 0.8;

    private static final double RPM_PER_MPS_MAX = RPM_PER_MPS_DEFAULT * 1.2;

    /**
     * Live handle on the coupling, on the dashboard as {@code ShotCalc/RpmPerMps}.
     *
     * <p>Dashboard only and deliberately not a gamepad axis or a {@link Preferences} key: the
     * flywheel trim that {@link #FLYWHEEL_TRIM_PREF_KEY} documents was walked to both caps inside
     * one match and died for it. This is a pit knob that a person types a number into, it starts at
     * {@link #RPM_PER_MPS_DEFAULT} every boot, and {@code ShotCalc/RpmPerMps} is logged every loop
     * so a log says which number a shot was taken at. The clamp moves with the boot value, so it is
     * 292 to 438 at the current default.
     */
    private static final DoubleSubscriber RPM_PER_MPS_TUNE =
            Telemetry.tunable("ShotCalc/RpmPerMps", RPM_PER_MPS_DEFAULT);

    /**
     * The coupling to command with this loop, clamped to +/-20 % of the fitted value.
     *
     * @return RPM per m/s of wanted exit speed
     */
    private static double rpmPerMps() {
        return MathUtil.clamp(
                RPM_PER_MPS_TUNE.get(RPM_PER_MPS_DEFAULT), RPM_PER_MPS_MIN, RPM_PER_MPS_MAX);
    }

    // =========================================================================
    // Near-shot RPM drop -- the shot map correction for inside tower radius
    // =========================================================================

    /**
     * Shape of the near-shot correction: fraction of {@link #NEAR_SHOT_RPM_DROP_DEFAULT} to take
     * off the flywheel command, indexed by distance to the hub in metres. 1.0 at 3.0 m and inside,
     * zero from 3.75 m out, and rising below 2.5 m.
     *
     * <p>Practice-field shooting on 2026-09-19 (at the fitted 365 RPM per m/s) had every hub shot
     * from about tower radius inward landing long, and the far shots landing. Taking 1.5 deg of
     * hood out fixed the near shots and dropped the far ones short, because the model's hood is
     * worth 0.2 m per degree at 2 m and 0.4 m per degree at 3.5 m. The team's preference is to
     * leave the hood alone and take the range out with exit speed, which also keeps the near shot
     * lower.
     *
     * <p>The size comes from converting that 1.5 deg, about 0.25 to 0.3 m of range, into the exit
     * speed change that removes the same range at the model's own hood angle, in vacuum: 4.4 % at
     * 3.0 m, 5.2 % at 2.5 m, 6.8 % at 2.0 m, 10.6 % at 1.5 m. Speed is a weak knob at the steep
     * near angles (84 deg launch at 1.5 m), which is why the shape rises so fast inside 2.5 m.
     * Normalised to the 3.0 m value and rounded; the 3.0 m value at 365 RPM per m/s is about 130
     * RPM by that vacuum sum and about 240 RPM by the model's own hood slope, so the default sits
     * between them. Endpoints hold outside the table, so the hub-face set shot at 0.98 m gets the
     * 1.5 m value.
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
     * Live handle on the near-shot drop, on the dashboard as {@code ShotCalc/NearShotRpmDrop}.
     *
     * <p>Same rules as {@link #RPM_PER_MPS_TUNE}: a pit knob a person types a number into, back to
     * {@link #NEAR_SHOT_RPM_DROP_DEFAULT} every boot, never on the gamepad and never persisted. The
     * value in force and the RPM actually removed are both logged every loop a shot is in progress.
     */
    private static final DoubleSubscriber NEAR_SHOT_RPM_DROP_TUNE =
            Telemetry.tunable("ShotCalc/NearShotRpmDrop", NEAR_SHOT_RPM_DROP_DEFAULT);

    /**
     * RPM to take off a hub shot at the given range.
     *
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
     * fitted {@code velocityKv = 0.1425}). The gearing does not set the ceiling -- the battery
     * does. At the fastest moment ever logged, 4194 RPM in Q45, the motor was applying 9.94 V
     * against a 9.95 V bus: saturated, with 5.5 % of that match's launch samples within a volt of
     * the same wall.
     */
    private static final double RPM_PER_VOLT = 416.5;

    /**
     * Bus voltage to size the ceiling against. Not the 12 V of a resting battery: during a launch
     * burst the measured bus sits between 8 and 10.5 V, and Q45 touched 7.35 V at 402 A. 9.85 V is
     * what a healthy pack held at the top of its range.
     */
    private static final double USABLE_BUS_VOLTS = 9.85;

    /**
     * Fastest flywheel speed worth commanding with the current gearing: about 4100 RPM.
     *
     * <p>Nothing in the normal range reaches this: the model tops out at 3780 RPM at the fitted
     * coupling. It is a backstop for a coupling raise or a longer shot.
     *
     * <p>Asking for more does not make the ball faster, it makes {@link
     * frc.robot.subsystems.launcher.Launcher#isAtSpeed()} unsatisfiable -- that gate is a +/-200
     * RPM window around the command, so a command the flywheel cannot reach keeps the shot-ready
     * gate shut and no fuel feeds. In Q11 every sample commanded above 4000 RPM read not-at-speed.
     * Clamping here means a far shot fires a little short instead of not firing at all.
     *
     * <p>Raise this when the battery situation improves: the number is {@link #RPM_PER_VOLT} times
     * the bus volts a burst actually holds, less a little so the gate's window can close.
     */
    private static final double MAX_FEASIBLE_FLYWHEEL_RPM =
            Math.floor(RPM_PER_VOLT * USABLE_BUS_VOLTS / 50.0) * 50.0;

    /**
     * Caps a wanted flywheel speed at what the hardware can hold.
     *
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

    // =========================================================================
    // Set shot -- the fallback when the pose is gone
    // =========================================================================

    /**
     * The fixed shots, one per parking spot. Each is a range to the hub centre and a turret angle;
     * the hood and flywheel come off the model at that range, at a standstill.
     *
     * <p>Ranges are worked from the field geometry, not measured. Robot centre, and therefore the
     * launcher ({@code robotToLauncher} is zero), sits 15 in inside the bumper of a 30 in robot.
     * The hub centre is the midpoint of tags 26 and 20, (4.626, 4.035) m.
     *
     * <p>The turret's zero points away from the intake, so "intake facing the hub" means the turret
     * turns a half turn to shoot back over it. {@code -180} rather than {@code +180}: the travel is
     * -216 to +180 deg, and a command sitting exactly on the forward soft limit has no margin.
     * Every spot except the hub face parks intake-away, turret at zero: from most of the tracked
     * turret positions that is the short move, so the shot is ready sooner than one that latches a
     * profiled half turn (see {@code Turret.longMoveDegrees}).
     *
     * <p>Forgiving to be off by: the model moves about 0.5 deg of hood and 35 RPM per 15 cm at
     * these ranges, so lining up by eye against the field element is good enough.
     *
     * <p>Hood and flywheel are not stored here: {@code setShotSolution()} reads them off the live
     * hub model, the near-shot RPM drop and the operator's hood trim every loop, so a set shot
     * follows the shot map without anyone retyping numbers. For reference, at the fitted model with
     * the 150 RPM drop and zero trim (2026-09-20): Tower 14.3 deg / 2850 RPM, HubFace 6.1 deg /
     * 2470 RPM, either trench 15.4 deg / 3020 RPM. The logged {@code Hood/CommandedDegrees} and
     * {@code Launcher/CommandedRPM} are the numbers actually in force.
     */
    public enum SetShot {
        /**
         * Parked against the tower's field-facing wall, intake to the wall, turret at zero shooting
         * the hub. Tower front face 43.51 in on the tower centreline (tag 31, y = 3.746 m), so the
         * robot centre is at (1.486, 3.746) m: 3.15 m, 10.3 ft.
         */
        TOWER("Tower", 3.15, 0.0),
        /**
         * Bumper against the hub's near face, intake to the hub, turret over the intake. Hub half
         * width 23.5 in plus 15 in: 0.98 m. That is below the model's fitted 1.5 m floor, so the
         * model is evaluated at 1.5 m and clamped there. Untested at the time of writing; treat it
         * as a lob until the practice field says otherwise.
         */
        HUB_FACE("HubFace", 0.978, -180.0),
        /**
         * Sitting in the left trench lane with the robot just clear of the trench, intake pointed
         * away from the hub, turret at zero shooting back over the far bumper. Trench opening is
         * 50.34 in wide at the wall, so its centreline is 3.395 m from the field centreline; the
         * robot centre is 23.5 + 15 in along x from the hub centre once it has cleared the 47 in
         * trench: 3.53 m. Sitting inside the trench instead is 3.40 m, 13 cm less, which the model
         * barely notices. The robot is 30 in square, so which end faces the hub does not move its
         * centre, and the range is the same as it was intake-to-hub (2026-09-20: turned round so
         * the turret makes no half turn on the way to the shot).
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

    /**
     * Range the tower set shot is fitted for. Kept as the name older notes and logs use; the value
     * is {@link SetShot#TOWER}.
     */
    public static final double SET_SHOT_DISTANCE_METERS = SetShot.TOWER.distanceMeters;

    private static volatile SetShot selectedSetShot = SetShot.TOWER;

    /**
     * Picks which fixed shot the SET_SHOT super state runs. Called from the pilot binding before
     * the state is requested; the selection sticks until the next binding changes it.
     *
     * @param shot the parking spot
     */
    public static void selectSetShot(SetShot shot) {
        selectedSetShot = shot;
        Telemetry.logDashAlways("ShotCalc/SetShot", shot.label);
    }

    /**
     * The fixed shot currently selected.
     *
     * @return the selected shot, {@link SetShot#TOWER} until anything picks one
     */
    public static SetShot getSelectedSetShot() {
        return selectedSetShot;
    }

    /**
     * Hood angle and flywheel speed for the selected {@link SetShot}, at a standstill.
     *
     * <p>Read off the same fitted surface a tracked shot uses, at a fixed distance with zero
     * velocity, so it moves with the model and with the operator's D-pad hood trim instead of being
     * a pair of magic numbers that quietly go stale the next time the model is refitted. It never
     * touches the robot pose, which is the entire point: this is what gets used when the pose is
     * the thing that has failed.
     *
     * @return {@code { hoodDegrees, flywheelRPM }}
     */
    private static double[] setShotSolution() {
        ShotMap map = getInstance().hubMap;
        LaunchParameter shot = map.get(selectedSetShot.distanceMeters);
        double hoodDegrees =
                MathUtil.clamp(
                        (90 - shot.launchAngleDeg()) + map.hoodOffsetDeg() + HOOD_ANGLE_OFFSET,
                        Robot.getHood().getConfig().getMinRotations() * 360.0,
                        Robot.getHood().getConfig().getMaxRotations() * 360.0);
        double rpm =
                shot.launchVelocityMps() * MPS_FACTOR * rpmPerMps()
                        - nearShotRpmDrop(selectedSetShot.distanceMeters);
        return new double[] {hoodDegrees, feasibleFlywheelRPM(rpm)};
    }

    /**
     * Hood angle for the selected set shot, in degrees.
     *
     * @return the commanded hood angle
     */
    public static double getSetShotHoodDegrees() {
        return setShotSolution()[0];
    }

    /**
     * Flywheel speed for the selected set shot, in RPM.
     *
     * @return the commanded flywheel speed
     */
    public static double getSetShotFlywheelRPM() {
        return setShotSolution()[1];
    }

    /**
     * Turret mechanism angle for the selected set shot, in degrees from zero.
     *
     * @return the commanded turret angle
     */
    public static double getSetShotTurretDegrees() {
        return selectedSetShot.turretDegrees;
    }

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
        // Feed and hub shots read separate CSV shot maps.
        ShotMap map = feed ? feedMap : hubMap;

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

        // ── Shot map + 1690 virtual-target solver ─────────────────────────────
        // Returns: { exitSpeed_ms, launchAngle_deg, yawOffset_deg, virtualDist_m, tof_s }
        double[] shot =
                solveVirtualTarget(map, distanceNoLookahead, radialVelocity, tangentialVelocity);
        double exitSpeedMs = shot[0];
        double rawHoodAngle = 90 - shot[1]; // degrees, before HOOD_ANGLE_OFFSET
        double yawOffsetDeg = shot[2];
        double lookaheadDist = shot[3];
        double tofFinal = shot[4];

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
                        rawHoodAngle + map.hoodOffsetDeg() + HOOD_ANGLE_OFFSET,
                        Robot.getHood().getConfig().getMinRotations() * 360.0,
                        Robot.getHood().getConfig().getMaxRotations() * 360.0);

        // ── Flywheel speed: exit speed (m/s) → RPM ──────────────────────────
        // The near-shot drop is a hub-model correction; feed shots are not characterised.
        double nearShotDrop = feed ? 0.0 : nearShotRpmDrop(lookaheadDist);
        double wantedFlywheelSpeed = exitSpeedMs * rpmPerMps() - nearShotDrop;
        double flywheelSpeed = feasibleFlywheelRPM(wantedFlywheelSpeed);

        // Snapshot for the shot record: the five values a burst row needs that
        // ShootingParameters does not carry. Kept here rather than widened into the record because
        // nothing in the control path reads them.
        activeModelName = map.name();
        activeModelHoodOffsetDeg = map.hoodOffsetDeg();
        activeRadialVelocityMs = radialVelocity;
        activeTangentialVelocityMs = tangentialVelocity;
        activeFeedShot = feed;

        // ── Validity ──────────────────────────────────────────────────────────
        boolean isValid = map.inRange(distanceNoLookahead);

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
            Telemetry.log("ShotCalc/RpmPerMps", rpmPerMps(), "RPM per m/s");
            Telemetry.log("ShotCalc/FlywheelWantedRPM", wantedFlywheelSpeed, "RPM");
            Telemetry.log("ShotCalc/NearShotRpmDropApplied", nearShotDrop, "RPM");
            Telemetry.log(
                    "ShotCalc/FlywheelClamped", wantedFlywheelSpeed > MAX_FEASIBLE_FLYWHEEL_RPM);
            Telemetry.log("ShotCalc/RadialVelocityMs", radialVelocity, "m/s");
            Telemetry.log("ShotCalc/TangentialVelocityMs", tangentialVelocity, "m/s");
            Telemetry.logDash("ShotCalc/TimeOfFlight", tofFinal, "seconds");
            Telemetry.log("ShotCalc/FeedShot", feed);
            Telemetry.logDash("ShotCalc/HubShotMap", hubMap.name());
            Telemetry.logDash("ShotCalc/TurretAngleOffsetDegrees", TURRET_ANGLE_OFFSET, "degrees");
            Telemetry.logDash("ShotCalc/HoodAngleOffsetDegrees", HOOD_ANGLE_OFFSET, "degrees");
            Telemetry.logDash("ShotCalc/HoodModelOffsetDegrees", hubMap.hoodOffsetDeg(), "degrees");
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
    // Private — Virtual-Target Solver
    // =========================================================================

    /**
     * 1690 Orbit iterative virtual-target solver.
     *
     * <p>Each pass reads the shot map at the current virtual aim point, takes its time of flight,
     * shifts the aim point by how far the launcher moves during that flight, and repeats until TOF
     * converges. Terminates in ≤ 5 iterations (typically 2–3).
     *
     * @param map the shot map (hub or feed) to read
     * @param distance horizontal distance to goal centre (metres)
     * @param radialVelocity launcher velocity toward/away from goal (m/s); positive = closing on
     *     goal
     * @param tangentialVelocity launcher velocity perpendicular to goal line (m/s)
     * @return {@code double[]} with indices:
     *     <ul>
     *       <li>0 — exit speed (m/s), scaled by {@link #MPS_FACTOR}
     *       <li>1 — launch angle (degrees), straight from the map
     *       <li>2 — yaw offset (degrees); add to static bearing before firing
     *       <li>3 — converged virtual aim distance (metres)
     *       <li>4 — converged time of flight (seconds)
     *     </ul>
     */
    private static double[] solveVirtualTarget(
            ShotMap map, double distance, double radialVelocity, double tangentialVelocity) {
        double vdx = distance; // virtual aim point — radial component (m)
        double vdz = 0.0; // virtual aim point — lateral component (m)
        double tof = 0.0;
        double lead = 0.0; // effective lead time after drag bleed (s)

        for (int iter = 0; iter < 5; iter++) {
            double vDist = Math.sqrt(vdx * vdx + vdz * vdz);
            if (vDist < 0.1) break;

            // Read the map at the virtual point; robot motion is already encoded in the shifted
            // aim point.
            double prevTof = tof;
            tof = map.get(vDist).timeOfFlightSec();
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

        LaunchParameter result = map.get(virtualDist);
        return new double[] {
            result.launchVelocityMps() * MPS_FACTOR, // exitSpeed_ms
            result.launchAngleDeg(), // launchAngle_deg
            yawOffsetDeg, // yaw correction (degrees)
            virtualDist, // converged lookahead distance (m)
            result.timeOfFlightSec() // time of flight at the converged aim point (s)
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

    // =========================================================================
    // LaunchInfo + ballistic/geometry helpers
    // =========================================================================

    public static class LaunchInfo {
        public final double yaw;
        public double yawVelocity;
        public double yawAcceleration;
        public final double elevation;
        public final double launchVelocity;

        public LaunchInfo(
                double yaw,
                double yawVelocity,
                double yawAcceleration,
                double elevation,
                double launchVelocity) {
            this.yaw = yaw;
            this.yawVelocity = yawVelocity;
            this.yawAcceleration = yawAcceleration;
            this.elevation = elevation;
            this.launchVelocity = launchVelocity;
        }
    }

    private static LaunchInfo computeStateAtTime(
            final boolean isFeed,
            final double t,
            final Pose2d robotFieldPose,
            final Pose2d targetPose,
            final ShotCalculator launchCalculator,
            final Twist2d robotVel,
            final Twist2d robotAccel,
            final Twist2d robotFieldVel,
            final Twist2d robotFieldAccel,
            final double imuRoll,
            final double imuPitch) {
        final Twist2d dTwist =
                new Twist2d(
                        robotVel.dx * t + 0.5 * robotAccel.dx * t * t,
                        robotVel.dy * t + 0.5 * robotAccel.dy * t * t,
                        robotVel.dtheta * t + 0.5 * robotAccel.dtheta * t * t);
        final Pose2d pose = robotFieldPose.exp(dTwist);

        final Twist2d vel =
                new Twist2d(
                        robotVel.dx + robotAccel.dx * t,
                        robotVel.dy + robotAccel.dy * t,
                        robotVel.dtheta + robotAccel.dtheta * t);

        final Twist2d fieldVel =
                new Twist2d(
                        robotFieldVel.dx + robotFieldAccel.dx * t,
                        robotFieldVel.dy + robotFieldAccel.dy * t,
                        robotFieldVel.dtheta + robotFieldAccel.dtheta * t);

        return computeInstantaneousLaunchInfo(
                isFeed,
                pose,
                targetPose,
                launchCalculator,
                vel,
                robotAccel,
                fieldVel,
                robotFieldAccel,
                imuRoll,
                imuPitch);
    }

    public static LaunchInfo computeLaunchInfo(
            final boolean isFeed,
            final Pose2d robotFieldPose,
            final Pose2d targetPose,
            final ShotCalculator launchCalculator,
            final Twist2d robotVel,
            final Twist2d robotAccel,
            final Twist2d robotFieldVel,
            final Twist2d robotFieldAccel,
            final double imuRoll,
            final double imuPitch) {
        final double dt = 0.005;

        final LaunchInfo infoMinus =
                computeStateAtTime(
                        isFeed,
                        latencySec - dt,
                        robotFieldPose,
                        targetPose,
                        launchCalculator,
                        robotVel,
                        robotAccel,
                        robotFieldVel,
                        robotFieldAccel,
                        imuRoll,
                        imuPitch);
        final LaunchInfo info0 =
                computeStateAtTime(
                        isFeed,
                        latencySec,
                        robotFieldPose,
                        targetPose,
                        launchCalculator,
                        robotVel,
                        robotAccel,
                        robotFieldVel,
                        robotFieldAccel,
                        imuRoll,
                        imuPitch);
        final LaunchInfo infoPlus =
                computeStateAtTime(
                        isFeed,
                        latencySec + dt,
                        robotFieldPose,
                        targetPose,
                        launchCalculator,
                        robotVel,
                        robotAccel,
                        robotFieldVel,
                        robotFieldAccel,
                        imuRoll,
                        imuPitch);

        info0.yawVelocity = constrainAngleNegPiToPi(infoPlus.yaw - infoMinus.yaw) / (2.0 * dt);
        info0.yawAcceleration =
                (constrainAngleNegPiToPi(infoPlus.yaw - info0.yaw)
                                - constrainAngleNegPiToPi(info0.yaw - infoMinus.yaw))
                        / (dt * dt);

        return info0;
    }

    public static LaunchInfo computeInstantaneousLaunchInfo(
            final boolean isFeed,
            final Pose2d robotFieldPose,
            final Pose2d targetPose,
            final ShotCalculator launchCalculator,
            final Twist2d robotVel,
            final Twist2d robotAccel,
            final Twist2d robotFieldVel,
            final Twist2d robotFieldAccel,
            final double imuRoll,
            final double imuPitch) {
        final Pose2d turretFieldPose = robotFieldPose.transformBy(Transform2d.kZero);
        final Twist2d turretVelocity =
                getVelocityAtOffset(robotFieldPose, Transform2d.kZero, robotFieldVel);

        final Translation2d turretToTarget =
                targetPose.getTranslation().minus(turretFieldPose.getTranslation());
        final double distanceToTarget = turretToTarget.getNorm();
        Telemetry.log("LaunchCalculator/distanceToTarget", distanceToTarget);

        final double v0;
        final double phi0;
        final double theta0 = Math.atan2(turretToTarget.getY(), turretToTarget.getX());
        if (isFeed) {
            phi0 = 35;
            v0 = solveBallisticVelocity(distanceToTarget, -19, phi0).orElse(0.0);
        } else {
            final var fieldRelativeLaunchParams =
                    launchCalculator.interpolateLaunchEntry(distanceToTarget);
            v0 = fieldRelativeLaunchParams.launchVelocityMps();
            phi0 = Math.toRadians(fieldRelativeLaunchParams.launchAngleDeg());
        }

        final double vx_target = v0 * Math.cos(phi0) * Math.cos(theta0);
        final double vy_target = v0 * Math.cos(phi0) * Math.sin(theta0);
        final double vz_target = v0 * Math.sin(phi0);

        final double vx_field = vx_target - turretVelocity.dx;
        final double vy_field = vy_target - turretVelocity.dy;
        final double vz_field = vz_target;

        final double heading = turretFieldPose.getRotation().getRadians();
        final double[] vRel =
                fieldToRobotVelocity(vx_field, vy_field, vz_field, heading, imuRoll, imuPitch);
        final double vx_rel = vRel[0];
        final double vy_rel = vRel[1];
        final double vz_rel = vRel[2];

        final double v_rel = Math.sqrt(vx_rel * vx_rel + vy_rel * vy_rel + vz_rel * vz_rel);
        final double phi_rel = Math.asin(vz_rel / v_rel);
        final double yaw_rel =
                Math.atan2(vy_rel, vx_rel) - turretFieldPose.getRotation().getRadians();

        return new LaunchInfo(yaw_rel, 0, 0, phi_rel, v_rel);
    }

    public static double[] fieldToRobotVelocity(
            final double vxField,
            final double vyField,
            final double vzField,
            final double heading,
            final double roll,
            final double pitch) {
        final double cosH = Math.cos(heading);
        final double sinH = Math.sin(heading);
        final double vfwd = cosH * vxField + sinH * vyField;
        final double vlat = -sinH * vxField + cosH * vyField;
        final double vup = vzField;

        final double cosR = Math.cos(roll);
        final double sinR = Math.sin(roll);
        final double cosP = Math.cos(pitch);
        final double sinP = Math.sin(pitch);
        final double vfwd2 = cosP * vfwd - sinP * vup;
        final double vlat2 = vlat;
        final double vup2 = sinP * vfwd + cosP * vup;
        final double vfwd3 = vfwd2;
        final double vlat3 = cosR * vlat2 + sinR * vup2;
        final double vup3 = -sinR * vlat2 + cosR * vup2;

        return new double[] {cosH * vfwd3 - sinH * vlat3, sinH * vfwd3 + cosH * vlat3, vup3};
    }

    public static double getTargetYaw(final Pose2d robotPose, final Translation2d target) {
        final double dx = target.getX() - robotPose.getX();
        final double dy = target.getY() - robotPose.getY();
        return Math.atan2(dy, dx) - robotPose.getRotation().getRadians();
    }

    public static double getTargetYawVelocity(
            final Pose2d robotPose, final Twist2d robotVelocity, final Translation2d target) {
        final double x = target.getX() - robotPose.getX();
        final double y = target.getY() - robotPose.getY();
        final double xVelocity = robotVelocity.dx;
        final double yVelocity = robotVelocity.dy;
        final double rotationVelocity = robotVelocity.dtheta;
        return ((y * xVelocity) - (x * yVelocity)) / (x * x + y * y) - rotationVelocity;
    }

    public static double getTargetYawAcceleration(
            final Pose2d robotPose,
            final Twist2d robotVelocity,
            final Twist2d robotAcceleration,
            final Translation2d target) {
        final double x = target.getX() - robotPose.getX();
        final double y = target.getY() - robotPose.getY();
        final double xVelocity = robotVelocity.dx;
        final double yVelocity = robotVelocity.dy;
        final double xAcceleration = robotAcceleration.dx;
        final double yAcceleration = robotAcceleration.dy;
        final double rotationAcceleration = robotAcceleration.dtheta;
        final double r2 = x * x + y * y;
        return (((y * xAcceleration) - (x * yAcceleration)) * r2
                                - 2
                                        * ((y * xVelocity) - (x * yVelocity))
                                        * ((x * xVelocity) + (y * yVelocity)))
                        / (r2 * r2)
                - rotationAcceleration;
    }

    public static double constrainAngleNegPiToPi(final double angle) {
        double x = (angle + Math.PI) % (2.0 * Math.PI);
        if (x < 0.0) x += 2.0 * Math.PI;
        return x - Math.PI;
    }

    public static Twist2d getVelocityAtOffset(
            final Pose2d basePose, final Transform2d offset, final Twist2d velocity) {
        final double omega = velocity.dtheta;
        final Translation2d offsetField = offset.getTranslation().rotateBy(basePose.getRotation());
        return new Twist2d(
                velocity.dx + (-omega * offsetField.getY()),
                velocity.dy + (omega * offsetField.getX()),
                velocity.dtheta);
    }

    static OptionalDouble solveBallisticVelocity(
            final double horizontalDistance,
            final double heightDifference,
            final double launchAngle) {
        final double g = 9.81;
        final double cosTheta = Math.cos(launchAngle);
        final double tanTheta = Math.tan(launchAngle);
        final double numerator = g * horizontalDistance * horizontalDistance;
        final double denominator =
                2.0 * cosTheta * cosTheta * (horizontalDistance * tanTheta - heightDifference);
        if (denominator <= 0) return OptionalDouble.empty();
        return OptionalDouble.of(Math.sqrt(numerator / denominator));
    }
}
