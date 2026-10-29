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
    // Shots come from distance-indexed CSVs in src/main/deploy, interpolated between rows and held
    // at the ends. Edit the CSV and redeploy to change a shot. Columns (any order):
    //   distance_m          launcher to target centre (m)
    //   optimal_v_mps       ball exit speed (m/s)
    //   optimal_angle_deg   launch angle above horizontal (deg); hood = 90 - this
    //   tof_s               optional time of flight (s); defaults to d / (v cos θ)

    public static final String kDISTANCE_COLUMN_LABEL = "distance_m";
    public static final String kVELOCITY_COLUMN_STRING = "optimal_v_mps";
    public static final String kANGLE_COLUMN_LABEL = "optimal_angle_deg";
    public static final String kTOF_COLUMN_LABEL = "tof_s";

    /** Hub shot map. Sampled from the retired 3 m ceiling polynomial (2026-10-29). */
    public static final String HUB_SHOT_MAP_FILE = "lab_shot_map.csv";

    /** Feed shot map. Sampled from the retired feed polynomial; never calibrated. */
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
         * @param hoodOffsetDeg hood trim for this map only (deg); prefer fixing the CSV. The
         *     operator's D-pad trim is added on top.
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

    /** Hub shot map; name logged to {@code ShotCalc/HubShotMap}. */
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

    /** Degrees per hood D-pad press, about 3 in of range. (0.1 needed ~40 presses.) */
    public static final double HOOD_OFFSET_STEP_DEG = 0.25;

    /** Degrees per operator D-pad press on the turret trim. */
    public static final double TURRET_OFFSET_STEP_DEG = 1.0;

    /**
     * Turret trim is session-only: zero every boot, never saved. It corrects one match's pose
     * error; Chezy QM4 booted with a stale +10 deg saved trim (2026-09-19).
     */
    public static final double STARTING_TURRET_ANGLE_OFFSET = 0; // degrees

    public static double TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;

    /** The two operator trims. */
    public enum TrimAxis {
        HOOD,
        TURRET
    }

    /**
     * Max trim either way (deg). Ten degrees of hood is ~10 ft of range, far past any real
     * correction; the cap guards against a corrupt saved value.
     */
    public static final double MAX_TRIM_DEG = 10.0;

    /**
     * Preferences key for the saved hood trim. No slash: a slash makes a sub-table that {@code
     * Preferences.getKeys()} skips. Public so tests and the robot app read the same key.
     */
    public static final String HOOD_TRIM_PREF_KEY = "ShotHoodTrimDeg";

    /** Old saved turret trim key; deleted at boot by {@link #loadPersistedTrims()}. */
    public static final String TURRET_TRIM_PREF_KEY = "ShotTurretTrimDeg";

    /**
     * Old flywheel trim key, deleted at boot. Removed after Chezy Q11 (2026-09-19): it was pushed
     * to both caps in one match and made hood presses impossible to read.
     */
    public static final String FLYWHEEL_TRIM_PREF_KEY = "ShotFlywheelTrimPct";

    /**
     * Loads the saved hood trim; call once at robot construction. The trim survives redeploys and
     * power cycles, so it is printed at boot and cleared with operator Start+Select.
     */
    public static void loadPersistedTrims() {
        Preferences.initDouble(HOOD_TRIM_PREF_KEY, STARTING_HOOD_ANGLE_OFFSET);

        // Turret trim is session-only; drop any copy an older build saved.
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

        // Flywheel trim is retired; drop any saved copy.
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

    /** Saves the hood trim (the turret trim is never saved). */
    private static void writeTrimPreferences() {
        Preferences.setDouble(HOOD_TRIM_PREF_KEY, HOOD_ANGLE_OFFSET);
    }

    /**
     * Applies a trim nudge and logs it as the last shot's outcome: hood up = short, hood down =
     * long, turret = which side it missed. No press = made. See {@code docs/tools/shot-log.md}.
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
            // Flash write: fine once per press, never from a periodic.
            writeTrimPreferences();
        } else {
            before = TURRET_ANGLE_OFFSET;
            after = MathUtil.clamp(before + delta, -MAX_TRIM_DEG, MAX_TRIM_DEG);
            TURRET_ANGLE_OFFSET = after;
        }
        logTrimEvent(axis, after - before, after, false);
    }

    /** Raise the hood trim: last shot was short. */
    public static Command increaseHoodAngleOffset() {
        return Commands.runOnce(() -> nudgeTrim(TrimAxis.HOOD, HOOD_OFFSET_STEP_DEG))
                .ignoringDisable(true)
                .withName("ShotCalculator.increaseHoodTrim");
    }

    /** Lower the hood trim: last shot was long. */
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
     * Zeroes both trims and the saved hood trim. Bound to a two-button chord so it can't be hit by
     * accident.
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
    // wpilog-only, one row per event (see docs/tools/shot-log.md):
    //   ShotCalc/Shot/*  when the feed gate opens: what was aimed
    //   ShotCalc/Trim/*  per D-pad press: how it went
    // DogLog skips unchanged values, so read each key's last value at or before the row's time.
    // Balls per burst come from the dips in Launcher/RPM.

    /** Bursts since boot; pairs a shot row with the trim row that judges it. */
    private static long shotIndex = 0;

    /** FPGA time of the last burst, or NaN before the first one. */
    private static double lastShotTimestampSeconds = Double.NaN;

    /** Distance of the last burst, copied into trim rows. */
    private static double lastShotDistanceMeters = Double.NaN;

    /** D-pad presses since boot. */
    private static long trimEventIndex = 0;

    // Shot-row values not in ShootingParameters; set by getParameters().
    private static String activeModelName = "none";
    private static double activeModelHoodOffsetDeg = 0;
    private static double activeRadialVelocityMs = 0;
    private static double activeTangentialVelocityMs = 0;
    private static boolean activeFeedShot = false;

    /**
     * Logs one row for the burst starting now (feed gate rising edge): commanded vs actual. The
     * outcome comes later as a trim press.
     *
     * @param poseTrusted whether vision is recent enough to trust the distance
     */
    public static void recordShot(boolean poseTrusted) {
        ShootingParameters params = getInstance().getParameters();
        double now = Timer.getFPGATimestamp();

        shotIndex++;
        lastShotTimestampSeconds = now;
        lastShotDistanceMeters = params.distanceNoLookahead();

        // On NetworkTables so the operator can see records being written.
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

        // Actuals; NaN if a mechanism doesn't exist yet (sim/bench).
        Telemetry.log(
                "ShotCalc/Shot/ActualRPM",
                Robot.getLauncher() == null ? Double.NaN : Robot.getLauncher().getVelocityRPM(),
                "RPM");
        Telemetry.log(
                "ShotCalc/Shot/ActualHoodDeg",
                Robot.getHood() == null ? Double.NaN : Robot.getHood().getPositionDegrees(),
                "degrees");
        // Measured minus commanded (Turret/PositionError has the opposite sign). Treat as a
        // magnitude; the real-world direction of positive is unverified.
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
     * Logs one row per trim press, paired to the last burst by {@code ShotIndex} and {@code
     * SecondsSinceShot}. With no prior burst: index -1, infinite age.
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
            // At the cap; still a verdict, so keep the row.
            verdict = "AtLimit";
        } else if (axis == TrimAxis.HOOD) {
            verdict = delta > 0 ? "Short" : "Long";
        } else {
            // A CCW correction means the ball landed CW.
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
     * Scales every map's exit speed (1.0 = none). Use when the ball really leaves faster/slower
     * than the map says; it also moves the sim ball.
     */
    private static final double MPS_FACTOR = 1;

    /**
     * Flywheel RPM per m/s of exit speed. 365 on the 4 in wheel = 0.515 ball/surface speed, the
     * no-slip figure; worse grip (worn wheels, cold ball) makes every shot short. Changes RPM only,
     * not the ballistics or sim ball.
     */
    private static final double RPM_PER_MPS_FITTED = 365.0;

    /**
     * Boot coupling. Each +1 % adds ~0.07 m at 3.5 m, more farther out: use it for a bias that is
     * the same at all ranges, not near-only (that's {@link #nearShotRpmDrop}). A raise to 390 was
     * rejected on 2026-09-19 because far shots were already landing.
     */
    private static final double RPM_PER_MPS_DEFAULT = RPM_PER_MPS_FITTED;

    /** Live coupling limits, ±20 % of default (292–438). */
    private static final double RPM_PER_MPS_MIN = RPM_PER_MPS_DEFAULT * 0.8;

    private static final double RPM_PER_MPS_MAX = RPM_PER_MPS_DEFAULT * 1.2;

    /**
     * Pit knob {@code ShotCalc/RpmPerMps}: dashboard only, resets every boot, logged per loop.
     * Never on the gamepad (see {@link #FLYWHEEL_TRIM_PREF_KEY}).
     */
    private static final DoubleSubscriber RPM_PER_MPS_TUNE =
            Telemetry.tunable("ShotCalc/RpmPerMps", RPM_PER_MPS_DEFAULT);

    /**
     * Current coupling, clamped.
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
     * Near-shot RPM drop, as a fraction of {@link #NEAR_SHOT_RPM_DROP_DEFAULT} by hub distance (m):
     * 1.0 at 3.0 m, 0 from 3.75 m, rising inside 2.5 m where speed is a weak knob at steep angles.
     *
     * <p>On 2026-09-19 near shots (inside tower radius) landed ~0.3 m long while far shots landed.
     * Taking out hood fixed near but broke far, so the range comes out of exit speed instead. Shape
     * is the vacuum speed change for 0.3 m, normalised to 3.0 m. Held at the ends.
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

    /** Hub RPM drop at 3.0 m before shape scaling. Best guess (130–240 estimated); tune ±50–100. */
    private static final double NEAR_SHOT_RPM_DROP_DEFAULT = 150.0;

    /** Max live near-shot drop (~15 % at 3 m). Zero disables it. */
    private static final double NEAR_SHOT_RPM_DROP_MAX = 400.0;

    /** Pit knob {@code ShotCalc/NearShotRpmDrop}; same rules as {@link #RPM_PER_MPS_TUNE}. */
    private static final DoubleSubscriber NEAR_SHOT_RPM_DROP_TUNE =
            Telemetry.tunable("ShotCalc/NearShotRpmDrop", NEAR_SHOT_RPM_DROP_DEFAULT);

    /**
     * RPM to take off a hub shot at the given range.
     *
     * @param distanceMeters launcher to hub centre
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

    /** Measured flywheel RPM per applied volt (median of 8 Chezy logs, 407–430). */
    private static final double RPM_PER_VOLT = 416.5;

    /** Bus voltage a healthy battery holds during a launch burst (8–10.5 V measured). */
    private static final double USABLE_BUS_VOLTS = 9.85;

    /**
     * Highest reachable flywheel speed (~4100 RPM); the battery is the limit. Above it {@code
     * Launcher.isAtSpeed()} never passes and nothing feeds (Q11), so clamp and shoot a little short
     * instead. The hub map tops out near 3620 RPM.
     */
    private static final double MAX_FEASIBLE_FLYWHEEL_RPM =
            Math.floor(RPM_PER_VOLT * USABLE_BUS_VOLTS / 50.0) * 50.0;

    /**
     * Caps a flywheel speed at {@link #MAX_FEASIBLE_FLYWHEEL_RPM}.
     *
     * @param wantedRPM wanted flywheel speed
     * @return the capped speed
     */
    private static double feasibleFlywheelRPM(double wantedRPM) {
        return Math.min(wantedRPM, MAX_FEASIBLE_FLYWHEEL_RPM);
    }

    /**
     * Drag loss per second of inherited robot velocity. Raise if shoot-on-move over-leads, lower if
     * it under-leads.
     */
    private static final double LEAD_DRAG_BLEED_PER_SEC = 0.085;

    // =========================================================================
    // Set shot -- the fallback when the pose is gone
    // =========================================================================

    /**
     * Fixed shots for when the pose is lost: a hub distance and turret angle per parking spot. Hood
     * and RPM come from the hub map at that distance (plus near-shot drop and trim) every loop.
     *
     * <p>Distances are from field geometry: robot centre = launcher, 15 in inside the bumper; hub
     * centre (4.626, 4.035) m. Being off by 15 cm is only ~0.5 deg / 35 RPM. Turret zero faces away
     * from the intake; use -180 not +180 (travel is -216 to +180).
     */
    public enum SetShot {
        /** Intake against the tower's front wall, turret at zero. 3.15 m. */
        TOWER("Tower", 3.15, 0.0),
        /**
         * Bumper on the hub's near face, shooting back over the intake. 0.98 m, below the map, so
         * it uses the 1.5 m row. Untested.
         */
        HUB_FACE("HubFace", 0.978, -180.0),
        /**
         * Left trench lane just clear of the trench, intake away from the hub, turret at zero. 3.53
         * m (3.40 m inside the trench, close enough).
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

    /** Old name for {@link SetShot#TOWER}'s distance. */
    public static final double SET_SHOT_DISTANCE_METERS = SetShot.TOWER.distanceMeters;

    private static volatile SetShot selectedSetShot = SetShot.TOWER;

    /**
     * Picks the fixed shot SET_SHOT uses. Sticks until changed.
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
     * Hood and RPM for the selected {@link SetShot}, from the hub map. Never uses the pose.
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

        // ── Radial (+ = closing on target) and tangential velocity ───────────
        double ux = launcherToTarget.getX() / distanceNoLookahead;
        double uy = launcherToTarget.getY() / distanceNoLookahead;
        double radialVelocity = launcherVelocityX * ux + launcherVelocityY * uy;
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

        // ── Lookahead pose: launcher position when the ball arrives (for viz) ──
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
        // Velocity uses the raw angle so trim changes don't spike it.
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
        // Near-shot drop is hub-only.
        double nearShotDrop = feed ? 0.0 : nearShotRpmDrop(lookaheadDist);
        double wantedFlywheelSpeed = exitSpeedMs * rpmPerMps() - nearShotDrop;
        double flywheelSpeed = feasibleFlywheelRPM(wantedFlywheelSpeed);

        // For recordShot().
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

        // Every loop while launching, 10 Hz otherwise.
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
     * <p>Reads the map's time of flight at the aim point, shifts the aim point by robot motion over
     * that time, and repeats until TOF converges (≤ 5 passes).
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
     * Fraction of inherited robot velocity that survives drag to the target (sim: 0.89 / 0.86 /
     * 0.82 at 4 / 6 / 8 m).
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
