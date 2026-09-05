package frc.rebuilt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.rebuilt.targetFactories.HubTargetFactory;
import frc.robot.Robot;
import frc.spectrumLib.telemetry.Telemetry;

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

    public static final double STARTING_TURRET_ANGLE_OFFSET = 0; // degrees
    public static double TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;
    /** Increase hood angle offset. */
    public static Command increaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET += 0.1).ignoringDisable(true);
    }
    /** Decrease hood angle offset. */
    public static Command decreaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET -= 0.1).ignoringDisable(true);
    }
    /** Increase turret angle offset. */
    public static Command increaseTurretAngleOffset() {
        return Commands.runOnce(() -> TURRET_ANGLE_OFFSET += 1).ignoringDisable(true);
    }
    /** Decrease turret angle offset. */
    public static Command decreaseTurretAngleOffset() {
        return Commands.runOnce(() -> TURRET_ANGLE_OFFSET -= 1).ignoringDisable(true);
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
            double[] tofCoeffs) {}

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
                    });

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
                    });

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
                    });

    /**
     * Active hub model. {@link #CEILING_3M_HUB_MODEL} is the one that actually scores; {@link
     * #HUB_MODEL} is the full-field fit and has not yet been made to work. The active model name is
     * logged to {@code ShotCalc/HubPolyModel} — check it before a match.
     *
     * <p>Switched back to the ceiling fit on 2026-09-05 after a day of testing under it. Whatever
     * the full-field model was fitted against, it does not describe this robot: it overshot by one
     * to two feet consistently, and neither the flywheel nor the hood showed a matching bias, which
     * leaves the model itself. Going back to a lofted trajectory under a 3 m ceiling is a real
     * constraint to re-check before trusting this on a field with headroom.
     */
    private static final PolyModel WANTED_HUB_MODEL = CEILING_3M_HUB_MODEL;

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
                        rawHoodAngle + HOOD_ANGLE_OFFSET,
                        Robot.getHood().getConfig().getMinRotations() * 360.0,
                        Robot.getHood().getConfig().getMaxRotations() * 360.0);

        // ── Flywheel speed: exit speed (m/s) → RPM ───────────────────────────
        double flywheelSpeed = exitSpeedMs * RPM_PER_MPS;

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

        Telemetry.log("ShotCalc/LookaheadPose", lookaheadPose);
        Telemetry.log("ShotCalc/DistanceMeters", lookaheadDist, "meters");
        Telemetry.log("ShotCalc/DistanceNoLookahead", distanceNoLookahead, "meters");
        Telemetry.log("ShotCalc/TurretAngleDeg", turretAngle.getDegrees(), "degrees");
        Telemetry.log("ShotCalc/YawOffsetDeg", yawOffsetDeg, "degrees");
        Telemetry.log("ShotCalc/HoodAngleDeg", hoodAngle, "degrees");
        Telemetry.log("ShotCalc/FlywheelSpeedRPM", flywheelSpeed, "RPM");
        Telemetry.log("ShotCalc/ExitSpeedMs", exitSpeedMs, "m/s");
        Telemetry.log("ShotCalc/RadialVelocityMs", radialVelocity, "m/s");
        Telemetry.log("ShotCalc/TangentialVelocityMs", tangentialVelocity, "m/s");
        Telemetry.log("ShotCalc/TimeOfFlight", tofFinal, "seconds");
        Telemetry.log("ShotCalc/FeedShot", feed);
        Telemetry.log("ShotCalc/HubPolyModel", WANTED_HUB_MODEL.name);
        Telemetry.log("ShotCalc/TurretAngleOffsetDegrees", TURRET_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotCalc/HoodAngleOffsetDegrees", HOOD_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotCalc/Target", target);

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
