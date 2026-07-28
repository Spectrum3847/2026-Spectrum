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
            /** Rate of change of {@code turretAngle} (rad/s) for heading feedforward. */
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

    public static final double STARTING_HOOD_ANGLE_OFFSET = -1; // degrees
    public static double HOOD_ANGLE_OFFSET = STARTING_HOOD_ANGLE_OFFSET;

    public static final double STARTING_TURRET_ANGLE_OFFSET = 0; // degrees
    public static double TURRET_ANGLE_OFFSET = STARTING_TURRET_ANGLE_OFFSET;

    public static Command increaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET += 0.1).ignoringDisable(true);
    }

    public static Command decreaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET -= 0.1).ignoringDisable(true);
    }

    public static Command increaseTurretAngleOffset() {
        return Commands.runOnce(() -> TURRET_ANGLE_OFFSET += 1).ignoringDisable(true);
    }

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
    private static final double RPM_PER_MPS = 255.0;

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
            double[] angleCoeffs) {}

    /** Hub-shot model — used when the robot is in a scoring zone. */
    private static final PolyModel NO_CEILING_HUB_MODEL =
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
                        /* 1    */ 9.4808087843e+0,
                        /* d    */ 1.5863141017e+0,
                        /* v    */ -1.7719538139e+0,
                        /* d²   */ 1.5935526957e-1,
                        /* d·v  */ -7.7598049499e-2,
                        /* v²   */ -6.1476953429e-2,
                        /* d³   */ -5.3055707891e-2,
                        /* d²·v */ -1.5105348088e-1,
                        /* d·v² */ 1.0029855931e-1,
                        /* v³   */ 2.0939312404e-1
                    },
                    new double[] {
                        /* 1    */ 6.7125256559e+1,
                        /* d    */ -1.9123086140e+0,
                        /* v    */ 5.5563652874e+0,
                        /* d²   */ 1.5494062236e+0,
                        /* d·v  */ -8.5813952802e-1,
                        /* v²   */ -7.8644976757e-1,
                        /* d³   */ -6.7442518882e-1,
                        /* d²·v */ -5.6684392430e-1,
                        /* d·v² */ 1.0834081188e+0,
                        /* v³   */ 6.3724546139e-1
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
                        /* 1    */ 8.9009110528e+0,
                        /* d    */ 1.3529080973e+0,
                        /* v    */ -1.0940426893e+0,
                        /* d²   */ -1.0603732127e-1,
                        /* d·v  */ -8.0783506354e-4,
                        /* v²   */ 4.1114510233e-2,
                        /* d³   */ -1.0472409654e-1,
                        /* d²·v */ -3.1049132730e-2,
                        /* d·v² */ 8.8698302579e-2,
                        /* v³   */ 1.1306215407e-2
                    },
                    new double[] {
                        /* 1    */ 6.2065694833e+1,
                        /* d    */ -5.0007714147e+0,
                        /* v    */ 1.0770747163e+1,
                        /* d²   */ -5.8756872976e-1,
                        /* d·v  */ -5.6652782738e-1,
                        /* v²   */ 1.5707900653e-1,
                        /* d³   */ -1.2431833294e+0,
                        /* d²·v */ -8.6287641206e-1,
                        /* d·v² */ 1.6030720931e+0,
                        /* v³   */ -5.1017644668e-1
                    });

    /** Feed-shot model — used when the robot is in a feed zone. */
    private static final PolyModel FEED_MODEL =
            new PolyModel(
                    "Feed Model",
                    5.0, // distMin (m)
                    10.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    7.5000000000, // dMean
                    1.5430334996, // dStd
                    0.0000000000, // vMean
                    2.0000000000, // vStd
                    new double[] {
                        /* 1    */ 8.8895297764e+0,
                        /* d    */ 1.1819083840e+0,
                        /* v    */ -1.4689777292e+0,
                        /* d²   */ -7.2679959558e-2,
                        /* d·v  */ -9.4108087065e-2,
                        /* v²   */ 2.5641025641e-1,
                        /* d³   */ -4.5671770573e-2,
                        /* d²·v */ -3.7127276946e-2,
                        /* d·v² */ -1.4130343403e-2,
                        /* v³   */ 1.7094017094e-2
                    },
                    new double[] {
                        /* 1    */ 4.4556592644e+1,
                        /* d    */ 1.1495781344e+0,
                        /* v    */ 2.1597654794e+0,
                        /* d²   */ -7.2910546467e-1,
                        /* d·v  */ -9.7880549625e-1,
                        /* v²   */ 2.2580512821e+0,
                        /* d³   */ -4.6161906648e-1,
                        /* d²·v */ -7.2988513749e-1,
                        /* d·v² */ -5.8568803850e-1,
                        /* v³   */ 6.7391452991e-1
                    });

    private static final PolyModel WANTED_HUB_MODEL = NO_CEILING_HUB_MODEL;

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

        // Turret angular velocity (rad/s) for heading feedforward
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
        double hoodAngle = Math.max(rawHoodAngle + HOOD_ANGLE_OFFSET, 9);

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
     * <p>Each pass evaluates the polynomial at the current virtual aim point, estimates
     * time-of-flight from horizontal kinematics, shifts the aim point by how far the launcher moves
     * during that flight, and repeats until TOF converges. Terminates in ≤ 5 iterations (typically
     * 2–3).
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

        for (int iter = 0; iter < 5; iter++) {
            double vDist = Math.sqrt(vdx * vdx + vdz * vdz);
            if (vDist < 0.1) break;

            // Evaluate polynomial at virtual point with rv = 0 (robot motion is
            // already encoded in the shifted aim point)
            double[] raw = evalPolyRaw(model, vDist, 0.0);
            double speed = raw[0] * MPS_FACTOR;
            double cosA = Math.cos(raw[1] * Math.PI / 180.0);
            double prevTof = tof;
            tof = vDist / Math.max(speed * cosA, 0.5); // guard against div-by-zero

            // Shift aim point: where the target will be relative to the launcher
            // when the ball arrives
            vdx = distance - radialVelocity * tof;
            vdz = -tangentialVelocity * tof;

            if (iter > 0 && Math.abs(tof - prevTof) < 0.002) break;
        }

        double virtualDist = Math.sqrt(vdx * vdx + vdz * vdz);
        double yawOffsetDeg =
                Math.atan2(-tangentialVelocity * tof, distance - radialVelocity * tof)
                        * (180.0 / Math.PI);

        double[] result = evalPolyRaw(model, virtualDist, 0.0);
        return new double[] {
            result[0] * MPS_FACTOR, // exitSpeed_ms
            result[1], // launchAngle_deg
            yawOffsetDeg, // yaw correction (degrees)
            virtualDist, // converged lookahead distance (m)
            tof // converged time of flight (s)
        };
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
     * @return double[] { exitSpeed_ms (raw, before MPS_FACTOR), launchAngle_deg }
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
        double exitSpeed = 0.0, launchAngle = 0.0;
        for (int i = 0; i < terms.length; i++) {
            exitSpeed += speedCoeffs[i] * terms[i];
            launchAngle += angleCoeffs[i] * terms[i];
        }
        return new double[] {exitSpeed, launchAngle};
    }
}
