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
     * Immutable snapshot of all quantities needed to command the drive, hood, and flywheel
     * subsystems for a single shot.
     */
    public record ShootingParameters(
            /** {@code true} when distance is within the polynomial's fitted range. */
            boolean isValid,
            /** Field-relative heading the robot must face to aim at the goal. */
            Rotation2d driveAngle,
            /** Rate of change of {@code driveAngle} (rad/s) for heading feedforward. */
            double driveAngularVelocity,
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

    public static final double STARTING_DRIVE_ANGLE_OFFSET = 0; // degrees
    public static double DRIVE_ANGLE_OFFSET = STARTING_DRIVE_ANGLE_OFFSET;

    public static Command increaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET += 0.1).ignoringDisable(true);
    }

    public static Command decreaseHoodAngleOffset() {
        return Commands.runOnce(() -> HOOD_ANGLE_OFFSET -= 0.1).ignoringDisable(true);
    }

    public static Command increaseDriveAngleOffset() {
        return Commands.runOnce(() -> DRIVE_ANGLE_OFFSET += 1).ignoringDisable(true);
    }

    public static Command decreaseDriveAngleOffset() {
        return Commands.runOnce(() -> DRIVE_ANGLE_OFFSET -= 1).ignoringDisable(true);
    }

    // =========================================================================
    // Polynomial Model — Generated 2026-05-27
    // =========================================================================
    // 2D degree-3 polynomial surface:
    //   f(distance_m, radialVel_ms) → { exitSpeed_ms, launchAngle_deg }
    // Monomial basis: 1, d, v, d², d·v, v², d³, d²·v, d·v², v³

    /**
     * Global exit-speed scale factor. Adjust post-characterization to correct for ball compression,
     * wear, or temperature without re-fitting the polynomial. 1.0 = no scaling.
     */
    private static final double MPS_FACTOR = 0.86;

    /**
     * Fitted distance range (metres). Inputs to the polynomial are clamped here; shots outside this
     * range are flagged {@code isValid = false}.
     */
    private static final double DIST_MIN = 1.5;

    private static final double DIST_MAX = 8.0;

    /**
     * Fitted radial-velocity range (m/s). Inputs are clamped; the polynomial is only reliable
     * within the training data envelope.
     */
    private static final double RV_MIN = -3.0;

    private static final double RV_MAX = 3.0;

    /**
     * Scale factor converting polynomial exit speed (m/s) to flywheel RPM.
     *
     * <p>Tune this constant so mid-distance shots (3–5 m) land in the 2000–2500 RPM band. At 4 m
     * the polynomial yields ~8.64 m/s; 260 RPM/(m/s) × 8.64 ≈ 2246 RPM.
     */
    private static final double RPM_PER_MPS = 260.0;

    /** Polynomial coefficients for ball exit speed (m/s). */
    private static final double[] SPEED_COEFFS = {
        /* 1    */ 5.3893753991e+0,
        /* d    */ 7.6770669910e-1,
        /* v    */ -9.1413239448e-1,
        /* d²   */ 1.3055521448e-2,
        /* d·v  */ 9.7274725618e-2,
        /* v²   */ 2.8263559613e-2,
        /* d³   */ -4.8010121843e-4,
        /* d²·v */ -9.0509996412e-3,
        /* d·v² */ -8.6995503921e-4,
        /* v³   */ -2.5418343175e-3
    };

    /** Polynomial coefficients for launch angle (degrees). */
    private static final double[] ANGLE_COEFFS = {
        /* 1    */ 7.6359986384e+1,
        /* d    */ -5.2935902069e+0,
        /* v    */ 8.0286655180e-1,
        /* d²   */ 3.7188988633e-1,
        /* d·v  */ 1.0699214272e+0,
        /* v²   */ -8.1720220665e-1,
        /* d³   */ -2.1747714773e-2,
        /* d²·v */ -9.6406498556e-2,
        /* d·v² */ 1.5434678111e-1,
        /* v³   */ -7.2395347174e-2
    };

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

    private final LinearFilter driveAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100 ms window

    private double lastHoodAngle = Double.NaN;
    private Rotation2d lastDriveAngle = null;

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
     *   <li>Derive the drive angle, hood angle, and flywheel RPM from the result.
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
        boolean feed = Robot.getSuperStructure().robotInFeedZone().getAsBoolean();
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();

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
                        + fieldVelocity.omegaRadiansPerSecond
                                * (robotToLauncher.getY() * Math.cos(robotAngle)
                                        - robotToLauncher.getX() * Math.sin(robotAngle));
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
        double[] poly = solveVirtualTarget(distanceNoLookahead, radialVelocity, tangentialVelocity);
        double exitSpeedMs = poly[0];
        double rawHoodAngle = 90 - poly[1]; // degrees, before HOOD_ANGLE_OFFSET
        double yawOffsetDeg = poly[2];
        double lookaheadDist = poly[3];
        double tofFinal = poly[4];

        // ── Drive angle: static bearing + shoot-on-move yaw + user offset ────
        Rotation2d driveAngle =
                launcherToTarget
                        .getAngle()
                        .plus(Rotation2d.fromDegrees(yawOffsetDeg))
                        .plus(Rotation2d.fromDegrees(DRIVE_ANGLE_OFFSET))
                        .plus(Rotation2d.k180deg);

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
                        driveAngle);

        // Drive angular velocity (rad/s) for heading feedforward
        if (lastDriveAngle == null) lastDriveAngle = driveAngle;
        double deltaRot =
                MathUtil.inputModulus(driveAngle.minus(lastDriveAngle).getRotations(), -0.5, 0.5);
        double driveAngularVelocity = driveAngleFilter.calculate(deltaRot / LOOP_PERIOD_SECS);
        lastDriveAngle = driveAngle;

        // ── Hood angle + velocity ─────────────────────────────────────────────
        // Compute velocity on the raw (un-offset) angle so HOOD_ANGLE_OFFSET (a
        // near-constant) does not bleed into the derivative.
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = rawHoodAngle;
        double hoodVelocity =
                hoodAngleFilter.calculate((rawHoodAngle - lastHoodAngle) / LOOP_PERIOD_SECS);
        lastHoodAngle = rawHoodAngle;
        double hoodAngle = rawHoodAngle + HOOD_ANGLE_OFFSET;

        // ── Flywheel speed: exit speed (m/s) → RPM ───────────────────────────
        double flywheelSpeed = exitSpeedMs * RPM_PER_MPS;

        // ── Validity ──────────────────────────────────────────────────────────
        boolean isValid = distanceNoLookahead >= DIST_MIN && distanceNoLookahead <= DIST_MAX;

        latestParameters =
                new ShootingParameters(
                        isValid,
                        driveAngle,
                        driveAngularVelocity,
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
        Telemetry.log("ShotCalc/DriveAngleDeg", driveAngle.getDegrees(), "degrees");
        Telemetry.log("ShotCalc/YawOffsetDeg", yawOffsetDeg, "degrees");
        Telemetry.log("ShotCalc/HoodAngleDeg", hoodAngle, "degrees");
        Telemetry.log("ShotCalc/FlywheelSpeedRPM", flywheelSpeed, "RPM");
        Telemetry.log("ShotCalc/ExitSpeedMs", exitSpeedMs, "m/s");
        Telemetry.log("ShotCalc/RadialVelocityMs", radialVelocity, "m/s");
        Telemetry.log("ShotCalc/TangentialVelocityMs", tangentialVelocity, "m/s");
        Telemetry.log("ShotCalc/TimeOfFlight", tofFinal, "seconds");
        Telemetry.log("ShotCalc/DriveAngleOffsetDegrees", DRIVE_ANGLE_OFFSET, "degrees");
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
            double distance, double radialVelocity, double tangentialVelocity) {
        double vdx = distance; // virtual aim point — radial component (m)
        double vdz = 0.0; // virtual aim point — lateral component (m)
        double tof = 0.0;

        for (int iter = 0; iter < 5; iter++) {
            double vDist = Math.sqrt(vdx * vdx + vdz * vdz);
            if (vDist < 0.1) break;

            // Evaluate polynomial at virtual point with rv = 0 (robot motion is
            // already encoded in the shifted aim point)
            double[] raw = evalPolyRaw(vDist, 0.0);
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

        double[] result = evalPolyRaw(virtualDist, 0.0);
        return new double[] {
            result[0] * MPS_FACTOR, // exitSpeed_ms
            result[1], // launchAngle_deg
            yawOffsetDeg, // yaw correction (degrees)
            virtualDist, // converged lookahead distance (m)
            tof // converged time of flight (s)
        };
    }

    /**
     * Evaluates the raw polynomial surface at ({@code distance}, {@code radialVel}). Both inputs
     * are clamped to the fitted data range before evaluation.
     *
     * @param distance horizontal distance to goal (metres)
     * @param radialVel robot velocity toward goal (m/s)
     * @return {@code double[]} { rawExitSpeed_ms, rawLaunchAngle_deg }
     */
    private static double[] evalPolyRaw(double distance, double radialVel) {
        double d = Math.max(DIST_MIN, Math.min(DIST_MAX, distance));
        double v = Math.max(RV_MIN, Math.min(RV_MAX, radialVel));

        double d2 = d * d;
        double v2 = v * v;
        double d3 = d2 * d;
        double v3 = v2 * v;

        // Monomial basis: 1, d, v, d², d·v, v², d³, d²·v, d·v², v³
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

        double exitSpeed = 0.0, launchAngle = 0.0;
        for (int i = 0; i < terms.length; i++) {
            exitSpeed += SPEED_COEFFS[i] * terms[i];
            launchAngle += ANGLE_COEFFS[i] * terms[i];
        }
        return new double[] {exitSpeed, launchAngle};
    }
}
