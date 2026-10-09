package frc.rebuilt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.rebuilt.targetFactories.FeedTargetFactory;
import frc.rebuilt.targetFactories.HubTargetFactory;
import frc.robot.Robot;
import frc.spectrumLib.telemetry.Telemetry;

@SuppressWarnings("unused")
public class ShotCalculator {

    private static ShotCalculator instance;

    /** Launcher offset in the robot frame. Zero puts the launcher at the robot centre. */
    private static final Transform2d robotToLauncher = Transform2d.kZero;

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

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

    public static final double STARTING_HOOD_ANGLE_OFFSET = -2; // degrees
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

    /**
     * Global exit-speed scale factor for both the hub and feed models. 1.0 leaves the fitted
     * polynomial unscaled; raise it to correct for ball compression, wear, or temperature.
     */
    private static final double MPS_FACTOR = 0.8;

    /** Scale factor converting polynomial exit speed (m/s) to flywheel RPM. */
    private static final double RPM_PER_MPS = 255.0;

    /**
     * A fitted degree-3 polynomial surface plus its input domain and normalisation. Inputs are
     * mapped to zero-mean unit-variance before evaluation, so the coefficients are only valid for
     * normalised inputs.
     *
     * <p>Coefficient order is the monomial basis 1, d, v, d², d·v, v², d³, d²·v, d·v², v³, which
     * has to match the expansion in {@link #evalPolyRaw}.
     *
     * @param name label shown on the dashboard
     * @param distMin fitted distance lower bound (metres); evalPolyRaw clamps to it
     * @param distMax fitted distance upper bound (metres)
     * @param rvMin fitted radial-velocity lower bound (m/s)
     * @param rvMax fitted radial-velocity upper bound (m/s)
     * @param dMean distance normalisation mean (metres)
     * @param dStd distance normalisation standard deviation (metres)
     * @param vMean radial-velocity normalisation mean (m/s)
     * @param vStd radial-velocity normalisation standard deviation (m/s)
     * @param speedCoeffs exit-speed coefficients, one per term in evalPolyRaw
     * @param angleCoeffs launch-angle coefficients in the same order
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

    private static final PolyModel NO_CEILING_HUB_MODEL =
            new PolyModel(
                    "No Ceiling Hub Model",
                    1.5, // distMin (m)
                    8.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    4.7946224256, // dMean (m)
                    1.9514199579, // dStd (m)
                    -0.0434782609, // vMean (m/s)
                    1.9813242725, // vStd (m/s)
                    new double[] {
                        /* 1    */ 1.1143628795e+1,
                        /* d    */ 1.0138658152e+0,
                        /* v    */ -3.2159777567e-1,
                        /* d²   */ -5.1612304349e-2,
                        /* d·v  */ 3.6484374359e-1,
                        /* v²   */ -1.7402563290e-1,
                        /* d³   */ 7.9863642916e-2,
                        /* d²·v */ -1.2476148148e-1,
                        /* d·v² */ 3.8502387398e-1,
                        /* v³   */ -3.2039252056e-1
                    },
                    new double[] {
                        /* 1    */ 6.6464926591e+1,
                        /* d    */ -7.7256852841e+0,
                        /* v    */ 1.1734641473e+1,
                        /* d²   */ 8.1692458382e-3,
                        /* d·v  */ 5.2897492237e-1,
                        /* v²   */ -2.6845198119e-1,
                        /* d³   */ 1.7588060294e-1,
                        /* d²·v */ -2.0564699991e-3,
                        /* d·v² */ 1.2509312471e+0,
                        /* v³   */ -1.4532157984e+0
                    });

    private static final PolyModel CEILING_3M_HUB_MODEL =
            new PolyModel(
                    "3 Meter Ceiling Hub Model",
                    1.5, // distMin (m)
                    8.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    4.7330253114, // dMean (m)
                    1.8890844725, // dStd (m)
                    0.0229007634, // vMean (m/s)
                    1.9319161427, // vStd (m/s)
                    new double[] {
                        /* 1    */ 9.1291597222e+0,
                        /* d    */ 1.4704411927e+0,
                        /* v    */ -1.1245274618e+0,
                        /* d²   */ 5.9711528113e-2,
                        /* d·v  */ -1.0395358193e-1,
                        /* v²   */ 6.1638746813e-2,
                        /* d³   */ -3.2358373131e-2,
                        /* d²·v */ 1.7465238201e-2,
                        /* d·v² */ 3.5205649680e-2,
                        /* v³   */ -1.6138070441e-2
                    },
                    new double[] {
                        /* 1    */ 5.4780057238e+1,
                        /* d    */ -8.0553943910e+0,
                        /* v    */ 9.8969071974e+0,
                        /* d²   */ 9.3479980881e-1,
                        /* d·v  */ -2.3620060557e+0,
                        /* v²   */ 6.9825040437e-1,
                        /* d³   */ -4.9506580821e-1,
                        /* d²·v */ -6.4209468217e-1,
                        /* d·v² */ 9.0324521099e-1,
                        /* v³   */ -5.4437632941e-1
                    });

    private static final PolyModel FEED_MODEL =
            new PolyModel(
                    "Feed Model",
                    5.0, // distMin (m)
                    10.0, // distMax (m)
                    -3.0, // rvMin (m/s)
                    3.0, // rvMax (m/s)
                    7.5, // dMean (m)
                    1.5430334996, // dStd (m)
                    0.0, // vMean (m/s)
                    2.0, // vStd (m/s)
                    new double[] {
                        /* 1    */ 1.2074547373e+1,
                        /* d    */ 1.1124598419e+0,
                        /* v    */ -9.2720217607e-1,
                        /* d²   */ -5.8674357317e-2,
                        /* d·v  */ -7.2912571960e-2,
                        /* v²   */ -7.0818070818e-2,
                        /* d³   */ 5.6161425197e-2,
                        /* d²·v */ -6.0497571810e-3,
                        /* d·v² */ 2.1986814335e-1,
                        /* v³   */ -1.5954415954e-1
                    },
                    new double[] {
                        /* 1    */ 5.7369141664e+1,
                        /* d    */ -4.3735130912e+0,
                        /* v    */ 1.0080892292e+1,
                        /* d²   */ 7.8858336234e-2,
                        /* d·v  */ -1.5120778737e+0,
                        /* v²   */ 3.9384615385e-1,
                        /* d³   */ 2.6249905834e-1,
                        /* d²·v */ 2.9750087017e-1,
                        /* d·v² */ 1.0186869775e+0,
                        /* v³   */ -1.2099829060e+0
                    });

    /**
     * Dashboard selector for the hub model, so the ceiling-limited surface can be picked when
     * testing indoors without a redeploy. Feed shots always use {@link #FEED_MODEL}.
     */
    private final SendableChooser<PolyModel> hubModelChooser = new SendableChooser<>();

    private ShotCalculator() {
        hubModelChooser.setDefaultOption(NO_CEILING_HUB_MODEL.name(), NO_CEILING_HUB_MODEL);
        hubModelChooser.addOption(CEILING_3M_HUB_MODEL.name(), CEILING_3M_HUB_MODEL);
        SmartDashboard.putData("Hub Model Chooser", hubModelChooser);
    }

    /** Falls back to {@link #NO_CEILING_HUB_MODEL} when nothing has been selected yet. */
    private PolyModel selectedHubModel() {
        PolyModel selected = hubModelChooser.getSelected();
        return selected != null ? selected : NO_CEILING_HUB_MODEL;
    }

    private static final double LOOP_PERIOD_SECS = 0.02;

    /**
     * Phase delay applied to the estimated robot pose before computing shot parameters, to cover
     * sensor and network latency (seconds).
     */
    private static final double PHASE_DELAY_SECS = 0.03;

    private final LinearFilter hoodAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100 ms window

    private final LinearFilter driveAngleFilter =
            LinearFilter.movingAverage((int) (0.1 / LOOP_PERIOD_SECS)); // ~100 ms window

    private double lastHoodAngle = Double.NaN;
    private Rotation2d lastDriveAngle = null;

    /**
     * Shot parameters for the current pose and velocity, cached until {@link
     * #clearShootingParameters()} is called.
     *
     * <ol>
     *   <li>Delay the odometry pose by {@link #PHASE_DELAY_SECS} to cover sensor latency.
     *   <li>Split the launcher's field-relative velocity into radial and tangential parts.
     *   <li>Run the 1690 Orbit virtual-target solver, which corrects the aim for robot motion
     *       during flight.
     *   <li>Turn that into a drive angle, a hood angle, and a flywheel speed.
     * </ol>
     *
     * @return the latest {@link ShootingParameters}
     */
    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        boolean feed = Robot.getSuperStructure().isRobotInFeedZone();
        Translation2d target =
                feed ? FeedTargetFactory.generate() : HubTargetFactory.generate().toTranslation2d();
        // Feed and hub shots use separately-fitted polynomial surfaces.
        PolyModel model = feed ? FEED_MODEL : selectedHubModel();

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

        // Launcher velocity picks up an omega x r term from rotating about the robot centre
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

        double ux = launcherToTarget.getX() / distanceNoLookahead;
        double uy = launcherToTarget.getY() / distanceNoLookahead;
        // Positive radialVelocity = closing on target
        double radialVelocity = launcherVelocityX * ux + launcherVelocityY * uy;
        // Tangential: the radial axis rotated a quarter turn
        double tangentialVelocity = -launcherVelocityX * uy + launcherVelocityY * ux;

        double[] poly =
                solveVirtualTarget(model, distanceNoLookahead, radialVelocity, tangentialVelocity);
        double exitSpeedMs = poly[0];
        double rawHoodAngle = 90 - poly[1]; // degrees, before HOOD_ANGLE_OFFSET
        double yawOffsetDeg = poly[2];
        double lookaheadDist = poly[3];
        double tofFinal = poly[4];

        Rotation2d driveAngle =
                launcherToTarget
                        .getAngle()
                        .plus(Rotation2d.fromDegrees(yawOffsetDeg))
                        .plus(Rotation2d.fromDegrees(DRIVE_ANGLE_OFFSET))
                        .plus(Rotation2d.k180deg);

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

        // Differentiate the raw angle so the near-constant HOOD_ANGLE_OFFSET does not
        // bleed into the derivative
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = rawHoodAngle;
        double hoodVelocity =
                hoodAngleFilter.calculate((rawHoodAngle - lastHoodAngle) / LOOP_PERIOD_SECS);
        lastHoodAngle = rawHoodAngle;
        double hoodAngle = Math.max(rawHoodAngle + HOOD_ANGLE_OFFSET, 9);

        double flywheelSpeed = exitSpeedMs * RPM_PER_MPS;

        boolean isValid =
                distanceNoLookahead >= model.distMin() && distanceNoLookahead <= model.distMax();

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
        Telemetry.logDash("ShotCalc/DistanceMeters", lookaheadDist, "meters");
        Telemetry.log("ShotCalc/DistanceNoLookahead", distanceNoLookahead, "meters");
        Telemetry.logDash("ShotCalc/DriveAngleDeg", driveAngle.getDegrees(), "degrees");
        Telemetry.log("ShotCalc/YawOffsetDeg", yawOffsetDeg, "degrees");
        Telemetry.logDash("ShotCalc/HoodAngleDeg", hoodAngle, "degrees");
        Telemetry.logDash("ShotCalc/FlywheelSpeedRPM", flywheelSpeed, "RPM");
        Telemetry.logDash("ShotCalc/ExitSpeedMs", exitSpeedMs, "m/s");
        Telemetry.log("ShotCalc/RadialVelocityMs", radialVelocity, "m/s");
        Telemetry.log("ShotCalc/TangentialVelocityMs", tangentialVelocity, "m/s");
        Telemetry.logDash("ShotCalc/TimeOfFlight", tofFinal, "seconds");
        Telemetry.logDash("ShotCalc/FeedShot", feed);
        Telemetry.log("ShotCalc/HubPolyModel", model.name());
        Telemetry.logDash("ShotCalc/DriveAngleOffsetDegrees", DRIVE_ANGLE_OFFSET, "degrees");
        Telemetry.logDash("ShotCalc/HoodAngleOffsetDegrees", HOOD_ANGLE_OFFSET, "degrees");
        Telemetry.log("ShotCalc/Target", target);

        return latestParameters;
    }

    public void clearShootingParameters() {
        latestParameters = null;
    }

    /**
     * 1690 Orbit virtual-target solver. Each pass evaluates the polynomial at the aim point,
     * estimates time of flight, then shifts the aim point by how far the launcher moves during that
     * flight. Converges in two or three of the five allowed iterations.
     *
     * @param model the hub or feed surface to evaluate against
     * @param distance horizontal distance to the goal centre (metres)
     * @param radialVelocity launcher velocity toward the goal (m/s); positive is closing
     * @param tangentialVelocity launcher velocity across the goal line (m/s)
     * @return exit speed (m/s), launch angle (degrees), yaw offset (degrees), converged lookahead
     *     distance (metres), time of flight (seconds), in that order
     */
    private static double[] solveVirtualTarget(
            PolyModel model, double distance, double radialVelocity, double tangentialVelocity) {
        double vdx = distance; // virtual aim point, radial (m)
        double vdz = 0.0; // virtual aim point, lateral (m)
        double tof = 0.0;

        for (int iter = 0; iter < 5; iter++) {
            double vDist = Math.sqrt(vdx * vdx + vdz * vdz);
            if (vDist < 0.1) break;

            // Evaluate at rv = 0; the shifted aim point already carries the robot's motion
            double[] raw = evalPolyRaw(model, vDist, 0.0);
            double speed = raw[0] * MPS_FACTOR;
            double cosA = Math.cos(raw[1] * Math.PI / 180.0);
            double prevTof = tof;
            tof = vDist / Math.max(speed * cosA, 0.5); // guard against div-by-zero

            // Aim at where the target will be when the ball arrives
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
            result[0] * MPS_FACTOR, // m/s
            result[1], // degrees
            yawOffsetDeg, // degrees
            virtualDist, // m
            tof // s
        };
    }

    /**
     * Evaluates the surface at (distance, radialVel), clamping both to the model's fitted range.
     * Output is raw, so callers apply {@link #MPS_FACTOR} to the exit speed.
     *
     * @param distance horizontal distance to the aim point (metres)
     * @param radialVel radial velocity (m/s)
     * @return exit speed (m/s) then launch angle (degrees)
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
