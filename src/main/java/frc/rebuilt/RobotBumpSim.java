// Copyright (c) 2025-2026 KAISER 6989
// https://github.com/haar09/FRC-Rebuilt-BumpSim
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Robot Z, pitch and roll over the 2026 REBUILT bumps, plus a frictionless slide model so a robot
 * that lacks the speed to crest a bump slides back down instead of ghosting through it.
 *
 * <p>All positions are metres, origin at the blue driver station corner.
 *
 * <p>The caller must call {@link #update} every loop after MapleSim has stepped, and while {@link
 * #isOnRamp()} is true must feed {@link #getSimWorldPose} back into MapleSim, or the robot only
 * appears to slide rather than actually sliding.
 *
 * <p>Minimum crossing speed is sqrt(2·g·h) at a 0.165 m bump, about 1.80 m/s. {@link
 * #WHEEL_RADIUS}, {@link #CHASSIS_HEIGHT} and {@link #BUMP_COR} are the knobs to turn for a
 * different chassis.
 */
public class RobotBumpSim {

    /** Robot control-loop period (seconds). Matches the WPILib default of 20 ms. */
    private static final double PERIOD = 0.02;

    /** Gravitational acceleration vector (m/s², pointing in the -Z direction). */
    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81);

    private static final double FIELD_LENGTH = Field.fieldLength;

    private static final double FIELD_WIDTH = Field.fieldWidth;

    /**
     * Start points of the eight bump XZ line segments (four per alliance, ascending + descending).
     *
     * <p>Each {@link Translation3d} stores {@code (fieldX, yMin, fieldZ)}: the world-X start of the
     * ramp face, the minimum field-Y at which this segment is present, and the ramp Z at that X.
     * {@link #BUMP_LINE_ENDS} holds the matching end point with the maximum Y. Indices 0 to 3 are
     * the blue-side bump, 4 to 7 the red-side.
     */
    static final Translation3d[] BUMP_LINE_STARTS = {
        // Blue bump: ascending faces, Z rises from 0 to 0.165 m
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
        // Blue bump: descending faces, Z falls from 0.165 to 0 m
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        // Red bump: ascending faces
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
        // Red bump: descending faces
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    };

    /**
     * End points of the eight bump XZ line segments.
     *
     * <p>Each {@link Translation3d} stores {@code (fieldX, yMax, fieldZ)}: the world-X end of the
     * ramp face, the maximum field-Y at which this segment is present, and the ramp Z at that X.
     */
    static final Translation3d[] BUMP_LINE_ENDS = {
        // Blue bump: ascending faces
        new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        // Blue bump: descending faces
        new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        // Red bump: ascending faces
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
        // Red bump: descending faces
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    };

    private static final int BUMP_LINE_FIRST = 0;

    private static final int BUMP_LINE_LAST = BUMP_LINE_STARTS.length - 1;

    /** Effective wheel contact radius against the bump surface (metres). */
    private static final double WHEEL_RADIUS = 0.048;

    /** Height offset from the average module-contact Z to the robot-body origin (metres). */
    private static final double CHASSIS_HEIGHT = 0.0;

    /**
     * Coefficient of restitution for vertical (Z) robot-bump collisions. 0 = perfectly inelastic
     * (no bounce), 1 = perfectly elastic.
     */
    private static final double BUMP_COR = 0.15;

    /**
     * Tolerance on the segment projection test, so rounding cannot make a module sitting exactly on
     * an endpoint read as a miss.
     */
    private static final double SEGMENT_PROJECTION_TOLERANCE = 1e-6;

    /** Robot-relative module positions, order: FL(0), FR(1), BL(2), BR(3). */
    private final Translation2d[] moduleOffsets;

    /** Absolute Z position of each module contact point (metres above the floor). */
    private final double[] moduleZPos;

    /** Z velocity of each module contact point (m/s). */
    private final double[] moduleZVel;

    /** Distance between front and back module pairs along the robot X axis (metres). */
    private final double frontBackDist;

    /** Distance between left and right module pairs along the robot Y axis (metres). */
    private final double leftRightDist;

    /**
     * True while the robot is on the ramp and this sim owns the field-X position. The caller must
     * apply {@link #getSimWorldPose(Pose2d)} to MapleSim whenever this is true.
     */
    private boolean onRamp = false;

    /**
     * Absolute field-X position of the robot while on the ramp (frictionless model). Ignored when
     * {@link #onRamp} is false.
     */
    private double simXPos = 0.0;

    /**
     * Field-X velocity of the robot on the ramp (m/s, frictionless model). Initialized to the
     * robot's field-Vx at first ramp contact; decelerated by gravity-along-ramp each subtick.
     * Ignored when {@link #onRamp} is false.
     */
    private double simXVel = 0.0;

    /**
     * Bump sim for a swerve drivetrain.
     *
     * @param moduleOffsets robot-relative module positions in order FL, FR, BL, BR (metres).
     *     Typically obtained via {@code CommandSwerveDrivetrain#getModuleLocations()}.
     */
    public RobotBumpSim(Translation2d[] moduleOffsets) {
        this.moduleOffsets = moduleOffsets;
        this.moduleZPos = new double[4];
        this.moduleZVel = new double[4];

        double frontX = (moduleOffsets[0].getX() + moduleOffsets[1].getX()) / 2.0;
        double backX = (moduleOffsets[2].getX() + moduleOffsets[3].getX()) / 2.0;
        frontBackDist = Math.max(Math.abs(frontX - backX), 1e-3);

        double leftY = (moduleOffsets[0].getY() + moduleOffsets[2].getY()) / 2.0;
        double rightY = (moduleOffsets[1].getY() + moduleOffsets[3].getY()) / 2.0;
        leftRightDist = Math.max(Math.abs(leftY - rightY), 1e-3);
    }

    /**
     * True while the robot is on the ramp in frictionless-slide mode. While it is, the caller must
     * apply {@link #getSimWorldPose(Pose2d)} to MapleSim so the robot actually slides backward
     * rather than only appearing to.
     */
    public boolean isOnRamp() {
        return onRamp;
    }

    /**
     * The 2D pose to set on MapleSim while on the ramp.
     *
     * <p>X is the frictionless {@link #simXPos}; Y and rotation come from {@code latestMaplePose}
     * so MapleSim keeps owning lateral motion.
     *
     * @param latestMaplePose the most recent 2D pose read from MapleSim
     * @return a pose to pass to {@code setSimulationWorldPose}
     */
    public Pose2d getSimWorldPose(Pose2d latestMaplePose) {
        return new Pose2d(simXPos, latestMaplePose.getY(), latestMaplePose.getRotation());
    }

    /**
     * Advances the bump simulation by one 20 ms period and returns the robot's 3D pose.
     *
     * <p>While on the ramp the returned pose uses {@link #simXPos} for X. The caller must also
     * apply {@link #getSimWorldPose(Pose2d)} to MapleSim so the simulated position matches.
     *
     * @param robotPose2d robot's 2D pose from the MapleSim drivetrain
     * @param fieldRelativeSpeeds field-relative chassis speeds from the MapleSim drivetrain
     * @param subticks physics sub-steps per period. Must match the value used by any companion ball
     *     or object sim so they stay in sync. Typical value: 5, four 4 ms sub-steps per 20 ms loop.
     * @return a pose with the crossed X, Z, pitch and roll
     */
    public Pose3d update(Pose2d robotPose2d, ChassisSpeeds fieldRelativeSpeeds, int subticks) {
        double vx = fieldRelativeSpeeds.vxMetersPerSecond;
        double dt = PERIOD / subticks;

        // A diagonal crossing decelerates less: contactFactor is 1 at 0 and 90 degrees of yaw,
        // straight on or sideways, and |cos(90 deg)| = 0.71 at 45 degrees.
        double contactFactor = Math.abs(Math.cos(2 * robotPose2d.getRotation().getRadians()));

        // Y positions of each module (MapleSim-owned, constant for the whole period)
        double[] worldY = new double[4];
        for (int i = 0; i < 4; i++) {
            Translation2d wo = moduleOffsets[i].rotateBy(robotPose2d.getRotation());
            worldY[i] = robotPose2d.getY() + wo.getY();
        }

        for (int tick = 0; tick < subticks; tick++) {
            double currentRobotX = onRamp ? simXPos : robotPose2d.getX();

            double gravAccelXSum = 0.0;
            int contactCount = 0;

            for (int i = 0; i < 4; i++) {
                Translation2d wo = moduleOffsets[i].rotateBy(robotPose2d.getRotation());
                double wx = currentRobotX + wo.getX();

                moduleZVel[i] += GRAVITY.getZ() * dt;
                moduleZPos[i] += moduleZVel[i] * dt;

                for (int lineIdx = BUMP_LINE_FIRST; lineIdx <= BUMP_LINE_LAST; lineIdx++) {
                    double gax =
                            handleModuleBumpCollision(
                                    i, wx, worldY[i], onRamp ? simXVel : vx, lineIdx);
                    if (!Double.isNaN(gax)) {
                        gravAccelXSum += gax;
                        contactCount++;
                    }
                }

                if (moduleZPos[i] < 0.0) {
                    moduleZPos[i] = 0.0;
                    if (moduleZVel[i] < 0.0) moduleZVel[i] = -moduleZVel[i] * BUMP_COR;
                }
            }

            if (contactCount > 0) {
                if (!onRamp) {
                    // First ramp contact: take X and X velocity off MapleSim.
                    onRamp = true;
                    simXPos = robotPose2d.getX();
                    simXVel = vx;
                }
                // Average gravity-along-ramp deceleration, frictionless surface.
                double avgGravAccelX = (gravAccelXSum / contactCount) * contactFactor;
                simXVel += avgGravAccelX * dt;
                simXPos += simXVel * dt;
            } else if (onRamp) {
                boolean allFlat = true;
                for (int i = 0; i < 4; i++) {
                    if (moduleZPos[i] > 0.01) {
                        allFlat = false;
                        break;
                    }
                }
                if (allFlat) {
                    // Backed out or crossed, so MapleSim reclaims X.
                    onRamp = false;
                } else {
                    // Briefly airborne after the peak: keep sliding under simXVel.
                    simXPos += simXVel * dt;
                }
            }
        }

        return computePose3d(robotPose2d);
    }

    /**
     * Handles the XZ-plane bump collision for one module against one ramp segment, applying a Z
     * position correction and a Z velocity impulse.
     *
     * <p>The returned acceleration is the gravity component along the ramp surface. On an ascending
     * face it is negative and pulls the robot back; on a descending face it is positive and pushes
     * it forward.
     *
     * @param worldX module's world-X position (metres)
     * @param worldY module's world-Y position (metres)
     * @param currentXVel robot's current field-X velocity (simXVel when on ramp, else vx)
     * @param lineIdx index into {@link #BUMP_LINE_STARTS} and {@link #BUMP_LINE_ENDS}
     * @return gravity-along-ramp X acceleration in m/s², or {@link Double#NaN} when not in contact
     */
    private double handleModuleBumpCollision(
            int moduleIdx, double worldX, double worldY, double currentXVel, int lineIdx) {
        Translation3d lineStart = BUMP_LINE_STARTS[lineIdx];
        Translation3d lineEnd = BUMP_LINE_ENDS[lineIdx];

        // Each segment carries a Y range, not a Y position.
        if (worldY < lineStart.getY() || worldY > lineEnd.getY()) return Double.NaN;

        Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
        Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
        Translation2d pos2d = new Translation2d(worldX, moduleZPos[moduleIdx]);
        Translation2d lineVec = end2d.minus(start2d);

        // Closest point on the XZ segment to the module (parametric projection)
        Translation2d toModule = pos2d.minus(start2d);
        double projectionT = toModule.dot(lineVec) / lineVec.getSquaredNorm();
        Translation2d projected = start2d.plus(lineVec.times(projectionT));

        if (projected.getDistance(start2d) + projected.getDistance(end2d)
                > lineVec.getNorm() + SEGMENT_PROJECTION_TOLERANCE)
            return Double.NaN; // off segment

        double dist = pos2d.getDistance(projected);
        if (dist > WHEEL_RADIUS) return Double.NaN; // not intersecting

        // Outward normal in XZ: lineVec = (deltaX, deltaZ) -> normal = (-deltaZ, deltaX) /
        // |lineVec|
        double normalX = -lineVec.getY() / lineVec.getNorm();
        double normalZ = lineVec.getX() / lineVec.getNorm();

        moduleZPos[moduleIdx] += normalZ * (WHEEL_RADIUS - dist);

        // This impulse only tilts the visual pose; it does not move the robot along X
        double velDotNormal = currentXVel * normalX + moduleZVel[moduleIdx] * normalZ;
        if (velDotNormal < 0.0) {
            moduleZVel[moduleIdx] += normalZ * (-(1.0 + BUMP_COR) * velDotNormal);
        }

        // Frictionless surface, so no drive force contributes to the X acceleration
        return -GRAVITY.getZ() * normalX * normalZ;
    }

    /**
     * Derives a {@link Pose3d} from the robot's 2D pose and the four module Z positions. Pitch and
     * roll come from front/back and left/right height differences. X uses {@link #simXPos} when on
     * the ramp.
     */
    private Pose3d computePose3d(Pose2d robotPose2d) {
        // FL=0, FR=1, BL=2, BR=3
        double frontZ = (moduleZPos[0] + moduleZPos[1]) / 2.0;
        double backZ = (moduleZPos[2] + moduleZPos[3]) / 2.0;
        double leftZ = (moduleZPos[0] + moduleZPos[2]) / 2.0;
        double rightZ = (moduleZPos[1] + moduleZPos[3]) / 2.0;
        double centerZ = (frontZ + backZ) / 2.0 + CHASSIS_HEIGHT;

        // Negated because WPILib Rotation3d pitch positive is nose down, so this is nose up
        double pitch = -Math.atan2(frontZ - backZ, frontBackDist);
        // Positive means the left side is higher than the right
        double roll = Math.atan2(leftZ - rightZ, leftRightDist);

        double visualX = onRamp ? simXPos : robotPose2d.getX();

        return new Pose3d(
                visualX,
                robotPose2d.getY(),
                centerZ,
                new Rotation3d(roll, pitch, robotPose2d.getRotation().getRadians()));
    }
}
