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
 * Bump physics for the 2026 REBUILT field, to run alongside MapleSim. Tracks the robot's Z height,
 * pitch, and roll over the raised bumps, and takes over field X while the robot is on a ramp so it
 * slides back rather than ghosting through.
 *
 * <p>Cresting a bump needs 1.8 m/s, from sqrt(2 * g * 0.165).
 *
 * <p>While {@link #isOnRamp()} is true the caller must push {@link #getSimWorldPose} into MapleSim.
 * Positions are in metres, with the origin at the blue alliance driver-station corner.
 */
public class RobotBumpSim {

    /** Robot control-loop period (seconds), matching the WPILib default. */
    private static final double PERIOD = 0.02;

    /** Gravitational acceleration vector (m/s²). */
    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81);

    /** Field length (metres). */
    private static final double FIELD_LENGTH = Field.fieldLength;

    /** Field width (metres). */
    private static final double FIELD_WIDTH = Field.fieldWidth;

    /**
     * Start points of the eight bump XZ line segments, as (fieldX, yMin, fieldZ): where the face
     * starts in X, the lowest field-Y it spans, and the ramp height there. Indices 0 and 1 are the
     * Blue ascending faces, 2 and 3 the Blue descending, 4 and 5 the Red ascending, 6 and 7 the Red
     * descending. {@link #BUMP_LINE_ENDS} holds the matching end points.
     */
    static final Translation3d[] BUMP_LINE_STARTS = {
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    };

    /**
     * End points of the eight bump XZ line segments, as (fieldX, yMax, fieldZ). The index layout
     * matches {@link #BUMP_LINE_STARTS}.
     */
    static final Translation3d[] BUMP_LINE_ENDS = {
        new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
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
     * Coefficient of restitution for vertical collisions with the bump. 0 is fully inelastic, 1 is
     * fully elastic.
     */
    private static final double BUMP_COR = 0.15;

    /** Tolerance so rounding at a segment endpoint does not read as a miss. */
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

    /** True while this sim owns the robot's field-X position. */
    private boolean onRamp = false;

    /** Field-X position owned by the ramp model (m). Ignored while {@link #onRamp} is false. */
    private double simXPos = 0.0;

    /**
     * Field-X velocity owned by the ramp model (m/s), seeded from the robot's field-Vx at first
     * contact and decelerated by gravity along the ramp each subtick. Ignored while {@link #onRamp}
     * is false.
     */
    private double simXVel = 0.0;

    /**
     * @param moduleOffsets robot-relative module positions, in FL, FR, BL, BR order (metres).
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
     * While this returns true the caller must push {@link #getSimWorldPose} into MapleSim, so the
     * robot slides back instead of only appearing to.
     */
    public boolean isOnRamp() {
        return onRamp;
    }

    /**
     * X comes from the ramp model; Y and rotation pass through from {@code latestMaplePose} so
     * MapleSim keeps owning lateral motion.
     *
     * @return the pose to pass to {@code setSimulationWorldPose}
     */
    public Pose2d getSimWorldPose(Pose2d latestMaplePose) {
        return new Pose2d(simXPos, latestMaplePose.getY(), latestMaplePose.getRotation());
    }

    /**
     * Advances the sim by one {@link #PERIOD} and returns the robot's 3D pose. X is the ramp
     * model's position while on the ramp, so the caller must also push {@link #getSimWorldPose}
     * into MapleSim.
     *
     * @param subticks physics sub-steps per period, which must match the value a companion ball sim
     *     uses. 5 gives 4 ms sub-steps in a 20 ms loop
     * @return the robot's 3D pose
     */
    public Pose3d update(Pose2d robotPose2d, ChassisSpeeds fieldRelativeSpeeds, int subticks) {
        double vx = fieldRelativeSpeeds.vxMetersPerSecond;
        double dt = PERIOD / subticks;

        // A diagonal approach eases the climb: the factor is abs(cos(2 * heading)), so it is 1.0
        // straight across the bump, 0 at 45 degrees to it, and back to 1.0 at 90 degrees.
        double contactFactor = Math.abs(Math.cos(2 * robotPose2d.getRotation().getRadians()));

        // Module Y is MapleSim-owned and fixed for the whole period
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
                    onRamp = true;
                    simXPos = robotPose2d.getX();
                    simXVel = vx;
                }
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
                    // Backed out or crossed, so MapleSim reclaims X
                    onRamp = false;
                } else {
                    // Airborne over the peak; keep coasting on simXVel
                    simXPos += simXVel * dt;
                }
            }
        }

        return computePose3d(robotPose2d);
    }

    /**
     * Resolves module {@code moduleIdx} against bump segment {@code lineIdx} in the XZ plane,
     * correcting Z position and applying a Z impulse. Returns the gravity-along-ramp X acceleration
     * (m/s²), or {@link Double#NaN} with no contact. Gravity opposes travel up an ascending face
     * and adds to it on a descending one. The surface is frictionless, so the drivetrain
     * contributes no force here.
     */
    private double handleModuleBumpCollision(
            int moduleIdx, double worldX, double worldY, double currentXVel, int lineIdx) {
        Translation3d lineStart = BUMP_LINE_STARTS[lineIdx];
        Translation3d lineEnd = BUMP_LINE_ENDS[lineIdx];

        if (worldY < lineStart.getY() || worldY > lineEnd.getY()) return Double.NaN;

        Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
        Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
        Translation2d pos2d = new Translation2d(worldX, moduleZPos[moduleIdx]);
        Translation2d lineVec = end2d.minus(start2d);

        Translation2d toModule = pos2d.minus(start2d);
        double projectionT = toModule.dot(lineVec) / lineVec.getSquaredNorm();
        Translation2d projected = start2d.plus(lineVec.times(projectionT));

        if (projected.getDistance(start2d) + projected.getDistance(end2d)
                > lineVec.getNorm() + SEGMENT_PROJECTION_TOLERANCE)
            return Double.NaN; // off segment

        double dist = pos2d.getDistance(projected);
        if (dist > WHEEL_RADIUS) return Double.NaN;

        double normalX = -lineVec.getY() / lineVec.getNorm();
        double normalZ = lineVec.getX() / lineVec.getNorm();

        moduleZPos[moduleIdx] += normalZ * (WHEEL_RADIUS - dist);

        // Z velocity impulse. The X model is unaffected.
        double velDotNormal = currentXVel * normalX + moduleZVel[moduleIdx] * normalZ;
        if (velDotNormal < 0.0) {
            moduleZVel[moduleIdx] += normalZ * (-(1.0 + BUMP_COR) * velDotNormal);
        }

        return -GRAVITY.getZ() * normalX * normalZ;
    }

    private Pose3d computePose3d(Pose2d robotPose2d) {
        double frontZ = (moduleZPos[0] + moduleZPos[1]) / 2.0;
        double backZ = (moduleZPos[2] + moduleZPos[3]) / 2.0;
        double leftZ = (moduleZPos[0] + moduleZPos[2]) / 2.0;
        double rightZ = (moduleZPos[1] + moduleZPos[3]) / 2.0;
        double centerZ = (frontZ + backZ) / 2.0 + CHASSIS_HEIGHT;

        // Pitch is negated: WPILib takes positive Rotation3d pitch as nose-down
        double pitch = -Math.atan2(frontZ - backZ, frontBackDist);
        // Roll: positive when the left side is higher
        double roll = Math.atan2(leftZ - rightZ, leftRightDist);

        double visualX = onRamp ? simXPos : robotPose2d.getX();

        return new Pose3d(
                visualX,
                robotPose2d.getY(),
                centerZ,
                new Rotation3d(roll, pitch, robotPose2d.getRotation().getRadians()));
    }
}
