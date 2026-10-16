/*
 * FuelPhysicsSim.java - Full-field ball physics simulation for FRC 2026 REBUILT
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

package frc.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Full-field ball physics for simulation: gravity, drag, Magnus lift, friction, ball and field
 * collisions, hub scoring, sleeping, continuous collision detection and robot interaction. All
 * quantities are SI. Depends only on WPILib.
 *
 * <p>Symplectic Euler integration, sequential impulse contacts with warm starting, spatial hashing
 * for ball pairs, and sleeping for resting balls, which keeps 350 or more resting balls under 2 ms
 * per tick. A caller supplies the robot pose and speeds and calls {@link #tick()} every loop; the
 * sim owns nothing else.
 *
 * <pre>{@code
 * FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");
 * ballSim.enable();
 * ballSim.placeFieldBalls();
 * ballSim.configureRobot(width, length, bumperH, poseSupplier, speedsSupplier);
 * ballSim.tick();   // from simulationPeriodic()
 * }</pre>
 */
public class FuelPhysicsSim {

    private static final double GRAVITY = 9.81; // m/s^2
    private static final double AIR_DENSITY = 1.225; // kg/m^3, standard atmosphere
    private static final double BALL_MASS = 0.215; // kg, game manual 5.10.1 midpoint
    private static final double BALL_DIAMETER = 0.1501; // m, game manual 5.10.1
    private static final double BALL_RADIUS = BALL_DIAMETER / 2.0;
    private static final double BALL_CROSS_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS;
    private static final double BALL_MOMENT_OF_INERTIA =
            0.4 * BALL_MASS * BALL_RADIUS * BALL_RADIUS; // 2/5 * m * r^2, solid sphere

    private static final double DEFAULT_CD = 0.47; // drag coefficient, smooth sphere
    private static final double DEFAULT_CM = 0.2; // Magnus coefficient, conservative estimate

    // Precomputed, already divided by mass, so they apply as accelerations
    private static final double DRAG_ACCEL_FACTOR =
            0.5 * AIR_DENSITY * DEFAULT_CD * BALL_CROSS_AREA / BALL_MASS;
    // The extra BALL_RADIUS turns the omega x v cross product into an acceleration
    private static final double MAGNUS_ACCEL_FACTOR =
            0.5 * AIR_DENSITY * DEFAULT_CM * BALL_CROSS_AREA * BALL_RADIUS / BALL_MASS;

    // Coefficients of restitution, per material, from the field element build instructions
    private static final double COR_CARPET = 0.65; // foam on low-pile carpet
    private static final double COR_WALL =
            0.70; // foam on polycarbonate (alliance walls, guardrails)
    private static final double COR_STEEL = 0.72; // foam on powder-coated steel (tower, rungs)
    private static final double COR_HUB = 0.70; // foam on polycarbonate (hub body panels)
    private static final double COR_HDPE = 0.60; // foam on textured HDPE (bump ramps, 15deg)
    private static final double COR_NET = 0.15; // mesh fabric absorbs most of the energy
    private static final double COR_BUMPER = 0.08; // polycarb-backed foam, nearly inelastic
    private static final double COR_BALL_BALL = 0.45; // foam-on-foam, high deformation loss

    private static final double MU_GROUND_KINETIC = 0.3; // kinetic friction on carpet
    private static final double MU_GROUND_ROLLING = 0.05; // rolling friction
    private static final double MU_WALL = 0.4; // foam on polycarbonate/fabric
    private static final double MU_BALL_BALL = 0.3; // foam on foam

    // Velocity-dependent COR reference speed
    private static final double COR_VREF = 3.0; // m/s
    private static final double COR_EXPONENT = 0.15;

    // Reused so the hot loops do not allocate a normal per contact
    private static final Translation3d AXIS_X_POS = new Translation3d(1, 0, 0);
    private static final Translation3d AXIS_X_NEG = new Translation3d(-1, 0, 0);
    private static final Translation3d AXIS_Y_POS = new Translation3d(0, 1, 0);
    private static final Translation3d AXIS_Y_NEG = new Translation3d(0, -1, 0);
    private static final Translation3d AXIS_Z_POS = new Translation3d(0, 0, 1);
    private static final Translation3d AXIS_Z_NEG = new Translation3d(0, 0, -1);

    private static final double PERIOD = 0.02; // 20ms

    private static final double FIELD_LENGTH = 16.541; // m
    private static final double FIELD_WIDTH = 8.052; // m

    private static final double ALLIANCE_WALL_HEIGHT = 0.935; // m, 36.8 in
    private static final double GUARDRAIL_HEIGHT = 0.508; // m, 20 in

    // Bump ramps, tent shaped with 15 degree faces
    private static final double BUMP_HEIGHT = 0.165; // m, 6.513 in

    // Hub positions from the game manual field layout
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5974, 4.035);
    private static final Translation2d RED_HUB_CENTER = new Translation2d(11.938, 4.035);

    private static final double HUB_ENTRY_HEIGHT = 1.829; // m, 72 in
    private static final double HUB_ENTRY_RADIUS = 0.5295; // m, 41.7 in / 2 across flats
    private static final double HUB_SIDE = 1.194; // m, 47 in square base
    // Base frame only: it deflects ground-level balls but is not a wall up to scoring height
    private static final double HUB_BASE_HEIGHT = 0.5; // m, ~20 in base frame

    private static final double NET_HEIGHT_MIN = 1.5; // m
    private static final double NET_HEIGHT_MAX = 3.057; // m
    private static final double NET_WIDTH = 1.484; // m
    private static final double NET_OFFSET = HUB_SIDE / 2.0 + 0.261;

    private static final double TRENCH_WIDTH = 1.265; // m
    private static final double TRENCH_BLOCK_WIDTH = 0.305; // m, 12 in
    private static final double TRENCH_HEIGHT = 0.565; // m, 22.25 in underpass
    private static final double TRENCH_PILLAR_HEIGHT = 1.346; // m, 53 in

    private static final double TOWER_POLE_WIDTH = 0.051; // m, 2 in
    private static final double TOWER_POLE_HEIGHT = 1.194; // m, 47 in
    private static final double TOWER_UPRIGHT_THICK = 0.038; // m, 1.5 in
    private static final double TOWER_UPRIGHT_DEPTH = 0.089; // m, 3.5 in
    private static final double TOWER_UPRIGHT_HEIGHT = 1.831; // m, 72.1 in
    private static final double TOWER_UPRIGHT_SPACING = 0.819; // m, 32.25 in center-to-center

    // Tower rungs are schedule 40 pipe
    private static final double RUNG_OUTER_DIAMETER = 0.042; // m, 1.66 in OD
    private static final double RUNG_RADIUS = RUNG_OUTER_DIAMETER / 2.0;
    private static final double RUNG_OVERHANG = 0.149; // m, 5.875 in past upright
    private static final double RUNG_LOW_HEIGHT = 0.686; // m, 27 in
    private static final double RUNG_MID_HEIGHT = 1.143; // m, 45 in
    private static final double RUNG_HIGH_HEIGHT = 1.600; // m, 63 in

    private static final double BRACING_BOTTOM = 0.721; // m, 28.4 in
    private static final double BRACING_TOP = 1.102; // m, 43.4 in

    private static final double HUB_RAMP_WIDTH = 1.194; // m, 47 in
    private static final double HUB_RAMP_LENGTH = 5.512; // m, 217 in
    private static final double HUB_RAMP_HEIGHT = 0.165; // m, 6.5 in

    private static final double CELL_SIZE = 0.25;
    private static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
    private static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

    private static final int MAX_BALLS = 2000;

    private static final BumpSegment[] BUMP_SEGMENTS = {
        // Blue lower bump ramp up/down
        new BumpSegment(3.96, 0, 4.524, BUMP_HEIGHT, 1.88, 3.73),
        new BumpSegment(4.524, BUMP_HEIGHT, 5.088, 0, 1.88, 3.73),
        // Blue upper bump ramp up/down
        new BumpSegment(3.96, 0, 4.524, BUMP_HEIGHT, 4.32, 6.17),
        new BumpSegment(4.524, BUMP_HEIGHT, 5.088, 0, 4.32, 6.17),
        // Red lower bump ramp up/down
        new BumpSegment(FIELD_LENGTH - 5.088, 0, FIELD_LENGTH - 4.524, BUMP_HEIGHT, 1.88, 3.73),
        new BumpSegment(FIELD_LENGTH - 4.524, BUMP_HEIGHT, FIELD_LENGTH - 3.96, 0, 1.88, 3.73),
        // Red upper bump ramp up/down
        new BumpSegment(FIELD_LENGTH - 5.088, 0, FIELD_LENGTH - 4.524, BUMP_HEIGHT, 4.32, 6.17),
        new BumpSegment(FIELD_LENGTH - 4.524, BUMP_HEIGHT, FIELD_LENGTH - 3.96, 0, 4.32, 6.17),
    };

    private static final AABB[] AABB_OBSTACLES;

    static {
        List<AABB> aabbs = new ArrayList<>();

        // Trench pillars (4): 12in wide x 53in tall, steel structure
        aabbs.add(
                new AABB(
                        3.96,
                        TRENCH_WIDTH,
                        0,
                        3.96 + TRENCH_BLOCK_WIDTH,
                        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        3.96,
                        FIELD_WIDTH - 1.57 - TRENCH_BLOCK_WIDTH,
                        0,
                        3.96 + TRENCH_BLOCK_WIDTH,
                        FIELD_WIDTH - 1.57,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        FIELD_LENGTH - 3.96 - TRENCH_BLOCK_WIDTH,
                        TRENCH_WIDTH,
                        0,
                        FIELD_LENGTH - 3.96,
                        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        FIELD_LENGTH - 3.96 - TRENCH_BLOCK_WIDTH,
                        FIELD_WIDTH - 1.57 - TRENCH_BLOCK_WIDTH,
                        0,
                        FIELD_LENGTH - 3.96,
                        FIELD_WIDTH - 1.57,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));

        // Trench ceilings (4): steel/aluminum above underpass height
        double trenchXBlue1 = 3.96;
        double trenchXBlue2 = 5.18;
        double trenchXRed1 = FIELD_LENGTH - 5.18;
        double trenchXRed2 = FIELD_LENGTH - 3.96;
        aabbs.add(
                new AABB(
                        trenchXBlue1,
                        1.57,
                        TRENCH_HEIGHT,
                        trenchXBlue2,
                        3.73,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        trenchXBlue1,
                        FIELD_WIDTH - 3.73,
                        TRENCH_HEIGHT,
                        trenchXBlue2,
                        FIELD_WIDTH - 1.57,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        trenchXRed1,
                        1.57,
                        TRENCH_HEIGHT,
                        trenchXRed2,
                        3.73,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        trenchXRed1,
                        FIELD_WIDTH - 3.73,
                        TRENCH_HEIGHT,
                        trenchXRed2,
                        FIELD_WIDTH - 1.57,
                        TRENCH_PILLAR_HEIGHT,
                        COR_STEEL));

        // Tower poles (2): 2in wide x 47in tall
        double blueTowerX = 1.067;
        double redTowerX = 15.494;
        double blueTowerY = 4.039;
        double redTowerY = 4.318;
        aabbs.add(
                new AABB(
                        blueTowerX - TOWER_POLE_WIDTH / 2,
                        blueTowerY - TOWER_POLE_WIDTH / 2,
                        0,
                        blueTowerX + TOWER_POLE_WIDTH / 2,
                        blueTowerY + TOWER_POLE_WIDTH / 2,
                        TOWER_POLE_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        redTowerX - TOWER_POLE_WIDTH / 2,
                        redTowerY - TOWER_POLE_WIDTH / 2,
                        0,
                        redTowerX + TOWER_POLE_WIDTH / 2,
                        redTowerY + TOWER_POLE_WIDTH / 2,
                        TOWER_POLE_HEIGHT,
                        COR_STEEL));

        // Tower uprights (4): 1.5in x 3.5in x 72.1in, two per tower
        double halfSpacing = TOWER_UPRIGHT_SPACING / 2.0;
        aabbs.add(
                new AABB(
                        0,
                        blueTowerY - halfSpacing - TOWER_UPRIGHT_THICK / 2,
                        0,
                        TOWER_UPRIGHT_DEPTH,
                        blueTowerY - halfSpacing + TOWER_UPRIGHT_THICK / 2,
                        TOWER_UPRIGHT_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        0,
                        blueTowerY + halfSpacing - TOWER_UPRIGHT_THICK / 2,
                        0,
                        TOWER_UPRIGHT_DEPTH,
                        blueTowerY + halfSpacing + TOWER_UPRIGHT_THICK / 2,
                        TOWER_UPRIGHT_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
                        redTowerY - halfSpacing - TOWER_UPRIGHT_THICK / 2,
                        0,
                        FIELD_LENGTH,
                        redTowerY - halfSpacing + TOWER_UPRIGHT_THICK / 2,
                        TOWER_UPRIGHT_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
                        redTowerY + halfSpacing - TOWER_UPRIGHT_THICK / 2,
                        0,
                        FIELD_LENGTH,
                        redTowerY + halfSpacing + TOWER_UPRIGHT_THICK / 2,
                        TOWER_UPRIGHT_HEIGHT,
                        COR_STEEL));

        // Tower bracing (2): between uprights, 28.4-43.4in height
        aabbs.add(
                new AABB(
                        0,
                        blueTowerY - halfSpacing,
                        BRACING_BOTTOM,
                        TOWER_UPRIGHT_DEPTH,
                        blueTowerY + halfSpacing,
                        BRACING_TOP,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
                        redTowerY - halfSpacing,
                        BRACING_BOTTOM,
                        FIELD_LENGTH,
                        redTowerY + halfSpacing,
                        BRACING_TOP,
                        COR_STEEL));

        // Hub ramp colliders (2): 47in x 217in ground-level area around each hub
        aabbs.add(
                new AABB(
                        BLUE_HUB_CENTER.getX() - HUB_RAMP_WIDTH / 2,
                        BLUE_HUB_CENTER.getY() - HUB_RAMP_LENGTH / 2,
                        0,
                        BLUE_HUB_CENTER.getX() + HUB_RAMP_WIDTH / 2,
                        BLUE_HUB_CENTER.getY() + HUB_RAMP_LENGTH / 2,
                        HUB_RAMP_HEIGHT,
                        COR_STEEL));
        aabbs.add(
                new AABB(
                        RED_HUB_CENTER.getX() - HUB_RAMP_WIDTH / 2,
                        RED_HUB_CENTER.getY() - HUB_RAMP_LENGTH / 2,
                        0,
                        RED_HUB_CENTER.getX() + HUB_RAMP_WIDTH / 2,
                        RED_HUB_CENTER.getY() + HUB_RAMP_LENGTH / 2,
                        HUB_RAMP_HEIGHT,
                        COR_STEEL));

        // Hub body panels deflect ground-level balls
        aabbs.add(
                new AABB(
                        BLUE_HUB_CENTER.getX() - HUB_SIDE / 2,
                        BLUE_HUB_CENTER.getY() - HUB_SIDE / 2,
                        0,
                        BLUE_HUB_CENTER.getX() + HUB_SIDE / 2,
                        BLUE_HUB_CENTER.getY() + HUB_SIDE / 2,
                        HUB_BASE_HEIGHT,
                        COR_HUB));
        aabbs.add(
                new AABB(
                        RED_HUB_CENTER.getX() - HUB_SIDE / 2,
                        RED_HUB_CENTER.getY() - HUB_SIDE / 2,
                        0,
                        RED_HUB_CENTER.getX() + HUB_SIDE / 2,
                        RED_HUB_CENTER.getY() + HUB_SIDE / 2,
                        HUB_BASE_HEIGHT,
                        COR_HUB));

        AABB_OBSTACLES = aabbs.toArray(new AABB[0]);
    }

    private static final CylinderObstacle[] CYLINDER_OBSTACLES;

    static {
        List<CylinderObstacle> cyls = new ArrayList<>();
        double blueTowerY = 4.039;
        double redTowerY = 4.318;
        double halfSpacing = TOWER_UPRIGHT_SPACING / 2.0;

        // Blue tower rungs (3)
        for (double h : new double[] {RUNG_LOW_HEIGHT, RUNG_MID_HEIGHT, RUNG_HIGH_HEIGHT}) {
            cyls.add(
                    new CylinderObstacle(
                            0,
                            blueTowerY - halfSpacing - RUNG_OVERHANG,
                            h,
                            0,
                            blueTowerY + halfSpacing + RUNG_OVERHANG,
                            h,
                            RUNG_RADIUS,
                            COR_STEEL));
        }

        // Red tower rungs (3)
        for (double h : new double[] {RUNG_LOW_HEIGHT, RUNG_MID_HEIGHT, RUNG_HIGH_HEIGHT}) {
            cyls.add(
                    new CylinderObstacle(
                            FIELD_LENGTH,
                            redTowerY - halfSpacing - RUNG_OVERHANG,
                            h,
                            FIELD_LENGTH,
                            redTowerY + halfSpacing + RUNG_OVERHANG,
                            h,
                            RUNG_RADIUS,
                            COR_STEEL));
        }

        CYLINDER_OBSTACLES = cyls.toArray(new CylinderObstacle[0]);
    }

    /** Axis-aligned bounding box with restitution coefficient. */
    private record AABB(
            double minX,
            double minY,
            double minZ,
            double maxX,
            double maxY,
            double maxZ,
            double cor) {}

    /** Cylinder obstacle defined by two axis endpoints, a radius, and restitution. */
    private record CylinderObstacle(
            double ax,
            double ay,
            double az,
            double bx,
            double by,
            double bz,
            double radius,
            double cor,
            double abx,
            double aby,
            double abz,
            double abLenSq) {
        CylinderObstacle(
                double ax,
                double ay,
                double az,
                double bx,
                double by,
                double bz,
                double radius,
                double cor) {
            this(
                    ax,
                    ay,
                    az,
                    bx,
                    by,
                    bz,
                    radius,
                    cor,
                    bx - ax,
                    by - ay,
                    bz - az,
                    (bx - ax) * (bx - ax) + (by - ay) * (by - ay) + (bz - az) * (bz - az));
        }
    }

    /** XZ line segment extruded along a Y range, used for bump ramp geometry. */
    private record BumpSegment(
            double xStart,
            double zStart,
            double xEnd,
            double zEnd,
            double yStart,
            double yEnd,
            double lineX,
            double lineZ,
            double lineLen,
            double nx,
            double nz) {
        BumpSegment(
                double xStart,
                double zStart,
                double xEnd,
                double zEnd,
                double yStart,
                double yEnd) {
            this(
                    xStart,
                    zStart,
                    xEnd,
                    zEnd,
                    yStart,
                    yEnd,
                    xEnd - xStart,
                    zEnd - zStart,
                    Math.hypot(xEnd - xStart, zEnd - zStart),
                    // Normal perpendicular to line in XZ, flipped so nz >= 0 (points away from
                    // ground)
                    (xEnd - xStart) >= 0
                            ? -(zEnd - zStart) / Math.hypot(xEnd - xStart, zEnd - zStart)
                            : (zEnd - zStart) / Math.hypot(xEnd - xStart, zEnd - zStart),
                    Math.abs(xEnd - xStart) / Math.hypot(xEnd - xStart, zEnd - zStart));
        }
    }

    /** Physics feature toggles. Flip these on or off to debug or simplify the sim. */
    public static class PhysicsConfig {
        public boolean dragEnabled = true;
        public boolean magnusEnabled = true;
        public boolean frictionEnabled = true;
        public boolean spinTransferEnabled = true;
        public boolean sleepingEnabled = true;
        public boolean ccdEnabled = true;
        public boolean velocityDependentCOR = true;
        public boolean spinDecayEnabled = true;
        public int solverIterations = 4;
        public int subticks = 5;
        public double spinDecayTau = 3.0; // seconds
        public double sleepVelocityThreshold = 0.01; // m/s
        public int sleepFrameThreshold = 10; // consecutive frames
        public double ccdSpeedThreshold = 10.0; // m/s
        public double baumgarteBeta = 0.2;
        public double baumgarteSlop = 0.005; // 5mm allowed penetration
        public boolean deterministic = false;
        public long deterministicSeed = 42L;
        public boolean conservationMonitor = false;

        /** Everything on. */
        public PhysicsConfig() {}

        /** Deep copy. */
        public PhysicsConfig copy() {
            PhysicsConfig c = new PhysicsConfig();
            c.dragEnabled = dragEnabled;
            c.magnusEnabled = magnusEnabled;
            c.frictionEnabled = frictionEnabled;
            c.spinTransferEnabled = spinTransferEnabled;
            c.sleepingEnabled = sleepingEnabled;
            c.ccdEnabled = ccdEnabled;
            c.velocityDependentCOR = velocityDependentCOR;
            c.spinDecayEnabled = spinDecayEnabled;
            c.solverIterations = solverIterations;
            c.subticks = subticks;
            c.spinDecayTau = spinDecayTau;
            c.sleepVelocityThreshold = sleepVelocityThreshold;
            c.sleepFrameThreshold = sleepFrameThreshold;
            c.ccdSpeedThreshold = ccdSpeedThreshold;
            c.baumgarteBeta = baumgarteBeta;
            c.baumgarteSlop = baumgarteSlop;
            c.deterministic = deterministic;
            c.deterministicSeed = deterministicSeed;
            c.conservationMonitor = conservationMonitor;
            return c;
        }
    }

    /** One ball in the simulation. Tracks position, velocity, spin, and lifecycle flags. */
    public static class SimBall {
        Translation3d pos; // field-frame position (m)
        Translation3d vel; // field-frame velocity (m/s)
        Translation3d omega; // angular velocity (rad/s), 3D spin axis

        // Previous state, for CCD and scoring detection
        Translation3d prevPos;
        Translation3d prevVel;

        boolean sleeping;
        int sleepCounter;

        // Stuck-on-obstacle detection
        int elevatedSlowCounter;
        Translation3d elevatedAnchorPos;
        boolean hadFieldContact;

        boolean intaked;
        boolean outOfBounds;

        SimBall(Translation3d pos, Translation3d vel, Translation3d omega) {
            this.pos = pos;
            this.vel = vel;
            this.omega = omega;
            this.prevPos = pos;
            this.prevVel = vel;
            this.sleeping = false;
            this.sleepCounter = 0;
            this.elevatedSlowCounter = 0;
            this.intaked = false;
            this.outOfBounds = false;
        }

        SimBall(Translation3d pos, Translation3d vel) {
            this(pos, vel, new Translation3d());
        }

        SimBall(Translation3d pos) {
            this(pos, new Translation3d(), new Translation3d());
        }

        /** Backspin in RPM, taken from the Y component of {@link #omega}. */
        public double getSpinRPM() {
            return omega.getY() * 60.0 / (2.0 * Math.PI);
        }
    }

    /** Contact point between two colliding objects. Used by the impulse solver. */
    static class Contact {
        int ballIndexA; // index into balls list
        int ballIndexB; // index into balls list, or -1 for field geometry
        Translation3d normal; // contact normal (A -> B or outward from field)
        double penetration; // overlap depth (positive = overlapping)
        Translation3d contactPoint; // world-space contact location
        double normalImpulseAccum; // warm-start accumulated normal impulse
        double tangentImpulseAccum; // warm-start accumulated tangent impulse
        double restitution; // effective COR for this pair
        double friction; // Coulomb mu for this pair
        double restitutionVelocity; // target bounce-back speed, set once before solving

        Contact() {
            normal = new Translation3d();
            contactPoint = new Translation3d();
        }
    }

    /** A hub that can be scored in. Detects balls falling through the opening. */
    public static class ScoringTarget {
        final Translation2d center;
        final Translation3d exit;
        final int exitVelXSign; // +1 for blue (exits toward red), -1 for red
        int score;

        ScoringTarget(Translation2d center, Translation3d exit, int exitVelXSign) {
            this.center = center;
            this.exit = exit;
            this.exitVelXSign = exitVelXSign;
            this.score = 0;
        }

        boolean didScore(SimBall ball) {
            double dist2d = ball.pos.toTranslation2d().getDistance(center);
            if (dist2d > HUB_ENTRY_RADIUS) return false;
            double currZ = ball.pos.getZ();
            double prevZ = ball.prevPos.getZ();
            // Only count balls falling through the opening (top-down entry)
            return prevZ > HUB_ENTRY_HEIGHT && currZ <= HUB_ENTRY_HEIGHT;
        }

        Translation3d getDispersalVelocity(Random rng) {
            double vx = exitVelXSign * (rng.nextDouble() + 0.1) * 1.5;
            double vy = rng.nextDouble() * 2.0 - 1.0;
            return new Translation3d(vx, vy, 0);
        }

        public int getScore() {
            return score;
        }

        void resetScore() {
            score = 0;
        }
    }

    /** Intake zone defined in robot-relative coordinates. Picks up balls that enter the box. */
    static class IntakeZone {
        final double xMin, xMax, yMin, yMax;
        final BooleanSupplier active;
        final Runnable callback;

        IntakeZone(
                double xMin,
                double xMax,
                double yMin,
                double yMax,
                BooleanSupplier active,
                Runnable callback) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.active = active;
            this.callback = callback;
        }

        /** Runs the zone's callback when the ball is inside, so the pickup is not lost. */
        boolean shouldIntake(SimBall ball, Pose2d robotPose, double bumperHeight) {
            if (!active.getAsBoolean() || ball.pos.getZ() > bumperHeight) return false;
            Translation2d relPos =
                    new Pose2d(ball.pos.toTranslation2d(), Rotation2d.kZero)
                            .relativeTo(robotPose)
                            .getTranslation();
            boolean inside =
                    relPos.getX() >= xMin
                            && relPos.getX() <= xMax
                            && relPos.getY() >= yMin
                            && relPos.getY() <= yMax;
            if (inside) {
                callback.run();
            }
            return inside;
        }
    }

    private final List<SimBall> balls = new ArrayList<>();
    private final List<Contact> contacts = new ArrayList<>();
    private final List<Contact> contactPool = new ArrayList<>(); // pre-allocated contact pool
    private int contactPoolIndex = 0;

    private PhysicsConfig config;
    private Random rng;

    // Spatial hash grid for ball-ball broadphase
    @SuppressWarnings("unchecked")
    private final List<Integer>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

    private final ScoringTarget blueHub;
    private final ScoringTarget redHub;

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;
    private double robotWidth;
    private double robotLength;
    private double bumperHeight;
    private int hopperSize =
            Integer.MAX_VALUE; // max balls the robot can hold; unlimited by default

    private final List<IntakeZone> intakes = new ArrayList<>();

    private int totalLaunched;
    private int totalScored;
    private int totalIntaked;
    private double lastLaunchSpeed;

    private boolean running;

    private StructArrayPublisher<Translation3d> positionPublisher;
    private StructArrayPublisher<Translation3d> inFlightPublisher;
    private StructArrayPublisher<Translation3d> lastShotArcPublisher;
    private IntegerPublisher blueScorePub;
    private IntegerPublisher redScorePub;
    private IntegerPublisher ballCountPub;
    private IntegerPublisher activeBallsPub;
    private IntegerPublisher sleepingBallsPub;
    private IntegerPublisher contactCountPub;
    private DoublePublisher physicsTimePub;
    private DoublePublisher totalEnergyPub;

    // Last shot arc for trajectory visualization (predicted path in Field3d)
    private Translation3d[] lastShotArc = new Translation3d[0];
    private long lastPhysicsNanos;

    private double totalKE;
    private double totalPE;
    private Translation3d totalMomentum = new Translation3d();

    /** Publishes ball positions under {@code tableKey}, using the default physics config. */
    public FuelPhysicsSim(String tableKey) {
        this(tableKey, new PhysicsConfig());
    }

    /**
     * Publishes ball positions under {@code tableKey}.
     *
     * @param config physics feature toggles and tuning
     */
    public FuelPhysicsSim(String tableKey, PhysicsConfig config) {
        this.config = config;
        this.rng = config.deterministic ? new Random(config.deterministicSeed) : new Random();

        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j] = new ArrayList<>();
            }
        }

        for (int i = 0; i < 200; i++) {
            contactPool.add(new Contact());
        }

        blueHub =
                new ScoringTarget(
                        BLUE_HUB_CENTER, new Translation3d(5.3, FIELD_WIDTH / 2.0, 0.89), 1);
        redHub =
                new ScoringTarget(
                        RED_HUB_CENTER,
                        new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2.0, 0.89),
                        -1);

        var nt = NetworkTableInstance.getDefault();
        positionPublisher =
                nt.getStructArrayTopic(tableKey + "/Positions", Translation3d.struct).publish();
        inFlightPublisher =
                nt.getStructArrayTopic(tableKey + "/InFlight", Translation3d.struct).publish();
        lastShotArcPublisher =
                nt.getStructArrayTopic(tableKey + "/LastShotArc", Translation3d.struct).publish();
        blueScorePub = nt.getIntegerTopic(tableKey + "/BlueScore").publish();
        redScorePub = nt.getIntegerTopic(tableKey + "/RedScore").publish();
        ballCountPub = nt.getIntegerTopic(tableKey + "/Stats/BallCount").publish();
        activeBallsPub = nt.getIntegerTopic(tableKey + "/Stats/ActiveBalls").publish();
        sleepingBallsPub = nt.getIntegerTopic(tableKey + "/Stats/SleepingBalls").publish();
        contactCountPub = nt.getIntegerTopic(tableKey + "/Stats/ContactsPerTick").publish();
        physicsTimePub = nt.getDoubleTopic(tableKey + "/Stats/PhysicsMs").publish();
        totalEnergyPub = nt.getDoubleTopic(tableKey + "/Stats/TotalEnergy").publish();

        running = false;
        totalLaunched = 0;
        totalScored = 0;
        totalIntaked = 0;
        lastLaunchSpeed = 0;
    }

    /** Default constructor, publishes to "Sim/FuelPositions". */
    public FuelPhysicsSim() {
        this("Sim/FuelPositions");
    }

    /** Turn the sim on. You still need to call tick() every cycle. */
    public void enable() {
        running = true;
    }

    public void disable() {
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    /**
     * Tells the sim about your robot so it can handle bumper collisions and intake pickup. Hopper
     * capacity is unlimited.
     *
     * @param width robot width along the Y axis (m)
     * @param length robot length along the X axis (m)
     * @param bumperHeight bumper height (m)
     * @param hopperSize maximum number of balls the hopper can hold; intake stops when full
     * @param poseSupplier field-relative pose supplier
     * @param speedsSupplier field-relative chassis speeds supplier
     */
    public void configureRobot(
            double width,
            double length,
            double bumperHeight,
            int hopperSize,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier) {
        this.robotWidth = width;
        this.robotLength = length;
        this.bumperHeight = bumperHeight;
        this.hopperSize = hopperSize;
        this.robotPoseSupplier = poseSupplier;
        this.robotSpeedsSupplier = speedsSupplier;
    }

    /**
     * Tells the sim about your robot so it can handle bumper collisions and intake pickup. Hopper
     * capacity is unlimited.
     *
     * @param width robot width along the Y axis (m)
     * @param length robot length along the X axis (m)
     * @param bumperHeight bumper height (m)
     * @param poseSupplier field-relative pose supplier
     * @param speedsSupplier field-relative chassis speeds supplier
     */
    public void configureRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier) {
        configureRobot(
                width, length, bumperHeight, Integer.MAX_VALUE, poseSupplier, speedsSupplier);
    }

    /**
     * Add an intake zone. Balls that enter this box (in robot-relative coords) get picked up.
     *
     * @param xMin near edge in the robot frame
     * @param xMax far edge in the robot frame
     * @param yMin low edge in the robot frame
     * @param yMax high edge in the robot frame
     * @param active returns true when the intake is actually running
     * @param callback fires when a ball gets picked up
     */
    public void addIntakeZone(
            double xMin,
            double xMax,
            double yMin,
            double yMax,
            BooleanSupplier active,
            Runnable callback) {
        intakes.add(new IntakeZone(xMin, xMax, yMin, yMax, active, callback));
    }

    /** Add an intake zone without a callback. */
    public void addIntakeZone(
            double xMin, double xMax, double yMin, double yMax, BooleanSupplier active) {
        addIntakeZone(xMin, xMax, yMin, yMax, active, () -> {});
    }

    /**
     * Shoot a ball into the sim.
     *
     * @param pos where the ball leaves the launcher (field frame, meters)
     * @param vel launch velocity (field frame, m/s)
     * @param spinRPM backspin in RPM (positive = backspin = Magnus lift)
     */
    public void launchBall(Translation3d pos, Translation3d vel, double spinRPM) {
        // Backspin axis is horizontal and perpendicular to the launch heading, so omega x v lifts
        // for any heading. Pinning it to a fixed axis would give no lift when firing along that
        // axis and invert the lift when firing against it.
        double omegaMag = spinRPM * 2.0 * Math.PI / 60.0;
        double hLen = Math.hypot(vel.getX(), vel.getY());
        Translation3d omega =
                hLen < 1e-6
                        ? new Translation3d(0, -omegaMag, 0)
                        : new Translation3d(
                                omegaMag * vel.getY() / hLen, -omegaMag * vel.getX() / hLen, 0);
        launchBall(pos, vel, omega);
    }

    /**
     * Shoot a ball with full 3D spin control.
     *
     * @param pos launch position (field frame, meters)
     * @param vel launch velocity (field frame, m/s)
     * @param omega 3D angular velocity (rad/s)
     */
    public void launchBall(Translation3d pos, Translation3d vel, Translation3d omega) {
        // The ball leaves the hopper even when the field is full and it isn't simulated, so a
        // caller waiting on getTotalIntaked() > 0 doesn't retry forever.
        totalIntaked = Math.max(0, totalIntaked - 1);
        if (balls.size() >= MAX_BALLS) return;
        SimBall ball = new SimBall(pos, vel, omega);
        balls.add(ball);
        totalLaunched++;
        lastLaunchSpeed = vel.getNorm();

        // Predict the trajectory arc for Field3d visualization (up to 10 s, sampled every 50 ms)
        lastShotArc = predictArc(pos, vel, omega, 200, 0.05);

        // Wake nearby sleeping balls
        if (config.sleepingEnabled) {
            wakeNearbyBalls(pos, 1.0);
        }
    }

    public void spawnBall(Translation3d pos) {
        if (balls.size() >= MAX_BALLS) return;
        balls.add(new SimBall(pos));
    }

    public void spawnBall(Translation3d pos, Translation3d vel) {
        if (balls.size() >= MAX_BALLS) return;
        balls.add(new SimBall(pos, vel));
    }

    public void clearBalls() {
        balls.clear();
    }

    /** Spawn all game pieces in their starting positions (neutral zone + depots). */
    public void placeFieldBalls() {
        double cx = FIELD_LENGTH / 2.0;
        double cy = FIELD_WIDTH / 2.0;
        int nzCols = 12; // 12 * 5.91 in = 70.9 in, inside the 72.0 in depth
        int nzRows = 30; // 30 * 5.91 in = 177.3 in, inside the 206.0 in width
        double halfDivider = 0.0254; // half of the 2.0in center divider

        for (int col = 0; col < nzCols; col++) {
            double x = cx + (col - (nzCols - 1) * 0.5) * BALL_DIAMETER;
            for (int row = 0; row < nzRows; row++) {
                double y = cy + (row - (nzRows - 1) * 0.5) * BALL_DIAMETER;
                if (Math.abs(y - cy) < halfDivider) continue;
                spawnBall(new Translation3d(x, y, BALL_RADIUS));
            }
        }

        int depotCols = 4; // 4 * 5.91 in = 23.6 in, inside the 27.0 in depth
        int depotRows = 6; // 6 * 5.91 in = 35.5 in, inside the 42.0 in width

        fillBallGrid(FIELD_LENGTH - 0.37, 2.10, depotCols, depotRows);
        fillBallGrid(0.37, FIELD_WIDTH - 2.10, depotCols, depotRows);
    }

    /** Fill a grid of balls centred on (cx, cy). */
    private void fillBallGrid(double cx, double cy, int cols, int rows) {
        for (int c = 0; c < cols; c++) {
            double x = cx + (c - (cols - 1) * 0.5) * BALL_DIAMETER;
            for (int r = 0; r < rows; r++) {
                double y = cy + (r - (rows - 1) * 0.5) * BALL_DIAMETER;
                spawnBall(new Translation3d(x, y, BALL_RADIUS));
            }
        }
    }

    /**
     * Step the sim forward one period (20ms) and publish ball positions to NT. Does nothing if the
     * sim isn't enabled.
     */
    public void tick() {
        if (!running) return;
        long t0 = System.nanoTime();
        advancePhysics(PERIOD);
        lastPhysicsNanos = System.nanoTime() - t0;
        publishPositions();
    }

    /**
     * Advance physics by dt seconds. Splits into subticks for stability.
     *
     * @param dt time to advance (seconds)
     */
    public void advancePhysics(double dt) {
        int ticks = Math.max(1, config.subticks);
        double subDt = dt / ticks;

        for (int tick = 0; tick < ticks; tick++) {
            stepSubtick(subDt);
        }

        removeFlaggedBalls();
    }

    private void stepSubtick(double subDt) {
        contactPoolIndex = 0;
        contacts.clear();

        for (int i = 0; i < balls.size(); i++) {
            SimBall ball = balls.get(i);
            ball.hadFieldContact = false;
            if (ball.intaked || ball.outOfBounds) continue;
            if (ball.sleeping && config.sleepingEnabled) continue;

            ball.prevPos = ball.pos;
            ball.prevVel = ball.vel;

            Translation3d accel = computeAcceleration(ball);

            // Symplectic Euler: update velocity first so we don't accumulate energy drift
            ball.vel = ball.vel.plus(accel.times(subDt));
            ball.pos = ball.pos.plus(ball.vel.times(subDt));

            if (config.spinDecayEnabled && ball.omega.getNorm() > 1e-6) {
                double decayFactor = Math.exp(-subDt / config.spinDecayTau);
                ball.omega = ball.omega.times(decayFactor);
            }
        }

        if (config.ccdEnabled) {
            for (int i = 0; i < balls.size(); i++) {
                SimBall ball = balls.get(i);
                if (ball.intaked || ball.outOfBounds) continue;
                if (ball.sleeping && config.sleepingEnabled) continue;
                if (ball.vel.getNorm() > config.ccdSpeedThreshold) {
                    handleCCD(ball);
                }
            }
        }

        buildSpatialHash();
        generateBallBallContacts();
        generateBallFieldContacts();
        solveContacts();

        // Walls and ground are handled by a direct impulse rather than a contact
        for (int i = 0; i < balls.size(); i++) {
            SimBall ball = balls.get(i);
            if (ball.intaked || ball.outOfBounds) continue;
            if (ball.sleeping && config.sleepingEnabled) continue;
            handleWallBounce(ball);
            handleGroundContact(ball, subDt);
            handleBumpCollisions(ball);
        }

        for (int i = 0; i < balls.size(); i++) {
            SimBall ball = balls.get(i);
            if (ball.intaked || ball.outOfBounds) continue;
            handleHubScoring(ball);
        }

        for (int i = 0; i < balls.size(); i++) {
            SimBall ball = balls.get(i);
            if (ball.intaked || ball.outOfBounds) continue;
            if (ball.sleeping && config.sleepingEnabled) continue;
            handleNetCollision(ball, blueHub);
            handleNetCollision(ball, redHub);
        }

        if (robotPoseSupplier != null && robotSpeedsSupplier != null) {
            Pose2d robotPose = robotPoseSupplier.get();
            ChassisSpeeds speeds = robotSpeedsSupplier.get();
            Translation2d robotVel =
                    new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

            // Robot half-diagonal plus a margin, so balls react before they are touched
            double wakeRadius = Math.hypot(robotLength, robotWidth) / 2.0 + 0.3;
            double wakeRadiusSq = wakeRadius * wakeRadius;

            for (int i = 0; i < balls.size(); i++) {
                SimBall ball = balls.get(i);
                if (ball.intaked || ball.outOfBounds) continue;
                if (ball.sleeping && config.sleepingEnabled) {
                    double dx = ball.pos.getX() - robotPose.getX();
                    double dy = ball.pos.getY() - robotPose.getY();
                    if (dx * dx + dy * dy < wakeRadiusSq) {
                        wakeBall(ball);
                    } else {
                        continue;
                    }
                }
                // Intake first, so balls are consumed before the bumpers push them away
                handleIntakePickup(ball, robotPose);
                if (ball.intaked) continue;
                handleRobotCollision(ball, robotPose, robotVel);
            }
        }

        if (config.sleepingEnabled) {
            for (int i = 0; i < balls.size(); i++) {
                updateSleepState(balls.get(i));
            }
        }

        // Out-of-bounds cleanup: NaN guard, upper Z limit, stuck-on-obstacle removal
        for (int i = 0; i < balls.size(); i++) {
            SimBall ball = balls.get(i);
            if (!Double.isFinite(ball.pos.getX())
                    || !Double.isFinite(ball.pos.getY())
                    || !Double.isFinite(ball.pos.getZ())
                    || !Double.isFinite(ball.vel.getX())
                    || !Double.isFinite(ball.vel.getY())
                    || !Double.isFinite(ball.vel.getZ())
                    || !Double.isFinite(ball.omega.getX())
                    || !Double.isFinite(ball.omega.getY())
                    || !Double.isFinite(ball.omega.getZ())
                    || ball.pos.getX() < -2.0
                    || ball.pos.getX() > FIELD_LENGTH + 2.0
                    || ball.pos.getY() < -2.0
                    || ball.pos.getY() > FIELD_WIDTH + 2.0
                    || ball.pos.getZ() < -1.0
                    || ball.pos.getZ() > 15.0) {
                ball.outOfBounds = true;
            }
            if (ball.hadFieldContact && ball.vel.getNorm() < 0.5) {
                ball.elevatedSlowCounter++;
                if (ball.elevatedSlowCounter > 250) {
                    ball.outOfBounds = true;
                }
            } else {
                ball.elevatedSlowCounter = 0;
            }
        }

        if (config.conservationMonitor) {
            computeConservationQuantities();
        }
    }

    /** Compute acceleration: gravity + drag + Magnus lift. Magnus only kicks in when airborne. */
    private Translation3d computeAcceleration(SimBall ball) {
        double ax = 0, ay = 0, az = -GRAVITY;

        double speed = ball.vel.getNorm();
        boolean airborne = ball.pos.getZ() > BALL_RADIUS + 0.01;

        // Drag acts whether on ground or airborne (air doesn't vanish at carpet level)
        if (config.dragEnabled && speed > 1e-6) {
            ax -= DRAG_ACCEL_FACTOR * speed * ball.vel.getX();
            ay -= DRAG_ACCEL_FACTOR * speed * ball.vel.getY();
            az -= DRAG_ACCEL_FACTOR * speed * ball.vel.getZ();
        }

        // Magnus only applies airborne (ground friction dominates spin behavior on carpet)
        if (airborne && speed > 1e-6) {
            if (config.magnusEnabled && ball.omega.getNorm() > 1e-3) {
                Translation3d magnusDir = cross(ball.omega, ball.vel);
                double magnusMag = magnusDir.getNorm();
                if (magnusMag > 1e-6) {
                    ax += MAGNUS_ACCEL_FACTOR * magnusDir.getX();
                    ay += MAGNUS_ACCEL_FACTOR * magnusDir.getY();
                    az += MAGNUS_ACCEL_FACTOR * magnusDir.getZ();
                }
            }
        }

        return new Translation3d(ax, ay, az);
    }

    /** Continuous collision detection: sweep fast balls so they don't tunnel through walls. */
    private void handleCCD(SimBall ball) {
        Translation3d delta = ball.pos.minus(ball.prevPos);
        double dist = delta.getNorm();
        if (dist < 1e-6) return;

        Translation3d dir = delta.div(dist);

        double tMin = 1.0;
        Translation3d hitNormal = null;

        // X walls
        if (dir.getX() < -1e-6) {
            double t = (BALL_RADIUS - ball.prevPos.getX()) / (delta.getX());
            if (t > 0 && t < tMin) {
                tMin = t;
                hitNormal = AXIS_X_POS;
            }
        } else if (dir.getX() > 1e-6) {
            double t = (FIELD_LENGTH - BALL_RADIUS - ball.prevPos.getX()) / (delta.getX());
            if (t > 0 && t < tMin) {
                tMin = t;
                hitNormal = AXIS_X_NEG;
            }
        }

        // Y walls
        if (dir.getY() < -1e-6) {
            double t = (BALL_RADIUS - ball.prevPos.getY()) / (delta.getY());
            if (t > 0 && t < tMin) {
                tMin = t;
                hitNormal = AXIS_Y_POS;
            }
        } else if (dir.getY() > 1e-6) {
            double t = (FIELD_WIDTH - BALL_RADIUS - ball.prevPos.getY()) / (delta.getY());
            if (t > 0 && t < tMin) {
                tMin = t;
                hitNormal = AXIS_Y_NEG;
            }
        }

        // Ground
        if (dir.getZ() < -1e-6) {
            double t = (BALL_RADIUS - ball.prevPos.getZ()) / (delta.getZ());
            if (t > 0 && t < tMin) {
                tMin = t;
                hitNormal = AXIS_Z_POS;
            }
        }

        for (AABB aabb : AABB_OBSTACLES) {
            double tHit = sweepSphereAABB(ball.prevPos, delta, aabb);
            if (tHit >= 0 && tHit < tMin) {
                Translation3d hitPos = ball.prevPos.plus(delta.times(tHit));
                Translation3d n = computeAABBNormal(hitPos, aabb);
                if (n != null) {
                    tMin = tHit;
                    hitNormal = n;
                }
            }
        }

        if (hitNormal != null && tMin < 1.0) {
            ball.pos = ball.prevPos.plus(delta.times(tMin));

            double vDotN = ball.vel.dot(hitNormal);
            if (vDotN < 0) {
                double cor = config.velocityDependentCOR ? velocityCOR(COR_WALL, -vDotN) : COR_WALL;
                ball.vel = ball.vel.minus(hitNormal.times((1.0 + cor) * vDotN));
            }
        }
    }

    /**
     * Ray-AABB intersection with sphere expansion (Minkowski sum). Returns hit time in [0,1] or -1.
     */
    private double sweepSphereAABB(Translation3d origin, Translation3d delta, AABB aabb) {
        double minX = aabb.minX() - BALL_RADIUS;
        double minY = aabb.minY() - BALL_RADIUS;
        double minZ = aabb.minZ() - BALL_RADIUS;
        double maxX = aabb.maxX() + BALL_RADIUS;
        double maxY = aabb.maxY() + BALL_RADIUS;
        double maxZ = aabb.maxZ() + BALL_RADIUS;

        double tEnter = 0;
        double tExit = 1;

        // X slab
        if (Math.abs(delta.getX()) > 1e-9) {
            double invD = 1.0 / delta.getX();
            double t1 = (minX - origin.getX()) * invD;
            double t2 = (maxX - origin.getX()) * invD;
            if (t1 > t2) {
                double tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            tEnter = Math.max(tEnter, t1);
            tExit = Math.min(tExit, t2);
        } else {
            if (origin.getX() < minX || origin.getX() > maxX) return -1;
        }

        // Y slab
        if (Math.abs(delta.getY()) > 1e-9) {
            double invD = 1.0 / delta.getY();
            double t1 = (minY - origin.getY()) * invD;
            double t2 = (maxY - origin.getY()) * invD;
            if (t1 > t2) {
                double tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            tEnter = Math.max(tEnter, t1);
            tExit = Math.min(tExit, t2);
        } else {
            if (origin.getY() < minY || origin.getY() > maxY) return -1;
        }

        // Z slab
        if (Math.abs(delta.getZ()) > 1e-9) {
            double invD = 1.0 / delta.getZ();
            double t1 = (minZ - origin.getZ()) * invD;
            double t2 = (maxZ - origin.getZ()) * invD;
            if (t1 > t2) {
                double tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            tEnter = Math.max(tEnter, t1);
            tExit = Math.min(tExit, t2);
        } else {
            if (origin.getZ() < minZ || origin.getZ() > maxZ) return -1;
        }

        if (tEnter > tExit || tExit < 0) return -1;
        return tEnter > 0 ? tEnter : -1; // Already inside if tEnter <= 0
    }

    private void buildSpatialHash() {
        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j].clear();
            }
        }
        // Include sleeping balls so awake balls can detect them for wake-on-collision
        for (int i = 0; i < balls.size(); i++) {
            SimBall ball = balls.get(i);
            if (ball.intaked || ball.outOfBounds) continue;
            int col = (int) (ball.pos.getX() / CELL_SIZE);
            int row = (int) (ball.pos.getY() / CELL_SIZE);
            if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                grid[col][row].add(i);
            }
        }
    }

    private void generateBallBallContacts() {
        for (int i = 0; i < balls.size(); i++) {
            SimBall ballA = balls.get(i);
            if (ballA.intaked || ballA.outOfBounds) continue;
            if (ballA.sleeping && config.sleepingEnabled) continue;

            int col = (int) (ballA.pos.getX() / CELL_SIZE);
            int row = (int) (ballA.pos.getY() / CELL_SIZE);

            for (int di = -1; di <= 1; di++) {
                for (int dj = -1; dj <= 1; dj++) {
                    int ci = col + di;
                    int cj = row + dj;
                    if (ci < 0 || ci >= GRID_COLS || cj < 0 || cj >= GRID_ROWS) continue;
                    List<Integer> cell = grid[ci][cj];
                    for (int k = 0; k < cell.size(); k++) {
                        int j = cell.get(k);
                        if (j == i) continue;
                        if (j < i && !(config.sleepingEnabled && balls.get(j).sleeping)) continue;

                        SimBall ballB = balls.get(j);
                        double dx = ballA.pos.getX() - ballB.pos.getX();
                        double dy = ballA.pos.getY() - ballB.pos.getY();
                        double dz = ballA.pos.getZ() - ballB.pos.getZ();
                        double distSq = dx * dx + dy * dy + dz * dz;
                        double minDist = BALL_RADIUS * 2;

                        if (distSq < minDist * minDist) {
                            double dist = Math.sqrt(distSq);
                            Contact c = allocateContact();
                            if (dist < 1e-9) {
                                c.normal = AXIS_X_POS;
                                dist = 1e-9;
                            } else {
                                c.normal = ballA.pos.minus(ballB.pos).div(dist);
                            }
                            c.ballIndexA = i;
                            c.ballIndexB = j;
                            c.penetration = minDist - dist;
                            c.contactPoint =
                                    ballA.pos.plus(ballB.pos).div(2.0); // midpoint between centers
                            c.restitution = COR_BALL_BALL;
                            c.friction = config.frictionEnabled ? MU_BALL_BALL : 0;
                            c.normalImpulseAccum = 0;
                            c.tangentImpulseAccum = 0;
                            contacts.add(c);

                            // Wake sleeping ball on contact
                            if (ballB.sleeping) {
                                wakeBall(ballB);
                            }
                        }
                    }
                }
            }
        }
    }

    private void generateBallFieldContacts() {
        for (int i = 0; i < balls.size(); i++) {
            SimBall ball = balls.get(i);
            if (ball.intaked || ball.outOfBounds) continue;
            if (ball.sleeping && config.sleepingEnabled) continue;

            for (AABB aabb : AABB_OBSTACLES) {
                generateSphereAABBContact(i, ball, aabb);
            }

            for (CylinderObstacle cyl : CYLINDER_OBSTACLES) {
                generateSphereCylinderContact(i, ball, cyl);
            }
        }
    }

    private void generateSphereAABBContact(int ballIndex, SimBall ball, AABB aabb) {
        double cx = Math.max(aabb.minX(), Math.min(ball.pos.getX(), aabb.maxX()));
        double cy = Math.max(aabb.minY(), Math.min(ball.pos.getY(), aabb.maxY()));
        double cz = Math.max(aabb.minZ(), Math.min(ball.pos.getZ(), aabb.maxZ()));

        double dx = ball.pos.getX() - cx;
        double dy = ball.pos.getY() - cy;
        double dz = ball.pos.getZ() - cz;
        double distSq = dx * dx + dy * dy + dz * dz;

        if (distSq < BALL_RADIUS * BALL_RADIUS && distSq >= 1e-9) {
            double dist = Math.sqrt(distSq);
            Contact c = allocateContact();
            c.ballIndexA = ballIndex;
            c.ballIndexB = -1;
            c.normal = new Translation3d(dx / dist, dy / dist, dz / dist);
            c.penetration = BALL_RADIUS - dist;
            c.contactPoint = new Translation3d(cx, cy, cz);
            c.restitution = aabb.cor();
            c.friction = config.frictionEnabled ? MU_WALL : 0;
            c.normalImpulseAccum = 0;
            c.tangentImpulseAccum = 0;
            contacts.add(c);
            ball.hadFieldContact = true;
        } else if (distSq < 1e-9) {
            // Ball centre is inside the box
            Translation3d normal = computeEntryFaceNormal(ball.prevPos, ball.pos, aabb);
            if (normal != null) {
                double pen = computeAABBPenetration(ball.pos, aabb);
                Contact c = allocateContact();
                c.ballIndexA = ballIndex;
                c.ballIndexB = -1;
                c.normal = normal;
                c.penetration = pen + BALL_RADIUS;
                c.contactPoint = ball.pos;
                c.restitution = aabb.cor();
                c.friction = config.frictionEnabled ? MU_WALL : 0;
                c.normalImpulseAccum = 0;
                c.tangentImpulseAccum = 0;
                contacts.add(c);
            }
        }
    }

    /** Which face of the AABB is the point closest to? Returns outward normal of that face. */
    private Translation3d computeAABBNormal(Translation3d p, AABB aabb) {
        double dxMin = p.getX() - aabb.minX();
        double dxMax = aabb.maxX() - p.getX();
        double dyMin = p.getY() - aabb.minY();
        double dyMax = aabb.maxY() - p.getY();
        double dzMin = p.getZ() - aabb.minZ();
        double dzMax = aabb.maxZ() - p.getZ();

        double min = dxMin;
        Translation3d normal = AXIS_X_NEG;

        if (dxMax < min) {
            min = dxMax;
            normal = AXIS_X_POS;
        }
        if (dyMin < min) {
            min = dyMin;
            normal = AXIS_Y_NEG;
        }
        if (dyMax < min) {
            min = dyMax;
            normal = AXIS_Y_POS;
        }
        if (dzMin < min) {
            min = dzMin;
            normal = AXIS_Z_NEG;
        }
        if (dzMax < min) {
            min = dzMax;
            normal = AXIS_Z_POS;
        }
        return normal;
    }

    /** How deep is a point inside an AABB? Returns min distance to any face. */
    private double computeAABBPenetration(Translation3d p, AABB aabb) {
        double dxMin = p.getX() - aabb.minX();
        double dxMax = aabb.maxX() - p.getX();
        double dyMin = p.getY() - aabb.minY();
        double dyMax = aabb.maxY() - p.getY();
        double dzMin = p.getZ() - aabb.minZ();
        double dzMax = aabb.maxZ() - p.getZ();
        return Math.min(
                Math.min(Math.min(dxMin, dxMax), Math.min(dyMin, dyMax)), Math.min(dzMin, dzMax));
    }

    /**
     * If a ball is inside an AABB, figure out which face it came in through by raycasting from
     * prevPos to pos. Without this, balls pop onto the top of obstacles when they actually entered
     * from the side. Falls back to nearest-face if the ray is degenerate (ball spawned inside).
     */
    private Translation3d computeEntryFaceNormal(Translation3d from, Translation3d to, AABB aabb) {
        if (from == null) return computeAABBNormal(to, aabb);

        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        double dz = to.getZ() - from.getZ();

        // Slab intersection: the entry face is the axis with the latest entry time
        double tMax = Double.NEGATIVE_INFINITY;
        Translation3d bestNormal = null;

        if (Math.abs(dx) > 1e-12) {
            double invD = 1.0 / dx;
            double t1 = (aabb.minX() - from.getX()) * invD;
            double t2 = (aabb.maxX() - from.getX()) * invD;
            double tEntry = Math.min(t1, t2);
            if (tEntry > tMax) {
                tMax = tEntry;
                bestNormal = dx > 0 ? AXIS_X_NEG : AXIS_X_POS;
            }
        }
        if (Math.abs(dy) > 1e-12) {
            double invD = 1.0 / dy;
            double t1 = (aabb.minY() - from.getY()) * invD;
            double t2 = (aabb.maxY() - from.getY()) * invD;
            double tEntry = Math.min(t1, t2);
            if (tEntry > tMax) {
                tMax = tEntry;
                bestNormal = dy > 0 ? AXIS_Y_NEG : AXIS_Y_POS;
            }
        }
        if (Math.abs(dz) > 1e-12) {
            double invD = 1.0 / dz;
            double t1 = (aabb.minZ() - from.getZ()) * invD;
            double t2 = (aabb.maxZ() - from.getZ()) * invD;
            double tEntry = Math.min(t1, t2);
            if (tEntry > tMax) {
                tMax = tEntry;
                bestNormal = dz > 0 ? AXIS_Z_NEG : AXIS_Z_POS;
            }
        }

        return bestNormal != null ? bestNormal : computeAABBNormal(to, aabb);
    }

    private void generateSphereCylinderContact(int ballIndex, SimBall ball, CylinderObstacle cyl) {
        if (cyl.abLenSq() < 1e-12) return;

        double apx = ball.pos.getX() - cyl.ax();
        double apy = ball.pos.getY() - cyl.ay();
        double apz = ball.pos.getZ() - cyl.az();
        double t =
                Math.max(
                        0,
                        Math.min(
                                1,
                                (apx * cyl.abx() + apy * cyl.aby() + apz * cyl.abz())
                                        / cyl.abLenSq()));

        double nearX = cyl.ax() + t * cyl.abx();
        double nearY = cyl.ay() + t * cyl.aby();
        double nearZ = cyl.az() + t * cyl.abz();

        double dx = ball.pos.getX() - nearX;
        double dy = ball.pos.getY() - nearY;
        double dz = ball.pos.getZ() - nearZ;
        double distSq = dx * dx + dy * dy + dz * dz;
        double minDist = BALL_RADIUS + cyl.radius();

        if (distSq < minDist * minDist && distSq > 0) {
            double dist = Math.sqrt(distSq);
            Contact c = allocateContact();
            c.ballIndexA = ballIndex;
            c.ballIndexB = -1;
            c.normal = new Translation3d(dx / dist, dy / dist, dz / dist);
            c.penetration = minDist - dist;
            c.contactPoint = new Translation3d(nearX, nearY, nearZ);
            c.restitution = cyl.cor();
            c.friction = config.frictionEnabled ? MU_WALL : 0;
            c.normalImpulseAccum = 0;
            c.tangentImpulseAccum = 0;
            contacts.add(c);
            ball.hadFieldContact = true;
        }
    }

    /**
     * Sequential impulse solver: resolve bounces, friction, and spin transfer across all contacts.
     */
    private void solveContacts() {
        if (contacts.isEmpty()) return;

        // Compute restitution targets from initial approach velocities (before any solving)
        for (int i = 0; i < contacts.size(); i++) {
            Contact c = contacts.get(i);
            SimBall ballA = balls.get(c.ballIndexA);
            Translation3d relVel;
            if (c.ballIndexB >= 0) {
                relVel = ballA.vel.minus(balls.get(c.ballIndexB).vel);
            } else {
                relVel = ballA.vel;
            }
            double vn = relVel.dot(c.normal);
            double e = c.restitution;
            if (config.velocityDependentCOR) {
                e = velocityCOR(e, Math.abs(vn));
            }
            // Only bounce for significant approach speeds.
            c.restitutionVelocity = vn < -0.5 ? -e * vn : 0;
        }

        for (int iter = 0; iter < config.solverIterations; iter++) {
            for (int i = 0; i < contacts.size(); i++) {
                Contact c = contacts.get(i);
                solveContact(c);
            }
        }

        for (int i = 0; i < contacts.size(); i++) {
            applyPositionCorrection(contacts.get(i));
        }
    }

    private void solveContact(Contact c) {
        SimBall ballA = balls.get(c.ballIndexA);
        Translation3d relVel;
        double invMassSum;

        if (c.ballIndexB >= 0) {
            SimBall ballB = balls.get(c.ballIndexB);
            relVel = ballA.vel.minus(ballB.vel);
            invMassSum = 2.0 / BALL_MASS; // both balls have same mass
        } else {
            relVel = ballA.vel;
            invMassSum = 1.0 / BALL_MASS;
        }

        double vRelNormal = relVel.dot(c.normal);
        double jn = -(vRelNormal - c.restitutionVelocity) / invMassSum;
        double oldAccum = c.normalImpulseAccum;
        c.normalImpulseAccum = Math.max(0, oldAccum + jn);
        jn = c.normalImpulseAccum - oldAccum;

        Translation3d normalImpulse = c.normal.times(jn);
        ballA.vel = ballA.vel.plus(normalImpulse.div(BALL_MASS));
        if (c.ballIndexB >= 0) {
            SimBall ballB = balls.get(c.ballIndexB);
            ballB.vel = ballB.vel.minus(normalImpulse.div(BALL_MASS));
        }

        if (c.friction > 0 && config.frictionEnabled) {
            // Recompute relative velocity after normal impulse
            if (c.ballIndexB >= 0) {
                relVel = ballA.vel.minus(balls.get(c.ballIndexB).vel);
            } else {
                relVel = ballA.vel;
            }

            double vn = relVel.dot(c.normal);
            Translation3d vTangent = relVel.minus(c.normal.times(vn));
            double vTangentMag = vTangent.getNorm();

            if (vTangentMag > 1e-6) {
                Translation3d tangentDir = vTangent.div(vTangentMag);

                double jt = -vTangentMag / invMassSum;

                double maxFriction = c.friction * c.normalImpulseAccum;
                double oldTangentAccum = c.tangentImpulseAccum;
                c.tangentImpulseAccum =
                        Math.max(-maxFriction, Math.min(maxFriction, oldTangentAccum + jt));
                jt = c.tangentImpulseAccum - oldTangentAccum;

                Translation3d frictionImpulse = tangentDir.times(jt);
                ballA.vel = ballA.vel.plus(frictionImpulse.div(BALL_MASS));
                if (c.ballIndexB >= 0) {
                    SimBall ballB = balls.get(c.ballIndexB);
                    ballB.vel = ballB.vel.minus(frictionImpulse.div(BALL_MASS));
                }

                if (config.spinTransferEnabled && Math.abs(jt) > 1e-9) {
                    Translation3d rContact = c.normal.times(-BALL_RADIUS); // from center to contact
                    Translation3d torqueImpulse = cross(rContact, frictionImpulse);
                    Translation3d deltaOmega = torqueImpulse.div(BALL_MOMENT_OF_INERTIA);
                    ballA.omega = ballA.omega.plus(deltaOmega);

                    if (c.ballIndexB >= 0) {
                        SimBall ballB = balls.get(c.ballIndexB);
                        Translation3d rContactB = c.normal.times(BALL_RADIUS);
                        Translation3d torqueImpulseB =
                                cross(rContactB, frictionImpulse.unaryMinus());
                        Translation3d deltaOmegaB = torqueImpulseB.div(BALL_MOMENT_OF_INERTIA);
                        ballB.omega = ballB.omega.plus(deltaOmegaB);
                    }
                }
            }
        }
    }

    private void applyPositionCorrection(Contact c) {
        double slop = config.baumgarteSlop;
        double beta = config.baumgarteBeta;
        double correction = Math.max(c.penetration - slop, 0) * beta;

        if (correction < 1e-6) return;

        SimBall ballA = balls.get(c.ballIndexA);
        if (c.ballIndexB >= 0) {
            SimBall ballB = balls.get(c.ballIndexB);
            Translation3d corrVec = c.normal.times(correction * 0.5);
            ballA.pos = ballA.pos.plus(corrVec);
            ballB.pos = ballB.pos.minus(corrVec);
        } else {
            ballA.pos = ballA.pos.plus(c.normal.times(correction));
        }
    }

    private void handleWallBounce(SimBall ball) {
        double z = ball.pos.getZ();

        // X walls: alliance walls, diamond plate on polycarbonate, height aware
        if (ball.pos.getX() < BALL_RADIUS) {
            if (z < ALLIANCE_WALL_HEIGHT) {
                ball.pos = new Translation3d(BALL_RADIUS, ball.pos.getY(), ball.pos.getZ());
                if (ball.vel.getX() < 0) {
                    applyWallSpinTransfer(ball, AXIS_X_POS);
                    double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getX()));
                    ball.vel =
                            new Translation3d(
                                    -ball.vel.getX() * cor, ball.vel.getY(), ball.vel.getZ());
                }
            }
        } else if (ball.pos.getX() > FIELD_LENGTH - BALL_RADIUS) {
            if (z < ALLIANCE_WALL_HEIGHT) {
                ball.pos =
                        new Translation3d(
                                FIELD_LENGTH - BALL_RADIUS, ball.pos.getY(), ball.pos.getZ());
                if (ball.vel.getX() > 0) {
                    applyWallSpinTransfer(ball, AXIS_X_NEG);
                    double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getX()));
                    ball.vel =
                            new Translation3d(
                                    -ball.vel.getX() * cor, ball.vel.getY(), ball.vel.getZ());
                }
            }
        }

        // Y walls: guardrails, polycarbonate on aluminium extrusion, height aware
        if (ball.pos.getY() < BALL_RADIUS) {
            if (z < GUARDRAIL_HEIGHT) {
                ball.pos = new Translation3d(ball.pos.getX(), BALL_RADIUS, ball.pos.getZ());
                if (ball.vel.getY() < 0) {
                    applyWallSpinTransfer(ball, AXIS_Y_POS);
                    double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getY()));
                    ball.vel =
                            new Translation3d(
                                    ball.vel.getX(), -ball.vel.getY() * cor, ball.vel.getZ());
                }
            }
        } else if (ball.pos.getY() > FIELD_WIDTH - BALL_RADIUS) {
            if (z < GUARDRAIL_HEIGHT) {
                ball.pos =
                        new Translation3d(
                                ball.pos.getX(), FIELD_WIDTH - BALL_RADIUS, ball.pos.getZ());
                if (ball.vel.getY() > 0) {
                    applyWallSpinTransfer(ball, AXIS_Y_NEG);
                    double cor = effectiveCOR(COR_WALL, Math.abs(ball.vel.getY()));
                    ball.vel =
                            new Translation3d(
                                    ball.vel.getX(), -ball.vel.getY() * cor, ball.vel.getZ());
                }
            }
        }
    }

    private void handleGroundContact(SimBall ball, double subDt) {
        if (ball.pos.getZ() < BALL_RADIUS) {
            ball.pos = new Translation3d(ball.pos.getX(), ball.pos.getY(), BALL_RADIUS);

            if (ball.vel.getZ() < 0) {
                double cor = effectiveCOR(COR_CARPET, Math.abs(ball.vel.getZ()));

                // A bounce this small is dropped and the vertical velocity zeroed
                if (Math.abs(ball.vel.getZ() * cor) < 0.05) {
                    ball.vel = new Translation3d(ball.vel.getX(), ball.vel.getY(), 0);
                } else {
                    ball.vel =
                            new Translation3d(
                                    ball.vel.getX(), ball.vel.getY(), -ball.vel.getZ() * cor);
                }
            }

            // Ground friction: uses surface velocity (vel + omega x r) so spin-on-contact works
            if (config.frictionEnabled) {
                double surfVelX = ball.vel.getX() - ball.omega.getY() * BALL_RADIUS;
                double surfVelY = ball.vel.getY() + ball.omega.getX() * BALL_RADIUS;
                double surfSpeed = Math.sqrt(surfVelX * surfVelX + surfVelY * surfVelY);
                double hSpeed =
                        Math.sqrt(
                                ball.vel.getX() * ball.vel.getX()
                                        + ball.vel.getY() * ball.vel.getY());

                if (surfSpeed > 0.01) {
                    double fdx = -surfVelX / surfSpeed;
                    double fdy = -surfVelY / surfSpeed;

                    double maxImpulse = (2.0 / 7.0) * BALL_MASS * surfSpeed;
                    double frictionImpulse =
                            Math.min(MU_GROUND_KINETIC * BALL_MASS * GRAVITY * subDt, maxImpulse);

                    ball.vel =
                            new Translation3d(
                                    ball.vel.getX() + fdx * frictionImpulse / BALL_MASS,
                                    ball.vel.getY() + fdy * frictionImpulse / BALL_MASS,
                                    ball.vel.getZ());
                    if (config.spinTransferEnabled) {
                        ball.omega =
                                new Translation3d(
                                        ball.omega.getX()
                                                + BALL_RADIUS
                                                        * fdy
                                                        * frictionImpulse
                                                        / BALL_MOMENT_OF_INERTIA,
                                        ball.omega.getY()
                                                - BALL_RADIUS
                                                        * fdx
                                                        * frictionImpulse
                                                        / BALL_MOMENT_OF_INERTIA,
                                        ball.omega.getZ());
                    }
                } else if (hSpeed > 1e-4) {
                    // Rolling resistance decelerates vel and omega together
                    double decel = MU_GROUND_ROLLING * GRAVITY * subDt;
                    double scale = Math.max(0, hSpeed - decel) / hSpeed;
                    ball.vel =
                            new Translation3d(
                                    ball.vel.getX() * scale,
                                    ball.vel.getY() * scale,
                                    ball.vel.getZ());
                    if (config.spinTransferEnabled) {
                        ball.omega =
                                new Translation3d(
                                        ball.omega.getX() * scale,
                                        ball.omega.getY() * scale,
                                        ball.omega.getZ());
                    }
                }
            }

            // Settle near-zero vertical velocity when on ground
            if (Math.abs(ball.vel.getZ()) < 0.05 && ball.pos.getZ() <= BALL_RADIUS + 0.01) {
                ball.vel = new Translation3d(ball.vel.getX(), ball.vel.getY(), 0);
            }
        }
    }

    /** Handle collisions with the tent-shaped bump ramps. */
    private void handleBumpCollisions(SimBall ball) {
        for (BumpSegment seg : BUMP_SEGMENTS) {
            if (ball.pos.getY() < seg.yStart() || ball.pos.getY() > seg.yEnd()) continue;

            // Parametric projection onto line segment
            double px = ball.pos.getX() - seg.xStart();
            double pz = ball.pos.getZ() - seg.zStart();
            double t = (px * seg.lineX() + pz * seg.lineZ()) / (seg.lineLen() * seg.lineLen());
            if (t < 0 || t > 1) continue;

            // Distance from ball center to nearest point on line (in XZ plane)
            double nearX = seg.xStart() + t * seg.lineX();
            double nearZ = seg.zStart() + t * seg.lineZ();
            double dx = ball.pos.getX() - nearX;
            double dz = ball.pos.getZ() - nearZ;
            double dist = Math.sqrt(dx * dx + dz * dz);

            if (dist < BALL_RADIUS) {
                double nx = seg.nx(), nz = seg.nz();

                ball.pos =
                        ball.pos.plus(
                                new Translation3d(
                                        nx * (BALL_RADIUS - dist), 0, nz * (BALL_RADIUS - dist)));

                double vDotN = ball.vel.getX() * nx + ball.vel.getZ() * nz;
                if (vDotN < 0) {
                    double cor = effectiveCOR(COR_HDPE, Math.abs(vDotN));
                    ball.vel =
                            ball.vel.minus(
                                    new Translation3d(
                                            nx * (1 + cor) * vDotN, 0, nz * (1 + cor) * vDotN));
                }
            }
        }
    }

    private void handleHubScoring(SimBall ball) {
        if (blueHub.didScore(ball)) {
            ball.pos = blueHub.exit;
            ball.vel = blueHub.getDispersalVelocity(rng);
            ball.omega = new Translation3d();
            blueHub.score++;
            totalScored++;
        } else if (redHub.didScore(ball)) {
            ball.pos = redHub.exit;
            ball.vel = redHub.getDispersalVelocity(rng);
            ball.omega = new Translation3d();
            redHub.score++;
            totalScored++;
        }
    }

    /** Hub net collision: catches overshots, but lets balls pass through from behind. */
    private void handleNetCollision(SimBall ball, ScoringTarget hub) {
        if (ball.pos.getZ() > NET_HEIGHT_MAX || ball.pos.getZ() < NET_HEIGHT_MIN) return;
        if (ball.pos.getY() > hub.center.getY() + NET_WIDTH / 2.0
                || ball.pos.getY() < hub.center.getY() - NET_WIDTH / 2.0) return;

        double netX = hub.center.getX() + NET_OFFSET * hub.exitVelXSign;
        double distToNet = ball.pos.getX() - netX;

        if (Math.abs(distToNet) < BALL_RADIUS) {
            // Only block balls moving toward the net
            boolean movingTowardNet = ball.vel.getX() * hub.exitVelXSign > 0;
            if (!movingTowardNet) return;

            double pushDir = distToNet >= 0 ? 1 : -1;
            ball.pos =
                    new Translation3d(
                            netX + pushDir * BALL_RADIUS, ball.pos.getY(), ball.pos.getZ());
            ball.vel =
                    new Translation3d(
                            -ball.vel.getX() * COR_NET, ball.vel.getY() * COR_NET, ball.vel.getZ());
        }
    }

    private void handleRobotCollision(SimBall ball, Pose2d robotPose, Translation2d robotVel) {
        if (ball.pos.getZ() > bumperHeight) return;

        Translation2d relPos =
                new Pose2d(ball.pos.toTranslation2d(), Rotation2d.kZero)
                        .relativeTo(robotPose)
                        .getTranslation();

        double halfL = robotLength / 2.0 + BALL_RADIUS;
        double halfW = robotWidth / 2.0 + BALL_RADIUS;

        if (relPos.getX() < -halfL
                || relPos.getX() > halfL
                || relPos.getY() < -halfW
                || relPos.getY() > halfW) {
            return;
        }

        double dxMin = relPos.getX() + halfL;
        double dxMax = halfL - relPos.getX();
        double dyMin = relPos.getY() + halfW;
        double dyMax = halfW - relPos.getY();

        double minDist = dxMin;
        Translation2d pushDir = new Translation2d(-1, 0);

        if (dxMax < minDist) {
            minDist = dxMax;
            pushDir = new Translation2d(1, 0);
        }
        if (dyMin < minDist) {
            minDist = dyMin;
            pushDir = new Translation2d(0, -1);
        }
        if (dyMax < minDist) {
            minDist = dyMax;
            pushDir = new Translation2d(0, 1);
        }

        // pushDir is in the robot frame until here.
        pushDir = pushDir.rotateBy(robotPose.getRotation());
        ball.pos = ball.pos.plus(new Translation3d(pushDir.times(minDist)));

        Translation3d normal3d = new Translation3d(pushDir.getX(), pushDir.getY(), 0);
        double vDotN = ball.vel.toTranslation2d().dot(pushDir);
        double robotVDotN = robotVel.dot(pushDir);
        double closingVel = vDotN - robotVDotN;

        if (closingVel < 0) {
            ball.vel = ball.vel.minus(normal3d.times((1 + COR_BUMPER) * closingVel));
        }
    }

    private void handleIntakePickup(SimBall ball, Pose2d robotPose) {
        if (totalIntaked >= hopperSize) return;
        for (IntakeZone intake : intakes) {
            if (intake.shouldIntake(ball, robotPose, bumperHeight)) {
                ball.intaked = true;
                totalIntaked++;
                return;
            }
        }
    }

    private void updateSleepState(SimBall ball) {
        double speed = ball.vel.getNorm();
        double omegaMag = ball.omega.getNorm();

        if (speed < config.sleepVelocityThreshold
                && omegaMag < 0.1
                && ball.pos.getZ() <= BALL_RADIUS + 0.01) {
            ball.sleepCounter++;
            if (ball.sleepCounter >= config.sleepFrameThreshold) {
                ball.sleeping = true;
                ball.vel = new Translation3d();
                ball.omega = new Translation3d();
            }
        } else {
            ball.sleepCounter = 0;
            ball.sleeping = false;
        }
    }

    private void wakeBall(SimBall ball) {
        ball.sleeping = false;
        ball.sleepCounter = 0;
    }

    /** Wake up sleeping balls near a point (so launched balls disturb resting ones). */
    private void wakeNearbyBalls(Translation3d pos, double radius) {
        double radiusSq = radius * radius;
        for (SimBall ball : balls) {
            if (ball.sleeping) {
                double dx = ball.pos.getX() - pos.getX();
                double dy = ball.pos.getY() - pos.getY();
                double dz = ball.pos.getZ() - pos.getZ();
                if (dx * dx + dy * dy + dz * dz < radiusSq) {
                    wakeBall(ball);
                }
            }
        }
    }

    private void computeConservationQuantities() {
        totalKE = 0;
        totalPE = 0;
        double mx = 0, my = 0, mz = 0;

        for (SimBall ball : balls) {
            double speed = ball.vel.getNorm();
            totalKE += 0.5 * BALL_MASS * speed * speed;

            double omegaMag = ball.omega.getNorm();
            totalKE += 0.5 * BALL_MOMENT_OF_INERTIA * omegaMag * omegaMag;

            // Gravitational PE, relative to ground level
            totalPE += BALL_MASS * GRAVITY * ball.pos.getZ();

            mx += BALL_MASS * ball.vel.getX();
            my += BALL_MASS * ball.vel.getY();
            mz += BALL_MASS * ball.vel.getZ();
        }

        totalMomentum = new Translation3d(mx, my, mz);
    }

    /** 3D cross product. Using Translation3d directly to avoid WPILib Vector conversion. */
    private static Translation3d cross(Translation3d a, Translation3d b) {
        return new Translation3d(
                a.getY() * b.getZ() - a.getZ() * b.getY(),
                a.getZ() * b.getX() - a.getX() * b.getZ(),
                a.getX() * b.getY() - a.getY() * b.getX());
    }

    /** COR drops at higher impact speeds (balls deform more and lose more energy). */
    private double velocityCOR(double e0, double impactSpeed) {
        if (!config.velocityDependentCOR) return e0;
        if (impactSpeed <= COR_VREF) return e0;
        return e0 * Math.pow(COR_VREF / impactSpeed, COR_EXPONENT);
    }

    private double effectiveCOR(double baseCOR, double impactSpeed) {
        if (config.velocityDependentCOR) {
            return velocityCOR(baseCOR, impactSpeed);
        }
        return baseCOR;
    }

    /** When a ball hits a wall, friction changes its spin. This handles that. */
    private void applyWallSpinTransfer(SimBall ball, Translation3d wallNormal) {
        if (!config.spinTransferEnabled || !config.frictionEnabled) return;

        Translation3d rContact = wallNormal.times(-BALL_RADIUS);
        Translation3d surfaceVel = ball.vel.plus(cross(ball.omega, rContact));

        double vn = surfaceVel.dot(wallNormal);
        Translation3d vTangent = surfaceVel.minus(wallNormal.times(vn));
        double vTangentMag = vTangent.getNorm();

        if (vTangentMag > 1e-4) {
            // Restitution is folded in: j_n = m * |v_n| * (1 + e)
            double impactSpeed = Math.abs(ball.vel.dot(wallNormal));
            double cor = effectiveCOR(COR_WALL, impactSpeed);
            double normalImpulse = BALL_MASS * impactSpeed * (1.0 + cor);
            double frictionImpulse = Math.min(MU_WALL * normalImpulse, BALL_MASS * vTangentMag);
            Translation3d frictionDir = vTangent.div(vTangentMag).unaryMinus();

            ball.vel = ball.vel.plus(frictionDir.times(frictionImpulse / BALL_MASS));
            Translation3d torqueImpulse = cross(rContact, frictionDir.times(frictionImpulse));
            ball.omega = ball.omega.plus(torqueImpulse.div(BALL_MOMENT_OF_INERTIA));
        }
    }

    /** Grab a contact from the pool (grows the pool if we run out). */
    private Contact allocateContact() {
        if (contactPoolIndex >= contactPool.size()) {
            Contact c = new Contact();
            contactPool.add(c);
            contactPoolIndex++;
            return c;
        }
        return contactPool.get(contactPoolIndex++);
    }

    /** Clean up balls that got eaten by intakes or flew out of bounds. */
    private void removeFlaggedBalls() {
        balls.removeIf(b -> b.intaked || b.outOfBounds);
    }

    /**
     * Predicts where a shot will go (gravity, drag and Magnus, no collisions), integrating with the
     * same forces and substep as the live sim so the drawn arc tracks the ball. Emits one point
     * every {@code sampleDt} seconds and stops at the ground.
     */
    private Translation3d[] predictArc(
            Translation3d pos,
            Translation3d vel,
            Translation3d omega,
            int maxPoints,
            double sampleDt) {
        double subDt = PERIOD / Math.max(1, config.subticks);
        int stepsPerPoint = Math.max(1, (int) Math.round(sampleDt / subDt));

        List<Translation3d> arc = new ArrayList<>();
        arc.add(pos);
        double px = pos.getX(), py = pos.getY(), pz = pos.getZ();
        double vx = vel.getX(), vy = vel.getY(), vz = vel.getZ();
        double wx = omega.getX(), wy = omega.getY(), wz = omega.getZ();
        boolean landed = false;

        for (int p = 0; p < maxPoints && !landed; p++) {
            for (int s = 0; s < stepsPerPoint; s++) {
                double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
                double ax = 0, ay = 0, az = -GRAVITY;

                if (config.dragEnabled && speed > 1e-6) {
                    ax -= DRAG_ACCEL_FACTOR * speed * vx;
                    ay -= DRAG_ACCEL_FACTOR * speed * vy;
                    az -= DRAG_ACCEL_FACTOR * speed * vz;
                }

                // Matches computeAcceleration: Magnus only once airborne
                if (config.magnusEnabled
                        && speed > 1e-6
                        && pz > BALL_RADIUS + 0.01
                        && (wx * wx + wy * wy + wz * wz) > 1e-6) {
                    ax += MAGNUS_ACCEL_FACTOR * (wy * vz - wz * vy);
                    ay += MAGNUS_ACCEL_FACTOR * (wz * vx - wx * vz);
                    az += MAGNUS_ACCEL_FACTOR * (wx * vy - wy * vx);
                }

                // Symplectic Euler, same ordering as stepSubtick
                vx += ax * subDt;
                vy += ay * subDt;
                vz += az * subDt;
                px += vx * subDt;
                py += vy * subDt;
                pz += vz * subDt;

                if (config.spinDecayEnabled) {
                    double decay = Math.exp(-subDt / config.spinDecayTau);
                    wx *= decay;
                    wy *= decay;
                    wz *= decay;
                }

                if (pz < BALL_RADIUS) {
                    landed = true;
                    break;
                }
            }
            if (!landed) {
                arc.add(new Translation3d(px, py, pz));
            }
        }
        return arc.toArray(new Translation3d[0]);
    }

    /** Push ball positions and sim stats to NetworkTables for visualization. */
    public void publishPositions() {
        Translation3d[] positions = new Translation3d[balls.size()];
        for (int i = 0; i < balls.size(); i++) {
            positions[i] = balls.get(i).pos;
        }
        positionPublisher.set(positions);

        // In-flight positions only, so a Field3d can render them separately
        List<Translation3d> inFlight = new ArrayList<>();
        int sleeping = 0;
        for (SimBall b : balls) {
            if (b.intaked || b.outOfBounds) continue;
            if (b.pos.getZ() > BALL_RADIUS + 0.1) {
                inFlight.add(b.pos);
            }
            if (b.sleeping) sleeping++;
        }
        inFlightPublisher.set(inFlight.toArray(new Translation3d[0]));

        // Last shot arc, the predicted trajectory, visible in Field3d
        lastShotArcPublisher.set(lastShotArc);

        blueScorePub.set(blueHub.score);
        redScorePub.set(redHub.score);

        ballCountPub.set(balls.size());
        activeBallsPub.set(balls.size() - sleeping);
        sleepingBallsPub.set(sleeping);
        contactCountPub.set(contacts.size());
        physicsTimePub.set(lastPhysicsNanos / 1_000_000.0);
        computeConservationQuantities();
        totalEnergyPub.set(totalKE + totalPE);
    }

    public int getBallCount() {
        return balls.size();
    }

    /** Counts balls whose centre clears the ground by more than 0.1 m. */
    public int getBallsInFlight() {
        int count = 0;
        for (SimBall ball : balls) {
            if (ball.pos.getZ() > BALL_RADIUS + 0.1) count++;
        }
        return count;
    }

    public int getBallsOnGround() {
        int count = 0;
        for (SimBall ball : balls) {
            if (ball.pos.getZ() <= BALL_RADIUS + 0.1) count++;
        }
        return count;
    }

    public List<Translation3d> getBallPositions() {
        List<Translation3d> positions = new ArrayList<>(balls.size());
        for (SimBall ball : balls) {
            positions.add(ball.pos);
        }
        return positions;
    }

    public List<Translation3d> getBallVelocities() {
        List<Translation3d> velocities = new ArrayList<>(balls.size());
        for (SimBall ball : balls) {
            velocities.add(ball.vel);
        }
        return velocities;
    }

    public List<Translation3d> getBallOmegas() {
        List<Translation3d> omegas = new ArrayList<>(balls.size());
        for (SimBall ball : balls) {
            omegas.add(ball.omega);
        }
        return omegas;
    }

    public PhysicsConfig getConfig() {
        return config;
    }

    public void setConfig(PhysicsConfig config) {
        this.config = config;
        if (config.deterministic) {
            this.rng = new Random(config.deterministicSeed);
        }
    }

    /** Lock the RNG seed so tests are repeatable. */
    public void setDeterministic(long seed) {
        config.deterministic = true;
        config.deterministicSeed = seed;
        this.rng = new Random(seed);
    }

    /** Total kinetic energy in joules, translational plus rotational. */
    public double getTotalKineticEnergy() {
        computeConservationQuantities();
        return totalKE;
    }

    /** Total potential energy in joules, measured from ground level. */
    public double getTotalPotentialEnergy() {
        computeConservationQuantities();
        return totalPE;
    }

    /** Total momentum in kg·m/s. */
    public Translation3d getTotalMomentum() {
        computeConservationQuantities();
        return totalMomentum;
    }

    public int getTotalLaunched() {
        return totalLaunched;
    }

    public int getTotalScored() {
        return totalScored;
    }

    public int getTotalIntaked() {
        return totalIntaked;
    }

    public double getLastLaunchSpeed() {
        return lastLaunchSpeed;
    }

    public int getBlueScore() {
        return blueHub.score;
    }

    public int getRedScore() {
        return redHub.score;
    }

    public ScoringTarget getBlueHub() {
        return blueHub;
    }

    public ScoringTarget getRedHub() {
        return redHub;
    }

    // Package-private, for tests
    List<SimBall> getBalls() {
        return balls;
    }

    public int getSleepingBallCount() {
        int count = 0;
        for (SimBall ball : balls) {
            if (ball.sleeping) count++;
        }
        return count;
    }

    public static double getFieldLength() {
        return FIELD_LENGTH;
    }

    public static double getFieldWidth() {
        return FIELD_WIDTH;
    }

    public static double getBallRadius() {
        return BALL_RADIUS;
    }

    public static double getBallMass() {
        return BALL_MASS;
    }

    public static double getMomentOfInertia() {
        return BALL_MOMENT_OF_INERTIA;
    }

    public static double getDragAccelFactor() {
        return DRAG_ACCEL_FACTOR;
    }

    public static double getMagnusAccelFactor() {
        return MAGNUS_ACCEL_FACTOR;
    }

    public static double getFieldCOR() {
        return COR_CARPET;
    }

    public static double getBallBallCOR() {
        return COR_BALL_BALL;
    }

    public void resetCounters() {
        totalLaunched = 0;
        totalScored = 0;
        totalIntaked = 0;
        lastLaunchSpeed = 0;
        blueHub.resetScore();
        redHub.resetScore();
    }
}
