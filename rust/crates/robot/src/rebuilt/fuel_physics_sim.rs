// Port of frc.rebuilt.FuelPhysicsSim.
//
// MIT License
//
// Copyright (c) 2026 FRC Team 5962 perSEVERE
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

//! Simulates fuel on the 2026 REBUILT field.
//!
//! Symplectic Euler integration, sequential impulse contacts with warm starting and
//! Baumgarte stabilization, spatial hash broadphase, and ball sleeping. Sleeping keeps 350 or
//! more resting balls cheap per tick.

use std::f64::consts::PI;
use std::sync::LazyLock;
use std::time::{Instant, SystemTime, UNIX_EPOCH};
use wpilib::nt::{Double, Instance, Integer, Options, Publisher, StructArrayKind};
use wpimath::geometry::{Pose2d, Rotation2d, Translation2d, Translation3d};
use wpimath::kinematics::ChassisSpeeds;

#[cfg(test)]
#[path = "fuel_physics_sim/tests.rs"]
mod tests;

const GRAVITY: f64 = 9.81; // m/s^2
const AIR_DENSITY: f64 = 1.225; // kg/m^3, standard atmosphere
const BALL_MASS: f64 = 0.215; // kg, game manual 5.10.1 midpoint
const BALL_DIAMETER: f64 = 0.1501; // m, game manual 5.10.1
pub const BALL_RADIUS: f64 = BALL_DIAMETER / 2.0;
const BALL_CROSS_AREA: f64 = PI * BALL_RADIUS * BALL_RADIUS;
const BALL_MOMENT_OF_INERTIA: f64 = 0.4 * BALL_MASS * BALL_RADIUS * BALL_RADIUS; // solid sphere

const DEFAULT_CD: f64 = 0.47; // drag coefficient, smooth sphere
const DEFAULT_CM: f64 = 0.2; // Magnus coefficient, conservative estimate

// Force factors, already divided by ball mass so they apply straight to acceleration
const DRAG_ACCEL_FACTOR: f64 = 0.5 * AIR_DENSITY * DEFAULT_CD * BALL_CROSS_AREA / BALL_MASS;
// The extra BALL_RADIUS turns the omega x v cross product into an acceleration
const MAGNUS_ACCEL_FACTOR: f64 =
    0.5 * AIR_DENSITY * DEFAULT_CM * BALL_CROSS_AREA * BALL_RADIUS / BALL_MASS;

// Restitution per material, from the field element build instructions
const COR_CARPET: f64 = 0.65; // foam on low-pile carpet
const COR_WALL: f64 = 0.70; // foam on polycarbonate (alliance walls, guardrails)
const COR_STEEL: f64 = 0.72; // foam on powder-coated steel (tower, rungs)
const COR_HUB: f64 = 0.70; // foam on polycarbonate (hub body panels)
const COR_HDPE: f64 = 0.60; // foam on textured HDPE (bump ramps, 15deg)
const COR_NET: f64 = 0.15; // mesh fabric absorbs most of the energy
const COR_BUMPER: f64 = 0.08; // polycarb-backed foam, nearly inelastic
const COR_BALL_BALL: f64 = 0.45; // foam-on-foam, high deformation loss

const MU_GROUND_KINETIC: f64 = 0.3; // kinetic friction on carpet
const MU_GROUND_ROLLING: f64 = 0.05; // rolling friction
const MU_WALL: f64 = 0.4; // foam on polycarbonate/fabric
const MU_BALL_BALL: f64 = 0.3; // foam on foam

const COR_VREF: f64 = 3.0; // m/s
const COR_EXPONENT: f64 = 0.15;

const AXIS_X_POS: Translation3d = Translation3d::new(1.0, 0.0, 0.0);
const AXIS_X_NEG: Translation3d = Translation3d::new(-1.0, 0.0, 0.0);
const AXIS_Y_POS: Translation3d = Translation3d::new(0.0, 1.0, 0.0);
const AXIS_Y_NEG: Translation3d = Translation3d::new(0.0, -1.0, 0.0);
const AXIS_Z_POS: Translation3d = Translation3d::new(0.0, 0.0, 1.0);
const AXIS_Z_NEG: Translation3d = Translation3d::new(0.0, 0.0, -1.0);

const PERIOD: f64 = 0.02; // 20ms

const FIELD_LENGTH: f64 = 16.541; // m
const FIELD_WIDTH: f64 = 8.052; // m

const ALLIANCE_WALL_HEIGHT: f64 = 0.935; // 36.8 in
const GUARDRAIL_HEIGHT: f64 = 0.508; // 20 in

const BUMP_HEIGHT: f64 = 0.165; // m, 6.513 in

// Hub positions, from the game manual field layout
const BLUE_HUB_CENTER: Translation2d = Translation2d::new(4.5974, 4.035);
const RED_HUB_CENTER: Translation2d = Translation2d::new(11.938, 4.035);

const HUB_ENTRY_HEIGHT: f64 = 1.829; // m, 72 in
const HUB_ENTRY_RADIUS: f64 = 0.5295; // m, 41.7 in / 2 across flats
const HUB_SIDE: f64 = 1.194; // m, 47 in square base
// Deflects ground-level balls; not a solid wall up to scoring height
const HUB_BASE_HEIGHT: f64 = 0.5; // m, about 20 in

const NET_HEIGHT_MIN: f64 = 1.5; // m
const NET_HEIGHT_MAX: f64 = 3.057; // m
const NET_WIDTH: f64 = 1.484; // m
const NET_OFFSET: f64 = HUB_SIDE / 2.0 + 0.261;

const TRENCH_WIDTH: f64 = 1.265; // m
const TRENCH_BLOCK_WIDTH: f64 = 0.305; // m, 12 in
const TRENCH_HEIGHT: f64 = 0.565; // m, 22.25 in underpass
const TRENCH_PILLAR_HEIGHT: f64 = 1.346; // m, 53 in

const TOWER_POLE_WIDTH: f64 = 0.051; // m, 2 in
const TOWER_POLE_HEIGHT: f64 = 1.194; // m, 47 in
const TOWER_UPRIGHT_THICK: f64 = 0.038; // m, 1.5 in
const TOWER_UPRIGHT_DEPTH: f64 = 0.089; // m, 3.5 in
const TOWER_UPRIGHT_HEIGHT: f64 = 1.831; // m, 72.1 in
const TOWER_UPRIGHT_SPACING: f64 = 0.819; // m, 32.25 in center-to-center

const RUNG_OUTER_DIAMETER: f64 = 0.042; // m, 1.66 in OD
const RUNG_RADIUS: f64 = RUNG_OUTER_DIAMETER / 2.0;
const RUNG_OVERHANG: f64 = 0.149; // m, 5.875 in past upright
const RUNG_LOW_HEIGHT: f64 = 0.686; // m, 27 in
const RUNG_MID_HEIGHT: f64 = 1.143; // m, 45 in
const RUNG_HIGH_HEIGHT: f64 = 1.600; // m, 63 in

const BRACING_BOTTOM: f64 = 0.721; // m, 28.4 in
const BRACING_TOP: f64 = 1.102; // m, 43.4 in

const HUB_RAMP_WIDTH: f64 = 1.194; // m, 47 in
const HUB_RAMP_LENGTH: f64 = 5.512; // m, 217 in
const HUB_RAMP_HEIGHT: f64 = 0.165; // m, 6.5 in

const CELL_SIZE: f64 = 0.25;
const GRID_COLS: usize = ceil_to_usize(FIELD_LENGTH / CELL_SIZE);
const GRID_ROWS: usize = ceil_to_usize(FIELD_WIDTH / CELL_SIZE);

const MAX_BALLS: usize = 2000;

const fn ceil_to_usize(value: f64) -> usize {
    let truncated = value as usize;
    if (truncated as f64) < value {
        truncated + 1
    } else {
        truncated
    }
}

/// XZ line segment in meters, present only over the `y_start` to `y_end` range.
struct BumpSegment {
    x_start: f64,
    z_start: f64,
    y_start: f64,
    y_end: f64,
    line_x: f64,
    line_z: f64,
    line_len: f64,
    nx: f64,
    nz: f64,
}

impl BumpSegment {
    fn new(x_start: f64, z_start: f64, x_end: f64, z_end: f64, y_start: f64, y_end: f64) -> Self {
        let hypot = (x_end - x_start).hypot(z_end - z_start);
        Self {
            x_start,
            z_start,
            y_start,
            y_end,
            line_x: x_end - x_start,
            line_z: z_end - z_start,
            line_len: hypot,
            // Normal in XZ, flipped so nz >= 0 so it points away from the ground
            nx: if (x_end - x_start) >= 0.0 {
                -(z_end - z_start) / hypot
            } else {
                (z_end - z_start) / hypot
            },
            nz: (x_end - x_start).abs() / hypot,
        }
    }
}

static BUMP_SEGMENTS: LazyLock<[BumpSegment; 8]> = LazyLock::new(|| {
    [
        // Blue lower bump ramp up/down
        BumpSegment::new(3.96, 0.0, 4.524, BUMP_HEIGHT, 1.88, 3.73),
        BumpSegment::new(4.524, BUMP_HEIGHT, 5.088, 0.0, 1.88, 3.73),
        // Blue upper bump ramp up/down
        BumpSegment::new(3.96, 0.0, 4.524, BUMP_HEIGHT, 4.32, 6.17),
        BumpSegment::new(4.524, BUMP_HEIGHT, 5.088, 0.0, 4.32, 6.17),
        // Red lower bump ramp up/down
        BumpSegment::new(
            FIELD_LENGTH - 5.088,
            0.0,
            FIELD_LENGTH - 4.524,
            BUMP_HEIGHT,
            1.88,
            3.73,
        ),
        BumpSegment::new(
            FIELD_LENGTH - 4.524,
            BUMP_HEIGHT,
            FIELD_LENGTH - 3.96,
            0.0,
            1.88,
            3.73,
        ),
        // Red upper bump ramp up/down
        BumpSegment::new(
            FIELD_LENGTH - 5.088,
            0.0,
            FIELD_LENGTH - 4.524,
            BUMP_HEIGHT,
            4.32,
            6.17,
        ),
        BumpSegment::new(
            FIELD_LENGTH - 4.524,
            BUMP_HEIGHT,
            FIELD_LENGTH - 3.96,
            0.0,
            4.32,
            6.17,
        ),
    ]
});

struct Aabb {
    min_x: f64,
    min_y: f64,
    min_z: f64,
    max_x: f64,
    max_y: f64,
    max_z: f64,
    cor: f64,
}

impl Aabb {
    const fn new(
        min_x: f64,
        min_y: f64,
        min_z: f64,
        max_x: f64,
        max_y: f64,
        max_z: f64,
        cor: f64,
    ) -> Self {
        Self {
            min_x,
            min_y,
            min_z,
            max_x,
            max_y,
            max_z,
            cor,
        }
    }
}

struct CylinderObstacle {
    a: Translation3d,
    radius: f64,
    cor: f64,
    ab: Translation3d,
    ab_len_sq: f64,
}

impl CylinderObstacle {
    fn new(a: Translation3d, b: Translation3d, radius: f64, cor: f64) -> Self {
        Self {
            a,
            radius,
            cor,
            ab: Translation3d::new(b.x - a.x, b.y - a.y, b.z - a.z),
            ab_len_sq: (b.x - a.x) * (b.x - a.x)
                + (b.y - a.y) * (b.y - a.y)
                + (b.z - a.z) * (b.z - a.z),
        }
    }
}

// The order of this list sets the order contacts are generated and CCD ties are broken.
static AABB_OBSTACLES: LazyLock<Vec<Aabb>> = LazyLock::new(|| {
    let mut aabbs = Vec::with_capacity(20);

    // Trench pillars (4)
    aabbs.push(Aabb::new(
        3.96,
        TRENCH_WIDTH,
        0.0,
        3.96 + TRENCH_BLOCK_WIDTH,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        3.96,
        FIELD_WIDTH - 1.57 - TRENCH_BLOCK_WIDTH,
        0.0,
        3.96 + TRENCH_BLOCK_WIDTH,
        FIELD_WIDTH - 1.57,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        FIELD_LENGTH - 3.96 - TRENCH_BLOCK_WIDTH,
        TRENCH_WIDTH,
        0.0,
        FIELD_LENGTH - 3.96,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        FIELD_LENGTH - 3.96 - TRENCH_BLOCK_WIDTH,
        FIELD_WIDTH - 1.57 - TRENCH_BLOCK_WIDTH,
        0.0,
        FIELD_LENGTH - 3.96,
        FIELD_WIDTH - 1.57,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));

    // Trench ceilings (4)
    let trench_x_blue1 = 3.96;
    let trench_x_blue2 = 5.18;
    let trench_x_red1 = FIELD_LENGTH - 5.18;
    let trench_x_red2 = FIELD_LENGTH - 3.96;
    aabbs.push(Aabb::new(
        trench_x_blue1,
        1.57,
        TRENCH_HEIGHT,
        trench_x_blue2,
        3.73,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        trench_x_blue1,
        FIELD_WIDTH - 3.73,
        TRENCH_HEIGHT,
        trench_x_blue2,
        FIELD_WIDTH - 1.57,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        trench_x_red1,
        1.57,
        TRENCH_HEIGHT,
        trench_x_red2,
        3.73,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        trench_x_red1,
        FIELD_WIDTH - 3.73,
        TRENCH_HEIGHT,
        trench_x_red2,
        FIELD_WIDTH - 1.57,
        TRENCH_PILLAR_HEIGHT,
        COR_STEEL,
    ));

    // Tower poles (2)
    let blue_tower_x = 1.067;
    let red_tower_x = 15.494;
    let blue_tower_y = 4.039;
    let red_tower_y = 4.318;
    aabbs.push(Aabb::new(
        blue_tower_x - TOWER_POLE_WIDTH / 2.0,
        blue_tower_y - TOWER_POLE_WIDTH / 2.0,
        0.0,
        blue_tower_x + TOWER_POLE_WIDTH / 2.0,
        blue_tower_y + TOWER_POLE_WIDTH / 2.0,
        TOWER_POLE_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        red_tower_x - TOWER_POLE_WIDTH / 2.0,
        red_tower_y - TOWER_POLE_WIDTH / 2.0,
        0.0,
        red_tower_x + TOWER_POLE_WIDTH / 2.0,
        red_tower_y + TOWER_POLE_WIDTH / 2.0,
        TOWER_POLE_HEIGHT,
        COR_STEEL,
    ));

    // Tower uprights (4)
    let half_spacing = TOWER_UPRIGHT_SPACING / 2.0;
    aabbs.push(Aabb::new(
        0.0,
        blue_tower_y - half_spacing - TOWER_UPRIGHT_THICK / 2.0,
        0.0,
        TOWER_UPRIGHT_DEPTH,
        blue_tower_y - half_spacing + TOWER_UPRIGHT_THICK / 2.0,
        TOWER_UPRIGHT_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        0.0,
        blue_tower_y + half_spacing - TOWER_UPRIGHT_THICK / 2.0,
        0.0,
        TOWER_UPRIGHT_DEPTH,
        blue_tower_y + half_spacing + TOWER_UPRIGHT_THICK / 2.0,
        TOWER_UPRIGHT_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
        red_tower_y - half_spacing - TOWER_UPRIGHT_THICK / 2.0,
        0.0,
        FIELD_LENGTH,
        red_tower_y - half_spacing + TOWER_UPRIGHT_THICK / 2.0,
        TOWER_UPRIGHT_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
        red_tower_y + half_spacing - TOWER_UPRIGHT_THICK / 2.0,
        0.0,
        FIELD_LENGTH,
        red_tower_y + half_spacing + TOWER_UPRIGHT_THICK / 2.0,
        TOWER_UPRIGHT_HEIGHT,
        COR_STEEL,
    ));

    // Tower bracing (2)
    aabbs.push(Aabb::new(
        0.0,
        blue_tower_y - half_spacing,
        BRACING_BOTTOM,
        TOWER_UPRIGHT_DEPTH,
        blue_tower_y + half_spacing,
        BRACING_TOP,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        FIELD_LENGTH - TOWER_UPRIGHT_DEPTH,
        red_tower_y - half_spacing,
        BRACING_BOTTOM,
        FIELD_LENGTH,
        red_tower_y + half_spacing,
        BRACING_TOP,
        COR_STEEL,
    ));

    // Hub ramp colliders (2)
    aabbs.push(Aabb::new(
        BLUE_HUB_CENTER.x - HUB_RAMP_WIDTH / 2.0,
        BLUE_HUB_CENTER.y - HUB_RAMP_LENGTH / 2.0,
        0.0,
        BLUE_HUB_CENTER.x + HUB_RAMP_WIDTH / 2.0,
        BLUE_HUB_CENTER.y + HUB_RAMP_LENGTH / 2.0,
        HUB_RAMP_HEIGHT,
        COR_STEEL,
    ));
    aabbs.push(Aabb::new(
        RED_HUB_CENTER.x - HUB_RAMP_WIDTH / 2.0,
        RED_HUB_CENTER.y - HUB_RAMP_LENGTH / 2.0,
        0.0,
        RED_HUB_CENTER.x + HUB_RAMP_WIDTH / 2.0,
        RED_HUB_CENTER.y + HUB_RAMP_LENGTH / 2.0,
        HUB_RAMP_HEIGHT,
        COR_STEEL,
    ));

    // Hub body panels
    aabbs.push(Aabb::new(
        BLUE_HUB_CENTER.x - HUB_SIDE / 2.0,
        BLUE_HUB_CENTER.y - HUB_SIDE / 2.0,
        0.0,
        BLUE_HUB_CENTER.x + HUB_SIDE / 2.0,
        BLUE_HUB_CENTER.y + HUB_SIDE / 2.0,
        HUB_BASE_HEIGHT,
        COR_HUB,
    ));
    aabbs.push(Aabb::new(
        RED_HUB_CENTER.x - HUB_SIDE / 2.0,
        RED_HUB_CENTER.y - HUB_SIDE / 2.0,
        0.0,
        RED_HUB_CENTER.x + HUB_SIDE / 2.0,
        RED_HUB_CENTER.y + HUB_SIDE / 2.0,
        HUB_BASE_HEIGHT,
        COR_HUB,
    ));

    aabbs
});

// Tower rungs
static CYLINDER_OBSTACLES: LazyLock<Vec<CylinderObstacle>> = LazyLock::new(|| {
    let mut cyls = Vec::with_capacity(6);
    let blue_tower_y = 4.039;
    let red_tower_y = 4.318;
    let half_spacing = TOWER_UPRIGHT_SPACING / 2.0;
    let heights = [RUNG_LOW_HEIGHT, RUNG_MID_HEIGHT, RUNG_HIGH_HEIGHT];

    // Blue tower rungs (3)
    for h in heights {
        cyls.push(CylinderObstacle::new(
            Translation3d::new(0.0, blue_tower_y - half_spacing - RUNG_OVERHANG, h),
            Translation3d::new(0.0, blue_tower_y + half_spacing + RUNG_OVERHANG, h),
            RUNG_RADIUS,
            COR_STEEL,
        ));
    }

    // Red tower rungs (3)
    for h in heights {
        cyls.push(CylinderObstacle::new(
            Translation3d::new(FIELD_LENGTH, red_tower_y - half_spacing - RUNG_OVERHANG, h),
            Translation3d::new(FIELD_LENGTH, red_tower_y + half_spacing + RUNG_OVERHANG, h),
            RUNG_RADIUS,
            COR_STEEL,
        ));
    }

    cyls
});

/// Physics feature toggles, for isolating terms when debugging.
#[derive(Clone, Debug)]
pub struct PhysicsConfig {
    pub drag_enabled: bool,
    pub magnus_enabled: bool,
    pub friction_enabled: bool,
    pub spin_transfer_enabled: bool,
    pub sleeping_enabled: bool,
    pub ccd_enabled: bool,
    pub velocity_dependent_cor: bool,
    pub spin_decay_enabled: bool,
    pub solver_iterations: i32,
    pub subticks: i32,
    pub spin_decay_tau: f64,           // seconds
    pub sleep_velocity_threshold: f64, // m/s
    pub sleep_frame_threshold: i32,    // consecutive frames
    pub ccd_speed_threshold: f64,      // m/s
    pub baumgarte_beta: f64,
    pub baumgarte_slop: f64, // 5mm allowed penetration
    pub deterministic: bool,
    pub deterministic_seed: i64,
    pub conservation_monitor: bool,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {
            drag_enabled: true,
            magnus_enabled: true,
            friction_enabled: true,
            spin_transfer_enabled: true,
            sleeping_enabled: true,
            ccd_enabled: true,
            velocity_dependent_cor: true,
            spin_decay_enabled: true,
            solver_iterations: 4,
            subticks: 5,
            spin_decay_tau: 3.0,
            sleep_velocity_threshold: 0.01,
            sleep_frame_threshold: 10,
            ccd_speed_threshold: 10.0,
            baumgarte_beta: 0.2,
            baumgarte_slop: 0.005,
            deterministic: false,
            deterministic_seed: 42,
            conservation_monitor: false,
        }
    }
}

/// `java.util.Random`, so seeded runs match the Java sim.
struct JavaRandom {
    seed: i64,
}

impl JavaRandom {
    const MULTIPLIER: i64 = 0x5DEECE66D;
    const MASK: i64 = (1 << 48) - 1;

    fn new(seed: i64) -> Self {
        Self {
            seed: (seed ^ Self::MULTIPLIER) & Self::MASK,
        }
    }

    /// Java's `new Random()`: any seed that differs per run.
    fn from_time() -> Self {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos());
        Self::new(nanos as i64)
    }

    fn next(&mut self, bits: u32) -> i32 {
        self.seed = self.seed.wrapping_mul(Self::MULTIPLIER).wrapping_add(0xB) & Self::MASK;
        (self.seed >> (48 - bits)) as i32
    }

    fn next_double(&mut self) -> f64 {
        let high = i64::from(self.next(26)) << 27;
        let low = i64::from(self.next(27));
        (high + low) as f64 * (1.0 / (1_u64 << 53) as f64)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SimBall {
    pos: Translation3d,   // field-frame position (m)
    vel: Translation3d,   // field-frame velocity (m/s)
    omega: Translation3d, // angular velocity (rad/s), 3D spin axis

    // Previous sub-tick position, used for CCD and scoring detection
    prev_pos: Translation3d,

    sleeping: bool,
    sleep_counter: i32,

    // Stuck-on-obstacle detection
    elevated_slow_counter: i32,

    intaked: bool,
    out_of_bounds: bool,
}

impl SimBall {
    fn new(pos: Translation3d, vel: Translation3d, omega: Translation3d) -> Self {
        Self {
            pos,
            vel,
            omega,
            prev_pos: pos,
            sleeping: false,
            sleep_counter: 0,
            elevated_slow_counter: 0,
            intaked: false,
            out_of_bounds: false,
        }
    }

    pub fn pos(&self) -> Translation3d {
        self.pos
    }

    pub fn vel(&self) -> Translation3d {
        self.vel
    }

    pub fn omega(&self) -> Translation3d {
        self.omega
    }

    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }
}

/// One contact between two balls, or between a ball and the field geometry.
#[derive(Clone, Copy)]
struct Contact {
    ball_index_a: usize,
    ball_index_b: Option<usize>, // None for field geometry
    normal: Translation3d,       // contact normal (A -> B or outward from field)
    penetration: f64,            // overlap depth (positive = overlapping)
    normal_impulse_accum: f64,   // warm-start accumulator for the normal impulse
    tangent_impulse_accum: f64,  // warm-start accumulator for the friction impulse
    restitution: f64,            // effective COR for this pair
    friction: f64,               // Coulomb mu for this pair
    restitution_velocity: f64,   // target separating speed, set once before the solver runs
}

impl Contact {
    fn new(
        ball_index_a: usize,
        ball_index_b: Option<usize>,
        normal: Translation3d,
        penetration: f64,
        restitution: f64,
        friction: f64,
    ) -> Self {
        Self {
            ball_index_a,
            ball_index_b,
            normal,
            penetration,
            normal_impulse_accum: 0.0,
            tangent_impulse_accum: 0.0,
            restitution,
            friction,
            restitution_velocity: 0.0,
        }
    }
}

pub struct ScoringTarget {
    center: Translation2d,
    exit: Translation3d,
    exit_vel_x_sign: i32, // +1 for blue (exits toward red), -1 for red
    score: i32,
}

impl ScoringTarget {
    fn new(center: Translation2d, exit: Translation3d, exit_vel_x_sign: i32) -> Self {
        Self {
            center,
            exit,
            exit_vel_x_sign,
            score: 0,
        }
    }

    fn did_score(&self, ball: &SimBall) -> bool {
        let dist2d = ball.pos.to_translation2d().distance(self.center);
        if dist2d > HUB_ENTRY_RADIUS {
            return false;
        }
        let curr_z = ball.pos.z;
        let prev_z = ball.prev_pos.z;
        // Count only top-down entries, not balls rising through the opening
        prev_z > HUB_ENTRY_HEIGHT && curr_z <= HUB_ENTRY_HEIGHT
    }

    fn get_dispersal_velocity(&self, rng: &mut JavaRandom) -> Translation3d {
        let vx = f64::from(self.exit_vel_x_sign) * (rng.next_double() + 0.1) * 1.5;
        let vy = rng.next_double() * 2.0 - 1.0;
        Translation3d::new(vx, vy, 0.0)
    }

    pub fn get_score(&self) -> i32 {
        self.score
    }

    fn reset_score(&mut self) {
        self.score = 0;
    }
}

/// Intake box in robot-relative coordinates. Balls inside it get picked up.
struct IntakeZone {
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    active: Box<dyn Fn() -> bool>,
    callback: Box<dyn Fn()>,
}

impl IntakeZone {
    fn should_intake(&self, ball: &SimBall, robot_pose: Pose2d, bumper_height: f64) -> bool {
        if !(self.active)() || ball.pos.z > bumper_height {
            return false;
        }
        let rel_pos = Pose2d::new(ball.pos.to_translation2d(), Rotation2d::ZERO)
            .relative_to(robot_pose)
            .translation;
        let inside = rel_pos.x >= self.x_min
            && rel_pos.x <= self.x_max
            && rel_pos.y >= self.y_min
            && rel_pos.y <= self.y_max;
        if inside {
            (self.callback)();
        }
        inside
    }
}

struct Publishers {
    positions: Publisher<StructArrayKind<Translation3d>>,
    in_flight: Publisher<StructArrayKind<Translation3d>>,
    last_shot_arc: Publisher<StructArrayKind<Translation3d>>,
    blue_score: Publisher<Integer>,
    red_score: Publisher<Integer>,
    ball_count: Publisher<Integer>,
    active_balls: Publisher<Integer>,
    sleeping_balls: Publisher<Integer>,
    contact_count: Publisher<Integer>,
    physics_time: Publisher<Double>,
    total_energy: Publisher<Double>,
}

impl Publishers {
    fn new(table_key: &str) -> Self {
        let nt = Instance::default_instance();
        let options = Options::default();
        let arrays = |name: &str| {
            nt.topic::<StructArrayKind<Translation3d>>(&format!("{table_key}/{name}"))
                .publish(&options)
        };
        let integer = |name: &str| {
            nt.topic::<Integer>(&format!("{table_key}/{name}"))
                .publish(&options)
        };
        let double = |name: &str| {
            nt.topic::<Double>(&format!("{table_key}/{name}"))
                .publish(&options)
        };
        Self {
            positions: arrays("Positions"),
            in_flight: arrays("InFlight"),
            last_shot_arc: arrays("LastShotArc"),
            blue_score: integer("BlueScore"),
            red_score: integer("RedScore"),
            ball_count: integer("Stats/BallCount"),
            active_balls: integer("Stats/ActiveBalls"),
            sleeping_balls: integer("Stats/SleepingBalls"),
            contact_count: integer("Stats/ContactsPerTick"),
            physics_time: double("Stats/PhysicsMs"),
            total_energy: double("Stats/TotalEnergy"),
        }
    }
}

pub struct FuelPhysicsSim {
    balls: Vec<SimBall>,
    contacts: Vec<Contact>, // keeps its capacity between ticks

    config: PhysicsConfig,
    rng: JavaRandom,

    // Spatial hash grid, used for ball-ball broadphase. Cell (col, row) is `col * GRID_ROWS + row`.
    grid: Vec<Vec<u32>>,

    blue_hub: ScoringTarget,
    red_hub: ScoringTarget,

    robot_pose_supplier: Option<Box<dyn Fn() -> Pose2d>>,
    robot_speeds_supplier: Option<Box<dyn Fn() -> ChassisSpeeds>>,
    robot_width: f64,
    robot_length: f64,
    bumper_height: f64,
    hopper_size: i32, // max balls the robot can hold; unlimited by default

    intakes: Vec<IntakeZone>,

    total_launched: i32,
    total_scored: i32,
    total_intaked: i32,
    last_launch_speed: f64,

    running: bool,

    publishers: Option<Publishers>,
    positions_scratch: Vec<Translation3d>,
    in_flight_scratch: Vec<Translation3d>,

    // Predicted path of the last shot, published for Field3d
    last_shot_arc: Vec<Translation3d>,
    last_physics_nanos: u64,

    total_ke: f64,
    total_pe: f64,
    total_momentum: Translation3d,
}

impl FuelPhysicsSim {
    /// `table_key` is where to publish in NetworkTables, for example `"Sim/Fuel"`.
    pub fn new(table_key: &str) -> Self {
        Self::with_config(table_key, PhysicsConfig::default())
    }

    pub fn with_config(table_key: &str, config: PhysicsConfig) -> Self {
        Self::build(config, Some(Publishers::new(table_key)))
    }

    /// A sim that never touches NetworkTables, for tests.
    pub fn headless(config: PhysicsConfig) -> Self {
        Self::build(config, None)
    }

    fn build(config: PhysicsConfig, publishers: Option<Publishers>) -> Self {
        let rng = if config.deterministic {
            JavaRandom::new(config.deterministic_seed)
        } else {
            JavaRandom::from_time()
        };

        Self {
            balls: Vec::new(),
            contacts: Vec::with_capacity(200),
            config,
            rng,
            grid: vec![Vec::new(); GRID_COLS * GRID_ROWS],
            blue_hub: ScoringTarget::new(
                BLUE_HUB_CENTER,
                Translation3d::new(5.3, FIELD_WIDTH / 2.0, 0.89),
                1,
            ),
            red_hub: ScoringTarget::new(
                RED_HUB_CENTER,
                Translation3d::new(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2.0, 0.89),
                -1,
            ),
            robot_pose_supplier: None,
            robot_speeds_supplier: None,
            robot_width: 0.0,
            robot_length: 0.0,
            bumper_height: 0.0,
            hopper_size: i32::MAX,
            intakes: Vec::new(),
            total_launched: 0,
            total_scored: 0,
            total_intaked: 0,
            last_launch_speed: 0.0,
            running: false,
            publishers,
            positions_scratch: Vec::new(),
            in_flight_scratch: Vec::new(),
            last_shot_arc: Vec::new(),
            last_physics_nanos: 0,
            total_ke: 0.0,
            total_pe: 0.0,
            total_momentum: Translation3d::ZERO,
        }
    }

    /// Turns the sim on. `tick` must still be called every cycle.
    pub fn enable(&mut self) {
        self.running = true;
    }

    pub fn disable(&mut self) {
        self.running = false;
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    /// `width` is the robot width along the Y axis and `length` along the X axis, in meters.
    /// Balls above `bumper_height` ignore the robot. Intake stops once the robot holds
    /// `hopper_size` balls. The suppliers return the field-relative pose and chassis speeds.
    pub fn configure_robot(
        &mut self,
        width: f64,
        length: f64,
        bumper_height: f64,
        hopper_size: i32,
        pose_supplier: impl Fn() -> Pose2d + 'static,
        speeds_supplier: impl Fn() -> ChassisSpeeds + 'static,
    ) {
        self.robot_width = width;
        self.robot_length = length;
        self.bumper_height = bumper_height;
        self.hopper_size = hopper_size;
        self.robot_pose_supplier = Some(Box::new(pose_supplier));
        self.robot_speeds_supplier = Some(Box::new(speeds_supplier));
    }

    /// Edges are in the robot frame, in meters: `x_min` front, `x_max` back, `y_min` left,
    /// `y_max` right.
    pub fn add_intake_zone(
        &mut self,
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        active: impl Fn() -> bool + 'static,
    ) {
        self.add_intake_zone_with_callback(x_min, x_max, y_min, y_max, active, || {});
    }

    pub fn add_intake_zone_with_callback(
        &mut self,
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        active: impl Fn() -> bool + 'static,
        callback: impl Fn() + 'static,
    ) {
        self.intakes.push(IntakeZone {
            x_min,
            x_max,
            y_min,
            y_max,
            active: Box::new(active),
            callback: Box::new(callback),
        });
    }

    /// `pos` is where the ball leaves the launcher and `vel` its launch velocity, both in the
    /// field frame. Positive `spin_rpm` is backspin, which lifts the ball through the Magnus
    /// effect.
    pub fn launch_ball(&mut self, pos: Translation3d, vel: Translation3d, spin_rpm: f64) {
        // Backspin is a negative rotation about Y
        let omega_y = -spin_rpm * 2.0 * PI / 60.0;
        let omega = Translation3d::new(0.0, omega_y, 0.0);
        self.launch_ball_omega(pos, vel, omega);
    }

    /// `omega` is the 3D angular velocity in rad/s; negative Y is backspin.
    pub fn launch_ball_omega(
        &mut self,
        pos: Translation3d,
        vel: Translation3d,
        omega: Translation3d,
    ) {
        if self.balls.len() >= MAX_BALLS {
            return;
        }
        self.balls.push(SimBall::new(pos, vel, omega));
        self.total_launched += 1;
        // Java: every launch counts against the intaked total, even for balls that were spawned.
        self.total_intaked = (self.total_intaked - 1).max(0);
        self.last_launch_speed = vel.norm();

        // Field3d arc; gravity and drag only, no collisions
        self.last_shot_arc = self.predict_arc(pos, vel, 20, 0.05);

        if self.config.sleeping_enabled {
            self.wake_nearby_balls(pos, 1.0);
        }
    }

    pub fn spawn_ball(&mut self, pos: Translation3d) {
        if self.balls.len() >= MAX_BALLS {
            return;
        }
        self.balls
            .push(SimBall::new(pos, Translation3d::ZERO, Translation3d::ZERO));
    }

    pub fn spawn_ball_with_velocity(&mut self, pos: Translation3d, vel: Translation3d) {
        if self.balls.len() >= MAX_BALLS {
            return;
        }
        self.balls.push(SimBall::new(pos, vel, Translation3d::ZERO));
    }

    pub fn clear_balls(&mut self) {
        self.balls.clear();
    }

    /// Spawns the neutral zone and both depot grids at their starting positions.
    pub fn place_field_balls(&mut self) {
        let cx = FIELD_LENGTH / 2.0;
        let cy = FIELD_WIDTH / 2.0;
        let nz_cols = 12; // 12 * 5.91in = 70.9in, fits inside the 72.0in depth
        let nz_rows = 30; // 30 * 5.91in = 177.3in, fits inside the 206.0in width
        let half_divider = 0.0254; // half of the 2.0in center divider

        for col in 0..nz_cols {
            let x = cx + (f64::from(col) - f64::from(nz_cols - 1) * 0.5) * BALL_DIAMETER;
            for row in 0..nz_rows {
                let y = cy + (f64::from(row) - f64::from(nz_rows - 1) * 0.5) * BALL_DIAMETER;
                if (y - cy).abs() < half_divider {
                    continue;
                }
                self.spawn_ball(Translation3d::new(x, y, BALL_RADIUS));
            }
        }

        let depot_cols = 4; // 4 * 5.91in = 23.6in fits inside 27.0in depth
        let depot_rows = 6; // 6 * 5.91in = 35.5in fits inside 42.0in width

        // Blue-side depot
        self.fill_ball_grid(FIELD_LENGTH - 0.37, 2.10, depot_cols, depot_rows);
        // Red-side depot
        self.fill_ball_grid(0.37, FIELD_WIDTH - 2.10, depot_cols, depot_rows);
    }

    fn fill_ball_grid(&mut self, cx: f64, cy: f64, cols: i32, rows: i32) {
        for c in 0..cols {
            let x = cx + (f64::from(c) - f64::from(cols - 1) * 0.5) * BALL_DIAMETER;
            for r in 0..rows {
                let y = cy + (f64::from(r) - f64::from(rows - 1) * 0.5) * BALL_DIAMETER;
                self.spawn_ball(Translation3d::new(x, y, BALL_RADIUS));
            }
        }
    }

    /// Advances one period and publishes ball positions. No-op while disabled.
    pub fn tick(&mut self) {
        if !self.running {
            return;
        }
        let start = Instant::now();
        self.advance_physics(PERIOD);
        self.last_physics_nanos = start.elapsed().as_nanos() as u64;
        self.publish_positions();
    }

    /// Advances the sim by `dt` seconds.
    pub fn advance_physics(&mut self, dt: f64) {
        let ticks = self.config.subticks.max(1);
        let sub_dt = dt / f64::from(ticks);

        for _ in 0..ticks {
            self.step_subtick(sub_dt);
        }

        // Remove flagged balls
        self.balls.retain(|b| !(b.intaked || b.out_of_bounds));
    }

    fn step_subtick(&mut self, sub_dt: f64) {
        self.contacts.clear();
        let sleeping_enabled = self.config.sleeping_enabled;

        let spin_decay_factor = (-sub_dt / self.config.spin_decay_tau).exp();
        for ball in &mut self.balls {
            if ball.intaked || ball.out_of_bounds {
                continue;
            }
            if ball.sleeping && sleeping_enabled {
                continue;
            }

            ball.prev_pos = ball.pos;

            let accel = compute_acceleration(&self.config, ball);

            // Symplectic Euler: update velocity first, or energy drifts.
            ball.vel = ball.vel + accel * sub_dt;
            ball.pos = ball.pos + ball.vel * sub_dt;

            if self.config.spin_decay_enabled && ball.omega.norm() > 1e-6 {
                ball.omega = ball.omega * spin_decay_factor;
            }
        }

        if self.config.ccd_enabled {
            for ball in &mut self.balls {
                if ball.intaked || ball.out_of_bounds {
                    continue;
                }
                if ball.sleeping && sleeping_enabled {
                    continue;
                }
                if ball.vel.norm() > self.config.ccd_speed_threshold {
                    handle_ccd(&self.config, ball);
                }
            }
        }

        self.build_spatial_hash();

        self.generate_ball_ball_contacts();
        self.generate_ball_field_contacts();

        self.solve_contacts();

        // Wall, ground, and bump impulses apply directly instead of going through the solver
        for ball in &mut self.balls {
            if ball.intaked || ball.out_of_bounds {
                continue;
            }
            if ball.sleeping && sleeping_enabled {
                continue;
            }
            handle_wall_bounce(&self.config, ball);
            handle_ground_contact(&self.config, ball, sub_dt);
            handle_bump_collisions(&self.config, ball);
        }

        for ball in &mut self.balls {
            if ball.intaked || ball.out_of_bounds {
                continue;
            }
            handle_hub_scoring(
                ball,
                &mut self.blue_hub,
                &mut self.red_hub,
                &mut self.rng,
                &mut self.total_scored,
            );
        }

        for ball in &mut self.balls {
            if ball.intaked || ball.out_of_bounds {
                continue;
            }
            if ball.sleeping && sleeping_enabled {
                continue;
            }
            handle_net_collision(ball, &self.blue_hub);
            handle_net_collision(ball, &self.red_hub);
        }

        if let (Some(pose_supplier), Some(speeds_supplier)) =
            (&self.robot_pose_supplier, &self.robot_speeds_supplier)
        {
            let robot_pose = pose_supplier();
            let speeds = speeds_supplier();
            let robot_vel = Translation2d::new(speeds.vx, speeds.vy);

            // Half-diagonal plus a margin, so balls react before the bumpers reach them
            let wake_radius = self.robot_length.hypot(self.robot_width) / 2.0 + 0.3;
            let wake_radius_sq = wake_radius * wake_radius;

            for ball in &mut self.balls {
                if ball.intaked || ball.out_of_bounds {
                    continue;
                }
                if ball.sleeping && sleeping_enabled {
                    let dx = ball.pos.x - robot_pose.x();
                    let dy = ball.pos.y - robot_pose.y();
                    if dx * dx + dy * dy < wake_radius_sq {
                        wake_ball(ball);
                    } else {
                        continue;
                    }
                }
                // Intake first so balls are consumed before bumper pushes them away
                if self.total_intaked < self.hopper_size {
                    for intake in &self.intakes {
                        if intake.should_intake(ball, robot_pose, self.bumper_height) {
                            ball.intaked = true;
                            self.total_intaked += 1;
                            break;
                        }
                    }
                }
                if ball.intaked {
                    continue;
                }
                handle_robot_collision(
                    ball,
                    robot_pose,
                    robot_vel,
                    self.bumper_height,
                    self.robot_length,
                    self.robot_width,
                );
            }
        }

        if sleeping_enabled {
            for ball in &mut self.balls {
                update_sleep_state(&self.config, ball);
            }
        }

        // NaN, field bounds, and upper-Z guard, plus removal of balls stuck on obstacles
        for ball in &mut self.balls {
            if !ball.pos.x.is_finite()
                || !ball.pos.y.is_finite()
                || !ball.pos.z.is_finite()
                || !ball.vel.x.is_finite()
                || !ball.vel.y.is_finite()
                || !ball.vel.z.is_finite()
                || !ball.omega.x.is_finite()
                || !ball.omega.y.is_finite()
                || !ball.omega.z.is_finite()
                || ball.pos.x < -2.0
                || ball.pos.x > FIELD_LENGTH + 2.0
                || ball.pos.y < -2.0
                || ball.pos.y > FIELD_WIDTH + 2.0
                || ball.pos.z < -1.0
                || ball.pos.z > 15.0
            {
                ball.out_of_bounds = true;
            }
            if ball.pos.z > BALL_RADIUS + 0.3 && ball.vel.norm() < 0.5 {
                ball.elevated_slow_counter += 1;
                if ball.elevated_slow_counter > 250 {
                    ball.out_of_bounds = true;
                }
            } else {
                ball.elevated_slow_counter = 0;
            }
        }

        if self.config.conservation_monitor {
            self.compute_conservation_quantities();
        }
    }

    fn build_spatial_hash(&mut self) {
        for cell in &mut self.grid {
            cell.clear();
        }
        // Include sleeping balls so awake balls can detect them for wake-on-collision
        for (i, ball) in self.balls.iter().enumerate() {
            if ball.intaked || ball.out_of_bounds {
                continue;
            }
            if let Some(cell) = cell_index(ball.pos.x, ball.pos.y, 0, 0) {
                self.grid[cell].push(i as u32);
            }
        }
    }

    fn generate_ball_ball_contacts(&mut self) {
        let sleeping_enabled = self.config.sleeping_enabled;
        let friction = if self.config.friction_enabled {
            MU_BALL_BALL
        } else {
            0.0
        };

        for i in 0..self.balls.len() {
            let ball_a = self.balls[i];
            if ball_a.intaked || ball_a.out_of_bounds {
                continue;
            }
            if ball_a.sleeping && sleeping_enabled {
                continue;
            }

            for di in -1..=1 {
                for dj in -1..=1 {
                    let Some(cell) = cell_index(ball_a.pos.x, ball_a.pos.y, di, dj) else {
                        continue;
                    };
                    for &j in &self.grid[cell] {
                        let j = j as usize;
                        if j == i {
                            continue;
                        }
                        if j < i && !(sleeping_enabled && self.balls[j].sleeping) {
                            continue;
                        }

                        let ball_b = &mut self.balls[j];
                        let dx = ball_a.pos.x - ball_b.pos.x;
                        let dy = ball_a.pos.y - ball_b.pos.y;
                        let dz = ball_a.pos.z - ball_b.pos.z;
                        let dist_sq = dx * dx + dy * dy + dz * dz;
                        let min_dist = BALL_RADIUS * 2.0;

                        if dist_sq < min_dist * min_dist {
                            let mut dist = dist_sq.sqrt();
                            let normal = if dist < 1e-9 {
                                dist = 1e-9;
                                AXIS_X_POS
                            } else {
                                (ball_a.pos - ball_b.pos) / dist
                            };
                            self.contacts.push(Contact::new(
                                i,
                                Some(j),
                                normal,
                                min_dist - dist,
                                COR_BALL_BALL,
                                friction,
                            ));

                            if ball_b.sleeping {
                                wake_ball(ball_b);
                            }
                        }
                    }
                }
            }
        }
    }

    fn generate_ball_field_contacts(&mut self) {
        let sleeping_enabled = self.config.sleeping_enabled;
        let friction = if self.config.friction_enabled {
            MU_WALL
        } else {
            0.0
        };

        for (i, ball) in self.balls.iter().enumerate() {
            if ball.intaked || ball.out_of_bounds {
                continue;
            }
            if ball.sleeping && sleeping_enabled {
                continue;
            }

            for aabb in AABB_OBSTACLES.iter() {
                generate_sphere_aabb_contact(&mut self.contacts, friction, i, ball, aabb);
            }

            for cyl in CYLINDER_OBSTACLES.iter() {
                generate_sphere_cylinder_contact(&mut self.contacts, friction, i, ball, cyl);
            }
        }
    }

    /// Sequential impulse solver over every contact, with warm starting.
    fn solve_contacts(&mut self) {
        if self.contacts.is_empty() {
            return;
        }
        let config = &self.config;
        let balls = &mut self.balls;

        // Restitution targets from the approach velocity, captured before any solving
        for c in &mut self.contacts {
            let ball_a = &balls[c.ball_index_a];
            let rel_vel = match c.ball_index_b {
                Some(b) => ball_a.vel - balls[b].vel,
                None => ball_a.vel,
            };
            let vn = rel_vel.dot(c.normal);
            let mut e = c.restitution;
            if config.velocity_dependent_cor {
                e = velocity_cor(config, e, vn.abs());
            }
            // Drop the bounce below 0.5 m/s of approach speed
            c.restitution_velocity = if vn < -0.5 { -e * vn } else { 0.0 };
        }

        for _ in 0..config.solver_iterations {
            for c in &mut self.contacts {
                solve_contact(config, balls, c);
            }
        }

        // Position correction
        for c in &self.contacts {
            apply_position_correction(config, balls, c);
        }
    }

    fn wake_nearby_balls(&mut self, pos: Translation3d, radius: f64) {
        let radius_sq = radius * radius;
        for ball in &mut self.balls {
            if ball.sleeping {
                let dx = ball.pos.x - pos.x;
                let dy = ball.pos.y - pos.y;
                let dz = ball.pos.z - pos.z;
                if dx * dx + dy * dy + dz * dz < radius_sq {
                    wake_ball(ball);
                }
            }
        }
    }

    fn compute_conservation_quantities(&mut self) {
        (self.total_ke, self.total_pe, self.total_momentum) = conservation_quantities(&self.balls);
    }

    /// Ballistic arc for the Field3d shot visual. Gravity and drag only; stops at the ground.
    fn predict_arc(
        &self,
        pos: Translation3d,
        vel: Translation3d,
        steps: i32,
        dt: f64,
    ) -> Vec<Translation3d> {
        let mut arc = Vec::with_capacity(steps as usize + 1);
        arc.push(pos);
        let (mut px, mut py, mut pz) = (pos.x, pos.y, pos.z);
        let (mut vx, mut vy, mut vz) = (vel.x, vel.y, vel.z);
        for _ in 0..steps {
            let speed = (vx * vx + vy * vy + vz * vz).sqrt();
            let drag = if self.config.drag_enabled && speed > 1e-6 {
                DRAG_ACCEL_FACTOR * speed
            } else {
                0.0
            };
            vx -= drag * vx * dt;
            vy -= drag * vy * dt;
            vz -= (GRAVITY + drag * vz) * dt;
            px += vx * dt;
            py += vy * dt;
            pz += vz * dt;
            if pz < BALL_RADIUS {
                break;
            }
            arc.push(Translation3d::new(px, py, pz));
        }
        arc
    }

    pub fn publish_positions(&mut self) {
        let Some(publishers) = &self.publishers else {
            return;
        };

        self.positions_scratch.clear();
        self.positions_scratch
            .extend(self.balls.iter().map(|b| b.pos));
        publishers.positions.set(&self.positions_scratch);

        self.in_flight_scratch.clear();
        let mut sleeping = 0;
        for b in &self.balls {
            if b.intaked || b.out_of_bounds {
                continue;
            }
            if b.pos.z > BALL_RADIUS + 0.1 {
                self.in_flight_scratch.push(b.pos);
            }
            if b.sleeping {
                sleeping += 1;
            }
        }
        publishers.in_flight.set(&self.in_flight_scratch);

        publishers.last_shot_arc.set(&self.last_shot_arc);

        publishers.blue_score.set(i64::from(self.blue_hub.score));
        publishers.red_score.set(i64::from(self.red_hub.score));

        publishers.ball_count.set(self.balls.len() as i64);
        publishers
            .active_balls
            .set(self.balls.len() as i64 - sleeping);
        publishers.sleeping_balls.set(sleeping);
        publishers.contact_count.set(self.contacts.len() as i64);
        publishers
            .physics_time
            .set(self.last_physics_nanos as f64 / 1_000_000.0);
        (self.total_ke, self.total_pe, self.total_momentum) = conservation_quantities(&self.balls);
        publishers.total_energy.set(self.total_ke + self.total_pe);
    }

    pub fn get_ball_count(&self) -> usize {
        self.balls.len()
    }

    /// Counts balls more than 0.1 m above the resting height.
    pub fn get_balls_in_flight(&self) -> usize {
        self.balls
            .iter()
            .filter(|b| b.pos.z > BALL_RADIUS + 0.1)
            .count()
    }

    pub fn get_balls_on_ground(&self) -> usize {
        self.balls
            .iter()
            .filter(|b| b.pos.z <= BALL_RADIUS + 0.1)
            .count()
    }

    pub fn get_balls(&self) -> &[SimBall] {
        &self.balls
    }

    pub fn get_config(&self) -> &PhysicsConfig {
        &self.config
    }

    pub fn set_config(&mut self, config: PhysicsConfig) {
        self.config = config;
        if self.config.deterministic {
            self.rng = JavaRandom::new(self.config.deterministic_seed);
        }
    }

    /// Lock the RNG seed so tests are repeatable.
    pub fn set_deterministic(&mut self, seed: i64) {
        self.config.deterministic = true;
        self.config.deterministic_seed = seed;
        self.rng = JavaRandom::new(seed);
    }

    /// Translational and rotational kinetic energy, in joules.
    pub fn get_total_kinetic_energy(&mut self) -> f64 {
        self.compute_conservation_quantities();
        self.total_ke
    }

    pub fn get_total_potential_energy(&mut self) -> f64 {
        self.compute_conservation_quantities();
        self.total_pe
    }

    pub fn get_total_momentum(&mut self) -> Translation3d {
        self.compute_conservation_quantities();
        self.total_momentum
    }

    pub fn get_total_launched(&self) -> i32 {
        self.total_launched
    }

    pub fn get_total_scored(&self) -> i32 {
        self.total_scored
    }

    pub fn get_total_intaked(&self) -> i32 {
        self.total_intaked
    }

    pub fn get_last_launch_speed(&self) -> f64 {
        self.last_launch_speed
    }

    pub fn get_blue_score(&self) -> i32 {
        self.blue_hub.score
    }

    pub fn get_red_score(&self) -> i32 {
        self.red_hub.score
    }

    /// Contacts the last sub-step generated.
    pub fn get_contact_count(&self) -> usize {
        self.contacts.len()
    }

    /// Predicted path of the last launched ball.
    pub fn get_last_shot_arc(&self) -> &[Translation3d] {
        &self.last_shot_arc
    }

    pub fn get_sleeping_ball_count(&self) -> usize {
        self.balls.iter().filter(|b| b.sleeping).count()
    }

    pub fn reset_counters(&mut self) {
        self.total_launched = 0;
        self.total_scored = 0;
        self.total_intaked = 0;
        self.last_launch_speed = 0.0;
        self.blue_hub.reset_score();
        self.red_hub.reset_score();
    }
}

/// Grid cell of the point offset by `(di, dj)` cells, or `None` outside the grid.
fn cell_index(x: f64, y: f64, di: i32, dj: i32) -> Option<usize> {
    // Truncates toward zero and saturates like Java's (int) cast, so -0.2 lands in cell 0
    let col = ((x / CELL_SIZE) as i32).wrapping_add(di);
    let row = ((y / CELL_SIZE) as i32).wrapping_add(dj);
    if col < 0 || col >= GRID_COLS as i32 || row < 0 || row >= GRID_ROWS as i32 {
        return None;
    }
    Some(col as usize * GRID_ROWS + row as usize)
}

fn conservation_quantities(balls: &[SimBall]) -> (f64, f64, Translation3d) {
    let mut total_ke = 0.0;
    let mut total_pe = 0.0;
    let (mut mx, mut my, mut mz) = (0.0, 0.0, 0.0);

    for ball in balls {
        let speed = ball.vel.norm();
        total_ke += 0.5 * BALL_MASS * speed * speed;

        let omega_mag = ball.omega.norm();
        total_ke += 0.5 * BALL_MOMENT_OF_INERTIA * omega_mag * omega_mag;

        // Potential energy, with the ground as the zero
        total_pe += BALL_MASS * GRAVITY * ball.pos.z;

        mx += BALL_MASS * ball.vel.x;
        my += BALL_MASS * ball.vel.y;
        mz += BALL_MASS * ball.vel.z;
    }

    (total_ke, total_pe, Translation3d::new(mx, my, mz))
}

fn compute_acceleration(config: &PhysicsConfig, ball: &SimBall) -> Translation3d {
    let (mut ax, mut ay, mut az) = (0.0, 0.0, -GRAVITY);

    let speed = ball.vel.norm();
    let airborne = ball.pos.z > BALL_RADIUS + 0.01;

    // Drag applies on the ground too; the air does not vanish at carpet level
    if config.drag_enabled && speed > 1e-6 {
        ax -= DRAG_ACCEL_FACTOR * speed * ball.vel.x;
        ay -= DRAG_ACCEL_FACTOR * speed * ball.vel.y;
        az -= DRAG_ACCEL_FACTOR * speed * ball.vel.z;
    }

    // Airborne only: ground friction dominates spin on carpet
    if airborne && speed > 1e-6 && config.magnus_enabled && ball.omega.norm() > 1e-3 {
        let magnus_dir = cross(ball.omega, ball.vel);
        let magnus_mag = magnus_dir.norm();
        if magnus_mag > 1e-6 {
            ax += MAGNUS_ACCEL_FACTOR * magnus_dir.x;
            ay += MAGNUS_ACCEL_FACTOR * magnus_dir.y;
            az += MAGNUS_ACCEL_FACTOR * magnus_dir.z;
        }
    }

    Translation3d::new(ax, ay, az)
}

fn handle_ccd(config: &PhysicsConfig, ball: &mut SimBall) {
    let delta = ball.pos - ball.prev_pos;
    let dist = delta.norm();
    if dist < 1e-6 {
        return;
    }

    let dir = delta / dist;

    let mut t_min = 1.0;
    let mut hit_normal: Option<Translation3d> = None;

    if dir.x < -1e-6 {
        let t = (BALL_RADIUS - ball.prev_pos.x) / delta.x;
        if t > 0.0 && t < t_min {
            t_min = t;
            hit_normal = Some(AXIS_X_POS);
        }
    } else if dir.x > 1e-6 {
        let t = (FIELD_LENGTH - BALL_RADIUS - ball.prev_pos.x) / delta.x;
        if t > 0.0 && t < t_min {
            t_min = t;
            hit_normal = Some(AXIS_X_NEG);
        }
    }

    if dir.y < -1e-6 {
        let t = (BALL_RADIUS - ball.prev_pos.y) / delta.y;
        if t > 0.0 && t < t_min {
            t_min = t;
            hit_normal = Some(AXIS_Y_POS);
        }
    } else if dir.y > 1e-6 {
        let t = (FIELD_WIDTH - BALL_RADIUS - ball.prev_pos.y) / delta.y;
        if t > 0.0 && t < t_min {
            t_min = t;
            hit_normal = Some(AXIS_Y_NEG);
        }
    }

    if dir.z < -1e-6 {
        let t = (BALL_RADIUS - ball.prev_pos.z) / delta.z;
        if t > 0.0 && t < t_min {
            t_min = t;
            hit_normal = Some(AXIS_Z_POS);
        }
    }

    for aabb in AABB_OBSTACLES.iter() {
        let t_hit = sweep_sphere_aabb(ball.prev_pos, delta, aabb);
        if t_hit >= 0.0 && t_hit < t_min {
            let hit_pos = ball.prev_pos + delta * t_hit;
            t_min = t_hit;
            hit_normal = Some(compute_aabb_normal(hit_pos, aabb));
        }
    }

    if let Some(hit_normal) = hit_normal
        && t_min < 1.0
    {
        ball.pos = ball.prev_pos + delta * t_min;

        let v_dot_n = ball.vel.dot(hit_normal);
        if v_dot_n < 0.0 {
            let cor = if config.velocity_dependent_cor {
                velocity_cor(config, COR_WALL, -v_dot_n)
            } else {
                COR_WALL
            };
            ball.vel = ball.vel - hit_normal * ((1.0 + cor) * v_dot_n);
        }
    }
}

/// Swept sphere against an AABB expanded by the ball radius. Returns the hit time in [0, 1]
/// or -1 on a miss.
fn sweep_sphere_aabb(origin: Translation3d, delta: Translation3d, aabb: &Aabb) -> f64 {
    let min_x = aabb.min_x - BALL_RADIUS;
    let min_y = aabb.min_y - BALL_RADIUS;
    let min_z = aabb.min_z - BALL_RADIUS;
    let max_x = aabb.max_x + BALL_RADIUS;
    let max_y = aabb.max_y + BALL_RADIUS;
    let max_z = aabb.max_z + BALL_RADIUS;

    let mut t_enter: f64 = 0.0;
    let mut t_exit: f64 = 1.0;

    if delta.x.abs() > 1e-9 {
        let inv_d = 1.0 / delta.x;
        let mut t1 = (min_x - origin.x) * inv_d;
        let mut t2 = (max_x - origin.x) * inv_d;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        t_enter = t_enter.max(t1);
        t_exit = t_exit.min(t2);
    } else if origin.x < min_x || origin.x > max_x {
        return -1.0;
    }

    if delta.y.abs() > 1e-9 {
        let inv_d = 1.0 / delta.y;
        let mut t1 = (min_y - origin.y) * inv_d;
        let mut t2 = (max_y - origin.y) * inv_d;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        t_enter = t_enter.max(t1);
        t_exit = t_exit.min(t2);
    } else if origin.y < min_y || origin.y > max_y {
        return -1.0;
    }

    if delta.z.abs() > 1e-9 {
        let inv_d = 1.0 / delta.z;
        let mut t1 = (min_z - origin.z) * inv_d;
        let mut t2 = (max_z - origin.z) * inv_d;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        t_enter = t_enter.max(t1);
        t_exit = t_exit.min(t2);
    } else if origin.z < min_z || origin.z > max_z {
        return -1.0;
    }

    if t_enter > t_exit || t_exit < 0.0 {
        return -1.0;
    }
    // Already inside when t_enter <= 0
    if t_enter > 0.0 { t_enter } else { -1.0 }
}

fn generate_sphere_aabb_contact(
    contacts: &mut Vec<Contact>,
    friction: f64,
    ball_index: usize,
    ball: &SimBall,
    aabb: &Aabb,
) {
    let cx = ball.pos.x.min(aabb.max_x).max(aabb.min_x);
    let cy = ball.pos.y.min(aabb.max_y).max(aabb.min_y);
    let cz = ball.pos.z.min(aabb.max_z).max(aabb.min_z);

    let dx = ball.pos.x - cx;
    let dy = ball.pos.y - cy;
    let dz = ball.pos.z - cz;
    let dist_sq = dx * dx + dy * dy + dz * dz;

    if (1e-9..BALL_RADIUS * BALL_RADIUS).contains(&dist_sq) {
        let dist = dist_sq.sqrt();
        contacts.push(Contact::new(
            ball_index,
            None,
            Translation3d::new(dx / dist, dy / dist, dz / dist),
            BALL_RADIUS - dist,
            aabb.cor,
            friction,
        ));
    } else if dist_sq < 1e-9 {
        let normal = compute_entry_face_normal(ball.prev_pos, ball.pos, aabb);
        let pen = compute_aabb_penetration(ball.pos, aabb);
        contacts.push(Contact::new(
            ball_index,
            None,
            normal,
            pen + BALL_RADIUS,
            aabb.cor,
            friction,
        ));
    }
}

fn compute_aabb_normal(p: Translation3d, aabb: &Aabb) -> Translation3d {
    let dx_min = p.x - aabb.min_x;
    let dx_max = aabb.max_x - p.x;
    let dy_min = p.y - aabb.min_y;
    let dy_max = aabb.max_y - p.y;
    let dz_min = p.z - aabb.min_z;
    let dz_max = aabb.max_z - p.z;

    let mut min = dx_min;
    let mut normal = AXIS_X_NEG;

    if dx_max < min {
        min = dx_max;
        normal = AXIS_X_POS;
    }
    if dy_min < min {
        min = dy_min;
        normal = AXIS_Y_NEG;
    }
    if dy_max < min {
        min = dy_max;
        normal = AXIS_Y_POS;
    }
    if dz_min < min {
        min = dz_min;
        normal = AXIS_Z_NEG;
    }
    if dz_max < min {
        normal = AXIS_Z_POS;
    }
    normal
}

/// Distance from a point inside the box to the nearest face.
fn compute_aabb_penetration(p: Translation3d, aabb: &Aabb) -> f64 {
    let dx_min = p.x - aabb.min_x;
    let dx_max = aabb.max_x - p.x;
    let dy_min = p.y - aabb.min_y;
    let dy_max = aabb.max_y - p.y;
    let dz_min = p.z - aabb.min_z;
    let dz_max = aabb.max_z - p.z;
    dx_min
        .min(dx_max)
        .min(dy_min.min(dy_max))
        .min(dz_min.min(dz_max))
}

/// Picks the face a ball entered through by raycasting from `from` to `to`. Without this, a
/// ball that entered from the side pops up on top of the obstacle. Falls back to the nearest
/// face when the ray is degenerate.
fn compute_entry_face_normal(from: Translation3d, to: Translation3d, aabb: &Aabb) -> Translation3d {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let dz = to.z - from.z;

    // Slab intersection: the entry face is the axis with the latest entry time
    let mut t_max = f64::NEG_INFINITY;
    let mut best_normal: Option<Translation3d> = None;

    if dx.abs() > 1e-12 {
        let inv_d = 1.0 / dx;
        let t1 = (aabb.min_x - from.x) * inv_d;
        let t2 = (aabb.max_x - from.x) * inv_d;
        let t_entry = t1.min(t2);
        if t_entry > t_max {
            t_max = t_entry;
            best_normal = Some(if dx > 0.0 { AXIS_X_NEG } else { AXIS_X_POS });
        }
    }
    if dy.abs() > 1e-12 {
        let inv_d = 1.0 / dy;
        let t1 = (aabb.min_y - from.y) * inv_d;
        let t2 = (aabb.max_y - from.y) * inv_d;
        let t_entry = t1.min(t2);
        if t_entry > t_max {
            t_max = t_entry;
            best_normal = Some(if dy > 0.0 { AXIS_Y_NEG } else { AXIS_Y_POS });
        }
    }
    if dz.abs() > 1e-12 {
        let inv_d = 1.0 / dz;
        let t1 = (aabb.min_z - from.z) * inv_d;
        let t2 = (aabb.max_z - from.z) * inv_d;
        let t_entry = t1.min(t2);
        if t_entry > t_max {
            best_normal = Some(if dz > 0.0 { AXIS_Z_NEG } else { AXIS_Z_POS });
        }
    }

    best_normal.unwrap_or_else(|| compute_aabb_normal(to, aabb))
}

fn generate_sphere_cylinder_contact(
    contacts: &mut Vec<Contact>,
    friction: f64,
    ball_index: usize,
    ball: &SimBall,
    cyl: &CylinderObstacle,
) {
    if cyl.ab_len_sq < 1e-12 {
        return;
    }

    let apx = ball.pos.x - cyl.a.x;
    let apy = ball.pos.y - cyl.a.y;
    let apz = ball.pos.z - cyl.a.z;
    let t = ((apx * cyl.ab.x + apy * cyl.ab.y + apz * cyl.ab.z) / cyl.ab_len_sq).clamp(0.0, 1.0);

    let near_x = cyl.a.x + t * cyl.ab.x;
    let near_y = cyl.a.y + t * cyl.ab.y;
    let near_z = cyl.a.z + t * cyl.ab.z;

    let dx = ball.pos.x - near_x;
    let dy = ball.pos.y - near_y;
    let dz = ball.pos.z - near_z;
    let dist_sq = dx * dx + dy * dy + dz * dz;
    let min_dist = BALL_RADIUS + cyl.radius;

    if dist_sq < min_dist * min_dist && dist_sq > 0.0 {
        let dist = dist_sq.sqrt();
        contacts.push(Contact::new(
            ball_index,
            None,
            Translation3d::new(dx / dist, dy / dist, dz / dist),
            min_dist - dist,
            cyl.cor,
            friction,
        ));
    }
}

fn solve_contact(config: &PhysicsConfig, balls: &mut [SimBall], c: &mut Contact) {
    let a = c.ball_index_a;
    let (mut rel_vel, inv_mass_sum) = match c.ball_index_b {
        // Same mass on both sides
        Some(b) => (balls[a].vel - balls[b].vel, 2.0 / BALL_MASS),
        None => (balls[a].vel, 1.0 / BALL_MASS),
    };

    let v_rel_normal = rel_vel.dot(c.normal);
    let mut jn = -(v_rel_normal - c.restitution_velocity) / inv_mass_sum;
    let old_accum = c.normal_impulse_accum;
    c.normal_impulse_accum = (old_accum + jn).max(0.0);
    jn = c.normal_impulse_accum - old_accum;

    let normal_impulse = c.normal * jn;
    balls[a].vel = balls[a].vel + normal_impulse / BALL_MASS;
    if let Some(b) = c.ball_index_b {
        balls[b].vel = balls[b].vel - normal_impulse / BALL_MASS;
    }

    if c.friction > 0.0 && config.friction_enabled {
        // Relative velocity has to be re-read after the normal impulse
        rel_vel = match c.ball_index_b {
            Some(b) => balls[a].vel - balls[b].vel,
            None => balls[a].vel,
        };

        let vn = rel_vel.dot(c.normal);
        let v_tangent = rel_vel - c.normal * vn;
        let v_tangent_mag = v_tangent.norm();

        if v_tangent_mag > 1e-6 {
            let tangent_dir = v_tangent / v_tangent_mag;

            let mut jt = -v_tangent_mag / inv_mass_sum;

            let max_friction = c.friction * c.normal_impulse_accum;
            let old_tangent_accum = c.tangent_impulse_accum;
            c.tangent_impulse_accum = (old_tangent_accum + jt)
                .min(max_friction)
                .max(-max_friction);
            jt = c.tangent_impulse_accum - old_tangent_accum;

            let friction_impulse = tangent_dir * jt;
            balls[a].vel = balls[a].vel + friction_impulse / BALL_MASS;
            if let Some(b) = c.ball_index_b {
                balls[b].vel = balls[b].vel - friction_impulse / BALL_MASS;
            }

            if config.spin_transfer_enabled && jt.abs() > 1e-9 {
                let r_contact = c.normal * -BALL_RADIUS; // from center to contact
                let torque_impulse = cross(r_contact, friction_impulse);
                let delta_omega = torque_impulse / BALL_MOMENT_OF_INERTIA;
                balls[a].omega = balls[a].omega + delta_omega;

                if let Some(b) = c.ball_index_b {
                    let r_contact_b = c.normal * BALL_RADIUS;
                    let torque_impulse_b = cross(r_contact_b, -friction_impulse);
                    let delta_omega_b = torque_impulse_b / BALL_MOMENT_OF_INERTIA;
                    balls[b].omega = balls[b].omega + delta_omega_b;
                }
            }
        }
    }
}

fn apply_position_correction(config: &PhysicsConfig, balls: &mut [SimBall], c: &Contact) {
    let slop = config.baumgarte_slop;
    let beta = config.baumgarte_beta;
    let correction = (c.penetration - slop).max(0.0) * beta;

    if correction < 1e-6 {
        return;
    }

    match c.ball_index_b {
        Some(b) => {
            let corr_vec = c.normal * (correction * 0.5);
            balls[c.ball_index_a].pos = balls[c.ball_index_a].pos + corr_vec;
            balls[b].pos = balls[b].pos - corr_vec;
        }
        None => {
            balls[c.ball_index_a].pos = balls[c.ball_index_a].pos + c.normal * correction;
        }
    }
}

// Wall and ground handling

fn handle_wall_bounce(config: &PhysicsConfig, ball: &mut SimBall) {
    let z = ball.pos.z;

    // Alliance walls; the height check lets balls fly over them
    if ball.pos.x < BALL_RADIUS {
        if z < ALLIANCE_WALL_HEIGHT {
            ball.pos = Translation3d::new(BALL_RADIUS, ball.pos.y, ball.pos.z);
            if ball.vel.x < 0.0 {
                apply_wall_spin_transfer(config, ball, AXIS_X_POS);
                let cor = effective_cor(config, COR_WALL, ball.vel.x.abs());
                ball.vel = Translation3d::new(-ball.vel.x * cor, ball.vel.y, ball.vel.z);
            }
        }
    } else if ball.pos.x > FIELD_LENGTH - BALL_RADIUS && z < ALLIANCE_WALL_HEIGHT {
        ball.pos = Translation3d::new(FIELD_LENGTH - BALL_RADIUS, ball.pos.y, ball.pos.z);
        if ball.vel.x > 0.0 {
            apply_wall_spin_transfer(config, ball, AXIS_X_NEG);
            let cor = effective_cor(config, COR_WALL, ball.vel.x.abs());
            ball.vel = Translation3d::new(-ball.vel.x * cor, ball.vel.y, ball.vel.z);
        }
    }

    // Guardrails; the height check lets balls fly over them
    if ball.pos.y < BALL_RADIUS {
        if z < GUARDRAIL_HEIGHT {
            ball.pos = Translation3d::new(ball.pos.x, BALL_RADIUS, ball.pos.z);
            if ball.vel.y < 0.0 {
                apply_wall_spin_transfer(config, ball, AXIS_Y_POS);
                let cor = effective_cor(config, COR_WALL, ball.vel.y.abs());
                ball.vel = Translation3d::new(ball.vel.x, -ball.vel.y * cor, ball.vel.z);
            }
        }
    } else if ball.pos.y > FIELD_WIDTH - BALL_RADIUS && z < GUARDRAIL_HEIGHT {
        ball.pos = Translation3d::new(ball.pos.x, FIELD_WIDTH - BALL_RADIUS, ball.pos.z);
        if ball.vel.y > 0.0 {
            apply_wall_spin_transfer(config, ball, AXIS_Y_NEG);
            let cor = effective_cor(config, COR_WALL, ball.vel.y.abs());
            ball.vel = Translation3d::new(ball.vel.x, -ball.vel.y * cor, ball.vel.z);
        }
    }
}

fn handle_ground_contact(config: &PhysicsConfig, ball: &mut SimBall, sub_dt: f64) {
    if ball.pos.z < BALL_RADIUS {
        ball.pos = Translation3d::new(ball.pos.x, ball.pos.y, BALL_RADIUS);

        if ball.vel.z < 0.0 {
            let cor = effective_cor(config, COR_CARPET, ball.vel.z.abs());

            // Bounces under 0.05 m/s are dropped rather than reflected
            if (ball.vel.z * cor).abs() < 0.05 {
                ball.vel = Translation3d::new(ball.vel.x, ball.vel.y, 0.0);
            } else {
                ball.vel = Translation3d::new(ball.vel.x, ball.vel.y, -ball.vel.z * cor);
            }
        }

        // Friction acts on surface velocity (vel + omega x r), so spin still bites
        if config.friction_enabled {
            let surf_vel_x = ball.vel.x - ball.omega.y * BALL_RADIUS;
            let surf_vel_y = ball.vel.y + ball.omega.x * BALL_RADIUS;
            let surf_speed = (surf_vel_x * surf_vel_x + surf_vel_y * surf_vel_y).sqrt();
            let h_speed = (ball.vel.x * ball.vel.x + ball.vel.y * ball.vel.y).sqrt();

            if surf_speed > 0.01 {
                let fdx = -surf_vel_x / surf_speed;
                let fdy = -surf_vel_y / surf_speed;

                let max_impulse = (2.0 / 7.0) * BALL_MASS * surf_speed;
                let friction_impulse =
                    (MU_GROUND_KINETIC * BALL_MASS * GRAVITY * sub_dt).min(max_impulse);

                ball.vel = Translation3d::new(
                    ball.vel.x + fdx * friction_impulse / BALL_MASS,
                    ball.vel.y + fdy * friction_impulse / BALL_MASS,
                    ball.vel.z,
                );
                if config.spin_transfer_enabled {
                    ball.omega = Translation3d::new(
                        ball.omega.x
                            + BALL_RADIUS * fdy * friction_impulse / BALL_MOMENT_OF_INERTIA,
                        ball.omega.y
                            - BALL_RADIUS * fdx * friction_impulse / BALL_MOMENT_OF_INERTIA,
                        ball.omega.z,
                    );
                }
            } else if h_speed > 1e-4 {
                // Rolling resistance scales vel and omega together
                let decel = MU_GROUND_ROLLING * GRAVITY * sub_dt;
                let scale = (h_speed - decel).max(0.0) / h_speed;
                ball.vel = Translation3d::new(ball.vel.x * scale, ball.vel.y * scale, ball.vel.z);
                if config.spin_transfer_enabled {
                    ball.omega = Translation3d::new(
                        ball.omega.x * scale,
                        ball.omega.y * scale,
                        ball.omega.z,
                    );
                }
            }
        }

        if ball.vel.z.abs() < 0.05 && ball.pos.z <= BALL_RADIUS + 0.01 {
            ball.vel = Translation3d::new(ball.vel.x, ball.vel.y, 0.0);
        }
    }
}

fn handle_bump_collisions(config: &PhysicsConfig, ball: &mut SimBall) {
    for seg in BUMP_SEGMENTS.iter() {
        if ball.pos.y < seg.y_start || ball.pos.y > seg.y_end {
            continue;
        }

        let px = ball.pos.x - seg.x_start;
        let pz = ball.pos.z - seg.z_start;
        let t = (px * seg.line_x + pz * seg.line_z) / (seg.line_len * seg.line_len);
        if !(0.0..=1.0).contains(&t) {
            continue;
        }

        let near_x = seg.x_start + t * seg.line_x;
        let near_z = seg.z_start + t * seg.line_z;
        let dx = ball.pos.x - near_x;
        let dz = ball.pos.z - near_z;
        let dist = (dx * dx + dz * dz).sqrt();

        if dist < BALL_RADIUS {
            let (nx, nz) = (seg.nx, seg.nz);

            ball.pos = ball.pos
                + Translation3d::new(nx * (BALL_RADIUS - dist), 0.0, nz * (BALL_RADIUS - dist));

            let v_dot_n = ball.vel.x * nx + ball.vel.z * nz;
            if v_dot_n < 0.0 {
                let cor = effective_cor(config, COR_HDPE, v_dot_n.abs());
                ball.vel = ball.vel
                    - Translation3d::new(
                        nx * (1.0 + cor) * v_dot_n,
                        0.0,
                        nz * (1.0 + cor) * v_dot_n,
                    );
            }
        }
    }
}

fn handle_hub_scoring(
    ball: &mut SimBall,
    blue_hub: &mut ScoringTarget,
    red_hub: &mut ScoringTarget,
    rng: &mut JavaRandom,
    total_scored: &mut i32,
) {
    if blue_hub.did_score(ball) {
        ball.pos = blue_hub.exit;
        ball.vel = blue_hub.get_dispersal_velocity(rng);
        ball.omega = Translation3d::ZERO;
        blue_hub.score += 1;
        *total_scored += 1;
    } else if red_hub.did_score(ball) {
        ball.pos = red_hub.exit;
        ball.vel = red_hub.get_dispersal_velocity(rng);
        ball.omega = Translation3d::ZERO;
        red_hub.score += 1;
        *total_scored += 1;
    }
}

/// Net collision. Blocks balls moving toward the net and lets them pass out the back.
fn handle_net_collision(ball: &mut SimBall, hub: &ScoringTarget) {
    if ball.pos.z > NET_HEIGHT_MAX || ball.pos.z < NET_HEIGHT_MIN {
        return;
    }
    if ball.pos.y > hub.center.y + NET_WIDTH / 2.0 || ball.pos.y < hub.center.y - NET_WIDTH / 2.0 {
        return;
    }

    let net_x = hub.center.x + NET_OFFSET * f64::from(hub.exit_vel_x_sign);
    let dist_to_net = ball.pos.x - net_x;

    if dist_to_net.abs() < BALL_RADIUS {
        let moving_toward_net = ball.vel.x * f64::from(hub.exit_vel_x_sign) > 0.0;
        if !moving_toward_net {
            return;
        }

        let push_dir = if dist_to_net >= 0.0 { 1.0 } else { -1.0 };
        ball.pos = Translation3d::new(net_x + push_dir * BALL_RADIUS, ball.pos.y, ball.pos.z);
        ball.vel = Translation3d::new(-ball.vel.x * COR_NET, ball.vel.y * COR_NET, ball.vel.z);
    }
}

fn handle_robot_collision(
    ball: &mut SimBall,
    robot_pose: Pose2d,
    robot_vel: Translation2d,
    bumper_height: f64,
    robot_length: f64,
    robot_width: f64,
) {
    if ball.pos.z > bumper_height {
        return;
    }

    let rel_pos = Pose2d::new(ball.pos.to_translation2d(), Rotation2d::ZERO)
        .relative_to(robot_pose)
        .translation;

    let half_l = robot_length / 2.0 + BALL_RADIUS;
    let half_w = robot_width / 2.0 + BALL_RADIUS;

    if rel_pos.x < -half_l || rel_pos.x > half_l || rel_pos.y < -half_w || rel_pos.y > half_w {
        return;
    }

    let dx_min = rel_pos.x + half_l;
    let dx_max = half_l - rel_pos.x;
    let dy_min = rel_pos.y + half_w;
    let dy_max = half_w - rel_pos.y;

    let mut min_dist = dx_min;
    let mut push_dir = Translation2d::new(-1.0, 0.0);

    if dx_max < min_dist {
        min_dist = dx_max;
        push_dir = Translation2d::new(1.0, 0.0);
    }
    if dy_min < min_dist {
        min_dist = dy_min;
        push_dir = Translation2d::new(0.0, -1.0);
    }
    if dy_max < min_dist {
        min_dist = dy_max;
        push_dir = Translation2d::new(0.0, 1.0);
    }

    // Rotate push direction back to field frame
    push_dir = push_dir.rotate_by(robot_pose.rotation);
    ball.pos = ball.pos + Translation3d::from_translation2d(push_dir * min_dist);

    // Velocity reflection
    let normal3d = Translation3d::new(push_dir.x, push_dir.y, 0.0);
    let v_dot_n = ball.vel.to_translation2d().dot(push_dir);
    let robot_v_dot_n = robot_vel.dot(push_dir);
    let closing_vel = v_dot_n - robot_v_dot_n;

    if closing_vel < 0.0 {
        ball.vel = ball.vel - normal3d * ((1.0 + COR_BUMPER) * closing_vel);
    }
}

fn update_sleep_state(config: &PhysicsConfig, ball: &mut SimBall) {
    let speed = ball.vel.norm();
    let omega_mag = ball.omega.norm();

    if speed < config.sleep_velocity_threshold
        && omega_mag < 0.1
        && ball.pos.z <= BALL_RADIUS + 0.01
    {
        ball.sleep_counter += 1;
        if ball.sleep_counter >= config.sleep_frame_threshold {
            ball.sleeping = true;
            ball.vel = Translation3d::ZERO;
            ball.omega = Translation3d::ZERO;
        }
    } else {
        ball.sleep_counter = 0;
        ball.sleeping = false;
    }
}

fn wake_ball(ball: &mut SimBall) {
    ball.sleeping = false;
    ball.sleep_counter = 0;
}

/// 3D cross product.
fn cross(a: Translation3d, b: Translation3d) -> Translation3d {
    Translation3d::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

/// Restitution falls as impact speed rises, as the ball deforms and loses energy.
fn velocity_cor(config: &PhysicsConfig, e0: f64, impact_speed: f64) -> f64 {
    if !config.velocity_dependent_cor {
        return e0;
    }
    if impact_speed <= COR_VREF {
        return e0;
    }
    e0 * (COR_VREF / impact_speed).powf(COR_EXPONENT)
}

fn effective_cor(config: &PhysicsConfig, base_cor: f64, impact_speed: f64) -> f64 {
    if config.velocity_dependent_cor {
        return velocity_cor(config, base_cor, impact_speed);
    }
    base_cor
}

fn apply_wall_spin_transfer(
    config: &PhysicsConfig,
    ball: &mut SimBall,
    wall_normal: Translation3d,
) {
    if !config.spin_transfer_enabled || !config.friction_enabled {
        return;
    }

    let r_contact = wall_normal * -BALL_RADIUS;
    let surface_vel = ball.vel + cross(ball.omega, r_contact);

    let vn = surface_vel.dot(wall_normal);
    let v_tangent = surface_vel - wall_normal * vn;
    let v_tangent_mag = v_tangent.norm();

    if v_tangent_mag > 1e-4 {
        let impact_speed = ball.vel.dot(wall_normal).abs();
        let cor = effective_cor(config, COR_WALL, impact_speed);
        let normal_impulse = BALL_MASS * impact_speed * (1.0 + cor);
        let friction_impulse = (MU_WALL * normal_impulse).min(BALL_MASS * v_tangent_mag);
        let friction_dir = -(v_tangent / v_tangent_mag);

        ball.vel = ball.vel + friction_dir * (friction_impulse / BALL_MASS);
        let torque_impulse = cross(r_contact, friction_dir * friction_impulse);
        ball.omega = ball.omega + torque_impulse / BALL_MOMENT_OF_INERTIA;
    }
}
