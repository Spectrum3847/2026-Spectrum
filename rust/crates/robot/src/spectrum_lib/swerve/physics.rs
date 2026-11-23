//! The part of the dyn4j 5.0.2 physics engine that maple-sim's swerve drivetrain relies on.
//!
//! Port source: dyn4j (BSD-3, Copyright (c) 2010-2024 William Bittle), tag 5.0.2. Covered: rigid
//! body integration (force, torque, damping, at-rest detection), a convex body against static
//! obstacles with penetration and the clipping manifold, and the sequential impulse solver with
//! warm starting, friction, restitution and Baumgarte position correction, and the continuous
//! collision step against static bodies (dyn4j's default mode, which maple-sim keeps). Not
//! covered: dynamic against dynamic contacts, joints, circles, and the broadphase.
//!
//! Approximated against dyn4j: penetration comes from SAT where dyn4j runs GJK and EPA (the same
//! axes for these shapes), separation distance is exact where GJK stops at a tolerance, the robot
//! is always the first body of a pair, and constraints are solved in obstacle order where dyn4j
//! uses the order its broadphase first reported each pair.

use std::f64::consts::PI;

/// dyn4j's `Epsilon.E`: the largest power of two that leaves 1.0 unchanged when added.
const EPSILON: f64 = f64::EPSILON / 2.0;

const MAXIMUM_TRANSLATION: f64 = 2.0;
const MAXIMUM_TRANSLATION_SQUARED: f64 = MAXIMUM_TRANSLATION * MAXIMUM_TRANSLATION;
const MAXIMUM_ROTATION: f64 = 0.5 * PI;
const MAXIMUM_AT_REST_LINEAR_VELOCITY_SQUARED: f64 = 0.01 * 0.01;
const MAXIMUM_AT_REST_ANGULAR_VELOCITY: f64 = 2.0 * PI / 180.0;
const VELOCITY_ITERATIONS: usize = 6;
const POSITION_ITERATIONS: usize = 2;
const LINEAR_TOLERANCE: f64 = 0.005;
const MAXIMUM_LINEAR_CORRECTION: f64 = 0.2;
const BAUMGARTE: f64 = 0.2;
const MAXIMUM_CONDITION: f64 = 1000.0;
/// `ConservativeAdvancement.DEFAULT_DISTANCE_EPSILON`.
const TOI_DISTANCE_EPSILON: f64 = 4.808_273_516_868_82e-6;
const TOI_MAX_ITERATIONS: usize = 30;
/// dyn4j's default step frequency, the previous step of the first one.
const DEFAULT_STEP: f64 = 1.0 / 60.0;
const RESTITUTION_VELOCITY: f64 = 1.0;
/// `Settings.setMinimumAtRestTime` in maple-sim's `Arena2026Rebuilt`.
const MINIMUM_AT_REST_TIME: f64 = 0.02;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V2 {
    pub x: f64,
    pub y: f64,
}

impl V2 {
    pub const ZERO: V2 = V2 { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// `Vector2.create(magnitude, direction)`.
    pub fn polar(magnitude: f64, direction: f64) -> Self {
        Self::new(magnitude * direction.cos(), magnitude * direction.sin())
    }

    pub fn plus(self, o: V2) -> V2 {
        V2::new(self.x + o.x, self.y + o.y)
    }

    pub fn minus(self, o: V2) -> V2 {
        V2::new(self.x - o.x, self.y - o.y)
    }

    pub fn scale(self, s: f64) -> V2 {
        V2::new(self.x * s, self.y * s)
    }

    pub fn negated(self) -> V2 {
        V2::new(-self.x, -self.y)
    }

    pub fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }

    pub fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }

    /// `Vector2.cross(double z)`: the velocity of a point at `self` on a body spinning at `z`.
    pub fn cross_z(self, z: f64) -> V2 {
        V2::new(-self.y * z, self.x * z)
    }

    pub fn magnitude_squared(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    pub fn magnitude(self) -> f64 {
        self.magnitude_squared().sqrt()
    }

    pub fn normalized(self) -> V2 {
        let m = self.magnitude();
        if m <= 0.0 { self } else { self.scale(1.0 / m) }
    }

    /// dyn4j's `right()`, which is a counter clockwise quarter turn.
    pub fn right(self) -> V2 {
        V2::new(-self.y, self.x)
    }

    pub fn direction(self) -> f64 {
        self.y.atan2(self.x)
    }

    /// `getAngleBetween(other)`: the signed angle from `self` to `other`, within -pi to pi.
    pub fn angle_between(self, other: V2) -> f64 {
        let a = other.direction() - self.direction();
        if a > PI {
            a - 2.0 * PI
        } else if a < -PI {
            a + 2.0 * PI
        } else {
            a
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub translation: V2,
    pub cost: f64,
    pub sint: f64,
}

impl Transform {
    pub fn new(translation: V2, angle: f64) -> Self {
        Self {
            translation,
            cost: angle.cos(),
            sint: angle.sin(),
        }
    }

    pub fn angle(&self) -> f64 {
        self.sint.atan2(self.cost)
    }

    pub fn transformed(&self, p: V2) -> V2 {
        V2::new(
            self.cost * p.x - self.sint * p.y + self.translation.x,
            self.sint * p.x + self.cost * p.y + self.translation.y,
        )
    }

    pub fn transformed_r(&self, v: V2) -> V2 {
        V2::new(
            self.cost * v.x - self.sint * v.y,
            self.sint * v.x + self.cost * v.y,
        )
    }

    pub fn inverse_transformed(&self, p: V2) -> V2 {
        self.inverse_transformed_r(p.minus(self.translation))
    }

    pub fn inverse_transformed_r(&self, v: V2) -> V2 {
        V2::new(
            self.cost * v.x + self.sint * v.y,
            -self.sint * v.x + self.cost * v.y,
        )
    }

    pub fn translate(&mut self, v: V2) {
        self.translation = self.translation.plus(v);
    }

    /// `lerp(end, alpha)`: position linearly, angle along the shorter way.
    pub fn lerp(&self, end: &Transform, alpha: f64) -> Transform {
        let x = (1.0 - alpha) * self.translation.x + alpha * end.translation.x;
        let y = (1.0 - alpha) * self.translation.y + alpha * end.translation.y;
        let rs = self.angle();
        let mut diff = end.angle() - rs;
        if diff < -PI {
            diff += 2.0 * PI;
        }
        if diff > PI {
            diff -= 2.0 * PI;
        }
        let a = diff * alpha + rs;
        Transform {
            translation: V2::new(x, y),
            cost: a.cos(),
            sint: a.sin(),
        }
    }

    /// `lerp(dp, da, alpha)`: moves by a fraction of a change in position and angle. The
    /// rotation turns about the transform origin.
    fn lerp_motion(&self, dp: V2, da: f64, alpha: f64) -> Transform {
        let (s, c) = (da * alpha).sin_cos();
        Transform {
            translation: self.translation.plus(dp.scale(alpha)),
            cost: (c * self.cost - s * self.sint).clamp(-1.0, 1.0),
            sint: (s * self.cost + c * self.sint).clamp(-1.0, 1.0),
        }
    }

    /// Rotates by `theta` about the world point `c`.
    pub fn rotate(&mut self, theta: f64, c: V2) {
        let (s, co) = theta.sin_cos();
        let cost = (co * self.cost - s * self.sint).clamp(-1.0, 1.0);
        let sint = (s * self.cost + co * self.sint).clamp(-1.0, 1.0);
        self.cost = cost;
        self.sint = sint;
        let cx = self.translation.x - c.x;
        let cy = self.translation.y - c.y;
        self.translation = V2::new(co * cx - s * cy + c.x, s * cx + co * cy + c.y);
    }
}

/// A convex shape in body space.
#[derive(Clone, Debug)]
pub enum Shape {
    /// Centered on the body origin, `width` along local x.
    Rectangle { width: f64, height: f64 },
    /// Between two local points.
    Segment { a: V2, b: V2 },
}

#[derive(Clone, Copy)]
struct PointFeature {
    point: V2,
    index: usize,
}

struct EdgeFeature {
    vertex1: PointFeature,
    vertex2: PointFeature,
    maximum: PointFeature,
    edge: V2,
    index: usize,
}

impl Shape {
    /// Counter clockwise, like dyn4j's rectangle.
    fn local_vertices(&self) -> Vec<V2> {
        match self {
            Shape::Rectangle { width, height } => vec![
                V2::new(-width * 0.5, -height * 0.5),
                V2::new(width * 0.5, -height * 0.5),
                V2::new(width * 0.5, height * 0.5),
                V2::new(-width * 0.5, height * 0.5),
            ],
            Shape::Segment { a, b } => vec![*a, *b],
        }
    }

    /// Outward normal of the edge that starts at each vertex. A segment has both sides.
    fn local_normals(&self) -> Vec<V2> {
        let v = self.local_vertices();
        let n = v.len();
        (0..n)
            .map(|i| {
                let e = v[(i + 1) % n].minus(v[i]);
                V2::new(e.y, -e.x).normalized()
            })
            .collect()
    }

    fn world_vertices(&self, t: &Transform) -> Vec<V2> {
        self.local_vertices()
            .into_iter()
            .map(|p| t.transformed(p))
            .collect()
    }

    fn world_axes(&self, t: &Transform) -> Vec<V2> {
        self.local_normals()
            .into_iter()
            .map(|n| t.transformed_r(n))
            .collect()
    }

    /// `getFarthestFeature`: the edge at the farthest vertex that faces `direction` most.
    fn farthest_feature(&self, direction: V2, t: &Transform) -> EdgeFeature {
        if let Shape::Segment { a, b } = self {
            let p1 = t.transformed(*a);
            let p2 = t.transformed(*b);
            let (max, max_index) = if direction.dot(p1) >= direction.dot(p2) {
                (p1, 0)
            } else {
                (p2, 1)
            };
            let vp1 = PointFeature {
                point: p1,
                index: 0,
            };
            let vp2 = PointFeature {
                point: p2,
                index: 1,
            };
            let maximum = PointFeature {
                point: max,
                index: max_index,
            };
            return if p2.minus(p1).right().dot(direction) > 0.0 {
                EdgeFeature {
                    vertex1: vp2,
                    vertex2: vp1,
                    maximum,
                    edge: p1.minus(p2),
                    index: 0,
                }
            } else {
                EdgeFeature {
                    vertex1: vp1,
                    vertex2: vp2,
                    maximum,
                    edge: p2.minus(p1),
                    index: 0,
                }
            };
        }

        let vertices = self.local_vertices();
        let normals = self.local_normals();
        let count = vertices.len();
        let local = t.inverse_transformed_r(direction);
        let mut index = 0;
        for i in 1..count {
            if local.dot(vertices[i]) > local.dot(vertices[index]) {
                index = i;
            }
        }
        let maximum = PointFeature {
            point: t.transformed(vertices[index]),
            index,
        };
        let left_n = normals[if index == 0 { count - 1 } else { index - 1 }];
        let right_n = normals[index];
        if left_n.dot(local) < right_n.dot(local) {
            let l = if index == count - 1 { 0 } else { index + 1 };
            let left = PointFeature {
                point: t.transformed(vertices[l]),
                index: l,
            };
            EdgeFeature {
                vertex1: maximum,
                vertex2: left,
                maximum,
                edge: left.point.minus(maximum.point),
                // dyn4j numbers this edge index + 1
                index: index + 1,
            }
        } else {
            let r = if index == 0 { count - 1 } else { index - 1 };
            let right = PointFeature {
                point: t.transformed(vertices[r]),
                index: r,
            };
            EdgeFeature {
                vertex1: right,
                vertex2: maximum,
                maximum,
                edge: maximum.point.minus(right.point),
                index,
            }
        }
    }
}

/// The shortest push that separates `s1` from `s2`: unit normal from 1 to 2, and depth. SAT over
/// the edge normals of both shapes, trying both directions of each axis.
fn penetration(s1: &Shape, t1: &Transform, s2: &Shape, t2: &Transform) -> Option<(V2, f64)> {
    let v1 = s1.world_vertices(t1);
    let v2 = s2.world_vertices(t2);
    let mut best: Option<(V2, f64)> = None;
    for axis in s1.world_axes(t1).into_iter().chain(s2.world_axes(t2)) {
        let (min1, max1) = project(&v1, axis);
        let (min2, max2) = project(&v2, axis);
        let toward_2 = max1 - min2;
        let toward_1 = max2 - min1;
        if toward_2 <= 0.0 || toward_1 <= 0.0 {
            return None;
        }
        for (normal, depth) in [(axis, toward_2), (axis.negated(), toward_1)] {
            if best.is_none_or(|(_, d)| depth < d) {
                best = Some((normal, depth));
            }
        }
    }
    best
}

fn project(vertices: &[V2], axis: V2) -> (f64, f64) {
    vertices
        .iter()
        .map(|v| v.dot(axis))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), d| {
            (lo.min(d), hi.max(d))
        })
}

impl Shape {
    /// `getRadius()`: the rotation radius about the shape's own center.
    fn radius(&self) -> f64 {
        match self {
            Shape::Rectangle { width, height } => (width * 0.5).hypot(height * 0.5),
            Shape::Segment { a, b } => b.minus(*a).magnitude() * 0.5,
        }
    }

    fn world_edges(&self, t: &Transform) -> Vec<(V2, V2)> {
        let v = self.world_vertices(t);
        if v.len() == 2 {
            return vec![(v[0], v[1])];
        }
        (0..v.len()).map(|i| (v[i], v[(i + 1) % v.len()])).collect()
    }
}

fn closest_on_segment(p: V2, a: V2, b: V2) -> V2 {
    let ab = b.minus(a);
    let length_squared = ab.magnitude_squared();
    if length_squared == 0.0 {
        return a;
    }
    a.plus(ab.scale((p.minus(a).dot(ab) / length_squared).clamp(0.0, 1.0)))
}

/// `Gjk.distance`: distance and unit normal from shape 1 to shape 2, or `None` when they overlap.
fn separation(s1: &Shape, t1: &Transform, s2: &Shape, t2: &Transform) -> Option<(f64, V2)> {
    if penetration(s1, t1, s2, t2).is_some() {
        return None;
    }
    let mut best: Option<(f64, V2, V2)> = None;
    for (from, to, swapped) in [(s1, s2, false), (s2, s1, true)] {
        let (tf, tt) = if swapped { (t2, t1) } else { (t1, t2) };
        for vertex in from.world_vertices(tf) {
            for (a, b) in to.world_edges(tt) {
                let on_edge = closest_on_segment(vertex, a, b);
                let d = on_edge.minus(vertex).magnitude();
                if best.is_none_or(|(bd, _, _)| d < bd) {
                    let (p1, p2) = if swapped {
                        (on_edge, vertex)
                    } else {
                        (vertex, on_edge)
                    };
                    best = Some((d, p1, p2));
                }
            }
        }
    }
    let (d, p1, p2) = best?;
    if d <= 0.0 {
        return None;
    }
    Some((d, p2.minus(p1).scale(1.0 / d)))
}

struct TimeOfImpact {
    time: f64,
    normal: V2,
    distance: f64,
}

/// `ConservativeAdvancement.getTimeOfImpact` against a static shape: advances shape 1 along its
/// change in position `dp1` and angle `da1` until it is within the distance epsilon.
#[allow(clippy::too_many_arguments)]
fn time_of_impact(
    s1: &Shape,
    tx1: &Transform,
    dp1: V2,
    da1: f64,
    s2: &Shape,
    tx2: &Transform,
    t1: f64,
    t2: f64,
) -> Option<TimeOfImpact> {
    let (mut d, mut n) = separation(s1, tx1, s2, tx2)?;
    if d < TOI_DISTANCE_EPSILON {
        return None;
    }
    let amax = s1.radius() * da1.abs();
    if dp1.magnitude() + amax == 0.0 {
        return None;
    }
    let mut l = t1;
    let mut l0 = l;
    let mut iterations = 0;
    while d > TOI_DISTANCE_EPSILON && iterations < TOI_MAX_ITERATIONS {
        let drel = dp1.dot(n) + amax;
        if drel <= EPSILON {
            return None;
        }
        l += d / drel;
        if l < t1 || l > t2 {
            return None;
        }
        if l <= l0 {
            break;
        }
        l0 = l;
        iterations += 1;
        match separation(s1, &tx1.lerp_motion(dp1, da1, l), s2, tx2) {
            Some((next_d, next_n)) => {
                d = next_d;
                n = next_n;
            }
            None => {
                // Overlapping: dyn4j keeps a cleared separation
                return Some(TimeOfImpact {
                    time: l,
                    normal: V2::ZERO,
                    distance: 0.0,
                });
            }
        }
    }
    Some(TimeOfImpact {
        time: l,
        normal: n,
        distance: d,
    })
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct PointId {
    reference: usize,
    incident: usize,
    vertex: usize,
    flipped: bool,
}

struct ManifoldPoint {
    id: PointId,
    point: V2,
    depth: f64,
}

struct Manifold {
    normal: V2,
    points: Vec<ManifoldPoint>,
}

/// `ClippingManifoldSolver.getManifold`, for edge features, which every shape here has. The
/// manifold normal points from shape 2 to shape 1.
fn manifold(
    s1: &Shape,
    t1: &Transform,
    s2: &Shape,
    t2: &Transform,
    normal: V2,
) -> Option<Manifold> {
    let mut reference = s1.farthest_feature(normal, t1);
    let mut incident = s2.farthest_feature(normal.negated(), t2);
    let mut flipped = false;
    if reference.edge.dot(normal).abs() > incident.edge.dot(normal).abs() {
        std::mem::swap(&mut reference, &mut incident);
        flipped = true;
    }

    let refev = reference.edge.normalized();
    let offset1 = -refev.dot(reference.vertex1.point);
    let clip1 = clip(incident.vertex1, incident.vertex2, refev.negated(), offset1);
    if clip1.len() < 2 {
        return None;
    }
    let offset2 = refev.dot(reference.vertex2.point);
    let clip2 = clip(clip1[0], clip1[1], refev, offset2);
    if clip2.len() < 2 {
        return None;
    }

    let front_normal = refev.right();
    let front_offset = front_normal.dot(reference.maximum.point);
    let mut points = Vec::new();
    for vertex in clip2 {
        let depth = front_normal.dot(vertex.point) - front_offset;
        if depth >= 0.0 {
            points.push(ManifoldPoint {
                id: PointId {
                    reference: reference.index,
                    incident: incident.index,
                    vertex: vertex.index,
                    flipped,
                },
                point: vertex.point,
                depth,
            });
        }
    }
    if points.is_empty() {
        return None;
    }
    Some(Manifold {
        normal: if flipped {
            front_normal.negated()
        } else {
            front_normal
        },
        points,
    })
}

fn clip(v1: PointFeature, v2: PointFeature, n: V2, offset: f64) -> Vec<PointFeature> {
    let mut points = Vec::with_capacity(3);
    let d1 = n.dot(v1.point) - offset;
    let d2 = n.dot(v2.point) - offset;
    if d1 <= 0.0 {
        points.push(v1);
    }
    if d2 <= 0.0 {
        points.push(v2);
    }
    if d1 * d2 < 0.0 {
        let u = d1 / (d1 - d2);
        let point = v2.point.minus(v1.point).scale(u).plus(v1.point);
        let index = if d1 > 0.0 { v1.index } else { v2.index };
        points.push(PointFeature { point, index });
    }
    points
}

/// A rigid body with one rectangle fixture, integrated like `org.dyn4j.dynamics.Body`.
#[derive(Clone, Debug)]
pub struct RigidBody {
    pub transform: Transform,
    pub linear_velocity: V2,
    pub angular_velocity: f64,
    pub linear_damping: f64,
    pub angular_damping: f64,
    shape: Shape,
    friction: f64,
    restitution: f64,
    mass: f64,
    inertia: f64,
    force: V2,
    torque: f64,
    at_rest: bool,
    at_rest_time: f64,
}

impl RigidBody {
    /// `addFixture(createRectangle(width, height), density, friction, restitution)` and
    /// `setMass(MassType.NORMAL)`. The center of mass is the body origin.
    pub fn rectangle(
        width: f64,
        height: f64,
        density: f64,
        friction: f64,
        restitution: f64,
    ) -> Self {
        let mass = density * height * width;
        Self {
            transform: Transform::new(V2::ZERO, 0.0),
            linear_velocity: V2::ZERO,
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.01,
            shape: Shape::Rectangle { width, height },
            friction,
            restitution,
            mass,
            inertia: mass * (height * height + width * width) / 12.0,
            force: V2::ZERO,
            torque: 0.0,
            at_rest: false,
            at_rest_time: 0.0,
        }
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn inertia(&self) -> f64 {
        self.inertia
    }

    fn inverse_mass(&self) -> f64 {
        1.0 / self.mass
    }

    fn inverse_inertia(&self) -> f64 {
        1.0 / self.inertia
    }

    /// `getLinearVelocity(point)`: the velocity of the body at a world point.
    pub fn linear_velocity_at(&self, point: V2) -> V2 {
        point
            .minus(self.transform.translation)
            .cross_z(self.angular_velocity)
            .plus(self.linear_velocity)
    }

    pub fn is_at_rest(&self) -> bool {
        self.at_rest
    }

    fn set_at_rest(&mut self, at_rest: bool) {
        if at_rest {
            self.at_rest = true;
            self.linear_velocity = V2::ZERO;
            self.angular_velocity = 0.0;
            self.force = V2::ZERO;
            self.torque = 0.0;
        } else if self.at_rest {
            self.at_rest_time = 0.0;
            self.at_rest = false;
        }
    }

    pub fn apply_force(&mut self, force: V2) {
        self.force = self.force.plus(force);
        self.set_at_rest(false);
    }

    /// A force at a world point: the force plus its torque about the center of mass.
    pub fn apply_force_at(&mut self, force: V2, point: V2) {
        self.force = self.force.plus(force);
        let r = point.minus(self.transform.translation);
        if r != V2::ZERO {
            self.torque += r.cross(force);
        }
        self.set_at_rest(false);
    }

    pub fn apply_torque(&mut self, torque: f64) {
        self.torque += torque;
        self.set_at_rest(false);
    }

    fn integrate_velocity(&mut self, dt: f64) {
        let inverse_mass = self.inverse_mass();
        let inverse_inertia = self.inverse_inertia();
        if inverse_mass > EPSILON {
            self.linear_velocity.x += dt * inverse_mass * self.force.x;
            self.linear_velocity.y += dt * inverse_mass * self.force.y;
        }
        if inverse_inertia > EPSILON {
            self.angular_velocity += inverse_inertia * self.torque * dt;
        }
        self.force = V2::ZERO;
        self.torque = 0.0;

        if self.linear_damping != 0.0 {
            let linear = (1.0 - dt * self.linear_damping).clamp(0.0, 1.0);
            self.linear_velocity = self.linear_velocity.scale(linear);
        }
        let angular = (1.0 - dt * self.angular_damping).clamp(0.0, 1.0);
        self.angular_velocity *= angular;
    }

    fn integrate_position(&mut self, dt: f64) {
        let mut translation = self.linear_velocity.scale(dt);
        let squared = translation.magnitude_squared();
        if squared > MAXIMUM_TRANSLATION_SQUARED {
            let ratio = MAXIMUM_TRANSLATION / squared.sqrt();
            self.linear_velocity = self.linear_velocity.scale(ratio);
            translation = translation.scale(ratio);
        }
        let mut rotation = self.angular_velocity * dt;
        // dyn4j only limits a positive rotation
        if rotation > MAXIMUM_ROTATION {
            let ratio = MAXIMUM_ROTATION / rotation.abs();
            self.angular_velocity *= ratio;
            rotation *= ratio;
        }
        self.transform.translate(translation);
        let center = self.transform.translation;
        self.transform.rotate(rotation, center);
    }

    fn update_at_rest_time(&mut self, dt: f64) -> f64 {
        if self.linear_velocity.magnitude_squared() > MAXIMUM_AT_REST_LINEAR_VELOCITY_SQUARED
            || self.angular_velocity.abs() > MAXIMUM_AT_REST_ANGULAR_VELOCITY
        {
            self.at_rest_time = 0.0;
        } else {
            self.at_rest_time += dt;
        }
        self.at_rest_time
    }
}

/// A fixed body with one fixture: field borders and elements.
#[derive(Clone, Debug)]
pub struct Obstacle {
    pub shape: Shape,
    pub transform: Transform,
    friction: f64,
    restitution: f64,
}

impl Obstacle {
    pub fn new(shape: Shape, transform: Transform, friction: f64, restitution: f64) -> Self {
        Self {
            shape,
            transform,
            friction,
            restitution,
        }
    }
}

struct SolvableContact {
    id: PointId,
    p: V2,
    depth: f64,
    p1: V2,
    p2: V2,
    r1: V2,
    jn: f64,
    jt: f64,
    jp: f64,
    mass_n: f64,
    mass_t: f64,
    vb: f64,
}

#[derive(Clone, Copy, Default)]
struct Matrix22 {
    m00: f64,
    m01: f64,
    m10: f64,
    m11: f64,
}

impl Matrix22 {
    fn determinant(&self) -> f64 {
        self.m00 * self.m11 - self.m01 * self.m10
    }

    fn inverse(&self) -> Matrix22 {
        let mut det = self.determinant();
        det = if det.abs() > EPSILON { 1.0 / det } else { 0.0 };
        Matrix22 {
            m00: det * self.m11,
            m01: -det * self.m01,
            m10: -det * self.m10,
            m11: det * self.m00,
        }
    }

    fn product(&self, v: V2) -> V2 {
        V2::new(
            self.m00 * v.x + self.m01 * v.y,
            self.m10 * v.x + self.m11 * v.y,
        )
    }
}

/// `ContactConstraint` between the body (first) and one obstacle (second).
#[derive(Default)]
struct ContactConstraint {
    contacts: Vec<SolvableContact>,
    size: usize,
    normal: V2,
    tangent: V2,
    friction: f64,
    restitution: f64,
    k: Matrix22,
    inv_k: Matrix22,
    active: bool,
}

impl ContactConstraint {
    /// `ContactConstraint.update`: replaces the contacts, keeping the impulses of the old ones
    /// that match for warm starting.
    fn update(&mut self, found: Option<Manifold>, body: &RigidBody, obstacle: &Obstacle) {
        let (normal, points) = match found {
            Some(m) => (m.normal, m.points),
            None => (V2::ZERO, Vec::new()),
        };
        self.normal = normal;
        self.tangent = V2::new(normal.y, -normal.x);
        self.friction = (body.friction * obstacle.friction).sqrt();
        self.restitution = body.restitution.max(obstacle.restitution);

        let mut contacts = Vec::with_capacity(points.len());
        for point in points {
            let mut contact = SolvableContact {
                id: point.id,
                p: point.point,
                depth: point.depth,
                p1: body.transform.inverse_transformed(point.point),
                p2: obstacle.transform.inverse_transformed(point.point),
                r1: V2::ZERO,
                jn: 0.0,
                jt: 0.0,
                jp: 0.0,
                mass_n: 0.0,
                mass_t: 0.0,
                vb: 0.0,
            };
            if let Some(j) = (0..self.contacts.len())
                .rev()
                .find(|&j| self.contacts[j].id == contact.id)
            {
                let old = self.contacts.remove(j);
                contact.jn = old.jn;
                contact.jt = old.jt;
            }
            contacts.push(contact);
        }
        self.contacts = contacts;
        self.size = self.contacts.len();
    }
}

/// `SequentialImpulses` with a dynamic first body and a static second body.
struct Solver<'a> {
    body: &'a mut RigidBody,
    obstacle: &'a Obstacle,
}

impl Solver<'_> {
    fn mass_coefficient(&self, r1: V2, n: V2) -> f64 {
        let r1_cross_n = r1.cross(n);
        self.body.inverse_mass() + self.body.inverse_inertia() * r1_cross_n * r1_cross_n
    }

    fn update_bodies(&mut self, r1: V2, j: V2) {
        let inverse_mass = self.body.inverse_mass();
        let inverse_inertia = self.body.inverse_inertia();
        self.body.linear_velocity.x += j.x * inverse_mass;
        self.body.linear_velocity.y += j.y * inverse_mass;
        self.body.angular_velocity += inverse_inertia * r1.cross(j);
    }

    fn relative_velocity(&self, r1: V2) -> V2 {
        r1.cross_z(self.body.angular_velocity)
            .plus(self.body.linear_velocity)
    }

    fn initialize(&self, constraint: &mut ContactConstraint) {
        let c1 = self.body.transform.translation;
        let n = constraint.normal;
        let t = constraint.tangent;
        for contact in &mut constraint.contacts {
            contact.r1 = contact.p.minus(c1);
            contact.mass_n = 1.0 / self.mass_coefficient(contact.r1, n);
            contact.mass_t = 1.0 / self.mass_coefficient(contact.r1, t);
            contact.vb = 0.0;
            let rvn = n.dot(self.relative_velocity(contact.r1));
            if rvn < -RESTITUTION_VELOCITY {
                contact.vb += -constraint.restitution * rvn;
            }
        }
        constraint.size = constraint.contacts.len();
        if constraint.contacts.len() == 2 {
            let inverse_mass = self.body.inverse_mass();
            let inverse_inertia = self.body.inverse_inertia();
            let rn1 = constraint.contacts[0].r1.cross(n);
            let rn2 = constraint.contacts[1].r1.cross(n);
            let m01 = inverse_mass + inverse_inertia * rn1 * rn2;
            let k = Matrix22 {
                m00: inverse_mass + inverse_inertia * rn1 * rn1,
                m01,
                m10: m01,
                m11: inverse_mass + inverse_inertia * rn2 * rn2,
            };
            if k.m00 * k.m00 < MAXIMUM_CONDITION * k.determinant() {
                constraint.k = k;
                constraint.inv_k = k.inverse();
            } else {
                // Nearly parallel contact points: solve the deeper one only
                constraint.size = 1;
                if constraint.contacts[0].depth < constraint.contacts[1].depth {
                    constraint.contacts.swap(0, 1);
                }
            }
        }
    }

    fn warm_start(&mut self, constraint: &mut ContactConstraint, ratio: f64) {
        let n = constraint.normal;
        let t = constraint.tangent;
        for contact in &mut constraint.contacts[..constraint.size] {
            contact.jn *= ratio;
            contact.jt *= ratio;
            let j = V2::new(
                n.x * contact.jn + t.x * contact.jt,
                n.y * contact.jn + t.y * contact.jt,
            );
            self.update_bodies(contact.r1, j);
        }
    }

    fn solve_velocity(&mut self, constraint: &mut ContactConstraint) {
        let size = constraint.size;
        if size == 0 {
            return;
        }
        let n = constraint.normal;
        let t = constraint.tangent;
        for k in 0..size {
            let r1 = constraint.contacts[k].r1;
            let rvt = t.dot(self.relative_velocity(r1));
            let contact = &mut constraint.contacts[k];
            let max_jt = constraint.friction * contact.jn;
            let jt0 = contact.jt;
            contact.jt = (-max_jt).max((jt0 + contact.mass_t * (-rvt)).min(max_jt));
            let jt = contact.jt - jt0;
            self.update_bodies(r1, t.scale(jt));
        }

        if size == 1 {
            let r1 = constraint.contacts[0].r1;
            let rvn = n.dot(self.relative_velocity(r1));
            let contact = &mut constraint.contacts[0];
            let j0 = contact.jn;
            contact.jn = (j0 - contact.mass_n * (rvn - contact.vb)).max(0.0);
            let j = contact.jn - j0;
            self.update_bodies(r1, n.scale(j));
            return;
        }

        let r1a = constraint.contacts[0].r1;
        let r1b = constraint.contacts[1].r1;
        let a = V2::new(constraint.contacts[0].jn, constraint.contacts[1].jn);
        let rvn1 = n.dot(self.relative_velocity(r1a));
        let rvn2 = n.dot(self.relative_velocity(r1b));
        let b = V2::new(
            rvn1 - constraint.contacts[0].vb,
            rvn2 - constraint.contacts[1].vb,
        )
        .minus(constraint.k.product(a));

        // The block solver tries each guess of which contacts stay active
        let k = constraint.k;
        let x = constraint.inv_k.product(b).negated();
        let solution = if x.x >= 0.0 && x.y >= 0.0 {
            Some(x)
        } else {
            let x = V2::new(-constraint.contacts[0].mass_n * b.x, 0.0);
            if x.x >= 0.0 && k.m10 * x.x + b.y >= 0.0 {
                Some(x)
            } else {
                let x = V2::new(0.0, -constraint.contacts[1].mass_n * b.y);
                if x.y >= 0.0 && k.m01 * x.y + b.x >= 0.0 {
                    Some(x)
                } else if b.x >= 0.0 && b.y >= 0.0 {
                    Some(V2::ZERO)
                } else {
                    None
                }
            }
        };
        if let Some(x) = solution {
            let j1 = n.scale(x.x - a.x);
            let j2 = n.scale(x.y - a.y);
            let j = j1.plus(j2);
            let inverse_mass = self.body.inverse_mass();
            self.body.linear_velocity.x += j.x * inverse_mass;
            self.body.linear_velocity.y += j.y * inverse_mass;
            self.body.angular_velocity +=
                self.body.inverse_inertia() * (r1a.cross(j1) + r1b.cross(j2));
            constraint.contacts[0].jn = x.x;
            constraint.contacts[1].jn = x.y;
        }
    }

    /// One pass of the position solver over a constraint. Returns the smallest separation seen.
    fn solve_position(&mut self, constraint: &mut ContactConstraint) -> f64 {
        let mut min_separation: f64 = 0.0;
        let n = constraint.normal;
        for contact in &mut constraint.contacts[..constraint.size] {
            let c1 = self.body.transform.translation;
            let r1 = self.body.transform.transformed_r(contact.p1);
            let p1 = c1.plus(r1);
            let p2 = self.obstacle.transform.transformed(contact.p2);
            let penetration = p1.minus(p2).dot(n) - contact.depth;
            min_separation = min_separation.min(penetration);
            let cp =
                BAUMGARTE * (penetration + LINEAR_TOLERANCE).clamp(-MAXIMUM_LINEAR_CORRECTION, 0.0);
            let k_mass = self.mass_coefficient(r1, n);
            let jp = if k_mass > EPSILON { -cp / k_mass } else { 0.0 };
            let jp0 = contact.jp;
            contact.jp = (jp0 + jp).max(0.0);
            let j = n.scale(contact.jp - jp0);
            self.body
                .transform
                .translate(j.scale(self.body.inverse_mass()));
            let angle = self.body.inverse_inertia() * r1.cross(j);
            self.body.transform.rotate(angle, c1);
        }
        min_separation
    }
}

/// The dyn4j `World` with zero gravity, restricted to dynamic bodies and static obstacles.
pub struct World {
    obstacles: Vec<Obstacle>,
    constraints: Vec<Vec<ContactConstraint>>,
    previous_dt: f64,
    update_required: bool,
}

impl World {
    pub fn new(obstacles: Vec<Obstacle>) -> Self {
        Self {
            obstacles,
            constraints: Vec::new(),
            previous_dt: DEFAULT_STEP,
            update_required: true,
        }
    }

    pub fn obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }

    /// `World.step(1, dt)`: solves the contacts found at the end of the last step, then detects
    /// again.
    pub fn step(&mut self, bodies: &mut [&mut RigidBody], dt: f64) {
        if self.constraints.len() != bodies.len() {
            self.constraints = bodies
                .iter()
                .map(|_| {
                    self.obstacles
                        .iter()
                        .map(|_| ContactConstraint::default())
                        .collect()
                })
                .collect();
            self.update_required = true;
        }
        if self.update_required {
            self.detect(bodies);
            self.update_required = false;
        }
        let dt_ratio = dt / self.previous_dt;
        self.previous_dt = dt;
        let previous: Vec<Transform> = bodies.iter().map(|b| b.transform).collect();

        for (b, body) in bodies.iter_mut().enumerate() {
            if !body.is_at_rest() {
                self.solve_island(b, body, dt, dt_ratio);
            }
        }
        for (b, body) in bodies.iter_mut().enumerate() {
            self.solve_time_of_impact(b, body, &previous[b], dt);
        }
        self.detect(bodies);
    }

    /// `solveTOI` for one body: a body that was not touching an obstacle at the start of the step
    /// and now overlaps it moves back to where it first reached the obstacle, then a little in.
    fn solve_time_of_impact(&self, b: usize, body: &mut RigidBody, previous: &Transform, dt: f64) {
        let dp1 = body.linear_velocity.scale(dt);
        let da1 = body.angular_velocity * dt;
        let mut t2 = 1.0;
        let mut earliest: Option<TimeOfImpact> = None;
        for (obstacle, constraint) in self.obstacles.iter().zip(&self.constraints[b]) {
            if constraint.active {
                continue;
            }
            let toi = time_of_impact(
                &body.shape,
                previous,
                dp1,
                da1,
                &obstacle.shape,
                &obstacle.transform,
                0.0,
                t2,
            );
            if let Some(toi) = toi {
                if toi.time == 0.0 {
                    return;
                }
                if toi.time < t2 {
                    t2 = toi.time;
                    earliest = Some(toi);
                }
            }
        }
        if let Some(toi) = earliest {
            body.transform = previous.lerp(&body.transform, toi.time);
            if toi.distance <= 0.0 {
                return;
            }
            let correction =
                (toi.distance + LINEAR_TOLERANCE).clamp(0.0, MAXIMUM_LINEAR_CORRECTION);
            body.transform.translate(toi.normal.scale(correction));
        }
    }

    fn solve_island(&mut self, b: usize, body: &mut RigidBody, dt: f64, dt_ratio: f64) {
        body.integrate_velocity(dt);

        let mut island: Vec<(&Obstacle, &mut ContactConstraint)> = self
            .obstacles
            .iter()
            .zip(self.constraints[b].iter_mut())
            .filter(|(_, c)| c.active)
            .collect();

        for (obstacle, constraint) in &mut island {
            Solver {
                body: &mut *body,
                obstacle,
            }
            .initialize(constraint);
        }
        for (obstacle, constraint) in &mut island {
            Solver {
                body: &mut *body,
                obstacle,
            }
            .warm_start(constraint, 1.0 / dt_ratio);
        }
        for _ in 0..VELOCITY_ITERATIONS {
            for (obstacle, constraint) in &mut island {
                Solver {
                    body: &mut *body,
                    obstacle,
                }
                .solve_velocity(constraint);
            }
        }

        body.integrate_position(dt);

        let mut position_solved = island.is_empty();
        if !position_solved {
            for _ in 0..POSITION_ITERATIONS {
                let mut min_separation: f64 = 0.0;
                for (obstacle, constraint) in &mut island {
                    let separation = Solver {
                        body: &mut *body,
                        obstacle,
                    }
                    .solve_position(constraint);
                    min_separation = min_separation.min(separation);
                }
                if min_separation >= -3.0 * LINEAR_TOLERANCE {
                    position_solved = true;
                    break;
                }
            }
        }

        if body.update_at_rest_time(dt) >= MINIMUM_AT_REST_TIME && position_solved {
            body.set_at_rest(true);
        }
    }

    fn detect(&mut self, bodies: &[&mut RigidBody]) {
        for (b, body) in bodies.iter().enumerate() {
            for (o, obstacle) in self.obstacles.iter().enumerate() {
                let found = penetration(
                    &body.shape,
                    &body.transform,
                    &obstacle.shape,
                    &obstacle.transform,
                )
                .and_then(|(normal, _)| {
                    manifold(
                        &body.shape,
                        &body.transform,
                        &obstacle.shape,
                        &obstacle.transform,
                        normal,
                    )
                });
                let constraint = &mut self.constraints[b][o];
                constraint.active = found.is_some();
                if found.is_some() || !constraint.contacts.is_empty() {
                    constraint.update(found, body, obstacle);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.005;

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} vs {b}");
    }

    fn body(density: f64) -> RigidBody {
        let mut b = RigidBody::rectangle(0.76, 0.76, density, 0.65, 0.08);
        b.angular_damping = 0.0;
        b
    }

    fn step_free(b: &mut RigidBody, steps: usize) {
        let mut world = World::new(Vec::new());
        for _ in 0..steps {
            world.step(&mut [&mut *b], DT);
        }
    }

    fn wall(x: f64) -> Obstacle {
        Obstacle::new(
            Shape::Segment {
                a: V2::new(x, -5.0),
                b: V2::new(x, 5.0),
            },
            Transform::new(V2::ZERO, 0.0),
            0.6,
            0.3,
        )
    }

    #[test]
    fn mass_and_inertia_from_density() {
        let b = body(90.0);
        let m = 90.0 * 0.76 * 0.76;
        close(b.mass(), m, 1e-12);
        close(b.inertia(), m * (0.76 * 0.76 * 2.0) / 12.0, 1e-12);
    }

    #[test]
    fn constant_force_matches_semi_implicit_euler() {
        let mut b = body(100.0);
        let steps = 100;
        for _ in 0..steps {
            b.apply_force(V2::new(50.0, 0.0));
            step_free(&mut b, 1);
        }
        let n = steps as f64;
        let m = b.mass();
        close(b.linear_velocity.x, 50.0 * n * DT / m, 1e-12);
        close(
            b.transform.translation.x,
            50.0 / m * DT * DT * n * (n + 1.0) / 2.0,
            1e-12,
        );
        close(b.linear_velocity.y, 0.0, 1e-12);
    }

    #[test]
    fn impulse_conserves_momentum_and_energy() {
        let mut b = body(100.0);
        b.linear_velocity = V2::new(2.0, -1.0);
        let m = b.mass();
        let p0 = b.linear_velocity.scale(m);
        let e0 = 0.5 * m * b.linear_velocity.magnitude_squared();
        step_free(&mut b, 400);
        let p = b.linear_velocity.scale(m);
        close(p.x, p0.x, 1e-9);
        close(p.y, p0.y, 1e-9);
        close(0.5 * m * b.linear_velocity.magnitude_squared(), e0, 1e-9);
        close(b.transform.translation.x, 2.0 * 400.0 * DT, 1e-9);
        close(b.transform.translation.y, -(400.0 * DT), 1e-9);
    }

    #[test]
    fn linear_damping_scales_velocity_each_step() {
        let mut b = body(100.0);
        b.linear_damping = 1.4;
        b.angular_damping = 1.4;
        b.linear_velocity = V2::new(3.0, 0.0);
        b.angular_velocity = 2.0;
        step_free(&mut b, 10);
        let factor = (1.0 - DT * 1.4_f64).powi(10);
        close(b.linear_velocity.x, 3.0 * factor, 1e-12);
        close(b.angular_velocity, 2.0 * factor, 1e-12);
    }

    #[test]
    fn torque_free_spin_keeps_angular_momentum_and_center() {
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(3.0, 2.0), 0.3);
        b.angular_velocity = 1.5;
        step_free(&mut b, 200);
        close(b.angular_velocity, 1.5, 1e-12);
        close(b.transform.translation.x, 3.0, 1e-9);
        close(b.transform.translation.y, 2.0, 1e-9);
        close(b.transform.angle(), 0.3 + 1.5 * 200.0 * DT, 1e-9);
    }

    #[test]
    fn force_at_a_point_adds_torque_about_the_center() {
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(1.0, 1.0), 0.0);
        b.apply_force_at(V2::new(0.0, 10.0), V2::new(1.5, 1.0));
        step_free(&mut b, 1);
        close(b.linear_velocity.y, 10.0 / b.mass() * DT, 1e-12);
        close(b.angular_velocity, 0.5 * 10.0 / b.inertia() * DT, 1e-12);
    }

    #[test]
    fn velocity_at_a_point_adds_the_spin() {
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(1.0, 1.0), 0.0);
        b.linear_velocity = V2::new(1.0, 0.0);
        b.angular_velocity = 2.0;
        let v = b.linear_velocity_at(V2::new(2.0, 1.0));
        close(v.x, 1.0, 1e-12);
        close(v.y, 2.0, 1e-12);
    }

    #[test]
    fn slow_body_goes_to_rest_and_applied_force_wakes_it() {
        let mut b = body(100.0);
        b.linear_velocity = V2::new(0.005, 0.0);
        step_free(&mut b, 4);
        assert!(b.is_at_rest());
        close(b.linear_velocity.x, 0.0, 0.0);
        b.apply_force(V2::new(1.0, 0.0));
        assert!(!b.is_at_rest());
    }

    #[test]
    fn rectangle_against_wall_has_two_contacts_with_normal_into_the_body() {
        let w = wall(0.0);
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(0.38 - 0.01, 0.0), 0.0);
        let (n, depth) = penetration(&b.shape, &b.transform, &w.shape, &w.transform).unwrap();
        close(depth, 0.01, 1e-12);
        close(n.x, -1.0, 1e-12);
        let m = manifold(&b.shape, &b.transform, &w.shape, &w.transform, n).unwrap();
        assert_eq!(m.points.len(), 2);
        close(m.normal.x, 1.0, 1e-12);
        close(m.normal.y, 0.0, 1e-12);
        for p in &m.points {
            close(p.depth, 0.01, 1e-12);
        }
    }

    #[test]
    fn separated_shapes_have_no_penetration() {
        let w = wall(0.0);
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(0.5, 0.0), 0.0);
        assert!(penetration(&b.shape, &b.transform, &w.shape, &w.transform).is_none());
    }

    #[test]
    fn wall_hit_bounces_with_the_mixed_restitution() {
        let mut world = World::new(vec![wall(0.0)]);
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(1.0, 0.0), 0.0);
        b.linear_velocity = V2::new(-3.0, 0.0);
        let mut bounced = false;
        for _ in 0..400 {
            world.step(&mut [&mut b], DT);
            if b.linear_velocity.x > 0.0 {
                bounced = true;
                // max(0.08, 0.3) of the approach speed, less a little damping of the step
                close(b.linear_velocity.x, 0.3 * 3.0, 0.05);
                break;
            }
        }
        assert!(bounced);
        assert!(b.transform.translation.x >= 0.38 - 0.03);
    }

    #[test]
    fn slow_wall_hit_does_not_bounce_and_stops_at_the_wall() {
        let mut world = World::new(vec![wall(0.0)]);
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(1.0, 0.0), 0.0);
        b.linear_velocity = V2::new(-0.8, 0.0);
        for _ in 0..600 {
            world.step(&mut [&mut b], DT);
        }
        assert!(b.linear_velocity.x.abs() < 0.02, "{}", b.linear_velocity.x);
        // Rests within dyn4j's allowed penetration of the face at x = 0.38
        close(b.transform.translation.x, 0.38, 0.01);
    }

    #[test]
    fn sustained_push_into_a_wall_stays_inside_the_penetration_tolerance() {
        let mut world = World::new(vec![wall(0.0)]);
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(0.5, 0.0), 0.0);
        for _ in 0..800 {
            b.apply_force(V2::new(-400.0, 0.0));
            world.step(&mut [&mut b], DT);
        }
        let face = b.transform.translation.x - 0.38;
        assert!(face > -3.0 * LINEAR_TOLERANCE, "{face}");
        assert!(b.linear_velocity.x.abs() < 0.05);
        assert!(b.transform.angle().abs() < 0.05);
    }

    #[test]
    fn friction_slows_a_body_sliding_along_a_wall() {
        let mut world = World::new(vec![wall(0.0)]);
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(0.37, 0.0), 0.0);
        b.linear_velocity = V2::new(-1.5, 3.0);
        for _ in 0..40 {
            world.step(&mut [&mut b], DT);
        }
        assert!(b.linear_velocity.y < 3.0);
        assert!(b.linear_velocity.y > 0.0);
    }

    #[test]
    fn teleport_into_open_space_leaves_the_body_unconstrained() {
        let mut world = World::new(vec![wall(0.0)]);
        let mut b = body(100.0);
        b.transform = Transform::new(V2::new(0.3, 0.0), 0.0);
        for _ in 0..4 {
            world.step(&mut [&mut b], DT);
        }
        b.transform = Transform::new(V2::new(3.0, 0.0), 0.0);
        b.linear_velocity = V2::new(1.0, 0.0);
        for _ in 0..10 {
            world.step(&mut [&mut b], DT);
        }
        assert!(b.transform.translation.x > 3.0);
    }
}
