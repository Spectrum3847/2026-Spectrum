use wpimath::geometry::Translation2d;
use wpimath::interpolation::Interpolatable;

pub fn quadratic_lerp(
    a: Translation2d,
    b: Translation2d,
    c: Translation2d,
    t: f64,
) -> Translation2d {
    let p0 = a.interpolate(&b, t);
    let p1 = b.interpolate(&c, t);
    p0.interpolate(&p1, t)
}

pub fn cubic_lerp(
    a: Translation2d,
    b: Translation2d,
    c: Translation2d,
    d: Translation2d,
    t: f64,
) -> Translation2d {
    let p0 = quadratic_lerp(a, b, c, t);
    let p1 = quadratic_lerp(b, c, d, t);
    p0.interpolate(&p1, t)
}

/// Signed radius of the circle through three points. Infinite for collinear points.
pub fn calculate_radius(a: Translation2d, b: Translation2d, c: Translation2d) -> f64 {
    let vba = a - b;
    let vbc = c - b;
    let cross_z = (vba.x * vbc.y) - (vba.y * vbc.x);
    let sign = if cross_z < 0.0 { 1.0 } else { -1.0 };

    let ab = a.distance(b);
    let bc = b.distance(c);
    let ac = a.distance(c);

    let p = (ab + bc + ac) / 2.0;
    let area = (p * (p - ab) * (p - bc) * (p - ac)).abs().sqrt();
    sign * (ab * bc * ac) / (4.0 * area)
}
