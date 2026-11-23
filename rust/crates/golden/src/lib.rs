//! Test helper that checks Rust results against Java golden fixtures.
//!
//! Fixtures live in `rust/fixtures/<name>.json`. The Java robot code wrote them before the
//! cutover to Rust; they are frozen now. Each case holds an op name, input values and the
//! Java outputs.

use serde::Deserialize;
use std::path::PathBuf;

#[derive(Deserialize)]
pub struct Case {
    pub op: String,
    #[serde(rename = "in")]
    pub input: Vec<f64>,
    pub out: Vec<f64>,
}

#[derive(Deserialize)]
struct File {
    cases: Vec<Case>,
}

pub fn load(name: &str) -> Vec<Case> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../fixtures")
        .join(format!("{name}.json"));
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let file: File =
        serde_json::from_str(&text).unwrap_or_else(|e| panic!("parse {}: {e}", path.display()));
    assert!(!file.cases.is_empty(), "{name}: no cases");
    file.cases
}

/// Absolute plus relative tolerance, so large and near-zero values both compare sanely.
pub fn close(a: f64, b: f64, tol: f64) -> bool {
    if a.is_nan() || b.is_nan() {
        return a.is_nan() && b.is_nan();
    }
    if a.is_infinite() || b.is_infinite() {
        return a == b;
    }
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
}

/// Runs every case in a fixture through `eval` and panics with all mismatches.
/// `eval` returns `None` for an op it does not know, which fails the test.
pub fn check(name: &str, tol: f64, eval: impl FnMut(&str, &[f64]) -> Option<Vec<f64>>) {
    check_angles(name, tol, |_, _| false, eval);
}

/// Like [`check`], but outputs where `is_angle(op, index)` is true compare modulo a full turn.
/// Use it only for angles Java computes with `atan2`: on the pi branch cut the sign of a
/// last-bit sine (which differs between math libraries) picks pi or -pi.
pub fn check_angles(
    name: &str,
    tol: f64,
    is_angle: impl Fn(&str, usize) -> bool,
    mut eval: impl FnMut(&str, &[f64]) -> Option<Vec<f64>>,
) {
    let mut failures = Vec::new();
    let cases = load(name);
    for (i, c) in cases.iter().enumerate() {
        match eval(&c.op, &c.input) {
            None => failures.push(format!("#{i} {}: unknown op", c.op)),
            Some(got) => {
                let ok = got.len() == c.out.len()
                    && got.iter().zip(&c.out).enumerate().all(|(k, (g, w))| {
                        if is_angle(&c.op, k) {
                            close_angle(*g, *w, tol)
                        } else {
                            close(*g, *w, tol)
                        }
                    });
                if !ok {
                    failures.push(format!(
                        "#{i} {} in={:?}\n  want={:?}\n  got ={:?}",
                        c.op, c.input, c.out, got
                    ));
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "{name}: {} of {} cases failed\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
}

/// Radians equal modulo a full turn, within `tol`.
pub fn close_angle(a: f64, b: f64, tol: f64) -> bool {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    d.min(std::f64::consts::TAU - d) <= tol
}

#[cfg(test)]
mod tests {
    use super::close;

    #[test]
    fn close_handles_scale_and_specials() {
        assert!(close(1e9, 1e9 + 0.5, 1e-9));
        assert!(!close(0.0, 1e-6, 1e-9));
        assert!(close(f64::NAN, f64::NAN, 1e-9));
        assert!(!close(f64::INFINITY, 1e300, 1e-9));
    }

    #[test]
    fn close_angle_wraps_at_pi() {
        use super::close_angle;
        use std::f64::consts::PI;
        assert!(close_angle(PI, -PI, 1e-9));
        assert!(close_angle(0.1, 0.1 + 2.0 * PI, 1e-9));
        assert!(!close_angle(0.1, 0.2, 1e-9));
    }
}
