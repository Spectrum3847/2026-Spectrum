//! Proves the Java to Rust fixture round trip with a trivial op.

#[test]
fn rotation2d_matches_java() {
    golden::check("smoke", 1e-12, |op, x| match op {
        "rotation2d" => {
            let (s, c) = x[0].sin_cos();
            // Java keeps the radians it was given, unwrapped.
            Some(vec![c, s, x[0]])
        }
        _ => None,
    });
}
