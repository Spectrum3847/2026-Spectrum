use nalgebra::{DMatrix, SMatrix};

// Padé coefficients and degree thresholds from Eigen's unsupported MatrixFunctions
// MatrixExponential (MPL-2.0), as bundled in WPILib, so results match WPILib's `Matrix.exp`.
const B3: [f64; 4] = [120.0, 60.0, 12.0, 1.0];
const B5: [f64; 6] = [30240.0, 15120.0, 3360.0, 420.0, 30.0, 1.0];
const B7: [f64; 8] = [
    17297280.0, 8648640.0, 1995840.0, 277200.0, 25200.0, 1512.0, 56.0, 1.0,
];
const B9: [f64; 10] = [
    17643225600.0,
    8821612800.0,
    2075673600.0,
    302702400.0,
    30270240.0,
    2162160.0,
    110880.0,
    3960.0,
    90.0,
    1.0,
];
const B13: [f64; 14] = [
    64764752532480000.0,
    32382376266240000.0,
    7771770303897600.0,
    1187353796428800.0,
    129060195264000.0,
    10559470521600.0,
    670442572800.0,
    33522128640.0,
    1323241920.0,
    40840800.0,
    960960.0,
    16380.0,
    182.0,
    1.0,
];

/// `sum(coef * matrix) + identity_coef * I`, summed element-wise left to right.
fn lin(terms: &[(f64, &DMatrix<f64>)], identity_coef: f64) -> DMatrix<f64> {
    let (n, m) = terms[0].1.shape();
    DMatrix::from_fn(n, m, |r, c| {
        let mut acc = terms[0].0 * terms[0].1[(r, c)];
        for (coef, mat) in &terms[1..] {
            acc += coef * mat[(r, c)];
        }
        if r == c {
            acc += identity_coef;
        }
        acc
    })
}

/// Padé numerator and denominator parts `(U, V)` for the matrix `a`.
fn pade_uv(a: &DMatrix<f64>, degree: u32) -> (DMatrix<f64>, DMatrix<f64>) {
    let a2 = a * a;
    match degree {
        3 => {
            let tmp = lin(&[(B3[3], &a2)], B3[1]);
            (a * tmp, lin(&[(B3[2], &a2)], B3[0]))
        }
        5 => {
            let a4 = &a2 * &a2;
            let tmp = lin(&[(B5[5], &a4), (B5[3], &a2)], B5[1]);
            (a * tmp, lin(&[(B5[4], &a4), (B5[2], &a2)], B5[0]))
        }
        7 => {
            let a4 = &a2 * &a2;
            let a6 = &a4 * &a2;
            let tmp = lin(&[(B7[7], &a6), (B7[5], &a4), (B7[3], &a2)], B7[1]);
            (
                a * tmp,
                lin(&[(B7[6], &a6), (B7[4], &a4), (B7[2], &a2)], B7[0]),
            )
        }
        9 => {
            let a4 = &a2 * &a2;
            let a6 = &a4 * &a2;
            let a8 = &a6 * &a2;
            let tmp = lin(
                &[(B9[9], &a8), (B9[7], &a6), (B9[5], &a4), (B9[3], &a2)],
                B9[1],
            );
            (
                a * tmp,
                lin(
                    &[(B9[8], &a8), (B9[6], &a6), (B9[4], &a4), (B9[2], &a2)],
                    B9[0],
                ),
            )
        }
        _ => {
            let a4 = &a2 * &a2;
            let a6 = &a4 * &a2;
            let v = lin(&[(B13[13], &a6), (B13[11], &a4), (B13[9], &a2)], 0.0);
            let mut tmp = &a6 * v;
            tmp += lin(&[(B13[7], &a6), (B13[5], &a4), (B13[3], &a2)], B13[1]);
            let u = a * tmp;
            let tmp = lin(&[(B13[12], &a6), (B13[10], &a4), (B13[8], &a2)], 0.0);
            let mut v = &a6 * tmp;
            v += lin(&[(B13[6], &a6), (B13[4], &a4), (B13[2], &a2)], B13[0]);
            (u, v)
        }
    }
}

/// Exponent `e` with `x = m * 2^e` and `m` in `[0.5, 1)`, like C `frexp`.
fn frexp_exponent(x: f64) -> i32 {
    if x == 0.0 || !x.is_finite() {
        return 0;
    }
    let exp = ((x.to_bits() >> 52) & 0x7ff) as i32;
    if exp == 0 {
        // Subnormal: scale into the normal range first.
        return frexp_exponent(x * 2f64.powi(54)) - 54;
    }
    exp - 1022
}

/// Matrix exponential by Padé approximation with scaling and squaring, as Eigen does.
///
/// # Panics
/// If `a` is not square.
pub fn matrix_exp(a: &DMatrix<f64>) -> DMatrix<f64> {
    assert!(
        a.is_square(),
        "Non-square matrices cannot be exponentiated!"
    );
    let l1_norm = (0..a.ncols())
        .map(|c| a.column(c).iter().map(|v| v.abs()).sum::<f64>())
        .fold(0.0_f64, f64::max);
    let mut squarings = 0;
    let (u, v) = if l1_norm < 1.495585217958292e-2 {
        pade_uv(a, 3)
    } else if l1_norm < 2.539_398_330_063_23e-1 {
        pade_uv(a, 5)
    } else if l1_norm < 9.504178996162932e-1 {
        pade_uv(a, 7)
    } else if l1_norm < 2.097847961257068 {
        pade_uv(a, 9)
    } else {
        let max_norm = 5.371920351148152;
        squarings = frexp_exponent(l1_norm / max_norm).max(0);
        let scaled = a * 2f64.powi(-squarings);
        pade_uv(&scaled, 13)
    };
    let numer = &u + &v;
    let denom = &v - &u;
    let mut result = denom
        .lu()
        .solve(&numer)
        .expect("Padé denominator is singular");
    for _ in 0..squarings {
        result = &result * &result;
    }
    result
}

/// Exact zero-order-hold discretization of `x' = A x + B u` over `dt_seconds`.
pub fn discretize_ab<const S: usize, const I: usize>(
    cont_a: &SMatrix<f64, S, S>,
    cont_b: &SMatrix<f64, S, I>,
    dt_seconds: f64,
) -> (SMatrix<f64, S, S>, SMatrix<f64, S, I>) {
    let mut m = DMatrix::zeros(S + I, S + I);
    m.view_mut((0, 0), (S, S)).copy_from(cont_a);
    m.view_mut((0, S), (S, I)).copy_from(cont_b);
    let phi = matrix_exp(&(m * dt_seconds));
    let disc_a = SMatrix::<f64, S, S>::from_fn(|r, c| phi[(r, c)]);
    let disc_b = SMatrix::<f64, S, I>::from_fn(|r, c| phi[(r, S + c)]);
    (disc_a, disc_b)
}
