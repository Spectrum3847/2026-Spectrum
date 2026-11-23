use nalgebra::SVector;

// Butcher tableau, rows zero-padded to six columns.
const A: [[f64; 6]; 6] = [
    [1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0],
    [44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0],
    [
        19372.0 / 6561.0,
        -25360.0 / 2187.0,
        64448.0 / 6561.0,
        -212.0 / 729.0,
        0.0,
        0.0,
    ],
    [
        9017.0 / 3168.0,
        -355.0 / 33.0,
        46732.0 / 5247.0,
        49.0 / 176.0,
        -5103.0 / 18656.0,
        0.0,
    ],
    [
        35.0 / 384.0,
        0.0,
        500.0 / 1113.0,
        125.0 / 192.0,
        -2187.0 / 6784.0,
        11.0 / 84.0,
    ],
];
const B1: [f64; 7] = [
    35.0 / 384.0,
    0.0,
    500.0 / 1113.0,
    125.0 / 192.0,
    -2187.0 / 6784.0,
    11.0 / 84.0,
    0.0,
];
const B2: [f64; 7] = [
    5179.0 / 57600.0,
    0.0,
    7571.0 / 16695.0,
    393.0 / 640.0,
    -92097.0 / 339200.0,
    187.0 / 2100.0,
    1.0 / 40.0,
];

/// Scaled Frobenius norm, computed like EJML's `normF` so step control matches WPILib bit for bit.
fn frobenius_norm<const N: usize>(v: &SVector<f64, N>) -> f64 {
    let scale = v.iter().fold(0.0_f64, |m, e| m.max(e.abs()));
    if scale == 0.0 {
        return 0.0;
    }
    let total: f64 = v.iter().map(|e| (e / scale) * (e / scale)).sum();
    scale * total.sqrt()
}

/// Dormand-Prince integration of `x' = f(x, u)` over `dt_seconds` with error limit 1e-6.
pub fn rkdp<const N: usize, const M: usize>(
    f: impl FnMut(&SVector<f64, N>, &SVector<f64, M>) -> SVector<f64, N>,
    x: SVector<f64, N>,
    u: SVector<f64, M>,
    dt_seconds: f64,
) -> SVector<f64, N> {
    rkdp_with_error(f, x, u, dt_seconds, 1e-6)
}

/// Adaptive Dormand-Prince step control.
///
/// Like WPILib, the elapsed time advances by the already-rescaled step size, not the size
/// of the step just taken.
pub fn rkdp_with_error<const N: usize, const M: usize>(
    mut f: impl FnMut(&SVector<f64, N>, &SVector<f64, M>) -> SVector<f64, N>,
    mut x: SVector<f64, N>,
    u: SVector<f64, M>,
    dt_seconds: f64,
    max_error: f64,
) -> SVector<f64, N> {
    let mut dt_elapsed = 0.0;
    let mut h = dt_seconds;
    while dt_elapsed < dt_seconds {
        loop {
            h = h.min(dt_seconds - dt_elapsed);
            let k1 = f(&x, &u);
            let k2 = f(&(x + (k1 * A[0][0]) * h), &u);
            let k3 = f(&(x + (k1 * A[1][0] + k2 * A[1][1]) * h), &u);
            let k4 = f(&(x + (k1 * A[2][0] + k2 * A[2][1] + k3 * A[2][2]) * h), &u);
            let k5 = f(
                &(x + (k1 * A[3][0] + k2 * A[3][1] + k3 * A[3][2] + k4 * A[3][3]) * h),
                &u,
            );
            let k6 = f(
                &(x + (k1 * A[4][0] + k2 * A[4][1] + k3 * A[4][2] + k4 * A[4][3] + k5 * A[4][4])
                    * h),
                &u,
            );
            let new_x = x
                + (k1 * A[5][0]
                    + k2 * A[5][1]
                    + k3 * A[5][2]
                    + k4 * A[5][3]
                    + k5 * A[5][4]
                    + k6 * A[5][5])
                    * h;
            let k7 = f(&new_x, &u);
            let truncation_error = frobenius_norm(
                &((k1 * (B1[0] - B2[0])
                    + k2 * (B1[1] - B2[1])
                    + k3 * (B1[2] - B2[2])
                    + k4 * (B1[3] - B2[3])
                    + k5 * (B1[4] - B2[4])
                    + k6 * (B1[5] - B2[5])
                    + k7 * (B1[6] - B2[6]))
                    * h),
            );
            if truncation_error == 0.0 {
                h = dt_seconds - dt_elapsed;
            } else {
                h *= 0.9 * (max_error / truncation_error).powf(1.0 / 5.0);
            }
            // A NaN error is accepted, as in WPILib, so the loop always ends.
            if truncation_error > max_error {
                continue;
            }
            dt_elapsed += h;
            x = new_x;
            break;
        }
    }
    x
}
