/**
 * Camera calibration maths. Pure functions, no DOM, so node:test can run them against frames
 * captured from the real cameras (test/fixtures/limelight-*.json).
 *
 * Two independent ways to measure how a Limelight is mounted, both usable on a robot sitting still
 * on a flat floor and neither needing to know where the robot is:
 *
 *  - The camera's own accelerometer. Gravity in the camera frame gives pitch and roll directly.
 *    Works with no tag in view, so it is the always-on sanity check.
 *  - The AprilTag solve. `t6c_ts` is the camera's pose in the tag's frame, and field tags hang
 *    vertically at a known height, so the camera's optical axis against the tag's vertical is its
 *    pitch, its horizontal axis against it is its roll, and its offset below the tag centre is its
 *    height. Sharper than the accelerometer, and it also gives the height, but needs a tag.
 *
 * Neither can give forward, right or yaw: those need a surveyed robot position, so they stay CAD.
 *
 * Conventions, checked against captures from this robot on 2026-09-07 (see the tests):
 *
 *  - Limelight target space: X right (facing the tag), Y down, Z out of the tag toward the camera.
 *  - Limelight camera space: X right, Y down, Z out of the lens. The three rotation numbers are
 *    roll, pitch, yaw in degrees, applied as Rz * Ry * Rx, the same reading LimelightHelpers uses
 *    when it builds a WPILib Rotation3d from them.
 *  - Pitch is positive with the camera tilted up; roll is positive with the right side down, and
 *    an upside-down camera has a roll near 180. That is the convention Vision.java writes.
 *  - The camera IMU reports proper acceleration in g with X pointing out the BACK of the camera
 *    and Z toward the camera's top. A camera tilted up by p therefore reads ax = -sin p, and one
 *    mounted upside down reads az < 0.
 */

const DEG = Math.PI / 180;

export const mean = (xs) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : NaN);

export function std(xs) {
    if (xs.length < 2) return NaN;
    const m = mean(xs);
    return Math.sqrt(xs.reduce((a, x) => a + (x - m) ** 2, 0) / (xs.length - 1));
}

/** Wraps to (-180, 180]. */
export function wrapDeg(d) {
    let x = ((d + 180) % 360 + 360) % 360 - 180;
    if (x === -180) x = 180;
    return x;
}

/** Mean of angles that may straddle +/-180, e.g. rolls near 180. */
export function circularMeanDeg(degs) {
    if (!degs.length) return NaN;
    const s = mean(degs.map((d) => Math.sin(d * DEG)));
    const c = mean(degs.map((d) => Math.cos(d * DEG)));
    return Math.atan2(s, c) / DEG;
}

/** Rotation matrix for Limelight's roll/pitch/yaw degrees: R = Rz(yaw) * Ry(pitch) * Rx(roll). */
export function rotationMatrix(rollDeg, pitchDeg, yawDeg) {
    const [rx, ry, rz] = [rollDeg * DEG, pitchDeg * DEG, yawDeg * DEG];
    const [cx, sx, cy, sy, cz, sz] = [Math.cos(rx), Math.sin(rx), Math.cos(ry), Math.sin(ry), Math.cos(rz), Math.sin(rz)];
    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ];
}

/**
 * The camera's mount angles and height from one tag observation.
 *
 * @param {number[]} t6c_ts camera pose in target space: x, y, z metres, then roll, pitch, yaw deg
 * @param {number} tagHeightM height of that tag's centre above the floor, from the field layout
 * @returns {{pitchDeg: number, rollDeg: number, heightM: number, rangeM: number}}
 */
export function mountFromTagSolve(t6c_ts, tagHeightM) {
    const [x, y, z, rr, rp, ry] = t6c_ts;
    const R = rotationMatrix(rr, rp, ry);
    // Columns of R are the camera axes expressed in target space. Target Y is down.
    const rightY = R[1][0];
    const downY = R[1][1];
    const opticalY = R[1][2];
    return {
        pitchDeg: Math.asin(Math.max(-1, Math.min(1, -opticalY))) / DEG,
        rollDeg: wrapDeg(Math.atan2(rightY, downY) / DEG),
        heightM: tagHeightM - y,
        rangeM: Math.hypot(x, y, z),
    };
}

/**
 * The camera's mount angles from its accelerometer.
 *
 * Pitch is well determined. Roll is |roll| plus a sign this code could not verify on a real
 * misaligned mount, so treat it as a magnitude; the tag solve's roll carries the sign.
 *
 * @param {number[]} accel proper acceleration in g, the last three numbers of results.imu.data
 */
export function mountFromAccel(accel) {
    const [ax, ay, az] = accel;
    return {
        pitchDeg: Math.atan2(-ax, Math.hypot(ay, az)) / DEG,
        rollDeg: wrapDeg(Math.atan2(ay, az) / DEG),
        gMagnitude: Math.hypot(ax, ay, az),
    };
}

/** Tag ID -> centre height in metres, from a WPILib AprilTagFieldLayout JSON. */
export function tagHeightsFromLayout(layout) {
    const out = new Map();
    for (const t of layout?.tags || []) out.set(t.ID, t.pose.translation.z);
    return out;
}

/**
 * Reduces a run of captured frames to one measurement of the mount.
 *
 * @param {object[]} frames Limelight /results documents, one per distinct frame
 * @param {Map<number, number>} tagHeights from tagHeightsFromLayout
 * @param {{minTagFrames?: number, maxAmbiguity?: number}} [opts]
 */
export function summarizeMount(frames, tagHeights, opts = {}) {
    const minTagFrames = opts.minTagFrames ?? 10;
    const maxAmbiguity = opts.maxAmbiguity ?? 0.5;

    const accelSamples = frames.map((f) => f.imu?.data?.slice(7, 10)).filter((a) => a && a.length === 3 && a.every(Number.isFinite));
    const accel = accelSamples.map(mountFromAccel);
    const accelSummary = accel.length
        ? {
              n: accel.length,
              pitchDeg: mean(accel.map((a) => a.pitchDeg)),
              pitchStd: std(accel.map((a) => a.pitchDeg)),
              rollDeg: circularMeanDeg(accel.map((a) => a.rollDeg)),
              gMagnitude: mean(accel.map((a) => a.gMagnitude)),
          }
        : null;

    const perTag = new Map();
    for (const f of frames) {
        for (const fid of f.Fiducial || []) {
            if (!Array.isArray(fid.t6c_ts) || fid.t6c_ts.length < 6) continue;
            if (fid.ambig > maxAmbiguity) continue;
            const h = tagHeights.get(fid.fID);
            if (h === undefined) continue;
            const m = mountFromTagSolve(fid.t6c_ts, h);
            if (!perTag.has(fid.fID)) perTag.set(fid.fID, []);
            perTag.get(fid.fID).push({ ...m, ambig: fid.ambig });
        }
    }
    const tags = [...perTag.entries()]
        .map(([id, ms]) => ({
            id,
            n: ms.length,
            pitchDeg: mean(ms.map((m) => m.pitchDeg)),
            pitchStd: std(ms.map((m) => m.pitchDeg)),
            rollDeg: circularMeanDeg(ms.map((m) => m.rollDeg)),
            heightM: mean(ms.map((m) => m.heightM)),
            heightStd: std(ms.map((m) => m.heightM)),
            rangeM: mean(ms.map((m) => m.rangeM)),
            ambig: mean(ms.map((m) => m.ambig)),
        }))
        .sort((a, b) => a.id - b.id);

    // Every tag observation weighs the same, so a tag seen in more frames counts for more.
    const all = [...perTag.values()].flat();
    const tagSummary =
        all.length >= minTagFrames
            ? {
                  n: all.length,
                  tagCount: tags.length,
                  pitchDeg: mean(all.map((m) => m.pitchDeg)),
                  pitchStd: std(all.map((m) => m.pitchDeg)),
                  rollDeg: circularMeanDeg(all.map((m) => m.rollDeg)),
                  heightM: mean(all.map((m) => m.heightM)),
                  heightStd: std(all.map((m) => m.heightM)),
              }
            : null;

    // What we would write: the tag solve where we have it, the accelerometer otherwise.
    const source = tagSummary ? "tags" : accelSummary ? "accel" : null;
    const proposed = source
        ? {
              source,
              pitchDeg: (tagSummary || accelSummary).pitchDeg,
              rollDeg: (tagSummary || accelSummary).rollDeg,
              upM: tagSummary ? tagSummary.heightM : null,
          }
        : null;

    const agreementDeg = tagSummary && accelSummary ? Math.abs(tagSummary.pitchDeg - accelSummary.pitchDeg) : null;

    return { frames: frames.length, accel: accelSummary, tags, tagSummary, proposed, agreementDeg };
}

/** Thresholds below which a measured difference is not worth a write. */
export const WRITE_THRESHOLDS = { pitchDeg: 0.3, rollDeg: 0.5, upM: 0.01 };

/**
 * Compares a measurement with what Vision.java has and says what is worth changing.
 *
 * @param {{roll: number, pitch: number, up: number}} current values from Vision.java
 * @param {{pitchDeg: number, rollDeg: number, upM: number|null}} measured
 */
export function proposeWrites(current, measured) {
    const rows = [];
    const add = (key, label, from, to, threshold, fmt) => {
        if (to === null || to === undefined || !Number.isFinite(to)) return;
        const delta = key === "roll" ? wrapDeg(to - from) : to - from;
        rows.push({ key, label, from, to, delta, significant: Math.abs(delta) >= threshold, fmt });
    };
    add("pitch", "Pitch", current.pitch, measured.pitchDeg, WRITE_THRESHOLDS.pitchDeg, "deg");
    add("roll", "Roll", current.roll, measured.rollDeg, WRITE_THRESHOLDS.rollDeg, "deg");
    add("up", "Height (up)", current.up, measured.upM, WRITE_THRESHOLDS.upM, "m");
    return rows;
}

// ---------------------------------------------------------------------------
// Image tuning: how well is the camera seeing tags at a given exposure / gain / black level?
// ---------------------------------------------------------------------------

/**
 * Detection quality over a run of frames at one setting.
 *
 * @param {object[]} frames Limelight /results documents, one per distinct frame
 * @returns {{frames, detectRate, meanTags, meanAmbig, cornerJitterPx, poseJitterM, latencyMs}}
 */
export function summarizeFrames(frames) {
    const n = frames.length;
    if (!n) return { frames: 0, detectRate: 0, meanTags: 0, meanAmbig: null, cornerJitterPx: null, poseJitterM: null, latencyMs: null };

    const counts = frames.map((f) => (f.Fiducial || []).length);
    const ambigs = frames.flatMap((f) => (f.Fiducial || []).map((t) => t.ambig).filter(Number.isFinite));

    // Corner jitter: the same physical tag corner, frame to frame, on a still robot. Any spread is
    // detection noise, which is what the exposure is being tuned to minimise.
    const cornersById = new Map();
    for (const f of frames) {
        for (const t of f.Fiducial || []) {
            if (!Array.isArray(t.pts) || t.pts.length !== 4) continue;
            if (!cornersById.has(t.fID)) cornersById.set(t.fID, []);
            cornersById.get(t.fID).push(t.pts.flat());
        }
    }
    const cornerVariances = [];
    for (const runs of cornersById.values()) {
        if (runs.length < 3) continue;
        for (let i = 0; i < 8; i++) {
            const s = std(runs.map((r) => r[i]));
            if (Number.isFinite(s)) cornerVariances.push(s * s);
        }
    }
    const cornerJitterPx = cornerVariances.length ? Math.sqrt(mean(cornerVariances)) : null;

    const poses = frames.filter((f) => (f.botpose_tagcount || 0) > 0 && Array.isArray(f.botpose)).map((f) => f.botpose);
    const poseJitterM = poses.length >= 3 ? Math.hypot(std(poses.map((p) => p[0])), std(poses.map((p) => p[1]))) : null;

    const lat = frames.map((f) => (f.tl || 0) + (f.cl || 0)).filter((v) => v > 0);

    return {
        frames: n,
        detectRate: counts.filter((c) => c > 0).length / n,
        meanTags: mean(counts),
        meanAmbig: ambigs.length ? mean(ambigs) : null,
        cornerJitterPx,
        poseJitterM,
        latencyMs: lat.length ? mean(lat) : null,
    };
}

/**
 * One number to rank settings by. Seeing tags at all dominates; then how many; then how cleanly
 * (ambiguity, corner jitter, pose jitter). The tiny exposure term only breaks ties, in favour of
 * the shorter exposure, because less exposure is less motion blur once the robot moves.
 *
 * @param {object} m from summarizeFrames
 * @param {number} [exposure] the exposure setting the sample was taken at (0.01 ms units)
 */
export function scoreMetrics(m, exposure = 0) {
    if (!m.frames) return -Infinity;
    return (
        100 * m.detectRate +
        10 * m.meanTags -
        20 * (m.meanAmbig ?? 1) -
        2 * (m.cornerJitterPx ?? 5) -
        200 * (m.poseJitterM ?? 0.05) -
        exposure / 1000
    );
}

/**
 * The best row of a sweep, or null if no setting saw a tag.
 *
 * @param {{value: number, metrics: object, exposure?: number}[]} rows
 */
export function pickBest(rows) {
    let best = null;
    for (const r of rows) {
        if (!r.metrics || r.metrics.detectRate <= 0) continue;
        const score = scoreMetrics(r.metrics, r.exposure ?? r.value);
        if (!best || score > best.score) best = { ...r, score };
    }
    return best;
}

/** Default sweep candidates. Exposure is in the camera's 0.01 ms units, as the pipeline stores it. */
export const DEFAULT_SWEEPS = {
    exposure: [100, 150, 200, 300, 400, 500, 650, 800, 1000, 1300],
    lcgain: [1.5, 2, 2.7, 3.5, 4.5, 6],
    black_level: [0, 5, 10, 15, 25],
};
