/**
 * Turret analyses, aimed at the question "is it slipping?".
 *
 * The strong signal here is that the turret has TWO independent measurements of where it points:
 *
 *   Turret/PositionDegrees            where the encoder thinks it is
 *   Vision/TurretLL/HeadingErrorDeg   how far the turret camera's view disagrees with the gyro
 *
 * The handoff doc records that the second key *is* the turret zero error. So a steady offset means
 * the zero is wrong by that many degrees -- annoying, fixed by re-zeroing. An offset that WALKS
 * during a session means the mechanism moved relative to its encoder, which is what slipping is.
 * Telling those two apart is the whole point of this page, and it is only possible because the
 * camera is an independent witness.
 */
import { inWindows } from "./log-model.js";

/** Sample a change-only-logged series at time t. */
export function valueAt(series, t) {
    if (!series.length || t < series[0][0]) return null;
    let lo = 0;
    let hi = series.length - 1;
    while (lo < hi) {
        const mid = (lo + hi + 1) >> 1;
        if (series[mid][0] <= t) lo = mid;
        else hi = mid - 1;
    }
    return series[lo][1];
}

function median(values) {
    if (!values.length) return null;
    const s = [...values].sort((a, b) => a - b);
    return s[s.length >> 1];
}

/** Least-squares slope of v against t, in units per second. */
function slope(series) {
    if (series.length < 10) return null;
    let st = 0, sv = 0, stt = 0, stv = 0;
    for (const [t, v] of series) {
        st += t; sv += v; stt += t * t; stv += t * v;
    }
    const n = series.length;
    const denom = n * stt - st * st;
    if (Math.abs(denom) < 1e-9) return null;
    return (n * stv - st * sv) / denom;
}

/**
 * Windows the turret was deliberately slewing a full turn to unwrap its wire.
 *
 * Every motion check has to exclude these. An unwrap is a legitimate ~360 degree jump -- the
 * handoff doc records one at t=101s going -206 to +153 mid-shot -- and would otherwise read as the
 * largest slip event in the log.
 */
export function unwrapWindows(model, padSec = 0.3) {
    const flag = model.ch("Turret/Unwrapping");
    const out = [];
    let open = null;
    for (const [t, v] of flag) {
        if (v && open === null) open = t;
        else if (!v && open !== null) {
            out.push([open - padSec, t + padSec]);
            open = null;
        }
    }
    if (open !== null) out.push([open - padSec, model.log.lastTs]);
    return out;
}

/**
 * Zero-error behaviour from the turret camera.
 *
 * Returns the drift rate, the baseline offset, and any step changes -- a step that persists is a
 * slip event; camera noise comes back.
 */
export function zeroErrorAnalysis(model, { stepDeg = 3, settleSec = 1.0 } = {}) {
    const raw = model.ch("Vision/TurretLL/HeadingErrorDeg");
    if (raw.length < 20) return null;

    // Only trust samples while enabled and not mid-unwrap: during a slew the camera's transform
    // lags the turret, which looks like error but is not.
    const unwraps = unwrapWindows(model);
    const usable = raw.filter(([t]) => !inWindows(t, unwraps));
    if (usable.length < 20) return null;

    const firstTenth = usable.slice(0, Math.max(10, Math.floor(usable.length * 0.1)));
    const lastTenth = usable.slice(-Math.max(10, Math.floor(usable.length * 0.1)));
    const baseline = median(firstTenth.map(([, v]) => v));
    const ending = median(lastTenth.map(([, v]) => v));
    const perSec = slope(usable);

    // A step is a jump between consecutive samples that is still there a second later.
    const steps = [];
    for (let i = 1; i < usable.length; i++) {
        const jump = usable[i][1] - usable[i - 1][1];
        if (Math.abs(jump) < stepDeg) continue;
        const after = usable.filter(([t]) => t > usable[i][0] && t <= usable[i][0] + settleSec).map(([, v]) => v);
        if (!after.length) continue;
        const held = median(after);
        if (Math.abs(held - usable[i - 1][1]) >= stepDeg * 0.7) {
            steps.push({ t: usable[i][0], from: usable[i - 1][1], to: held, jump: held - usable[i - 1][1] });
        }
    }

    // Collapse steps closer together than a second; one physical event can trip several samples.
    const merged = [];
    for (const s of steps) {
        const last = merged[merged.length - 1];
        if (last && s.t - last.t < 1) last.to = s.to, (last.jump = last.to - last.from);
        else merged.push({ ...s });
    }

    const driftPerMin = perSec === null ? null : perSec * 60;
    const total = ending - baseline;
    return {
        samples: usable.length,
        baselineDeg: baseline,
        endingDeg: ending,
        totalDriftDeg: total,
        driftPerMinDeg: driftPerMin,
        steps: merged,
        series: usable,
        // A steady offset is a zeroing problem; movement is a mechanical one.
        verdict:
            merged.length > 0
                ? "slipped"
                : Math.abs(total) >= 3
                  ? "drifting"
                  : Math.abs(baseline) >= 3
                    ? "zero-off"
                    : "ok",
    };
}

/**
 * Tracking: does the turret reach what it was told, and how long does it take?
 *
 * Counts windows where the error stays outside tolerance despite the controller pushing -- the
 * mechanism not following its command, as opposed to simply being mid-move.
 */
export function trackingAnalysis(model, { toleranceDeg = 2, minStuckSec = 0.5 } = {}) {
    const err = model.ch("Turret/PositionError");
    if (!err.length) return null;
    const unwraps = unwrapWindows(model);
    const volts = model.ch("Turret/Voltage");

    const usable = err.filter(([t]) => inWindows(t, model.enabled) && !inWindows(t, unwraps));
    if (!usable.length) return null;

    const abs = usable.map(([, v]) => Math.abs(v));
    const withinPct = (100 * abs.filter((v) => v <= toleranceDeg).length) / abs.length;

    // "Stuck" = outside tolerance, with the controller applying voltage, for a sustained stretch.
    const stuck = [];
    let open = null;
    for (let i = 0; i < usable.length; i++) {
        const [t, e] = usable[i];
        const v = valueAt(volts, t);
        const bad = Math.abs(e) > toleranceDeg && v !== null && Math.abs(v) > 0.5;
        if (bad && open === null) open = t;
        else if (!bad && open !== null) {
            if (t - open >= minStuckSec) stuck.push([open, t]);
            open = null;
        }
    }
    if (open !== null && model.log.lastTs - open >= minStuckSec) stuck.push([open, model.log.lastTs]);

    return {
        samples: usable.length,
        withinTolerancePct: withinPct,
        toleranceDeg,
        medianAbsErrDeg: median(abs),
        maxAbsErrDeg: Math.max(...abs),
        stuckWindows: stuck,
        stuckSec: stuck.reduce((a, [s, e]) => a + (e - s), 0),
    };
}

/**
 * Position jumps the commanded velocity cannot account for.
 *
 * A belt skipping a tooth or an encoder losing count shows up as position moving further in one
 * loop than any real slew could. Unwrap windows are excluded, and so is the wrap seam itself.
 */
export function motionAnomalies(model, { maxDegPerSec = 400, minJumpDeg = 4 } = {}) {
    const pos = model.ch("Turret/PositionDegrees");
    if (pos.length < 10) return [];
    const unwraps = unwrapWindows(model);

    const out = [];
    for (let i = 1; i < pos.length; i++) {
        const [t0, p0] = pos[i - 1];
        const [t1, p1] = pos[i];
        const dt = t1 - t0;
        if (dt <= 0 || dt > 0.25) continue;
        if (inWindows(t1, unwraps) || inWindows(t0, unwraps)) continue;

        const dp = p1 - p0;
        if (Math.abs(dp) < minJumpDeg) continue;
        const rate = Math.abs(dp) / dt;
        if (rate <= maxDegPerSec) continue;

        out.push({ t: t1, from: p0, to: p1, deltaDeg: dp, degPerSec: rate });
    }
    return out.sort((a, b) => Math.abs(b.deltaDeg) - Math.abs(a.deltaDeg)).slice(0, 20);
}

/**
 * Effort with no motion, and motion with no effort.
 *
 * The first is a stall -- the motor pushing against something that will not move, which is how a
 * belt gets chewed. The second means something other than the motor moved the turret.
 */
export function effortAnomalies(model, { stallAmps = 20, stillDegPerSec = 2, minSec = 0.3 } = {}) {
    const pos = model.ch("Turret/PositionDegrees");
    const cur = model.ch("Turret/StatorCurrent");
    if (!pos.length || !cur.length) return { stalls: [], backdriven: [] };
    const unwraps = unwrapWindows(model);

    const rateAt = (i) => {
        if (i === 0) return 0;
        const dt = pos[i][0] - pos[i - 1][0];
        return dt > 0 ? Math.abs(pos[i][1] - pos[i - 1][1]) / dt : 0;
    };

    const collect = (predicate) => {
        const out = [];
        let open = null;
        for (let i = 1; i < pos.length; i++) {
            const t = pos[i][0];
            if (inWindows(t, unwraps) || !inWindows(t, model.enabled)) {
                if (open !== null && t - open >= minSec) out.push([open, t]);
                open = null;
                continue;
            }
            const hit = predicate(rateAt(i), Math.abs(valueAt(cur, t) ?? 0));
            if (hit && open === null) open = t;
            else if (!hit && open !== null) {
                if (t - open >= minSec) out.push([open, t]);
                open = null;
            }
        }
        return out;
    };

    return {
        stalls: collect((rate, amps) => amps >= stallAmps && rate < stillDegPerSec),
        backdriven: collect((rate, amps) => amps < 2 && rate > 20),
    };
}

/**
 * Any Turret/* channel this page does not already know about.
 *
 * New checks land in the robot code faster than in this app, so rather than ignore them until
 * someone updates the page, they get rendered generically the moment they appear in a log.
 */
const KNOWN_TURRET_KEYS = new Set([
    "WantedState", "SystemState", "CurrentCommand", "Voltage", "StatorCurrent", "SupplyCurrent",
    "Temp", "MotorConnected", "CommandedDegrees", "PositionDegrees", "PositionError",
    "CommandedRotPerSec", "Unwrapping", "ReadyToShoot", "TrackingErrorDegrees",
]);

export function extraTurretChannels(model) {
    return model.log
        .matching(/^\/Robot\/Turret\/(.+)$/)
        .filter(({ capture }) => !KNOWN_TURRET_KEYS.has(capture))
        .map(({ capture, values, name }) => ({
            key: capture,
            name,
            values,
            type: model.log.types.get(name),
            unit: (() => {
                try {
                    return JSON.parse(model.log.metadata.get(name) || "{}").unit || null;
                } catch {
                    return null;
                }
            })(),
        }));
}

/** Windows where a boolean channel was true. */
export function trueWindows(series, lastTs) {
    const out = [];
    let open = null;
    for (const [t, v] of series) {
        if (v && open === null) open = t;
        else if (!v && open !== null) {
            out.push([open, t]);
            open = null;
        }
    }
    if (open !== null) out.push([open, lastTs]);
    return out;
}

/**
 * For each slip candidate: how big it was, how much the closed loop clawed back, and how much aim
 * error was left over.
 *
 * There are two slip modes and they behave completely differently, which is the thing worth
 * understanding:
 *
 *   Encoder loses counts, mechanism did not move. PositionDegrees jumps, so the controller sees a
 *     sudden error and drives it out. Visible as a spike in PositionError that decays. The robot
 *     DOES correct this, and the recovery is measurable.
 *
 *   Mechanism moves, encoder did not. PositionDegrees is unchanged, so the controller sees nothing
 *     wrong and does nothing. The turret is now aiming wrong and stays wrong. Only the camera
 *     notices. The robot CANNOT correct this -- it does not know.
 *
 * So "how much did the robot correct" is answered from PositionError, and "how much is still
 * wrong" from the camera's zero error. They are different numbers and both matter.
 */
export function slipCorrection(model, zero, jumps, { windowSec = 4, toleranceDeg = 2 } = {}) {
    const err = model.ch("Turret/PositionError");
    const pos = model.ch("Turret/PositionDegrees");
    const heading = zero?.series ?? [];

    const between = (series, a, b) => series.filter(([t]) => t >= a && t <= b).map(([, v]) => v);
    const med = (arr) => (arr.length ? [...arr].sort((x, y) => x - y)[arr.length >> 1] : null);

    const candidates = [
        ...(zero?.steps || []).map((s) => ({ t: s.t, kind: "zero step", magnitudeDeg: s.jump })),
        ...jumps.map((j) => ({ t: j.t, kind: "position jump", magnitudeDeg: j.deltaDeg })),
    ].sort((a, b) => a.t - b.t);

    const events = candidates.map((c) => {
        const end = Math.min(c.t + windowSec, model.log.lastTs);

        // What the controller did about it.
        const after = between(err, c.t, end).map(Math.abs);
        const errPeak = after.length ? Math.max(...after) : null;
        const errEnd = med(between(err, Math.max(c.t, end - 0.5), end).map(Math.abs));
        const recoveredDeg = errPeak !== null && errEnd !== null ? Math.max(0, errPeak - errEnd) : null;

        // How long until it was back inside the readiness tolerance.
        let recoverySec = null;
        for (const [t, v] of err) {
            if (t <= c.t) continue;
            if (t > end) break;
            if (Math.abs(v) <= toleranceDeg) {
                recoverySec = t - c.t;
                break;
            }
        }

        // What the camera says is still wrong afterwards -- the part nothing corrected.
        const before = med(between(heading, c.t - 2, c.t - 0.2));
        const settled = med(between(heading, Math.max(c.t, end - 2), end));
        const residualDeg = before !== null && settled !== null ? settled - before : null;

        const corrected = recoveredDeg !== null && errPeak > toleranceDeg && recoverySec !== null;
        const leftWrong = residualDeg !== null && Math.abs(residualDeg) >= 1;

        return {
            ...c,
            errPeakDeg: errPeak,
            errEndDeg: errEnd,
            recoveredDeg,
            recoverySec,
            residualDeg,
            travelDeg: (() => {
                const p = between(pos, c.t, end);
                return p.length ? Math.max(...p) - Math.min(...p) : null;
            })(),
            mode: leftWrong && !corrected
                ? "uncorrectable"
                : corrected && leftWrong
                  ? "partly-corrected"
                  : corrected
                    ? "corrected"
                    : "unclear",
        };
    });

    const sum = (f) => events.reduce((a, e) => a + (f(e) ?? 0), 0);
    return {
        events,
        totalCorrectedDeg: sum((e) => e.recoveredDeg),
        totalResidualDeg: sum((e) => Math.abs(e.residualDeg ?? 0)),
        // The bottom line for aim: where the zero ended up versus where it started.
        netZeroShiftDeg: zero ? zero.endingDeg - zero.baselineDeg : null,
    };
}

/**
 * What the Limelights did about it.
 *
 * Vision does not correct the turret's zero -- nothing does, the encoder is the only thing the
 * turret controller trusts. What vision corrects is the robot POSE, and the commanded turret angle
 * is computed from that pose. So every accepted estimate nudges where the turret is told to point,
 * and the size of that nudge is the Limelight's correction measured in turret degrees.
 *
 * There are three kinds, and they are logged separately:
 *
 *   Vision/<cam>/IntegratedThisLoop  a normal estimate folded into the pose estimator
 *   Vision/HeadingCorrection/Applied the gross-heading safety net firing after a bad seed
 *   Vision/PoseReset/Before|After    someone pressing LB+Select to re-seed from the camera
 *
 * The sting in the tail: the turret camera's transform is computed from the turret angle, so a
 * turret that has slipped feeds the camera a wrong transform, its estimates disagree with the
 * gyro, and it gets rejected. A slip does not just spoil the aim -- it costs you the camera that
 * would have revealed it.
 */
export function limelightCorrections(model, { cameras = ["TurretLL", "BackLeftLL", "BackRightLL"] } = {}) {
    const commanded = model.ch("Turret/CommandedDegrees");
    const shotAngle = model.ch("ShotCalc/TurretAngleDeg");
    const aim = shotAngle.length ? shotAngle : commanded;

    // How much the aim command moved across an instant. Vision arrives between loops, so compare
    // just before and just after.
    const nudgeAt = (t, halfWindow = 0.06) => {
        const before = valueAt(aim, t - halfWindow);
        const after = valueAt(aim, t + halfWindow);
        return before === null || after === null ? null : after - before;
    };

    const perCamera = cameras
        .map((cam) => {
            const integrated = model.ch(`Vision/${cam}/IntegratedThisLoop`);
            if (!integrated.length) return null;

            // Rising edges only: the key stays true across consecutive loops.
            const events = [];
            let prev = false;
            for (const [t, v] of integrated) {
                if (v && !prev) {
                    const nudge = nudgeAt(t);
                    if (nudge !== null) events.push({ t, nudgeDeg: nudge });
                }
                prev = v;
            }

            const accepted = integrated.filter(([, v]) => v).length;
            const nudges = events.map((e) => Math.abs(e.nudgeDeg)).filter((v) => v > 0.01);
            nudges.sort((a, b) => a - b);

            return {
                camera: cam,
                acceptedSamples: accepted,
                totalSamples: integrated.length,
                acceptRate: integrated.length ? accepted / integrated.length : 0,
                corrections: events.length,
                totalCorrectionDeg: nudges.reduce((a, v) => a + v, 0),
                medianCorrectionDeg: nudges.length ? nudges[nudges.length >> 1] : null,
                maxCorrectionDeg: nudges.length ? nudges[nudges.length - 1] : null,
                events,
            };
        })
        .filter(Boolean);

    // The gross-heading safety net.
    const applied = model.ch("Vision/HeadingCorrection/Applied");
    const errDeg = model.ch("Vision/HeadingCorrection/ErrorDeg");
    const headingFixes = [];
    let wasApplied = false;
    for (const [t, v] of applied) {
        if (v && !wasApplied) headingFixes.push({ t, errorDeg: valueAt(errDeg, t), nudgeDeg: nudgeAt(t, 0.15) });
        wasApplied = v;
    }

    // Manual re-seeds from LB+Select.
    const before = model.ch("Vision/PoseReset/Before");
    const afterPose = model.ch("Vision/PoseReset/After");
    const resets = afterPose.map(([t, a], i) => {
        const b = before[i]?.[1];
        return {
            t,
            headingDeltaDeg: b && a ? a.deg - b.deg : null,
            nudgeDeg: nudgeAt(t, 0.2),
        };
    });

    return {
        perCamera,
        headingFixes,
        resets,
        totalCorrectionDeg: perCamera.reduce((a, c) => a + c.totalCorrectionDeg, 0),
    };
}

/**
 * Windows where the turret camera was disagreeing with the gyro badly enough to be distrusted.
 *
 * This is the cost of a slip: past the rejection threshold the turret Limelight stops contributing,
 * so the robot loses its best look at the hub exactly when its aim is worst.
 */
export function turretCameraRejection(model, { thresholdDeg = 5, minSec = 0.5 } = {}) {
    const err = model.ch("Vision/TurretLL/HeadingErrorDeg");
    if (!err.length) return { windows: [], totalSec: 0, thresholdDeg };
    const windows = [];
    let open = null;
    for (const [t, v] of err) {
        const bad = Math.abs(v) > thresholdDeg;
        if (bad && open === null) open = t;
        else if (!bad && open !== null) {
            if (t - open >= minSec) windows.push([open, t]);
            open = null;
        }
    }
    if (open !== null && model.log.lastTs - open >= minSec) windows.push([open, model.log.lastTs]);
    return { windows, totalSec: windows.reduce((a, [s, e]) => a + (e - s), 0), thresholdDeg };
}
