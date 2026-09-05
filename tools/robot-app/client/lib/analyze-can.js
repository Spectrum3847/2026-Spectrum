/**
 * CAN-health analyses that go beyond reading a channel out of the log.
 *
 * The motivating case is 2026-09-04: the hood was commanded to 18-20 degrees in eleven launch
 * windows and reported exactly 0 V and 0 A in nine of them, while turret and flywheel ran fine.
 * Only three motors log MotorConnected, so for everything else that failure has to be inferred.
 */
import { inWindows } from "./log-model.js";

/** Sample a step-series (change-only logged) at time t. */
function valueAt(series, t) {
    if (!series.length) return null;
    let lo = 0;
    let hi = series.length - 1;
    if (t < series[0][0]) return null;
    while (lo < hi) {
        const mid = (lo + hi + 1) >> 1;
        if (series[mid][0] <= t) lo = mid;
        else hi = mid - 1;
    }
    return series[lo][1];
}

/** Merge [start, end] windows that are closer together than `gap`. */
function coalesce(windows, gap = 0.25) {
    if (!windows.length) return [];
    const out = [windows[0].slice()];
    for (const w of windows.slice(1)) {
        const last = out[out.length - 1];
        if (w[0] - last[1] <= gap) last[1] = Math.max(last[1], w[1]);
        else out.push(w.slice());
    }
    return out;
}

/**
 * Windows where MotorConnected went false. This is the direct signal, but it exists only for
 * Turret, Launcher and Hood, and it only checks the leader -- a dead follower is invisible.
 */
export function connectionDropouts(model) {
    const out = [];
    for (const mech of model.mechanisms) {
        if (!mech.motorConnected.length) continue;
        const windows = [];
        let open = null;
        for (const [t, v] of mech.motorConnected) {
            if (v === false && open === null) open = t;
            else if (v === true && open !== null) {
                windows.push([open, t]);
                open = null;
            }
        }
        if (open !== null) windows.push([open, model.log.lastTs]);
        if (windows.length) {
            out.push({
                motor: mech.displayName,
                canId: mech.spec?.canId ?? null,
                windows: coalesce(windows),
                totalSec: windows.reduce((a, [s, e]) => a + (e - s), 0),
            });
        }
    }
    return out;
}

/**
 * Motors reporting zero output while something was asking them to move.
 *
 * A controller that is commanded to a position and reports exactly 0 V and 0 A is not a motor
 * that decided to rest -- it is a motor that is not receiving or executing control frames. This
 * needs a commanded-vs-measured pair, so it covers Hood, Turret and Launcher on current logs.
 */
export function zeroOutputWhileCommanded(model, { minSec = 0.25, tolerance = { degrees: 1.5, rpm: 150 } } = {}) {
    const pairs = [
        { cmd: "CommandedDegrees", actual: "PositionDegrees", tol: tolerance.degrees, unit: "deg" },
        { cmd: "CommandedRPM", actual: "RPM", tol: tolerance.rpm, unit: "RPM" },
    ];
    const findings = [];

    for (const mech of model.mechanisms) {
        const volts = mech.voltage;
        if (!volts.length) continue;

        for (const { cmd, actual, tol, unit } of pairs) {
            const cmdSeries = model.ch(`${mech.name}/${cmd}`);
            const actSeries = model.ch(`${mech.name}/${actual}`);
            if (!cmdSeries.length || !actSeries.length) continue;

            const windows = [];
            let open = null;
            for (let i = 0; i < volts.length; i++) {
                const [t, v] = volts[i];
                const next = volts[i + 1]?.[0] ?? t + 0.02;
                const enabled = inWindows(t, model.enabled);
                const c = valueAt(cmdSeries, t);
                const a = valueAt(actSeries, t);
                const stator = valueAt(mech.stator, t);

                const dead =
                    enabled &&
                    c !== null &&
                    a !== null &&
                    Math.abs(v) < 0.05 &&
                    (stator === null || Math.abs(stator) < 0.5) &&
                    Math.abs(c - a) > tol;

                if (dead && open === null) open = t;
                else if (!dead && open !== null) {
                    if (next - open >= minSec) windows.push([open, t]);
                    open = null;
                }
            }
            if (open !== null) windows.push([open, model.log.lastTs]);

            const merged = coalesce(windows).filter(([s, e]) => e - s >= minSec);
            if (merged.length) {
                findings.push({
                    motor: mech.displayName,
                    canId: mech.spec?.canId ?? null,
                    bus: mech.spec?.bus ?? null,
                    signal: `${cmd} vs ${actual}`,
                    unit,
                    windows: merged,
                    totalSec: merged.reduce((a, [s, e]) => a + (e - s), 0),
                    worstErr: Math.max(
                        ...merged.map(([s]) => Math.abs((valueAt(cmdSeries, s) ?? 0) - (valueAt(actSeries, s) ?? 0)))
                    ),
                });
            }
        }
    }
    return findings;
}

/**
 * Gaps in a channel's record stream while the robot was enabled.
 *
 * DogLog only writes a record when a value CHANGES, so a controller that has stopped answering
 * produces silence rather than a flat line. That makes a long gap in an otherwise chatty channel
 * a genuine staleness signal -- weaker than the two detectors above, because an idle mechanism is
 * also silent, so this is advisory only.
 */
export function staleTraces(model, { minGapSec = 1.0, multiple = 25 } = {}) {
    const out = [];
    for (const mech of model.mechanisms) {
        const series = mech.stator.length ? mech.stator : mech.supply;
        if (series.length < 50) continue;

        const enabled = series.filter(([t]) => inWindows(t, model.enabled));
        if (enabled.length < 50) continue;

        const deltas = [];
        for (let i = 1; i < enabled.length; i++) deltas.push(enabled[i][0] - enabled[i - 1][0]);
        deltas.sort((a, b) => a - b);
        const median = deltas[deltas.length >> 1];
        const threshold = Math.max(minGapSec, median * multiple);

        const gaps = [];
        for (let i = 1; i < enabled.length; i++) {
            const gap = enabled[i][0] - enabled[i - 1][0];
            if (gap >= threshold) gaps.push([enabled[i - 1][0], enabled[i][0], gap]);
        }
        if (gaps.length) {
            out.push({
                motor: mech.displayName,
                canId: mech.spec?.canId ?? null,
                medianIntervalMs: median * 1000,
                thresholdSec: threshold,
                gaps: gaps.sort((a, b) => b[2] - a[2]).slice(0, 5),
                count: gaps.length,
            });
        }
    }
    return out.sort((a, b) => b.gaps[0][2] - a.gaps[0][2]);
}

/** Parse the deduplicated Alerts string channel into discrete events. */
export function alertEvents(model) {
    return model.ch("Alerts").map(([t, text]) => {
        const m = /^(ERROR|WARNING|INFO):\s*(.*)$/.exec(String(text));
        return { t, level: m ? m[1].toLowerCase() : "info", text: m ? m[2] : String(text) };
    });
}

/** Step events on a monotonic counter, e.g. BusOffCount going 0 -> 1. */
export function counterSteps(series) {
    const steps = [];
    for (let i = 1; i < series.length; i++) {
        if (series[i][1] > series[i - 1][1]) steps.push({ t: series[i][0], from: series[i - 1][1], to: series[i][1] });
    }
    return steps;
}
