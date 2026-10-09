/**
 * Main-breaker trip curve and thermal simulation.
 *
 * Ported from the team's WPI-Log-Analizer. The curve is the Bussmann CB185-120 (Series 18X)
 * maximum time-to-trip, i.e. the slow end of the manufacturing tolerance band -- so the
 * simulation is optimistic about survival, which is the safe direction for "would this have
 * popped?" questions.
 */

export const CB185_120 = [
    [120, 1800],
    [144, 1200],
    [162, 600],
    [180, 300],
    [240, 100],
    [300, 50],
    [360, 30],
    [480, 18],
    [600, 10],
    [720, 6],
    [877, 4],
];

/** Time-to-trip in seconds at current I. Infinity when within rating. */
export function tripTime(curve, I) {
    if (I <= curve[0][0]) return Infinity;
    if (I >= curve[curve.length - 1][0]) return curve[curve.length - 1][1];
    for (let i = 1; i < curve.length; i++) {
        if (I <= curve[i][0]) {
            const [a0, t0] = curve[i - 1];
            const [a1, t1] = curve[i];
            return t0 + ((I - a0) / (a1 - a0)) * (t1 - t0);
        }
    }
    return 0;
}

/**
 * I²t accumulator calibrated against the trip curve, with an ambient-temperature correction
 * (breakers are rated at 25 C and gain roughly 1% capacity per degree below that).
 *
 * @param {Array<[number, number]>} samples  [tSec, amps]
 * @returns {{thermal: Array<[number, number]>, trips: number[], peakPct: number}}
 *          thermal is percent-of-trip over time; trips are the times it would have popped.
 */
export function simulateBreaker(samples, { curve = CB185_120, ratedAmps = 120, ambientC = 21 } = {}) {
    let heat = 0;
    let peak = 0;
    const thermal = [];
    const trips = [];

    const tempFactor = 1 + (25 - ambientC) * 0.01;
    const effectiveRated = ratedAmps * tempFactor;

    for (let i = 1; i < samples.length; i++) {
        const [t0] = samples[i - 1];
        const [t1, I] = samples[i];
        const dt = t1 - t0;
        // Skip the gaps: a log can pause for minutes between enables and that is not cooling time
        // we can reason about.
        if (dt <= 0 || dt > 1) continue;

        const amps = Math.abs(I);
        if (amps > effectiveRated) {
            const tt = tripTime(curve, amps / tempFactor);
            if (tt > 0 && tt !== Infinity) heat += dt / tt;
        } else {
            heat = Math.max(0, heat - dt / 200); // ~3.3 min cooling constant
        }

        if (heat >= 1) {
            trips.push(+t1.toFixed(3));
            heat = 0.05; // small residual after a trip
        }
        const pct = Math.min(heat * 100, 100);
        if (pct > peak) peak = pct;
        thermal.push([t1, pct]);
    }
    return { thermal, trips, peakPct: peak };
}
