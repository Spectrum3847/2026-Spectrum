import "../../styles.css";
import "../power/analysis.css";
import "./turret.css";
import { mountHeader, el, fmtDuration } from "../../lib/ui.js";
import { logPicker, loadProfile, LogModel } from "../../lib/log-loader.js";
import { clipToEnabled, seriesStats, inWindows } from "../../lib/log-model.js";
import { timeChart, xy, decimate, enabledShadePlugin, limitLinePlugin, seriesColor, theme, withAlpha } from "../../lib/charts.js";
import {
    zeroErrorAnalysis, trackingAnalysis, motionAnomalies, effortAnomalies,
    extraTurretChannels, unwrapWindows, trueWindows, valueAt,
    slipCorrection, limelightCorrections, turretCameraRejection,
} from "../../lib/analyze-turret.js";

mountHeader();
const root = document.getElementById("app");
const profile = await loadProfile();
const charts = [];

const content = el("div", {});
root.replaceChildren(
    el("h1", {}, "Turret"),
    el("p", { class: "lede" },
        "Where the turret pointed, whether it got where it was told, and whether it slipped. ",
        "The camera on the turret is an independent witness to where it really points, so a zero error that ",
        el("em", {}, "moves"), " during a session is mechanical, not a bad zero."),
    el("div", { class: "card", style: "margin-bottom:20px" }, logPicker({ onLoad: render })),
    content
);

const SVG = "http://www.w3.org/2000/svg";
const svgEl = (tag, attrs = {}) => {
    const n = document.createElementNS(SVG, tag);
    for (const [k, v] of Object.entries(attrs)) if (v !== null && v !== undefined) n.setAttribute(k, String(v));
    return n;
};

const tile = (label, value, note, cls) =>
    el("div", { class: "stat" },
        el("div", { class: "label" }, label),
        el("div", { class: `value ${cls || ""}` }, value),
        note ? el("div", { class: "note" }, note) : null);

function chartCard(title, note, height = "") {
    const canvas = el("canvas");
    const card = el("section", {},
        el("h2", {}, title),
        note ? el("div", { class: "section-note" }, note) : null,
        el("div", { class: "card" }, el("div", { class: `chart-box ${height}` }, canvas)));
    return { card, canvas };
}

const fmtDeg = (v) => (Number.isFinite(v) ? `${v.toFixed(1)}°` : "—");

/**
 * Top-down turret dial.
 *
 * 0 degrees is the turret's zero, which on this robot faces AWAY from the intake. Angles increase
 * counter-clockwise, matching the sign convention in the log, and the shaded arc is the actual
 * travel the soft limits allow -- the turret cannot spin freely, which is why it unwraps.
 */
function buildDial(limits) {
    const R = 130;
    const cx = 160;
    const cy = 160;
    const toXY = (deg, r) => {
        // Screen y grows downward, so negate to make positive angles read counter-clockwise.
        const rad = ((deg + 90) * Math.PI) / 180;
        return [cx + r * Math.cos(rad), cy - r * Math.sin(rad)];
    };

    const svg = svgEl("svg", { viewBox: "0 0 320 320", class: "dial", role: "img", "aria-label": "Turret angle dial" });

    // Chassis outline, with a marker for where the intake is (opposite the turret zero).
    svg.append(svgEl("rect", { class: "dial-chassis", x: cx - 62, y: cy - 62, width: 124, height: 124, rx: 10 }));
    const [ix, iy] = toXY(180, 62);
    svg.append(svgEl("circle", { class: "dial-intake", cx: ix, cy: iy, r: 5 }));

    // Travel arc between the soft limits.
    const [sx, sy] = toXY(limits.reverseDeg, R - 20);
    const [ex, ey] = toXY(limits.forwardDeg, R - 20);
    const sweep = limits.forwardDeg - limits.reverseDeg;
    svg.append(svgEl("path", {
        class: "dial-travel",
        d: `M ${sx} ${sy} A ${R - 20} ${R - 20} 0 ${sweep > 180 ? 1 : 0} 0 ${ex} ${ey}`,
    }));

    svg.append(svgEl("circle", { class: "dial-ring", cx, cy, r: R, "fill-opacity": "0" }));

    for (let deg = -180; deg < 180; deg += 15) {
        const major = deg % 45 === 0;
        const [x1, y1] = toXY(deg, R);
        const [x2, y2] = toXY(deg, R - (major ? 12 : 6));
        svg.append(svgEl("line", { class: `dial-tick${major ? " major" : ""}`, x1, y1, x2, y2 }));
        if (major) {
            const [lx, ly] = toXY(deg, R + 13);
            const t = svgEl("text", { class: "dial-label", x: lx, y: ly + 3, "text-anchor": "middle" });
            t.textContent = `${deg}°`;
            svg.append(t);
        }
    }

    for (const [deg, label] of [[limits.reverseDeg, "rev limit"], [limits.forwardDeg, "fwd limit"]]) {
        const [x1, y1] = toXY(deg, R - 34);
        const [x2, y2] = toXY(deg, R + 2);
        const line = svgEl("line", { class: "dial-limit", x1, y1, x2, y2 });
        // Node.append() returns undefined, so the title element has to be held on to.
        const tip = svgEl("title");
        tip.textContent = label;
        line.append(tip);
        svg.append(line);
    }

    const needle = (cls, len) => {
        const n = svgEl("line", { class: cls, x1: cx, y1: cy, x2: cx, y2: cy - len });
        svg.append(n);
        return { node: n, len };
    };
    const commanded = needle("needle-commanded", R - 30);
    const vision = needle("needle-vision", R - 52);
    const measured = needle("needle-measured", R - 34);
    svg.append(svgEl("circle", { class: "dial-hub", cx, cy, r: 7 }));

    const set = (handle, deg) => {
        if (!Number.isFinite(deg)) {
            handle.node.setAttribute("visibility", "hidden");
            return;
        }
        handle.node.setAttribute("visibility", "visible");
        const [x, y] = toXY(deg, handle.len);
        handle.node.setAttribute("x2", x);
        handle.node.setAttribute("y2", y);
    };

    return { svg, set: (m, c, v) => { set(measured, m); set(commanded, c); set(vision, v); } };
}

function render(log) {
    for (const c of charts.splice(0)) c.destroy();
    const m = new LogModel(log, profile);
    const shade = enabledShadePlugin(m.enabled, log.firstTs, log.lastTs);
    const t = theme();
    const blocks = [];

    const pos = m.ch("Turret/PositionDegrees");
    const cmd = m.ch("Turret/CommandedDegrees");
    const err = m.ch("Turret/PositionError");
    const track = m.ch("Turret/TrackingErrorDegrees");
    const cur = m.ch("Turret/StatorCurrent");
    const volts = m.ch("Turret/Voltage");
    const ready = m.ch("Turret/ReadyToShoot");
    const headingErr = m.ch("Vision/TurretLL/HeadingErrorDeg");

    if (!pos.length) {
        content.replaceChildren(el("div", { class: "notice warn" },
            el("strong", {}, "No turret data in this log. "),
            "Turret/PositionDegrees is missing — this is probably a log from the competition robot, which had no turret."));
        return;
    }

    const limits = profile?.turret ?? { reverseDeg: -180, forwardDeg: 162, toleranceDeg: 2 };
    const zero = zeroErrorAnalysis(m);
    const tracking = trackingAnalysis(m, { toleranceDeg: limits.toleranceDeg });
    const jumps = motionAnomalies(m);
    const effort = effortAnomalies(m);
    const unwraps = unwrapWindows(m);
    const extras = extraTurretChannels(m);
    const correction = slipCorrection(m, zero, jumps, { toleranceDeg: limits.toleranceDeg });
    const limelight = limelightCorrections(m);
    const rejection = turretCameraRejection(m);

    // ------------------------------------------------------------ verdict
    const VERDICTS = {
        slipped: { cls: "bad", icon: "✖", title: "The turret slipped",
            body: (z) => `The camera's zero error stepped ${z.steps.length} time${z.steps.length === 1 ? "" : "s"} and stayed moved — the largest by ${fmtDeg(Math.max(...z.steps.map((s) => Math.abs(s.jump))))}. A step that persists means the turret moved relative to its encoder. Re-zeroing will not fix this; check the belt and the encoder coupling.` },
        drifting: { cls: "bad", icon: "▲", title: "The zero drifted during this log",
            body: (z) => `Zero error went from ${fmtDeg(z.baselineDeg)} to ${fmtDeg(z.endingDeg)} — ${fmtDeg(z.totalDriftDeg)} over the session, about ${fmtDeg(z.driftPerMinDeg)} per minute. Gradual walk like this is a belt creeping rather than one clean skip.` },
        "zero-off": { cls: "warn", icon: "◑", title: "The zero is off, but steady",
            body: (z) => `Zero error sat at ${fmtDeg(z.baselineDeg)} and stayed there. Nothing is slipping — the turret was simply zeroed ${fmtDeg(Math.abs(z.baselineDeg))} away from true. Point it at its zero by hand and press operator B while disabled.` },
        ok: { cls: "ok", icon: "✔", title: "No slip detected",
            body: (z) => `Zero error stayed near ${fmtDeg(z.baselineDeg)} for the whole log with no persistent steps. The encoder and the camera agree about where the turret points.` },
    };

    if (zero) {
        const v = VERDICTS[zero.verdict];
        blocks.push(el("div", { class: `verdict ${v.cls}`, style: "margin-bottom:18px" },
            el("div", { class: "icon" }, v.icon),
            el("div", {}, el("h3", {}, v.title), el("p", {}, v.body(zero)))));
    } else {
        blocks.push(el("div", { class: "notice warn" },
            el("strong", {}, "No slip verdict. "),
            "Vision/TurretLL/HeadingErrorDeg has too few usable samples in this log — the turret camera needs to see tags for the independent cross-check. Everything below still works from the encoder alone."));
    }

    // ------------------------------------------------------------ tiles
    // Exclude unwraps: a full-turn slew is real motion but it is not travel the turret "used",
    // and including it reports a range beyond the soft limits, which reads as nonsense.
    const posNoUnwrap = clipToEnabled(pos, m.enabled).filter(([t]) => !inWindows(t, unwraps));
    const posStats = seriesStats(posNoUnwrap.length ? posNoUnwrap : clipToEnabled(pos, m.enabled));
    blocks.push(el("div", { class: "grid cols-4", style: "margin-bottom:6px" },
        tile("Zero error", zero ? fmtDeg(zero.baselineDeg) : "—", "at the start of the log",
            zero ? (Math.abs(zero.baselineDeg) > 5 ? "bad" : Math.abs(zero.baselineDeg) > 2 ? "warn" : "ok") : ""),
        tile("Drift", zero?.totalDriftDeg !== undefined ? fmtDeg(zero.totalDriftDeg) : "—",
            zero?.driftPerMinDeg != null ? `${fmtDeg(zero.driftPerMinDeg)}/min` : null,
            zero ? (Math.abs(zero.totalDriftDeg) > 3 ? "bad" : "ok") : ""),
        tile("On target", tracking ? `${tracking.withinTolerancePct.toFixed(0)}%` : "—",
            tracking ? `within ${tracking.toleranceDeg}° while enabled` : null,
            tracking ? (tracking.withinTolerancePct < 70 ? "warn" : "ok") : ""),
        tile("Not following", tracking ? fmtDuration(tracking.stuckSec) : "—", "error held with voltage applied",
            tracking ? (tracking.stuckSec > 2 ? "bad" : "ok") : ""),
        tile("Position jumps", jumps.length, "faster than physically plausible", jumps.length ? "bad" : "ok"),
        tile("Stalls", effort.stalls.length, "current, no motion", effort.stalls.length ? "warn" : "ok"),
        tile("Unwraps", unwraps.length, "full-turn slews", unwraps.length ? "warn" : "ok"),
        tile("Travel used", posStats ? `${fmtDeg(posStats.min)} … ${fmtDeg(posStats.max)}` : "—",
            `limits ${limits.reverseDeg}° … ${limits.forwardDeg}°`)));

    blocks.push(el("div", { class: "grid cols-4", style: "margin:14px 0 6px" },
        tile("Net zero shift", correction.netZeroShiftDeg != null ? fmtDeg(correction.netZeroShiftDeg) : "—",
            "start of log to end", correction.netZeroShiftDeg != null && Math.abs(correction.netZeroShiftDeg) > 3 ? "bad" : "ok"),
        tile("Controller clawed back", correction.events.length ? fmtDeg(correction.totalCorrectedDeg) : "—",
            "tracking error it drove out"),
        tile("Left uncorrected", correction.events.length ? fmtDeg(correction.totalResidualDeg) : "—",
            "aim error nothing fixed", correction.totalResidualDeg > 2 ? "bad" : "ok"),
        tile("Limelight aim nudges", limelight.perCamera.reduce((a, c) => a + c.corrections, 0) || "—",
            limelight.totalCorrectionDeg ? `${fmtDeg(limelight.totalCorrectionDeg)} total` : "no pose estimates integrated")));

    // ------------------------------------------------------------ dial + scrub
    const dial = buildDial(limits);
    const readout = el("dl", { class: "readout" });
    const slider = el("input", { type: "range", min: 0, max: 1000, value: 0 });
    const timeLabel = el("span", { class: "t" });

    const paintAt = (tSec) => {
        const p = valueAt(pos, tSec);
        const c = valueAt(cmd, tSec);
        const he = valueAt(headingErr, tSec);
        // The camera sees the turret's true heading; the encoder's idea plus the disagreement is
        // where it actually points.
        const trueDeg = p !== null && he !== null ? p + he : null;
        dial.set(p, c, trueDeg);
        timeLabel.textContent = `${tSec.toFixed(2)} s`;
        const rows = [
            ["swatch-measured", "Encoder", fmtDeg(p)],
            ["swatch-commanded", "Commanded", fmtDeg(c)],
            ["swatch-vision", "Camera says", trueDeg === null ? "no tags" : fmtDeg(trueDeg)],
            [null, "Error", fmtDeg(valueAt(err, tSec))],
            [null, "Current", (() => { const a = valueAt(cur, tSec); return a === null ? "—" : `${a.toFixed(0)} A`; })()],
            [null, "State", valueAt(m.ch("Turret/SystemState"), tSec) ?? "—"],
            [null, "Ready", valueAt(ready, tSec) === true ? "yes" : "no"],
            [null, "Unwrapping", inWindows(tSec, unwraps) ? "YES" : "no"],
        ];
        readout.replaceChildren(...rows.flatMap(([sw, k, v]) => [
            el("dt", {}, sw ? el("span", {
                class: "swatch",
                style: `background:${sw === "swatch-measured" ? seriesColor(0) : sw === "swatch-commanded" ? seriesColor(3) : seriesColor(2)}`,
            }) : null, k),
            el("dd", {}, v),
        ]));
    };

    const t0 = log.firstTs;
    const span = Math.max(0.001, log.lastTs - log.firstTs);
    slider.addEventListener("input", () => paintAt(t0 + (slider.value / 1000) * span));

    // Open on the most interesting moment rather than t=0, which is always the robot sitting still.
    const focus = zero?.steps[0]?.t ?? jumps[0]?.t ?? tracking?.stuckWindows[0]?.[0] ?? m.enabled[0]?.[0] ?? t0;
    slider.value = Math.round(((focus - t0) / span) * 1000);

    blocks.push(el("section", {},
        el("h2", {}, "Where it pointed"),
        el("div", { class: "section-note" },
            "0° is the turret's zero, facing away from the intake; the small dot marks the intake side. ",
            "Drag to scrub through the log. It opens on the first thing worth looking at."),
        el("div", { class: "turret-top" },
            el("div", { class: "card" },
                dial.svg,
                el("div", { class: "scrub" }, slider, timeLabel)),
            el("div", { class: "card" }, el("h3", { style: "margin-top:0" }, "At this moment"), readout))));
    queueMicrotask(() => paintAt(focus));

    // ------------------------------------------------------------ zero error over time
    if (zero) {
        const { card, canvas } = chartCard("Zero error over time",
            "The turret camera's disagreement with the gyro. Flat means the encoder and reality agree. A slope means creep; a step that stays means a slip.",
            "short");
        blocks.push(card);
        if (zero.steps.length) {
            card.append(el("div", { class: "notice bad" },
                el("strong", {}, "Persistent steps: "),
                zero.steps.map((s) => `${s.t.toFixed(1)}s (${s.jump > 0 ? "+" : ""}${s.jump.toFixed(1)}°)`).join(", ")));
        }
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "degrees",
            yBeginAtZero: false,
            datasets: [{ label: "zero error", data: xy(decimate(zero.series)), borderColor: seriesColor(2), fill: false }],
            plugins: [shade, limitLinePlugin([{ value: zero.baselineDeg, color: t.textFaint, label: "start", dash: [3, 4] }])],
        })));
    }

    // ------------------------------------------------------------ commanded vs measured
    {
        const { card, canvas } = chartCard("Commanded vs measured angle",
            "The two should sit on top of each other except during a slew. Grey bands are unwraps — a legitimate full turn, not a fault.", "tall");
        blocks.push(card);
        queueMicrotask(() => {
            const unwrapShade = {
                id: "unwrapShade",
                beforeDatasetsDraw(chart) {
                    const { ctx, chartArea, scales } = chart;
                    if (!chartArea) return;
                    ctx.save();
                    ctx.fillStyle = withAlpha(theme().warn, 0.16);
                    for (const [s, e] of unwraps) {
                        const x0 = scales.x.getPixelForValue(s);
                        const x1 = scales.x.getPixelForValue(e);
                        if (x1 > x0) ctx.fillRect(x0, chartArea.top, Math.max(x1 - x0, 2), chartArea.bottom - chartArea.top);
                    }
                    ctx.restore();
                },
            };
            charts.push(timeChart(canvas, {
                yTitle: "degrees",
                yBeginAtZero: false,
                datasets: [
                    { label: "measured", data: xy(decimate(pos)), borderColor: seriesColor(0), fill: false },
                    { label: "commanded", data: xy(decimate(cmd)), borderColor: seriesColor(3), fill: false, borderDash: [6, 4] },
                ],
                plugins: [shade, unwrapShade, limitLinePlugin([
                    { value: limits.reverseDeg, label: `${limits.reverseDeg}° limit` },
                    { value: limits.forwardDeg, label: `${limits.forwardDeg}° limit` },
                ])],
            }));
        });
    }

    // ------------------------------------------------------------ error + current
    // Unwrap samples are dropped from this chart rather than shaded: a 360 degree slew sets the
    // y-scale so wide that the tolerance band this chart exists to show becomes invisible.
    const errNoUnwrap = err.filter(([t]) => !inWindows(t, unwraps));
    const trackNoUnwrap = track.filter(([t]) => !inWindows(t, unwraps));
    if (errNoUnwrap.length) {
        const { card, canvas } = chartCard("Tracking error",
            `Positive means the turret is behind its command. The band is the ±${limits.toleranceDeg}° readiness tolerance that gates the feeder. Unwrap slews are left out so the band stays readable.`,
            "short");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "degrees",
            yBeginAtZero: false,
            datasets: [
                { label: "position error", data: xy(decimate(errNoUnwrap)), borderColor: seriesColor(7), fill: false },
                trackNoUnwrap.length && { label: "tracking error", data: xy(decimate(trackNoUnwrap)), borderColor: seriesColor(4), fill: false },
            ].filter(Boolean),
            plugins: [shade, limitLinePlugin([
                { value: limits.toleranceDeg, color: t.warn, label: `+${limits.toleranceDeg}°`, dash: [3, 4] },
                { value: -limits.toleranceDeg, color: t.warn, label: `-${limits.toleranceDeg}°`, dash: [3, 4] },
            ])],
        })));
    }

    if (cur.length) {
        const { card, canvas } = chartCard("Effort",
            "Stator current and applied voltage. Current with no motion is a stall; motion with no current means something else moved it.",
            "short");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "amps",
            yBeginAtZero: false,
            datasets: [{ label: "stator current", data: xy(decimate(cur)), borderColor: seriesColor(1), fill: false }],
            plugins: [shade],
        })));
    }

    // ------------------------------------------------------------ event tables
    const eventRows = [
        ...jumps.map((j) => ({ t: j.t, kind: "Position jump", detail: `${fmtDeg(j.from)} → ${fmtDeg(j.to)} in one loop (${j.degPerSec.toFixed(0)}°/s)`, sev: "bad" })),
        ...(zero?.steps || []).map((s) => ({ t: s.t, kind: "Zero step", detail: `zero error ${fmtDeg(s.from)} → ${fmtDeg(s.to)} and stayed`, sev: "bad" })),
        ...effort.stalls.map(([s, e]) => ({ t: s, kind: "Stall", detail: `${(e - s).toFixed(1)}s of current with no motion`, sev: "warn" })),
        ...effort.backdriven.map(([s, e]) => ({ t: s, kind: "Back-driven", detail: `${(e - s).toFixed(1)}s of motion with no current`, sev: "warn" })),
        ...(tracking?.stuckWindows || []).map(([s, e]) => ({ t: s, kind: "Not following", detail: `${(e - s).toFixed(1)}s outside tolerance with voltage applied`, sev: "warn" })),
        ...unwraps.map(([s, e]) => ({ t: s, kind: "Unwrap", detail: `${(e - s).toFixed(1)}s full-turn slew — expected, but fuel fed during it goes anywhere`, sev: "" })),
    ].sort((a, b) => a.t - b.t);

    // ------------------------------------------------------------ slip and correction
    if (correction.events.length) {
        const MODE = {
            corrected: ["ok", "controller fixed it"],
            "partly-corrected": ["warn", "partly fixed"],
            uncorrectable: ["bad", "robot never knew"],
            unclear: ["", "unclear"],
        };
        blocks.push(el("section", {},
            el("h2", {}, "Slip and correction"),
            el("div", { class: "section-note" },
                "Two different slips look nothing alike. If the encoder loses counts the controller sees a sudden error and drives it out — that is the ",
                el("strong", {}, "clawed back"), " column. If the mechanism moves and the encoder does not, the controller sees nothing wrong and does nothing; only the camera notices, and that is ",
                el("strong", {}, "left wrong"), "."),
            el("div", { class: "card table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {}, ["Time", "What", "Size", "Peak error", "Clawed back", "Back in tolerance", "Left wrong", "Verdict"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, correction.events.map((e) => {
                        const [cls, label] = MODE[e.mode];
                        return el("tr", {},
                            el("td", { class: "num" }, `${e.t.toFixed(1)}s`),
                            el("td", {}, e.kind),
                            el("td", { class: "num" }, fmtDeg(e.magnitudeDeg)),
                            el("td", { class: "num" }, fmtDeg(e.errPeakDeg)),
                            el("td", { class: "num" }, fmtDeg(e.recoveredDeg)),
                            el("td", { class: "num" }, e.recoverySec == null ? "never" : `${e.recoverySec.toFixed(2)}s`),
                            el("td", { class: "num" }, e.residualDeg == null ? "—" : fmtDeg(e.residualDeg)),
                            el("td", {}, el("span", { class: `tag ${cls}` }, label)));
                    }))))));
    }

    // ------------------------------------------------------------ limelight corrections
    {
        const rows = limelight.perCamera;
        const parts = [
            el("h2", {}, "Limelight corrections"),
            el("div", { class: "section-note" },
                "Vision never corrects the turret's zero — the controller only trusts the encoder. What vision corrects is the robot ",
                el("em", {}, "pose"), ", and the commanded turret angle is computed from that pose, so every accepted estimate nudges where the turret is told to point. That nudge, in turret degrees, is the correction."),
        ];

        if (rejection.windows.length) {
            parts.push(el("div", { class: "notice bad" },
                el("strong", {}, `The turret camera was distrusted for ${fmtDuration(rejection.totalSec)}. `),
                `Its heading disagreed with the gyro by more than ${rejection.thresholdDeg}° across ${rejection.windows.length} window${rejection.windows.length === 1 ? "" : "s"}. `,
                "This is the second cost of a slip: the camera's transform is built from the turret angle, so a slipped turret feeds it a wrong transform, its estimates get rejected, and the robot loses its best look at the hub exactly when the aim is worst."));
        }

        parts.push(rows.length
            ? el("div", { class: "card table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {}, ["Camera", "Estimates accepted", "Accept rate", "Aim nudges", "Median", "Largest", "Total"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, rows.map((c) =>
                        el("tr", {},
                            el("td", {}, c.camera, c.camera === "TurretLL" ? el("span", { class: "tag", style: "margin-left:6px" }, "on turret") : null),
                            el("td", { class: "num" }, c.acceptedSamples),
                            el("td", { class: `num ${c.acceptRate < 0.5 ? "warn-text" : ""}` }, `${(c.acceptRate * 100).toFixed(0)}%`),
                            el("td", { class: "num" }, c.corrections),
                            el("td", { class: "num" }, fmtDeg(c.medianCorrectionDeg)),
                            el("td", { class: "num" }, fmtDeg(c.maxCorrectionDeg)),
                            el("td", { class: "num" }, fmtDeg(c.totalCorrectionDeg)))))))
            : el("div", { class: "card" }, el("div", { class: "empty" }, "No per-camera vision keys in this log.")));

        if (limelight.headingFixes.length) {
            parts.push(el("div", { class: "notice warn" },
                el("strong", {}, `Gross-heading safety net fired ${limelight.headingFixes.length} time${limelight.headingFixes.length === 1 ? "" : "s"}: `),
                limelight.headingFixes.map((f) => `${f.t.toFixed(1)}s (${fmtDeg(f.errorDeg)} error, aim moved ${fmtDeg(f.nudgeDeg)})`).join(", "),
                ". That only fires when the pose heading was badly wrong to begin with."));
        }
        if (limelight.resets.length) {
            parts.push(el("div", { class: "notice" },
                el("strong", {}, `Manual pose reset ${limelight.resets.length} time${limelight.resets.length === 1 ? "" : "s"} (LB+Select): `),
                limelight.resets.map((r) => `${r.t.toFixed(1)}s (heading ${fmtDeg(r.headingDeltaDeg)}, aim moved ${fmtDeg(r.nudgeDeg)})`).join(", "), "."));
        }
        blocks.push(el("section", {}, ...parts));
    }

    blocks.push(el("section", {},
        el("h2", {}, "Events"),
        el("div", { class: "section-note" }, "Everything the checks above found, in time order. Click a row to scrub the dial to it."),
        el("div", { class: "card" }, eventRows.length
            ? el("div", { class: "table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {}, ["Time", "What", "Detail"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, eventRows.map((r) =>
                        el("tr", {
                            style: "cursor:pointer",
                            onclick: () => {
                                slider.value = Math.round(((r.t - t0) / span) * 1000);
                                paintAt(r.t);
                                slider.scrollIntoView({ behavior: "smooth", block: "center" });
                            },
                        },
                            el("td", { class: "num" }, `${r.t.toFixed(1)}s`),
                            el("td", {}, el("span", { class: `tag ${r.sev}` }, r.kind)),
                            el("td", {}, r.detail))))))
            : el("div", { class: "empty" }, "Nothing flagged. The turret tracked its commands and its zero held."))));

    // ------------------------------------------------------------ new checks, auto-discovered
    if (extras.length) {
        blocks.push(el("section", {},
            el("h2", {}, "Other turret channels in this log"),
            el("div", { class: "section-note" },
                "Turret/* keys this page does not have a purpose-built view for — including any checks added to the robot code since. ",
                "Tell me which of these matter and they get proper treatment."),
            el("div", { class: "card table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {}, ["Key", "Type", "Unit", "Records", "Range / values"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, extras.map((c) => {
                        const numeric = typeof c.values[0]?.[1] === "number";
                        const bool = typeof c.values[0]?.[1] === "boolean";
                        const s = numeric ? seriesStats(c.values) : null;
                        const summary = s
                            ? `${s.min.toFixed(2)} … ${s.max.toFixed(2)}`
                            : bool
                              ? `true for ${trueWindows(c.values, log.lastTs).length} window(s)`
                              : [...new Set(c.values.map(([, v]) => String(v)))].slice(0, 4).join(", ");
                        return el("tr", {},
                            el("td", { class: "mono" }, c.key),
                            el("td", { style: "color:var(--tx-dim)" }, c.type),
                            el("td", {}, c.unit || "—"),
                            el("td", { class: "num" }, c.values.length),
                            el("td", {}, summary));
                    }))))));
    }

    content.replaceChildren(...blocks);
}
