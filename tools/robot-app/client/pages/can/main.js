import "../../styles.css";
import "../power/analysis.css";
import { mountHeader, el, fmtDuration } from "../../lib/ui.js";
import { logPicker, loadProfile, LogModel } from "../../lib/log-loader.js";
import { clipToEnabled, seriesStats } from "../../lib/log-model.js";
import { timeChart, xy, decimate, enabledShadePlugin, limitLinePlugin, seriesColor, theme, withAlpha } from "../../lib/charts.js";
import { connectionDropouts, zeroOutputWhileCommanded, staleTraces, alertEvents, counterSteps } from "../../lib/analyze-can.js";

mountHeader();
const root = document.getElementById("app");
const profile = await loadProfile();
const charts = [];

const content = el("div", {});
root.replaceChildren(
    el("h1", {}, "CAN bus health"),
    el("p", { class: "lede" },
        "Bus load, error counters heading for bus-off, and motors that stopped answering. ",
        "Only three motors log a connection flag, so the rest are inferred from what they reported while being commanded."),
    el("div", { class: "card", style: "margin-bottom:20px" }, logPicker({ onLoad: render })),
    content
);

const tile = (label, value, note, cls) =>
    el("div", { class: "stat" },
        el("div", { class: "label" }, label),
        el("div", { class: `value ${cls || ""}` }, value),
        note ? el("div", { class: "note" }, note) : null);

const fmtWindows = (windows) =>
    windows.slice(0, 6).map(([s, e]) => `${s.toFixed(1)}–${e.toFixed(1)}s`).join(", ") + (windows.length > 6 ? ` +${windows.length - 6} more` : "");

function chartCard(title, note, height = "") {
    const canvas = el("canvas");
    const card = el("section", {},
        el("h2", {}, title),
        note ? el("div", { class: "section-note" }, note) : null,
        el("div", { class: "card" }, el("div", { class: `chart-box ${height}` }, canvas)));
    return { card, canvas };
}

function render(log) {
    for (const c of charts.splice(0)) c.destroy();
    const m = new LogModel(log, profile);
    const shade = enabledShadePlugin(m.enabled, log.firstTs, log.lastTs);
    const blocks = [];

    const util = m.ch("CANivore/BusUtilization");
    const tec = m.ch("CANivore/TransmitErrorCounter");
    const rec = m.ch("CANivore/ReceiveErrorCounter");
    const busOff = m.ch("CANivore/BusOffCount");
    const txFull = m.ch("CANivore/TxFullCount");
    const loop = m.ch("Scheduler/robotPeriodic");

    const dropouts = connectionDropouts(m);
    const commanded = zeroOutputWhileCommanded(m);
    const stale = staleTraces(m);
    const alerts = alertEvents(m);
    const busOffSteps = counterSteps(busOff);
    const txFullSteps = counterSteps(txFull);

    // ------------------------------------------------------------ no CAN telemetry at all
    if (!util.length) {
        blocks.push(el("div", { class: "notice warn" },
            el("strong", {}, "No CANivore telemetry in this log. "),
            "The CANivore/* keys were added to Robot.robotPeriodic after this log was recorded, so bus load and error counters ",
            "are unavailable. The motor-dropout analysis below still works."));
    }

    // ------------------------------------------------------------ tiles
    const uStats = seriesStats(clipToEnabled(util, m.enabled).length ? clipToEnabled(util, m.enabled) : util);
    const maxTec = tec.length ? Math.max(...tec.map(([, v]) => v)) : null;
    const maxRec = rec.length ? Math.max(...rec.map(([, v]) => v)) : null;
    const deadMotors = new Set([...dropouts.map((d) => d.motor), ...commanded.map((c) => c.motor)]);

    blocks.push(
        el("div", { class: "grid cols-4", style: "margin-bottom:6px" },
            tile("Peak bus load", uStats ? `${uStats.max.toFixed(0)}%` : "—",
                uStats ? `${uStats.mean.toFixed(0)}% average` : "not logged",
                uStats ? (uStats.max > 70 ? "bad" : uStats.max > 50 ? "warn" : "ok") : ""),
            tile("Peak TX errors", maxTec !== null ? maxTec : "—", "bus-off at 255",
                maxTec === null ? "" : maxTec > 200 ? "bad" : maxTec > 50 ? "warn" : "ok"),
            tile("Peak RX errors", maxRec !== null ? maxRec : "—", "bus-off at 255",
                maxRec === null ? "" : maxRec > 200 ? "bad" : maxRec > 50 ? "warn" : "ok"),
            tile("Bus-off events", busOffSteps.length || (busOff.length ? 0 : "—"), "controller reset the bus",
                busOffSteps.length ? "bad" : busOff.length ? "ok" : ""),
            tile("TX queue full", txFullSteps.length || (txFull.length ? 0 : "—"), "frames dropped before sending",
                txFullSteps.length ? "warn" : txFull.length ? "ok" : ""),
            tile("Motors that went dark", deadMotors.size, deadMotors.size ? [...deadMotors].join(", ") : "none detected",
                deadMotors.size ? "bad" : "ok"))
    );

    // ------------------------------------------------------------ the headline finding
    if (commanded.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Motors that reported nothing while being commanded"),
                el("div", { class: "section-note" },
                    "Zero volts and zero amps with a setpoint pending is not a resting motor — it is a controller that is not receiving or executing control frames. " +
                    "This is the signature that found the dead hood on 2026-09-04."),
                ...commanded.map((c) =>
                    el("div", { class: "notice bad" },
                        el("strong", {}, `${c.motor}${c.canId ? ` (CAN ${c.canId}${c.bus ? `, ${c.bus}` : ""})` : ""}: `),
                        `${c.totalSec.toFixed(1)}s across ${c.windows.length} window${c.windows.length === 1 ? "" : "s"} `,
                        `at 0 V / 0 A while ${c.signal} disagreed by up to ${c.worstErr.toFixed(1)} ${c.unit}. `,
                        el("div", { style: "margin-top:5px;color:var(--tx-faint);font-size:0.8rem" }, fmtWindows(c.windows)))))
        );
    }

    if (dropouts.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "MotorConnected dropped"),
                el("div", { class: "section-note" },
                    "The direct signal, logged by Turret, Launcher and Hood only, and only for the leader motor."),
                ...dropouts.map((d) =>
                    el("div", { class: "notice bad" },
                        el("strong", {}, `${d.motor}: `),
                        `disconnected for ${d.totalSec.toFixed(1)}s across ${d.windows.length} window${d.windows.length === 1 ? "" : "s"}. `,
                        el("div", { style: "margin-top:5px;color:var(--tx-faint);font-size:0.8rem" }, fmtWindows(d.windows)))))
        );
    }

    if (!commanded.length && !dropouts.length) {
        blocks.push(el("div", { class: "notice" },
            el("strong", {}, "No motor dropouts detected. "),
            "No motor reported zero output while commanded, and no MotorConnected flag went false."));
    }

    // ------------------------------------------------------------ utilization
    if (util.length) {
        const { card, canvas } = chartCard("Bus utilization",
            "CTRE wants this well under 60%. High load delays frames and produces the stale-frame warnings that precede an unresponsive motor.");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "%",
            yMax: 100,
            datasets: [{ label: "CANivore load", data: xy(decimate(util)), borderColor: seriesColor(0), fill: true, backgroundColor: withAlpha(seriesColor(0), 0.12) }],
            plugins: [shade, limitLinePlugin([
                { value: 50, color: theme().warn, label: "50% watch", dash: [3, 5] },
                { value: 70, color: theme().bad, label: "70% too high" },
            ])],
        })));
    }

    // ------------------------------------------------------------ error counters
    if (tec.length || rec.length) {
        const { card, canvas } = chartCard("CAN error counters",
            "A controller goes bus-off at 255 and stops talking entirely. These decay when the bus is healthy, so a rising trend matters more than any single value.",
            "short");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "count",
            datasets: [
                tec.length && { label: "transmit errors", data: xy(decimate(tec)), borderColor: seriesColor(7), fill: false },
                rec.length && { label: "receive errors", data: xy(decimate(rec)), borderColor: seriesColor(3), fill: false },
            ].filter(Boolean),
            plugins: [shade, limitLinePlugin([{ value: 255, label: "bus-off" }])],
        })));
    }

    // ------------------------------------------------------------ loop time
    if (loop.length) {
        const period = profile?.loopPeriodSec ?? 0.02;
        const overruns = loop.filter(([, v]) => v > period).length;
        const { card, canvas } = chartCard("Loop time",
            `Values are in seconds. ${((100 * overruns) / loop.length).toFixed(1)}% of loops ran longer than the ${(period * 1000).toFixed(0)} ms period. ` +
            "A CAN stall shows up here as a spike, because a blocking status-signal read holds up the whole loop.",
            "short");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "seconds",
            datasets: [{ label: "robotPeriodic", data: xy(decimate(loop)), borderColor: seriesColor(0), fill: false }],
            plugins: [shade, limitLinePlugin([{ value: period, color: theme().warn, label: `${(period * 1000).toFixed(0)} ms period` }])],
        })));
    }

    // ------------------------------------------------------------ stale traces
    if (stale.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Suspicious silences"),
                el("div", { class: "section-note" },
                    "Advisory only. Logging is change-based, so a controller that stops answering goes quiet rather than flat — " +
                    "but so does a mechanism that is simply idle. Read these alongside what the robot was doing at the time."),
                el("div", { class: "card table-wrap" },
                    el("table", {},
                        el("thead", {}, el("tr", {}, ["Motor", "CAN", "Normal interval", "Gaps", "Longest gap", "When"].map((h) => el("th", {}, h)))),
                        el("tbody", {}, stale.map((s) =>
                            el("tr", {},
                                el("td", {}, s.motor),
                                el("td", { class: "num" }, s.canId ?? "—"),
                                el("td", { class: "num" }, `${s.medianIntervalMs.toFixed(0)} ms`),
                                el("td", { class: "num" }, s.count),
                                el("td", { class: "num" }, `${s.gaps[0][2].toFixed(1)}s`),
                                el("td", { class: "mono", style: "font-size:0.75rem;color:var(--tx-faint)" },
                                    `${s.gaps[0][0].toFixed(1)}–${s.gaps[0][1].toFixed(1)}s`)))))))
        );
    }

    // ------------------------------------------------------------ alerts
    if (alerts.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Alerts"),
                el("div", { class: "section-note" }, "Deduplicated, so each line is the first time that message appeared."),
                el("div", { class: "card table-wrap" },
                    el("table", {},
                        el("thead", {}, el("tr", {}, ["Time", "Level", "Message"].map((h) => el("th", {}, h)))),
                        el("tbody", {}, alerts.map((a) =>
                            el("tr", {},
                                el("td", { class: "num" }, `${a.t.toFixed(1)}s`),
                                el("td", {}, el("span", { class: `tag ${a.level === "error" ? "bad" : a.level === "warning" ? "warn" : ""}` }, a.level)),
                                el("td", {}, a.text)))))))
        );
    }

    // ------------------------------------------------------------ inventory
    if (profile) {
        const rows = [];
        for (const mo of profile.motors) {
            rows.push({ name: mo.name, id: mo.canId, bus: mo.bus, kind: "TalonFX", flag: mo.logsMotorConnected ? "logs connection" : null });
            for (const f of mo.followers) rows.push({ name: f.name, id: f.canId, bus: mo.bus, kind: `TalonFX follower of ${mo.name}`, flag: "invisible in logs" });
        }
        const sw = profile.swerve;
        sw.drive.canIds.forEach((id, i) => rows.push({ name: `Swerve drive ${i + 1}`, id, bus: sw.drive.bus, kind: "TalonFX" }));
        sw.steer.canIds.forEach((id, i) => rows.push({ name: `Swerve steer ${i + 1}`, id, bus: sw.steer.bus, kind: "TalonFX" }));
        sw.cancoders.canIds.forEach((id, i) => rows.push({ name: `CANcoder ${i + 1}`, id, bus: sw.cancoders.bus, kind: "CANcoder" }));
        rows.push({ name: "Pigeon 2", id: sw.pigeon.canId, bus: sw.pigeon.bus, kind: "Pigeon2" });
        rows.sort((a, b) => (a.bus === b.bus ? a.id - b.id : a.bus.localeCompare(b.bus)));

        const rioBus = profile.buses.find((b) => !b.monitored);
        blocks.push(
            el("section", {},
                el("h2", {}, "Device inventory"),
                el("div", { class: "section-note" }, `${rows.length} devices from the robot profile. Not read from the log — this is what should be on the buses.`),
                rioBus?.note ? el("div", { class: "notice warn" }, el("strong", {}, "Blind spot. "), rioBus.note) : null,
                el("div", { class: "card table-wrap" },
                    el("table", {},
                        el("thead", {}, el("tr", {}, ["CAN", "Bus", "Device", "Type", ""].map((h) => el("th", {}, h)))),
                        el("tbody", {}, rows.map((r) =>
                            el("tr", {},
                                el("td", { class: "num" }, r.id),
                                el("td", {}, el("span", { class: `tag ${r.bus === "rio" ? "warn" : ""}` }, r.bus)),
                                el("td", {}, r.name),
                                el("td", { style: "color:var(--tx-dim)" }, r.kind),
                                el("td", {}, r.flag ? el("span", { class: "tag" }, r.flag) : "")))))))
        );
    }

    content.replaceChildren(...blocks);
}
