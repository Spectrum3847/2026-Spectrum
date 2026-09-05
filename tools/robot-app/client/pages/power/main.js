import "../../styles.css";
import "./analysis.css";
import { mountHeader, el, api, fmtDuration } from "../../lib/ui.js";
import { logPicker, loadProfile, LogModel } from "../../lib/log-loader.js";
import { clipToEnabled, seriesStats, timeAtOrAbove, fitInternalResistance } from "../../lib/log-model.js";
import { timeChart, xy, decimate, enabledShadePlugin, limitLinePlugin, seriesColor, theme, withAlpha, Chart } from "../../lib/charts.js";
import { simulateBreaker } from "../../lib/breaker.js";

mountHeader();
const root = document.getElementById("app");
const profile = await loadProfile();
const charts = [];

const content = el("div", {});
root.replaceChildren(
    el("h1", {}, "Power"),
    el("p", { class: "lede" },
        "Where the battery goes, and how close each motor gets to the limit it was configured with. ",
        "Those limits are compiled into the Java and never appear in the log, so they come from ",
        el("code", {}, "data/robot-profile.json"), "."),
    el("div", { class: "card", style: "margin-bottom:20px" }, logPicker({ onLoad: render })),
    content
);

function tile(label, value, note, cls) {
    return el("div", { class: "stat" },
        el("div", { class: "label" }, label),
        el("div", { class: `value ${cls || ""}` }, value),
        note ? el("div", { class: "note" }, note) : null);
}

function chartCard(title, note, height = "") {
    const canvas = el("canvas");
    const card = el("section", {},
        el("h2", {}, title),
        note ? el("div", { class: "section-note" }, note) : null,
        el("div", { class: "card" }, el("div", { class: `chart-box ${height}` }, canvas)));
    return { card, canvas };
}

function render(log, name) {
    for (const c of charts.splice(0)) c.destroy();
    const m = new LogModel(log, profile);
    const windows = m.enabled;
    const tMin = log.firstTs;
    const tMax = log.lastTs;
    const shade = enabledShadePlugin(windows, tMin, tMax);

    const volts = m.batteryVoltage();
    const amps = m.totalCurrent();
    const voltsEnabled = clipToEnabled(volts, windows);
    const ampsEnabled = clipToEnabled(amps, windows);
    const vStats = seriesStats(voltsEnabled.length ? voltsEnabled : volts);
    const aStats = seriesStats(ampsEnabled.length ? ampsEnabled : amps);
    const energy = m.totalEnergyWh();
    const resistance = fitInternalResistance(volts, amps);
    const breaker = amps.length ? simulateBreaker(amps, { ratedAmps: profile?.mainBreaker?.ratedAmps ?? 120 }) : null;
    const brownout = profile?.brownoutVolts ?? 6.3;

    const blocks = [];

    // ------------------------------------------------------------ warnings
    for (const w of m.warnings) blocks.push(el("div", { class: "notice warn" }, w));

    // ------------------------------------------------------------ tiles
    const capacity = profile?.battery?.usableWh;
    blocks.push(
        el("div", { class: "grid cols-4", style: "margin-bottom:6px" },
            tile("Enabled", fmtDuration(m.enabledSec), `of ${fmtDuration(log.durationSec)} logged`),
            tile("Peak current", aStats ? `${aStats.max.toFixed(0)} A` : "—", aStats ? `${aStats.mean.toFixed(0)} A average` : null,
                aStats && aStats.max > 300 ? "warn" : ""),
            tile("Min voltage", vStats ? `${vStats.min.toFixed(2)} V` : "—", `brownout set to ${brownout} V`,
                vStats ? (vStats.min < brownout + 1.5 ? "bad" : vStats.min < 8 ? "warn" : "ok") : ""),
            tile("Energy used", energy !== null ? `${energy.toFixed(1)} Wh` : "—",
                energy !== null && capacity ? `${((100 * energy) / capacity).toFixed(0)}% of a ${capacity} Wh pack` : null),
            tile("Pack resistance", resistance ? `${(resistance.ohms * 1000).toFixed(1)} mΩ` : "—",
                resistance ? `open-circuit ${resistance.openCircuitVolts.toFixed(2)} V` : "needs voltage + current",
                resistance ? (resistance.ohms > 0.025 ? "bad" : resistance.ohms > 0.018 ? "warn" : "ok") : ""),
            tile("Main breaker", breaker ? `${breaker.peakPct.toFixed(0)}%` : "—",
                breaker ? (breaker.trips.length ? `${breaker.trips.length} simulated trip${breaker.trips.length === 1 ? "" : "s"}` : "of trip threshold") : null,
                breaker ? (breaker.trips.length ? "bad" : breaker.peakPct > 60 ? "warn" : "ok") : ""))
    );

    // ------------------------------------------------------------ pack charts
    //
    // Current and voltage are shown as two charts sharing one x-range rather than one chart with
    // two y-scales. Two scales on one plot let whoever picks the scales imply any correlation they
    // want, and here the honest story -- current spikes, voltage sags -- reads perfectly well
    // stacked.
    if (amps.length || volts.length) {
        const xRange = [log.firstTs, log.lastTs];
        if (amps.length) {
            const { card, canvas } = chartCard("Pack current",
                "Shaded stretches are enabled. Everything outside them is the robot sitting on a cart.");
            blocks.push(card);
            queueMicrotask(() => charts.push(timeChart(canvas, {
                yTitle: "amps",
                xRange,
                datasets: [{ label: "total current", data: xy(decimate(amps)), borderColor: seriesColor(0), fill: false }],
                plugins: [shade],
            })));
        }
        if (volts.length > 1) {
            const { card, canvas } = chartCard("Battery voltage",
                `Same time range. The dashed line is this robot's configured brownout threshold, ${brownout} V.`,
                "short");
            blocks.push(card);
            queueMicrotask(() => charts.push(timeChart(canvas, {
                yTitle: "volts",
                xRange,
                yBeginAtZero: false,
                datasets: [{ label: "battery", data: xy(decimate(volts)), borderColor: seriesColor(3), fill: false }],
                plugins: [shade, limitLinePlugin([{ value: brownout, label: `brownout ${brownout} V` }])],
            })));
        }
    }

    // ------------------------------------------------------------ breaker
    if (breaker && breaker.thermal.length) {
        const { card, canvas } = chartCard("Main breaker thermal model",
            `Bussmann ${profile?.mainBreaker?.ratedAmps ?? 120} A I²t accumulator against the datasheet trip curve. ` +
            "100% means it would have popped. The curve is the slow end of the tolerance band, so this errs toward saying you survived.",
            "short");
        blocks.push(card);
        if (breaker.trips.length) {
            card.append(el("div", { class: "notice bad" },
                el("strong", {}, `Simulated trip at ${breaker.trips.map((t) => `${t.toFixed(1)}s`).join(", ")}. `),
                "Sustained current above the breaker rating for long enough to heat it out."));
        }
        queueMicrotask(() => {
            charts.push(timeChart(canvas, {
                yTitle: "% of trip",
                yMax: 105,
                datasets: [{ label: "breaker heat", data: xy(decimate(breaker.thermal)), borderColor: theme().bad, fill: true, backgroundColor: withAlpha(theme().bad, 0.14) }],
                plugins: [shade, limitLinePlugin([{ value: 100, label: "trip" }])],
            }));
        });
    }

    // ------------------------------------------------------------ per-motor table
    if (m.mechanisms.length && m.hasPerMotorCurrent) {
        const rows = m.mechanisms.map((mech) => {
            const spec = mech.spec;
            const statorE = clipToEnabled(mech.stator, windows);
            const supplyE = clipToEnabled(mech.supply, windows);
            const sStats = seriesStats(statorE);
            const pStats = seriesStats(supplyE);
            const statorLimit = spec?.statorAmps ?? null;
            const supplyLimit = spec?.supplyAmps ?? null;
            // "At the limit" means within 5% of it -- a motor pinned at 95% of its stator ceiling
            // is being clipped just as surely as one at 100%.
            const atStator = statorLimit ? timeAtOrAbove(statorE, statorLimit * 0.95) : null;
            const atSupply = supplyLimit ? timeAtOrAbove(supplyE, supplyLimit * 0.95) : null;
            const energyS = mech.batteryEnergy;
            const wh = energyS.length ? energyS[energyS.length - 1][1] : null;

            const pct = atStator ? atStator.fraction * 100 : 0;
            const barCls = pct > 20 ? "bad" : pct > 5 ? "warn" : "";

            return el("tr", { class: sStats ? null : "dim" },
                el("td", {}, mech.displayName,
                    spec?.followers?.length ? el("span", { class: "tag warn", style: "margin-left:6px", title: "Follower current is not in these traces" }, `+${spec.followers.length}`) : null,
                    mech.aliasText ? el("div", { style: "color:var(--tx-faint);font-size:0.72rem" }, `logged as ${mech.aliasText}`) : null),
                el("td", { class: "num" }, spec ? spec.canId : "—"),
                el("td", {}, spec ? spec.bus : "—"),
                el("td", { class: "num" }, sStats ? `${sStats.max.toFixed(0)} A` : "—"),
                el("td", { class: "num" }, statorLimit ? `${statorLimit} A` : "—"),
                el("td", {}, atStator
                    ? el("div", { style: "display:flex;align-items:center;gap:7px" },
                        el("span", { class: "mono", style: "min-width:44px;text-align:right" }, `${pct.toFixed(1)}%`),
                        el("div", { class: "bar", style: "flex:1" }, el("i", { class: barCls, style: `width:${Math.min(100, pct)}%` })))
                    : "—"),
                el("td", { class: "num" }, pStats ? `${pStats.max.toFixed(0)} A` : "—"),
                el("td", { class: "num" }, supplyLimit ? `${supplyLimit} A` : "—"),
                el("td", { class: "num" }, atSupply ? `${(atSupply.fraction * 100).toFixed(1)}%` : "—"),
                el("td", { class: "num" }, wh !== null ? `${wh.toFixed(1)} Wh` : "—"));
        });

        blocks.push(
            el("section", {},
                el("h2", {}, "Per-motor current against its limit"),
                el("div", { class: "section-note" },
                    "Statistics cover enabled time only. \"At limit\" is the share of samples within 5% of the configured ceiling — that is where the controller starts clipping."),
                el("div", { class: "card table-wrap" },
                    el("table", {},
                        el("thead", {}, el("tr", {},
                            ["Motor", "CAN", "Bus", "Peak stator", "Limit", "At stator limit", "Peak supply", "Limit", "At supply limit", "Energy"]
                                .map((h) => el("th", {}, h)))),
                        el("tbody", {}, rows))))
        );

        // -------------------------------------------------------- per-motor chart
        const plotted = new Set(m.mechanisms.filter((x) => x.stator.length || x.supply.length).slice(0, 4).map((x) => x.name));
        const { card, canvas } = chartCard("Motor current over time", "Click a motor to add or remove it. Dashed lines are that motor's stator limit.", "tall");
        const picker = el("div", { class: "motor-pick" });
        card.querySelector(".card").prepend(picker);
        blocks.push(card);

        const drawMotors = () => {
            picker.replaceChildren(...m.mechanisms.map((mech) =>
                el("button", {
                    class: plotted.has(mech.name) ? "on" : "",
                    onclick: () => { plotted.has(mech.name) ? plotted.delete(mech.name) : plotted.add(mech.name); drawMotors(); },
                }, mech.displayName)));

            for (let i = charts.length - 1; i >= 0; i--) {
                if (charts[i].canvas === canvas) charts.splice(i, 1)[0].destroy();
            }
            const chosen = m.mechanisms.filter((x) => plotted.has(x.name));
            charts.push(timeChart(canvas, {
                yTitle: "amps (stator)",
                datasets: chosen.map((mech, i) => ({
                    label: mech.displayName,
                    data: xy(decimate(mech.stator.length ? mech.stator : mech.supply)),
                    borderColor: seriesColor(i),
                    fill: false,
                })),
                // One dashed line per distinct limit value. Most motors share 80 A, and drawing
                // eight overlapping lines with stacked labels helps nobody.
                plugins: [shade, limitLinePlugin(
                    [...new Set(chosen.filter((c) => c.spec?.statorAmps).map((c) => c.spec.statorAmps))]
                        .sort((a, b) => a - b)
                        .map((v) => ({ value: v, label: `${v} A stator limit`, dash: [3, 4] })))],
            }));
        };
        queueMicrotask(drawMotors);

        // -------------------------------------------------------- energy split
        const withEnergy = m.mechanisms.filter((x) => x.batteryEnergy.length).map((x) => ({
            name: x.displayName,
            wh: x.batteryEnergy[x.batteryEnergy.length - 1][1],
        })).filter((x) => x.wh > 0.001).sort((a, b) => b.wh - a.wh);

        if (withEnergy.length) {
            const canvas2 = el("canvas");
            blocks.push(
                el("section", {},
                    el("h2", {}, "Energy by mechanism"),
                    el("div", { class: "section-note" },
                        "Cumulative watt-hours over the whole log, follower motors included. This is the honest \"what drains the battery\" ranking."),
                    el("div", { class: "card" }, el("div", { class: "chart-box" }, canvas2)))
            );
            queueMicrotask(() => {
                charts.push(new Chart(canvas2, {
                    type: "bar",
                    data: {
                        labels: withEnergy.map((x) => x.name),
                        datasets: [{
                            label: "Wh",
                            data: withEnergy.map((x) => +x.wh.toFixed(2)),
                            backgroundColor: withEnergy.map((_, i) => seriesColor(i)),
                            borderRadius: 4,
                            borderSkipped: "start",
                        }],
                    },
                    options: {
                        indexAxis: "y",
                        maintainAspectRatio: false,
                        plugins: { legend: { display: false } },
                        scales: { x: { title: { display: true, text: "watt-hours" }, grid: { color: theme().grid } }, y: { grid: { display: false } } },
                    },
                }));
            });
        }
    }

    // ------------------------------------------------------------ gotchas
    if (profile?.gotchas?.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Reading these numbers"),
                el("div", { class: "card" },
                    el("ul", { style: "margin:0;padding-left:18px;color:var(--tx-dim);font-size:0.83rem;line-height:1.65" },
                        profile.gotchas.map((g) => el("li", {}, g)))))
        );
    }

    content.replaceChildren(...blocks);
}
