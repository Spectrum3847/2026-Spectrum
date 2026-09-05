/**
 * Chart.js configured for the Spectrum theme.
 *
 * Colors are read from the CSS custom properties in styles.css rather than hardcoded, so the
 * light and dark palettes stay in one place and the theme toggle can re-skin live charts.
 *
 * The categorical series order is fixed and never cycled past slot 8 -- a ninth motor on one
 * chart is a prompt to split the chart, not to invent a hue. Both palettes were checked with the
 * dataviz validator against their own surface (lightness band, chroma floor, CVD separation,
 * normal-vision separation, 3:1 contrast).
 */
import Chart from "chart.js/auto";

const cssVar = (name, fallback) => {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
};

/** Current theme's tokens. Re-read on every chart build so the toggle takes effect. */
export function theme() {
    return {
        text: cssVar("--tx", "#1f1b23"),
        textDim: cssVar("--tx-dim", "#4a4255"),
        textFaint: cssVar("--tx-faint", "#635a72"),
        border: cssVar("--bd", "#e9ddf7"),
        grid: cssVar("--grid", "#efe9f6"),
        surface: cssVar("--s1", "#ffffff"),
        shade: cssVar("--shade", "rgba(107,17,153,0.06)"),
        accent: cssVar("--accent", "#6b1199"),
        ok: cssVar("--ok", "#15803d"),
        warn: cssVar("--warn", "#b45309"),
        bad: cssVar("--bad", "#b91c1c"),
        series: Array.from({ length: 8 }, (_, i) => cssVar(`--series-${i + 1}`, "#7e22ce")),
    };
}

/**
 * Translucent version of a resolved color.
 *
 * Chart fills go straight to a canvas fillStyle, which knows nothing about CSS -- `var(--bad)` or
 * a `color-mix()` string silently paints black there. Always resolve to a literal first.
 */
export function withAlpha(color, alpha) {
    const hex = color.trim();
    const m = /^#([0-9a-f]{3}|[0-9a-f]{6})$/i.exec(hex);
    if (!m) return hex;
    const h = m[1].length === 3 ? m[1].split("").map((c) => c + c).join("") : m[1];
    const r = parseInt(h.slice(0, 2), 16);
    const g = parseInt(h.slice(2, 4), 16);
    const b = parseInt(h.slice(4, 6), 16);
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

/** Fixed-order categorical colors. Index past the end folds back to the muted ink, not a new hue. */
export function seriesColor(i) {
    const t = theme();
    return i < t.series.length ? t.series[i] : t.textFaint;
}

export const COLORS = new Proxy([], {
    get(_, prop) {
        if (prop === "length") return 8;
        const i = Number(prop);
        return Number.isInteger(i) ? seriesColor(i) : undefined;
    },
});

function applyDefaults() {
    const t = theme();
    Chart.defaults.color = t.textDim;
    Chart.defaults.borderColor = t.border;
    Chart.defaults.font.family = '"Plus Jakarta Sans Variable", system-ui, -apple-system, sans-serif';
    Chart.defaults.font.size = 11;
    Chart.defaults.animation = false;
    Chart.defaults.elements.point.radius = 0;
    Chart.defaults.elements.line.borderWidth = 2;
    Chart.defaults.plugins.legend.labels.boxWidth = 10;
    Chart.defaults.plugins.legend.labels.boxHeight = 10;
    Chart.defaults.plugins.legend.labels.usePointStyle = true;
    Chart.defaults.plugins.tooltip.backgroundColor = t.text;
    Chart.defaults.plugins.tooltip.titleColor = t.surface;
    Chart.defaults.plugins.tooltip.bodyColor = t.surface;
    Chart.defaults.plugins.tooltip.padding = 9;
    Chart.defaults.plugins.tooltip.cornerRadius = 7;
    Chart.defaults.plugins.tooltip.displayColors = true;
    Chart.defaults.plugins.tooltip.boxPadding = 4;
}
applyDefaults();
document.addEventListener("themechange", applyDefaults);

/**
 * Shades the disabled stretches of a time chart. Without this every chart is dominated by the
 * robot sitting on a cart, and students read the flat parts as real behaviour.
 */
export function enabledShadePlugin(windows, tMin, tMax) {
    return {
        id: "enabledShade",
        beforeDatasetsDraw(chart) {
            if (!windows.length) return;
            const { ctx, chartArea, scales } = chart;
            if (!chartArea) return;
            ctx.save();
            ctx.fillStyle = theme().shade;
            for (const [s, e] of windows) {
                const x0 = scales.x.getPixelForValue(Math.max(s, tMin));
                const x1 = scales.x.getPixelForValue(Math.min(e, tMax));
                if (x1 > x0) ctx.fillRect(x0, chartArea.top, x1 - x0, chartArea.bottom - chartArea.top);
            }
            ctx.restore();
        },
    };
}

/** Horizontal reference line, used for current limits and bus-off thresholds. */
export function limitLinePlugin(lines) {
    return {
        id: "limitLines",
        afterDatasetsDraw(chart) {
            const { ctx, chartArea, scales } = chart;
            if (!chartArea) return;
            const t = theme();
            ctx.save();
            for (const { value, color = t.bad, label, axis = "y", dash = [5, 4] } of lines) {
                const scale = scales[axis];
                if (!scale) continue;
                const y = scale.getPixelForValue(value);
                if (y < chartArea.top || y > chartArea.bottom) continue;
                ctx.setLineDash(dash);
                ctx.strokeStyle = color;
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.moveTo(chartArea.left, y);
                ctx.lineTo(chartArea.right, y);
                ctx.stroke();
                if (label) {
                    ctx.setLineDash([]);
                    // A 2px surface halo keeps the label legible where it crosses a data line.
                    ctx.font = '700 10px "Plus Jakarta Sans Variable", system-ui, sans-serif';
                    ctx.textAlign = "right";
                    ctx.lineWidth = 3;
                    ctx.strokeStyle = t.surface;
                    ctx.strokeText(label, chartArea.right - 5, y - 5);
                    ctx.fillStyle = color;
                    ctx.fillText(label, chartArea.right - 5, y - 5);
                }
            }
            ctx.restore();
        },
    };
}

/**
 * A time-series line chart.
 *
 * Deliberately single-axis: two measures on one plot with two y-scales lets the author imply any
 * correlation they like by choosing the scales. Pair two charts sharing an x-range instead.
 */
export function timeChart(canvas, { datasets, yTitle, plugins = [], yMax, yMin, yBeginAtZero = true, xRange }) {
    const t = theme();
    return new Chart(canvas, {
        type: "line",
        data: {
            datasets: datasets.map((d, i) => ({
                pointRadius: 0,
                tension: 0,
                borderWidth: 2,
                borderColor: d.borderColor || seriesColor(i),
                ...d,
            })),
        },
        options: {
            parsing: false,
            normalized: true,
            maintainAspectRatio: false,
            interaction: { mode: "index", axis: "x", intersect: false },
            plugins: {
                legend: { display: datasets.length > 1, position: "top", align: "end" },
                tooltip: { callbacks: { title: (items) => `t = ${items[0].parsed.x.toFixed(2)} s` } },
            },
            scales: {
                x: {
                    type: "linear",
                    min: xRange?.[0],
                    max: xRange?.[1],
                    ticks: { maxTicksLimit: 12, callback: (v) => `${v.toFixed(0)}s`, color: t.textFaint },
                    grid: { color: t.grid },
                    border: { color: t.border },
                },
                y: {
                    title: { display: !!yTitle, text: yTitle, color: t.textFaint },
                    beginAtZero: yBeginAtZero,
                    min: yMin,
                    max: yMax,
                    ticks: { color: t.textFaint },
                    grid: { color: t.grid },
                    border: { color: t.border },
                },
            },
        },
        plugins,
    });
}

/** Chart.js wants {x, y}; the parser gives [t, v]. */
export function xy(series) {
    return series.map(([t, v]) => ({ x: t, y: v }));
}

/**
 * Thin a series for plotting. A 250 Hz swerve channel over a 15-minute log is 200k points, which
 * Chart.js will happily accept and then take seconds to draw. Keeps the extremes of each bucket so
 * spikes -- the whole point of a current chart -- survive.
 */
export function decimate(series, maxPoints = 3000) {
    if (series.length <= maxPoints) return series;
    const bucket = Math.ceil(series.length / (maxPoints / 2));
    const out = [];
    for (let i = 0; i < series.length; i += bucket) {
        let lo = series[i];
        let hi = series[i];
        for (let j = i; j < Math.min(i + bucket, series.length); j++) {
            if (series[j][1] < lo[1]) lo = series[j];
            if (series[j][1] > hi[1]) hi = series[j];
        }
        if (lo[0] <= hi[0]) out.push(lo, hi);
        else out.push(hi, lo);
    }
    return out;
}

export { Chart };
