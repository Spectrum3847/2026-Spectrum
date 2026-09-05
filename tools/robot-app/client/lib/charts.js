/** Chart.js defaults tuned for this app's dark shell, plus a couple of chart factories. */
import Chart from "chart.js/auto";

export const COLORS = [
    "#4f8ef7", "#3fbf7f", "#e8b13a", "#e35d6a", "#b57ff5",
    "#37c2c9", "#f08a4b", "#8ea6c8", "#d9679c", "#6ed0a0",
    "#c2a13a", "#7f8ff5",
];

Chart.defaults.color = "#93a0b4";
Chart.defaults.borderColor = "#2b3444";
Chart.defaults.font.family = "system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif";
Chart.defaults.font.size = 11;
Chart.defaults.animation = false;
Chart.defaults.elements.point.radius = 0;
Chart.defaults.elements.line.borderWidth = 1.4;
Chart.defaults.plugins.legend.labels.boxWidth = 10;
Chart.defaults.plugins.legend.labels.boxHeight = 10;
Chart.defaults.plugins.legend.labels.usePointStyle = true;

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
            ctx.fillStyle = "rgba(79,142,247,0.055)";
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
            ctx.save();
            for (const { value, color = "#e35d6a", label, axis = "y", dash = [5, 4] } of lines) {
                const scale = scales[axis];
                if (!scale) continue;
                const y = scale.getPixelForValue(value);
                if (y < chartArea.top || y > chartArea.bottom) continue;
                ctx.setLineDash(dash);
                ctx.strokeStyle = color;
                ctx.lineWidth = 1.2;
                ctx.beginPath();
                ctx.moveTo(chartArea.left, y);
                ctx.lineTo(chartArea.right, y);
                ctx.stroke();
                if (label) {
                    ctx.setLineDash([]);
                    ctx.fillStyle = color;
                    ctx.font = "600 10px system-ui, sans-serif";
                    ctx.textAlign = "right";
                    ctx.fillText(label, chartArea.right - 4, y - 4);
                }
            }
            ctx.restore();
        },
    };
}

const timeScale = (title) => ({
    type: "linear",
    title: { display: !!title, text: title },
    ticks: { maxTicksLimit: 12, callback: (v) => `${v.toFixed(0)}s` },
    grid: { color: "#1b2230" },
});

export function timeChart(canvas, { datasets, yTitle, y1Title, plugins = [], yMax, yBeginAtZero = true }) {
    const scales = {
        x: timeScale("time"),
        y: { title: { display: !!yTitle, text: yTitle }, beginAtZero: yBeginAtZero, max: yMax, grid: { color: "#1b2230" } },
    };
    if (datasets.some((d) => d.yAxisID === "y1")) {
        scales.y1 = { position: "right", title: { display: !!y1Title, text: y1Title }, grid: { drawOnChartArea: false } };
    }
    return new Chart(canvas, {
        type: "line",
        data: { datasets: datasets.map((d) => ({ pointRadius: 0, tension: 0, ...d })) },
        options: {
            parsing: false,
            normalized: true,
            maintainAspectRatio: false,
            interaction: { mode: "nearest", axis: "x", intersect: false },
            plugins: {
                legend: { position: "top", align: "end" },
                tooltip: { callbacks: { title: (items) => `t = ${items[0].parsed.x.toFixed(2)} s` } },
            },
            scales,
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
