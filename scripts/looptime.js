// Loop-time analysis for a WPILib data log written by this robot code.
//
//   node scripts/looptime.js <file.wpilog> [--worst N]
//
// Reads the Scheduler/* timers (Telemetry.time/timeEnd, logged in SECONDS), DS:enabled, the DogLog
// queue depth and CANivore utilization, and prints:
//   - records per loop and per second (what the log thread has to keep up with)
//   - per-section loop time, all / enabled / disabled: Vision.periodic, SuperStructure.periodic,
//     CommandScheduler.run, the rest of robotPeriodic, and the time outside robotPeriodic
//   - the worst loops, with which section carried the time and whether the robot was enabled
//
// "gap" is the time between consecutive robotPeriodic records, i.e. the true loop period.
// "outside" is gap minus robotPeriodic: DS refresh, SmartDashboard.updateValues, mode periodics.
//
// Reference numbers from 2026-09-05 (rio CPU at 92-95%): enabled loop period median 26-39 ms,
// 57-93% of enabled loops over 25 ms, CommandScheduler.run 15 ms median. Every loop over 300 ms
// was disabled and inside the scheduler at t=25-33 s (PathPlanner warmup); worst enabled 232 ms.
const path = require("path");
const { parse } = require(path.join(__dirname, "wpilog.js"));

const args = process.argv.slice(2);
const file = args.find((a) => !a.startsWith("--"));
if (!file) {
    console.error("usage: node scripts/looptime.js <file.wpilog> [--worst N]");
    process.exit(2);
}
const worstN = args.includes("--worst") ? Number(args[args.indexOf("--worst") + 1]) : 15;

const T = {
    rp: "/Robot/Scheduler/robotPeriodic",
    vis: "/Robot/Scheduler/Vision",
    ss: "/Robot/Scheduler/SuperStructure",
    sch: "/Robot/Scheduler/CommandScheduler",
};
const log = parse(file, new Set([...Object.values(T), "DS:enabled", "/Robot/DogLog/QueuedLogs", "/Robot/CANivore/BusUtilization"]));

let records = 0;
for (const e of log.entries.values()) records += e.count;
const rpEntry = log.byName.get(T.rp);
if (!rpEntry) {
    console.error("no " + T.rp + " in this log; is it from this robot code?");
    process.exit(1);
}
const span = log.lastTs - log.firstTs;
console.log(`${path.basename(file)}: ${span.toFixed(0)} s, ${log.entries.size} entries, ${records} records, ${rpEntry.count} loops`);
console.log(`records/loop ${(records / rpEntry.count).toFixed(1)}   records/s ${(records / span).toFixed(0)}   loops/s ${(rpEntry.count / span).toFixed(1)}`);

const pct = (a, p) => { const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };
const fmt = (a) => a.length
    ? `n=${String(a.length).padStart(6)} med=${pct(a, 0.5).toFixed(1).padStart(6)} p90=${pct(a, 0.9).toFixed(1).padStart(6)} p95=${pct(a, 0.95).toFixed(1).padStart(6)} p99=${pct(a, 0.99).toFixed(1).padStart(6)} max=${Math.max(...a).toFixed(0).padStart(5)}`
    : "n=0";

const enabled = log.byName.get("DS:enabled");
const enabledAt = (t) => { if (!enabled) return null; let v = false; for (const [tt, val] of enabled.values) { if (tt <= t) v = val; else break; } return v; };

// Sub-timers are matched to the robotPeriodic record they precede (they end before it does).
const series = {};
for (const [k, name] of Object.entries(T)) series[k] = (log.byName.get(name) || { values: [] }).values.map(([t, v]) => [t, v * 1000]);
const ptr = { vis: 0, ss: 0, sch: 0 };
const rows = [];
const rp = series.rp;
for (let i = 0; i < rp.length; i++) {
    const [t, v] = rp[i];
    const row = { t, rp: v, gap: i > 0 ? (t - rp[i - 1][0]) * 1000 : NaN, en: enabledAt(t) };
    for (const k of ["vis", "ss", "sch"]) {
        const s = series[k];
        while (ptr[k] + 1 < s.length && s[ptr[k] + 1][0] <= t + 0.0005) ptr[k]++;
        row[k] = s[ptr[k]] && s[ptr[k]][0] > t - 0.5 ? s[ptr[k]][1] : NaN;
    }
    row.rest = row.rp - (row.vis || 0) - (row.ss || 0) - (row.sch || 0);
    row.outside = row.gap - row.rp;
    rows.push(row);
}
const rs = rows.slice(1);

for (const [label, sel] of [["ALL", () => true], ["ENABLED", (r) => r.en === true], ["DISABLED", (r) => r.en === false]]) {
    const R = rs.filter(sel);
    if (!R.length) continue;
    console.log(`\n== ${label}: ${R.length} loops (ms) ==`);
    console.log("  loop period (gap)          ", fmt(R.map((r) => r.gap)));
    console.log("  robotPeriodic total        ", fmt(R.map((r) => r.rp)));
    console.log("    Vision.periodic          ", fmt(R.map((r) => r.vis).filter((x) => !isNaN(x))));
    console.log("    SuperStructure.periodic  ", fmt(R.map((r) => r.ss).filter((x) => !isNaN(x))));
    console.log("    CommandScheduler.run     ", fmt(R.map((r) => r.sch).filter((x) => !isNaN(x))));
    console.log("    rest of robotPeriodic    ", fmt(R.map((r) => r.rest).filter((x) => !isNaN(x))));
    console.log("  outside robotPeriodic      ", fmt(R.map((r) => r.outside)));
    const over = (ms) => R.filter((r) => r.gap > ms).length;
    console.log(`  loops over 25 ms: ${over(25)} (${(100 * over(25) / R.length).toFixed(0)}%)   over 40: ${over(40)}   over 100: ${over(100)}   over 200: ${over(200)}   over 500: ${over(500)}`);
}

console.log(`\n--- worst ${worstN} loops by robotPeriodic (ms) ---`);
for (const r of [...rs].sort((a, b) => b.rp - a.rp).slice(0, worstN)) {
    console.log(`t=${r.t.toFixed(1).padStart(7)} en=${String(r.en).padEnd(5)} rp=${r.rp.toFixed(0).padStart(5)} vision=${(r.vis || 0).toFixed(0).padStart(4)} ss=${(r.ss || 0).toFixed(0).padStart(4)} sched=${(r.sch || 0).toFixed(0).padStart(5)} rest=${r.rest.toFixed(0).padStart(5)} gap=${r.gap.toFixed(0).padStart(5)} outside=${r.outside.toFixed(0).padStart(5)}`);
}

const can = log.byName.get("/Robot/CANivore/BusUtilization");
if (can) console.log(`\nCANivore bus utilization %: ${fmt(can.values.map((x) => x[1]))}`);
const q = log.byName.get("/Robot/DogLog/QueuedLogs");
if (q) console.log(`DogLog queued entries:      ${fmt(q.values.map((x) => x[1]))}`);
