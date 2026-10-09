// Motor current analysis for a WPILib data log written by this robot code.
//
//   node scripts/currents.js <file.wpilog> [--limit-frac 0.9]
//
// Reads every */StatorCurrent, */SupplyCurrent and */Temp entry (Mechanism.logDiagnostics), the
// battery voltage, DS:enabled and SuperStructure/CurrentSuperState, and prints per motor:
//   - stator and supply current stats while enabled, and the temperature rise
//   - how long each motor spent at or above --limit-frac of its configured stator limit, and in
//     which super states that time was spent
//   - the lowest battery voltage and what was drawing at that moment
// Limits are the values in the subsystem configs; keep the table below in step with them.
const path = require("path");
const { parse } = require(path.join(__dirname, "wpilog.js"));

const args = process.argv.slice(2);
const file = args.find((a) => !a.startsWith("--"));
if (!file) {
    console.error("usage: node scripts/currents.js <file.wpilog> [--limit-frac 0.9]");
    process.exit(2);
}
const limitFrac = args.includes("--limit-frac") ? Number(args[args.indexOf("--limit-frac") + 1]) : 0.9;

// Stator limits (A) by log prefix, from the configs.
const STATOR_LIMITS = [
    ["Turret", 80],
    ["Hood", 80],
    ["IntakeExtension", 80],
    ["IntakeRoller", 80],
    ["IntakeKicker", 80],
    ["Launcher", 80],
    ["LauncherTower", 80],
    ["DyeRotor", 80],
    ["Rotor", 80],
    ["Feeder", 120],
    ["Drive", 120],
    ["Steer", 60],
];
const limitFor = (prefix) => {
    const hit = STATOR_LIMITS.find(([k]) => prefix.includes(k));
    return hit ? hit[1] : null;
};

// First pass: names only.
const names = [...parse(file, new Set()).byName.keys()];
const isCurrent = (n) => /\/(StatorCurrent|SupplyCurrent|Temp)$/.test(n);
const stateKey = names.find((n) => /SuperStructure\/CurrentSuperState$/.test(n));
// BatteryLogger's voltage is logged every loop; SystemStats' only every few seconds.
const battKey = names.find((n) => n.endsWith("/BatteryLogger/BatteryVoltage"))
    || names.find((n) => /Battery.*Voltage$|BatteryVoltage$/.test(n));
const wanted = new Set([...names.filter(isCurrent), "DS:enabled", stateKey, battKey].filter(Boolean));
const log = parse(file, wanted);
const span = log.lastTs - log.firstTs;

const pct = (a, p) => { const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };
const stepper = (entry) => {
    // Returns value at time t for a step series, with a moving pointer (calls must be in time order).
    if (!entry) return () => undefined;
    const v = entry.values; let i = 0;
    return (t) => { while (i + 1 < v.length && v[i + 1][0] <= t) i++; return v[i] && v[i][0] <= t ? v[i][1] : undefined; };
};

const enabledAt = stepper(log.byName.get("DS:enabled"));
const stateAt = stepper(stateKey && log.byName.get(stateKey));

console.log(`${path.basename(file)}: ${span.toFixed(0)} s`);
const enabledEntry = log.byName.get("DS:enabled");
if (enabledEntry) {
    let en = 0, last = null;
    for (const [t, v] of enabledEntry.values) { if (last !== null && last[1]) en += t - last[0]; last = [t, v]; }
    if (last && last[1]) en += log.lastTs - last[0];
    console.log(`enabled ${en.toFixed(0)} s of ${span.toFixed(0)} s`);
}

// Group by motor prefix.
const motors = new Map();
for (const n of wanted) {
    const m = n.match(/^(.*)\/(StatorCurrent|SupplyCurrent|Temp)$/);
    if (!m) continue;
    const prefix = m[1].replace(/^\/Robot\//, "");
    if (!motors.has(prefix)) motors.set(prefix, {});
    motors.get(prefix)[m[2]] = log.byName.get(n);
}

const rows = [];
for (const [prefix, e] of [...motors.entries()].sort()) {
    const st = e.StatorCurrent, su = e.SupplyCurrent, tp = e.Temp;
    if (!st || !st.values.length) continue;
    const limit = limitFor(prefix);
    const enAt = stepper(log.byName.get("DS:enabled"));
    const sAt = stepper(stateKey && log.byName.get(stateKey));
    const enabledStator = [], enabledSupply = [];
    let atLimitSecs = 0, atLimitEnabledSecs = 0;
    const byState = new Map();
    let prevT = null;
    for (const [t, v] of st.values) {
        const dt = prevT === null ? 0 : Math.min(t - prevT, 0.5);
        prevT = t;
        const en = enAt(t) === true;
        if (en) enabledStator.push(Math.abs(v));
        if (limit && Math.abs(v) >= limitFrac * limit) {
            atLimitSecs += dt;
            if (en) {
                atLimitEnabledSecs += dt;
                const s = sAt(t) || "?";
                byState.set(s, (byState.get(s) || 0) + dt);
            }
        }
    }
    // Fresh cursor: the stepper only walks forward, so it cannot be reused for a second series.
    const enAtSupply = stepper(log.byName.get("DS:enabled"));
    if (su) for (const [t, v] of su.values) if (enAtSupply(t) === true) enabledSupply.push(Math.abs(v));
    const temps = tp ? tp.values.map((x) => x[1]) : [];
    rows.push({ prefix, limit, enabledStator, enabledSupply, atLimitSecs, atLimitEnabledSecs, byState, tempStart: temps[0], tempEnd: temps[temps.length - 1], tempMax: temps.length ? Math.max(...temps) : undefined });
}

const f1 = (x) => (x === undefined || isNaN(x) ? "   -" : x.toFixed(0).padStart(4));
console.log(`\n== per motor, enabled only (A). "at limit" = stator >= ${(limitFrac * 100).toFixed(0)}% of configured stator limit ==`);
console.log("motor".padEnd(28) + "limit  stator med  p90  p99  max | supply med  p90  max | at-limit s (enabled) | temp start->max");
for (const r of rows.sort((a, b) => b.atLimitEnabledSecs - a.atLimitEnabledSecs)) {
    const s = r.enabledStator, u = r.enabledSupply;
    console.log(
        r.prefix.padEnd(28) + String(r.limit ?? "-").padStart(5) + "  " +
        (s.length ? `${f1(pct(s, 0.5))} ${f1(pct(s, 0.9))} ${f1(pct(s, 0.99))} ${f1(Math.max(...s))}` : "   no enabled data") +
        "        | " + (u.length ? `${f1(pct(u, 0.5))} ${f1(pct(u, 0.9))} ${f1(Math.max(...u))}` : "  -    -    -") +
        `     | ${r.atLimitSecs.toFixed(1).padStart(6)} (${r.atLimitEnabledSecs.toFixed(1)})` +
        `      | ${f1(r.tempStart)} -> ${f1(r.tempMax)}`
    );
}

console.log(`\n== at-limit time by super state (enabled, seconds) ==`);
for (const r of rows) {
    if (!r.byState.size) continue;
    console.log(`${r.prefix}: ` + [...r.byState.entries()].sort((a, b) => b[1] - a[1]).map(([s, t]) => `${s} ${t.toFixed(1)}`).join(", "));
}

// Time in each super state while enabled.
if (stateKey) {
    const st = log.byName.get(stateKey);
    const enAt = stepper(log.byName.get("DS:enabled"));
    const tot = new Map();
    for (let i = 0; i < st.values.length; i++) {
        const [t, s] = st.values[i];
        const tEnd = i + 1 < st.values.length ? st.values[i + 1][0] : log.lastTs;
        if (enAt(t) === true) tot.set(s, (tot.get(s) || 0) + (tEnd - t));
    }
    console.log(`\n== enabled time by super state (s) ==`);
    console.log([...tot.entries()].sort((a, b) => b[1] - a[1]).map(([s, t]) => `${s} ${t.toFixed(0)}`).join(", "));
}

// Battery.
if (battKey) {
    const b = log.byName.get(battKey);
    const vals = b.values.map((x) => x[1]);
    let min = b.values[0];
    for (const x of b.values) if (x[1] < min[1]) min = x;
    console.log(`\n== battery (${battKey}) ==`);
    console.log(`med ${pct(vals, 0.5).toFixed(2)} V   p5 ${pct(vals, 0.05).toFixed(2)} V   min ${min[1].toFixed(2)} V at t=${min[0].toFixed(1)} (${stateAt(min[0]) || "?"}, enabled=${enabledAt(min[0])})`);
    const belowN = (v) => { let s = 0, prev = null; for (const [t, x] of b.values) { if (prev !== null && x < v) s += Math.min(t - prev, 0.5); prev = t; } return s; };
    console.log(`time below 9 V: ${belowN(9).toFixed(1)} s   below 8 V: ${belowN(8).toFixed(1)} s   below 7 V: ${belowN(7).toFixed(1)} s`);
    // Who was drawing at the minimum: supply currents within 0.1 s of it.
    const draws = [];
    for (const [prefix, e] of motors) {
        if (!e.SupplyCurrent) continue;
        let best = null;
        for (const [t, v] of e.SupplyCurrent.values) { if (Math.abs(t - min[0]) <= 0.15 && (best === null || Math.abs(v) > best)) best = Math.abs(v); }
        if (best !== null) draws.push([prefix, best]);
    }
    draws.sort((a, b) => b[1] - a[1]);
    console.log("supply draw at the minimum: " + draws.slice(0, 8).map(([p, v]) => `${p} ${v.toFixed(0)} A`).join(", "));
}
