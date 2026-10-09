// What a motor is doing while it sits at its stator limit.
//   node atlimit.js <file.wpilog> <prefix> <limitAmps> [--frac 0.9]
// Prints, for at-limit samples while enabled: how many, median |RPM|, median |voltage|, share of
// samples with |RPM| < 5 (stalled), and for the turret the position vs commanded error and unwrap.
// Then a minute-by-minute (10 s bucket) table of at-limit seconds by state.
const path = require("path");
const { parse } = require(path.join(__dirname, "wpilog.js"));
const [file, prefix, limitStr, ...rest] = process.argv.slice(2);
const limit = Number(limitStr);
const frac = rest.includes("--frac") ? Number(rest[rest.indexOf("--frac") + 1]) : 0.9;
const P = "/Robot/" + prefix + "/";
const keys = ["StatorCurrent", "RPM", "Voltage", "PositionDegrees", "CommandedDegrees", "Unwrapping", "Position", "SupplyCurrent"].map((k) => P + k);
const stateKey = "/Robot/SuperStructure/CurrentSuperState";
const log = parse(file, new Set([...keys, stateKey, "DS:enabled", "/Robot/IntakeExtension/SystemState", "/Robot/DyeRotor/SystemState", "/Robot/Turret/SystemState"]));
const stepper = (e) => { if (!e) return () => undefined; const v = e.values; let i = 0; return (t) => { while (i + 1 < v.length && v[i + 1][0] <= t) i++; return v[i] && v[i][0] <= t ? v[i][1] : undefined; }; };
const en = stepper(log.byName.get("DS:enabled")), st = stepper(log.byName.get(stateKey));
const rpm = stepper(log.byName.get(P + "RPM")), volt = stepper(log.byName.get(P + "Voltage"));
const pos = stepper(log.byName.get(P + "PositionDegrees") || log.byName.get(P + "Position")), cmd = stepper(log.byName.get(P + "CommandedDegrees"));
const unw = stepper(log.byName.get(P + "Unwrapping"));
const pct = (a, p) => { const s = [...a].sort((x, y) => x - y); return s.length ? s[Math.min(s.length - 1, Math.floor(p * s.length))] : NaN; };
const sc = log.byName.get(P + "StatorCurrent");
if (!sc) { console.log("no " + P + "StatorCurrent"); process.exit(1); }
const rpms = [], volts = [], errs = [], buckets = new Map();
let n = 0, stalled = 0, unwrapN = 0, prev = null, total = 0;
for (const [t, v] of sc.values) {
    const dt = prev === null ? 0 : Math.min(t - prev, 0.5); prev = t;
    if (en(t) !== true) continue;
    total += dt;
    if (Math.abs(v) < frac * limit) continue;
    n++;
    const r = rpm(t); if (r !== undefined) { rpms.push(Math.abs(r)); if (Math.abs(r) < 5) stalled++; }
    const vv = volt(t); if (vv !== undefined) volts.push(Math.abs(vv));
    const p = pos(t), c = cmd(t); if (p !== undefined && c !== undefined) errs.push(Math.abs(c - p));
    if (unw(t) === true) unwrapN++;
    const b = Math.floor(t / 10) * 10, s = st(t) || "?";
    if (!buckets.has(b)) buckets.set(b, new Map());
    buckets.get(b).set(s, (buckets.get(b).get(s) || 0) + dt);
}
console.log(`${path.basename(file)} ${prefix} >= ${frac * limit} A while enabled: ${n} samples of ${sc.values.length} (${total.toFixed(0)} s enabled)`);
if (rpms.length) console.log(`  |RPM| med ${pct(rpms, 0.5).toFixed(0)} p90 ${pct(rpms, 0.9).toFixed(0)}   stalled (<5 RPM) ${(100 * stalled / rpms.length).toFixed(0)}%`);
if (volts.length) console.log(`  |V| med ${pct(volts, 0.5).toFixed(1)} p90 ${pct(volts, 0.9).toFixed(1)}`);
if (errs.length) console.log(`  |cmd-pos| deg med ${pct(errs, 0.5).toFixed(1)} p90 ${pct(errs, 0.9).toFixed(1)} max ${Math.max(...errs).toFixed(0)}   unwrapping ${(100 * unwrapN / n).toFixed(0)}%`);
console.log("  at-limit seconds per 10 s bucket (t=start): state seconds");
for (const [b, m] of [...buckets.entries()].sort((a, c) => a[0] - c[0])) {
    const tot = [...m.values()].reduce((a, c) => a + c, 0);
    if (tot < 0.5) continue;
    console.log(`   t=${String(b).padStart(5)}  ${tot.toFixed(1).padStart(5)} s   ` + [...m.entries()].sort((a, c) => c[1] - a[1]).map(([s, v]) => `${s} ${v.toFixed(1)}`).join(", "));
}
