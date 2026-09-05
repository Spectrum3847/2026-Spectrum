/**
 * Normalizes a parsed .wpilog into one shape the analysis pages can rely on, across both log eras.
 *
 * Era 1 (April 2026, competition robot): logExtras was on, so SystemStats/* exists; mechanisms
 *   were named Intake / IndexerBed / IndexerTower / Launcher Top Left; no CANivore/* keys.
 * Era 2 (2026 offseason bot, current): logExtras is off, so BatteryLogger is the only power
 *   source; mechanisms are IntakeRoller / Rotor / Feeder / LauncherTower / Turret; CANivore/* is
 *   logged explicitly by Robot.robotPeriodic.
 *
 * Mechanisms are DISCOVERED from the log rather than assumed from the profile, so a log from a
 * differently configured robot still opens. The profile only enriches what was found -- current
 * limits above all, since those are never logged.
 */

const P = "/Robot/";

export class LogModel {
    constructor(log, profile) {
        this.log = log;
        this.profile = profile;
        this.era = log.has(`${P}CANivore/BusUtilization`) ? "offseason" : log.has(`${P}SystemStats/BatteryVoltage`) ? "legacy" : "unknown";
        this.enabled = enabledWindows(log);
        this.enabledSec = this.enabled.reduce((a, [s, e]) => a + (e - s), 0);
        this.mechanisms = this.#discoverMechanisms();
        this.warnings = [];
        this.#checkWarnings();
    }

    ch(name) {
        return this.log.channel(name.startsWith("/") ? name : P + name);
    }

    has(name) {
        return this.log.has(name.startsWith("/") ? name : P + name);
    }

    /**
     * Pick whichever of several candidate channels actually carries data.
     *
     * Presence is not enough: some logs declare BatteryLogger/BatteryVoltage and then write a
     * single record to it, and taking that over a fully populated SystemStats channel produces a
     * confidently wrong "minimum voltage" -- the worst kind of bug in a tool people trust.
     */
    #richest(...names) {
        let best = [];
        let bestName = null;
        for (const n of names) {
            const s = this.ch(n);
            if (s.length > best.length) {
                best = s;
                bestName = n;
            }
        }
        return { series: best, name: bestName };
    }

    /** Battery pack voltage. BatteryLogger in current logs, SystemStats in April ones. */
    batteryVoltage() {
        const r = this.#richest("BatteryLogger/BatteryVoltage", "SystemStats/BatteryVoltage");
        this.batteryVoltageKey = r.name;
        return r.series;
    }

    /** Total pack current. BatteryLogger sums per-mechanism supply current plus fixed overhead. */
    totalCurrent() {
        const r = this.#richest("BatteryLogger/Current", "SystemStats/BatteryCurrent");
        this.totalCurrentKey = r.name;
        return r.series;
    }

    /** True when the log has no per-motor current channels at all, only BatteryLogger rollups. */
    get hasPerMotorCurrent() {
        return this.mechanisms.some((m) => m.stator.length || m.supply.length);
    }

    totalEnergyWh() {
        const e = this.ch("BatteryLogger/Energy");
        return e.length ? e[e.length - 1][1] : null;
    }

    /**
     * One entry per mechanism the log actually contains, joined to its profile entry (and hence
     * its current limits) through the profile key or the legacy rename table.
     */
    #discoverMechanisms() {
        const motors = this.profile?.motors || [];
        const byKey = new Map(motors.map((m) => [m.key, m]));
        const byChannel = new Map(motors.map((m) => [m.batteryChannel, m]));
        const renames = this.profile?.logKeys?.legacy?.renamedMechanisms || {};

        /**
         * Collapse the several names one physical motor goes by into a single canonical key.
         *
         * A motor can appear as its log key root ("FuelIntake"), its BatteryLogger Config name
         * ("Intake Roller Left"), or an older season's name for either ("Intake", "IndexerBed").
         * Resolving each namespace separately produced duplicate rows for the same motor, so every
         * discovery path goes through here.
         */
        const resolve = (name) => {
            const direct = byKey.get(name) || byChannel.get(name);
            if (direct) return { key: direct.key, spec: direct, alias: name === direct.key ? null : name };
            const renamed = renames[name];
            if (renamed && byKey.get(renamed)) return { key: renamed, spec: byKey.get(renamed), alias: name };
            return { key: name, spec: null, alias: null };
        };

        const found = new Map();
        const ensure = (name) => {
            const { key, spec, alias } = resolve(name);
            if (!found.has(key)) {
                found.set(key, {
                    name: key,
                    displayName: spec?.name || key,
                    spec,
                    aliases: new Set(),
                    stator: [],
                    supply: [],
                    voltage: [],
                    temp: [],
                    batteryCurrent: [],
                    batteryEnergy: [],
                    motorConnected: [],
                });
            }
            const entry = found.get(key);
            if (alias) entry.aliases.add(alias);
            if (spec && !entry.spec) entry.spec = spec;
            return entry;
        };

        const skip = (c) => c === "BatteryLogger" || c === "SystemStats" || c === "Swerve";

        for (const { capture, values } of this.log.matching(/^\/Robot\/([^/]+)\/StatorCurrent$/)) {
            if (!skip(capture)) ensure(capture).stator = values;
        }
        for (const { capture, values } of this.log.matching(/^\/Robot\/([^/]+)\/SupplyCurrent$/)) {
            if (!skip(capture)) ensure(capture).supply = values;
        }
        for (const { capture, values } of this.log.matching(/^\/Robot\/([^/]+)\/Voltage$/)) {
            if (!skip(capture)) ensure(capture).voltage = values;
        }
        for (const { capture, values } of this.log.matching(/^\/Robot\/([^/]+)\/Temp$/)) {
            if (!skip(capture)) ensure(capture).temp = values;
        }
        for (const { capture, values } of this.log.matching(/^\/Robot\/([^/]+)\/MotorConnected$/)) {
            if (!skip(capture)) ensure(capture).motorConnected = values;
        }
        for (const { capture, values } of this.log.matching(/^\/Robot\/BatteryLogger\/Current\/Mechanisms\/(.+)$/)) {
            ensure(capture).batteryCurrent = values;
        }
        for (const { capture, values } of this.log.matching(/^\/Robot\/BatteryLogger\/Energy\/Mechanisms\/(.+)$/)) {
            ensure(capture).batteryEnergy = values;
        }

        return [...found.values()]
            .filter((m) => m.stator.length || m.supply.length || m.batteryCurrent.length)
            .map((m) => ({ ...m, aliasText: [...m.aliases].join(", ") || null }))
            .sort((a, b) => a.displayName.localeCompare(b.displayName));
    }

    #checkWarnings() {
        if (this.era === "legacy") {
            this.warnings.push(
                "This is an April 2026 competition-robot log. Mechanism names and available channels differ from the offseason bot; " +
                    "renamed mechanisms are matched through the profile's legacy table, and current limits shown are the OFFSEASON bot's."
            );
        }
        const withFollowers = this.mechanisms.filter((m) => m.spec?.followers?.length);
        if (withFollowers.length) {
            this.warnings.push(
                `Leader-only currents: ${withFollowers.map((m) => m.displayName).join(", ")} ` +
                    `${withFollowers.length === 1 ? "has a follower" : "have followers"} whose current is NOT in the per-motor stator/supply traces. ` +
                    "True draw is roughly double. The BatteryLogger column does include followers."
            );
        }
        if (!this.has("BatteryLogger/Current") && !this.has("SystemStats/BatteryCurrent")) {
            this.warnings.push("No total-current channel in this log, so pack-level power and the breaker simulation are unavailable.");
        }
        // Sparse channels produce confident-looking numbers from almost no data. Say so.
        const v = this.batteryVoltage();
        const perSec = this.log.durationSec > 0 ? v.length / this.log.durationSec : 0;
        if (v.length && perSec < 1) {
            this.warnings.push(
                `Battery voltage was logged only ${v.length} time${v.length === 1 ? "" : "s"} across ` +
                    `${this.log.durationSec.toFixed(0)}s (${this.batteryVoltageKey}). Minimum voltage and pack resistance ` +
                    "are unreliable here -- a sag between samples is simply invisible."
            );
        }
        if (!this.hasPerMotorCurrent) {
            this.warnings.push(
                "This log has no per-motor current channels, only BatteryLogger rollups, so limit analysis is unavailable. " +
                    "Per-motor stator and supply current arrived in later logs."
            );
        }
    }
}

export function enabledWindows(log) {
    const ds = log.channel("DS:enabled");
    const out = [];
    let open = null;
    for (const [t, v] of ds) {
        if (v && open === null) open = t;
        else if (!v && open !== null) {
            out.push([open, t]);
            open = null;
        }
    }
    if (open !== null) out.push([open, log.lastTs]);
    return out;
}

export function inWindows(t, windows) {
    for (const [s, e] of windows) if (t >= s && t <= e) return true;
    return false;
}

/** Restrict a series to the enabled windows. Disabled time makes every average meaningless. */
export function clipToEnabled(series, windows) {
    if (!windows.length) return series;
    return series.filter(([t]) => inWindows(t, windows));
}

export function seriesStats(series) {
    if (!series.length) return null;
    let min = Infinity;
    let max = -Infinity;
    let sum = 0;
    let n = 0;
    for (const [, v] of series) {
        if (typeof v !== "number" || !Number.isFinite(v)) continue;
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
        n++;
    }
    return n ? { min, max, mean: sum / n, n } : null;
}

/**
 * Fraction of samples at or above a threshold, and the total time spent there.
 * Time is accumulated from sample spacing so it stays honest with DogLog's change-only logging.
 */
export function timeAtOrAbove(series, threshold) {
    let samples = 0;
    let seconds = 0;
    for (let i = 0; i < series.length; i++) {
        const [t, v] = series[i];
        if (Math.abs(v) < threshold) continue;
        samples++;
        const next = series[i + 1]?.[0];
        const dt = next !== undefined ? Math.min(next - t, 0.1) : 0.02;
        seconds += dt;
    }
    return { samples, fraction: series.length ? samples / series.length : 0, seconds };
}

/**
 * Least-squares fit of pack voltage against total current: V = V0 - I*R.
 * The slope is the pack's effective internal resistance, which is the single best number for
 * "is this battery tired?" A healthy FRC pack sits near 0.012-0.020 ohm.
 */
export function fitInternalResistance(voltage, current) {
    if (voltage.length < 10 || current.length < 10) return null;
    const pairs = [];
    let ci = 0;
    for (const [t, v] of voltage) {
        while (ci < current.length - 1 && current[ci + 1][0] <= t) ci++;
        const c = current[ci];
        if (c && Math.abs(c[0] - t) < 0.1) pairs.push([c[1], v]);
    }
    if (pairs.length < 10) return null;

    let sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const [x, y] of pairs) {
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const n = pairs.length;
    const denom = n * sxx - sx * sx;
    if (Math.abs(denom) < 1e-9) return null;
    const slope = (n * sxy - sx * sy) / denom;
    const intercept = (sy - slope * sx) / n;
    return { ohms: -slope, openCircuitVolts: intercept, samples: n };
}
