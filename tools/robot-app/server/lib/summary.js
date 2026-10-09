import { parseWpilog } from "../../client/lib/wpilog.js";

/**
 * Channels worth keeping when we only want headline numbers for the manifest. Everything else is
 * counted but not stored, which keeps a full-match log to a couple of MB instead of tens.
 */
const SUMMARY_RE =
    /^(\/Robot\/(BatteryLogger\/(Current|Power|Energy|BatteryVoltage)$|CANivore\/|Scheduler\/|Match Data\/|BuildConstants\/|SystemStats\/(BatteryVoltage|BatteryCurrent|BrownedOut)$|[^/]+\/(StatorCurrent|SupplyCurrent|MotorConnected)$)|DS:(enabled|autonomous|test|eventName|matchNumber|matchType|alliance)$)/;

function stats(series) {
    if (!series.length) return null;
    let min = Infinity;
    let max = -Infinity;
    let sum = 0;
    for (const [, v] of series) {
        if (typeof v !== "number" || !Number.isFinite(v)) continue;
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
    }
    if (min === Infinity) return null;
    return { min, max, mean: sum / series.length, n: series.length };
}

function percentile(series, p) {
    const vals = series.map(([, v]) => v).filter((v) => typeof v === "number" && Number.isFinite(v));
    if (!vals.length) return null;
    vals.sort((a, b) => a - b);
    return vals[Math.min(vals.length - 1, Math.floor((p / 100) * vals.length))];
}

function firstString(log, name) {
    const s = log.channel(name);
    return s.length ? s[s.length - 1][1] : null;
}

/**
 * Enabled windows, derived from DS:enabled. Almost every meaningful statistic should be scoped to
 * these -- a log is mostly the robot sitting disabled on a cart, and including that time makes
 * every average meaningless.
 */
export function enabledWindows(log) {
    const ds = log.channel("DS:enabled");
    const windows = [];
    let open = null;
    for (const [t, v] of ds) {
        if (v && open === null) open = t;
        else if (!v && open !== null) {
            windows.push([open, t]);
            open = null;
        }
    }
    if (open !== null) windows.push([open, log.lastTs]);
    return windows;
}

export function enabledSeconds(log) {
    return enabledWindows(log).reduce((a, [s, e]) => a + (e - s), 0);
}

/**
 * Headline numbers stored in the logs-repo manifest, so the team can scan a season of logs
 * without opening any of them.
 */
export function summarize(buffer, { name = null } = {}) {
    const log = parseWpilog(buffer, { keep: (n) => SUMMARY_RE.test(n) });

    const battery = log.has("/Robot/BatteryLogger/BatteryVoltage")
        ? log.channel("/Robot/BatteryLogger/BatteryVoltage")
        : log.channel("/Robot/SystemStats/BatteryVoltage");
    const totalCurrent = log.has("/Robot/BatteryLogger/Current")
        ? log.channel("/Robot/BatteryLogger/Current")
        : log.channel("/Robot/SystemStats/BatteryCurrent");
    const energy = log.channel("/Robot/BatteryLogger/Energy");
    const loop = log.channel("/Robot/Scheduler/robotPeriodic");
    const canUtil = log.channel("/Robot/CANivore/BusUtilization");
    const tec = log.channel("/Robot/CANivore/TransmitErrorCounter");

    const bStats = stats(battery);
    const cStats = stats(totalCurrent);
    const uStats = stats(canUtil);

    // Loop time. DogLog's timeEnd writes SECONDS, not milliseconds -- verified against
    // FRC_20260416_210554_TXCMP1_Q19, where p50 is 0.0214 and the max is 0.788, matching the
    // handoff doc's "median 15 to 19 ms ... worst 0.55 and 0.92 s". TimedRobot's period is 20 ms,
    // so anything above 0.020 is an overrun. Reported as a share of loops so it compares across
    // logs of any length.
    const LOOP_PERIOD_SEC = 0.02;
    const overruns = loop.filter(([, v]) => v > LOOP_PERIOD_SEC).length;

    const motorConnected = {};
    for (const { name: key, values } of log.matching(/^\/Robot\/([^/]+)\/MotorConnected$/)) {
        const mech = key.split("/")[2];
        const drops = values.filter(([, v]) => v === false).length;
        motorConnected[mech] = { samples: values.length, disconnectedSamples: drops };
    }

    return {
        name,
        durationSec: +log.durationSec.toFixed(1),
        enabledSec: +enabledSeconds(log).toFixed(1),
        entryCount: log.counts.size,
        parseError: log.error,
        build: {
            gitSha: firstString(log, "/Robot/BuildConstants/GitSHA"),
            gitBranch: firstString(log, "/Robot/BuildConstants/GitBranch"),
            gitDirty: firstString(log, "/Robot/BuildConstants/GitDirty"),
            buildDate: firstString(log, "/Robot/BuildConstants/BuildDate"),
        },
        match: {
            eventName: firstString(log, "DS:eventName"),
            matchNumber: firstString(log, "DS:matchNumber"),
            matchType: firstString(log, "DS:matchType"),
            autonomous: log.channel("DS:autonomous").some(([, v]) => v),
        },
        battery: bStats && { minVolts: +bStats.min.toFixed(2), maxVolts: +bStats.max.toFixed(2) },
        current: cStats && { peakAmps: +cStats.max.toFixed(1), meanAmps: +cStats.mean.toFixed(1) },
        energyWh: energy.length ? +energy[energy.length - 1][1].toFixed(1) : null,
        loop: loop.length
            ? {
                  medianMs: +(percentile(loop, 50) * 1000).toFixed(1),
                  p95Ms: +(percentile(loop, 95) * 1000).toFixed(1),
                  maxMs: +(Math.max(...loop.map(([, v]) => v)) * 1000).toFixed(1),
                  overrunPct: +((100 * overruns) / loop.length).toFixed(1),
              }
            : null,
        can: uStats
            ? {
                  maxUtilPct: +uStats.max.toFixed(1),
                  meanUtilPct: +uStats.mean.toFixed(1),
                  maxTec: tec.length ? Math.max(...tec.map(([, v]) => v)) : null,
              }
            : null,
        motorConnected,
    };
}
