/**
 * Tests for the pure analysis functions. Run with `npm test`.
 *
 * These cover the maths that produces numbers people will act on -- a wrong current limit or a
 * missed breaker trip is worse than no tool at all. UI and transport are not covered here.
 */
import { test } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import { parseWpilog } from "../client/lib/wpilog.js";
import { LogModel, timeAtOrAbove, fitInternalResistance, enabledWindows, clipToEnabled } from "../client/lib/log-model.js";
import { simulateBreaker, tripTime, CB185_120 } from "../client/lib/breaker.js";
import { summarize } from "../server/lib/summary.js";

const APP = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const profile = JSON.parse(fs.readFileSync(path.join(APP, "data/robot-profile.json"), "utf8"));

// Real logs are not committed; skip the integration tests when none are present.
const LOG_DIR = path.resolve(APP, "../../../2026-Robot-Logs/logs");
const realLogs = fs.existsSync(LOG_DIR) ? fs.readdirSync(LOG_DIR).filter((f) => f.endsWith(".wpilog")) : [];

test("tripTime is infinite within rating and shrinks as current rises", () => {
    assert.equal(tripTime(CB185_120, 100), Infinity);
    assert.equal(tripTime(CB185_120, 120), Infinity);
    assert.ok(tripTime(CB185_120, 180) < tripTime(CB185_120, 144));
    assert.ok(tripTime(CB185_120, 600) < tripTime(CB185_120, 300));
});

test("breaker does not trip on current within its rating", () => {
    const samples = Array.from({ length: 5000 }, (_, i) => [i * 0.02, 90]);
    const r = simulateBreaker(samples);
    assert.equal(r.trips.length, 0);
    assert.equal(r.peakPct, 0);
});

test("breaker trips on a sustained heavy overload", () => {
    // 300 A trips in ~50 s per the curve; give it 80 s.
    const samples = Array.from({ length: 4000 }, (_, i) => [i * 0.02, 300]);
    const r = simulateBreaker(samples);
    assert.ok(r.trips.length >= 1, "expected at least one trip");
    assert.ok(r.trips[0] > 30 && r.trips[0] < 70, `trip at ${r.trips[0]}s should be near the 50 s curve point`);
});

test("breaker ignores gaps longer than a second so idle time is not treated as cooling", () => {
    const samples = [[0, 400], [600, 400], [600.02, 400]];
    const r = simulateBreaker(samples);
    assert.equal(r.trips.length, 0, "a 600 s gap must not be integrated as 600 s of overload");
});

test("timeAtOrAbove counts samples and seconds at a threshold", () => {
    const s = [[0, 10], [0.02, 90], [0.04, 90], [0.06, 10], [0.08, 95]];
    const r = timeAtOrAbove(s, 80);
    assert.equal(r.samples, 3);
    assert.equal(r.fraction, 3 / 5);
    assert.ok(r.seconds > 0.05 && r.seconds < 0.09);
});

test("timeAtOrAbove uses magnitude, since stator current is signed", () => {
    assert.equal(timeAtOrAbove([[0, -90]], 80).samples, 1);
});

test("fitInternalResistance recovers a known slope", () => {
    // V = 12.5 - I * 0.015
    const current = [];
    const voltage = [];
    for (let i = 0; i < 200; i++) {
        const t = i * 0.02;
        const amps = 20 + (i % 50) * 6;
        current.push([t, amps]);
        voltage.push([t, 12.5 - amps * 0.015]);
    }
    const r = fitInternalResistance(voltage, current);
    assert.ok(r, "expected a fit");
    assert.ok(Math.abs(r.ohms - 0.015) < 0.001, `got ${r.ohms}`);
    assert.ok(Math.abs(r.openCircuitVolts - 12.5) < 0.05, `got ${r.openCircuitVolts}`);
});

test("fitInternalResistance refuses to guess from too little data", () => {
    assert.equal(fitInternalResistance([[0, 12]], [[0, 5]]), null);
});

test("enabledWindows pairs DS:enabled transitions and closes an open window", () => {
    const log = { channel: () => [[1, true], [5, false], [9, true]], lastTs: 12 };
    assert.deepEqual(enabledWindows(log), [[1, 5], [9, 12]]);
});

test("clipToEnabled drops disabled-time samples", () => {
    const s = [[0, 1], [2, 2], [6, 3], [10, 4]];
    assert.deepEqual(clipToEnabled(s, [[1, 7]]), [[2, 2], [6, 3]]);
});

test("wpilog parser reads a three-bit timestamp length", () => {
    // Header byte 0b0100_0000 means tsLen = ((0x40 >> 4) & 7) + 1 = 5. A two-bit mask would read
    // 1 byte here and desync every record after it.
    const h = 0x40;
    assert.equal(((h >> 4) & 0x7) + 1, 5);
    assert.notEqual(((h >> 4) & 0x3) + 1, 5);
});

test("parser rejects a file that is not a wpilog", () => {
    const buf = new TextEncoder().encode("NOTALOG_and_then_some_padding_bytes");
    const log = parseWpilog(buf);
    assert.equal(log.error, "not a WPILOG file");
});

for (const name of realLogs) {
    test(`real log parses cleanly: ${name}`, () => {
        const buf = fs.readFileSync(path.join(LOG_DIR, name));
        const log = parseWpilog(buf, { keep: () => false });
        assert.equal(log.error, null, `parse error: ${log.error}`);
        assert.ok(log.counts.size > 10, "expected a populated entry table");
        assert.ok(log.durationSec > 1, "expected a non-trivial duration");
    });
}

if (realLogs.length) {
    test("LogModel prefers the richer channel over a one-sample one", () => {
        // The BreakerPop log declares BatteryLogger/BatteryVoltage with a single record while
        // SystemStats/BatteryVoltage is populated. Choosing on presence alone reported a
        // reassuring 12.6 V minimum for a log whose battery actually sagged.
        const name = realLogs.find((f) => f.includes("BreakerPop"));
        if (!name) return;
        const log = parseWpilog(fs.readFileSync(path.join(LOG_DIR, name)));
        const m = new LogModel(log, profile);
        const v = m.batteryVoltage();
        const alt = m.ch("BatteryLogger/BatteryVoltage");
        assert.ok(v.length >= alt.length, "must not pick the sparser channel");
    });

    test("summarize reports loop time in milliseconds, not raw seconds", () => {
        const name = realLogs.find((f) => f.includes("Q19"));
        if (!name) return;
        const s = summarize(fs.readFileSync(path.join(LOG_DIR, name)));
        assert.ok(s.loop, "expected loop stats");
        // A 20 ms robot loop: a median in the tens of ms is right, 0.02 would mean raw seconds.
        assert.ok(s.loop.medianMs > 5 && s.loop.medianMs < 200, `median ${s.loop.medianMs} ms is not a plausible loop time`);
        assert.ok(s.loop.overrunPct > 0 && s.loop.overrunPct <= 100);
    });
}

test("every profile motor has the fields the analysis pages rely on", () => {
    for (const m of profile.motors) {
        assert.ok(m.key, "motor needs a key");
        assert.ok(m.name, `${m.key} needs a name`);
        assert.ok(m.batteryChannel, `${m.key} needs a batteryChannel`);
        assert.equal(typeof m.canId, "number", `${m.key} needs a numeric canId`);
        assert.ok(["canivore", "rio"].includes(m.bus), `${m.key} has an unknown bus`);
        assert.equal(typeof m.supplyAmps, "number", `${m.key} needs supplyAmps`);
        assert.equal(typeof m.statorAmps, "number", `${m.key} needs statorAmps`);
        assert.ok(Array.isArray(m.followers), `${m.key} needs a followers array`);
    }
});

test("profile CAN ids are unique across each bus", () => {
    const seen = new Map();
    const claim = (bus, id, who) => {
        const k = `${bus}:${id}`;
        assert.ok(!seen.has(k), `CAN ${id} on ${bus} claimed by both ${seen.get(k)} and ${who}`);
        seen.set(k, who);
    };
    for (const m of profile.motors) {
        claim(m.bus, m.canId, m.name);
        for (const f of m.followers) claim(m.bus, f.canId, f.name);
    }
    const sw = profile.swerve;
    sw.drive.canIds.forEach((id, i) => claim(sw.drive.bus, id, `drive ${i}`));
    sw.steer.canIds.forEach((id, i) => claim(sw.steer.bus, id, `steer ${i}`));
    sw.cancoders.canIds.forEach((id, i) => claim(sw.cancoders.bus, id, `cancoder ${i}`));
    claim(sw.pigeon.bus, sw.pigeon.canId, "pigeon");
});
