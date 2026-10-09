/**
 * Turret slip analyses, against a synthetic log with faults of known size injected.
 *
 * The robot this app targets has no turret log to test against yet, and a slip detector that
 * silently never fires is worse than none. The fixture injects a mechanical slip the controller
 * cannot see, an encoder jump it can, and an unwrap that must not be mistaken for either.
 */
import { test } from "node:test";
import assert from "node:assert/strict";

import { parseWpilog } from "../client/lib/wpilog.js";
import { LogModel } from "../client/lib/log-model.js";
import {
    zeroErrorAnalysis, trackingAnalysis, motionAnomalies, unwrapWindows,
    slipCorrection, limelightCorrections, turretCameraRejection, extraTurretChannels, valueAt,
} from "../client/lib/analyze-turret.js";
import { LogWriter } from "./helpers/make-log.mjs";
import { buildTurretLog, BASELINE_DEG, SLIP_DEG, SLIP_T, JUMP_T, UNWRAP_FROM, UNWRAP_TO, TOLERANCE_DEG } from "./helpers/turret-fixture.mjs";

const log = parseWpilog(buildTurretLog());
const model = new LogModel(log, null);

test("the fixture parses and covers the whole run", () => {
    assert.equal(log.error, null);
    assert.ok(model.enabledSec > 50, `enabled for ${model.enabledSec}s`);
});

test("finds the mechanical slip, at the right time and the right size", () => {
    const zero = zeroErrorAnalysis(model);
    assert.ok(zero, "expected a zero-error analysis");
    assert.equal(zero.verdict, "slipped");
    assert.equal(zero.steps.length, 1, `expected exactly one step, got ${zero.steps.length}`);

    const step = zero.steps[0];
    assert.ok(Math.abs(step.t - SLIP_T) < 0.5, `step at ${step.t}s, expected ~${SLIP_T}s`);
    assert.ok(Math.abs(step.jump - SLIP_DEG) < 1, `step of ${step.jump}°, expected ~${SLIP_DEG}°`);
});

test("reports the baseline zero error separately from the slip", () => {
    const zero = zeroErrorAnalysis(model);
    assert.ok(Math.abs(zero.baselineDeg - BASELINE_DEG) < 0.5, `baseline ${zero.baselineDeg}, expected ~${BASELINE_DEG}`);
    assert.ok(Math.abs(zero.totalDriftDeg - SLIP_DEG) < 1, `total drift ${zero.totalDriftDeg}, expected ~${SLIP_DEG}`);
});

/** A log whose zero error is wrong but perfectly steady -- a zeroing mistake, not a slip. */
function buildSteadyOffsetLog(offsetDeg = -6.4) {
    const w = new LogWriter();
    w.put("DS:enabled", "boolean", 0, false);
    w.put("DS:enabled", "boolean", 5, true);
    w.put("DS:enabled", "boolean", 60, false);
    w.series("/Robot/Turret/PositionDegrees", "double", { from: 0, to: 60, hz: 50, fn: (t) => 10 * Math.sin(t / 9) });
    w.series("/Robot/Vision/TurretLL/HeadingErrorDeg", "double", {
        from: 5, to: 60, hz: 10, fn: (t) => offsetDeg + 0.2 * Math.sin(t * 3),
    });
    return w.buffer();
}

test("a steady offset is called a bad zero, not a slip", () => {
    // Without this the page cries wolf every time someone forgets to re-zero, and the crew stops
    // believing it -- which is worse than not having the check.
    const steady = new LogModel(parseWpilog(buildSteadyOffsetLog()), null);
    const zero = zeroErrorAnalysis(steady);
    assert.equal(zero.verdict, "zero-off");
    assert.equal(zero.steps.length, 0);
    assert.ok(Math.abs(zero.baselineDeg - -6.4) < 0.5, `baseline ${zero.baselineDeg}`);
    assert.ok(Math.abs(zero.totalDriftDeg) < 1, `should not drift, got ${zero.totalDriftDeg}`);
});

test("a turret that is genuinely fine reads as fine", () => {
    const clean = new LogModel(parseWpilog(buildSteadyOffsetLog(0.3)), null);
    assert.equal(zeroErrorAnalysis(clean).verdict, "ok");
});

test("the encoder jump is found and the unwrap is not", () => {
    const jumps = motionAnomalies(model);
    assert.ok(jumps.length >= 1, "expected the encoder jump to be flagged");
    assert.ok(jumps.some((j) => Math.abs(j.t - JUMP_T) < 0.3), `no jump near ${JUMP_T}s: ${jumps.map((j) => j.t.toFixed(1))}`);

    const duringUnwrap = jumps.filter((j) => j.t >= UNWRAP_FROM - 0.5 && j.t <= UNWRAP_TO + 0.5);
    assert.equal(duringUnwrap.length, 0, `unwrap slew was misreported as ${duringUnwrap.length} fault(s)`);
});

test("unwrap windows are detected with their padding", () => {
    const w = unwrapWindows(model);
    assert.equal(w.length, 1);
    assert.ok(w[0][0] <= UNWRAP_FROM && w[0][1] >= UNWRAP_TO, `window ${w[0]} does not cover ${UNWRAP_FROM}-${UNWRAP_TO}`);
});

test("separates what the controller fixed from what it never saw", () => {
    const zero = zeroErrorAnalysis(model);
    const jumps = motionAnomalies(model);
    const c = slipCorrection(model, zero, jumps, { toleranceDeg: TOLERANCE_DEG });

    const mech = c.events.find((e) => Math.abs(e.t - SLIP_T) < 0.6);
    const enc = c.events.find((e) => Math.abs(e.t - JUMP_T) < 0.6);
    assert.ok(mech, "mechanical slip missing from the correction table");
    assert.ok(enc, "encoder jump missing from the correction table");

    // The mechanism moved and the encoder did not: the camera is still wrong afterwards.
    assert.ok(Math.abs(mech.residualDeg) > 3, `mechanical slip left only ${mech.residualDeg}° wrong`);

    // The encoder jumped and the controller drove it out: real recovery, and the camera is fine.
    assert.ok(enc.recoveredDeg > 3, `controller only clawed back ${enc.recoveredDeg}°`);
    assert.ok(enc.recoverySec !== null && enc.recoverySec < 2, `recovery took ${enc.recoverySec}s`);
    assert.ok(Math.abs(enc.residualDeg ?? 0) < 2, `encoder jump should leave no lasting aim error, left ${enc.residualDeg}°`);
});

test("net zero shift is the number that matters for aim", () => {
    const zero = zeroErrorAnalysis(model);
    const c = slipCorrection(model, zero, motionAnomalies(model), { toleranceDeg: TOLERANCE_DEG });
    assert.ok(Math.abs(c.netZeroShiftDeg - SLIP_DEG) < 1.5, `net shift ${c.netZeroShiftDeg}, expected ~${SLIP_DEG}`);
});

test("counts Limelight pose corrections per camera", () => {
    const ll = limelightCorrections(model);
    const turret = ll.perCamera.find((c) => c.camera === "TurretLL");
    const back = ll.perCamera.find((c) => c.camera === "BackLeftLL");
    assert.ok(turret && turret.corrections > 0, "expected turret camera corrections");
    assert.ok(back && back.corrections > 0, "expected rear camera corrections");
    assert.ok(turret.acceptRate > 0 && turret.acceptRate <= 1);
});

test("flags the window where the slip made the turret camera untrustworthy", () => {
    const r = turretCameraRejection(model);
    // Baseline -1.2 plus a 7.5 slip clears the 5 degree threshold, so everything after the slip.
    assert.ok(r.totalSec > 20, `only ${r.totalSec}s over threshold`);
    assert.ok(r.windows.every(([s]) => s >= SLIP_T - 1), "rejection should start at the slip, not before");
});

test("tracking finds the not-following stretch without counting the unwrap", () => {
    const tr = trackingAnalysis(model, { toleranceDeg: TOLERANCE_DEG });
    assert.ok(tr.withinTolerancePct > 50, `only ${tr.withinTolerancePct}% within tolerance`);
    assert.ok(tr.maxAbsErrDeg < 100, `max error ${tr.maxAbsErrDeg}° suggests the unwrap leaked in`);
});

test("valueAt samples a change-only series correctly", () => {
    const s = [[0, "a"], [5, "b"], [9, "c"]];
    assert.equal(valueAt(s, -1), null);
    assert.equal(valueAt(s, 0), "a");
    assert.equal(valueAt(s, 4.9), "a");
    assert.equal(valueAt(s, 5), "b");
    assert.equal(valueAt(s, 100), "c");
});

test("unknown Turret/* channels are surfaced rather than ignored", () => {
    // This is what lets checks added to the robot code show up here without anyone editing the
    // page first. Simulate two new keys of the kind a slip check would publish.
    assert.equal(extraTurretChannels(model).length, 0, "the base fixture has no unknown keys");

    const w = new LogWriter();
    w.put("DS:enabled", "boolean", 0, true);
    w.series("/Robot/Turret/PositionDegrees", "double", { from: 0, to: 10, hz: 50, fn: () => 0 });
    w.series("/Robot/Turret/SlipDetected", "boolean", { from: 0, to: 10, hz: 10, fn: (t) => t > 6 });
    w.series("/Robot/Turret/EncoderDisagreementDeg", "double", { from: 0, to: 10, hz: 50, fn: (t) => t * 0.5 }, );
    const withNew = new LogModel(parseWpilog(w.buffer()), null);

    const extras = extraTurretChannels(withNew);
    const keys = extras.map((e) => e.key).sort();
    assert.deepEqual(keys, ["EncoderDisagreementDeg", "SlipDetected"]);
    assert.equal(extras.find((e) => e.key === "SlipDetected").type, "boolean");
    assert.equal(extras.find((e) => e.key === "EncoderDisagreementDeg").type, "double");
});
