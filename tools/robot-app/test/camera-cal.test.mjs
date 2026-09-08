/**
 * Camera calibration maths, checked against frames captured from the robot's three Limelights on
 * 2026-09-07 with the robot parked on the practice field.
 *
 * The expected numbers were established independently that day: the tag solves put the back-right
 * camera's optical axis 32.2 and 31.3 deg above horizontal (tags 21 and 24), its accelerometer said
 * 31.8, the back-left accelerometer said 28.7 and the turret's 28.5. Those are the values in the
 * Vision.java comments. If this test starts disagreeing with them, the conventions changed.
 */
import { test } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

import {
    mountFromTagSolve,
    mountFromAccel,
    tagHeightsFromLayout,
    summarizeMount,
    proposeWrites,
    summarizeFrames,
    scoreMetrics,
    pickBest,
    wrapDeg,
    circularMeanDeg,
} from "../client/lib/camera-cal.js";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const fixture = (name) => JSON.parse(fs.readFileSync(path.join(HERE, "fixtures", name), "utf8"));
const layout = JSON.parse(fs.readFileSync(path.join(HERE, "..", "data", "apriltag-2026-rebuilt-welded.json"), "utf8"));
const heights = tagHeightsFromLayout(layout);

const right = fixture("limelight-right-results.json");
const left = fixture("limelight-left-results.json");
const turret = fixture("limelight-turret-results.json");

const near = (actual, expected, tol, msg) => assert.ok(Math.abs(actual - expected) <= tol, `${msg}: ${actual} vs ${expected}`);

test("tag solve reproduces the hand-computed pitches for tags 21 and 24", () => {
    const byId = Object.fromEntries(right.Fiducial.map((f) => [f.fID, f]));
    const t21 = mountFromTagSolve(byId[21].t6c_ts, heights.get(21));
    const t24 = mountFromTagSolve(byId[24].t6c_ts, heights.get(24));
    near(t21.pitchDeg, 32.2, 0.15, "tag 21 pitch");
    // The fixture is a different frame from the one quoted in Vision.java, hence the wider band.
    near(t24.pitchDeg, 31.3, 0.3, "tag 24 pitch");
    // The camera is upside down: roll near 180, off by a degree or two.
    near(Math.abs(t21.rollDeg), 180, 3, "tag 21 roll");
    // Height: the tags hang at 1.124 m and the camera is configured at 0.443 m up.
    near(t21.heightM, 0.443, 0.012, "tag 21 height");
    near(t24.heightM, 0.443, 0.012, "tag 24 height");
});

test("accelerometer reproduces the pitches read off the cameras that day", () => {
    near(mountFromAccel(right.imu.data.slice(7, 10)).pitchDeg, 31.8, 0.4, "right accel pitch");
    near(mountFromAccel(left.imu.data.slice(7, 10)).pitchDeg, 28.7, 0.4, "left accel pitch");
    near(mountFromAccel(turret.imu.data.slice(7, 10)).pitchDeg, 28.5, 0.4, "turret accel pitch");
    // Upside-down cameras read roll near 180; the upright turret reads near 0.
    near(Math.abs(mountFromAccel(right.imu.data.slice(7, 10)).rollDeg), 180, 5, "right accel roll");
    near(Math.abs(mountFromAccel(turret.imu.data.slice(7, 10)).rollDeg), 0, 5, "turret accel roll");
});

test("an accelerometer alone gives no height, tags do", () => {
    const accelOnly = summarizeMount([left], heights, { minTagFrames: 1 });
    assert.equal(accelOnly.tagSummary, null);
    assert.equal(accelOnly.proposed.source, "accel");
    assert.equal(accelOnly.proposed.upM, null);

    const withTags = summarizeMount([right, right, right], heights, { minTagFrames: 3 });
    assert.equal(withTags.proposed.source, "tags");
    assert.equal(withTags.tags.length, 2);
    near(withTags.proposed.pitchDeg, 31.7, 0.5, "combined pitch");
    near(withTags.proposed.upM, 0.44, 0.01, "combined height");
    near(withTags.agreementDeg, 0.1, 0.5, "tag vs accel agreement");
});

test("proposals flag only differences worth writing", () => {
    const rows = proposeWrites({ roll: 180, pitch: 31.8, up: 0.4434 }, { pitchDeg: 31.9, rollDeg: -178.4, upM: 0.443 });
    const byKey = Object.fromEntries(rows.map((r) => [r.key, r]));
    assert.equal(byKey.pitch.significant, false);
    assert.equal(byKey.up.significant, false);
    // Roll compares around the wrap: -178.4 is 1.6 deg from 180, not 358.
    near(byKey.roll.delta, 1.6, 0.01, "roll delta wraps");
    assert.equal(byKey.roll.significant, true);
});

test("angle helpers wrap and average across the +/-180 seam", () => {
    assert.equal(wrapDeg(190), -170);
    assert.equal(wrapDeg(-180), 180);
    near(Math.abs(circularMeanDeg([179, -179])), 180, 1e-9, "mean of 179 and -179");
});

test("frame metrics: a steady, unambiguous sample scores above a noisy one", () => {
    const steady = Array.from({ length: 10 }, () => structuredClone(right));
    const noisy = steady.map((f, i) => {
        const g = structuredClone(f);
        for (const t of g.Fiducial) {
            t.ambig += 0.4;
            t.pts = t.pts.map(([x, y]) => [x + (i % 2 ? 3 : -3), y]);
        }
        g.botpose[0] += i % 2 ? 0.05 : -0.05;
        return g;
    });
    const dropouts = steady.map((f, i) => (i % 2 ? { ...structuredClone(f), Fiducial: [], botpose_tagcount: 0 } : structuredClone(f)));

    const ms = summarizeFrames(steady);
    const mn = summarizeFrames(noisy);
    const md = summarizeFrames(dropouts);
    assert.equal(ms.detectRate, 1);
    assert.equal(md.detectRate, 0.5);
    near(ms.cornerJitterPx, 0, 1e-9, "identical frames have no corner jitter");
    assert.ok(mn.cornerJitterPx > 2, `noisy corners jitter: ${mn.cornerJitterPx}`);
    assert.ok(scoreMetrics(ms, 400) > scoreMetrics(mn, 400));
    assert.ok(scoreMetrics(ms, 400) > scoreMetrics(md, 400));
    // Equal quality: the shorter exposure wins.
    assert.ok(scoreMetrics(ms, 300) > scoreMetrics(ms, 800));

    const best = pickBest([
        { value: 300, metrics: md },
        { value: 400, metrics: ms },
        { value: 800, metrics: mn },
    ]);
    assert.equal(best.value, 400);
    assert.equal(pickBest([{ value: 1, metrics: summarizeFrames([]) }]), null);
});
