/**
 * Parsing and rewriting the Limelight mounts in Vision.java.
 *
 * This is the second thing in the app that writes to a source file, and a bad write lands in the
 * vision subsystem, so it is tested against the real file as well as the shapes it has to survive.
 */
import { test } from "node:test";
import assert from "node:assert/strict";

// Provenance stamps use the local date, so the test dates are built locally too, not from ISO strings.
import { parseCameras, rewriteCamera, formatDegrees, formatLength, readTarget, WRITABLE } from "../server/lib/vision-config.js";

const MAX_LINE_LENGTH = 100;

/** A compact stand-in for the real file, with every argument shape it uses. */
const SAMPLE = `
class V {
    static class VisionConfig {
        @Getter final String leftLL = "limelight-left";
        @Getter final String turretLL = "limelight-turret";

        @Getter
        final LimelightConfig leftConfig =
                new LimelightConfig(leftLL)
                        .withTranslation(
                                Units.inchesToMeters(-11.103), // forward (behind centre)
                                Units.inchesToMeters(-12.490), // right (left of centre)
                                Units.inchesToMeters(17.058)) // up
                        .withRotation(180, 31.8, 135) // upside down, 31.8 deg up, facing rear-left
                        .setAttached(true);

        @Getter
        final LimelightConfig turretConfig =
                new LimelightConfig(turretLL)
                        .withTranslation(
                                -0.138, // forward at turret zero (unused; see turretCenterToCamera)
                                0.0, // right (unused)
                                Units.inchesToMeters(18.632)) // up (measured on robot, not CAD)
                        .withRotation(0, 30, 0); // yaw unused; live turret angle is used
    }
}
`;

test("parses the live Vision.java: three cameras with hostnames and six values each", () => {
    const cams = parseCameras(readTarget());
    assert.deepEqual(
        cams.map((c) => [c.key, c.ntName]),
        [
            ["backLeftConfig", "limelight-left"],
            ["backRightConfig", "limelight-right"],
            ["turretConfig", "limelight-turret"],
        ]
    );
    for (const c of cams) {
        for (const k of ["forward", "right", "up", "roll", "pitch", "yaw"]) {
            assert.ok(Number.isFinite(c.values[k]), `${c.key}.${k} did not parse`);
        }
        assert.deepEqual(c.writable, WRITABLE);
    }
});

test("reads inches, plain metres, and a semicolon inside a comment", () => {
    const cams = parseCameras(SAMPLE);
    assert.equal(cams.length, 2);
    const [left, turret] = cams;
    assert.equal(left.ntName, "limelight-left");
    assert.ok(Math.abs(left.values.up - 17.058 * 0.0254) < 1e-9);
    assert.equal(left.units.up, "in");
    assert.equal(left.values.yaw, 135);
    assert.equal(turret.values.forward, -0.138);
    assert.equal(turret.units.forward, "m");
    assert.equal(turret.values.pitch, 30);
    assert.equal(turret.measuredOn, null);
});

test("rewrites pitch and height in place and fixes the trailing comment", () => {
    const r = rewriteCamera(SAMPLE, "leftConfig", { pitch: 29.5, up: 0.4382 }, new Date(2026, 8, 7));
    assert.deepEqual(r.changed, ["pitch", "up"]);
    assert.ok(r.source.includes("Units.inchesToMeters(17.252)) // up"), "height rewritten in inches");
    assert.ok(r.source.includes(".withRotation(180, 29.5, 135) // upside down, 29.5 deg up, facing rear-left"), r.source);
    assert.ok(r.source.includes("// Mount measured by the robot app on 2026-09-07\n"), "provenance line present");
    // Untouched arguments and comments survive byte-for-byte.
    assert.ok(r.source.includes("Units.inchesToMeters(-11.103), // forward (behind centre)"));
    assert.ok(r.source.includes("Units.inchesToMeters(-12.490), // right (left of centre)"));
    const again = parseCameras(r.source).find((c) => c.key === "leftConfig");
    assert.ok(Math.abs(again.values.pitch - 29.5) < 1e-9);
    assert.ok(Math.abs(again.values.up - 17.252 * 0.0254) < 1e-9);
    assert.equal(again.measuredOn, "2026-09-07");
});

test("wraps the rotation call when the line would pass 100 columns, as google-java-format does", () => {
    const r = rewriteCamera(SAMPLE, "leftConfig", { roll: 178.3 }, new Date(2026, 8, 7));
    assert.ok(r.source.includes(".withRotation(\n                                178.3, 31.8, 135) // upside down"), r.source);
    for (const line of r.source.split("\n")) assert.ok(line.length <= MAX_LINE_LENGTH, `line too long: ${line}`);
    // And unwraps again when it fits.
    const back = rewriteCamera(r.source, "leftConfig", { roll: 180 }, new Date(2026, 8, 8));
    assert.ok(back.source.includes(".withRotation(180, 31.8, 135) // upside down"));
});

test("the turret's semicolon-and-comment tail stays attached", () => {
    const r = rewriteCamera(SAMPLE, "turretConfig", { pitch: 29.5, up: 0.544 }, new Date(2026, 8, 7));
    assert.ok(r.source.includes(".withRotation(0, 29.5, 0); // yaw unused; live turret angle is used"), r.source);
    assert.ok(r.source.includes("Units.inchesToMeters(21.417)) // up (measured on robot, not CAD)"));
    const again = parseCameras(r.source).find((c) => c.key === "turretConfig");
    assert.ok(Math.abs(again.values.up - 0.544) < 0.0005);
});

test("two writes do not stack provenance lines", () => {
    const first = rewriteCamera(SAMPLE, "leftConfig", { pitch: 29.5 }, new Date(2026, 8, 7));
    const second = rewriteCamera(first.source, "leftConfig", { pitch: 29.6 }, new Date(2026, 8, 8));
    const count = (second.source.match(/Mount measured by the robot app on/g) || []).length;
    assert.equal(count, 1);
    assert.ok(second.source.includes("on 2026-09-08"));
});

test("a write to one camera leaves the other cameras and everything else untouched", () => {
    const src = readTarget();
    const r = rewriteCamera(src, "backRightConfig", { pitch: 31.7 }, new Date(2026, 8, 7));
    const strip = (s) => s.replace(/final LimelightConfig backRightConfig[\s\S]*?\.setAttached\(true\);/, "");
    assert.equal(strip(src), strip(r.source));
    for (const line of r.source.split("\n")) assert.ok(line.length <= MAX_LINE_LENGTH, `line too long: ${line}`);
});

test("refuses what a stationary robot cannot measure", () => {
    assert.throws(() => rewriteCamera(SAMPLE, "leftConfig", { yaw: 140 }), /not something this tool writes/);
    assert.throws(() => rewriteCamera(SAMPLE, "leftConfig", { forward: 0 }), /not something this tool writes/);
    assert.throws(() => rewriteCamera(SAMPLE, "nope", { pitch: 1 }), /No camera "nope"/);
    assert.throws(() => rewriteCamera(SAMPLE, "leftConfig", {}), /Nothing to write/);
});

test("formats numbers the way the file already does", () => {
    assert.equal(formatDegrees(180), "180");
    assert.equal(formatDegrees(31.84), "31.8");
    assert.equal(formatDegrees(-135.0), "-135");
    assert.equal(formatLength(0.4382, "in"), "17.252");
    assert.equal(formatLength(0.4382, "m"), "0.438");
});
