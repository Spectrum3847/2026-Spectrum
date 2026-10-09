/**
 * Config parsing and rewriting for the swerve alignment page.
 *
 * These are the standalone tool's `--self-test` cases, turned into real tests now that the code
 * lives in the robot app. They matter more than most: this is the only thing in the app that
 * writes to a source file, and a bad write lands in `OM2026.java`.
 */
import { test } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";

import {
    MODULE_KEYS,
    TARGET_CONFIG,
    parseConfig,
    rewriteConfig,
    renderCall,
    formatOffset,
    targetPath,
} from "../server/lib/swerve-config.js";

const MAX_LINE_LENGTH = 100;
const TARGET = { fl: -0.480712890625, fr: 0.35986328125, bl: -0.138916015625, br: 0.22216796875 };

const SINGLE_LINE = "class T {\n    T() {\n        swerve.configEncoderOffsets(0.1, -0.2, 0.3, -0.4);\n    }\n}\n";

const ARITHMETIC =
    "class T {\n    T() {\n        swerve.configEncoderOffsets(\n" +
    "                -0.23046875 - 0.25,\n                0.10986328125 + 0.25,\n" +
    "                -0.263916015625 + 0.125,\n                0.34716796875 - 0.125);\n    }\n}\n";

const shapes = [
    { name: "live config file", source: () => fs.readFileSync(targetPath(), "utf8") },
    { name: "single-line call", source: () => SINGLE_LINE },
    { name: "arithmetic arguments", source: () => ARITHMETIC },
];

for (const shape of shapes) {
    test(`parses every offset: ${shape.name}`, () => {
        const parsed = parseConfig(shape.source());
        for (const key of MODULE_KEYS) {
            assert.ok(Number.isFinite(parsed.offsets[key]), `${key} did not parse to a number`);
        }
    });

    test(`round-trips through a write: ${shape.name}`, () => {
        const written = rewriteConfig(shape.source(), TARGET, new Date("2026-09-05"));
        const reparsed = parseConfig(written.source);
        for (const key of MODULE_KEYS) {
            assert.ok(
                Math.abs(reparsed.offsets[key] - TARGET[key]) < 1e-12,
                `${key} round-tripped to ${reparsed.offsets[key]}, expected ${TARGET[key]}`
            );
        }
    });

    test(`two writes do not stack provenance comments: ${shape.name}`, () => {
        const first = rewriteConfig(shape.source(), TARGET, new Date("2026-09-05"));
        const second = rewriteConfig(first.source, TARGET, new Date("2026-09-06"));
        const comments = (second.source.match(/Aligned by the Spectrum robot app on/g) || []).length;
        assert.equal(comments, 1, `found ${comments} provenance comments after two writes`);
    });

    test(`stays inside google-java-format's line limit: ${shape.name}`, () => {
        const written = rewriteConfig(shape.source(), TARGET, new Date("2026-09-05"));
        for (const line of written.source.split("\n")) {
            assert.ok(line.length <= MAX_LINE_LENGTH, `emitted a ${line.length}-column line: ${line.trim()}`);
        }
    });
}

test("arithmetic arguments are evaluated, not parseFloat'd", () => {
    const parsed = parseConfig(ARITHMETIC);
    // -0.23046875 - 0.25, not -0.23046875.
    assert.ok(Math.abs(parsed.offsets.fl - -0.48046875) < 1e-12, `got ${parsed.offsets.fl}`);
    assert.ok(Math.abs(parsed.offsets.fr - 0.35986328125) < 1e-12, `got ${parsed.offsets.fr}`);
});

test("offsets round to the CANcoder's 1/4096 resolution", () => {
    // A value a hair off a 1/4096 step must snap to it, and never print in exponent form.
    const snapped = formatOffset(0.25 + 1e-9);
    assert.equal(snapped, "0.25");
    assert.ok(!/e/i.test(formatOffset(0.00000001)), "must not emit exponent notation");
    assert.equal(formatOffset(-0.0000001), "0", "must never emit -0");
});

test("renderCall wraps rather than exceeding the line limit", () => {
    const wide = { fl: -0.480712890625, fr: -0.480712890625, bl: -0.480712890625, br: -0.480712890625 };
    for (const indent of ["        ", " ".repeat(40)]) {
        for (const line of renderCall(wide, indent).split("\n")) {
            assert.ok(line.length <= MAX_LINE_LENGTH, `${line.length} columns at indent ${indent.length}`);
        }
    }
});

test("a rewrite touches only the offsets call", () => {
    const source = fs.readFileSync(targetPath(), "utf8");
    const written = rewriteConfig(source, TARGET, new Date("2026-09-05"));
    // Remove the call and its provenance comment, then compare what is left. Blank lines are
    // dropped because the call itself changes line count -- a five-line wrapped call can come
    // back as one line -- and that is not a change to the rest of the file.
    const strip = (s) =>
        s
            .replace(/^\s*\/\/ Aligned by the Spectrum robot app on .*\n/m, "")
            .replace(/swerve\.configEncoderOffsets\([\s\S]*?\);/, "")
            .split("\n")
            .filter((line) => line.trim())
            .join("\n");
    assert.equal(strip(written.source), strip(source), `${TARGET_CONFIG} changed outside the offsets call`);
});

test("refuses a file with no offsets call", () => {
    assert.throws(() => parseConfig("class T {}\n"), /No swerve\.configEncoderOffsets/);
});

test("refuses a file with two offsets calls", () => {
    const two = SINGLE_LINE + SINGLE_LINE;
    assert.throws(() => parseConfig(two), /more than one/i);
});
