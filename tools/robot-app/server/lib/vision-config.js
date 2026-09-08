/**
 * Reads and rewrites the Limelight mount poses in Vision.java.
 *
 * The robot code is the source of truth for where each camera sits: Vision.sendCameraSettings()
 * pushes the six numbers in each LimelightConfig to the camera every couple of seconds, so a
 * value typed into a camera's web UI survives about that long. Calibrating a mount therefore means
 * editing the Java, and this is the only thing in the app that does so.
 *
 * It is deliberately narrow. It understands exactly one shape of code --
 *
 *     final LimelightConfig backLeftConfig =
 *             new LimelightConfig(backLeftLL)
 *                     .withTranslation(
 *                             Units.inchesToMeters(-11.103), // forward
 *                             Units.inchesToMeters(-12.490), // right
 *                             Units.inchesToMeters(17.058)) // up
 *                     .withRotation(180, 31.8, 135) // upside down ...
 *                     .setAttached(true);
 *
 * -- and rewrites individual numeric literals in place, keeping every comment and every other
 * argument byte-for-byte. Only the values a stationary robot can actually measure are writable:
 * roll, pitch and height. Forward, right and yaw need a surveyed robot position and stay CAD.
 *
 * Like swerve-config.js, the output is shaped so googleJavaFormat().aosp() leaves it alone: a
 * .withRotation(...) line that would run past 100 columns with its trailing comment is wrapped the
 * way the formatter wraps it, which is why the right camera's call is already on two lines.
 */
import fs from "node:fs";
import path from "node:path";
import { execFile } from "node:child_process";
import { APP_ROOT, config } from "./config.js";

const REPO_ROOT = path.resolve(APP_ROOT, "..", "..");

/** The file holding VisionConfig. Override with `vision.targetConfig` in config.local.json. */
export const TARGET_CONFIG = config.vision?.targetConfig || "src/main/java/frc/robot/subsystems/vision/Vision.java";

const PROVENANCE_PREFIX = "// Mount measured by the robot app on ";
const MAX_LINE_LENGTH = 100;
const INCH = 0.0254;

/** The argument names, in the order they appear in each call. */
const TRANSLATION_ARGS = ["forward", "right", "up"];
const ROTATION_ARGS = ["roll", "pitch", "yaw"];

/** What a stationary robot on a flat floor can measure, and therefore what this tool will write. */
export const WRITABLE = ["roll", "pitch", "up"];

// ---------------------------------------------------------------------------
// Lexical helpers
// ---------------------------------------------------------------------------

/** Index of the `)` closing the `(` at `open`, ignoring parens inside comments and strings. */
function closingParen(source, open) {
    let depth = 0;
    for (let i = open; i < source.length; i++) {
        const c = source[i];
        if (c === "/" && source[i + 1] === "/") {
            i = source.indexOf("\n", i);
            if (i === -1) break;
            continue;
        }
        if (c === "/" && source[i + 1] === "*") {
            i = source.indexOf("*/", i) + 1;
            continue;
        }
        if (c === '"') {
            i = source.indexOf('"', i + 1);
            continue;
        }
        if (c === "(") depth++;
        else if (c === ")" && --depth === 0) return i;
    }
    throw new Error(`Unbalanced parentheses in ${TARGET_CONFIG}.`);
}

/** Splits argument text on top-level commas; a comment stays in whichever piece it falls in. */
function splitArguments(text) {
    const args = [];
    let depth = 0;
    let start = 0;
    for (let i = 0; i < text.length; i++) {
        const c = text[i];
        if (c === "/" && text[i + 1] === "/") {
            i = text.indexOf("\n", i);
            if (i === -1) break;
            continue;
        }
        if (c === "(") depth++;
        else if (c === ")") depth--;
        else if (c === "," && depth === 0) {
            args.push({ text: text.slice(start, i), start, end: i });
            start = i + 1;
        }
    }
    args.push({ text: text.slice(start), start, end: text.length });
    return args;
}

/** The text with its comments blanked to spaces, so offsets into it still line up. */
function withoutComments(text) {
    return text
        .replace(/\/\/[^\n]*/g, (m) => " ".repeat(m.length))
        .replace(/\/\*[\s\S]*?\*\//g, (m) => " ".repeat(m.length));
}

function escapeRe(s) {
    return s.replace(/[.*+?^${}()|[\]\\/]/g, "\\$&");
}

function evalNumeric(expr) {
    if (!/^[-+*/().\d\s]+$/.test(expr)) throw new Error(`Refusing to evaluate "${expr}".`);
    // Safe: the charset check admits only numeric arithmetic.
    const value = Function(`"use strict"; return (${expr});`)();
    if (typeof value !== "number" || !Number.isFinite(value)) throw new Error(`"${expr}" is not a finite number.`);
    return value;
}

/**
 * Evaluates one numeric argument. Accepts plain arithmetic and the `Units.inchesToMeters(...)`
 * wrapper the CAD numbers are written in. Anything else is refused rather than guessed at.
 *
 * @returns {{value: number, unit: string, literalStart: number, literalEnd: number}} the value in
 *     the call's own units (metres or degrees), the unit the literal itself is written in ("in",
 *     "m" or "deg"), and where that literal sits within the argument text
 */
function readArgument(argText, unitIfPlain) {
    const clean = withoutComments(argText);
    const inches = /Units\.inchesToMeters\(\s*([-+*/().\d\s]+?)\s*\)/.exec(clean);
    if (inches) {
        const inner = inches[1];
        const start = inches.index + inches[0].indexOf(inner);
        return { value: evalNumeric(inner) * INCH, unit: "in", literalStart: start, literalEnd: start + inner.length };
    }
    const trimmed = clean.trim();
    if (!/^[-+*/().\d\s]+$/.test(trimmed)) {
        throw new Error(
            `Cannot read the argument "${trimmed}" in ${TARGET_CONFIG}. This tool understands plain numbers, ` +
                `+ - * / arithmetic, and Units.inchesToMeters(...). Simplify it by hand and try again.`
        );
    }
    const start = clean.indexOf(trimmed);
    return { value: evalNumeric(trimmed), unit: unitIfPlain, literalStart: start, literalEnd: start + trimmed.length };
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

/** One call in a chain: its bounds in `source` and its parsed arguments. */
function readCall(source, from, to, method, names, plainUnit) {
    const idx = source.indexOf(`.${method}(`, from);
    if (idx === -1 || idx > to) {
        throw new Error(`No .${method}(...) in the LimelightConfig chain in ${TARGET_CONFIG}.`);
    }
    const open = idx + method.length + 1;
    const close = closingParen(source, open);
    const pieces = splitArguments(source.slice(open + 1, close));
    if (pieces.length !== names.length) {
        throw new Error(`Expected ${names.length} arguments to .${method}(...) in ${TARGET_CONFIG}, found ${pieces.length}.`);
    }
    const args = pieces.map((p, i) => {
        const read = readArgument(p.text, plainUnit);
        const base = open + 1 + p.start;
        return {
            name: names[i],
            text: p.text,
            value: read.value,
            unit: read.unit,
            literalStart: base + read.literalStart,
            literalEnd: base + read.literalEnd,
        };
    });
    return { callStart: idx, open, close, args };
}

/**
 * Every LimelightConfig in the file.
 *
 * @param {string} source Vision.java
 * @returns {object[]} one entry per camera: `key` (the Java field), `ntName` (the camera's
 *     hostname), the six `values` (metres and degrees), the `units` each literal is written in,
 *     when the app last measured it, and private offsets used by the rewriter
 */
export function parseCameras(source) {
    const cameras = [];
    const re = /final\s+LimelightConfig\s+(\w+)\s*=\s*new\s+LimelightConfig\((\w+)\)/g;
    for (const m of source.matchAll(re)) {
        const key = m[1];
        const nameIdent = m[2];
        const nameMatch = new RegExp(`String\\s+${nameIdent}\\s*=\\s*"([^"]+)"`).exec(source);
        const chainStart = m.index;

        // The statement ends at the first semicolon that leaves the parentheses balanced.
        let end = chainStart;
        for (;;) {
            end = source.indexOf(";", end + 1);
            if (end === -1) throw new Error(`Unterminated LimelightConfig ${key} in ${TARGET_CONFIG}.`);
            const between = withoutComments(source.slice(chainStart, end));
            const depth = (between.match(/\(/g) || []).length - (between.match(/\)/g) || []).length;
            if (depth === 0) break;
        }

        const translation = readCall(source, chainStart, end, "withTranslation", TRANSLATION_ARGS, "m");
        const rotation = readCall(source, chainStart, end, "withRotation", ROTATION_ARGS, "deg");
        const provenance = new RegExp(`${escapeRe(PROVENANCE_PREFIX)}(\\S+)`).exec(source.slice(chainStart, end));

        const values = {};
        const units = {};
        for (const a of [...translation.args, ...rotation.args]) {
            values[a.name] = a.value;
            units[a.name] = a.unit;
        }
        cameras.push({
            key,
            ntName: nameMatch ? nameMatch[1] : nameIdent,
            values,
            units,
            writable: WRITABLE,
            measuredOn: provenance ? provenance[1] : null,
            _translation: translation,
            _rotation: rotation,
        });
    }
    if (!cameras.length) throw new Error(`No LimelightConfig found in ${TARGET_CONFIG}.`);
    return cameras;
}

/** The parsed cameras without their internal offsets, for the API. */
export function publicCameras(cameras) {
    return cameras.map(({ _translation, _rotation, ...rest }) => rest);
}

// ---------------------------------------------------------------------------
// Rewriting
// ---------------------------------------------------------------------------

/** Degrees to one decimal, written the way the file already writes them: 180, not 180.0. */
export function formatDegrees(v) {
    const r = Math.round(v * 10) / 10;
    return Number.isInteger(r) ? String(r) : r.toFixed(1);
}

/** A length literal in the unit the existing literal uses: thousandths of an inch or a millimetre. */
export function formatLength(metres, unit) {
    return unit === "in" ? (metres / INCH).toFixed(3) : metres.toFixed(3);
}

/**
 * Rewrites some of one camera's mount values.
 *
 * @param {string} source current file contents
 * @param {string} key the Java field, e.g. "backLeftConfig"
 * @param {object} values any of roll, pitch (degrees) and up (metres)
 * @param {Date} now for the provenance comment
 * @returns {{source: string, before: object, after: object, changed: string[]}}
 */
export function rewriteCamera(source, key, values, now = new Date()) {
    const camera = parseCameras(source).find((c) => c.key === key);
    if (!camera) throw new Error(`No camera "${key}" in ${TARGET_CONFIG}.`);

    const wanted = Object.entries(values).filter(([, v]) => v !== undefined && v !== null);
    for (const [k, v] of wanted) {
        if (!WRITABLE.includes(k)) throw new Error(`"${k}" is not something this tool writes; only ${WRITABLE.join(", ")} are.`);
        if (!Number.isFinite(Number(v))) throw new Error(`Value for "${k}" is not a number.`);
    }
    if (!wanted.length) throw new Error("Nothing to write.");

    // Replace literals back-to-front so earlier offsets stay valid.
    const args = [...camera._translation.args, ...camera._rotation.args];
    const edits = wanted.map(([k, v]) => {
        const arg = args.find((a) => a.name === k);
        const text = k === "up" ? formatLength(Number(v), arg.unit) : formatDegrees(Number(v));
        return { start: arg.literalStart, end: arg.literalEnd, text, name: k, old: source.slice(arg.literalStart, arg.literalEnd) };
    });
    let out = source;
    for (const e of [...edits].sort((a, b) => b.start - a.start)) {
        out = out.slice(0, e.start) + e.text + out.slice(e.end);
    }

    out = renderRotation(out, key, edits, now);

    const after = parseCameras(out).find((c) => c.key === key);
    return { source: out, before: camera.values, after: after.values, changed: wanted.map(([k]) => k) };
}

/**
 * Re-renders the .withRotation(...) call after the literals changed: fixes a "31.8 deg" in its
 * trailing comment, puts the provenance line above it, and wraps it as google-java-format would.
 */
function renderRotation(source, key, edits, now) {
    const camera = parseCameras(source).find((c) => c.key === key);
    const rot = camera._rotation;
    const lineStart = source.lastIndexOf("\n", rot.callStart) + 1;
    const indent = /^\s*/.exec(source.slice(lineStart, rot.callStart))[0];
    const lineEnd = source.indexOf("\n", rot.close);

    // Whatever follows the closing paren on its line: "; // comment", " // comment", or nothing.
    let tail = source.slice(rot.close + 1, lineEnd);
    const pitchEdit = edits.find((e) => e.name === "pitch");
    if (pitchEdit && pitchEdit.old !== pitchEdit.text) {
        tail = tail.replace(new RegExp(`(^|[^\\d.])${escapeRe(pitchEdit.old)}(\\s*deg)`), `$1${pitchEdit.text}$2`);
    }

    const args = rot.args.map((a) => withoutComments(a.text).trim());
    const single = `${indent}.withRotation(${args.join(", ")})${tail}`;
    const rendered =
        single.length <= MAX_LINE_LENGTH ? single : `${indent}.withRotation(\n${indent}        ${args.join(", ")})${tail}`;

    // Drop an existing provenance line so they never stack.
    let head = source.slice(0, lineStart);
    head = head.replace(new RegExp(`[ \\t]*${escapeRe(PROVENANCE_PREFIX)}[^\\n]*\\n$`), "");
    // Local date, not UTC: a write at 11 pm in the pit should not be stamped tomorrow.
    const stamp = `${now.getFullYear()}-${String(now.getMonth() + 1).padStart(2, "0")}-${String(now.getDate()).padStart(2, "0")}`;

    return `${head}${indent}${PROVENANCE_PREFIX}${stamp}\n${rendered}${source.slice(lineEnd)}`;
}

// ---------------------------------------------------------------------------
// File and git access
// ---------------------------------------------------------------------------

export function targetPath() {
    return path.join(REPO_ROOT, TARGET_CONFIG);
}

export function readTarget() {
    return fs.readFileSync(targetPath(), "utf8");
}

export function writeTarget(source) {
    fs.writeFileSync(targetPath(), source);
}

function git(args) {
    return new Promise((resolve) => {
        execFile("git", args, { cwd: REPO_ROOT }, (err, stdout) => resolve(err ? null : stdout.trim()));
    });
}

export async function gitStatus() {
    const branch = await git(["rev-parse", "--abbrev-ref", "HEAD"]);
    const dirty = await git(["status", "--porcelain", "--", TARGET_CONFIG]);
    return { branch, targetModified: Boolean(dirty) };
}
