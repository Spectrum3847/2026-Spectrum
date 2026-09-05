/**
 * Reads and rewrites the swerve encoder offsets in the robot's config .java file.
 *
 * Moved here from the standalone tools/swerve-align app so the alignment page can live in the
 * robot app alongside everything else. The parsing and formatting below is that tool's, unchanged:
 * it survives the arithmetic expressions already in the configs (`-0.23046875 - 0.25`), rounds to
 * the CANcoder's own 1/4096 resolution, and emits the call formatted the way
 * googleJavaFormat().aosp() would, so `./gradlew spotlessCheck` stays green afterwards.
 *
 * The browser never touches this. It talks NT4 to the robot; only the server reads the source tree.
 */
import fs from "node:fs";
import path from "node:path";
import { execFile } from "node:child_process";
import { APP_ROOT, config } from "./config.js";

/** Repo root, two levels up from tools/robot-app. */
const REPO_ROOT = path.resolve(APP_ROOT, "..", "..");

/**
 * Which robot config the app writes to. Defaults to OM2026 because Robot.java currently selects
 * that config for every RoboRIO; override with `swerveAlign.targetConfig` in config.local.json.
 */
export const TARGET_CONFIG = config.swerveAlign?.targetConfig || "src/main/java/frc/robot/configs/OM2026.java";

/** The call we parse and rewrite. */
const CALL_NAME = "swerve.configEncoderOffsets";

/** Marker line written above the call so the file records when it was last aligned. */
const PROVENANCE_PREFIX = "// Aligned by the Spectrum robot app on ";

/** CANcoder resolution. Rounding to this reproduces the exact style of the existing constants. */
const ROTATION_STEPS = 4096;

/** google-java-format's line limit, so what we write survives spotless untouched. */
const MAX_LINE_LENGTH = 100;

export const MODULE_KEYS = ["fl", "fr", "bl", "br"];
export const MODULE_LABELS = ["Front Left", "Front Right", "Back Left", "Back Right"];

// ---------------------------------------------------------------------------
// Java config file parsing and rewriting
// ---------------------------------------------------------------------------

/**
 * Evaluates one argument expression from the config call.
 *
 * The existing values are arithmetic rather than plain literals -- OM2026.java has entries like
 * `-0.23046875 - 0.25` where someone recorded the raw reading and the correction separately -- so
 * this has to do more than parseFloat. Anything outside a strict numeric-expression charset is
 * rejected rather than guessed at.
 *
 * @param {string} expression the source text of a single argument
 * @returns {number} its value in rotations
 */
function evaluateNumericExpression(expression) {
    const cleaned = expression.replace(/\/\*[\s\S]*?\*\//g, ' ').trim();
    if (!/^[-+*/().\d\s]+$/.test(cleaned)) {
        throw new Error(
            `Cannot read the offset expression "${expression.trim()}" in ${TARGET_CONFIG}. ` +
                'This tool only understands plain numbers and + - * / arithmetic. ' +
                'Simplify that argument by hand and try again.'
        );
    }
    let value;
    try {
        // Safe: the charset check above admits only numeric arithmetic.
        value = Function(`"use strict"; return (${cleaned});`)();
    } catch (err) {
        throw new Error(`Could not evaluate the offset expression "${expression.trim()}".`);
    }
    if (typeof value !== 'number' || !Number.isFinite(value)) {
        throw new Error(`The offset expression "${expression.trim()}" is not a finite number.`);
    }
    return value;
}

/**
 * Splits a call's argument text on top-level commas.
 *
 * @param {string} argumentText the text between the call's parentheses
 * @returns {string[]} one entry per argument
 */
function splitArguments(argumentText) {
    const args = [];
    let depth = 0;
    let current = '';
    for (const char of argumentText) {
        if (char === '(') depth++;
        if (char === ')') depth--;
        if (char === ',' && depth === 0) {
            args.push(current);
            current = '';
        } else {
            current += char;
        }
    }
    args.push(current);
    return args;
}

/**
 * Locates the configEncoderOffsets call in a config file.
 *
 * @param {string} source the file's contents
 * @returns {{start: number, end: number, argumentText: string, indent: string}} call bounds, the
 *     text between the parentheses, and the leading whitespace of the line the call starts on
 */
function locateCall(source) {
    const callStart = source.indexOf(CALL_NAME);
    if (callStart === -1) {
        throw new Error(`No ${CALL_NAME}(...) call found in ${TARGET_CONFIG}.`);
    }
    if (source.indexOf(CALL_NAME, callStart + 1) !== -1) {
        throw new Error(
            `Found more than one ${CALL_NAME}(...) call in ${TARGET_CONFIG}. ` +
                'Remove the extras so there is exactly one set of offsets to update.'
        );
    }

    const open = source.indexOf('(', callStart);
    let depth = 0;
    let close = -1;
    for (let i = open; i < source.length; i++) {
        if (source[i] === '(') depth++;
        else if (source[i] === ')') {
            depth--;
            if (depth === 0) {
                close = i;
                break;
            }
        }
    }
    if (close === -1) {
        throw new Error(`The ${CALL_NAME}(...) call in ${TARGET_CONFIG} is not closed.`);
    }

    const semicolon = source.indexOf(';', close);
    const lineStart = source.lastIndexOf('\n', callStart) + 1;
    const indent = source.slice(lineStart, callStart).match(/^\s*/)[0];

    return {
        start: lineStart,
        end: semicolon + 1,
        argumentText: source.slice(open + 1, close),
        indent
    };
}

/**
 * Reads the four offsets currently in the config file.
 *
 * @param {string} source the file's contents
 * @returns {{offsets: object, expressions: object, alignedOn: (string|null)}}
 */
function parseConfig(source) {
    const call = locateCall(source);
    const args = splitArguments(call.argumentText);
    if (args.length !== 4) {
        throw new Error(
            `Expected 4 arguments to ${CALL_NAME}(...) in ${TARGET_CONFIG}, found ${args.length}.`
        );
    }

    const offsets = {};
    const expressions = {};
    MODULE_KEYS.forEach((key, i) => {
        offsets[key] = evaluateNumericExpression(args[i]);
        expressions[key] = args[i].trim();
    });

    const provenance = source.slice(0, call.start).match(
        new RegExp(`${PROVENANCE_PREFIX.replace(/[/*]/g, '\\$&')}(.*)\\s*$`)
    );

    return { offsets, expressions, alignedOn: provenance ? provenance[1].trim() : null };
}

/**
 * Rounds an offset to the encoder's own resolution.
 *
 * Sub-LSB precision is noise, and rounding here reproduces the tidy values already in the file
 * (-0.83544921875 and friends are all multiples of 1/4096).
 *
 * @param {number} rotations the raw computed offset
 * @returns {string} the value formatted for Java source
 */
function formatOffset(rotations) {
    const rounded = Math.round(rotations * ROTATION_STEPS) / ROTATION_STEPS;
    // Never emit -0, and never emit exponent notation.
    const normalized = rounded === 0 ? 0 : rounded;
    return String(normalized);
}

/**
 * Renders the replacement call, matching what google-java-format would produce so `spotlessCheck`
 * stays green without anyone having to run `spotlessApply`.
 *
 * @param {object} offsets keyed by fl/fr/bl/br
 * @param {string} indent the call's leading whitespace
 * @returns {string} the call source, without a trailing newline
 */
function renderCall(offsets, indent) {
    const values = MODULE_KEYS.map((key) => formatOffset(offsets[key]));
    const argIndent = `${indent}        `;

    // google-java-format tries these three shapes in order, so we have to as well.
    const singleLine = `${indent}${CALL_NAME}(${values.join(', ')});`;
    if (singleLine.length <= MAX_LINE_LENGTH) {
        return singleLine;
    }

    const wrapped = `${indent}${CALL_NAME}(\n${argIndent}${values.join(', ')});`;
    if (wrapped.split('\n').every((line) => line.length <= MAX_LINE_LENGTH)) {
        return wrapped;
    }

    return (
        `${indent}${CALL_NAME}(\n` +
        values.map((value, i) => `${argIndent}${value}${i === 3 ? ');' : ','}`).join('\n')
    );
}

/**
 * Produces the updated file contents.
 *
 * @param {string} source the current file contents
 * @param {object} offsets the new offsets, keyed by fl/fr/bl/br
 * @param {Date} now timestamp for the provenance comment
 * @returns {{source: string, before: string, after: string}}
 */
function rewriteConfig(source, offsets, now) {
    const call = locateCall(source);
    const before = source.slice(call.start, call.end);

    // Drop a previous provenance comment so they do not stack up.
    let head = source.slice(0, call.start);
    const provenanceLine = new RegExp(
        `[ \\t]*${PROVENANCE_PREFIX.replace(/[/*]/g, '\\$&')}.*\\r?\\n`
    );
    head = head.replace(provenanceLine, '');

    const stamp = now.toISOString().slice(0, 10);
    const comment = `${call.indent}${PROVENANCE_PREFIX}${stamp}\n`;
    const after = renderCall(offsets, call.indent);

    return {
        source: head + comment + after + source.slice(call.end),
        before,
        after
    };
}

// ---------------------------------------------------------------------------
// Git helpers (informational only -- the tool never runs a git write command)
// ---------------------------------------------------------------------------

function git(args) {
    return new Promise((resolve) => {
        execFile('git', args, { cwd: REPO_ROOT }, (err, stdout) => {
            resolve(err ? null : stdout.trim());
        });
    });
}

async function gitStatus() {
    const branch = await git(['rev-parse', '--abbrev-ref', 'HEAD']);
    const dirty = await git(['status', '--porcelain', '--', TARGET_CONFIG]);
    return { branch, targetModified: Boolean(dirty) };
}

export { parseConfig, rewriteConfig, renderCall, formatOffset, locateCall, gitStatus, REPO_ROOT };

/** Absolute path of the file the tool reads and writes. */
export function targetPath() {
    return path.join(REPO_ROOT, TARGET_CONFIG);
}

export function readTarget() {
    return fs.readFileSync(targetPath(), "utf8");
}

export function writeTarget(source) {
    fs.writeFileSync(targetPath(), source);
}
