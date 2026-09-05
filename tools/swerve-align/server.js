#!/usr/bin/env node
/*
 * Swerve alignment helper -- local web server.
 *
 * Serves the alignment UI in public/ and is the only thing allowed to touch the robot's config
 * source file. The browser talks NetworkTables to the robot directly; this process exists to read
 * and rewrite the four numbers in swerve.configEncoderOffsets(...).
 *
 * Usage:
 *   node tools/swerve-align/server.js              start the server and open a browser
 *   node tools/swerve-align/server.js --no-open    start without opening a browser
 *   node tools/swerve-align/server.js --self-test  round-trip the config parser and exit
 *
 * No dependencies. Node 18+.
 */

'use strict';

const fs = require('fs');
const http = require('http');
const os = require('os');
const path = require('path');
const { execFile, spawn } = require('child_process');

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/**
 * The robot config file the app writes to. Change this one line to point the tool at a different
 * robot (FM2026.java, PM2026.java, ...).
 */
const TARGET_CONFIG = 'src/main/java/frc/robot/configs/OM2026.java';

/** Bound to loopback only: this server can rewrite source files, so keep it off the pit network. */
const HOST = '127.0.0.1';
const PORT = 5817;

/** The call we parse and rewrite. */
const CALL_NAME = 'swerve.configEncoderOffsets';

/** Marker line written above the call so the file records when it was last aligned. */
const PROVENANCE_PREFIX = '// Aligned by tools/swerve-align on ';

/** CANcoder resolution. Rounding to this reproduces the exact style of the existing constants. */
const ROTATION_STEPS = 4096;

/** google-java-format's line limit, so what we write survives spotless untouched. */
const MAX_LINE_LENGTH = 100;

const REPO_ROOT = path.resolve(__dirname, '..', '..');
const PUBLIC_DIR = path.join(__dirname, 'public');
const MODULE_KEYS = ['fl', 'fr', 'bl', 'br'];
const MODULE_LABELS = ['Front Left', 'Front Right', 'Back Left', 'Back Right'];

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

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

const MIME_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon'
};

function sendJson(res, status, body) {
    const payload = JSON.stringify(body);
    res.writeHead(status, {
        'Content-Type': 'application/json; charset=utf-8',
        'Cache-Control': 'no-store'
    });
    res.end(payload);
}

function serveStatic(req, res) {
    const requested = req.url === '/' ? '/index.html' : req.url.split('?')[0];
    const filePath = path.join(PUBLIC_DIR, path.normalize(requested));
    if (!filePath.startsWith(PUBLIC_DIR)) {
        res.writeHead(403).end('Forbidden');
        return;
    }
    fs.readFile(filePath, (err, data) => {
        if (err) {
            res.writeHead(404).end('Not found');
            return;
        }
        res.writeHead(200, {
            'Content-Type': MIME_TYPES[path.extname(filePath)] || 'application/octet-stream',
            'Cache-Control': 'no-store'
        });
        res.end(data);
    });
}

function readBody(req) {
    return new Promise((resolve, reject) => {
        let body = '';
        req.on('data', (chunk) => {
            body += chunk;
            if (body.length > 1e6) reject(new Error('Request body too large'));
        });
        req.on('end', () => resolve(body));
        req.on('error', reject);
    });
}

async function handleGetTarget(res) {
    const absolute = path.join(REPO_ROOT, TARGET_CONFIG);
    const source = fs.readFileSync(absolute, 'utf8');
    const parsed = parseConfig(source);
    const git = await gitStatus();
    sendJson(res, 200, {
        file: TARGET_CONFIG,
        absolutePath: absolute,
        moduleKeys: MODULE_KEYS,
        moduleLabels: MODULE_LABELS,
        offsets: parsed.offsets,
        expressions: parsed.expressions,
        alignedOn: parsed.alignedOn,
        branch: git.branch,
        targetModified: git.targetModified
    });
}

async function handlePostApply(req, res) {
    const body = JSON.parse(await readBody(req));
    const offsets = {};
    for (const key of MODULE_KEYS) {
        const value = Number(body[key]);
        if (!Number.isFinite(value)) {
            throw new Error(`Missing or invalid offset for "${key}".`);
        }
        if (value < -0.5 || value >= 0.5) {
            throw new Error(
                `Offset for "${key}" is ${value} rotations; it must be in [-0.5, 0.5).`
            );
        }
        offsets[key] = value;
    }

    const absolute = path.join(REPO_ROOT, TARGET_CONFIG);
    const source = fs.readFileSync(absolute, 'utf8');
    const result = rewriteConfig(source, offsets, new Date());
    fs.writeFileSync(absolute, result.source, 'utf8');

    // Read it back so the UI shows what is really on disk, not what we intended to write.
    const verified = parseConfig(fs.readFileSync(absolute, 'utf8'));

    console.log(`[swerve-align] wrote new offsets to ${TARGET_CONFIG}`);
    sendJson(res, 200, {
        file: TARGET_CONFIG,
        before: result.before,
        after: result.after,
        offsets: verified.offsets
    });
}

function createServer() {
    return http.createServer(async (req, res) => {
        try {
            if (req.method === 'GET' && req.url.startsWith('/api/target')) {
                await handleGetTarget(res);
            } else if (req.method === 'POST' && req.url.startsWith('/api/apply')) {
                await handlePostApply(req, res);
            } else if (req.method === 'GET') {
                serveStatic(req, res);
            } else {
                res.writeHead(405).end('Method not allowed');
            }
        } catch (err) {
            console.error(`[swerve-align] ${err.message}`);
            sendJson(res, 400, { error: err.message });
        }
    });
}

// ---------------------------------------------------------------------------
// Browser launch
// ---------------------------------------------------------------------------

function openBrowser(url) {
    const platform = os.platform();
    try {
        if (platform === 'win32') {
            spawn('cmd', ['/c', 'start', '""', url], { detached: true, stdio: 'ignore' }).unref();
        } else if (platform === 'darwin') {
            spawn('open', [url], { detached: true, stdio: 'ignore' }).unref();
        } else {
            spawn('xdg-open', [url], { detached: true, stdio: 'ignore' }).unref();
        }
    } catch (err) {
        console.log(`[swerve-align] could not open a browser automatically: ${err.message}`);
    }
}

// ---------------------------------------------------------------------------
// Self test
// ---------------------------------------------------------------------------

/**
 * Round-trips the parser and the writer against the real config file, in memory. Catches the two
 * ways this tool could quietly corrupt source: misreading the current values, or writing something
 * it cannot read back.
 *
 * @returns {number} process exit code
 */
function selfTest() {
    const cases = [
        { name: 'live config file', source: fs.readFileSync(path.join(REPO_ROOT, TARGET_CONFIG), 'utf8') },
        {
            name: 'single-line call',
            source: 'class T {\n    T() {\n        swerve.configEncoderOffsets(0.1, -0.2, 0.3, -0.4);\n    }\n}\n'
        },
        {
            name: 'arithmetic arguments',
            source:
                'class T {\n    T() {\n        swerve.configEncoderOffsets(\n' +
                '                -0.23046875 - 0.25,\n                0.10986328125 + 0.25,\n' +
                '                -0.263916015625 + 0.125,\n                0.34716796875 - 0.125);\n    }\n}\n'
        }
    ];

    const target = { fl: -0.480712890625, fr: 0.35986328125, bl: -0.138916015625, br: 0.22216796875 };
    let failures = 0;

    for (const testCase of cases) {
        try {
            const parsed = parseConfig(testCase.source);
            for (const key of MODULE_KEYS) {
                if (!Number.isFinite(parsed.offsets[key])) {
                    throw new Error(`parsed ${key} is not a number`);
                }
            }

            const first = rewriteConfig(testCase.source, target, new Date('2026-09-05'));
            const reparsed = parseConfig(first.source);
            for (const key of MODULE_KEYS) {
                if (Math.abs(reparsed.offsets[key] - target[key]) > 1e-12) {
                    throw new Error(
                        `${key} round-tripped to ${reparsed.offsets[key]}, expected ${target[key]}`
                    );
                }
            }

            // Writing twice must not stack provenance comments or drift.
            const second = rewriteConfig(first.source, target, new Date('2026-09-06'));
            const commentCount = (second.source.match(/Aligned by tools\/swerve-align/g) || [])
                .length;
            if (commentCount !== 1) {
                throw new Error(`found ${commentCount} provenance comments after two writes`);
            }

            for (const line of second.source.split('\n')) {
                if (line.length > MAX_LINE_LENGTH) {
                    throw new Error(`emitted a ${line.length}-column line: ${line.trim()}`);
                }
            }

            console.log(`  ok    ${testCase.name}`);
        } catch (err) {
            failures++;
            console.error(`  FAIL  ${testCase.name}: ${err.message}`);
        }
    }

    console.log(failures === 0 ? '\nself-test passed' : `\nself-test failed (${failures})`);
    return failures === 0 ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

function main() {
    const args = process.argv.slice(2);

    if (!fs.existsSync(path.join(REPO_ROOT, TARGET_CONFIG))) {
        console.error(`Could not find ${TARGET_CONFIG} under ${REPO_ROOT}.`);
        console.error('Run this from a checkout of the robot code.');
        process.exit(1);
    }

    if (args.includes('--self-test')) {
        process.exit(selfTest());
    }

    const server = createServer();
    server.on('error', (err) => {
        if (err.code === 'EADDRINUSE') {
            console.error(
                `Port ${PORT} is already in use -- the alignment app may already be running.`
            );
            console.error(`Try opening http://${HOST}:${PORT} first.`);
        } else {
            console.error(err.message);
        }
        process.exit(1);
    });

    server.listen(PORT, HOST, () => {
        const url = `http://${HOST}:${PORT}`;
        console.log('Spectrum swerve alignment');
        console.log(`  repo:   ${REPO_ROOT}`);
        console.log(`  writes: ${TARGET_CONFIG}`);
        console.log(`  open:   ${url}`);
        console.log('\nPress Ctrl+C to stop.');
        if (!args.includes('--no-open')) {
            openBrowser(url);
        }
    });
}

main();
