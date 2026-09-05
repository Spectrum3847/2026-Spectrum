/*
 * Swerve alignment page.
 *
 * The offset maths, the 180-degree checks and the capture logic below are the standalone
 * tools/swerve-align app's, unchanged. Folding it into the robot app changed three things:
 * the shared shell header is mounted, NT4Client is imported instead of read off `window`, and
 * the two endpoints moved under /api/swerve/.
 */
import "../../styles.css";
import "./align.css";
import { mountHeader } from "../../lib/ui.js";
import { NT4Client } from "../../lib/nt4.js";

mountHeader();

/*
 * Swerve alignment UI.
 *
 * Reads the robot's raw CANcoder data over NetworkTables (published by SwerveAlignment.java),
 * works out what each module's offset should be so that "wheel pointed forward" reads zero, sanity
 * checks the result against what is currently in the source, and asks the server to rewrite it.
 */

'use strict';

const NT_PREFIX = '/Robot/Swerve/Align/';

/** NT module name -> the short key used everywhere else, in SwerveConfig.getModules() order. */
const MODULES = [
    { key: 'fl', nt: 'FrontLeft', label: 'Front Left' },
    { key: 'fr', nt: 'FrontRight', label: 'Front Right' },
    { key: 'bl', nt: 'BackLeft', label: 'Back Left' },
    { key: 'br', nt: 'BackRight', label: 'Back Right' }
];

/** CANcoder resolution; the server rounds to this too, so the UI shows exactly what gets written. */
const ROTATION_STEPS = 4096;

/** A module is "still enough to trust" below this steer speed (rot/s). About 1.8 deg/s. */
const STILL_ROTATIONS_PER_SEC = 0.005;

/** Data older than this is not to be trusted for a capture. */
const STALE_MS = 1000;

/** How long to average encoder readings when capturing. */
const CAPTURE_MS = 600;

/** Below this the offset has not meaningfully moved. */
const NO_CHANGE_DEG = 3;

/** Above this a change stops looking like a normal touch-up. */
const NORMAL_CHANGE_DEG = 30;

/** Window around 180 deg that reads as "this wheel is backwards". */
const BACKWARDS_MIN_DEG = 150;
const BACKWARDS_MAX_DEG = 210;

/** How far the robot's magnet offset may differ from the source before we call it a mismatch. */
const OFFSET_MISMATCH_ROTATIONS = 0.5 / 360;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

const state = {
    /** From GET /api/target. */
    target: null,
    /** Per-module live values keyed by short key. */
    live: Object.fromEntries(MODULES.map((m) => [m.key, {}])),
    /** Wall-clock ms at which each module first reported anything. */
    firstSeen: Object.fromEntries(MODULES.map((m) => [m.key, 0])),
    /**
     * Wall-clock ms of the last heartbeat.
     *
     * NetworkTables only sends a topic when its value changes, so a wheel sitting perfectly still
     * stops producing updates entirely. Freshness has to come from the timestamp the robot
     * publishes alongside the module data, which always moves.
     */
    heartbeatAt: 0,
    connection: 'disconnected',
    capture: null,
    capturing: false,
    /** Per-module 'accept' | 'flip' for modules that need a decision. */
    resolutions: {}
};

let client = null;

// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------

/**
 * Wraps rotations into [-0.5, 0.5), matching how CTRE reports CANcoder absolute position and how
 * SwerveAlignment.wrapRotations does it on the robot.
 *
 * @param {number} rotations
 * @returns {number}
 */
function wrapRotations(rotations) {
    return rotations - Math.floor(rotations + 0.5);
}

/**
 * Averages angles without breaking across the wrap point: a wheel sitting near +/-0.5 rotations
 * would otherwise average to zero, which is exactly wrong.
 *
 * @param {number[]} samples rotations
 * @returns {number} the circular mean, wrapped to [-0.5, 0.5)
 */
function circularMean(samples) {
    let sumSin = 0;
    let sumCos = 0;
    for (const sample of samples) {
        const radians = sample * 2 * Math.PI;
        sumSin += Math.sin(radians);
        sumCos += Math.cos(radians);
    }
    return wrapRotations(Math.atan2(sumSin, sumCos) / (2 * Math.PI));
}

/**
 * Rounds to the encoder's own resolution. Sub-LSB digits are noise, and this makes the UI show the
 * exact value the server will write.
 *
 * @param {number} rotations
 * @returns {number}
 */
function quantize(rotations) {
    const rounded = Math.round(rotations * ROTATION_STEPS) / ROTATION_STEPS;
    return rounded === 0 ? 0 : rounded;
}

const rotationsToDegrees = (rotations) => rotations * 360;

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

const fmtRotations = (value) => (Number.isFinite(value) ? String(quantize(value)) : '--');
const fmtDegrees = (value) => (Number.isFinite(value) ? `${value.toFixed(1)}°` : '--');
const fmtSignedDegrees = (value) =>
    Number.isFinite(value) ? `${value >= 0 ? '+' : ''}${value.toFixed(1)}°` : '--';

function element(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    return node;
}

// ---------------------------------------------------------------------------
// NetworkTables
// ---------------------------------------------------------------------------

const NT_FIELDS = {
    AbsoluteRotations: 'absolute',
    RawRotations: 'raw',
    AppliedOffset: 'appliedOffset',
    AppliedOffsetValid: 'appliedOffsetValid',
    ConfigOffset: 'configOffset',
    SteerVelocity: 'velocity',
    Connected: 'connected',
    EncoderId: 'encoderId'
};

function handleNtValue(topic, value) {
    if (!topic.startsWith(NT_PREFIX)) return;
    const path = topic.slice(NT_PREFIX.length).split('/');

    if (path.length === 1 && path[0] === 'Timestamp') {
        state.heartbeatAt = Date.now();
        return;
    }
    if (path.length !== 2) return;

    const module = MODULES.find((m) => m.nt === path[0]);
    const field = NT_FIELDS[path[1]];
    if (!module || !field) return;

    state.live[module.key][field] = value;
    if (!state.firstSeen[module.key]) {
        state.firstSeen[module.key] = Date.now();
    }
}

function connect(host) {
    localStorage.setItem('swerve-align-host', host);
    if (!client) {
        client = new NT4Client({
            prefix: NT_PREFIX,
            onValue: handleNtValue,
            onState: (connectionState) => {
                state.connection = connectionState;
                render();
            }
        });
    }
    for (const module of MODULES) {
        state.live[module.key] = {};
        state.firstSeen[module.key] = 0;
    }
    state.heartbeatAt = 0;
    client.connect(host);
}

/** @returns {boolean} whether the robot is sending fresh alignment data right now */
function dataIsFresh() {
    return state.heartbeatAt !== 0 && Date.now() - state.heartbeatAt <= STALE_MS;
}

/**
 * The modules the student ticked in "Modules to align".
 *
 * There is no single reference that squares two of these wheels at once on this drivetrain, so
 * aligning one module at a time is a normal thing to want -- after swapping a single module, say.
 * Everything unticked keeps the offset it already has in the code.
 *
 * @returns {object[]} entries from MODULES
 */
function selectedModules() {
    return MODULES.filter((module) => {
        const box = document.querySelector(`#module-select input[data-module="${module.key}"]`);
        return !box || box.checked;
    });
}

/** @param {string} key @returns {boolean} */
function isSelected(key) {
    return selectedModules().some((module) => module.key === key);
}

// ---------------------------------------------------------------------------
// Health checks
// ---------------------------------------------------------------------------

/**
 * Everything that has to be true before a capture is worth taking.
 *
 * @returns {{ready: boolean, reasons: string[]}}
 */
function readiness() {
    const reasons = [];

    if (state.connection !== 'connected') {
        reasons.push('not connected to the robot');
    } else if (!dataIsFresh()) {
        reasons.push(
            state.heartbeatAt ? 'the robot stopped sending data' : 'no alignment data yet'
        );
    }

    // Only the modules being aligned have to be healthy and still.
    const selected = selectedModules();
    if (!selected.length) reasons.push('no modules selected');

    const missing = selected.filter((m) => !state.firstSeen[m.key]);
    const offline = selected.filter((m) => state.live[m.key].connected === false);
    const moving = selected.filter(
        (m) => Math.abs(state.live[m.key].velocity || 0) > STILL_ROTATIONS_PER_SEC
    );

    if (missing.length) reasons.push(`no data yet from ${missing.map((m) => m.label).join(', ')}`);
    if (offline.length) reasons.push(`${offline.map((m) => m.label).join(', ')} encoder offline`);
    if (moving.length) reasons.push(`${moving.map((m) => m.label).join(', ')} still moving`);

    const unchecked = [...document.querySelectorAll('#checklist input')].filter((i) => !i.checked);
    if (unchecked.length) reasons.push(`${unchecked.length} checklist item(s) left`);

    if (!state.target) reasons.push('config file not loaded');

    return { ready: reasons.length === 0, reasons };
}

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------

/** How often to take a reading while capturing. The robot publishes at 20 Hz. */
const SAMPLE_INTERVAL_MS = 50;

function startCapture() {
    const modules = selectedModules();
    state.capture = {
        modules,
        samples: Object.fromEntries(modules.map((m) => [m.key, []])),
        maxVelocity: Object.fromEntries(modules.map((m) => [m.key, 0])),
        wentStale: false,
        results: null,
        error: null
    };
    state.resolutions = {};
    state.capturing = true;
    render();

    // Sample on a timer rather than off NT updates: NetworkTables only sends a topic when it
    // changes, and a wheel that is genuinely still may not send anything at all during the window.
    const sampler = setInterval(() => {
        if (!dataIsFresh()) {
            state.capture.wentStale = true;
            return;
        }
        for (const module of state.capture.modules) {
            const live = state.live[module.key];
            if (Number.isFinite(live.raw)) {
                state.capture.samples[module.key].push(live.raw);
            }
            state.capture.maxVelocity[module.key] = Math.max(
                state.capture.maxVelocity[module.key],
                Math.abs(live.velocity || 0)
            );
        }
    }, SAMPLE_INTERVAL_MS);

    setTimeout(() => {
        clearInterval(sampler);
        state.capturing = false;
        finishCapture();
        render();
    }, CAPTURE_MS);
}

function finishCapture() {
    const capture = state.capture;

    if (capture.wentStale) {
        capture.error =
            'The robot stopped sending data partway through the capture. Check the connection ' +
            'and try again.';
        return;
    }

    const thin = capture.modules.filter((m) => capture.samples[m.key].length < 3);
    if (thin.length) {
        capture.error =
            `Only got a couple of readings from ${thin.map((m) => m.label).join(', ')}. ` +
            'Check the connection and try again.';
        return;
    }

    const jumpy = capture.modules.filter(
        (m) => capture.maxVelocity[m.key] > STILL_ROTATIONS_PER_SEC
    );
    if (jumpy.length) {
        capture.error =
            `${jumpy.map((m) => m.label).join(', ')} moved while capturing. ` +
            'Check the pin is seated, let the wheel settle, and capture again.';
        return;
    }

    capture.results = capture.modules.map((module) => {
        const rawMean = circularMean(capture.samples[module.key]);
        const newOffset = quantize(wrapRotations(-rawMean));
        const sourceOffset = state.target.offsets[module.key];
        const deltaDeg = rotationsToDegrees(wrapRotations(newOffset - sourceOffset));
        const magnitude = Math.abs(deltaDeg);

        let verdict;
        if (magnitude <= NO_CHANGE_DEG) {
            verdict = { kind: 'ok', label: 'Already aligned' };
        } else if (magnitude <= NORMAL_CHANGE_DEG) {
            verdict = { kind: 'change', label: 'Normal correction' };
        } else if (magnitude >= BACKWARDS_MIN_DEG && magnitude <= BACKWARDS_MAX_DEG) {
            verdict = { kind: 'bad', label: 'Wheel likely backwards' };
        } else {
            verdict = { kind: 'warn', label: 'Unexpected change' };
        }

        return {
            ...module,
            rawMean,
            newOffset,
            sourceOffset,
            deltaDeg,
            verdict,
            sampleCount: capture.samples[module.key].length
        };
    });

    // Default every module that needs a decision to the safest reading of the situation: trust the
    // physical alignment the student just did.
    for (const result of capture.results) {
        if (result.verdict.kind === 'bad' || result.verdict.kind === 'warn') {
            state.resolutions[result.key] = null;
        }
    }
}

/**
 * The offset that will actually be written for a module, after any per-module decision.
 *
 * @param {object} result one entry from capture.results
 * @returns {number} rotations
 */
function resolvedOffset(result) {
    if (state.resolutions[result.key] === 'flip') {
        return quantize(wrapRotations(result.newOffset + 0.5));
    }
    return result.newOffset;
}

/** @returns {boolean} whether every module needing a decision has one */
function allResolved() {
    if (!state.capture || !state.capture.results) return false;
    return Object.keys(state.resolutions).every((key) => state.resolutions[key] !== null);
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

function renderStatus() {
    const dot = document.getElementById('status-dot');
    const text = document.getElementById('status-text');
    const age = document.getElementById('data-age');

    const labels = {
        connected: 'Connected',
        connecting: 'Connecting…',
        disconnected: 'Not connected'
    };

    const ageMs = state.heartbeatAt ? Date.now() - state.heartbeatAt : Infinity;
    const stale = state.connection === 'connected' && !dataIsFresh();

    dot.className = `dot ${stale ? 'stale' : state.connection}`;
    text.textContent = labels[state.connection];

    if (state.connection !== 'connected') {
        age.textContent = '';
    } else if (!state.heartbeatAt) {
        age.textContent = '— waiting for alignment data';
    } else if (stale) {
        age.textContent = `— data is ${(ageMs / 1000).toFixed(1)}s old`;
    } else {
        age.textContent = '— live';
    }
}

function dialSvg(degrees, connected) {
    const size = 72;
    const center = size / 2;
    const radius = center - 4;
    // 0 rotations is straight forward, which is up on screen; positive rotations are CCW.
    const radians = ((90 + degrees) * Math.PI) / 180;
    const x = center + Math.cos(radians) * (radius - 6);
    const y = center - Math.sin(radians) * (radius - 6);
    const stroke = connected ? 'var(--purple)' : 'var(--bad)';
    return `
        <svg class="dial" width="${size}" height="${size}" viewBox="0 0 ${size} ${size}">
            <circle cx="${center}" cy="${center}" r="${radius}"
                    fill="var(--surface-alt)" stroke="var(--line-strong)" stroke-width="2" />
            <line x1="${center}" y1="${center - radius}" x2="${center}" y2="${center - radius + 7}"
                  stroke="var(--muted)" stroke-width="2" />
            <line x1="${center}" y1="${center}" x2="${x}" y2="${y}"
                  stroke="${stroke}" stroke-width="3" stroke-linecap="round" />
            <circle cx="${center}" cy="${center}" r="3.5" fill="${stroke}" />
        </svg>`;
}

function renderModules() {
    const container = document.getElementById('modules');
    const fresh = dataIsFresh();
    container.innerHTML = MODULES.map((module) => {
        const live = state.live[module.key];
        const seen = state.firstSeen[module.key];
        const connected = fresh && seen && live.connected !== false;
        const degrees = Number.isFinite(live.absolute) ? rotationsToDegrees(live.absolute) : NaN;
        const sourceOffset = state.target ? state.target.offsets[module.key] : NaN;
        const selected = isSelected(module.key);

        let note = '';
        if (!selected) {
            note = '<span class="badge skip">not selected</span>';
        } else if (!seen) {
            note = '<span class="badge bad">no data</span>';
        } else if (!fresh) {
            note = '<span class="badge warn">stale</span>';
        } else if (live.connected === false) {
            note = '<span class="badge bad">encoder offline</span>';
        } else if (Math.abs(live.velocity || 0) > STILL_ROTATIONS_PER_SEC) {
            note = '<span class="badge warn">moving</span>';
        }

        const classes = [
            'module',
            connected || !selected ? '' : 'offline',
            selected ? '' : 'excluded'
        ]
            .filter(Boolean)
            .join(' ');

        return `
            <div class="${classes}">
                ${dialSvg(Number.isFinite(degrees) ? degrees : 0, connected)}
                <div class="body">
                    <h3>${module.label} ${note}</h3>
                    <div class="angle">${fmtDegrees(degrees)}</div>
                    <dl>
                        <dt>Raw (no offset)</dt><dd>${fmtRotations(live.raw)}</dd>
                        <dt>Offset on robot</dt><dd>${fmtRotations(live.appliedOffset)}</dd>
                        <dt>Offset in source</dt><dd>${fmtRotations(sourceOffset)}</dd>
                        <dt>CANcoder ID</dt><dd>${live.encoderId ?? '--'}</dd>
                    </dl>
                </div>
            </div>`;
    }).join('');
}

function banner(kind, title, ...paragraphs) {
    return `<div class="banner ${kind}"><h3>${title}</h3>${paragraphs
        .map((p) => `<p>${p}</p>`)
        .join('')}</div>`;
}

function renderBanners() {
    const container = document.getElementById('banners');
    const banners = [];

    // The robot is running different offsets than the file we are about to edit. Aligning against
    // that would fold the difference into the new numbers.
    if (state.target && state.connection === 'connected') {
        const mismatched = MODULES.filter((module) => {
            const applied = state.live[module.key].appliedOffset;
            if (!Number.isFinite(applied)) return false;
            const source = state.target.offsets[module.key];
            return Math.abs(wrapRotations(applied - source)) > OFFSET_MISMATCH_ROTATIONS;
        });
        if (mismatched.length) {
            banners.push(
                banner(
                    'bad',
                    'The robot is not running this code',
                    `The magnet offset programmed into ${mismatched
                        .map((m) => m.label)
                        .join(', ')} does not match <code>${state.target.file}</code>.`,
                    'Deploy the current code before aligning, otherwise the new offsets will be ' +
                        'computed against the wrong baseline.'
                )
            );
        }

        const unreadable = MODULES.filter(
            (m) => state.live[m.key].appliedOffsetValid === false
        );
        if (unreadable.length) {
            banners.push(
                banner(
                    'warn',
                    'Could not read the offset off the encoder',
                    `The robot could not read the magnet offset back from ${unreadable
                        .map((m) => m.label)
                        .join(', ')} and is reporting the compiled-in value instead.`,
                    'Numbers for that module may be wrong if anything ever wrote an offset into ' +
                        'the CANcoder directly.'
                )
            );
        }
    }

    if (state.capture && state.capture.results) {
        const backwards = state.capture.results.filter((r) => r.verdict.kind === 'bad');
        if (backwards.length === state.capture.results.length && backwards.length > 1) {
            banners.push(
                banner(
                    'bad',
                    `All ${backwards.length} modules look backwards`,
                    'Every module you captured came out about 180&deg; from what is in the ' +
                        'source. That means either the whole robot is pointed backwards right ' +
                        'now, or the offsets currently in the code were taken with the wheels ' +
                        'backwards.',
                    'This exact thing happened between the Aug 2 and Aug 20 2026 alignments. ' +
                        'Work out which end of the robot is the front before writing anything.'
                )
            );
        } else if (backwards.length) {
            banners.push(
                banner(
                    'bad',
                    `${backwards.length} wheel(s) look backwards`,
                    `${backwards
                        .map((r) => r.label)
                        .join(', ')} came out about 180&deg; from the current offset, while the ` +
                        'others did not. That usually means the wheel is physically pointed the ' +
                        'wrong way, or its bevel gear faces the opposite side from the rest.'
                )
            );
        }
    }

    if (state.target && state.target.targetModified) {
        banners.push(
            banner(
                'info',
                'Uncommitted changes already in this file',
                `<code>${state.target.file}</code> has uncommitted edits on branch ` +
                    `<code>${state.target.branch}</code>. Writing offsets will add to them.`
            )
        );
    }

    container.innerHTML = banners.join('');
}

/** Guards the review markup against being rebuilt on every timer tick, which would steal focus. */
let lastReviewSignature = null;

function renderReview() {
    const review = document.getElementById('review');
    const body = document.getElementById('review-body');
    const resolutions = document.getElementById('resolutions');
    const capture = state.capture;

    if (!capture || (!capture.results && !capture.error)) {
        review.hidden = true;
        lastReviewSignature = null;
        return;
    }
    review.hidden = false;

    const signature = JSON.stringify([capture.error, capture.results, state.resolutions]);
    if (signature === lastReviewSignature) return;
    lastReviewSignature = signature;

    if (capture.error) {
        body.innerHTML = `<tr><td colspan="5">${capture.error}</td></tr>`;
        resolutions.innerHTML = '';
        return;
    }

    // Every module gets a row, captured or not, so the table always shows the full set of four
    // numbers that will end up in the file.
    body.innerHTML = MODULES.map((module) => {
        const result = capture.results.find((r) => r.key === module.key);
        const sourceOffset = state.target.offsets[module.key];

        if (!result) {
            return `
            <tr class="skipped">
                <td>${module.label}</td>
                <td>${fmtRotations(sourceOffset)}</td>
                <td>${fmtRotations(sourceOffset)}</td>
                <td>&mdash;</td>
                <td><span class="badge skip">Not captured</span></td>
            </tr>`;
        }

        const final = resolvedOffset(result);
        const flipped = state.resolutions[result.key] === 'flip';
        const shownDelta = flipped
            ? rotationsToDegrees(wrapRotations(final - result.sourceOffset))
            : result.deltaDeg;
        return `
            <tr>
                <td>${result.label}</td>
                <td>${fmtRotations(result.sourceOffset)}</td>
                <td><strong>${fmtRotations(final)}</strong></td>
                <td>${fmtSignedDegrees(shownDelta)}</td>
                <td><span class="badge ${result.verdict.kind}">${result.verdict.label}</span></td>
            </tr>`;
    }).join('');

    resolutions.innerHTML = capture.results
        .filter((result) => result.key in state.resolutions)
        .map((result) => {
            const chosen = state.resolutions[result.key];
            const flippedValue = fmtRotations(wrapRotations(result.newOffset + 0.5));
            const isBackwards = result.verdict.kind === 'bad';
            const options = isBackwards
                ? `
                <label>
                    <input type="radio" name="res-${result.key}" value="accept"
                           ${chosen === 'accept' ? 'checked' : ''} />
                    <span>The pin is in and this wheel really is pointed forward. Use
                    <strong>${fmtRotations(result.newOffset)}</strong>, the old offset was wrong.</span>
                </label>
                <label>
                    <input type="radio" name="res-${result.key}" value="flip"
                           ${chosen === 'flip' ? 'checked' : ''} />
                    <span>This module is pinned backwards. Correct it in software and use
                    <strong>${flippedValue}</strong> instead.</span>
                </label>`
                : `
                <label>
                    <input type="radio" name="res-${result.key}" value="accept"
                           ${chosen === 'accept' ? 'checked' : ''} />
                    <span>I checked the module and this reading is right. Use
                    <strong>${fmtRotations(result.newOffset)}</strong>.</span>
                </label>`;

            const why = isBackwards
                ? 'Roughly a half turn from the current offset.'
                : `${fmtSignedDegrees(result.deltaDeg)} is a bigger jump than a touch-up but not a ` +
                  'half turn. Check that the pin is fully seated and that this module is not ' +
                  'wired to a different CANcoder than the code thinks.';

            return `
            <div class="resolution">
                <h4>${result.label}: ${result.verdict.label}</h4>
                <div class="muted">${why} Re-pinning and capturing again is always fine.</div>
                <div class="options" data-module="${result.key}">${options}</div>
            </div>`;
        })
        .join('');

    for (const input of resolutions.querySelectorAll('input[type="radio"]')) {
        input.addEventListener('change', (event) => {
            const key = event.target.closest('.options').dataset.module;
            state.resolutions[key] = event.target.value;
            render();
        });
    }
}

function renderControls() {
    const captureButton = document.getElementById('capture');
    const captureBlocked = document.getElementById('capture-blocked');
    const writeButton = document.getElementById('write');
    const writeBlocked = document.getElementById('write-blocked');

    const ready = readiness();
    captureButton.disabled = !ready.ready || state.capturing;
    captureButton.textContent = state.capturing ? 'Capturing…' : 'Capture alignment';
    captureBlocked.textContent = ready.ready ? '' : `Waiting on: ${ready.reasons.join('; ')}.`;

    const haveResults = Boolean(state.capture && state.capture.results);
    const resolved = allResolved();
    writeButton.disabled = !haveResults || !resolved;
    if (!haveResults) {
        writeBlocked.textContent = 'Capture first.';
    } else if (!resolved) {
        writeBlocked.textContent = 'Choose what to do about the flagged module(s) above.';
    } else {
        writeBlocked.textContent = '';
    }
}

function render() {
    renderStatus();
    renderBanners();
    renderModules();
    renderReview();
    renderControls();
}

// ---------------------------------------------------------------------------
// Server calls
// ---------------------------------------------------------------------------

async function loadTarget() {
    const response = await fetch('/api/swerve/target');
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || 'Could not read the config file');
    state.target = data;

    const aligned = data.alignedOn ? `last aligned ${data.alignedOn}` : 'no alignment recorded';
    document.getElementById('target-summary').innerHTML =
        `Writes <code>${data.file}</code> &middot; branch <code>${data.branch || '?'}</code> ` +
        `&middot; ${aligned}`;
    document.getElementById('write-target').innerHTML =
        `Updates the four numbers in <code>swerve.configEncoderOffsets(...)</code> in ` +
        `<code>${data.file}</code>. Nothing else in the file is touched.`;
}

async function writeOffsets() {
    const button = document.getElementById('write');
    const result = document.getElementById('write-result');
    button.disabled = true;

    // Modules that were not captured keep whatever is already in the file.
    const payload = { ...state.target.offsets };
    for (const entry of state.capture.results) {
        payload[entry.key] = resolvedOffset(entry);
    }

    try {
        const response = await fetch('/api/swerve/apply', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
        const data = await response.json();
        if (!response.ok) throw new Error(data.error || 'Write failed');

        result.hidden = false;
        result.innerHTML =
            banner('ok', 'Offsets written', `Updated <code>${data.file}</code>.`) +
            `<pre>${escapeHtml(data.after)}</pre>` +
            `<ol class="next-steps">
                <li>Deploy the code to the robot.</li>
                <li>With the pins still in, come back here: every module you captured should now
                    read close to 0&deg;.</li>
                <li><strong>Pull all the alignment pins.</strong> Do not skip this &mdash; enabling
                    with a pin in will break something.</li>
                <li>Enable and drive slowly straight forward. If a wheel fights or the robot
                    crabs, that module is a half turn out.</li>
                <li>Commit the change. To undo instead:
                    <code>git checkout -- ${data.file}</code></li>
            </ol>`;

        // Re-read the file so the "in source" column reflects what is now on disk.
        await loadTarget();
        state.capture = null;
        state.resolutions = {};
    } catch (err) {
        result.hidden = false;
        result.innerHTML = banner('bad', 'Could not write the file', escapeHtml(err.message));
    }
    render();
}

function escapeHtml(text) {
    return String(text).replace(
        /[&<>"']/g,
        (char) =>
            ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[char]
    );
}

// ---------------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------------

function init() {
    const hostInput = document.getElementById('host');
    hostInput.value = localStorage.getItem('swerve-align-host') || '10.85.15.2';

    const connectToInput = () => connect(hostInput.value.trim());
    document.getElementById('connect').addEventListener('click', connectToInput);
    hostInput.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') connectToInput();
    });
    // Picking an entry from the datalist fires change, not keydown.
    hostInput.addEventListener('change', connectToInput);
    document.getElementById('capture').addEventListener('click', startCapture);
    document.getElementById('write').addEventListener('click', writeOffsets);
    for (const input of document.querySelectorAll('#checklist input, #module-select input')) {
        input.addEventListener('change', () => {
            // A capture is only valid for the selection it was taken with.
            if (input.closest('#module-select')) {
                state.capture = null;
                state.resolutions = {};
            }
            render();
        });
    }

    loadTarget()
        .catch((err) => {
            document.getElementById('banners').innerHTML = banner(
                'bad',
                'Could not read the config file',
                escapeHtml(err.message)
            );
        })
        .finally(() => {
            connect(hostInput.value);
            render();
        });

    // Freshness and the "still moving" gate are time-based, so redraw on a timer rather than only
    // when NT sends something.
    setInterval(render, 200);
}

init();
