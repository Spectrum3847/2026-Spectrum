/*
 * Cameras page.
 *
 * One card per Limelight: its stream, its health, and how it is mounted -- as Vision.java has it,
 * as the camera has saved it, and as the camera can measure it right now from its accelerometer
 * and from any AprilTag it can see. Two calibrators sit under that:
 *
 *  - Measure mount: sample a few seconds on a still robot, solve pitch, roll and height, and write
 *    the result into Vision.java (the source of truth; the robot code pushes it to the camera) and
 *    into the camera's saved pipeline (so it is right on boot, before the code has pushed anything).
 *  - Auto-tune image: sweep exposure, then sensor gain, then black level, scoring each setting by
 *    how steadily the camera sees the tags in front of it, then save the best to the camera.
 *
 * The browser talks to the cameras directly over their HTTP API (lib/limelight.js); only the
 * server touches the .java file (/api/vision). That is the same split the swerve page keeps.
 */
import "../../styles.css";
import "./cameras.css";
import { mountHeader, el, api } from "../../lib/ui.js";
import { LimelightClient, sampleFrames, PIPELINE_MOUNT_KEYS, PIPELINE_IMAGE_KEYS } from "../../lib/limelight.js";
import {
    summarizeMount,
    proposeWrites,
    tagHeightsFromLayout,
    summarizeFrames,
    pickBest,
    scoreMetrics,
    mountFromAccel,
    wrapDeg,
    DEFAULT_SWEEPS,
} from "../../lib/camera-cal.js";

mountHeader();
const root = document.getElementById("app");

const LAYOUT_URL = "/data/apriltag-2026-rebuilt-welded.json";
const POLL_MS = 1000;
const PIPELINE_POLL_MS = 10000;
const MEASURE_SECONDS = 5;
const SWEEP_SETTLE_MS = 700;
const SWEEP_SAMPLE_MS = 1500;

/** How far the saved-on-camera mount may sit from the Java before it is called a mismatch. */
const MOUNT_MISMATCH = { m: 0.003, deg: 0.2 };

/** Live tilt against the configured pitch: fine, worth a look, wrong. */
const TILT_WARN_DEG = 1.0;
const TILT_BAD_DEG = 3.0;

/** Two cameras that both see tags should agree on where the robot is to about this. */
const POSE_DISAGREE_M = 0.15;

const LABELS = { backLeftConfig: "Back left", backRightConfig: "Back right", turretConfig: "Turret" };
const humanize = (key) => LABELS[key] || key.replace(/Config$/, "").replace(/([A-Z])/g, " $1").replace(/^./, (c) => c.toUpperCase());

const fmt = {
    m: (v) => (Number.isFinite(v) ? `${v.toFixed(3)} m` : "—"),
    cm: (v) => (Number.isFinite(v) ? `${(v * 100).toFixed(1)} cm` : "—"),
    deg: (v) => (Number.isFinite(v) ? `${v.toFixed(1)}°` : "—"),
    deg2: (v) => (Number.isFinite(v) ? `${v.toFixed(2)}°` : "—"),
    sdeg: (v) => (Number.isFinite(v) ? `${v >= 0 ? "+" : ""}${v.toFixed(1)}°` : "—"),
    num: (v, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : "—"),
    pct: (v) => (Number.isFinite(v) ? `${Math.round(v * 100)}%` : "—"),
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

const state = {
    target: null, // /api/vision/target
    tagHeights: new Map(),
    layoutError: null,
    cameras: [], // see makeCamera
};

function hostKey(ntName) {
    return `cameras.host.${ntName}`;
}

function makeCamera(cfg) {
    let host = `${cfg.ntName}.local`;
    try {
        host = localStorage.getItem(hostKey(cfg.ntName)) || host;
    } catch {
        // No storage: use the default.
    }
    return {
        key: cfg.key,
        ntName: cfg.ntName,
        config: cfg, // values, units, measuredOn from Vision.java
        host,
        client: new LimelightClient(host),
        status: null,
        results: null,
        pipeline: null,
        pipelineAt: 0,
        error: null,
        lastSeen: 0,
        streamOn: true,
        measuring: false,
        measurement: null,
        tuning: null, // { stage, rows, best, original, aborted }
        dom: {},
    };
}

// ---------------------------------------------------------------------------
// Data loading
// ---------------------------------------------------------------------------

async function loadTarget() {
    state.target = await api("/api/vision/target");
    // Keep per-camera runtime state across reloads of the target file.
    const existing = new Map(state.cameras.map((c) => [c.key, c]));
    state.cameras = state.target.cameras.map((cfg) => {
        const cam = existing.get(cfg.key) || makeCamera(cfg);
        cam.config = cfg;
        return cam;
    });
}

async function loadLayout() {
    try {
        const res = await fetch(LAYOUT_URL);
        if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
        state.tagHeights = tagHeightsFromLayout(await res.json());
    } catch (e) {
        state.layoutError = e.message;
    }
}

async function pollCamera(cam) {
    const [status, results] = await Promise.allSettled([cam.client.status(), cam.client.results()]);
    if (status.status === "fulfilled" && status.value) {
        cam.status = status.value;
        cam.lastSeen = Date.now();
        cam.error = null;
    } else {
        cam.error = status.reason?.message || "unreachable";
    }
    if (results.status === "fulfilled" && results.value) cam.results = results.value;
    if (!cam.error && Date.now() - cam.pipelineAt > PIPELINE_POLL_MS) {
        try {
            cam.pipeline = await cam.client.pipeline(cam.status?.pipelineIndex ?? 0);
            cam.pipelineAt = Date.now();
        } catch {
            // Keep the last one.
        }
    }
}

function setHost(cam, host) {
    cam.host = host.trim();
    cam.client = new LimelightClient(cam.host);
    cam.status = cam.results = cam.pipeline = null;
    cam.pipelineAt = 0;
    cam.error = null;
    try {
        localStorage.setItem(hostKey(cam.ntName), cam.host);
    } catch {
        // Not remembering is fine.
    }
    cam.dom.stream.src = cam.streamOn ? cam.client.streamUrl() : "";
    cam.dom.uiLink.href = cam.client.webUiUrl();
    refreshCard(cam);
}

// ---------------------------------------------------------------------------
// Page skeleton
// ---------------------------------------------------------------------------

const banners = el("div", {});
const poseSection = el("section", {});
const grid = el("div", { class: "cam-grid" });

root.replaceChildren(
    el("h1", {}, "Cameras"),
    el("p", { class: "lede" },
        "Each Limelight, live: what it sees, how it is mounted according to the code and according to itself, and two ",
        "calibrators that write their answers back. The mount comes from the camera's own accelerometer and from the ",
        "AprilTag solve; neither needs to know where the robot is, only that it is sitting still on a flat floor."),
    banners,
    poseSection,
    grid
);

function renderBanners() {
    const items = [];
    const t = state.target;
    if (t) {
        items.push(el("div", { class: "notice" },
            el("strong", {}, "Source of truth: "), el("code", {}, t.file), ` on branch `, el("code", {}, t.branch || "?"),
            ". The robot code pushes these six numbers to each camera every couple of seconds, so a mount fix has to land here, then be deployed. ",
            "Writing here also saves the value into the camera's pipeline so it is right on boot."));
        if (t.targetModified) {
            items.push(el("div", { class: "notice warn" },
                el("strong", {}, "Uncommitted changes already in "), el("code", {}, t.file), ". A write from this page will add to them."));
        }
    }
    if (state.layoutError) {
        items.push(el("div", { class: "notice bad" },
            el("strong", {}, "No field layout. "), `Could not load ${LAYOUT_URL} (${state.layoutError}). Tag heights are unknown, so the tag solve cannot give camera height; pitch and roll still work.`));
    }
    banners.replaceChildren(...items);
}

/** Where each camera thinks the robot is. Cameras with independent mounts agreeing is the yaw check this page cannot solve for. */
function renderPoseSection() {
    const rows = state.cameras.map((cam) => {
        const r = cam.results;
        const tags = r?.botpose_tagcount ?? 0;
        const pose = tags > 0 && Array.isArray(r?.botpose_wpiblue) ? r.botpose_wpiblue : null;
        return { cam, tags, pose };
    });
    const seeing = rows.filter((r) => r.pose);
    let worst = 0;
    for (let i = 0; i < seeing.length; i++) {
        for (let j = i + 1; j < seeing.length; j++) {
            worst = Math.max(worst, Math.hypot(seeing[i].pose[0] - seeing[j].pose[0], seeing[i].pose[1] - seeing[j].pose[1]));
        }
    }
    const verdict =
        seeing.length < 2
            ? el("span", { class: "tag" }, "need two cameras on tags")
            : worst > POSE_DISAGREE_M
              ? el("span", { class: "tag bad" }, `disagree by ${fmt.cm(worst)}`)
              : el("span", { class: "tag ok" }, `agree within ${fmt.cm(worst)}`);

    poseSection.replaceChildren(
        el("h2", {}, "Do the cameras agree on where the robot is? ", verdict),
        el("div", { class: "card" },
            el("div", { class: "table-wrap" },
                el("table", { class: "pose-table" },
                    el("thead", {}, el("tr", {}, ["Camera", "Tags", "X (blue origin)", "Y", "Heading", "MegaTag1 std dev"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, rows.map(({ cam, tags, pose }) =>
                        el("tr", {},
                            el("td", {}, humanize(cam.key)),
                            el("td", { class: "num" }, String(tags)),
                            el("td", { class: "num" }, pose ? fmt.m(pose[0]) : "—"),
                            el("td", { class: "num" }, pose ? fmt.m(pose[1]) : "—"),
                            el("td", { class: "num" }, pose ? fmt.deg(pose[5]) : "—"),
                            el("td", { class: "num" }, cam.results?.stdev_mt1 && tags > 0 ? `${fmt.cm(Math.hypot(cam.results.stdev_mt1[0], cam.results.stdev_mt1[1]))}` : "—")))))),
            el("div", { class: "footnote" },
                "Every camera solves the robot's pose through its own mount transform, so two cameras seeing tags at once is a test of both mounts at once. ",
                "A yaw or sideways error in one mount shows up here as a disagreement that grows with distance, and this page cannot fix it: ",
                "forward, right and yaw need a surveyed robot position and come from CAD.")));
}

// ---------------------------------------------------------------------------
// Camera card
// ---------------------------------------------------------------------------

function buildCard(cam) {
    const d = cam.dom;
    d.hostInput = el("input", { type: "text", value: cam.host, spellcheck: "false", title: "Hostname or IP of this camera" });
    const useBtn = el("button", { onclick: () => setHost(cam, d.hostInput.value) }, "Use");
    d.hostInput.addEventListener("keydown", (e) => e.key === "Enter" && setHost(cam, d.hostInput.value));
    d.uiLink = el("a", { href: cam.client.webUiUrl(), target: "_blank", rel: "noopener" }, "web UI ↗");
    const streamToggle = el("input", { type: "checkbox", checked: cam.streamOn });
    streamToggle.addEventListener("change", () => {
        cam.streamOn = streamToggle.checked;
        d.stream.src = cam.streamOn ? cam.client.streamUrl() : "";
        d.stream.classList.toggle("hidden", !cam.streamOn);
    });

    d.stream = el("img", { class: "stream", src: cam.streamOn ? cam.client.streamUrl() : "", alt: `${cam.ntName} stream` });
    d.stream.addEventListener("error", () => d.stream.classList.add("hidden"));
    d.stream.addEventListener("load", () => d.stream.classList.toggle("hidden", !cam.streamOn));

    d.chips = el("div", { class: "chips" });
    d.mountTable = el("div", { class: "table-wrap" });
    d.tilt = el("div", { class: "chips" });

    // Measure mount
    d.checks = ["The robot is sitting still and nobody is touching it", "It is on a flat floor, all four wheels down", "For height: at least one AprilTag is in this camera's view"].map((t) =>
        el("li", {}, el("label", {}, el("input", { type: "checkbox" }), el("span", {}, t))));
    d.measureBtn = el("button", { class: "primary", onclick: () => measureMount(cam) }, `Measure mount (${MEASURE_SECONDS} s)`);
    d.measureProgress = el("div", { class: "progress" });
    d.measureOut = el("div", {});

    // Image tuning
    d.sweep = {};
    for (const k of PIPELINE_IMAGE_KEYS) d.sweep[k] = el("input", { type: "text", value: DEFAULT_SWEEPS[k].join(", ") });
    d.tuneBtn = el("button", { class: "primary", onclick: () => autoTune(cam) }, "Auto-tune image");
    d.abortBtn = el("button", { disabled: true, onclick: () => cam.tuning?.abort?.() }, "Stop");
    d.tuneProgress = el("div", { class: "progress" });
    d.tuneOut = el("div", {});
    d.imageNow = el("div", { class: "chips" });

    const card = el("div", { class: "card cam-card" },
        el("h2", {}, humanize(cam.key)),
        el("div", { class: "ntname" }, cam.ntName),
        el("div", { class: "host-row" }, d.hostInput, useBtn, d.uiLink, el("span", { class: "spacer" }), el("label", {}, streamToggle, "stream")),
        d.stream,
        d.chips,
        el("h3", {}, "Mount"),
        d.mountTable,
        d.tilt,
        el("h3", {}, "Measure the mount"),
        el("p", { class: "hint" },
            "Solves pitch, roll and height from the camera's accelerometer and from every tag it can see, then offers to write them. ",
            "Tick these first; none of them can be checked from here."),
        el("ul", { class: "checklist" }, d.checks),
        el("div", { class: "row" }, d.measureBtn, d.measureProgress),
        d.measureOut,
        el("h3", {}, "Image"),
        d.imageNow,
        el("p", { class: "hint" },
            "Sweeps exposure, then sensor gain, then black level, holding each setting for ",
            `${(SWEEP_SAMPLE_MS / 1000).toFixed(1)} s and scoring how steadily the tags in view are detected. `,
            "Point the camera at tags at a realistic range first. Nothing is saved until you say so, and the original settings come back when the sweep ends."),
        el("div", { class: "sweep-inputs" },
            el("span", {}, "Exposure (0.01 ms)"), d.sweep.exposure,
            el("span", {}, "Sensor gain"), d.sweep.lcgain,
            el("span", {}, "Black level"), d.sweep.black_level),
        el("div", { class: "row" }, d.tuneBtn, d.abortBtn, d.tuneProgress),
        d.tuneOut
    );
    d.card = card;
    return card;
}

const chip = (label, value, cls) => el("span", { class: `chip ${cls || ""}` }, `${label} `, el("b", {}, value));

function refreshCard(cam) {
    const d = cam.dom;
    const s = cam.status;
    const r = cam.results;
    const fresh = Date.now() - cam.lastSeen < POLL_MS * 3;

    // Health chips
    const chips = [];
    if (!fresh) chips.push(chip("", cam.error ? `unreachable: ${cam.error}` : "connecting…", cam.error ? "bad" : ""));
    if (s && fresh) {
        chips.push(chip("", s.name === cam.ntName ? "reachable" : `reachable, but calls itself ${s.name}`, s.name === cam.ntName ? "ok" : "warn"));
        chips.push(chip("fps", fmt.num(s.fps, 0), s.fps < 30 ? "warn" : ""));
        chips.push(chip("temp", `${fmt.num(s.temp, 0)} °C`, s.temp > 80 ? "bad" : s.temp > 70 ? "warn" : ""));
        chips.push(chip("cpu", fmt.pct(s.cpu / 100)));
        chips.push(chip("pipeline", `${s.pipelineIndex} ${s.pipelineType || ""}`));
        if (r) {
            chips.push(chip("NT", r.ntconnected ? "connected" : "no robot", r.ntconnected ? "ok" : "warn"));
            chips.push(chip("tags", String((r.Fiducial || []).length)));
            if (r.imu) chips.push(chip("IMU mode", String(r.botorient?.imumode ?? "?")));
        }
    }
    d.chips.replaceChildren(...chips);

    // Mount table: Java vs saved on camera vs live on camera
    const cfg = cam.config.values;
    const saved = cam.pipeline;
    const live = Array.isArray(r?.t6c_rs) && r.t6c_rs.length >= 6 ? r.t6c_rs : null;
    const order = ["forward", "right", "up", "roll", "pitch", "yaw"];
    const rows = order.map((k, i) => {
        const isAngle = i >= 3;
        const f = isAngle ? fmt.deg : fmt.m;
        const savedV = saved ? Number(saved[PIPELINE_MOUNT_KEYS[k]]) : NaN;
        const liveV = live ? live[i] : NaN;
        const tol = isAngle ? MOUNT_MISMATCH.deg : MOUNT_MISMATCH.m;
        // The camera echoes yaw with the opposite sign to what was set (seen on all three cameras
        // on 2026-09-07; both cameras still agreed on the robot pose, so it is a reporting
        // convention, not an error). Compare yaw by magnitude.
        const cmp = (a, b) => (k === "yaw" ? Math.abs(Math.abs(a) - Math.abs(b)) : isAngle ? Math.abs(wrapDeg(a - b)) : Math.abs(a - b));
        const savedBad = Number.isFinite(savedV) && cmp(savedV, cfg[k]) > tol;
        const liveBad = Number.isFinite(liveV) && cmp(liveV, cfg[k]) > tol;
        // The turret's yaw is the live turret angle, not the config's 0.
        const turretYaw = cam.key === "turretConfig" && k === "yaw";
        return el("tr", {},
            el("td", {}, k),
            el("td", { class: "num" }, f(cfg[k])),
            el("td", { class: `num ${savedBad && !turretYaw ? "mismatch" : ""}` }, f(savedV)),
            el("td", { class: `num ${liveBad && !turretYaw ? "mismatch" : "dim"}` }, f(liveV)));
    });
    d.mountTable.replaceChildren(
        el("table", {},
            el("thead", {}, el("tr", {}, ["", "Vision.java", "saved on camera", "live on camera"].map((h) => el("th", {}, h)))),
            el("tbody", {}, rows)),
        el("div", { class: "footnote" },
            cam.config.measuredOn ? `Last written by this page on ${cam.config.measuredOn}. ` : "",
            "Yaw reads back with the opposite sign to what is set, on all three cameras; compared by magnitude. ",
            cam.key === "turretConfig" ? "The turret camera's yaw is the live turret angle, pushed every loop." : ""));

    // Live tilt from the accelerometer
    const accel = r?.imu?.data?.slice(7, 10);
    if (accel && accel.length === 3 && fresh) {
        const m = mountFromAccel(accel);
        const dPitch = m.pitchDeg - cfg.pitch;
        const cls = Math.abs(dPitch) > TILT_BAD_DEG ? "bad" : Math.abs(dPitch) > TILT_WARN_DEG ? "warn" : "ok";
        d.tilt.replaceChildren(
            chip("accelerometer pitch", fmt.deg(m.pitchDeg), cls),
            chip("vs Vision.java", fmt.sdeg(dPitch), cls),
            chip("roll", fmt.deg(m.rollDeg)),
            chip("|g|", fmt.num(m.gMagnitude, 2), Math.abs(m.gMagnitude - 1) > 0.1 ? "warn" : ""));
    } else {
        d.tilt.replaceChildren();
    }

    // Current image settings
    if (saved) {
        d.imageNow.replaceChildren(
            chip("exposure", `${saved.exposure} (${(saved.exposure / 100).toFixed(2)} ms)`),
            chip("gain", String(saved.lcgain)),
            chip("black level", String(saved.black_level)),
            chip("flip", String(saved.image_flip)),
            chip("res", String(saved.pipeline_res)));
    }

    d.measureBtn.disabled = cam.measuring || !fresh;
    d.tuneBtn.disabled = Boolean(cam.tuning) || !fresh || !saved;
}

// ---------------------------------------------------------------------------
// Measure mount
// ---------------------------------------------------------------------------

async function measureMount(cam) {
    const d = cam.dom;
    const unchecked = d.checks.filter((li) => !li.querySelector("input").checked).length;
    if (unchecked) {
        d.measureProgress.textContent = `${unchecked} checklist item${unchecked === 1 ? "" : "s"} left.`;
        return;
    }
    cam.measuring = true;
    cam.measurement = null;
    d.measureOut.replaceChildren();
    refreshCard(cam);
    try {
        const frames = await sampleFrames(cam.client, MEASURE_SECONDS * 1000, {
            onFrame: (f, n) => (d.measureProgress.textContent = `sampling… ${n} frames, ${(f.Fiducial || []).length} tag${(f.Fiducial || []).length === 1 ? "" : "s"} in view`),
        });
        cam.measurement = summarizeMount(frames, state.tagHeights);
        d.measureProgress.textContent = `${frames.length} frames.`;
    } catch (e) {
        d.measureProgress.textContent = `Failed: ${e.message}`;
    } finally {
        cam.measuring = false;
    }
    renderMeasurement(cam);
    refreshCard(cam);
}

function renderMeasurement(cam) {
    const d = cam.dom;
    const m = cam.measurement;
    if (!m) return;
    const cfg = cam.config.values;

    const row = (label, n, pitch, pitchStd, roll, height, heightStd, extra) =>
        el("tr", {},
            el("td", {}, label),
            el("td", { class: "num" }, String(n)),
            el("td", { class: "num" }, pitch === null ? "—" : `${fmt.deg2(pitch)}${Number.isFinite(pitchStd) ? ` ± ${pitchStd.toFixed(2)}` : ""}`),
            el("td", { class: "num" }, roll === null ? "—" : fmt.deg(roll)),
            el("td", { class: "num" }, height === null ? "—" : `${fmt.m(height)}${Number.isFinite(heightStd) ? ` ± ${(heightStd * 1000).toFixed(0)} mm` : ""}`),
            el("td", { class: "dim" }, extra || ""));

    const rows = [];
    if (m.accel) rows.push(row("Accelerometer", m.accel.n, m.accel.pitchDeg, m.accel.pitchStd, m.accel.rollDeg, null, null, "no height; roll sign unverified"));
    for (const t of m.tags) rows.push(row(`Tag ${t.id}`, t.n, t.pitchDeg, t.pitchStd, t.rollDeg, t.heightM, t.heightStd, `range ${fmt.m(t.rangeM)}, ambiguity ${fmt.num(t.ambig, 2)}`));
    if (m.tagSummary) rows.push(row(`All tags (${m.tagSummary.tagCount})`, m.tagSummary.n, m.tagSummary.pitchDeg, m.tagSummary.pitchStd, m.tagSummary.rollDeg, m.tagSummary.heightM, m.tagSummary.heightStd, "what gets proposed"));
    rows.push(row("Vision.java now", "", cfg.pitch, NaN, cfg.roll, cfg.up, NaN, ""));

    const notes = [];
    if (!m.accel && !m.tagSummary) notes.push(el("div", { class: "notice bad" }, "Nothing usable came back. Is the camera reachable, and is a tag in view?"));
    if (m.agreementDeg !== null && m.agreementDeg > 1.5) {
        notes.push(el("div", { class: "notice warn" },
            el("strong", {}, `Accelerometer and tags disagree by ${fmt.deg(m.agreementDeg)}. `),
            "Either the robot is not flat, the tag is not vertical, or the IMU is off. The tag figure is proposed below; look before writing."));
    }
    if (m.tags.length && !m.tagSummary) notes.push(el("div", { class: "notice warn" }, "Too few clean tag frames to trust; only the accelerometer is proposed. Get closer or hold still longer."));
    if (m.tags.length === 0 && m.accel) notes.push(el("div", { class: "notice" }, "No tag in view, so height cannot be measured. Pitch comes from the accelerometer alone."));

    // Proposal
    const proposal = m.proposed ? proposeWrites(cfg, m.proposed) : [];
    const boxes = {};
    const propRows = proposal.map((p) => {
        const f = p.fmt === "m" ? fmt.m : fmt.deg;
        const box = el("input", { type: "checkbox", checked: p.significant && !(p.key === "roll" && m.proposed.source === "accel") });
        boxes[p.key] = box;
        return el("tr", {},
            el("td", {}, box),
            el("td", {}, p.label),
            el("td", { class: "num" }, f(p.from)),
            el("td", { class: "num" }, el("strong", {}, f(p.to))),
            el("td", { class: "num" }, p.fmt === "m" ? `${p.delta >= 0 ? "+" : ""}${(p.delta * 1000).toFixed(0)} mm` : fmt.sdeg(p.delta)),
            el("td", {}, el("span", { class: `tag ${p.significant ? "warn" : "ok"}` }, p.significant ? "differs" : "matches")));
    });

    const saveToCamera = el("input", { type: "checkbox", checked: true });
    const writeBtn = el("button", { class: "primary" }, "Write to Vision.java");
    const outcome = el("div", { class: "progress" });
    writeBtn.addEventListener("click", async () => {
        const values = {};
        for (const p of proposal) if (boxes[p.key].checked) values[p.key] = Number(p.to.toFixed(p.fmt === "m" ? 4 : 2));
        if (!Object.keys(values).length) {
            outcome.textContent = "Nothing ticked.";
            return;
        }
        writeBtn.disabled = true;
        outcome.textContent = "writing…";
        try {
            const res = await api("/api/vision/apply", { method: "POST", body: JSON.stringify({ camera: cam.key, ...values }) });
            let camNote = "";
            if (saveToCamera.checked) {
                const partial = {};
                for (const k of Object.keys(values)) partial[PIPELINE_MOUNT_KEYS[k]] = res.camera.values[k];
                try {
                    await cam.client.updatePipeline(partial, true);
                    cam.pipelineAt = 0;
                    camNote = " Saved to the camera's pipeline too.";
                } catch (e) {
                    camNote = ` The Java is written but the camera refused the save: ${e.message}`;
                }
            }
            outcome.textContent = `Wrote ${res.changed.join(", ")} to ${res.file}.${camNote} Deploy for the robot to push it.`;
            await loadTarget();
            renderBanners();
            refreshCard(cam);
        } catch (e) {
            outcome.textContent = `Write failed: ${e.message}`;
            writeBtn.disabled = false;
        }
    });

    d.measureOut.replaceChildren(
        el("div", { class: "table-wrap" },
            el("table", {},
                el("thead", {}, el("tr", {}, ["Method", "Frames", "Pitch", "Roll", "Height", ""].map((h) => el("th", {}, h)))),
                el("tbody", {}, rows))),
        ...notes,
        proposal.length
            ? el("div", {},
                  el("h3", {}, `Proposed from ${m.proposed.source === "tags" ? "the tag solve" : "the accelerometer"}`),
                  el("div", { class: "table-wrap" },
                      el("table", {},
                          el("thead", {}, el("tr", {}, ["Write", "Field", "Vision.java", "Measured", "Change", ""].map((h) => el("th", {}, h)))),
                          el("tbody", {}, propRows))),
                  el("div", { class: "row", style: "margin-top:10px" }, writeBtn, el("label", { class: "hint", style: "margin:0" }, saveToCamera, " also save to the camera")),
                  outcome,
                  el("div", { class: "footnote" },
                      "Roll from the accelerometer is a magnitude with an unverified sign, so it is unticked unless a tag confirmed it. ",
                      "Forward, right and yaw are not offered: they need a surveyed robot position."))
            : null
    );
}

// ---------------------------------------------------------------------------
// Auto-tune image
// ---------------------------------------------------------------------------

function parseCandidates(text, fallback) {
    const vals = text.split(/[,\s]+/).map(Number).filter(Number.isFinite);
    return vals.length ? vals : fallback;
}

async function autoTune(cam) {
    const d = cam.dom;
    const original = {};
    for (const k of PIPELINE_IMAGE_KEYS) original[k] = cam.pipeline[k];

    const controller = new AbortController();
    cam.tuning = { stages: [], original, abort: () => controller.abort() };
    d.abortBtn.disabled = false;
    d.tuneOut.replaceChildren();
    refreshCard(cam);

    const current = { ...original };
    const restore = () => cam.client.updatePipeline(original, false).catch(() => {});
    const onUnload = () => cam.client.updatePipeline(original, false).catch(() => {});
    window.addEventListener("beforeunload", onUnload);

    try {
        for (const key of PIPELINE_IMAGE_KEYS) {
            if (controller.signal.aborted) break;
            const candidates = parseCandidates(d.sweep[key].value, DEFAULT_SWEEPS[key]);
            const stage = { key, rows: [], best: null };
            cam.tuning.stages.push(stage);
            for (const value of candidates) {
                if (controller.signal.aborted) break;
                d.tuneProgress.textContent = `${key} = ${value} … settling`;
                await cam.client.updatePipeline({ ...current, [key]: value }, false);
                await new Promise((r) => setTimeout(r, SWEEP_SETTLE_MS));
                d.tuneProgress.textContent = `${key} = ${value} … sampling`;
                const frames = await sampleFrames(cam.client, SWEEP_SAMPLE_MS, { signal: controller.signal });
                const metrics = summarizeFrames(frames);
                const exposure = key === "exposure" ? value : current.exposure;
                stage.rows.push({ value, metrics, exposure, score: metrics.frames ? scoreMetrics(metrics, exposure) : -Infinity });
                renderTuning(cam);
            }
            stage.best = pickBest(stage.rows);
            if (stage.best) current[key] = stage.best.value;
            renderTuning(cam);
        }
        cam.tuning.result = current;
        cam.tuning.aborted = controller.signal.aborted;
    } catch (e) {
        cam.tuning.error = e.message;
    } finally {
        await restore();
        window.removeEventListener("beforeunload", onUnload);
        cam.pipelineAt = 0;
        d.abortBtn.disabled = true;
        d.tuneProgress.textContent = cam.tuning.error ? `Failed: ${cam.tuning.error}` : cam.tuning.aborted ? "Stopped; original settings restored." : "Done; original settings restored until you apply.";
        cam.tuning.done = true;
        renderTuning(cam);
        cam.tuning = null;
        refreshCard(cam);
    }
}

function renderTuning(cam) {
    const d = cam.dom;
    const t = cam.tuning;
    if (!t) return;
    const sections = t.stages.map((stage) =>
        el("div", {},
            el("h3", {}, `${stage.key} sweep`),
            el("div", { class: "table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {}, [stage.key, "frames", "seen", "tags", "ambiguity", "corner px", "pose", "latency", "score"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, stage.rows.map((r) =>
                        el("tr", { class: stage.best && stage.best.value === r.value ? "best" : "" },
                            el("td", { class: "num" }, String(r.value)),
                            el("td", { class: "num" }, String(r.metrics.frames)),
                            el("td", { class: "num" }, fmt.pct(r.metrics.detectRate)),
                            el("td", { class: "num" }, fmt.num(r.metrics.meanTags, 1)),
                            el("td", { class: "num" }, fmt.num(r.metrics.meanAmbig, 2)),
                            el("td", { class: "num" }, fmt.num(r.metrics.cornerJitterPx, 2)),
                            el("td", { class: "num" }, r.metrics.poseJitterM === null ? "—" : fmt.cm(r.metrics.poseJitterM)),
                            el("td", { class: "num" }, r.metrics.latencyMs === null ? "—" : `${fmt.num(r.metrics.latencyMs, 0)} ms`),
                            el("td", { class: "num" }, Number.isFinite(r.score) ? r.score.toFixed(1) : "—"))))))));

    if (t.done && t.result && !t.error) {
        const changed = PIPELINE_IMAGE_KEYS.filter((k) => t.result[k] !== t.original[k]);
        const applyBtn = el("button", { class: "primary", disabled: !changed.length }, "Apply and save to camera");
        const outcome = el("div", { class: "progress" });
        applyBtn.addEventListener("click", async () => {
            applyBtn.disabled = true;
            outcome.textContent = "saving…";
            try {
                await cam.client.updatePipeline(t.result, true);
                cam.pipelineAt = 0;
                outcome.textContent = `Saved ${PIPELINE_IMAGE_KEYS.map((k) => `${k}=${t.result[k]}`).join(", ")} to the camera's flash.`;
            } catch (e) {
                outcome.textContent = `Save failed: ${e.message}`;
                applyBtn.disabled = false;
            }
        });
        sections.push(
            el("div", { class: `notice ${changed.length ? "" : "warn"}` },
                el("strong", {}, changed.length ? "Best found: " : "Nothing better than the current settings. "),
                PIPELINE_IMAGE_KEYS.map((k) => `${k} ${t.original[k]} → ${t.result[k]}`).join(" · "),
                el("div", { class: "row", style: "margin-top:8px" }, applyBtn),
                outcome)
        );
    }
    d.tuneOut.replaceChildren(...sections);
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

try {
    await Promise.all([loadTarget(), loadLayout()]);
} catch (e) {
    root.append(el("div", { class: "notice bad" }, el("strong", {}, "Could not read the camera config from the server: "), e.message));
}
renderBanners();
grid.replaceChildren(...state.cameras.map(buildCard));
renderPoseSection();

async function tick() {
    await Promise.all(state.cameras.map(pollCamera));
    for (const cam of state.cameras) refreshCard(cam);
    renderPoseSection();
    setTimeout(tick, POLL_MS);
}
tick();
