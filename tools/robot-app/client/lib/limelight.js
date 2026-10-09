/**
 * Talks to a Limelight's own HTTP API, straight from the browser.
 *
 * Limelight OS serves a small REST API on port 5807 with `Access-Control-Allow-Origin: *`, so the
 * page can call it directly; the app server never sits between the browser and a camera, the same
 * split the swerve page keeps for NetworkTables. Confirmed against the three Limelight 4s on this
 * robot on 2026-09-07:
 *
 *   GET  /status                      name, fps, temp, cpu, pipeline index and type, IMU summary
 *   GET  /results                     the full frame result: fiducials with per-tag poses, botpose
 *                                     variants, the IMU sample, hardware stats, frame index `fidx`
 *   GET  /hwreport                    intrinsics and distortion for each calibration slot
 *   GET  /pipeline-atindex?index=N    the saved pipeline: exposure, lcgain (sensor gain),
 *                                     black_level, and the mount as rsf/rss/rsu/rsroll/rspitch/rsyaw
 *   GET  /pipeline-default?index=N    factory values for the same keys
 *   POST /update-pipeline?flush=0|1   body is a JSON object of pipeline keys to change; flush=1
 *                                     also writes it to the camera's flash. Replies "update complete".
 *
 * An unknown path closes the connection rather than returning 404, so every call has a timeout.
 *
 * The camera stream is a plain MJPEG on port 5800 and the web UI is on 5801.
 */

const API_PORT = 5807;
const STREAM_PORT = 5800;
const UI_PORT = 5801;
const TIMEOUT_MS = 2500;

export class LimelightClient {
    /** @param {string} host hostname or IP, e.g. "limelight-left.local" or "10.85.15.11" */
    constructor(host) {
        this.host = host;
    }

    url(path) {
        return `http://${this.host}:${API_PORT}/${path}`;
    }

    streamUrl() {
        return `http://${this.host}:${STREAM_PORT}/stream.mjpg`;
    }

    webUiUrl() {
        return `http://${this.host}:${UI_PORT}/`;
    }

    async getJson(path) {
        const res = await fetch(this.url(path), { signal: AbortSignal.timeout(TIMEOUT_MS), cache: "no-store" });
        if (!res.ok) throw new Error(`${this.host}: ${path} returned ${res.status}`);
        const text = await res.text();
        if (!text) return null;
        return JSON.parse(text);
    }

    status() {
        return this.getJson("status");
    }

    results() {
        return this.getJson("results");
    }

    hwreport() {
        return this.getJson("hwreport");
    }

    pipeline(index = 0) {
        return this.getJson(`pipeline-atindex?index=${index}`);
    }

    pipelineDefault(index = 0) {
        return this.getJson(`pipeline-default?index=${index}`);
    }

    /**
     * Changes settings on the running pipeline.
     *
     * @param {object} partial pipeline keys to change, e.g. { exposure: 400 }
     * @param {boolean} flush whether to also save to flash; without it the change lasts until the
     *     camera reboots, which is what a sweep wants and what a calibration does not
     */
    async updatePipeline(partial, flush = false) {
        const res = await fetch(this.url(`update-pipeline?flush=${flush ? 1 : 0}`), {
            method: "POST",
            headers: { "content-type": "application/json" },
            body: JSON.stringify(partial),
            signal: AbortSignal.timeout(TIMEOUT_MS),
            // Lets a restore go out while the page is being closed.
            keepalive: true,
        });
        const text = await res.text();
        if (!res.ok || !/update complete/i.test(text)) throw new Error(`${this.host}: pipeline update failed: ${text || res.status}`);
        return text;
    }
}

/**
 * Polls /results and hands back distinct frames.
 *
 * The camera runs at 50-60 fps and the poll is slower than that, so frames are never
 * duplicated by fetching too fast, but a stalled camera would repeat the same `fidx` and this
 * drops those repeats so a frozen camera cannot look like a steady one.
 *
 * @param {LimelightClient} client
 * @param {number} durationMs how long to sample
 * @param {{intervalMs?: number, onFrame?: function, signal?: AbortSignal}} [opts]
 * @returns {Promise<object[]>} distinct /results documents
 */
export async function sampleFrames(client, durationMs, opts = {}) {
    const intervalMs = opts.intervalMs ?? 60;
    const frames = [];
    let lastIdx = null;
    const until = Date.now() + durationMs;
    while (Date.now() < until) {
        if (opts.signal?.aborted) break;
        const started = Date.now();
        try {
            const r = await client.results();
            if (r && r.fidx !== lastIdx) {
                lastIdx = r.fidx;
                frames.push(r);
                opts.onFrame?.(r, frames.length);
            }
        } catch {
            // A missed poll is not a missed frame; keep sampling.
        }
        const wait = intervalMs - (Date.now() - started);
        if (wait > 0) await new Promise((r) => setTimeout(r, wait));
    }
    return frames;
}

/** Pipeline keys that describe the camera's mount, in Vision.java's order and units. */
export const PIPELINE_MOUNT_KEYS = { forward: "rsf", right: "rss", up: "rsu", roll: "rsroll", pitch: "rspitch", yaw: "rsyaw" };

/** Pipeline keys the image tuner sweeps. */
export const PIPELINE_IMAGE_KEYS = ["exposure", "lcgain", "black_level"];
