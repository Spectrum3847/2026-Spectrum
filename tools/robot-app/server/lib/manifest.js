import fs from "node:fs";
import path from "node:path";
import { config, logsDir, manifestPath } from "./config.js";
import { summarize } from "./summary.js";

export function readManifest() {
    const p = manifestPath();
    if (!fs.existsSync(p)) return { version: 1, robot: config.robotProfile, logs: [] };
    try {
        return JSON.parse(fs.readFileSync(p, "utf8"));
    } catch {
        return { version: 1, robot: config.robotProfile, logs: [], corrupt: true };
    }
}

export function writeManifest(manifest) {
    if (manifest.corrupt) {
        throw new Error(`${manifestPath()} is not valid JSON; fix or delete it before indexing more logs`);
    }
    fs.mkdirSync(logsDir(), { recursive: true });
    manifest.logs.sort((a, b) => (a.name < b.name ? 1 : -1));
    const tmp = `${manifestPath()}.tmp`;
    fs.writeFileSync(tmp, JSON.stringify(manifest, null, 2) + "\n");
    try {
        fs.renameSync(tmp, manifestPath());
    } catch (e) {
        fs.rmSync(tmp, { force: true });
        throw e;
    }
}

/** Summarize a synced log and fold it into the manifest, replacing any prior entry. */
export function indexLog(name, { note = null } = {}) {
    const file = path.join(logsDir(), name);
    const st = fs.statSync(file);
    const summary = summarize(fs.readFileSync(file), { name });
    const entry = {
        ...summary,
        bytes: st.size,
        syncedAt: new Date().toISOString(),
        note,
    };
    const manifest = readManifest();
    const i = manifest.logs.findIndex((l) => l.name === name);
    if (i >= 0) {
        // Re-indexing must not silently drop a note someone added by hand.
        entry.note = note ?? manifest.logs[i].note;
        manifest.logs[i] = entry;
    } else {
        manifest.logs.push(entry);
    }
    writeManifest(manifest);
    return entry;
}
