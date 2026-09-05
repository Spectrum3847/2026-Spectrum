import { Router } from "express";
import fs from "node:fs";
import path from "node:path";
import { logsDir } from "../lib/config.js";
import { readManifest, writeManifest, indexLog, repoStatus, commitAndPush } from "../lib/manifest.js";

export const logsRouter = Router();

function safeLogPath(name) {
    if (!name || !name.endsWith(".wpilog") || name.includes("/") || name.includes("\\")) return null;
    const dir = logsDir();
    const target = path.resolve(dir, name);
    return target.startsWith(dir + path.sep) ? target : null;
}

logsRouter.get("/", (req, res) => {
    const dir = logsDir();
    const manifest = readManifest();
    const byName = new Map(manifest.logs.map((l) => [l.name, l]));
    if (!fs.existsSync(dir)) return res.json({ dir, exists: false, logs: [] });
    const logs = fs
        .readdirSync(dir)
        .filter((f) => f.endsWith(".wpilog"))
        .map((f) => {
            const st = fs.statSync(path.join(dir, f));
            return { name: f, bytes: st.size, mtime: st.mtimeMs, summary: byName.get(f) || null };
        })
        .sort((a, b) => b.mtime - a.mtime);
    res.json({ dir, exists: true, logs });
});

logsRouter.get("/manifest", (req, res) => res.json(readManifest()));

logsRouter.get("/repo-status", async (req, res) => res.json(await repoStatus()));

/** Raw bytes, so the analysis pages can parse the log client-side with the shared parser. */
logsRouter.get("/file/:name", (req, res) => {
    const target = safeLogPath(req.params.name);
    if (!target || !fs.existsSync(target)) return res.status(404).json({ error: "log not found" });
    res.type("application/octet-stream");
    fs.createReadStream(target).pipe(res);
});

logsRouter.post("/:name/index", (req, res) => {
    const target = safeLogPath(req.params.name);
    if (!target || !fs.existsSync(target)) return res.status(404).json({ error: "log not found" });
    try {
        res.json(indexLog(req.params.name));
    } catch (e) {
        res.status(500).json({ error: "could not index log", detail: String(e.message) });
    }
});

logsRouter.post("/:name/pin", (req, res) => {
    const { pinned = true, note = null } = req.body || {};
    const manifest = readManifest();
    const entry = manifest.logs.find((l) => l.name === req.params.name);
    if (!entry) return res.status(404).json({ error: "log is not in the manifest -- index it first" });
    entry.pinned = !!pinned;
    if (note !== null) entry.note = note;
    writeManifest(manifest);
    res.json(entry);
});

logsRouter.post("/publish", async (req, res) => {
    const { message, push = true } = req.body || {};
    res.json(await commitAndPush({ message, push }));
});
