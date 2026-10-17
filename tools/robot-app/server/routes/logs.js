import { Router } from "express";
import fs from "node:fs";
import path from "node:path";
import { logsDir } from "../lib/config.js";
import { readManifest, indexLog } from "../lib/manifest.js";
import { siteStatus, saveKey, uploadLogs } from "../lib/log-site.js";

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

logsRouter.get("/site", (req, res) => res.json(siteStatus()));

/** Raw bytes, so the analysis pages can parse the log client-side with the shared parser. */
logsRouter.get("/file/:name", (req, res) => {
    const target = safeLogPath(req.params.name);
    if (!target || !fs.existsSync(target)) return res.status(404).json({ error: "log not found" });
    res.sendFile(target, { headers: { "content-type": "application/octet-stream" } });
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

logsRouter.post("/site/key", (req, res) => {
    const key = String(req.body?.key || "").trim();
    if (!key) return res.status(400).json({ error: "key is empty" });
    saveKey(key);
    res.json(siteStatus());
});

logsRouter.post("/upload", async (req, res) => {
    const dir = logsDir();
    const files = fs.existsSync(dir) ? fs.readdirSync(dir).filter((f) => f.endsWith(".wpilog")).map((f) => path.join(dir, f)) : [];
    if (!files.length) return res.json({ ok: true, output: "No synced logs to upload." });
    res.json(await uploadLogs(files));
});
