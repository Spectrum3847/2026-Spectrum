import { Router } from "express";
import fs from "node:fs";
import { probeAll, listRobotLogs, downloadLogs } from "../lib/robot.js";
import { logsDir } from "../lib/config.js";
import { indexLog } from "../lib/manifest.js";

export const robotRouter = Router();

robotRouter.get("/probe", async (req, res) => {
    res.json(await probeAll());
});

robotRouter.get("/logs", async (req, res) => {
    const host = req.query.host;
    if (!host) return res.status(400).json({ error: "host query parameter is required" });
    try {
        res.json(await listRobotLogs(String(host)));
    } catch (e) {
        res.status(502).json({ error: "could not list logs on the robot", detail: String(e.message) });
    }
});

/**
 * Pull logs from the robot. Streams newline-delimited JSON progress events rather than returning
 * one response at the end -- a full match log over the robot radio takes minutes, and a page that
 * shows nothing for that long looks broken.
 */
robotRouter.post("/sync", async (req, res) => {
    const { host, files } = req.body || {};
    if (!host || !Array.isArray(files) || !files.length) {
        return res.status(400).json({ error: "host and a non-empty files array are required" });
    }
    res.writeHead(200, { "content-type": "application/x-ndjson", "cache-control": "no-cache" });
    const emit = (o) => res.write(JSON.stringify(o) + "\n");

    const dest = logsDir();
    fs.mkdirSync(dest, { recursive: true });
    try {
        const done = await downloadLogs(String(host), files, dest, emit);
        for (const f of done) {
            try {
                emit({ name: f.name, state: "indexing" });
                const entry = indexLog(f.name);
                emit({ name: f.name, state: "indexed", summary: entry });
            } catch (e) {
                emit({ name: f.name, state: "index-failed", error: String(e.message) });
            }
        }
        emit({ state: "complete", count: done.length });
    } catch (e) {
        emit({ state: "failed", error: String(e.message) });
    }
    res.end();
});
