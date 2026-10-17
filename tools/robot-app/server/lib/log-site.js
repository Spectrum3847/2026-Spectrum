import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { config, analyzerPath } from "./config.js";

const exec = promisify(execFile);

/**
 * The upload key lives in the user's home folder, never in a repo: both repos are public. Same
 * file as the robot-log-upload skill's set_upload_key.py, so a key saved by either works for both.
 */
function keyFile() {
    const base =
        process.platform === "win32"
            ? process.env.APPDATA || path.join(os.homedir(), "AppData", "Roaming")
            : process.env.XDG_CONFIG_HOME || path.join(os.homedir(), ".config");
    return path.join(base, "spectrum-logs", "upload_key");
}

function readKey() {
    if (process.env.LOG_SITE_KEY) return process.env.LOG_SITE_KEY;
    try {
        return fs.readFileSync(keyFile(), "utf8").trim();
    } catch {
        return "";
    }
}

export function saveKey(key) {
    fs.mkdirSync(path.dirname(keyFile()), { recursive: true });
    fs.writeFileSync(keyFile(), key.trim() + "\n", { mode: 0o600 });
}

function importScript() {
    return path.join(analyzerPath(), "scripts", "import.mjs");
}

export function siteStatus() {
    return {
        url: config.logSite.url,
        analyzerPath: analyzerPath(),
        analyzerFound: fs.existsSync(importScript()),
        keySet: Boolean(readKey()),
    };
}

/**
 * Uploads with the site's own import script, which parses each log with the same code the
 * website runs and skips files already on the site.
 */
export async function uploadLogs(files) {
    if (!fs.existsSync(importScript())) {
        return { ok: false, error: `WPI-Log-Analizer is not cloned at ${analyzerPath()}` };
    }
    const key = readKey();
    if (!key) return { ok: false, error: "no upload key saved" };
    try {
        const { stdout } = await exec(process.execPath, [importScript(), "--url", config.logSite.url, ...files], {
            cwd: analyzerPath(),
            env: { ...process.env, LOG_SITE_KEY: key },
            timeout: 30 * 60 * 1000,
            maxBuffer: 8 << 20,
        });
        return { ok: true, output: stdout };
    } catch (e) {
        return { ok: false, error: (e.stderr || e.message || "").trim(), output: e.stdout || "" };
    }
}
