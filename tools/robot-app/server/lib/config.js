import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

export const APP_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../..");

function deepMerge(base, over) {
    for (const [k, v] of Object.entries(over)) {
        base[k] = v && typeof v === "object" && !Array.isArray(v) ? deepMerge({ ...(base[k] || {}) }, v) : v;
    }
    return base;
}

function load() {
    const cfg = JSON.parse(fs.readFileSync(path.join(APP_ROOT, "config.default.json"), "utf8"));
    const localPath = path.join(APP_ROOT, "config.local.json");
    if (fs.existsSync(localPath)) deepMerge(cfg, JSON.parse(fs.readFileSync(localPath, "utf8")));

    const portArg = process.argv.indexOf("--port");
    if (portArg !== -1 && process.argv[portArg + 1]) cfg.port = Number(process.argv[portArg + 1]);
    if (process.env.ROBOT_APP_PORT) cfg.port = Number(process.env.ROBOT_APP_PORT);
    if (process.env.ROBOT_APP_LOGS_DIR) cfg.logsDir = process.env.ROBOT_APP_LOGS_DIR;
    return cfg;
}

export const config = load();

/** Relative paths in config resolve against the app directory, so `npm start` works from anywhere. */
function resolve(p) {
    return path.isAbsolute(p) ? p : path.resolve(APP_ROOT, p);
}

export function logsDir() {
    return resolve(config.logsDir);
}

export function manifestPath() {
    return path.join(logsDir(), "manifest.json");
}

/** The WPI-Log-Analizer clone whose import script does the uploading. */
export function analyzerPath() {
    return resolve(config.logSite.analyzerPath);
}
