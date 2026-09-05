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
    if (process.env.ROBOT_APP_LOGS_REPO) cfg.logsRepo.path = process.env.ROBOT_APP_LOGS_REPO;
    return cfg;
}

export const config = load();

/**
 * Absolute path to the logs repo clone. Relative paths in config resolve against the app
 * directory rather than the shell's cwd, so `npm start` behaves the same from anywhere.
 */
export function logsRepoPath() {
    const p = config.logsRepo.path;
    return path.isAbsolute(p) ? p : path.resolve(APP_ROOT, p);
}

export function logsDir() {
    return path.join(logsRepoPath(), config.logsRepo.logsSubdir);
}

export function manifestPath() {
    return path.join(logsRepoPath(), config.logsRepo.manifest);
}
