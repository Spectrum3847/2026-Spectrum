import fs from "node:fs";
import path from "node:path";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { config, logsRepoPath, logsDir, manifestPath } from "./config.js";
import { summarize } from "./summary.js";

const exec = promisify(execFile);

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
    fs.mkdirSync(logsRepoPath(), { recursive: true });
    manifest.logs.sort((a, b) => (a.name < b.name ? 1 : -1));
    fs.writeFileSync(manifestPath(), JSON.stringify(manifest, null, 2) + "\n");
}

/** Summarize a synced log and fold it into the manifest, replacing any prior entry. */
export function indexLog(name, { pinned = false, note = null } = {}) {
    const file = path.join(logsDir(), name);
    const st = fs.statSync(file);
    const summary = summarize(fs.readFileSync(file), { name });
    const entry = {
        ...summary,
        bytes: st.size,
        syncedAt: new Date().toISOString(),
        pinned,
        note,
    };
    const manifest = readManifest();
    const i = manifest.logs.findIndex((l) => l.name === name);
    if (i >= 0) {
        // Re-indexing must not silently drop a pin or a note someone added by hand.
        entry.pinned = entry.pinned || manifest.logs[i].pinned;
        entry.note = note ?? manifest.logs[i].note;
        manifest.logs[i] = entry;
    } else {
        manifest.logs.push(entry);
    }
    writeManifest(manifest);
    return entry;
}

async function git(args) {
    try {
        const { stdout, stderr } = await exec("git", args, { cwd: logsRepoPath(), timeout: 120000, maxBuffer: 8 << 20 });
        return { ok: true, stdout, stderr };
    } catch (e) {
        return { ok: false, stdout: e.stdout || "", stderr: e.stderr || String(e.message) };
    }
}

export async function repoStatus() {
    const repo = logsRepoPath();
    if (!fs.existsSync(path.join(repo, ".git"))) {
        return { path: repo, isRepo: false, remote: config.logsRepo.remote };
    }
    const branch = await git(["rev-parse", "--abbrev-ref", "HEAD"]);
    const status = await git(["status", "--porcelain"]);
    const ahead = await git(["rev-list", "--count", "@{u}..HEAD"]);
    return {
        path: repo,
        isRepo: true,
        branch: branch.stdout.trim(),
        dirtyFiles: status.stdout.split("\n").filter(Boolean).length,
        unpushed: ahead.ok ? Number(ahead.stdout.trim()) : null,
    };
}

/**
 * Commit and push the manifest, plus any pinned logs.
 *
 * Unpinned .wpilog files stay out of git on purpose: a season of match logs is tens of gigabytes
 * and git has no good answer for that. The manifest carries the numbers worth keeping, so log
 * history survives in the repo even when the binaries do not.
 */
export async function commitAndPush({ message, push = true, pinnedOnly = true }) {
    const repo = logsRepoPath();
    if (!fs.existsSync(path.join(repo, ".git"))) {
        return { ok: false, error: `${repo} is not a git repo -- clone ${config.logsRepo.remote} there first` };
    }
    const manifest = readManifest();
    const toAdd = [config.logsRepo.manifest];
    if (pinnedOnly) {
        for (const l of manifest.logs.filter((l) => l.pinned)) {
            const rel = path.join(config.logsRepo.logsSubdir, l.name);
            if (fs.existsSync(path.join(repo, rel))) toAdd.push(rel);
        }
    }
    const add = await git(["add", "-f", ...toAdd]);
    if (!add.ok) return { ok: false, error: add.stderr };

    const diff = await git(["diff", "--cached", "--quiet"]);
    if (diff.ok) return { ok: true, noChanges: true };

    const commit = await git(["commit", "-m", message || "Sync robot logs"]);
    if (!commit.ok) return { ok: false, error: commit.stderr };
    if (!push) return { ok: true, committed: true, pushed: false };

    const branch = (await git(["rev-parse", "--abbrev-ref", "HEAD"])).stdout.trim() || "main";
    const p = await git(["push", "-u", "origin", branch]);
    return p.ok ? { ok: true, committed: true, pushed: true, branch } : { ok: false, committed: true, error: p.stderr };
}
