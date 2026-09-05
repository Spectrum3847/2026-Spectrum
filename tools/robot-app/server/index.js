import express from "express";
import { spawn } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { config, APP_ROOT, logsRepoPath, logsDir } from "./lib/config.js";
import { robotRouter } from "./routes/robot.js";
import { logsRouter } from "./routes/logs.js";
import { swerveRouter } from "./routes/swerve.js";
import { runDriftCheck } from "../scripts/check-drift.mjs";

const app = express();
app.use(express.json({ limit: "2mb" }));

app.get("/api/health", (req, res) => res.json({ ok: true, node: process.version }));

app.get("/api/config", (req, res) => {
    const repo = logsRepoPath();
    res.json({
        port: config.port,
        robot: config.robot,
        robotProfile: config.robotProfile,
        logsRepo: {
            ...config.logsRepo,
            resolvedPath: repo,
            exists: fs.existsSync(repo),
            isGitRepo: fs.existsSync(path.join(repo, ".git")),
            logsDir: logsDir(),
        },
    });
});

/*
 * Whether this app still describes the robot code.
 *
 * This used to be a Gradle task wired into `check`, which meant nobody could change robot code
 * without stopping to update a web app first -- exactly backwards. The check now lives here: the
 * app reports on itself, and the build is none the wiser.
 *
 * "problems" are parsed contradictions; "stale" is the softer and more useful signal, a data file
 * that has not been touched since the Java it mirrors moved. Cached briefly because it shells out
 * to git once per source file.
 */
let driftCache = { at: 0, value: null };
const DRIFT_CACHE_MS = 10_000;

app.get("/api/drift", (req, res) => {
    const now = Date.now();
    if (!driftCache.value || now - driftCache.at > DRIFT_CACHE_MS) {
        try {
            driftCache = { at: now, value: runDriftCheck() };
        } catch (e) {
            // A broken checker must never take the app down with it.
            return res.json({ problems: [], notes: [], stale: [], error: String(e.message) });
        }
    }
    res.json(driftCache.value);
});

app.use("/api/robot", robotRouter);
app.use("/api/logs", logsRouter);
app.use("/api/swerve", swerveRouter);

// Static data (controls map, robot profile) is served straight from data/ so the same files are
// both the app's input and the thing the Gradle drift check validates.
app.use("/data", express.static(path.join(APP_ROOT, "data"), { etag: false, maxAge: 0 }));

const dist = path.join(APP_ROOT, "dist");
const CLIENT_PAGES = path.join(APP_ROOT, "client", "pages");

/** The page routes this app is supposed to have, read from the source tree. */
function expectedPages() {
    if (!fs.existsSync(CLIENT_PAGES)) return [];
    return fs
        .readdirSync(CLIENT_PAGES, { withFileTypes: true })
        .filter((e) => e.isDirectory() && fs.existsSync(path.join(CLIENT_PAGES, e.name, "index.html")))
        .map((e) => e.name);
}

/** Newest mtime under a directory, used to notice a dist built before the last source edit. */
function newestMtime(dir) {
    let newest = 0;
    const walk = (d) => {
        for (const e of fs.readdirSync(d, { withFileTypes: true })) {
            const full = path.join(d, e.name);
            if (e.isDirectory()) walk(full);
            else newest = Math.max(newest, fs.statSync(full).mtimeMs);
        }
    };
    if (fs.existsSync(dir)) walk(dir);
    return newest;
}

/**
 * What is wrong with the current build, if anything.
 *
 * A stale or partial dist used to be invisible: the catch-all below served the home page for any
 * path it could not find, so clicking "Logs" quietly showed Home with no error anywhere. Pages
 * are checked against the source tree instead, and a miss is reported rather than papered over.
 */
function buildStatus() {
    if (!fs.existsSync(dist)) return { built: false, missing: expectedPages(), stale: false };
    const missing = expectedPages().filter((name) => !fs.existsSync(path.join(dist, "pages", name, "index.html")));
    const stale = newestMtime(path.join(APP_ROOT, "client")) > newestMtime(dist);
    return { built: true, missing, stale };
}

const BUILD_HINT =
    "Run `npm start` (build + serve) or `npm run build`, then reload.\n" +
    "For hot reload while developing, run `npm run dev` and use http://localhost:5173 instead.";

if (fs.existsSync(dist)) {
    app.use(express.static(dist, { redirect: true }));

    // Anything express.static did not serve is genuinely missing. Say which page and why, rather
    // than returning the home page and letting it look like the nav is broken.
    app.use((req, res, next) => {
        if (req.method !== "GET" || req.path.startsWith("/api/") || req.path.startsWith("/data/")) return next();

        const page = /^\/pages\/([^/]+)\/?$/.exec(req.path)?.[1];
        if (page && expectedPages().includes(page)) {
            return res
                .status(503)
                .type("text/plain")
                .send(`The "${page}" page exists in client/pages/ but is not in dist/ -- the build is out of date.\n\n${BUILD_HINT}\n`);
        }
        res.status(404).type("text/plain").send(`404 ${req.path}\n\nPages: / ${expectedPages().map((n) => `/pages/${n}/`).join(" ")}\n`);
    });
} else {
    app.use((req, res, next) => {
        if (req.path.startsWith("/api/") || req.path.startsWith("/data/")) return next();
        res.status(503).type("text/plain").send(`The client has not been built yet.\n\n${BUILD_HINT}\n`);
    });
}

app.use((err, req, res, next) => {
    console.error(err);
    res.status(500).json({ error: "unhandled server error", detail: String(err.message) });
});

/*
 * Loopback only. The swerve-align endpoint rewrites a source file in this repo, and the logs
 * endpoints run git; neither has any business being reachable from the pit network. Override with
 * `host` in config.local.json only if you know why you want that.
 */
app.listen(config.port, config.host || "127.0.0.1", () => {
    const url = `http://localhost:${config.port}`;
    console.log(`\n  Spectrum robot app   ${url}`);
    console.log(`  logs repo            ${logsRepoPath()}${fs.existsSync(logsRepoPath()) ? "" : "   (not cloned yet)"}`);
    console.log(`  robot profile        ${config.robotProfile}`);
    console.log(`  writes offsets to    ${config.swerveAlign?.targetConfig ?? "(unset)"}\n`);
    const build = buildStatus();
    if (!build.built) {
        console.log("  client               NOT BUILT -- run `npm run build`");
    } else if (build.missing.length) {
        console.log(`  client               INCOMPLETE -- missing ${build.missing.join(", ")}; run \`npm run build\``);
    } else if (build.stale) {
        console.log("  client               STALE -- sources are newer than dist/; run `npm run build`");
    }
    console.log("\n  Press Ctrl+C to stop.\n");

    // --open, optionally with a path, so a desktop shortcut can land on one page.
    const i = process.argv.indexOf("--open");
    if (i !== -1) {
        const page = process.argv[i + 1] && !process.argv[i + 1].startsWith("--") ? process.argv[i + 1] : "/";
        const opener = process.platform === "darwin" ? "open" : process.platform === "win32" ? "explorer" : "xdg-open";
        spawn(opener, [url + page], { detached: true, stdio: "ignore" }).unref();
    }
});
