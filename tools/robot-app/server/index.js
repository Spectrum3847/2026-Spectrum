import express from "express";
import { spawn } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { config, APP_ROOT, logsRepoPath, logsDir } from "./lib/config.js";
import { robotRouter } from "./routes/robot.js";
import { logsRouter } from "./routes/logs.js";
import { swerveRouter } from "./routes/swerve.js";

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

app.use("/api/robot", robotRouter);
app.use("/api/logs", logsRouter);
app.use("/api/swerve", swerveRouter);

// Static data (controls map, robot profile) is served straight from data/ so the same files are
// both the app's input and the thing the Gradle drift check validates.
app.use("/data", express.static(path.join(APP_ROOT, "data"), { etag: false, maxAge: 0 }));

const dist = path.join(APP_ROOT, "dist");
if (fs.existsSync(dist)) {
    app.use(express.static(dist));
    // Vite builds one page per entry; anything unmatched falls back to the shell.
    app.get("*", (req, res, next) => {
        if (req.path.startsWith("/api/")) return next();
        res.sendFile(path.join(dist, "index.html"));
    });
} else {
    app.get("/", (req, res) => {
        res.status(503).type("text/plain").send(
            "The client has not been built yet.\n\n" +
                "  Development:  npm run dev     (Vite dev server on 5173, proxying /api here)\n" +
                "  Production:   npm start       (builds to dist/ then serves from here)\n"
        );
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
    console.log("  Press Ctrl+C to stop.\n");

    // --open, optionally with a path, so a desktop shortcut can land on one page.
    const i = process.argv.indexOf("--open");
    if (i !== -1) {
        const page = process.argv[i + 1] && !process.argv[i + 1].startsWith("--") ? process.argv[i + 1] : "/";
        const opener = process.platform === "darwin" ? "open" : process.platform === "win32" ? "explorer" : "xdg-open";
        spawn(opener, [url + page], { detached: true, stdio: "ignore" }).unref();
    }
});
