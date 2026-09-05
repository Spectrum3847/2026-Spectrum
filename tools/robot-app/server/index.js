import express from "express";
import fs from "node:fs";
import path from "node:path";
import { config, APP_ROOT, logsRepoPath, logsDir } from "./lib/config.js";
import { robotRouter } from "./routes/robot.js";
import { logsRouter } from "./routes/logs.js";

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

app.listen(config.port, () => {
    console.log(`\n  Spectrum robot app   http://localhost:${config.port}`);
    console.log(`  logs repo            ${logsRepoPath()}${fs.existsSync(logsRepoPath()) ? "" : "   (not cloned yet)"}`);
    console.log(`  robot profile        ${config.robotProfile}\n`);
});
