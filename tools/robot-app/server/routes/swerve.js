import { Router } from "express";
import {
    TARGET_CONFIG,
    MODULE_KEYS,
    MODULE_LABELS,
    parseConfig,
    rewriteConfig,
    gitStatus,
    readTarget,
    writeTarget,
    targetPath,
} from "../lib/swerve-config.js";

export const swerveRouter = Router();

/**
 * Current offsets in the config file, plus enough git context to notice an unexpected edit.
 *
 * The response shape is the standalone tool's, unchanged, so its front end keeps working.
 */
swerveRouter.get("/target", async (req, res) => {
    try {
        const parsed = parseConfig(readTarget());
        const git = await gitStatus();
        res.json({
            file: TARGET_CONFIG,
            absolutePath: targetPath(),
            moduleKeys: MODULE_KEYS,
            moduleLabels: MODULE_LABELS,
            offsets: parsed.offsets,
            expressions: parsed.expressions,
            alignedOn: parsed.alignedOn,
            branch: git.branch,
            targetModified: git.targetModified,
        });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});

/**
 * Write new offsets into the config file.
 *
 * The only endpoint in the app that modifies the source tree, which is why the server binds to
 * loopback only.
 */
swerveRouter.post("/apply", async (req, res) => {
    try {
        const body = req.body || {};
        const offsets = {};
        for (const key of MODULE_KEYS) {
            const value = Number(body[key]);
            if (!Number.isFinite(value)) {
                throw new Error(`Missing or invalid offset for "${key}".`);
            }
            if (value < -0.5 || value >= 0.5) {
                throw new Error(`Offset for "${key}" is ${value} rotations; it must be in [-0.5, 0.5).`);
            }
            offsets[key] = value;
        }

        const result = rewriteConfig(readTarget(), offsets, new Date());
        writeTarget(result.source);

        // Read it back so the UI shows what is really on disk, not what we intended to write.
        const verified = parseConfig(readTarget());

        console.log(`[swerve-align] wrote new offsets to ${TARGET_CONFIG}`);
        res.json({
            file: TARGET_CONFIG,
            before: result.before,
            after: result.after,
            offsets: verified.offsets,
        });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});
