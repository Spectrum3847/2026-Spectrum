import { Router } from "express";
import {
    TARGET_CONFIG,
    WRITABLE,
    parseCameras,
    publicCameras,
    rewriteCamera,
    readTarget,
    writeTarget,
    targetPath,
    gitStatus,
} from "../lib/vision-config.js";

export const visionRouter = Router();

/** The camera mounts as the Java has them, plus enough git context to notice a dirty file. */
visionRouter.get("/target", async (req, res) => {
    try {
        const cameras = publicCameras(parseCameras(readTarget()));
        const git = await gitStatus();
        res.json({
            file: TARGET_CONFIG,
            absolutePath: targetPath(),
            writable: WRITABLE,
            cameras,
            branch: git.branch,
            targetModified: git.targetModified,
        });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});

/**
 * Writes measured mount values for one camera into Vision.java.
 *
 * Body: { camera: "backLeftConfig", roll?: degrees, pitch?: degrees, up?: metres }. Only those
 * three are accepted -- forward, right and yaw need a surveyed robot position and stay CAD. The
 * server binds to loopback, so nothing on the pit network can reach this.
 */
visionRouter.post("/apply", async (req, res) => {
    try {
        const body = req.body || {};
        if (!body.camera) throw new Error("camera is required.");
        const values = {};
        for (const k of WRITABLE) {
            if (body[k] === undefined || body[k] === null) continue;
            const v = Number(body[k]);
            if (!Number.isFinite(v)) throw new Error(`"${k}" must be a number.`);
            values[k] = v;
        }
        if (values.pitch !== undefined && Math.abs(values.pitch) > 90) throw new Error("pitch must be within +/-90 degrees.");
        if (values.roll !== undefined && Math.abs(values.roll) > 180) throw new Error("roll must be within +/-180 degrees.");
        if (values.up !== undefined && (values.up < 0 || values.up > 2)) throw new Error("up must be between 0 and 2 metres.");

        const result = rewriteCamera(readTarget(), body.camera, values, new Date());
        writeTarget(result.source);

        // Read it back so the UI shows what is really on disk, not what we meant to write.
        const verified = publicCameras(parseCameras(readTarget())).find((c) => c.key === body.camera);
        console.log(`[cameras] wrote ${result.changed.join(", ")} for ${body.camera} to ${TARGET_CONFIG}`);
        res.json({ file: TARGET_CONFIG, camera: verified, before: result.before, after: result.after, changed: result.changed });
    } catch (e) {
        res.status(500).json({ error: e.message });
    }
});
