import net from "node:net";
import fs from "node:fs";
import path from "node:path";
import { NodeSSH } from "node-ssh";
import { config } from "./config.js";

/**
 * Candidate roboRIO addresses, most-likely-first.
 *
 * 8515 is the number this robot's radio is configured for (the Limelights live at 10.85.15.x);
 * 3847 is the team's own number and is what a reflashed radio would use. 172.22.11.2 is the
 * fixed USB address and works with no radio at all.
 */
export function robotCandidates() {
    const out = [];
    for (const team of config.robot.teams) {
        const hi = Math.floor(team / 100);
        const lo = team % 100;
        out.push({ host: `10.${hi}.${lo}.2`, label: `Team ${team} radio`, team });
        out.push({ host: `roborio-${team}-frc.local`, label: `Team ${team} mDNS`, team });
    }
    for (const h of config.robot.extraHosts) {
        out.push({ host: h, label: h === "172.22.11.2" ? "USB" : h, team: null });
    }
    return out;
}

/**
 * TCP connect to sshd. Cheaper than ping, and it tests the thing we actually need rather than
 * mere reachability -- a RIO that answers ICMP but not SSH is useless to us.
 */
export function probeHost(host, { port = 22, timeoutMs = 1500 } = {}) {
    return new Promise((resolve) => {
        const started = Date.now();
        const sock = new net.Socket();
        let settled = false;
        const done = (up, reason) => {
            if (settled) return;
            settled = true;
            sock.destroy();
            resolve({ host, up, ms: Date.now() - started, reason: reason || null });
        };
        sock.setTimeout(timeoutMs);
        sock.once("connect", () => done(true));
        sock.once("timeout", () => done(false, "timeout"));
        sock.once("error", (e) => done(false, e.code || "error"));
        sock.connect(port, host);
    });
}

export async function probeAll() {
    const results = await Promise.all(robotCandidates().map((c) => probeHost(c.host).then((r) => ({ ...c, ...r }))));
    const reachable = results.filter((r) => r.up).sort((a, b) => a.ms - b.ms);
    return { candidates: results, reachable, best: reachable[0] || null };
}

/** Connect to the RIO. lvuser has no password on a stock roboRIO image. */
async function connect(host) {
    const ssh = new NodeSSH();
    await ssh.connect({
        host,
        username: config.robot.user,
        password: "",
        tryKeyboard: false,
        readyTimeout: config.robot.sshTimeoutSecs * 1000,
    });
    return ssh;
}

/**
 * List .wpilog files in every configured log directory. /U/logs is the USB stick (preferred, the
 * RIO writes there when a stick is present); /home/lvuser/logs is the fallback on internal flash.
 */
export async function listRobotLogs(host) {
    const ssh = await connect(host);
    try {
        const dirs = [];
        for (const dir of config.robot.logDirs) {
            // %s size, %Y mtime epoch, %n name -- one line per file, easy to parse and cheap.
            const res = await ssh.execCommand(`find ${dir} -maxdepth 1 -name '*.wpilog' -printf '%s\\t%T@\\t%f\\n' 2>/dev/null`);
            const files = res.stdout
                .split("\n")
                .filter(Boolean)
                .map((line) => {
                    const [bytes, mtime, ...rest] = line.split("\t");
                    return { name: rest.join("\t"), bytes: Number(bytes), mtime: Math.round(Number(mtime) * 1000), dir };
                })
                .sort((a, b) => b.mtime - a.mtime);
            dirs.push({ dir, exists: files.length > 0 || res.code === 0, files });
        }
        return { host, dirs };
    } finally {
        ssh.dispose();
    }
}

/**
 * Download the named logs into destDir. onProgress is called per file so the UI can show
 * something during what is often a multi-minute transfer over the robot radio.
 */
export async function downloadLogs(host, files, destDir, onProgress = () => {}) {
    fs.mkdirSync(destDir, { recursive: true });
    const ssh = await connect(host);
    const done = [];
    try {
        for (const f of files) {
            const remote = path.posix.join(f.dir, f.name);
            const local = path.join(destDir, f.name);
            onProgress({ name: f.name, state: "start", bytes: f.bytes });
            const started = Date.now();
            await ssh.getFile(local, remote);
            const st = fs.statSync(local);
            done.push({ name: f.name, bytes: st.size, ms: Date.now() - started, remote });
            onProgress({ name: f.name, state: "done", bytes: st.size, ms: Date.now() - started });
        }
    } finally {
        ssh.dispose();
    }
    return done;
}
