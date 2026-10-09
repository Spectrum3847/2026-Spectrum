// Driver Station log summary: roboRIO CPU, rio CAN utilization and trip time from the .dslog
// files, plus counts of the messages that matter from the matching .dsevents.
//
//   node scripts/dslog.js "<folder holding .dslog/.dsevents>" [filename prefix]
//
// The DS keeps these in C:\Users\Public\Documents\FRC\Log Files on the driver station laptop.
// The prefix filters by filename, e.g. "2026_09_05" for one day.
//
// .dslog format (version 4): a 20-byte header, then one 35-byte record per 20 ms:
//   byte 0 trip time (x0.5 ms), byte 1 lost packets (x4 %), bytes 2-3 voltage (/256, big endian),
//   byte 4 roboRIO CPU (x0.5 %), byte 5 status bits, byte 6 rio CAN utilization (x0.5 %),
//   byte 7 wifi dB (x0.5), bytes 8-9 wifi Mb (/256), then PDP currents.
// Records with voltage 256.0 or CPU 0 are "robot not connected" sentinels and are dropped.
// The status-bit layout is not verified here, so this does not split enabled from disabled.
//
// Reference numbers from 2026-09-05 before any of the load cuts: roboRIO CPU median 92-95%
// in every session, enabled or not; 4 to 150 WaitForAll -1003 errors per minute.
const fs = require("fs");
const path = require("path");

const [dir, prefix = ""] = process.argv.slice(2);
if (!dir) {
    console.error('usage: node scripts/dslog.js "<folder>" [filename prefix]');
    process.exit(2);
}

const pct = (a, p) => { const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };
const stat = (a) => (a.length ? `med ${pct(a, 0.5).toFixed(1).padStart(5)}  p90 ${pct(a, 0.9).toFixed(1).padStart(5)}  max ${Math.max(...a).toFixed(1).padStart(5)}` : "no data");
const count = (text, needle) => text.split(needle).length - 1;

const files = fs.readdirSync(dir).filter((f) => f.endsWith(".dslog") && f.startsWith(prefix)).sort();
if (!files.length) {
    console.error("no .dslog files matching in " + dir);
    process.exit(1);
}

for (const f of files) {
    const b = fs.readFileSync(path.join(dir, f));
    const version = b.readInt32BE(0);
    const recs = [];
    for (let off = 20; off + 35 <= b.length; off += 35) {
        const volt = b.readUInt16BE(off + 2) / 256;
        const cpu = b[off + 4] * 0.5;
        if (volt >= 255 || cpu === 0) continue; // not connected
        recs.push({ trip: b[off] * 0.5, lost: b.readInt8(off + 1) * 4, volt, cpu, can: b[off + 6] * 0.5 });
    }
    const minutes = recs.length / 50 / 60;
    if (minutes < 0.5) continue;

    let events = "";
    try { events = fs.readFileSync(path.join(dir, f.replace(/\.dslog$/, ".dsevents")), "latin1"); } catch (e) { /* no events file */ }
    const waitForAll = count(events, "too-stale");
    const stale = count(events, "CAN message is stale");
    const overruns = count(events, "Loop time of");
    const lostComms = count(events, "lost communication with the robot");
    const queueFull = count(events, "MAX_QUEUED_LOGS");

    console.log(`\n${f}  (v${version}, ${minutes.toFixed(1)} min connected)`);
    console.log(`  roboRIO CPU %     ${stat(recs.map((r) => r.cpu))}`);
    console.log(`  rio CAN %         ${stat(recs.map((r) => r.can))}`);
    console.log(`  trip time ms      ${stat(recs.map((r) => r.trip))}`);
    console.log(`  battery V         ${stat(recs.map((r) => r.volt))}`);
    console.log(`  events: WaitForAll -1003 ${waitForAll} (${(waitForAll / minutes).toFixed(0)}/min)   stale signals ${stale}   loop overrun prints ${overruns}   lost comms ${lostComms}   DogLog queue full ${queueFull}`);
}
