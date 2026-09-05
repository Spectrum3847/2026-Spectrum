// Minimal WPILib DataLog (.wpilog) reader. Usage:
//   node wpilog.js <file> list                 -> entries with type and record count
//   node wpilog.js <file> dump <name> [<name>...] -> CSV of timestamp,value for entries
const fs = require("fs");

function readVarInt(buf, off, len) {
    let v = 0;
    for (let i = 0; i < len; i++) v += buf[off + i] * 2 ** (8 * i);
    return v;
}

function parse(file, wanted) {
    const buf = fs.readFileSync(file);
    if (buf.toString("latin1", 0, 6) !== "WPILOG") throw new Error("not a wpilog: " + file);
    const version = buf.readUInt16LE(6);
    const extraLen = buf.readUInt32LE(8);
    let off = 12 + extraLen;
    const entries = new Map(); // id -> {name, type, count, values: []}
    const byName = new Map();
    let firstTs = null, lastTs = null;
    while (off < buf.length) {
        const h = buf[off++];
        const idLen = (h & 0x3) + 1, sizeLen = ((h >> 2) & 0x3) + 1, tsLen = ((h >> 4) & 0x3) + 1;
        const id = readVarInt(buf, off, idLen); off += idLen;
        const size = readVarInt(buf, off, sizeLen); off += sizeLen;
        const ts = readVarInt(buf, off, tsLen); off += tsLen;
        const payload = buf.subarray(off, off + size); off += size;
        if (firstTs === null) firstTs = ts;
        lastTs = ts;
        if (id === 0) {
            const type = payload[0];
            if (type === 0) {
                let p = 1;
                const eid = payload.readUInt32LE(p); p += 4;
                const nl = payload.readUInt32LE(p); p += 4;
                const name = payload.toString("utf8", p, p + nl); p += nl;
                const tl = payload.readUInt32LE(p); p += 4;
                const etype = payload.toString("utf8", p, p + tl); p += tl;
                const e = { id: eid, name, type: etype, count: 0, values: [] };
                entries.set(eid, e);
                byName.set(name, e);
            }
            continue;
        }
        const e = entries.get(id);
        if (!e) continue;
        e.count++;
        if (wanted && !wanted.has(e.name)) continue;
        e.values.push([ts / 1e6, decode(e.type, payload)]);
    }
    return { version, entries, byName, firstTs: firstTs / 1e6, lastTs: lastTs / 1e6 };
}

function decode(type, p) {
    switch (type) {
        case "double": return p.readDoubleLE(0);
        case "float": return p.readFloatLE(0);
        case "int64": return Number(p.readBigInt64LE(0));
        case "boolean": return p[0] !== 0;
        case "string": case "json": return p.toString("utf8");
        case "double[]": { const a = []; for (let i = 0; i + 8 <= p.length; i += 8) a.push(p.readDoubleLE(i)); return a; }
        case "boolean[]": return Array.from(p, b => b !== 0);
        case "int64[]": { const a = []; for (let i = 0; i + 8 <= p.length; i += 8) a.push(Number(p.readBigInt64LE(i))); return a; }
        case "float[]": { const a = []; for (let i = 0; i + 4 <= p.length; i += 4) a.push(+p.readFloatLE(i).toFixed(2)); return a; }
        case "string[]": { let o = 0; const n = p.readUInt32LE(o); o += 4; const a = []; for (let i = 0; i < n; i++) { const l = p.readUInt32LE(o); o += 4; a.push(p.toString("utf8", o, o + l)); o += l; } return a; }
        case "struct:Pose2d": return { x: p.readDoubleLE(0), y: p.readDoubleLE(8), deg: p.readDoubleLE(16) * 180 / Math.PI };
        case "struct:ChassisSpeeds": return { vx: p.readDoubleLE(0), vy: p.readDoubleLE(8), omega: p.readDoubleLE(16) };
        case "struct:Translation2d": return { x: p.readDoubleLE(0), y: p.readDoubleLE(8) };
        case "struct:Pose3d": {
            const w = p.readDoubleLE(24), x = p.readDoubleLE(32), y = p.readDoubleLE(40), z = p.readDoubleLE(48);
            const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / Math.PI;
            return { x: p.readDoubleLE(0), y: p.readDoubleLE(8), z: p.readDoubleLE(16), yawDeg: yaw };
        }
        default: return `<${type} ${p.length}B>`;
    }
}

const [file, cmd, ...names] = process.argv.slice(2);
if (cmd === "list") {
    const log = parse(file, new Set());
    console.log(`version ${log.version}  span ${log.firstTs.toFixed(1)}s .. ${log.lastTs.toFixed(1)}s (${(log.lastTs - log.firstTs).toFixed(1)} s)  entries ${log.entries.size}`);
    for (const e of [...log.entries.values()].sort((a, b) => a.name.localeCompare(b.name))) console.log(`${String(e.count).padStart(7)}  ${e.type.padEnd(22)} ${e.name}`);
} else if (cmd === "dump") {
    const log = parse(file, new Set(names));
    for (const n of names) {
        const e = log.byName.get(n);
        if (!e) { console.log(`# ${n}: not found`); continue; }
        console.log(`# ${n} (${e.type}, ${e.values.length} records)`);
        for (const [t, v] of e.values) console.log(`${t.toFixed(3)}\t${typeof v === "object" ? JSON.stringify(v) : v}`);
    }
}
module.exports = { parse };
