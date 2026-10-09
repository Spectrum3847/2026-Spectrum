/**
 * WPILib DataLog (.wpilog) reader.
 *
 * Runs unchanged in the browser and in Node -- both have DataView/TextDecoder -- so the sync
 * server and the analysis pages share one parser and can never disagree about what a log says.
 *
 * Record framing, per the WPILib datalog spec:
 *   header byte: bits 0-1 = entry-id length - 1
 *                bits 2-3 = payload-size length - 1
 *                bits 4-6 = timestamp length - 1   <-- three bits, not two
 *                bit  7   = spare
 *
 * That timestamp field really is three bits wide. A 2-bit mask happens to work for the first
 * ~71 minutes of robot uptime (the point where microsecond timestamps outgrow four bytes) and
 * then silently desyncs the entire stream. scripts/wpilog.js in this repo has that bug.
 */

const CONTROL_ENTRY = 0;
const CONTROL_START = 0;
const CONTROL_FINISH = 1;
const CONTROL_SET_METADATA = 2;

const utf8 = new TextDecoder("utf-8", { fatal: false });

function decodeValue(type, view, offset, size, buf) {
    switch (type) {
        case "double":
            return size >= 8 ? view.getFloat64(offset, true) : undefined;
        case "float":
            return size >= 4 ? view.getFloat32(offset, true) : undefined;
        case "int64": {
            if (size < 8) return undefined;
            // Values here are counters and timestamps; Number is exact to 2^53 which is far
            // beyond anything a match produces. Avoids BigInt in every downstream chart.
            const lo = view.getUint32(offset, true);
            const hi = view.getInt32(offset + 4, true);
            return hi * 4294967296 + lo;
        }
        case "boolean":
            return size >= 1 ? view.getUint8(offset) !== 0 : undefined;
        case "string":
        case "json":
            return utf8.decode(new Uint8Array(buf, offset, size));
        case "boolean[]": {
            const out = new Array(size);
            for (let i = 0; i < size; i++) out[i] = view.getUint8(offset + i) !== 0;
            return out;
        }
        case "double[]": {
            const out = [];
            for (let i = 0; i + 8 <= size; i += 8) out.push(view.getFloat64(offset + i, true));
            return out;
        }
        case "float[]": {
            const out = [];
            for (let i = 0; i + 4 <= size; i += 4) out.push(view.getFloat32(offset + i, true));
            return out;
        }
        case "int64[]": {
            const out = [];
            for (let i = 0; i + 8 <= size; i += 8) {
                const lo = view.getUint32(offset + i, true);
                const hi = view.getInt32(offset + i + 4, true);
                out.push(hi * 4294967296 + lo);
            }
            return out;
        }
        case "string[]": {
            let p = offset;
            const n = view.getUint32(p, true);
            p += 4;
            const out = [];
            for (let i = 0; i < n; i++) {
                const len = view.getUint32(p, true);
                p += 4;
                out.push(utf8.decode(new Uint8Array(buf, p, len)));
                p += len;
            }
            return out;
        }
        case "struct:Pose2d":
            return size >= 24
                ? { x: view.getFloat64(offset, true), y: view.getFloat64(offset + 8, true), deg: (view.getFloat64(offset + 16, true) * 180) / Math.PI }
                : undefined;
        case "struct:Translation2d":
            return size >= 16 ? { x: view.getFloat64(offset, true), y: view.getFloat64(offset + 8, true) } : undefined;
        case "struct:ChassisSpeeds":
            return size >= 24
                ? { vx: view.getFloat64(offset, true), vy: view.getFloat64(offset + 8, true), omega: view.getFloat64(offset + 16, true) }
                : undefined;
        case "struct:Pose3d": {
            if (size < 56) return undefined;
            const w = view.getFloat64(offset + 24, true);
            const qx = view.getFloat64(offset + 32, true);
            const qy = view.getFloat64(offset + 40, true);
            const qz = view.getFloat64(offset + 48, true);
            const yaw = (Math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) * 180) / Math.PI;
            return { x: view.getFloat64(offset, true), y: view.getFloat64(offset + 8, true), z: view.getFloat64(offset + 16, true), yawDeg: yaw };
        }
        default:
            return undefined;
    }
}

export class WpiLog {
    constructor() {
        this.channels = new Map(); // name -> [[tSec, value], ...]
        this.types = new Map(); // name -> type string
        this.metadata = new Map(); // name -> metadata string (DogLog puts units here)
        this.counts = new Map(); // name -> record count, including records we did not keep
        this.firstTs = null;
        this.lastTs = null;
        this.error = null;
    }

    channel(name) {
        return this.channels.get(name) || [];
    }

    has(name) {
        return (this.channels.get(name)?.length ?? 0) > 0;
    }

    names() {
        return [...this.channels.keys()];
    }

    /** Channel names matching a RegExp, with the first capture group returned alongside. */
    matching(re) {
        const out = [];
        for (const name of this.channels.keys()) {
            const m = name.match(re);
            if (m) out.push({ name, capture: m[1] ?? null, values: this.channels.get(name) });
        }
        return out;
    }

    get durationSec() {
        return this.firstTs === null ? 0 : this.lastTs - this.firstTs;
    }
}

/**
 * Parse a .wpilog.
 *
 * @param {ArrayBuffer|Uint8Array|Buffer} input
 * @param {{keep?: (name: string, type: string) => boolean}} [opts]
 *   keep() lets a caller skip storing values for channels it will not use. Entry metadata and
 *   record counts are still recorded, so "what is in this log" stays answerable cheaply. On an
 *   18 MB match log this is the difference between ~40 MB of arrays and ~2 MB.
 */
export function parseWpilog(input, opts = {}) {
    const keep = opts.keep || (() => true);
    const log = new WpiLog();

    let buf;
    if (input instanceof ArrayBuffer) buf = input;
    else if (ArrayBuffer.isView(input)) buf = input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength);
    else throw new TypeError("parseWpilog expects an ArrayBuffer or a typed array");

    const view = new DataView(buf);
    const end = view.byteLength;
    let pos = 0;

    const magic = utf8.decode(new Uint8Array(buf, 0, 6));
    if (magic !== "WPILOG") {
        log.error = "not a WPILOG file";
        return log;
    }
    log.version = view.getUint16(6, true);
    const extraLen = view.getUint32(8, true);
    pos = 12 + extraLen;

    const entries = new Map(); // id -> {name, type, kept}

    const readUint = (n) => {
        let v = 0;
        for (let i = 0; i < n; i++) v += view.getUint8(pos++) * 256 ** i;
        return v;
    };

    try {
        while (pos < end) {
            const h = view.getUint8(pos++);
            const idLen = (h & 0x3) + 1;
            const sizeLen = ((h >> 2) & 0x3) + 1;
            const tsLen = ((h >> 4) & 0x7) + 1;
            if (pos + idLen + sizeLen + tsLen > end) break;

            const entryId = readUint(idLen);
            const size = readUint(sizeLen);
            const tsUs = readUint(tsLen);
            if (pos + size > end) break;

            const payloadStart = pos;
            const tSec = tsUs / 1e6;
            if (log.firstTs === null) log.firstTs = tSec;
            log.lastTs = tSec;

            if (entryId === CONTROL_ENTRY) {
                const kind = size > 0 ? view.getUint8(pos) : -1;
                if (kind === CONTROL_START) {
                    pos += 1;
                    const id = readUint(4);
                    const nameLen = readUint(4);
                    const name = utf8.decode(new Uint8Array(buf, pos, nameLen));
                    pos += nameLen;
                    const typeLen = readUint(4);
                    const type = utf8.decode(new Uint8Array(buf, pos, typeLen));
                    pos += typeLen;
                    const metaLen = readUint(4);
                    const meta = utf8.decode(new Uint8Array(buf, pos, metaLen));
                    pos += metaLen;

                    const kept = keep(name, type);
                    entries.set(id, { name, type, kept });
                    log.types.set(name, type);
                    if (meta) log.metadata.set(name, meta);
                    if (!log.counts.has(name)) log.counts.set(name, 0);
                    if (kept && !log.channels.has(name)) log.channels.set(name, []);
                } else if (kind === CONTROL_SET_METADATA) {
                    pos += 1;
                    const id = readUint(4);
                    const metaLen = readUint(4);
                    const meta = utf8.decode(new Uint8Array(buf, pos, metaLen));
                    const e = entries.get(id);
                    if (e && meta) log.metadata.set(e.name, meta);
                }
                // CONTROL_FINISH carries no information we need.
                pos = payloadStart + size;
                continue;
            }

            const entry = entries.get(entryId);
            if (!entry) {
                pos = payloadStart + size;
                continue;
            }
            log.counts.set(entry.name, (log.counts.get(entry.name) || 0) + 1);
            if (entry.kept) {
                const value = decodeValue(entry.type, view, payloadStart, size, buf);
                if (value !== undefined) log.channels.get(entry.name).push([tSec, value]);
            }
            pos = payloadStart + size;
        }
    } catch (e) {
        // A truncated log -- the RIO lost power mid-write -- is normal and still worth reading.
        log.error = `stopped at byte ${pos}: ${e.message}`;
    }

    if (log.firstTs === null) log.firstTs = 0;
    if (log.lastTs === null) log.lastTs = 0;
    return log;
}
