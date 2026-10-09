/**
 * Minimal .wpilog writer, for building fixtures with known content.
 *
 * The turret analyses are the kind of thing that looks right and is wrong: a sign flip or an
 * off-by-one window makes a slip detector that never fires, and nobody notices until a match. The
 * only way to test it is to inject a slip of a known size and assert it comes back out.
 *
 * Record framing matches the WPILib datalog spec. Every field is written at its maximum width
 * (4-byte entry id, size and timestamp) because this is a test fixture, not something that needs
 * to be compact.
 */

const HEADER_BYTE = 3 | (3 << 2) | (3 << 4); // 4-byte id, size and timestamp

export class LogWriter {
    constructor() {
        this.chunks = [];
        this.nextId = 1;
        this.entries = new Map();

        const magic = Buffer.from("WPILOG", "latin1");
        const head = Buffer.alloc(6);
        head.writeUInt16LE(0x0100, 0); // version 1.0
        head.writeUInt32LE(0, 2); // no extra header
        this.chunks.push(magic, head);
    }

    #record(entryId, payload, tsSec) {
        const head = Buffer.alloc(13);
        head.writeUInt8(HEADER_BYTE, 0);
        head.writeUInt32LE(entryId, 1);
        head.writeUInt32LE(payload.length, 5);
        head.writeUInt32LE(Math.round(tsSec * 1e6), 9);
        this.chunks.push(head, payload);
    }

    /** Declare a channel. Returns its entry id. */
    start(name, type, tsSec = 0, metadata = "") {
        const id = this.nextId++;
        const nameBuf = Buffer.from(name, "utf8");
        const typeBuf = Buffer.from(type, "utf8");
        const metaBuf = Buffer.from(metadata, "utf8");
        const payload = Buffer.alloc(1 + 4 + 4 + nameBuf.length + 4 + typeBuf.length + 4 + metaBuf.length);
        let o = 0;
        payload.writeUInt8(0, o); o += 1; // control type: Start
        payload.writeUInt32LE(id, o); o += 4;
        payload.writeUInt32LE(nameBuf.length, o); o += 4;
        nameBuf.copy(payload, o); o += nameBuf.length;
        payload.writeUInt32LE(typeBuf.length, o); o += 4;
        typeBuf.copy(payload, o); o += typeBuf.length;
        payload.writeUInt32LE(metaBuf.length, o); o += 4;
        metaBuf.copy(payload, o);
        this.#record(0, payload, tsSec);
        this.entries.set(name, { id, type });
        return id;
    }

    /** Append one sample. The channel is declared on first use. */
    put(name, type, tsSec, value) {
        if (!this.entries.has(name)) this.start(name, type, 0);
        const { id } = this.entries.get(name);
        let payload;
        switch (type) {
            case "double": payload = Buffer.alloc(8); payload.writeDoubleLE(value, 0); break;
            case "boolean": payload = Buffer.from([value ? 1 : 0]); break;
            case "int64": payload = Buffer.alloc(8); payload.writeBigInt64LE(BigInt(Math.round(value))); break;
            case "string": payload = Buffer.from(String(value), "utf8"); break;
            default: throw new Error(`LogWriter has no encoder for ${type}`);
        }
        this.#record(id, payload, tsSec);
    }

    /** Write a whole series at a fixed rate, from a function of time. */
    series(name, type, { from, to, hz = 50, fn }) {
        const dt = 1 / hz;
        for (let t = from; t <= to + 1e-9; t += dt) {
            const v = fn(+t.toFixed(6));
            if (v !== undefined && v !== null) this.put(name, type, +t.toFixed(6), v);
        }
    }

    buffer() {
        return Buffer.concat(this.chunks);
    }
}
