/*
 * Minimal read-only NetworkTables 4 client.
 *
 * The alignment app only ever subscribes -- it never publishes a value and never needs a synced
 * clock -- so this is a fraction of a full NT4 implementation: a WebSocket, the JSON control
 * messages, and a msgpack decoder. No RTT handshake, no encoder, no dependencies.
 *
 * Protocol reference: https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc
 */

'use strict';

const NT4_SUBPROTOCOLS = ['v4.1.networktables.first.wpi.edu', 'networktables.first.wpi.edu'];
const NT4_PORT = 5810;

/**
 * Give up on a connection attempt after this long.
 *
 * A wrong or powered-off address leaves the TCP connect sitting in SYN for the OS timeout, tens of
 * seconds, during which the page just says "connecting". Failing fast and retrying makes a typo
 * look like a typo.
 */
const CONNECT_TIMEOUT_MS = 4000;

/** Wait between reconnect attempts. */
const RECONNECT_DELAY_MS = 1500;

/**
 * Decodes msgpack values out of a byte buffer.
 *
 * Covers only what an NT4 server sends in a value frame: integers, floats, strings, booleans, nil,
 * binary, and arrays. Maps and extension types are not used there.
 */
class MsgpackDecoder {
    /** @param {ArrayBuffer} buffer the raw binary frame */
    constructor(buffer) {
        this.view = new DataView(buffer);
        this.bytes = new Uint8Array(buffer);
        this.offset = 0;
    }

    /** @returns {boolean} whether any bytes remain */
    hasMore() {
        return this.offset < this.bytes.length;
    }

    /** @returns {*} the next decoded value */
    read() {
        const byte = this.view.getUint8(this.offset++);

        if (byte <= 0x7f) return byte; // positive fixint
        if (byte >= 0xe0) return byte - 256; // negative fixint
        if (byte >= 0x80 && byte <= 0x8f) return this.readMap(byte & 0x0f);
        if (byte >= 0x90 && byte <= 0x9f) return this.readArray(byte & 0x0f);
        if (byte >= 0xa0 && byte <= 0xbf) return this.readString(byte & 0x1f);

        switch (byte) {
            case 0xc0:
                return null;
            case 0xc2:
                return false;
            case 0xc3:
                return true;
            case 0xc4:
                return this.readBinary(this.readUint(1));
            case 0xc5:
                return this.readBinary(this.readUint(2));
            case 0xc6:
                return this.readBinary(this.readUint(4));
            case 0xca: {
                const value = this.view.getFloat32(this.offset);
                this.offset += 4;
                return value;
            }
            case 0xcb: {
                const value = this.view.getFloat64(this.offset);
                this.offset += 8;
                return value;
            }
            case 0xcc:
                return this.readUint(1);
            case 0xcd:
                return this.readUint(2);
            case 0xce:
                return this.readUint(4);
            case 0xcf:
                return this.readUint64(false);
            case 0xd0: {
                const value = this.view.getInt8(this.offset);
                this.offset += 1;
                return value;
            }
            case 0xd1: {
                const value = this.view.getInt16(this.offset);
                this.offset += 2;
                return value;
            }
            case 0xd2: {
                const value = this.view.getInt32(this.offset);
                this.offset += 4;
                return value;
            }
            case 0xd3:
                return this.readUint64(true);
            case 0xd9:
                return this.readString(this.readUint(1));
            case 0xda:
                return this.readString(this.readUint(2));
            case 0xdb:
                return this.readString(this.readUint(4));
            case 0xdc:
                return this.readArray(this.readUint(2));
            case 0xdd:
                return this.readArray(this.readUint(4));
            case 0xde:
                return this.readMap(this.readUint(2));
            case 0xdf:
                return this.readMap(this.readUint(4));
            default:
                throw new Error(`Unsupported msgpack byte 0x${byte.toString(16)}`);
        }
    }

    readUint(byteCount) {
        let value = 0;
        for (let i = 0; i < byteCount; i++) {
            value = value * 256 + this.view.getUint8(this.offset++);
        }
        return value;
    }

    /**
     * Reads a 64-bit integer as a JS number. NT4 uses these for timestamps (microseconds) and
     * integer topics, both well inside the safe-integer range for a robot session.
     */
    readUint64(signed) {
        const big = signed
            ? this.view.getBigInt64(this.offset)
            : this.view.getBigUint64(this.offset);
        this.offset += 8;
        return Number(big);
    }

    readString(length) {
        const slice = this.bytes.subarray(this.offset, this.offset + length);
        this.offset += length;
        return new TextDecoder('utf-8').decode(slice);
    }

    readBinary(length) {
        const slice = this.bytes.slice(this.offset, this.offset + length);
        this.offset += length;
        return slice;
    }

    readArray(length) {
        const result = [];
        for (let i = 0; i < length; i++) result.push(this.read());
        return result;
    }

    readMap(length) {
        const result = {};
        for (let i = 0; i < length; i++) {
            const key = this.read();
            result[key] = this.read();
        }
        return result;
    }
}

/**
 * A read-only NT4 subscription to one topic prefix.
 *
 * Reconnects on its own, because a robot reboot or a radio blip mid-alignment should not mean
 * reloading the page.
 */
class NT4Client {
    /**
     * @param {object} options
     * @param {string} options.prefix topic prefix to subscribe to, e.g. "/Robot/Swerve/Align/"
     * @param {function(string, *, number): void} options.onValue called with (topic, value, serverTimeUs)
     * @param {function(string): void} options.onState called with 'connecting' | 'connected' | 'disconnected'
     */
    constructor({ prefix, onValue, onState }) {
        this.prefix = prefix;
        this.onValue = onValue;
        this.onState = onState || (() => {});
        this.socket = null;
        this.topicsById = new Map();
        this.reconnectTimer = null;
        this.connectTimer = null;
        this.shouldRun = false;
        this.host = null;
    }

    /**
     * Connects, or reconnects to a different host.
     *
     * @param {string} host robot address, e.g. "10.85.15.2" or "127.0.0.1"
     */
    connect(host) {
        this.disconnect();
        this.host = host;
        this.shouldRun = true;
        this.open();
    }

    /** Closes the socket and stops reconnecting. */
    disconnect() {
        this.shouldRun = false;
        clearTimeout(this.reconnectTimer);
        clearTimeout(this.connectTimer);
        if (this.socket) {
            this.socket.onclose = null;
            this.socket.close();
            this.socket = null;
        }
        this.topicsById.clear();
    }

    open() {
        if (!this.shouldRun) return;

        const clientId = `swerve-align-${Math.floor(Math.random() * 1e6)}`;
        const url = `ws://${this.host}:${NT4_PORT}/nt/${clientId}`;
        this.onState('connecting');

        let socket;
        try {
            socket = new WebSocket(url, NT4_SUBPROTOCOLS);
        } catch (err) {
            this.scheduleReconnect();
            return;
        }
        socket.binaryType = 'arraybuffer';
        this.socket = socket;

        this.connectTimer = setTimeout(() => {
            if (socket.readyState === WebSocket.CONNECTING) {
                socket.close(); // Triggers onclose, which schedules the retry.
            }
        }, CONNECT_TIMEOUT_MS);

        socket.onopen = () => {
            clearTimeout(this.connectTimer);
            this.topicsById.clear();
            this.onState('connected');
            this.send([
                {
                    method: 'subscribe',
                    params: {
                        topics: [this.prefix],
                        subuid: Math.floor(Math.random() * 1e6),
                        options: { prefix: true, periodic: 0.05 }
                    }
                }
            ]);
        };

        socket.onmessage = (event) => {
            if (typeof event.data === 'string') {
                this.handleText(event.data);
            } else {
                this.handleBinary(event.data);
            }
        };

        socket.onclose = () => {
            clearTimeout(this.connectTimer);
            this.socket = null;
            this.onState('disconnected');
            this.scheduleReconnect();
        };

        // onclose always follows onerror, so reconnecting is handled in one place.
        socket.onerror = () => {};
    }

    scheduleReconnect() {
        if (!this.shouldRun) return;
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = setTimeout(() => this.open(), RECONNECT_DELAY_MS);
    }

    send(messages) {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            this.socket.send(JSON.stringify(messages));
        }
    }

    handleText(data) {
        let messages;
        try {
            messages = JSON.parse(data);
        } catch (err) {
            return;
        }
        for (const message of messages) {
            if (message.method === 'announce') {
                this.topicsById.set(message.params.id, message.params.name);
            } else if (message.method === 'unannounce') {
                this.topicsById.delete(message.params.id);
            }
        }
    }

    handleBinary(buffer) {
        const decoder = new MsgpackDecoder(buffer);
        while (decoder.hasMore()) {
            let frame;
            try {
                frame = decoder.read();
            } catch (err) {
                return; // Malformed tail; drop the rest of this frame.
            }
            if (!Array.isArray(frame) || frame.length !== 4) continue;

            const [topicId, timestamp, , value] = frame;
            if (topicId < 0) continue; // RTT response; we do not use the clock.

            const name = this.topicsById.get(topicId);
            if (name) {
                this.onValue(name, value, timestamp);
            }
        }
    }
}

window.NT4Client = NT4Client;
