#!/usr/bin/env python3
"""Minimal pure-Python WPILOG reader for fast triage.

No Java, no WPILib install, no compile step: the whole file is read into memory and
walked once. Payloads are decoded lazily so a scan that only cares about a few topics
stays cheap.

Format reference: https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterator

CONTROL_START = 0
CONTROL_FINISH = 1
CONTROL_SET_METADATA = 2


@dataclass
class Entry:
    id: int
    name: str
    type: str
    metadata: str
    count: int = 0
    first_us: int | None = None
    last_us: int | None = None
    gaps: list = field(default_factory=list)  # (prev_us, ts_us) pairs >= gap_us apart, capped


@dataclass
class LogFile:
    path: Path
    size_bytes: int
    version: int
    extra_header: str
    entries: dict[int, Entry] = field(default_factory=dict)
    by_name: dict[str, Entry] = field(default_factory=dict)
    first_us: int | None = None
    last_us: int | None = None
    truncated: bool = False


def _read_str(buf: bytes, pos: int) -> tuple[str, int]:
    (n,) = struct.unpack_from("<I", buf, pos)
    pos += 4
    return buf[pos : pos + n].decode("utf-8", "replace"), pos + n


def open_log(path: Path) -> tuple[LogFile, bytes, int]:
    data = Path(path).read_bytes()
    if len(data) < 12 or data[:6] != b"WPILOG":
        raise SystemExit(f"{path}: not a WPILOG file")
    version = struct.unpack_from("<H", data, 6)[0]
    extra, pos = _read_str(data, 8)
    return LogFile(Path(path), len(data), version, extra), data, pos


def iter_records(
    log: LogFile,
    data: bytes,
    pos: int,
    want: Callable[[Entry], bool] | None = None,
    gap_us: int = 0,
) -> Iterator[tuple[Entry, int, memoryview]]:
    """Yield (entry, timestamp_us, payload) for data records.

    Every record updates the per-entry count/first/last bookkeeping on ``log`` even when
    ``want`` rejects it, so callers get a topic census for free. With ``gap_us`` > 0, each
    entry also remembers up to 500 (prev, now) pairs where consecutive records were at least
    that far apart, so a stall detector needs no second pass.
    """
    mv = memoryview(data)
    n = len(data)
    entries = log.entries
    by_name = log.by_name
    wanted: dict[int, bool] = {}
    while pos < n:
        hdr = data[pos]
        id_len = (hdr & 0x3) + 1
        size_len = ((hdr >> 2) & 0x3) + 1
        ts_len = ((hdr >> 4) & 0x7) + 1
        p = pos + 1
        need = p + id_len + size_len + ts_len
        if need > n:
            log.truncated = True
            break
        entry_id = int.from_bytes(data[p : p + id_len], "little")
        p += id_len
        size = int.from_bytes(data[p : p + size_len], "little")
        p += size_len
        ts = int.from_bytes(data[p : p + ts_len], "little")
        p += ts_len
        end = p + size
        if end > n:
            log.truncated = True
            break
        pos = end
        if entry_id == 0:
            if size < 1:
                continue
            kind = data[p]
            if kind == CONTROL_START and size >= 17:
                (eid,) = struct.unpack_from("<I", data, p + 1)
                name, q = _read_str(data, p + 5)
                typ, q = _read_str(data, q)
                meta, q = _read_str(data, q)
                e = Entry(eid, name, typ, meta)
                entries[eid] = e
                by_name.setdefault(name, e)
                wanted.pop(eid, None)
            elif kind == CONTROL_FINISH:
                # keep the entry so late lookups still resolve; just stop counting
                pass
            elif kind == CONTROL_SET_METADATA and size >= 9:
                (eid,) = struct.unpack_from("<I", data, p + 1)
                meta, _ = _read_str(data, p + 5)
                if eid in entries:
                    entries[eid].metadata = meta
            continue
        e = entries.get(entry_id)
        if e is None:
            continue
        e.count += 1
        if e.first_us is None:
            e.first_us = ts
        elif gap_us and ts - e.last_us >= gap_us and len(e.gaps) < 500:
            e.gaps.append((e.last_us, ts))
        e.last_us = ts
        if log.first_us is None or ts < log.first_us:
            log.first_us = ts
        if log.last_us is None or ts > log.last_us:
            log.last_us = ts
        if want is not None:
            w = wanted.get(entry_id)
            if w is None:
                w = want(e)
                wanted[entry_id] = w
            if not w:
                continue
        yield e, ts, mv[p:end]


# ── payload decoders ──────────────────────────────────────────────────────


def decode(entry_type: str, payload: memoryview):
    """Decode a payload for the common primitive types. Unknown types return raw bytes."""
    t = entry_type
    if t == "double":
        return struct.unpack("<d", payload)[0] if len(payload) == 8 else None
    if t == "boolean":
        return bool(payload[0]) if len(payload) else None
    if t == "string" or t == "json":
        return bytes(payload).decode("utf-8", "replace")
    if t == "int64":
        return struct.unpack("<q", payload)[0] if len(payload) == 8 else None
    if t == "float":
        return struct.unpack("<f", payload)[0] if len(payload) == 4 else None
    if t == "double[]":
        return list(struct.unpack(f"<{len(payload) // 8}d", payload[: len(payload) // 8 * 8]))
    if t == "float[]":
        return list(struct.unpack(f"<{len(payload) // 4}f", payload[: len(payload) // 4 * 4]))
    if t == "int64[]":
        return list(struct.unpack(f"<{len(payload) // 8}q", payload[: len(payload) // 8 * 8]))
    if t == "boolean[]":
        return [bool(b) for b in bytes(payload)]
    if t == "string[]":
        buf = bytes(payload)
        if len(buf) < 4:
            return []
        (cnt,) = struct.unpack_from("<I", buf, 0)
        out = []
        pos = 4
        for _ in range(cnt):
            if pos + 4 > len(buf):
                break
            s, pos = _read_str(buf, pos)
            out.append(s)
        return out
    if t == "struct:Pose2d" and len(payload) == 24:
        x, y, th = struct.unpack("<3d", payload)
        return (x, y, th)
    if t == "struct:Translation2d" and len(payload) == 16:
        return struct.unpack("<2d", payload)
    if t == "struct:Rotation2d" and len(payload) == 8:
        return struct.unpack("<d", payload)[0]
    if t == "struct:ChassisSpeeds" and len(payload) == 24:
        return struct.unpack("<3d", payload)
    return bytes(payload)


def short_name(name: str) -> str:
    """Strip DogLog's ``/Robot/`` prefix and any leading slash: ``/Robot/Launcher/RPM`` → ``Launcher/RPM``."""
    if name.startswith("/Robot/"):
        return name[7:]
    if name.startswith("NT:/Robot/"):
        return name[10:]
    return name.lstrip("/")


def find_logs(paths: list[str], latest: int = 1) -> list[Path]:
    """Expand files/directories to concrete .wpilog paths, newest first for directories."""
    out: list[Path] = []
    for raw in paths:
        p = Path(raw).expanduser()
        if p.is_dir():
            sub = p / "SimLogs" if (p / "SimLogs").is_dir() and not list(p.glob("*.wpilog")) else p
            logs = sorted(sub.glob("*.wpilog"), key=lambda q: q.stat().st_mtime, reverse=True)
            if not logs:
                raise SystemExit(f"No .wpilog files in {sub}")
            out.extend(logs[:latest])
        elif p.is_file():
            out.append(p)
        else:
            raise SystemExit(f"Not found: {p}")
    return out
