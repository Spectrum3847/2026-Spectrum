#!/usr/bin/env python3
"""Find the match in each Limelight Rewind recording, trim the video to it, and package a .rwnd.

Offline: nothing here touches the network. Run ``pull_match_data.py`` first (it calls this
when it is done), or point this at recordings you already have.

How the match is found
----------------------
Every manifest line carries ``ts_nt``, the NetworkTables time of that frame in microseconds.
NT time is the roboRIO's own clock, the same one every ``.wpilog`` record is stamped with, so a
frame at ``ts_nt`` = 233.8 s is the same instant as a wpilog record at 233.8 s. The match window
is read from ``DS:enabled`` / ``DS:autonomous`` in that log.

NT time restarts at every roboRIO boot, so the same 200-400 s range shows up in many logs (TXMAN1
Q17 and Q20 overlap). The log is *confirmed* by comparing the Limelight's own ``botpose`` in the
manifest with the ``Vision/<camera>/MT1Pose`` / ``MT2Pose`` the robot logged at the same NT time:
the right log agrees to a few centimetres, a wrong one is metres off. With no tags in view there is
nothing to compare and the result is marked UNCONFIRMED.

What comes out
--------------
``<out>/<EVENT>_<MATCH>/<EVENT>_<MATCH>_<limelight>.rwnd``: a zip (stored, no compression) of
the trimmed ``.avi``, the trimmed ``_manifest.jsonl`` and the original ``_bootlog.txt.gz``, which is
exactly what the Limelight UI downloads and what https://tools.limelightvision.io/rewind-viewer
loads. The viewer pairs AVI frame *i* with manifest line *i*, and syncs slots on ``ts_nt``, so all
three cameras line up when loaded side by side. A ``README.txt`` next to them says which wpilog
matched and what was cut.

    python package_rewind.py                       # logs/rewind/raw -> logs/rewind/upload
    python package_rewind.py --pre 5 --post 3      # wider margins around the match
    python package_rewind.py --wpilog path/to.wpilog   # skip the search, use this log
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import shutil
import statistics
import struct
import sys
import time
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(REPO / ".agents" / "skills" / "fast-log-triage" / "scripts"))
from wpilog_fast import decode, iter_records, open_log  # noqa: E402

DEFAULT_RAW = REPO / "logs" / "rewind" / "raw"
DEFAULT_OUT = REPO / "logs" / "rewind" / "upload"
MATCH_NAME_RE = re.compile(r"^FRC_\d{8}_\d{6}_(?P<event>[A-Za-z0-9]+)_(?P<match>[A-Za-z0-9]+)$")
POSE_CONFIRM_M = 0.30  # median botpose disagreement below this = same match
POSE_MIN_SAMPLES = 20
POSE_MAX_DT_US = 30_000


def default_log_dirs() -> list[Path]:
    home = Path.home()
    public = Path(os.environ.get("PUBLIC", "C:/Users/Public"))
    return [
        REPO / "rio-logs",
        home / "Documents" / "Logs",
        public / "Documents" / "FRC" / "Log Files" / "WPILogs",
        REPO / "logs" / "matches",
        REPO / "logs",
    ]


# ── recordings ────────────────────────────────────────────────────────────


@dataclass
class Recording:
    camera: str  # limelight hostname, from the folder name
    name: str  # rewind_YYYYMMDD_HHMMSS
    folder: Path
    header: dict
    lines: list[str]  # raw manifest frame lines, one per AVI frame
    ts_nt: list[int | None]
    poses: list[tuple[int, list, list]]  # (ts_nt, botpose_wpiblue, botpose_orb_wpiblue) where tags seen

    @property
    def avi(self) -> Path:
        return self.folder / f"{self.name}.avi"

    @property
    def bootlog(self) -> Path:
        return self.folder / f"{self.name}_bootlog.txt.gz"

    def span_us(self) -> tuple[int, int] | None:
        good = [t for t in self.ts_nt if t]
        return (good[0], good[-1]) if good else None


def load_recording(manifest: Path) -> Recording:
    name = manifest.name[: -len("_manifest.jsonl")]
    with manifest.open("r", encoding="utf-8") as fh:
        header = json.loads(fh.readline())
        lines = [ln.rstrip("\n") for ln in fh if ln.strip()]
    ts_nt: list[int | None] = []
    poses = []
    for ln in lines:
        try:
            r = json.loads(ln).get("results", {})
        except json.JSONDecodeError:
            ts_nt.append(None)
            continue
        t = r.get("ts_nt")
        t = int(t) if isinstance(t, (int, float)) and t > 0 and r.get("ntconnected", 1) else None
        ts_nt.append(t)
        if t and r.get("botpose_tagcount", 0) > 0:
            poses.append((t, r.get("botpose_wpiblue") or [], r.get("botpose_orb_wpiblue") or []))
    return Recording(manifest.parent.name, name, manifest.parent, header, lines, ts_nt, poses)


def find_recordings(raw: Path) -> list[Recording]:
    return [load_recording(m) for m in sorted(raw.glob("*/*_manifest.jsonl")) if (m.parent / (m.name[: -len("_manifest.jsonl")] + ".avi")).exists()]


# ── robot logs ────────────────────────────────────────────────────────────


@dataclass
class RobotLog:
    path: Path
    first_us: int = 0
    last_us: int = 0
    segments: list[tuple[str, int, int]] = field(default_factory=list)  # (auto|teleop|test, start, end)
    meta: dict = field(default_factory=dict)
    poses: dict[str, tuple[list[int], list[tuple[float, float]]]] = field(default_factory=dict)

    def match_name(self) -> str:
        m = MATCH_NAME_RE.match(self.path.stem)
        if m:
            return f"{m['event']}_{m['match']}"
        ev, typ, num = self.meta.get("EventName"), self.meta.get("MatchType"), self.meta.get("MatchNumber")
        if ev and num and str(num) != "0":
            prefix = {"Qualification": "Q", "Practice": "P", "Elimination": "E"}.get(str(typ), str(typ or "M")[:1])
            return f"{ev}_{prefix}{num}"
        return self.path.stem


_log_cache: dict[Path, RobotLog] = {}


def read_log(path: Path) -> RobotLog:
    if path in _log_cache:
        return _log_cache[path]
    rl = RobotLog(path)
    log, data, pos = open_log(path)

    def want(e):
        n = e.name
        return n in ("DS:enabled", "DS:autonomous", "DS:test") or "/Match Data/" in n or (("/Vision/" in n) and n.endswith(("MT1Pose", "MT2Pose")))

    bools: dict[str, list[tuple[int, bool]]] = {"DS:enabled": [], "DS:autonomous": [], "DS:test": []}
    for e, ts, payload in iter_records(log, data, pos, want):
        v = decode(e.type, payload)
        if v is None:
            continue
        if e.name in bools:
            bools[e.name].append((ts, bool(v)))
        elif "/Match Data/" in e.name:
            rl.meta[e.name.rsplit("/", 1)[-1]] = v
        elif isinstance(v, tuple) and len(v) == 3:
            key = e.name.split("/Vision/", 1)[1]
            tsl, xy = rl.poses.setdefault(key, ([], []))
            tsl.append(ts)
            xy.append((v[0], v[1]))
    del data
    rl.first_us, rl.last_us = log.first_us or 0, log.last_us or 0

    def value_at(series, t):
        val = False
        for ts, v in series:
            if ts > t:
                break
            val = v
        return val

    start = None
    for ts, v in bools["DS:enabled"] + [(rl.last_us, False)]:
        if v and start is None:
            start = ts
        elif not v and start is not None:
            kind = "test" if value_at(bools["DS:test"], start) else ("auto" if value_at(bools["DS:autonomous"], start) else "teleop")
            rl.segments.append((kind, start, ts))
            start = None
    _log_cache[path] = rl
    return rl


def candidate_logs(dirs: list[Path], near_mtime: float, days: float) -> list[Path]:
    seen, out = set(), []
    for d in dirs:
        if not d.is_dir():
            continue
        for p in d.glob("*.wpilog"):
            try:
                key = p.resolve()
                st = p.stat()
            except OSError:
                continue
            if key in seen or st.st_size < 50_000 or abs(st.st_mtime - near_mtime) > days * 86400:
                continue
            seen.add(key)
            out.append(p)
    return out


def pose_disagreement(rec: Recording, rl: RobotLog) -> tuple[float | None, int, str]:
    """Median distance (m) between the Limelight's botpose and the best-matching logged Vision pose.

    Every camera reports the *robot* pose, so any camera's topic works; the best one is used.
    """
    best: tuple[float | None, int, str] = (None, 0, "")
    for key, (tsl, xy) in rl.poses.items():
        idx = 2 if key.endswith("MT2Pose") else 1
        d = []
        for t, mt1, mt2 in rec.poses:
            b = mt2 if idx == 2 else mt1
            if len(b) < 2 or (b[0] == 0 and b[1] == 0):
                continue
            i = bisect.bisect_left(tsl, t)
            js = [j for j in (i - 1, i) if 0 <= j < len(tsl) and abs(tsl[j] - t) <= POSE_MAX_DT_US]
            if not js:
                continue
            j = min(js, key=lambda j: abs(tsl[j] - t))
            d.append(math.hypot(xy[j][0] - b[0], xy[j][1] - b[1]))
        if len(d) >= POSE_MIN_SAMPLES:
            med = statistics.median(d)
            if best[0] is None or med < best[0]:
                best = (med, len(d), key)
    return best


@dataclass
class MatchResult:
    log: RobotLog
    confirmed: bool
    why: str
    start_us: int
    end_us: int
    segments: list[tuple[str, int, int]]


def find_match(rec: Recording, logs: list[Path]) -> MatchResult | None:
    span = rec.span_us()
    if not span:
        return None
    r0, r1 = span
    scored = []
    for p in logs:
        try:
            rl = read_log(p)
        except (SystemExit, OSError) as ex:
            print(f"   skip {p.name}: {ex}", file=sys.stderr)
            continue
        if rl.last_us < r0 or rl.first_us > r1:
            continue
        segs = [s for s in rl.segments if s[2] > r0 and s[1] < r1]
        med, n, key = pose_disagreement(rec, rl)
        scored.append((rl, segs, med, n, key))
    if not scored:
        return None
    confirmed = [s for s in scored if s[2] is not None and s[2] < POSE_CONFIRM_M]
    if confirmed:
        rl, segs, med, n, key = min(confirmed, key=lambda s: s[2])
        why = f"botpose agrees with {key} to {med * 100:.1f} cm (median of {n} frames)"
        ok = True
    else:
        # nothing to compare: prefer the log with the most enabled time inside the recording,
        # then the newest file
        def enabled_overlap(s):
            return sum(min(e, r1) - max(b, r0) for _, b, e in s[1])

        rl, segs, med, n, key = max(scored, key=lambda s: (enabled_overlap(s), s[0].path.stat().st_mtime))
        others = len(scored) - 1
        why = "UNCONFIRMED: " + (
            f"poses disagree ({med:.2f} m median via {key})" if med is not None else "no tags in view to compare poses"
        ) + (f"; {others} other log(s) also cover this NT time range" if others else "")
        ok = False
    if segs:
        start, end = min(s[1] for s in segs), max(s[2] for s in segs)
    else:
        start, end = r0, r1
    return MatchResult(rl, ok, why, start, end, segs)


# ── AVI trimming (single-stream MJPEG, as Limelight OS writes it) ─────────


def _chunks(fh, start: int, end: int):
    pos = start
    while pos + 8 <= end:
        fh.seek(pos)
        hdr = fh.read(8)
        if len(hdr) < 8:
            return
        cid, size = hdr[:4], struct.unpack("<I", hdr[4:])[0]
        yield cid, pos, size
        pos += 8 + size + (size & 1)


def avi_frames(path: Path) -> tuple[bytes, list[tuple[int, int]]]:
    """Return (hdrl LIST bytes, [(offset, size) of each 00dc payload])."""
    with path.open("rb") as fh:
        head = fh.read(12)
        if head[:4] != b"RIFF" or head[8:12] != b"AVI ":
            raise ValueError(f"{path.name}: not an AVI")
        riff_end = 8 + struct.unpack("<I", head[4:8])[0]
        hdrl = None
        frames: list[tuple[int, int]] = []
        for cid, pos, size in _chunks(fh, 12, riff_end):
            if cid != b"LIST":
                continue
            fh.seek(pos + 8)
            typ = fh.read(4)
            if typ == b"hdrl":
                fh.seek(pos)
                hdrl = fh.read(8 + size)
            elif typ == b"movi":
                for c2, p2, s2 in _chunks(fh, pos + 12, pos + 8 + size):
                    if c2[2:] in (b"dc", b"db"):
                        frames.append((p2 + 8, s2))
        fh.seek(0, 2)
        if fh.tell() > 0xFFFFFFFF or hdrl is None:
            raise ValueError(f"{path.name}: unsupported AVI layout (OpenDML or no hdrl)")
    return hdrl, frames


def _patch_hdrl(hdrl: bytes, n_frames: int) -> bytes:
    b = bytearray(hdrl)

    def walk(start, end):
        pos = start
        while pos + 8 <= end:
            cid = bytes(b[pos : pos + 4])
            size = struct.unpack_from("<I", b, pos + 4)[0]
            if cid == b"LIST":
                walk(pos + 12, pos + 8 + size)
            elif cid == b"avih":
                struct.pack_into("<I", b, pos + 8 + 16, n_frames)  # dwTotalFrames
            elif cid == b"strh":
                struct.pack_into("<I", b, pos + 8 + 32, n_frames)  # dwLength
            elif cid == b"dmlh":
                struct.pack_into("<I", b, pos + 8, n_frames)  # dwTotalFrames (OpenDML)
            pos += 8 + size + (size & 1)

    walk(12, len(b))
    return bytes(b)


def write_trimmed_avi(src: Path, dst: Path, i0: int, i1: int) -> int:
    hdrl, frames = avi_frames(src)
    keep = frames[i0 : i1 + 1]
    with src.open("rb") as fin, dst.open("wb") as out:
        out.write(b"RIFF\0\0\0\0AVI ")
        out.write(_patch_hdrl(hdrl, len(keep)))
        movi_pos = out.tell()
        out.write(b"LIST\0\0\0\0movi")
        index = []
        for off, size in keep:
            fin.seek(off)
            payload = fin.read(size)
            index.append((out.tell() - (movi_pos + 8), size))
            out.write(b"00dc" + struct.pack("<I", size) + payload + (b"\0" if size & 1 else b""))
        movi_end = out.tell()
        out.write(b"idx1" + struct.pack("<I", 16 * len(index)))
        for rel, size in index:
            out.write(b"00dc" + struct.pack("<III", 0x10, rel, size))  # AVIIF_KEYFRAME: every MJPEG frame is one
        riff_end = out.tell()
        out.seek(4)
        out.write(struct.pack("<I", riff_end - 8))
        out.seek(movi_pos + 4)
        out.write(struct.pack("<I", movi_end - movi_pos - 8))
    return len(keep)


# ── packaging ─────────────────────────────────────────────────────────────


def frame_range(rec: Recording, start_us: int, end_us: int) -> tuple[int, int]:
    idx = [i for i, t in enumerate(rec.ts_nt) if t and start_us <= t <= end_us]
    if not idx:
        return 0, len(rec.lines) - 1
    return idx[0], idx[-1]


def trimmed_manifest(rec: Recording, i0: int, i1: int, extra: dict) -> str:
    header = dict(rec.header)
    n = i1 - i0 + 1
    t = [x for x in rec.ts_nt[i0 : i1 + 1] if x]
    header["frameCount"] = n
    if len(t) >= 2:
        header["durationSeconds"] = round((t[-1] - t[0]) / 1e6, 3)
    if isinstance(header.get("mapChanges"), list):
        header["mapChanges"] = [
            {**c, "frameIndex": c["frameIndex"] - i0} for c in header["mapChanges"] if isinstance(c, dict) and isinstance(c.get("frameIndex"), int) and i0 <= c["frameIndex"] <= i1
        ]
    header["spectrum3847"] = extra
    out = [json.dumps(header, separators=(",", ":"))]
    for k, ln in enumerate(rec.lines[i0 : i1 + 1]):
        try:
            obj = json.loads(ln)
            obj["frameIndex"] = k
            out.append(json.dumps(obj, separators=(",", ":")))
        except json.JSONDecodeError:
            out.append(ln)
    return "\n".join(out) + "\n"


def package(rec: Recording, m: MatchResult | None, out_root: Path, pre_s: float, post_s: float, force_name: str | None) -> dict:
    span = rec.span_us()
    if m:
        w0, w1 = m.start_us - int(pre_s * 1e6), m.end_us + int(post_s * 1e6)
        match = force_name or m.log.match_name()
    else:
        w0, w1 = span if span else (0, 0)
        match = force_name or f"unmatched_{rec.name}"
    i0, i1 = frame_range(rec, w0, w1)
    _, frames = avi_frames(rec.avi)
    if len(frames) != len(rec.lines):
        print(f"   WARNING {rec.camera}: AVI has {len(frames)} frames, manifest {len(rec.lines)}; trimming by the shorter", file=sys.stderr)
    i1 = min(i1, len(frames) - 1, len(rec.lines) - 1)

    base = f"{match}_{rec.camera}"
    dest = out_root / match
    dest.mkdir(parents=True, exist_ok=True)
    tmp = dest / f".{base}.tmp"
    tmp.mkdir(exist_ok=True)
    try:
        n = write_trimmed_avi(rec.avi, tmp / f"{base}.avi", i0, i1)
        reparsed = len(avi_frames(tmp / f"{base}.avi")[1])
        if reparsed != n:
            raise RuntimeError(f"{base}.avi: wrote {n} frames but reads back {reparsed}")
        extra = {
            "source": rec.name,
            "camera": rec.camera,
            "match": match,
            "wpilog": m.log.path.name if m else None,
            "matchConfirmed": bool(m and m.confirmed),
            "matchEvidence": m.why if m else "no robot log covers this recording",
            "matchStartNtSeconds": round(m.start_us / 1e6, 3) if m else None,
            "matchEndNtSeconds": round(m.end_us / 1e6, 3) if m else None,
            "sourceFrames": [i0, i1],
            "sourceFrameCount": len(rec.lines),
        }
        (tmp / f"{base}_manifest.jsonl").write_text(trimmed_manifest(rec, i0, i1, extra), encoding="utf-8", newline="\n")
        members = [tmp / f"{base}.avi", tmp / f"{base}_manifest.jsonl"]
        if rec.bootlog.exists():
            shutil.copyfile(rec.bootlog, tmp / f"{base}_bootlog.txt.gz")
            members.append(tmp / f"{base}_bootlog.txt.gz")
        rwnd = dest / f"{base}.rwnd"
        with zipfile.ZipFile(rwnd, "w", zipfile.ZIP_STORED) as z:
            for p in members:
                z.write(p, arcname=p.name)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # the viewer pairs AVI frame i with manifest line i, so the counts must agree
    with zipfile.ZipFile(rwnd) as z:
        if z.testzip() is not None:
            raise RuntimeError(f"{rwnd.name}: zip CRC check failed")
        man_lines = z.read(f"{base}_manifest.jsonl").decode().strip().split("\n")
    if len(man_lines) - 1 != n:
        raise RuntimeError(f"{rwnd.name}: {n} video frames but {len(man_lines) - 1} manifest lines")

    t = [x for x in rec.ts_nt[i0 : i1 + 1] if x]
    return {
        "camera": rec.camera,
        "rwnd": rwnd,
        "frames": n,
        "of": len(rec.lines),
        "t0": t[0] / 1e6 if t else None,
        "t1": t[-1] / 1e6 if t else None,
        "mb": rwnd.stat().st_size / 1e6,
        "match": match,
        "m": m,
    }


def fmt_s(us: int) -> str:
    return f"{us / 1e6:.1f}s"


def write_readme(dest: Path, rows: list[dict]) -> Path:
    m: MatchResult | None = rows[0]["m"]
    L = [f"{rows[0]['match']}: Limelight Rewind, trimmed to the match", ""]
    if m:
        L.append(f"Robot log: {m.log.path}")
        L.append("Match windows (NT / wpilog time): " + "; ".join(f"{k} {fmt_s(a)} -> {fmt_s(b)}" for k, a, b in m.segments) if m.segments else "No enabled window inside the recording; kept the whole recording.")
    else:
        L.append("No robot log covered this recording; it was packaged untrimmed.")
    L.append("")
    for r in rows:
        mm = r["m"]
        L.append(
            f"{r['rwnd'].name}: {r['frames']}/{r['of']} frames, NT {r['t0']:.1f}s -> {r['t1']:.1f}s, {r['mb']:.0f} MB"
            + (f"  [{'confirmed' if mm.confirmed else 'UNCONFIRMED'}: {mm.why}]" if mm else "")
        )
    L += [
        "",
        "Upload: open https://tools.limelightvision.io/rewind-viewer, click or drop one .rwnd per slot.",
        "The viewer syncs slots on ts_nt, so the cameras play in step. Find the same moment in",
        "AdvantageScope with the robot log above: wpilog time = NT time.",
        "",
    ]
    p = dest / "README.txt"
    p.write_text("\n".join(L), encoding="utf-8", newline="\n")
    return p


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--raw", type=Path, default=DEFAULT_RAW, help="folder of <limelight>/<rewind files> (default logs/rewind/raw)")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="where the .rwnd files go (default logs/rewind/upload)")
    ap.add_argument("--logs", type=Path, action="append", help="folder of .wpilog files to search (repeatable; default rio-logs, ~/Documents/Logs, the DS log folder, logs/)")
    ap.add_argument("--wpilog", type=Path, help="use this robot log, skip the search")
    ap.add_argument("--days", type=float, default=7.0, help="only search logs modified within this many days of the recording download (default 7)")
    ap.add_argument("--pre", type=float, default=3.0, help="seconds kept before auto starts (default 3)")
    ap.add_argument("--post", type=float, default=3.0, help="seconds kept after teleop ends (default 3)")
    ap.add_argument("--name", help="override the match name used for folder and files")
    ap.add_argument("--only", action="append", help="only this recording name or camera folder (repeatable)")
    ap.add_argument("--redo", action="store_true", help="rebuild .rwnd files that already exist")
    args = ap.parse_args(argv)

    recs = find_recordings(args.raw)
    if args.only:
        recs = [r for r in recs if r.name in args.only or r.camera in args.only]
    if not recs:
        print(f"No rewind recordings under {args.raw}")
        return 1

    groups: dict[Path, list[dict]] = {}
    failures = 0
    for rec in recs:
        t_start = time.time()
        span = rec.span_us()
        print(f"== {rec.camera}/{rec.name}: {len(rec.lines)} frames" + (f", NT {span[0] / 1e6:.1f}s -> {span[1] / 1e6:.1f}s" if span else ", no NT time (Limelight was not connected to the robot)"))
        if args.wpilog:
            logs = [args.wpilog]
        else:
            logs = candidate_logs(args.logs or default_log_dirs(), rec.avi.stat().st_mtime, args.days)
        m = find_match(rec, logs) if span else None
        if m:
            print(f"   {m.log.path.name}: {'confirmed' if m.confirmed else 'best guess'}, {m.why}")
            for k, a, b in m.segments:
                print(f"   {k:6s} {fmt_s(a)} -> {fmt_s(b)} ({(b - a) / 1e6:.1f} s)")
        else:
            print("   no robot log covers this recording; packaging it untrimmed")
        name = args.name or (m.log.match_name() if m else f"unmatched_{rec.name}")
        existing = args.out / name / f"{name}_{rec.camera}.rwnd"
        if existing.exists() and not args.redo:
            print(f"   already packaged: {existing} (--redo to rebuild)")
            continue
        try:
            row = package(rec, m, args.out, args.pre, args.post, args.name)
        except (RuntimeError, ValueError, OSError) as ex:
            print(f"   FAILED: {ex}")
            failures += 1
            continue
        groups.setdefault(row["rwnd"].parent, []).append(row)
        print(f"   -> {row['rwnd']} ({row['frames']}/{row['of']} frames, {row['mb']:.0f} MB, {time.time() - t_start:.1f} s)")

    for dest, rows in groups.items():
        readme = write_readme(dest, rows)
        print(f"\nReady to upload: {dest}")
        for r in rows:
            print(f"   {r['rwnd'].name}")
        print(f"   {readme.name} (what matched, what was cut)")
        if any(r["m"] is None or not r["m"].confirmed for r in rows):
            print("   CHECK: at least one recording's match is UNCONFIRMED; see README.txt")
    if groups:
        print("\nUpload at https://tools.limelightvision.io/rewind-viewer (one .rwnd per slot).")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
