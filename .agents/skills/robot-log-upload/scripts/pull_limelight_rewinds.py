#!/usr/bin/env python3
"""List, download and pack Limelight rewind clips into .rwnd files the log site can read.

    python pull_limelight_rewinds.py list   [--team 3847 | --camera HOST ...]
    python pull_limelight_rewinds.py pull   [--team ... | --camera ...] (--latest N | --clip NAME ...) [-o DEST]
    python pull_limelight_rewinds.py pack   FOLDER [-o DEST]

Works on Windows, macOS and Linux; stdlib only. Read-only on the cameras.

Each Limelight serves its clips over HTTP on port 5807: ``/videolist`` lists them as JSON
({name, size}), and ``/recording/<file>`` downloads ``<clip>.avi``, ``<clip>_manifest.jsonl`` and
``<clip>_bootlog.txt.gz``. The site wants one ``.rwnd`` per clip: an uncompressed zip of those three
files, the same thing the camera's web UI "Download" button saves. ``pack`` builds that from clips
already on disk.

Camera clocks reset every boot, so clip names (``rewind_20251013_161032``) are time since boot, not
the match time; the site pairs a rewind with its match by the robot clock inside the manifest.
Clips are big (about 360 MB for a 165 s capture), so ``list`` first and pull only what is needed.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

PORT = 5807
SIDECARS = (".avi", "_manifest.jsonl", "_bootlog.txt.gz")
CLIP_RE = re.compile(r"^(?P<base>.+?)(\.avi|_manifest\.jsonl|_bootlog\.txt\.gz)$")

# 3847 names match Vision.java on main and 2026-offseason-bot. 8515: back .11, left .12, right .13.
CAMERAS = {
    3847: [f"limelight-{n}.local" for n in ("left", "right", "turret", "back")],
    8515: ["10.85.15.11", "10.85.15.12", "10.85.15.13"],
}


def cameras(args) -> list[str]:
    if args.camera:
        return args.camera
    if args.team in CAMERAS:
        return CAMERAS[args.team]
    sys.exit(f"no camera list for team {args.team}; pass --camera HOST (repeatable)")


def get(url: str, timeout: float = 5):
    return urllib.request.urlopen(url, timeout=timeout)


def clips_on(host: str) -> list[dict]:
    """[{base, size}] for each clip on a camera, in the order the camera lists them."""
    with get(f"http://{host}:{PORT}/videolist") as r:
        items = json.load(r)
    if isinstance(items, dict):  # tolerate {"videos": [...]} style wrappers
        items = next((v for v in items.values() if isinstance(v, list)), [])
    out: dict[str, int] = {}
    for it in items:
        name = it["name"] if isinstance(it, dict) else str(it)
        size = int(it.get("size", 0)) if isinstance(it, dict) else 0
        m = CLIP_RE.match(name)
        base = m.group("base") if m else name
        out[base] = out.get(base, 0) + size
    return [{"base": b, "size": s} for b, s in out.items()]


def download(url: str, path: Path) -> None:
    tmp = path.with_suffix(path.suffix + ".part")
    with get(url, timeout=30) as r, tmp.open("wb") as fh:
        total = int(r.headers.get("Content-Length") or 0)
        done = 0
        while chunk := r.read(1 << 20):
            fh.write(chunk)
            done += len(chunk)
            if total:
                print(f"\r    {path.name}: {done * 100 // total}%", end="", file=sys.stderr)
    print(file=sys.stderr)
    tmp.replace(path)


def pack(folder: Path, base: str, dest: Path, camera: str | None = None) -> Path:
    """Stored (uncompressed) zip of a clip's files: the .avi must be stored so the site can stream
    frames out of it with HTTP Range reads."""
    avi = folder / f"{base}.avi"
    if not avi.exists():
        raise FileNotFoundError(avi)
    name = f"{camera}_{base}.rwnd" if camera else f"{base}.rwnd"
    out = dest / name
    tmp = out.with_suffix(".rwnd.part")
    with zipfile.ZipFile(tmp, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as z:
        for suffix in SIDECARS:
            f = folder / f"{base}{suffix}"
            if f.exists():
                z.write(f, f.name)
            elif suffix != "_bootlog.txt.gz":
                print(f"  warning: {f.name} missing; the site needs it to sync frames", file=sys.stderr)
    tmp.replace(out)
    return out


def cmd_list(args) -> int:
    ok = False
    for host in cameras(args):
        try:
            clips = clips_on(host)
        except (OSError, ValueError) as e:
            print(f"{host}: unreachable ({e})")
            continue
        ok = True
        print(f"{host}: {len(clips)} clip(s)")
        for c in clips:
            print(f"  {c['base']}  {c['size'] / 1e6:.0f} MB")
    return 0 if ok else 1


def cmd_pull(args) -> int:
    dest = Path(args.out)
    dest.mkdir(parents=True, exist_ok=True)
    failed = 0
    for host in cameras(args):
        try:
            clips = clips_on(host)
        except (OSError, ValueError) as e:
            print(f"{host}: unreachable ({e})", file=sys.stderr)
            failed += 1
            continue
        names = [c["base"] for c in clips]
        if args.clip:
            wanted = {CLIP_RE.sub(r"\g<base>", c) for c in args.clip}
            want = [b for b in names if b in wanted]
            for missing in wanted - set(want):
                print(f"{host}: no clip {missing}", file=sys.stderr)
        else:
            want = sorted(names)[-args.latest:] if args.latest > 0 else []
        camera = host.split(".")[0] if not host[0].isdigit() else host.replace(".", "-")
        raw = dest / "raw" / camera
        raw.mkdir(parents=True, exist_ok=True)
        for base in want:
            print(f"{host}: {base}", file=sys.stderr)
            try:
                for suffix in SIDECARS:
                    try:
                        download(f"http://{host}:{PORT}/recording/{base}{suffix}", raw / f"{base}{suffix}")
                    except urllib.error.HTTPError as e:
                        if suffix != "_bootlog.txt.gz" or e.code != 404:
                            raise
                print(pack(raw, base, dest, camera))
            except OSError as e:
                failed += 1
                print(f"  FAILED: {e}", file=sys.stderr)
    return 1 if failed else 0


def cmd_pack(args) -> int:
    folder = Path(args.folder)
    dest = Path(args.out) if args.out else folder
    dest.mkdir(parents=True, exist_ok=True)
    bases = sorted(p.stem for p in folder.glob("*.avi"))
    if not bases:
        print(f"no .avi clips in {folder}", file=sys.stderr)
        return 1
    for base in bases:
        print(pack(folder, base, dest, args.camera or folder.name))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("list", "pull"):
        p = sub.add_parser(name)
        p.add_argument("--team", type=int, default=3847, help="pick the known camera list (3847 or 8515)")
        p.add_argument("--camera", action="append", help="camera host or IP (repeatable); overrides --team")
    pull = sub.choices["pull"]
    g = pull.add_mutually_exclusive_group(required=True)
    g.add_argument("--latest", type=int, help="newest N clips per camera, by clip name")
    g.add_argument("--clip", action="append", help="clip name from `list` (repeatable)")
    pull.add_argument("-o", "--out", default="rewinds", help="where .rwnd files go (default: rewinds)")
    pk = sub.add_parser("pack")
    pk.add_argument("folder", help="folder holding <clip>.avi, <clip>_manifest.jsonl, <clip>_bootlog.txt.gz")
    pk.add_argument("-o", "--out", help="where .rwnd files go (default: the same folder)")
    pk.add_argument("--camera", help="camera name for the file name (default: the folder name)")
    a = ap.parse_args()
    return {"list": cmd_list, "pull": cmd_pull, "pack": cmd_pack}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
