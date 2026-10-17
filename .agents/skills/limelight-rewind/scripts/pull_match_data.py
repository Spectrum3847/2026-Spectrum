#!/usr/bin/env python3
"""Pull the robot logs and every Limelight Rewind recording, say when it is safe to unplug, then
trim and package the rewinds for the Limelight Rewind Viewer.

    python pull_match_data.py                 # roboRIO newest 3 .wpilog + all 3 Limelights, then package
    python pull_match_data.py --no-package    # just pull
    python pull_match_data.py --rio-count 5 --limelight limelight-left=10.38.47.11

Order matters on a match day: everything that needs the robot network happens first, every file
is checked (size against what the device reported; AVI frames against manifest lines), and only
then does the script print SAFE TO DISCONNECT. Trimming and packaging run after that, offline,
with ``package_rewind.py``.

Read-only on every device: nothing on the roboRIO or a Limelight is deleted or changed. Stdlib
only (Python 3.9+), plus the ``ssh``/``scp`` that ships with Windows 10+ and Git Bash.

Where things land (all gitignored):
    rio-logs/*.wpilog                          roboRIO logs (same folder pull_rio_logs.sh uses)
    logs/rewind/raw/<limelight>/rewind_*       raw recordings: .avi, _manifest.jsonl, _bootlog.txt.gz
    logs/rewind/upload/<EVENT>_<MATCH>/*.rwnd  trimmed, ready for tools.limelightvision.io/rewind-viewer
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE))
import package_rewind  # noqa: E402

RIO_HOSTS = ["10.38.47.2", "172.22.11.2", "roboRIO-3847-FRC.local"]
RIO_LOG_GLOBS = "/U/logs/*.wpilog /home/lvuser/logs/*.wpilog"
# Hostnames from Vision.java (backLeftLL, backRightLL, turretLL); each must match the camera's own name.
LIMELIGHTS = ["limelight-left", "limelight-right", "limelight-turret"]
LL_API_PORT = 5807
SSH_OPTS = [
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=4",
    "-o", "StrictHostKeyChecking=no",
    "-o", "UserKnownHostsFile=/dev/null",
    "-o", "LogLevel=ERROR",
]


def say(msg: str = "") -> None:
    print(msg, flush=True)


def port_open(host: str, port: int, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


# ── roboRIO ───────────────────────────────────────────────────────────────


def pull_rio(dest: Path, count: int, hosts: list[str]) -> tuple[list[Path], list[str]]:
    """Copy the newest ``count`` .wpilog files. Returns (files, problems)."""
    host = next((h for h in hosts if port_open(h, 22)), None)
    if host is None:
        return [], [f"roboRIO not reachable on {', '.join(hosts)}"]
    say(f"roboRIO: {host}")
    listing = f'for f in $(ls -t {RIO_LOG_GLOBS} 2>/dev/null | head -n {count}); do echo "$(stat -c %s "$f") $f"; done'
    try:
        r = subprocess.run(["ssh", *SSH_OPTS, f"lvuser@{host}", listing], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as ex:
        return [], [f"roboRIO: ssh failed: {ex}"]
    if r.returncode != 0:
        return [], [f"roboRIO: ssh failed ({r.stderr.strip() or r.returncode}); try AdvantageScope's log download"]
    files, problems = [], []
    dest.mkdir(parents=True, exist_ok=True)
    for line in r.stdout.splitlines():
        try:
            size_s, remote = line.split(" ", 1)
            size = int(size_s)
        except ValueError:
            continue
        local = dest / Path(remote).name
        if local.exists() and local.stat().st_size == size:
            say(f"   have  {local.name} ({size / 1e6:.1f} MB)")
            files.append(local)
            continue
        say(f"   copy  {local.name} ({size / 1e6:.1f} MB)")
        try:
            c = subprocess.run(["scp", *SSH_OPTS, f"lvuser@{host}:{remote}", str(local)], capture_output=True, text=True, timeout=600)
        except (OSError, subprocess.TimeoutExpired) as ex:
            problems.append(f"roboRIO: {local.name}: {ex}")
            continue
        # the newest log is still being written, so it can only grow between ls and scp
        if c.returncode != 0 or not local.exists() or local.stat().st_size < size:
            problems.append(f"roboRIO: {local.name}: copy failed ({c.stderr.strip() or 'short file'})")
            continue
        files.append(local)
    if not r.stdout.strip():
        problems.append("roboRIO: no .wpilog files in /U/logs or /home/lvuser/logs")
    return files, problems


# ── Limelights ────────────────────────────────────────────────────────────


def http_json(url: str, timeout: float = 10.0):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def download(url: str, path: Path, timeout: float = 30.0) -> int:
    tmp = path.with_name(path.name + ".part")
    n = 0
    last = time.time()
    with urllib.request.urlopen(url, timeout=timeout) as resp, tmp.open("wb") as out:
        total = int(resp.headers.get("Content-Length") or 0)
        while True:
            buf = resp.read(1 << 20)
            if not buf:
                break
            out.write(buf)
            n += len(buf)
            if time.time() - last > 5 and total:
                say(f"      {path.name}: {n / total:.0%}")
                last = time.time()
    if total and n != total:
        tmp.unlink(missing_ok=True)
        raise IOError(f"{path.name}: got {n} of {total} bytes")
    tmp.replace(path)
    return n


def wait_for_flush(base: str, name: str, max_wait_s: float = 90.0) -> None:
    """A rewind saved at teleop end takes a while to write; don't list until it is done."""
    t0 = time.time()
    while time.time() - t0 < max_wait_s:
        try:
            rw = http_json(f"{base}/results", timeout=5).get("rewind") or {}
        except (urllib.error.URLError, OSError, ValueError):
            return
        if not rw.get("flushing"):
            return
        say(f"   {name}: still saving a rewind, waiting...")
        time.sleep(3)


def verify_recording(folder: Path, rec: str) -> str | None:
    """AVI frame count must equal manifest frame lines (what the viewer relies on)."""
    try:
        _, frames = package_rewind.avi_frames(folder / f"{rec}.avi")
        with (folder / f"{rec}_manifest.jsonl").open("r", encoding="utf-8") as fh:
            lines = sum(1 for ln in fh if ln.strip()) - 1
    except (OSError, ValueError) as ex:
        return str(ex)
    if len(frames) != lines:
        return f"{rec}: {len(frames)} video frames but {lines} manifest lines"
    return None


def pull_limelight(name: str, host: str, raw: Path) -> tuple[list[str], list[str]]:
    """Download every recording on one Limelight. Returns (recording names, problems)."""
    if not port_open(host, LL_API_PORT, 3.0):
        return [], [f"{name}: not reachable at {host}:{LL_API_PORT}"]
    base = f"http://{host}:{LL_API_PORT}"
    wait_for_flush(base, name)
    try:
        listing = http_json(f"{base}/videolist", timeout=20)
    except (urllib.error.URLError, OSError, ValueError) as ex:
        return [], [f"{name}: /videolist failed: {ex}"]
    if not listing:
        return [], [f"{name}: no rewind recordings saved (the robot saves one at teleop end only with the FMS attached)"]
    folder = raw / name
    folder.mkdir(parents=True, exist_ok=True)
    got, problems = [], []
    for item in listing:
        rec, size = item.get("name"), int(item.get("size") or 0)
        if not rec:
            continue
        avi, man, boot = folder / f"{rec}.avi", folder / f"{rec}_manifest.jsonl", folder / f"{rec}_bootlog.txt.gz"
        have = avi.exists() and man.exists() and (not size or avi.stat().st_size + man.stat().st_size == size)
        if have:
            say(f"   have  {name}/{rec} ({size / 1e6:.0f} MB)")
        else:
            say(f"   copy  {name}/{rec} ({size / 1e6:.0f} MB)")
            try:
                download(f"{base}/recording/{rec}.avi", avi, timeout=60)
                download(f"{base}/recording/{rec}_manifest.jsonl", man, timeout=60)
            except (urllib.error.URLError, OSError) as ex:
                problems.append(f"{name}/{rec}: download failed: {ex}")
                continue
            try:
                download(f"{base}/recording/{rec}_bootlog.txt.gz", boot)
            except (urllib.error.URLError, OSError):
                pass  # optional; the viewer loads without it
            # the listed size is exactly video + manifest
            if size and avi.stat().st_size + man.stat().st_size != size:
                problems.append(f"{name}/{rec}: size {avi.stat().st_size + man.stat().st_size} != listed {size}")
                continue
        bad = verify_recording(folder, rec)
        if bad:
            problems.append(f"{name}: {bad}")
            continue
        got.append(rec)
    return got, problems


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--rio-count", type=int, default=3, help="newest N .wpilog files from the roboRIO (default 3)")
    ap.add_argument("--rio-host", action="append", help=f"roboRIO address (repeatable; default {', '.join(RIO_HOSTS)})")
    ap.add_argument("--no-rio", action="store_true", help="skip the roboRIO")
    ap.add_argument("--limelight", action="append", metavar="NAME[=HOST]", help="Limelight to pull (repeatable; default the three in Vision.java at NAME.local)")
    ap.add_argument("--rio-dest", type=Path, default=REPO / "rio-logs")
    ap.add_argument("--raw", type=Path, default=package_rewind.DEFAULT_RAW)
    ap.add_argument("--no-package", action="store_true", help="pull only; do not trim and package")
    ap.add_argument("--pre", type=float, default=3.0, help="seconds kept before auto (default 3)")
    ap.add_argument("--post", type=float, default=3.0, help="seconds kept after teleop (default 3)")
    args = ap.parse_args(argv)

    t0 = time.time()
    problems: list[str] = []
    copied: list[str] = []

    say("== Pulling (keep the robot connected) ==")
    if not args.no_rio:
        files, p = pull_rio(args.rio_dest, args.rio_count, args.rio_host or RIO_HOSTS)
        copied += [f"rio-logs/{f.name}" for f in files]
        problems += p
    lls = []
    for spec in args.limelight or LIMELIGHTS:
        name, _, host = spec.partition("=")
        lls.append((name, host or f"{name}.local"))
    for name, host in lls:
        recs, p = pull_limelight(name, host, args.raw)
        copied += [f"{name}/{r}" for r in recs]
        problems += p

    say("")
    if problems:
        say("!! NOT SAFE TO DISCONNECT YET: something did not come off the robot !!")
        for p in problems:
            say(f"   - {p}")
        say("   Fix the connection and run this again (files already copied are skipped).")
        say("   If a device simply has nothing to pull, unplugging loses nothing from it.")
    else:
        say("############################################################")
        say("##  SAFE TO DISCONNECT: every file is copied and verified.  ##")
        say("##  Unplug the tether / power the robot down now.           ##")
        say("############################################################")
    say(f"   {len(copied)} item(s) in {time.time() - t0:.0f} s: " + ", ".join(copied))

    if args.no_package:
        return 1 if problems else 0
    say("")
    say("== Trimming and packaging (offline) ==")
    rc = package_rewind.main(["--raw", str(args.raw), "--logs", str(args.rio_dest), *sum((["--logs", str(d)] for d in package_rewind.default_log_dirs()), []), "--pre", str(args.pre), "--post", str(args.post)])
    return 1 if (problems or rc) else 0


if __name__ == "__main__":
    sys.exit(main())
