#!/usr/bin/env python3
"""Copy real match logs into logs/matches/ so they are committed next to the code.

Every parser and detector in this repo (the robot app's wpilog.js, the fast-log-triage
scripts) was first written against synthetic logs, and every one of them has been wrong
about a real match log in some way the synthetic ones could not show. This keeps a small
set of real match logs in git so ``npm test`` and the triage script always have real
data to run against, on every clone, with no second repository to find.

A "match log" is a .wpilog that WPILib renamed with the event and match once the FMS
attached: ``FRC_YYYYMMDD_HHMMSS_<event>_<match>.wpilog`` (``..._cc_P8.wpilog``,
``..._TXDRI1_Q20.wpilog``). Practice and shop logs never get that suffix, which is what
keeps this folder to a handful of files instead of the 1.5 GB in ``logs/``. A log that
matters but is not match-named (the restart half of a match, say) can be forced in with
``--include``.

    python tools/copy-match-logs.py                      # scan the usual download folders
    python tools/copy-match-logs.py D:/some/folder a.wpilog
    python tools/copy-match-logs.py --include ~/Documents/Logs/FRC_20260919_034444.wpilog
    python tools/copy-match-logs.py --dry-run

Nothing is deleted and nothing on the roboRIO is touched: this reads folders on this
laptop. Get logs off the RIO first with
``.agents/skills/fast-log-triage/scripts/pull_rio_logs.sh`` or AdvantageScope.

Size rules: GitHub refuses files over 100 MB and warns over 50 MB, and every byte here is
paid for by every clone forever. A file over --max-mb (default 50) is refused; the
2026-Robot-Logs archive (tools/archive-logs.sh) is where big logs go. Phoenix ``.hoot``
files are never copied for the same reason: the Q20 set alone is 139 MB.

Each copied file is read back with the fast-log-triage reader, so a truncated download is
caught here rather than by a failing test later.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEST = REPO / "logs" / "matches"
SKILL_SCRIPTS = REPO / ".agents" / "skills" / "fast-log-triage" / "scripts"

MATCH_RE = re.compile(r"^FRC_\d{8}_\d{6}_(?P<event>[A-Za-z0-9]+)_(?P<match>[A-Za-z0-9]+)\.wpilog$")
HARD_LIMIT_MB = 95.0


def default_sources() -> list[Path]:
    """Where logs land on a team laptop: AdvantageScope downloads, the Driver Station's own
    WPILog folder, the repo's scratch logs/ folder, and pull_rio_logs.sh's default folder."""
    home = Path.home()
    public = Path(os.environ.get("PUBLIC", "C:/Users/Public"))
    return [
        home / "Documents" / "Logs",
        public / "Documents" / "FRC" / "Log Files" / "WPILogs",
        REPO / "logs",
        REPO / "rio-logs",
    ]


def is_wpilog(path: Path) -> bool:
    try:
        with path.open("rb") as fh:
            return fh.read(6) == b"WPILOG"
    except OSError:
        return False


def verify(path: Path) -> str:
    """Parse the copy with the fast reader; return a one-line description or raise."""
    sys.path.insert(0, str(SKILL_SCRIPTS))
    from wpilog_fast import iter_records, open_log  # noqa: E402

    log, data, pos = open_log(path)
    enabled_records = 0
    for entry, _ts, payload in iter_records(log, data, pos):
        if entry.name == "DS:enabled" and payload and payload[0]:
            enabled_records += 1
    if log.truncated:
        raise ValueError("file is truncated: the last record is incomplete")
    if not log.entries:
        raise ValueError("no entries: not a robot log")
    dur = ((log.last_us or 0) - (log.first_us or 0)) / 1e6
    note = "" if enabled_records else "  (never enabled: is this the right log?)"
    return f"{dur:.0f} s, {len(log.entries)} topics{note}"


def candidates(paths: list[Path], includes: list[Path]) -> list[tuple[Path, str]]:
    """(source file, reason) for every file worth copying, de-duplicated by name."""
    seen: dict[str, tuple[Path, str]] = {}
    for p in paths:
        p = p.expanduser()
        if p.is_dir():
            if p.resolve() == DEST.resolve():
                continue
            for f in sorted(p.glob("*.wpilog")):
                if MATCH_RE.match(f.name):
                    seen.setdefault(f.name, (f, "match-named"))
        elif p.is_file():
            if MATCH_RE.match(p.name):
                seen.setdefault(p.name, (p, "match-named"))
            else:
                print(f"skip  {p.name}: not match-named (use --include to force it)")
    for p in includes:
        p = p.expanduser()
        if not p.is_file():
            sys.exit(f"--include {p}: no such file")
        seen[p.name] = (p, "--include")
    return list(seen.values())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help="folders or .wpilog files to scan (default: the usual download folders)")
    ap.add_argument("--include", action="append", default=[], metavar="FILE", help="copy this log even though it is not match-named")
    ap.add_argument("--max-mb", type=float, default=50.0, help="refuse files larger than this (default 50; GitHub warns at 50, blocks at 100)")
    ap.add_argument("--force", action="store_true", help="overwrite a file already in logs/matches that differs in size")
    ap.add_argument("--dry-run", action="store_true", help="say what would be copied, copy nothing")
    args = ap.parse_args()

    sources = [Path(p) for p in args.paths] if args.paths else default_sources()
    if not args.paths:
        print("scanning: " + ", ".join(str(s) for s in sources if s.exists()))
    found = candidates(sources, [Path(p) for p in args.include])
    if not found:
        print("no match-named .wpilog files found; pass a folder or file, or --include one")
        return 1

    DEST.mkdir(parents=True, exist_ok=True)
    copied: list[Path] = []
    problems = 0
    for src, reason in sorted(found, key=lambda x: x[0].name):
        mb = src.stat().st_size / 1e6
        dst = DEST / src.name
        if not is_wpilog(src):
            print(f"skip  {src.name}: not a WPILOG file")
            problems += 1
            continue
        if mb > min(args.max_mb, HARD_LIMIT_MB):
            print(f"skip  {src.name}: {mb:.1f} MB is over the {min(args.max_mb, HARD_LIMIT_MB):.0f} MB limit; archive it with tools/archive-logs.sh instead")
            problems += 1
            continue
        if dst.exists():
            if dst.stat().st_size == src.stat().st_size:
                print(f"have  {src.name} ({mb:.1f} MB)")
                continue
            if not args.force:
                print(f"skip  {src.name}: already in logs/matches with a different size ({dst.stat().st_size} vs {src.stat().st_size} bytes); --force to replace")
                problems += 1
                continue
        if args.dry_run:
            print(f"would copy  {src.name} ({mb:.1f} MB, {reason}) from {src.parent}")
            continue
        shutil.copy2(src, dst)
        try:
            desc = verify(dst)
        except Exception as ex:  # a bad copy must not stay in the folder
            dst.unlink(missing_ok=True)
            print(f"FAIL  {src.name}: {ex}; not kept")
            problems += 1
            continue
        print(f"copied {src.name} ({mb:.1f} MB, {reason}): {desc}")
        copied.append(dst)

    if copied:
        print()
        print("Now describe each new log in logs/matches/README.md, then:")
        print("    git add logs/matches && git commit")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
