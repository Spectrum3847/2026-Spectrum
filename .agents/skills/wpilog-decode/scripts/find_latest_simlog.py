#!/usr/bin/env python3
"""Print the newest WPILOG in a repo's SimLogs directory or a log directory."""

from __future__ import annotations

import argparse
from pathlib import Path


def latest_wpilog(path: Path) -> Path:
    directory = path / "SimLogs" if (path / "SimLogs").is_dir() else path
    logs = sorted(directory.glob("*.wpilog"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit(f"No .wpilog files found in {directory}")
    return logs[-1].resolve()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", nargs="?", default=".", help="Repo root, SimLogs directory, or log directory")
    args = parser.parse_args()
    print(latest_wpilog(Path(args.path)))


if __name__ == "__main__":
    main()
