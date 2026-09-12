#!/usr/bin/env python3
"""Compatibility wrapper for the wpilog-decode skill."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> None:
    """Find and print the time window of interest in a WPILOG file."""
    target = Path(__file__).resolve().parents[2] / "wpilog-decode/scripts/find_wpilog_window.py"
    raise SystemExit(subprocess.run([sys.executable, str(target), *sys.argv[1:]], check=False).returncode)


if __name__ == "__main__":
    main()
