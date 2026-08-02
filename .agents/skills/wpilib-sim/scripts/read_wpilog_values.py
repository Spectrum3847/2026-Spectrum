#!/usr/bin/env python3
"""Compatibility wrapper for the wpilog-decode skill."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> None:
    target = Path(__file__).resolve().parents[2] / "wpilog-decode/scripts/read_wpilog_values.py"
    raise SystemExit(subprocess.run([sys.executable, str(target), *sys.argv[1:]], check=False).returncode)


if __name__ == "__main__":
    main()
