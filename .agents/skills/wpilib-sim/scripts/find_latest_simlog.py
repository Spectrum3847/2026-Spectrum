#!/usr/bin/env python3
"""Compatibility wrapper for the wpilog-decode skill."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> None:
    target = Path(__file__).resolve().parents[2] / "wpilog-decode/scripts/find_latest_simlog.py"
    subprocess.run([sys.executable, str(target), *sys.argv[1:]], check=True)


if __name__ == "__main__":
    main()
