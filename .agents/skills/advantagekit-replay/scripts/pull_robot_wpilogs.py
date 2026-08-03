#!/usr/bin/env python3
"""Pull WPILOG files from a roboRIO /U directory."""

from __future__ import annotations

import argparse
import shlex
import subprocess
from pathlib import Path


def remote(user: str, host: str, path: str) -> str:
    return f"{user}@{host}:{path}"


def newest_remote_log(user: str, host: str, directory: str) -> str:
    quoted_dir = shlex.quote(directory)
    cmd = [
        "ssh",
        f"{user}@{host}",
        f"ls -t {quoted_dir}/*.wpilog 2>/dev/null | head -1",
    ]
    result = subprocess.run(cmd, check=True, text=True, capture_output=True)
    log = result.stdout.strip()
    if not log:
        raise SystemExit(f"No .wpilog files found in {user}@{host}:{directory}")
    return log


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="10.80.44.2", help="Robot IP")
    parser.add_argument("--user", default="admin", help="Robot SSH user")
    parser.add_argument("--remote-dir", default="/U", help="Robot log directory")
    parser.add_argument("--out", default="RobotLogs", help="Local output directory")
    parser.add_argument("--all", action="store_true", help="Copy all WPILOG files instead of only newest")
    args = parser.parse_args()

    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    if args.all:
        source = remote(args.user, args.host, f"{args.remote_dir}/*.wpilog")
    else:
        source = remote(args.user, args.host, newest_remote_log(args.user, args.host, args.remote_dir))

    subprocess.run(["scp", source, str(out)], check=True)
    print(f"copiedTo={out}")


if __name__ == "__main__":
    main()
