#!/usr/bin/env python3
"""Copy the newest .wpilog files off a roboRIO or a SystemCore into a local folder.

    python pull_robot_logs.py [--team 3847] [--host ADDR] [-n 3] [--interactive] [DEST]

Works on Windows, macOS and Linux with the OpenSSH ``ssh``/``scp`` each ships with; stdlib only.

Tries, in order: the robot address on the radio (10.TE.AM.2), SystemCore over USB (172.26.0.1),
roboRIO over USB (172.22.11.2), then the roboRIO's mDNS name. On each host it tries the SystemCore
user (``systemcore``, logs in /U/logs or /home/systemcore/logs) and the roboRIO user (``lvuser``,
/U/logs or /home/lvuser/logs). Read-only: nothing on the robot is changed or deleted.

Auth: SystemCore takes this laptop's SSH key (installed by the GradleRIO deploy connection);
``lvuser`` on a roboRIO has no password. Runs non-interactive by default so it never hangs on a
prompt; if a host asks for a password, rerun with ``--interactive`` in your own terminal and press
Enter at the prompt (empty password).
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

USERS = ("systemcore", "lvuser")


def hosts_for(team: int) -> list[str]:
    te, am = divmod(team, 100)
    return [f"10.{te}.{am}.2", "172.26.0.1", "172.22.11.2", f"roboRIO-{team}-FRC.local"]


def ssh_opts(interactive: bool) -> list[str]:
    return [
        "-o", f"BatchMode={'no' if interactive else 'yes'}",
        "-o", "ConnectTimeout=4",
        "-o", "StrictHostKeyChecking=no",
        "-o", f"UserKnownHostsFile={os.devnull}",
        "-o", "LogLevel=ERROR",
    ]


def list_logs(user: str, host: str, count: int, interactive: bool) -> list[str] | None:
    """Newest-first remote .wpilog paths, or None if this user@host is not reachable."""
    cmd = f"ls -t /U/logs/*.wpilog /home/{user}/logs/*.wpilog 2>/dev/null | head -n {count}; true"
    try:
        r = subprocess.run(
            ["ssh", *ssh_opts(interactive), f"{user}@{host}", cmd],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30,
        )
    except subprocess.TimeoutExpired:
        return None
    if r.returncode != 0:
        msg = r.stderr.strip().splitlines()[-1:] or [""]
        print(f"  {user}@{host}: {msg[0] or 'no connection'}", file=sys.stderr)
        return None
    return [line.strip() for line in r.stdout.splitlines() if line.strip()]


def copy(user: str, host: str, remote: str, dest: Path, interactive: bool) -> bool:
    base = ["scp", *ssh_opts(interactive)]
    src = f"{user}@{host}:{remote}"
    # Newer OpenSSH scp talks SFTP; -O falls back to the classic protocol if the robot has no sftp.
    for extra in ([], ["-O"]):
        if subprocess.run([*base, *extra, src, str(dest)]).returncode == 0:
            return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dest", nargs="?", default="robot-logs", help="local folder (default: robot-logs)")
    ap.add_argument("-n", "--count", type=int, default=1, help="how many of the newest logs (default 1)")
    ap.add_argument("--team", type=int, default=3847, help="team number, for the 10.TE.AM.2 address (default 3847)")
    ap.add_argument("--host", action="append", help="robot address to try (repeatable); overrides the defaults")
    ap.add_argument("--user", choices=USERS, help="only try this user")
    ap.add_argument("--interactive", action="store_true", help="allow ssh to prompt (run in your own terminal)")
    a = ap.parse_args()

    if not shutil.which("ssh") or not shutil.which("scp"):
        print("ssh/scp not found. Windows: Settings > Optional features > OpenSSH Client.", file=sys.stderr)
        return 2

    dest = Path(a.dest)
    dest.mkdir(parents=True, exist_ok=True)
    users = (a.user,) if a.user else USERS
    for host in a.host or hosts_for(a.team):
        for user in users:
            print(f"trying {user}@{host} ...", file=sys.stderr)
            files = list_logs(user, host, a.count, a.interactive)
            if files is None:
                continue
            kind = "SystemCore" if user == "systemcore" else "roboRIO"
            if not files:
                print(f"connected to {kind} at {host} but found no .wpilog files", file=sys.stderr)
                return 1
            print(f"{kind} at {host}: copying {len(files)} log(s) into {dest}", file=sys.stderr)
            failed = 0
            for f in files:
                print(f"  {f}", file=sys.stderr)
                if not copy(user, host, f, dest, a.interactive):
                    failed += 1
                    print(f"  FAILED: {f}", file=sys.stderr)
            for f in files:
                print(dest / Path(f).name)
            return 1 if failed else 0
    print("could not reach a robot. Check the cable/radio, or pass --host. "
          "If a host asked for a password, rerun with --interactive in your own terminal.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
