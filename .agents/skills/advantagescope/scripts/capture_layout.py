#!/usr/bin/env python3
"""Capture an AdvantageScope layout screenshot with the Team 8044 agent fork."""

from __future__ import annotations

import argparse
import json
import queue
import re
import subprocess
import threading
import time
import urllib.request
from pathlib import Path

from export_preview import find_fork, find_advantagescope, command_from_fork, resolve_assets_path


def post(port: int, token: str, payload: dict) -> dict:
    """POST a JSON command to the AdvantageScope agent-control HTTP server."""
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}",
        data=json.dumps(payload).encode(),
        headers={"authorization": "Bearer " + token, "content-type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        data = json.loads(response.read())
    if not data.get("ok"):
        raise RuntimeError(data)
    return data.get("result", {})


def wait_for_control(process: subprocess.Popen[str]) -> tuple[int, str]:
    """Wait until AdvantageScope prints its agent-control port and token."""
    port: int | None = None
    token: str | None = None
    output: queue.Queue[str] = queue.Queue()

    def reader() -> None:
        """Read AdvantageScope stdout lines into a thread-safe queue."""
        if process.stdout is not None:
            for line in process.stdout:
                output.put(line)

    threading.Thread(target=reader, daemon=True).start()
    start = time.time()
    while time.time() - start < 30 and (port is None or token is None):
        try:
            line = output.get(timeout=0.5)
        except queue.Empty:
            if process.poll() is not None:
                raise SystemExit(f"AdvantageScope exited early with code {process.returncode}")
            continue
        print(line, end="")
        port_match = re.search(r"127\.0\.0\.1:(\d+)", line)
        if port_match:
            port = int(port_match.group(1))
        token_match = re.search(r"AdvantageScope agent token: (\S+)", line)
        if token_match:
            token = token_match.group(1)
    if port is None or token is None:
        raise SystemExit("Timed out waiting for AdvantageScope agent control port/token")
    return port, token


def wait_for_ready(port: int, token: str) -> dict:
    """Poll the AdvantageScope agent-control status until the renderer is ready."""
    last_status: dict | None = None
    for _ in range(300):
        last_status = post(port, token, {"command": "status"})
        live_status = (last_status.get("rendererState") or {}).get("liveStatus") or {}
        if live_status.get("ready") is True:
            return last_status
        time.sleep(0.2)
    raise SystemExit("Timed out waiting for AdvantageScope readiness: " + json.dumps(last_status, indent=2))


def resolve_command(args: argparse.Namespace) -> tuple[list[str], Path | None]:
    """Resolve the AdvantageScope executable command from CLI arguments."""
    fork = find_fork(args.fork)
    if args.fork and fork is None:
        raise SystemExit(f"--fork is not a Team 8044 AdvantageScope checkout: {Path(args.fork).expanduser()}")
    if args.advantagescope:
        return find_advantagescope(args.advantagescope)
    if fork is not None:
        command = command_from_fork(fork)
        if command is None:
            raise SystemExit(
                f"Found Team 8044 AdvantageScope fork at {fork}, but it is not built. "
                "Run npm ci, npm run compile, npm run wasm:compile, and electron-builder build --dir."
            )
        return command, fork
    return find_advantagescope(None)


def main() -> None:
    """Capture an AdvantageScope layout screenshot via the agent-control API."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--advantagescope", help="AdvantageScope executable path")
    parser.add_argument("--fork", help="Team 8044 AdvantageScope fork checkout")
    parser.add_argument("--log", required=True, help="WPILOG path")
    parser.add_argument("--layout", required=True, help="AdvantageScope layout JSON path")
    parser.add_argument("--out", required=True, help="Output PNG path")
    parser.add_argument("--time", type=float, help="Selected timestamp before capture")
    parser.add_argument("--timeline-range", nargs=2, type=float, metavar=("START", "END"), help="Visible timeline range before capture")
    parser.add_argument("--assets", help="Custom AdvantageScope assets folder, defaults to ./AScope_Assets when present")
    parser.add_argument("--headless", action="store_true", help="Keep AdvantageScope windows hidden during capture")
    args = parser.parse_args()

    command_prefix, command_cwd = resolve_command(args)
    assets_path = resolve_assets_path(args.assets)
    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    command = [
        *command_prefix,
        "--agent-control",
        "--log",
        str(Path(args.log).resolve()),
        "--layout",
        str(Path(args.layout).resolve()),
    ]
    if args.headless:
        command.append("--agent-headless")
    if assets_path is not None:
        command.extend(["--agent-assets", str(assets_path)])

    process = subprocess.Popen(
        command,
        cwd=command_cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    port: int | None = None
    token: str | None = None
    try:
        port, token = wait_for_control(process)
        ready_status = wait_for_ready(port, token)
        if args.timeline_range is not None:
            post(port, token, {"command": "setTimelineRange", "range": args.timeline_range, "lockMaxZoom": False})
            time.sleep(0.5)
        if args.time is not None:
            post(port, token, {"command": "setTime", "time": args.time})
            time.sleep(0.5)
        post(port, token, {"command": "capture", "path": str(out)})
        manifest = {
            "log": str(Path(args.log).resolve()),
            "layout": str(Path(args.layout).resolve()),
            "image": str(out),
            "selectedTime": args.time,
            "assets": str(assets_path) if assets_path is not None else None,
            "status": ready_status,
        }
        manifest_path = out.with_name(out.stem + "-manifest.json")
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"image={out}")
        print(f"manifest={manifest_path}")
    finally:
        if port is not None and token is not None:
            try:
                post(port, token, {"command": "close"})
            except Exception:
                pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()


if __name__ == "__main__":
    main()
