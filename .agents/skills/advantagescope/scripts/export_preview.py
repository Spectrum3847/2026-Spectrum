#!/usr/bin/env python3
"""Export an AdvantageScope MP4 preview with the Team 8044 agent fork."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path


FORK_REMOTE_MARKER = "Team8044/AdvantageScope"


def find_advantagescope(explicit: str | None) -> tuple[list[str], Path | None]:
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if path.exists():
            return [str(path)], None
        raise SystemExit(f"AdvantageScope executable not found: {path}")

    env_executable = os.environ.get("ADVANTAGESCOPE_EXECUTABLE")
    if env_executable:
        return find_advantagescope(env_executable)

    fork = find_fork(None)
    if fork is not None:
        command = command_from_fork(fork)
        if command is not None:
            return command, fork

    path_executable = shutil.which("AdvantageScope")
    if path_executable:
        return [path_executable], None

    raise SystemExit(
        "Could not find the Team 8044 AdvantageScope fork.\n"
        "Pass --fork <checkout>, set ADVANTAGESCOPE_FORK, pass --advantagescope <executable>, "
        "or clone/build https://github.com/Team8044/AdvantageScope."
    )


def find_fork(explicit: str | None) -> Path | None:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    if os.environ.get("ADVANTAGESCOPE_FORK"):
        candidates.append(Path(os.environ["ADVANTAGESCOPE_FORK"]).expanduser())

    cwd = Path.cwd().resolve()
    for parent in [cwd, *cwd.parents]:
        candidates.append(parent / "AdvantageScope")
        for child in parent.glob("*AdvantageScope*"):
            candidates.append(child)

    github_root = Path.home() / "Documents" / "GitHub"
    if github_root.exists():
        candidates.extend(github_root.glob("*/AdvantageScope"))
        candidates.extend(github_root.glob("*/*AdvantageScope*"))

    seen: set[Path] = set()
    for candidate in candidates:
        path = candidate.resolve()
        if path in seen or not (path / ".git").is_dir():
            continue
        seen.add(path)
        if is_team8044_fork(path):
            return path
    return None


def is_team8044_fork(path: Path) -> bool:
    try:
        result = subprocess.run(
            ["git", "remote", "-v"],
            cwd=path,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
        )
    except subprocess.CalledProcessError:
        return False
    return FORK_REMOTE_MARKER in result.stdout


def command_from_fork(fork: Path) -> list[str] | None:
    electron = fork / "node_modules/.bin/electron"
    main = fork / "bundles/main.js"
    if electron.exists() and main.exists():
        return [str(electron), str(main)]

    packaged = packaged_executable(fork)
    if packaged.exists():
        return [str(packaged)]
    return None


def packaged_executable(fork: Path) -> Path:
    system = platform.system()
    machine = platform.machine().lower()
    if system == "Darwin":
        arch = "arm64" if machine in ("arm64", "aarch64") else "x64"
        return fork / f"dist/mac-{arch}/AdvantageScope.app/Contents/MacOS/AdvantageScope"
    if system == "Windows":
        return fork / "dist/win-unpacked/AdvantageScope.exe"
    return fork / "dist/linux-unpacked/advantagescope"


def resolve_assets_path(raw: str | None) -> Path | None:
    if raw is not None:
        path = Path(raw).expanduser().resolve()
        if not path.is_dir():
            raise SystemExit(f"AdvantageScope assets folder not found: {path}")
        return path

    repo_assets = Path.cwd().resolve() / "AScope_Assets"
    if repo_assets.is_dir():
        return repo_assets
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--advantagescope", help="AdvantageScope executable path")
    parser.add_argument("--fork", help="Team 8044 AdvantageScope fork checkout")
    parser.add_argument("--log", required=True, help="WPILOG path")
    parser.add_argument("--layout", required=True, help="AdvantageScope layout JSON path")
    parser.add_argument("--out", required=True, help="Output directory")
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--mode", choices=["realtime", "frames", "multiview"], default="realtime")
    parser.add_argument("--headless", action="store_true", help="Keep AdvantageScope windows hidden during agent export")
    parser.add_argument("--assets", help="Custom AdvantageScope assets folder, defaults to ./AScope_Assets when present")
    args = parser.parse_args()

    fork = find_fork(args.fork)
    if args.fork and fork is None:
        raise SystemExit(f"--fork is not a Team 8044 AdvantageScope checkout: {Path(args.fork).expanduser()}")
    if args.advantagescope:
        command_prefix, command_cwd = find_advantagescope(args.advantagescope)
    elif fork is not None:
        command_prefix = command_from_fork(fork)
        command_cwd = fork
        if command_prefix is None:
            raise SystemExit(
                f"Found Team 8044 AdvantageScope fork at {fork}, but it is not built. "
                "Run npm ci, npm run compile, npm run wasm:compile, and electron-builder build --dir."
            )
    else:
        command_prefix, command_cwd = find_advantagescope(None)
    assets_path = resolve_assets_path(args.assets)
    command = [
        *command_prefix,
        "--agent-export",
        "--log",
        str(Path(args.log).resolve()),
        "--layout",
        str(Path(args.layout).resolve()),
        "--out",
        str(Path(args.out).resolve()),
        "--start",
        str(args.start),
        "--end",
        str(args.end),
        "--fps",
        str(args.fps),
        "--agent-export-mode",
        args.mode,
    ]
    if args.headless:
        command.append("--agent-headless")
    if assets_path is not None:
        command.extend(["--agent-assets", str(assets_path)])
    subprocess.run(command, cwd=command_cwd, check=True)

    manifest = Path(args.out).resolve() / "manifest.json"
    if not manifest.exists():
        raise SystemExit(f"Export completed but manifest was not found: {manifest}")
    data = json.loads(manifest.read_text())
    ready = data.get("status", {}).get("rendererState", {}).get("liveStatus", {}).get("ready")
    missing_sources = data.get("status", {}).get("rendererState", {}).get("liveStatus", {}).get("missingSources", [])
    missing_assets = data.get("status", {}).get("rendererState", {}).get("liveStatus", {}).get("missingAssets", [])
    video = data.get("video")
    webm = data.get("webm")
    mode = data.get("mode")
    print(f"manifest={manifest}")
    print(f"mode={mode}")
    print(f"webm={webm}")
    print(f"video={video}")
    print(f"ready={ready}")
    print(f"missingSources={missing_sources}")
    print(f"missingAssets={missing_assets}")
    if assets_path is not None:
        print(f"assets={assets_path}")


if __name__ == "__main__":
    main()
