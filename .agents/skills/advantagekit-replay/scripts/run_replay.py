#!/usr/bin/env python3
"""Run Team 8044 AdvantageKit replay for a selected WPILOG."""

from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path


def replay_path(log: Path) -> Path:
    if log.stem.endswith("_REPLAY"):
        return log.with_name(log.stem + "_2" + log.suffix)
    return log.with_name(log.stem + "_REPLAY" + log.suffix)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--log", required=True, help="WPILOG to replay")
    parser.add_argument("--java-home", default=None, help="WPILib JDK path")
    parser.add_argument("--timeout", type=float, default=120.0, help="Replay timeout seconds")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    log = Path(args.log)
    if not log.is_absolute():
        log = (repo / log).resolve()
    if not log.is_file():
        raise SystemExit(f"Replay log not found: {log}")

    env = os.environ.copy()
    env["AKIT_LOG_PATH"] = str(log)
    java_tool_options = env.get("JAVA_TOOL_OPTIONS", "")
    replay_option = "-DrobotMode=REPLAY"
    if replay_option not in java_tool_options:
        env["JAVA_TOOL_OPTIONS"] = (java_tool_options + " " + replay_option).strip()
    if "-Djava.awt.headless=true" not in env["JAVA_TOOL_OPTIONS"]:
        env["JAVA_TOOL_OPTIONS"] = (env["JAVA_TOOL_OPTIONS"] + " -Djava.awt.headless=true").strip()
    if args.java_home:
        env["JAVA_HOME"] = args.java_home
    elif "JAVA_HOME" not in env:
        candidate = Path.home() / "wpilib/2026/jdk"
        if candidate.is_dir():
            env["JAVA_HOME"] = str(candidate)

    init_script = repo / ".agents/skills/wpilib-sim/references/headless-sim.gradle"
    command = ["./gradlew"]
    if init_script.is_file():
        command.extend(["--init-script", str(init_script)])
    command.append("simulateJava")

    subprocess.run(
        command,
        cwd=repo,
        env=env,
        check=True,
        timeout=args.timeout,
    )
    print(f"replayLog={replay_path(log)}")


if __name__ == "__main__":
    main()
