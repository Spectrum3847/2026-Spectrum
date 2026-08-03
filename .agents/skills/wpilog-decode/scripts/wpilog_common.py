#!/usr/bin/env python3
"""Shared WPILib discovery and version helpers for the wpilog-decode scripts."""

from __future__ import annotations

import re
from pathlib import Path


def _version_key(name: str) -> tuple[int, ...]:
    """Sort WPILib artifact versions, ranking beta < rc < stable."""
    key: list[int] = []
    for part in name.split("."):
        for token in re.split(r"[-_]", part):
            if token.isdigit():
                key.append(int(token))
            elif token == "alpha":
                key.append(-1)
            elif token == "beta":
                key.append(0)
            elif token == "rc":
                key.append(1)
            else:
                key.append(2)
    return tuple(key + [2] * (8 - len(key)))


def wpilib_root() -> Path:
    """Locate the local WPILib installation root directory."""
    roots = sorted((Path.home() / "wpilib").glob("*"), reverse=True)
    for root in roots:
        if (root / "jdk").is_dir() and (root / "maven").is_dir():
            return root
    raise SystemExit("Could not find WPILib install under ~/wpilib")


def wpilib_version(root: Path, group_path: str, artifact: str) -> str:
    """Read a WPILib Maven artifact version from the local repository."""
    base = root / "maven" / group_path / artifact
    versions = sorted([p.name for p in base.iterdir() if p.is_dir()], key=_version_key)
    if not versions:
        raise SystemExit(f"No versions found for {artifact} under {base}")
    return versions[-1]
