#!/usr/bin/env python3
"""Shared helpers for live read-only NT4 telemetry scripts."""

from __future__ import annotations

import os
import platform
import re
import subprocess
import tempfile
import zipfile
from pathlib import Path


def wpilib_root() -> Path:
    """Locate the local WPILib installation root directory."""
    candidates: list[Path] = []
    if os.environ.get("WPILIB_ROOT"):
        candidates.append(Path(os.environ["WPILIB_ROOT"]).expanduser())
    if os.environ.get("JAVA_HOME"):
        java_home = Path(os.environ["JAVA_HOME"]).expanduser()
        if java_home.name == "jdk":
            candidates.append(java_home.parent)
    candidates.extend(sorted((Path.home() / "wpilib").glob("*"), reverse=True))

    for candidate in candidates:
        if (candidate / "jdk/bin/java").exists() and (candidate / "maven").is_dir():
            return candidate.resolve()
    raise SystemExit(
        "Could not discover WPILib root. Set WPILIB_ROOT to the WPILib install directory "
        "or JAVA_HOME to its packaged jdk."
    )


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


def wpilib_version(root: Path, group_path: str, artifact: str) -> str:
    """Read a WPILib Maven artifact version from the local repository."""
    directory = root / "maven" / group_path / artifact
    versions = sorted((path.name for path in directory.glob("*") if path.is_dir()), key=_version_key)
    if not versions:
        raise SystemExit(f"Could not find WPILib artifact {artifact} under {directory}")
    return versions[-1]


def classpath(root: Path) -> str:
    """Build the JVM classpath from WPILib sim jars."""
    version = wpilib_version(root, "edu/wpi/first/ntcore", "ntcore-java")
    jars = [
        root / f"maven/edu/wpi/first/ntcore/ntcore-java/{version}/ntcore-java-{version}.jar",
        root / f"maven/edu/wpi/first/wpiutil/wpiutil-java/{version}/wpiutil-java-{version}.jar",
    ]
    missing = [str(jar) for jar in jars if not jar.exists()]
    if missing:
        raise SystemExit("Missing WPILib Java dependency jars:\n" + "\n".join(missing))
    return os.pathsep.join(str(jar) for jar in jars)


def extract_natives(root: Path, target: Path) -> Path:
    """Extract WPILib native libraries to a temporary directory."""
    version = wpilib_version(root, "edu/wpi/first/ntcore", "ntcore-cpp")
    system = platform.system().lower()
    if system == "darwin":
        classifier = "osxuniversal"
        native_dir = target / "osx/universal/shared"
    elif system == "windows":
        classifier = "windows"
        native_dir = target / "windows/x64/shared"
    else:
        classifier = "linux"
        native_dir = target / "linux/x64/shared"
    archives = [
        root / f"maven/edu/wpi/first/wpiutil/wpiutil-cpp/{version}/wpiutil-cpp-{version}-{classifier}.zip",
        root / f"maven/edu/wpi/first/wpinet/wpinet-cpp/{version}/wpinet-cpp-{version}-{classifier}.zip",
        root / f"maven/edu/wpi/first/ntcore/ntcore-cpp/{version}/ntcore-cpp-{version}-{classifier}.zip",
    ]
    for archive in archives:
        if not archive.exists():
            raise SystemExit(f"Missing WPILib native archive: {archive}")
        with zipfile.ZipFile(archive) as zip_file:
            zip_file.extractall(target)
    if not native_dir.is_dir():
        raise SystemExit(f"Could not find extracted WPILib native directory: {native_dir}")
    return native_dir


def compile_and_run(java_source: str, class_name: str, args: list[str]) -> int:
    """Compile the Java NT client source and execute it with the given arguments."""
    root = wpilib_root()
    with tempfile.TemporaryDirectory(prefix="live-nt-") as tmp_name:
        tmp = Path(tmp_name)
        source = tmp / f"{class_name}.java"
        classes = tmp / "classes"
        native_root = tmp / "native"
        classes.mkdir()
        native_root.mkdir()
        source.write_text(java_source)

        cp = classpath(root)
        native_dir = extract_natives(root, native_root)
        subprocess.run([str(root / "jdk/bin/javac"), "-cp", cp, "-d", str(classes), str(source)], check=True)
        run_cp = os.pathsep.join([str(classes), cp])
        command = [
            str(root / "jdk/bin/java"),
            f"-Djava.library.path={native_dir}",
            "-cp",
            run_cp,
            class_name,
            *args,
        ]
        return subprocess.run(command, check=False).returncode
