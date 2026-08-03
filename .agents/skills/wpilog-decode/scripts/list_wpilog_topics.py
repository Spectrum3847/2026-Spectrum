#!/usr/bin/env python3
"""List topic names and types from a WPILOG using WPILib DataLogReader."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from wpilog_common import wpilib_root, wpilib_version


JAVA_SOURCE = r'''
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;

public class ListWpilogTopics {
    public static void main(String[] args) throws Exception {
        String log = args[0];
        String filter = args.length > 1 ? args[1] : "";
        DataLogReader reader = new DataLogReader(log);
        for (DataLogRecord record : reader) {
            if (!record.isStart()) continue;
            StartRecordData data = record.getStartData();
            if (filter.isEmpty() || data.name.contains(filter) || data.type.contains(filter)) {
                System.out.println(data.name + "\t" + data.type);
            }
        }
    }
}
'''


def resolve_log(path: Path) -> Path:
    """Resolve the WPILOG file path from a CLI arg or auto-detect the latest."""
    if path.suffix == ".wpilog":
        return path.resolve()
    directory = path / "SimLogs" if (path / "SimLogs").is_dir() else path
    logs = sorted(directory.glob("*.wpilog"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit(f"No .wpilog files found in {directory}")
    return logs[-1].resolve()


def run_with_java(repo: Path, log: Path, filter_text: str) -> None:
    """Execute the Java topic-listing tool with the given classpath."""
    with tempfile.TemporaryDirectory(prefix="wpilog-decode-topics-") as tmp:
        tmp_path = Path(tmp)
        source = tmp_path / "ListWpilogTopics.java"
        classes = tmp_path / "classes"
        classes.mkdir()
        source.write_text(JAVA_SOURCE)
        wpilib = wpilib_root()
        version = wpilib_version(wpilib, "edu/wpi/first/wpiutil", "wpiutil-java")
        java_home = wpilib / "jdk"
        wpiutil = wpilib / f"maven/edu/wpi/first/wpiutil/wpiutil-java/{version}/wpiutil-java-{version}.jar"
        subprocess.run([str(java_home / "bin/javac"), "-cp", str(wpiutil), "-d", str(classes), str(source)], cwd=repo, check=True)
        subprocess.run([str(java_home / "bin/java"), "-cp", os.pathsep.join([str(classes), str(wpiutil)]), "ListWpilogTopics", str(log), filter_text], cwd=repo, check=True)


def main() -> None:
    """List all topics contained in a WPILOG file."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--log", default="SimLogs", help="WPILOG file, SimLogs directory, or repo-relative path")
    parser.add_argument("--filter", default="", help="Substring filter for name/type")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    raw_log = Path(args.log)
    log = resolve_log(raw_log if raw_log.is_absolute() else repo / raw_log)
    run_with_java(repo, log, args.filter)


if __name__ == "__main__":
    main()
