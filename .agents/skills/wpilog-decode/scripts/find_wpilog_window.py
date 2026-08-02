#!/usr/bin/env python3
"""Find the first WPILOG time window where requested boolean topics are true."""

from __future__ import annotations

import argparse
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
import java.util.*;

public class FindWpilogWindow {
    record Entry(String name, String type) {}

    public static void main(String[] args) throws Exception {
        String log = args[0];
        double duration = Double.parseDouble(args[1]);
        List<String> wanted = new ArrayList<>();
        for (int i = 2; i < args.length; i++) wanted.add(args[i]);
        if (wanted.isEmpty()) throw new IllegalArgumentException("At least one topic is required");

        Map<Integer, Entry> entries = new HashMap<>();
        Map<String, Boolean> values = new LinkedHashMap<>();
        for (String topic : wanted) values.put(topic, false);

        Double windowStart = null;
        Double windowEnd = null;
        Double firstTimestamp = null;
        Double latestTimestamp = null;

        Iterator<DataLogRecord> iterator = new DataLogReader(log).iterator();
        String readWarning = null;
        while (true) {
            DataLogRecord record;
            try {
                if (!iterator.hasNext()) break;
                record = iterator.next();
            } catch (RuntimeException ex) {
                readWarning = "stopped after partial read: " + ex.getClass().getSimpleName() + ": " + ex.getMessage();
                break;
            }
            double timestamp = record.getTimestamp() / 1_000_000.0;
            if (firstTimestamp == null) firstTimestamp = timestamp;
            latestTimestamp = timestamp;

            if (record.isStart()) {
                StartRecordData data = record.getStartData();
                entries.put(data.entry, new Entry(data.name, data.type));
                continue;
            }
            if (record.isControl()) continue;

            Entry entry = entries.get(record.getEntry());
            if (entry == null || !values.containsKey(entry.name)) continue;
            if (!entry.type.equals("boolean")) {
                throw new IllegalArgumentException(entry.name + " has type " + entry.type + ", expected boolean");
            }

            values.put(entry.name, record.getBoolean());
            boolean allMatch = true;
            for (boolean value : values.values()) allMatch &= value;

            if (allMatch && windowStart == null) {
                windowStart = timestamp;
            }
            if (windowStart != null && !allMatch && windowEnd == null) {
                windowEnd = timestamp;
            }
        }

        if (windowStart == null) {
            throw new IllegalStateException("No matching window found for " + wanted);
        }

        double exportEnd = windowStart + duration;
        System.out.printf("log=%s%n", log);
        System.out.printf("topics=%s%n", wanted);
        System.out.printf("logStart=%.6f%n", firstTimestamp == null ? 0.0 : firstTimestamp);
        System.out.printf("logEnd=%.6f%n", latestTimestamp == null ? 0.0 : latestTimestamp);
        System.out.printf("windowStart=%.6f%n", windowStart);
        System.out.printf("windowEnd=%s%n", windowEnd == null ? "-" : String.format("%.6f", windowEnd));
        System.out.printf("duration=%.6f%n", duration);
        System.out.printf("exportStart=%.6f%n", windowStart);
        System.out.printf("exportEnd=%.6f%n", exportEnd);
        if (latestTimestamp != null && exportEnd > latestTimestamp) {
            System.out.printf("warning=exportEnd exceeds logEnd by %.6fs%n", exportEnd - latestTimestamp);
        }
        if (readWarning != null) {
            System.out.printf("warning=%s%n", readWarning);
        }
    }
}
'''


def resolve_log(path: Path) -> Path:
    if path.suffix == ".wpilog":
        return path.resolve()
    directory = path / "SimLogs" if (path / "SimLogs").is_dir() else path
    logs = sorted(directory.glob("*.wpilog"), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise SystemExit(f"No .wpilog files found in {directory}")
    return logs[-1].resolve()


def run_with_java(repo: Path, log: Path, topics: list[str], duration: float) -> None:
    with tempfile.TemporaryDirectory(prefix="wpilog-decode-window-") as tmp:
        tmp_path = Path(tmp)
        source = tmp_path / "FindWpilogWindow.java"
        classes = tmp_path / "classes"
        classes.mkdir()
        source.write_text(JAVA_SOURCE)
        wpilib = wpilib_root()
        version = wpilib_version(wpilib, "edu/wpi/first/wpiutil", "wpiutil-java")
        java_home = wpilib / "jdk"
        wpiutil = wpilib / f"maven/edu/wpi/first/wpiutil/wpiutil-java/{version}/wpiutil-java-{version}.jar"
        subprocess.run([str(java_home / "bin/javac"), "-cp", str(wpiutil), "-d", str(classes), str(source)], cwd=repo, check=True)
        subprocess.run([str(java_home / "bin/java"), "-cp", f"{classes}:{wpiutil}", "FindWpilogWindow", str(log), str(duration), *topics], cwd=repo, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--log", default="SimLogs", help="WPILOG file, SimLogs directory, or repo-relative path")
    parser.add_argument("--all-true", action="append", required=True, help="Boolean topic that must be true")
    parser.add_argument("--duration", type=float, required=True, help="Export duration in seconds")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    raw_log = Path(args.log)
    log = resolve_log(raw_log if raw_log.is_absolute() else repo / raw_log)
    run_with_java(repo, log, args.all_true, args.duration)


if __name__ == "__main__":
    main()
