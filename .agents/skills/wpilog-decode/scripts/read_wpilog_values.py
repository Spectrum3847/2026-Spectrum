#!/usr/bin/env python3
"""Summarize selected WPILOG topics with common WPILib struct decoding."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from wpilog_common import wpilib_root, wpilib_version, resolve_log


JAVA_SOURCE = r'''
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogRecord.StartRecordData;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

public class ReadWpilogValues {
    record Entry(String name, String type) {}
    record Sample(double timestamp, String value) {}
    record StructSpec(String name, int size, Decoder decoder) {}
    interface Decoder { String decode(ByteBuffer buffer); }

    static class Summary {
        String type = "";
        long count = 0;
        String first = null;
        String latest = null;
        double firstTimestamp = 0.0;
        double latestTimestamp = 0.0;
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        boolean numeric = false;
        ArrayList<Sample> samples = new ArrayList<>();
    }

    static final Map<String, StructSpec> STRUCTS = buildStructs();

    public static void main(String[] args) throws Exception {
        if (args.length < 3) throw new IllegalArgumentException("Usage: <log> <json:true|false> <samples> <topics...>");
        String log = args[0];
        boolean json = Boolean.parseBoolean(args[1]);
        int sampleCount = Integer.parseInt(args[2]);
        Set<String> wanted = new LinkedHashSet<>();
        for (int i = 3; i < args.length; i++) wanted.add(args[i]);
        Map<Integer, Entry> entries = new HashMap<>();
        Map<String, Summary> summaries = new LinkedHashMap<>();
        for (String topic : wanted) summaries.put(topic, new Summary());

        DataLogReader reader = new DataLogReader(log);
        Iterator<DataLogRecord> iterator = reader.iterator();
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
            if (record.isStart()) {
                StartRecordData data = record.getStartData();
                entries.put(data.entry, new Entry(data.name, data.type));
                Summary summary = summaries.get(data.name);
                if (summary != null) summary.type = data.type;
                continue;
            }
            if (record.isControl()) continue;
            Entry entry = entries.get(record.getEntry());
            if (entry == null || !summaries.containsKey(entry.name)) continue;
            Summary summary = summaries.get(entry.name);
            summary.count++;
            double timestamp = record.getTimestamp() / 1_000_000.0;
            String value = valueString(record, entry.type, summary);
            if (summary.first == null) {
                summary.first = value;
                summary.firstTimestamp = timestamp;
            }
            summary.latest = value;
            summary.latestTimestamp = timestamp;
            if (sampleCount > 0) summary.samples.add(new Sample(timestamp, value));
        }

        if (json) {
            printJson(log, summaries, readWarning, sampleCount);
        } else {
            printText(summaries, readWarning, sampleCount);
        }
    }

    static Map<String, StructSpec> buildStructs() {
        Map<String, StructSpec> specs = new HashMap<>();
        specs.put("Rotation2d", new StructSpec("Rotation2d", 8, b -> String.format("Rotation2d(rad=%.6f,deg=%.3f)", d(b), Math.toDegrees(peek(b, -8)))));
        specs.put("Translation2d", new StructSpec("Translation2d", 16, b -> String.format("Translation2d(x=%.3f,y=%.3f)", d(b), d(b))));
        specs.put("Pose2d", new StructSpec("Pose2d", 24, b -> String.format("Pose2d(x=%.3f,y=%.3f,thetaRad=%.3f,thetaDeg=%.3f)", d(b), d(b), d(b), Math.toDegrees(peek(b, -8)))));
        specs.put("Transform2d", new StructSpec("Transform2d", 24, b -> String.format("Transform2d(dx=%.3f,dy=%.3f,thetaRad=%.3f,thetaDeg=%.3f)", d(b), d(b), d(b), Math.toDegrees(peek(b, -8)))));
        specs.put("Twist2d", new StructSpec("Twist2d", 24, b -> String.format("Twist2d(dx=%.3f,dy=%.3f,dtheta=%.6f)", d(b), d(b), d(b))));
        specs.put("Translation3d", new StructSpec("Translation3d", 24, b -> String.format("Translation3d(x=%.3f,y=%.3f,z=%.3f)", d(b), d(b), d(b))));
        specs.put("Rotation3d", new StructSpec("Rotation3d", 32, b -> String.format("Rotation3d(qw=%.6f,qx=%.6f,qy=%.6f,qz=%.6f)", d(b), d(b), d(b), d(b))));
        specs.put("Pose3d", new StructSpec("Pose3d", 56, b -> String.format("Pose3d(x=%.3f,y=%.3f,z=%.3f,qw=%.6f,qx=%.6f,qy=%.6f,qz=%.6f)", d(b), d(b), d(b), d(b), d(b), d(b), d(b))));
        specs.put("Transform3d", new StructSpec("Transform3d", 56, b -> String.format("Transform3d(dx=%.3f,dy=%.3f,dz=%.3f,qw=%.6f,qx=%.6f,qy=%.6f,qz=%.6f)", d(b), d(b), d(b), d(b), d(b), d(b), d(b))));
        specs.put("Twist3d", new StructSpec("Twist3d", 48, b -> String.format("Twist3d(dx=%.3f,dy=%.3f,dz=%.3f,rx=%.6f,ry=%.6f,rz=%.6f)", d(b), d(b), d(b), d(b), d(b), d(b))));
        specs.put("ChassisSpeeds", new StructSpec("ChassisSpeeds", 24, b -> String.format("ChassisSpeeds(vx=%.3f,vy=%.3f,omega=%.3f)", d(b), d(b), d(b))));
        specs.put("SwerveModuleState", new StructSpec("SwerveModuleState", 16, b -> String.format("SwerveModuleState(speed=%.3f,angleRad=%.6f,angleDeg=%.3f)", d(b), d(b), Math.toDegrees(peek(b, -8)))));
        specs.put("SwerveModulePosition", new StructSpec("SwerveModulePosition", 16, b -> String.format("SwerveModulePosition(distance=%.3f,angleRad=%.6f,angleDeg=%.3f)", d(b), d(b), Math.toDegrees(peek(b, -8)))));
        return specs;
    }

    static double d(ByteBuffer buffer) {
        return buffer.getDouble();
    }

    static double peek(ByteBuffer buffer, int relativeOffset) {
        return buffer.getDouble(buffer.position() + relativeOffset);
    }

    static String valueString(DataLogRecord record, String type, Summary summary) {
        try {
            return switch (type) {
                case "boolean" -> Boolean.toString(record.getBoolean());
                case "int64" -> {
                    long value = record.getInteger();
                    summary.numeric = true;
                    summary.min = Math.min(summary.min, value);
                    summary.max = Math.max(summary.max, value);
                    yield Long.toString(value);
                }
                case "float" -> {
                    float value = record.getFloat();
                    summary.numeric = true;
                    summary.min = Math.min(summary.min, value);
                    summary.max = Math.max(summary.max, value);
                    yield Float.toString(value);
                }
                case "double" -> {
                    double value = record.getDouble();
                    summary.numeric = true;
                    summary.min = Math.min(summary.min, value);
                    summary.max = Math.max(summary.max, value);
                    yield Double.toString(value);
                }
                case "string" -> record.getString();
                case "boolean[]" -> Arrays.toString(record.getBooleanArray());
                case "int64[]" -> Arrays.toString(record.getIntegerArray());
                case "float[]" -> Arrays.toString(record.getFloatArray());
                case "double[]" -> Arrays.toString(record.getDoubleArray());
                case "string[]" -> Arrays.toString(record.getStringArray());
                default -> decodeStruct(record, type);
            };
        } catch (RuntimeException ex) {
            return "unreadable(" + ex.getClass().getSimpleName() + ")";
        }
    }

    static String decodeStruct(DataLogRecord record, String type) {
        if (!type.startsWith("struct:")) return "raw[" + record.getSize() + " bytes]";
        String structName = type.substring("struct:".length());
        boolean declaredArray = structName.endsWith("[]");
        if (declaredArray) structName = structName.substring(0, structName.length() - 2);
        int dot = structName.lastIndexOf('.');
        if (dot >= 0) structName = structName.substring(dot + 1);
        StructSpec spec = STRUCTS.get(structName);
        if (spec == null) return "raw[" + record.getSize() + " bytes," + type + "]";
        byte[] raw = record.getRaw();
        if (raw.length < spec.size || raw.length % spec.size != 0) {
            return "raw[" + raw.length + " bytes," + type + "]";
        }
        ByteBuffer buffer = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
        int count = raw.length / spec.size;
        if (!declaredArray && count == 1) {
            return spec.decoder.decode(buffer);
        }
        ArrayList<String> values = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            ByteBuffer slice = ByteBuffer.wrap(raw, i * spec.size, spec.size).order(ByteOrder.LITTLE_ENDIAN);
            values.add(spec.decoder.decode(slice));
        }
        return values.toString();
    }

    static void printText(Map<String, Summary> summaries, String readWarning, int sampleCount) {
        for (Map.Entry<String, Summary> entry : summaries.entrySet()) {
            Summary s = entry.getValue();
            System.out.print(entry.getKey() + "\ttype=" + emptyDash(s.type) + "\tcount=" + s.count);
            System.out.print("\tfirst=" + emptyDash(s.first) + "\tlatest=" + emptyDash(s.latest));
            if (s.numeric) System.out.printf("\tmin=%.6f\tmax=%.6f", s.min, s.max);
            System.out.println();
            for (Sample sample : selectedSamples(s.samples, sampleCount)) {
                System.out.printf("  sample t=%.6f %s%n", sample.timestamp, sample.value);
            }
        }
        if (readWarning != null) System.out.println("warning=" + readWarning);
    }

    static void printJson(String log, Map<String, Summary> summaries, String readWarning, int sampleCount) {
        StringBuilder out = new StringBuilder();
        out.append("{\"log\":\"").append(escape(log)).append("\",\"topics\":{");
        boolean firstTopic = true;
        for (Map.Entry<String, Summary> entry : summaries.entrySet()) {
            if (!firstTopic) out.append(",");
            firstTopic = false;
            Summary s = entry.getValue();
            out.append("\"").append(escape(entry.getKey())).append("\":{");
            out.append("\"type\":\"").append(escape(s.type)).append("\",");
            out.append("\"count\":").append(s.count).append(",");
            out.append("\"first\":\"").append(escape(emptyDash(s.first))).append("\",");
            out.append("\"latest\":\"").append(escape(emptyDash(s.latest))).append("\"");
            if (s.numeric) {
                out.append(",\"min\":").append(s.min).append(",\"max\":").append(s.max);
            }
            List<Sample> selected = selectedSamples(s.samples, sampleCount);
            if (!selected.isEmpty()) {
                out.append(",\"samples\":[");
                for (int i = 0; i < selected.size(); i++) {
                    if (i > 0) out.append(",");
                    Sample sample = selected.get(i);
                    out.append("{\"timestamp\":").append(sample.timestamp).append(",\"value\":\"").append(escape(sample.value)).append("\"}");
                }
                out.append("]");
            }
            out.append("}");
        }
        out.append("}");
        if (readWarning != null) out.append(",\"warning\":\"").append(escape(readWarning)).append("\"");
        out.append("}");
        System.out.println(out);
    }

    static List<Sample> selectedSamples(List<Sample> samples, int sampleCount) {
        if (sampleCount <= 0 || samples.isEmpty()) return Collections.emptyList();
        if (samples.size() <= sampleCount) return samples;
        ArrayList<Sample> selected = new ArrayList<>();
        for (int i = 0; i < sampleCount; i++) {
            int index = (int)Math.round(i * (samples.size() - 1.0) / (sampleCount - 1.0));
            selected.add(samples.get(index));
        }
        return selected;
    }

    static String emptyDash(String value) {
        return value == null || value.isEmpty() ? "-" : value;
    }

    static String escape(String value) {
        return value.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n").replace("\r", "\\r");
    }
}
'''


def run_with_java(repo: Path, log: Path, topics: list[str], json_output: bool, sample_count: int) -> None:
    """Execute the Java WPILog value-reading tool with the given classpath."""
    with tempfile.TemporaryDirectory(prefix="wpilog-decode-values-") as tmp:
        tmp_path = Path(tmp)
        source = tmp_path / "ReadWpilogValues.java"
        classes = tmp_path / "classes"
        classes.mkdir()
        source.write_text(JAVA_SOURCE)
        wpilib = wpilib_root()
        version = wpilib_version(wpilib, "edu/wpi/first/wpiutil", "wpiutil-java")
        java_home = wpilib / "jdk"
        wpiutil = wpilib / f"maven/edu/wpi/first/wpiutil/wpiutil-java/{version}/wpiutil-java-{version}.jar"
        subprocess.run([str(java_home / "bin/javac"), "-cp", str(wpiutil), "-d", str(classes), str(source)], cwd=repo, check=True)
        subprocess.run(
            [
                str(java_home / "bin/java"),
                "-cp",
                os.pathsep.join([str(classes), str(wpiutil)]),
                "ReadWpilogValues",
                str(log),
                str(json_output).lower(),
                str(sample_count),
                *topics,
            ],
            cwd=repo,
            check=True,
        )


def main() -> None:
    """Read and print values from one or more WPILOG topics."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--log", default="SimLogs", help="WPILOG file, SimLogs directory, or repo-relative path")
    parser.add_argument("--topic", action="append", required=True, help="Topic to summarize; repeatable")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON")
    parser.add_argument("--samples", type=int, default=0, help="Print this many evenly spaced decoded samples per topic")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    raw_log = Path(args.log)
    log = resolve_log(raw_log if raw_log.is_absolute() else repo / raw_log)
    run_with_java(repo, log, args.topic, args.json, args.samples)


if __name__ == "__main__":
    main()
