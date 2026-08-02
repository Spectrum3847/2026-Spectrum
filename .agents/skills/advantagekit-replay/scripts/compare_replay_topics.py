#!/usr/bin/env python3
"""Compare real and replay WPILOG topic presence for replayable outputs."""

from __future__ import annotations

import argparse
import subprocess
import tempfile
from pathlib import Path


DEFAULT_TOPICS = [
    "Swerve/TeleopController/vxMPS",
    "Swerve/TeleopController/vyMPS",
    "Swerve/TeleopController/omegaRPS",
    "Swerve/TeleopController/Drive Speed",
    "Swerve/SwerveStates/Rawgyro",
    "Swerve/Actual_Velocity",
    "Vision/CameraPose",
    "Visualizer/Components",
    "SuperCalc/LookaheadPose",
    "SuperCalc/TurretToTargetDistance",
    "SuperCalc/TurretAngle",
    "SuperCalc/distance",
    "Shooter/hoodMotor/PositionSetpoint",
    "Turret/RotationDemand",
    "Turret/VelocityDemand",
    "Intake/Roller Voltage Demand",
    "Intake/Roller Velocity Demand",
    "Intake/Extension Voltage Demand",
    "Intake/Extension Position Demand",
    "Intake/Extension Position Demand Clamped",
]


JAVA_SOURCE = r'''
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.util.*;

public class ListTopicNames {
    public static void main(String[] args) throws Exception {
        DataLogReader reader = new DataLogReader(args[0]);
        Set<String> names = new TreeSet<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!iterator.hasNext()) break;
                record = iterator.next();
            } catch (RuntimeException ex) {
                break;
            }
            try {
                if (record.isStart()) {
                    names.add(record.getStartData().name);
                }
            } catch (RuntimeException ex) {
                break;
            }
        }
        for (String name : names) {
            System.out.println(name);
        }
    }
}
'''


def wpilib_root() -> Path:
    roots = sorted((Path.home() / "wpilib").glob("20*"), reverse=True)
    if not roots:
        raise SystemExit("No WPILib install found under ~/wpilib")
    return roots[0]


def wpilib_version(root: Path) -> str:
    base = root / "maven/edu/wpi/first/wpiutil/wpiutil-java"
    versions = sorted((p for p in base.iterdir() if p.is_dir()), key=lambda p: p.name)
    if not versions:
        raise SystemExit(f"No wpiutil-java versions found in {base}")
    return versions[-1].name


def topic_names(repo: Path, log: Path) -> set[str]:
    with tempfile.TemporaryDirectory(prefix="akit-replay-topics-") as tmp:
        tmp_path = Path(tmp)
        source = tmp_path / "ListTopicNames.java"
        classes = tmp_path / "classes"
        classes.mkdir()
        source.write_text(JAVA_SOURCE)
        root = wpilib_root()
        version = wpilib_version(root)
        java_home = root / "jdk"
        wpiutil = root / f"maven/edu/wpi/first/wpiutil/wpiutil-java/{version}/wpiutil-java-{version}.jar"
        subprocess.run([str(java_home / "bin/javac"), "-cp", str(wpiutil), "-d", str(classes), str(source)], cwd=repo, check=True)
        result = subprocess.run(
            [str(java_home / "bin/java"), "-cp", f"{classes}:{wpiutil}", "ListTopicNames", str(log)],
            cwd=repo,
            check=True,
            text=True,
            capture_output=True,
        )
        return set(result.stdout.splitlines())


def resolve(path: str, repo: Path) -> Path:
    output = Path(path)
    if not output.is_absolute():
        output = repo / output
    if not output.is_file():
        raise SystemExit(f"Log not found: {output}")
    return output.resolve()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--real", required=True, help="Real robot WPILOG")
    parser.add_argument("--replay", required=True, help="Replay WPILOG")
    parser.add_argument("--topic", action="append", help="Replayable topic to check; defaults to known gated outputs")
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    real_topics = topic_names(repo, resolve(args.real, repo))
    replay_topics = topic_names(repo, resolve(args.replay, repo))
    expected = [topic.removeprefix("/RealOutputs/").removeprefix("/ReplayOutputs/").removeprefix("/") for topic in (args.topic or DEFAULT_TOPICS)]

    present_in_real = ["/RealOutputs/" + topic for topic in expected if "/RealOutputs/" + topic in real_topics]
    missing_in_replay = ["/ReplayOutputs/" + topic for topic in expected if "/ReplayOutputs/" + topic not in replay_topics]

    print(f"checked={len(expected)}")
    print(f"presentInReal={len(present_in_real)}")
    for topic in present_in_real:
        print(f"real_has\t{topic}")
    print(f"missingInReplay={len(missing_in_replay)}")
    for topic in missing_in_replay:
        print(f"replay_missing\t{topic}")

    if present_in_real or missing_in_replay:
        raise SystemExit(1)
    print("status=ok")


if __name__ == "__main__":
    main()
