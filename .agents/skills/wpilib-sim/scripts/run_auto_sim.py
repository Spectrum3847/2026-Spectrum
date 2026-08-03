#!/usr/bin/env python3
"""Run a headless WPILib sim auto through NT Auto Chooser and SimAgentBridge."""

from __future__ import annotations

import argparse
import os
import platform
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
import zipfile
from pathlib import Path


JAVA_SOURCE = r'''
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;

public class RunAutoNtClient {
    public static void main(String[] args) throws Exception {
        String host = args[0];
        String autoName = args[1];
        String alliance = args[2];
        double duration = Double.parseDouble(args[3]);
        double buffer = Double.parseDouble(args[4]);
        double chooserTimeout = Double.parseDouble(args[5]);
        double connectionTimeout = Double.parseDouble(args[6]);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("wpilib-sim-agent-auto");
        inst.setServer(host, 5810);

        long connectionDeadline = System.nanoTime() + secondsToNanos(connectionTimeout);
        while (!inst.isConnected() && System.nanoTime() < connectionDeadline) {
            Thread.sleep(50);
        }
        if (!inst.isConnected()) {
            throw new RuntimeException("Timed out waiting for NT4 connection to " + host + ":5810");
        }

        NetworkTable sim = inst.getTable("SimAgent");
        NetworkTable ds = sim.getSubTable("DriverStation");
        NetworkTable control = sim.getSubTable("Control");
        NetworkTable chooser = inst.getTable("SmartDashboard").getSubTable("Auto Chooser");

        ds.getEntry("AllianceStation").setString(alliance);
        ds.getEntry("DsAttached").setBoolean(true);
        ds.getEntry("FmsAttached").setBoolean(true);
        ds.getEntry("Enabled").setBoolean(false);
        ds.getEntry("Autonomous").setBoolean(true);
        ds.getEntry("Test").setBoolean(false);
        control.getEntry("Exit").setBoolean(false);
        control.getEntry("ExitAfterSeconds").setDouble(duration + buffer + 8.0);
        inst.flush();

        long chooserDeadline = System.nanoTime() + secondsToNanos(chooserTimeout);
        boolean found = false;
        String[] latestOptions = new String[] {};
        while (System.nanoTime() < chooserDeadline) {
            latestOptions = chooser.getEntry("options").getStringArray(new String[] {});
            for (String option : latestOptions) {
                if (option.equals(autoName)) {
                    found = true;
                    break;
                }
            }
            if (found) break;
            Thread.sleep(50);
        }
        if (!found) {
            throw new RuntimeException(
                "Auto '" + autoName + "' was not present in /SmartDashboard/Auto Chooser/options. "
                    + "Latest options: " + Arrays.toString(latestOptions)
            );
        }

        chooser.getEntry("selected").setString(autoName);
        inst.flush();

        long activeDeadline = System.nanoTime() + secondsToNanos(5.0);
        String active = "";
        while (System.nanoTime() < activeDeadline) {
            active = chooser.getEntry("active").getString("");
            if (active.equals(autoName)) break;
            Thread.sleep(50);
        }
        if (!active.equals(autoName)) {
            throw new RuntimeException(
                "Auto Chooser selected '" + autoName + "' but active auto stayed '" + active + "'."
            );
        }

        ds.getEntry("Enabled").setBoolean(true);
        ds.getEntry("Autonomous").setBoolean(true);
        ds.getEntry("Test").setBoolean(false);
        inst.flush();

        Thread.sleep((long) ((duration + buffer) * 1000.0));

        ds.getEntry("Enabled").setBoolean(false);
        control.getEntry("Exit").setBoolean(true);
        inst.flush();
        Thread.sleep(500);
        inst.stopClient();

        System.out.println("selectedAuto=" + autoName);
        System.out.println("alliance=" + alliance);
        System.out.printf("duration=%.3f%n", duration);
    }

    private static long secondsToNanos(double seconds) {
        return (long) (seconds * 1_000_000_000.0);
    }
}
'''


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
    """Build a sort key that orders numeric version components numerically."""
    parts = re.split(r"[.\-+]", name)
    return tuple((0, int(part)) if part.isdigit() else (1, part) for part in parts)


def wpilib_version(root: Path, group_path: str, artifact: str) -> str:
    """Read a WPILib Maven artifact version from the local repository."""
    directory = root / "maven" / group_path / artifact
    versions = sorted(
        (path.name for path in directory.glob("*") if path.is_dir()),
        key=_version_key,
        reverse=True,
    )
    if not versions:
        raise SystemExit(f"Could not find WPILib artifact {artifact} under {directory}")
    return versions[0]


def classpath(root: Path) -> str:
    """Build the JVM classpath from WPILib sim jars and user code."""
    version = wpilib_version(root, "edu/wpi/first/ntcore", "ntcore-java")
    jackson = wpilib_version(root, "com/fasterxml/jackson/core", "jackson-core")
    jars = [
        root / f"maven/edu/wpi/first/ntcore/ntcore-java/{version}/ntcore-java-{version}.jar",
        root / f"maven/edu/wpi/first/wpiutil/wpiutil-java/{version}/wpiutil-java-{version}.jar",
        root / f"maven/com/fasterxml/jackson/core/jackson-core/{jackson}/jackson-core-{jackson}.jar",
        root / f"maven/com/fasterxml/jackson/core/jackson-databind/{jackson}/jackson-databind-{jackson}.jar",
        root / f"maven/com/fasterxml/jackson/core/jackson-annotations/{jackson}/jackson-annotations-{jackson}.jar",
    ]
    missing = [str(jar) for jar in jars if not jar.exists()]
    if missing:
        raise SystemExit("Missing WPILib Java dependency jars:\n" + "\n".join(missing))
    return os.pathsep.join(str(jar) for jar in jars)


def native_platform() -> tuple[str, str]:
    """Return the platform identifier for WPILib native library extraction."""
    system = platform.system()
    machine = platform.machine().lower()
    if system == "Darwin":
        return "osxuniversal", "osx/universal/shared"
    if system == "Linux":
        if machine in ("aarch64", "arm64"):
            return "linuxarm64", "linux/arm64/shared"
        return "linuxx86-64", "linux/x86-64/shared"
    if system == "Windows":
        return "windowsx86-64", "windows/x86-64/shared"
    raise SystemExit(f"Unsupported WPILib native platform: {system} {machine}")


def extract_natives(root: Path, target: Path) -> Path:
    """Extract WPILib native libraries to a temporary directory for the current platform."""
    version = wpilib_version(root, "edu/wpi/first/ntcore", "ntcore-cpp")
    classifier, shared_dir = native_platform()
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
    native_dir = target / shared_dir
    if not native_dir.is_dir():
        raise SystemExit(f"Could not find extracted WPILib native directory: {native_dir}")
    return native_dir


def latest_log(repo: Path) -> Path | None:
    """Find the most recently modified WPILog file in a directory."""
    logs = sorted((repo / "SimLogs").glob("*.wpilog"), key=lambda path: path.stat().st_mtime)
    return logs[-1].resolve() if logs else None


def compile_client(
    root: Path,
    tmp: Path,
    class_name: str,
    java_source: str,
    classpath_builder=classpath,
) -> tuple[Path, str, Path]:
    """Compile the sim client Java source with the WPILib classpath."""
    source = tmp / f"{class_name}.java"
    classes = tmp / "classes"
    native_root = tmp / "native"
    classes.mkdir()
    native_root.mkdir()
    source.write_text(java_source)
    cp = classpath_builder(root)
    native_dir = extract_natives(root, native_root)
    subprocess.run([str(root / "jdk/bin/javac"), "-cp", cp, "-d", str(classes), str(source)], check=True)
    return classes, cp, native_dir


def main() -> None:
    """Parse CLI arguments and run an autonomous period simulation."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--auto", required=True, help="Auto Chooser option to select")
    parser.add_argument("--alliance", default="Red1", help="Alliance station, e.g. Red1 or Blue1")
    parser.add_argument("--duration", type=float, required=True, help="Autonomous enabled duration in seconds")
    parser.add_argument("--buffer", type=float, default=1.0, help="Extra enabled/logging buffer in seconds")
    parser.add_argument("--chooser-timeout", type=float, default=20.0, help="Seconds to wait for chooser options")
    parser.add_argument("--connection-timeout", type=float, default=45.0, help="Seconds to wait for the sim NT4 server")
    parser.add_argument("--host", default="127.0.0.1", help="NetworkTables host")
    parser.add_argument("--java-home", help="WPILib JDK; defaults to the discovered WPILib packaged JDK")
    parser.add_argument(
        "--headless-init",
        default=str(Path(__file__).resolve().parents[1] / "references/headless-sim.gradle"),
        help="Gradle init script that disables Sim GUI and real DriverStation HAL extensions",
    )
    args = parser.parse_args()

    repo = Path(args.repo).resolve()
    root = wpilib_root()
    java_home = Path(args.java_home).expanduser().resolve() if args.java_home else root / "jdk"
    before_log = latest_log(repo)

    sim_command = [
        "./gradlew",
        "--init-script",
        str(Path(args.headless_init).resolve()),
        "simulateJava",
    ]
    sim_env = os.environ.copy()
    sim_env["JAVA_HOME"] = str(java_home)
    inherited_tool_options = os.environ.get("JAVA_TOOL_OPTIONS", "").strip()
    sim_env["JAVA_TOOL_OPTIONS"] = (
        f"{inherited_tool_options} -DrobotMode=SIM -Dsim.agent.enabled=true"
        if inherited_tool_options
        else "-DrobotMode=SIM -Dsim.agent.enabled=true"
    )

    sim = subprocess.Popen(
        sim_command,
        cwd=repo,
        env=sim_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        with tempfile.TemporaryDirectory(prefix="wpilib-sim-agent-") as tmpdir:
            classes, cp, native_dir = compile_client(root, Path(tmpdir), "RunAutoNtClient", JAVA_SOURCE)
            client_command = [
                str(root / "jdk/bin/java"),
                f"-Djava.library.path={native_dir}",
                "-cp",
                os.pathsep.join([str(classes), cp]),
                "RunAutoNtClient",
                args.host,
                args.auto,
                args.alliance,
                str(args.duration),
                str(args.buffer),
                str(args.chooser_timeout),
                str(args.connection_timeout),
            ]
            subprocess.run(client_command, cwd=repo, check=True)

        timeout = max(10.0, args.duration + args.buffer + 20.0)
        output, _ = sim.communicate(timeout=timeout)
        print(output, end="")
    except Exception:
        if sim.poll() is None:
            sim.send_signal(signal.SIGTERM)
            try:
                output, _ = sim.communicate(timeout=5)
                if output:
                    print(output, end="")
            except subprocess.TimeoutExpired:
                sim.kill()
        else:
            output, _ = sim.communicate(timeout=1)
            if output:
                print(output, end="")
        raise

    if sim.returncode != 0:
        raise SystemExit(f"simulateJava exited with {sim.returncode}")

    after_log = latest_log(repo)
    if after_log is None or after_log == before_log:
        raise SystemExit("No new SimLogs/*.wpilog was created")
    print(f"latestLog={after_log}")


if __name__ == "__main__":
    main()
