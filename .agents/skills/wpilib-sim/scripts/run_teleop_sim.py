#!/usr/bin/env python3
"""Run headless WPILib sim teleop with SimAgentBridge joystick inputs."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import tempfile
from pathlib import Path

from run_auto_sim import compile_client, latest_log, wpilib_root

JAVA_SOURCE = r'''
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class RunTeleopNtClient {
    public static void main(String[] args) throws Exception {
        String host = args[0];
        String alliance = args[1];
        double duration = Double.parseDouble(args[2]);
        int port = Integer.parseInt(args[3]);
        double[] axes = parseDoubles(args[4]);
        boolean[] buttons = parseBooleans(args[5]);
        long[] povs = parseLongs(args[6]);
        double connectionTimeout = Double.parseDouble(args[7]);
        Segment[] sequence = parseSequence(args[8], port, buttons, povs);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("wpilib-sim-agent-teleop");
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
        NetworkTable joystick = sim.getSubTable("Joystick").getSubTable(Integer.toString(port));

        ds.getEntry("AllianceStation").setString(alliance);
        ds.getEntry("DsAttached").setBoolean(true);
        ds.getEntry("FmsAttached").setBoolean(false);
        ds.getEntry("Enabled").setBoolean(false);
        ds.getEntry("Autonomous").setBoolean(false);
        ds.getEntry("Test").setBoolean(false);
        control.getEntry("Exit").setBoolean(false);
        double activeDuration = sequence.length == 0 ? duration : sequenceDuration(sequence);
        control.getEntry("ExitAfterSeconds").setDouble(activeDuration + 8.0);

        joystick.getEntry("Axes").setDoubleArray(axes);
        joystick.getEntry("Buttons").setBooleanArray(buttons);
        joystick.getEntry("POVs").setIntegerArray(povs);
        joystick.getEntry("IsXbox").setBoolean(true);
        joystick.getEntry("Type").setInteger(1);
        joystick.getEntry("Name").setString("SimAgent Xbox Controller");
        inst.flush();

        Map<Integer, double[]> lastAxes = new HashMap<>();
        Map<Integer, boolean[]> lastButtons = new HashMap<>();
        Map<Integer, long[]> lastPovs = new HashMap<>();
        lastAxes.put(port, axes);
        lastButtons.put(port, buttons);
        lastPovs.put(port, povs);

        Thread.sleep(500);
        ds.getEntry("Enabled").setBoolean(true);
        inst.flush();

        if (sequence.length == 0) {
            Thread.sleep((long) (duration * 1000.0));
        } else {
            for (Segment segment : sequence) {
                NetworkTable segmentJoystick = sim.getSubTable("Joystick").getSubTable(Integer.toString(segment.port));
                segmentJoystick.getEntry("Axes").setDoubleArray(segment.axes);
                segmentJoystick.getEntry("Buttons").setBooleanArray(segment.buttons);
                segmentJoystick.getEntry("POVs").setIntegerArray(segment.povs);
                segmentJoystick.getEntry("IsXbox").setBoolean(true);
                segmentJoystick.getEntry("Type").setInteger(1);
                segmentJoystick.getEntry("Name").setString("SimAgent Xbox Controller");
                inst.flush();
                lastAxes.put(segment.port, segment.axes);
                lastButtons.put(segment.port, segment.buttons);
                lastPovs.put(segment.port, segment.povs);
                Thread.sleep((long) (segment.durationSeconds * 1000.0));
            }
        }

        for (int clearPort : touchedPorts(port, sequence)) {
            NetworkTable clearJoystick = sim.getSubTable("Joystick").getSubTable(Integer.toString(clearPort));
            clearJoystick.getEntry("Axes").setDoubleArray(new double[lastAxes.get(clearPort).length]);
            clearJoystick.getEntry("Buttons").setBooleanArray(new boolean[lastButtons.get(clearPort).length]);
            long[] clearedPovs = new long[lastPovs.get(clearPort).length];
            Arrays.fill(clearedPovs, -1L);
            clearJoystick.getEntry("POVs").setIntegerArray(clearedPovs);
        }
        ds.getEntry("Enabled").setBoolean(false);
        control.getEntry("Exit").setBoolean(true);
        inst.flush();
        Thread.sleep(500);
        inst.stopClient();

        System.out.println("alliance=" + alliance);
        System.out.printf("duration=%.3f%n", activeDuration);
        System.out.println("axes=" + Arrays.toString(axes));
        System.out.println("sequence=" + Arrays.toString(sequence));
    }

    private static double[] parseDoubles(String value) {
        if (value.isBlank()) return new double[] {};
        return Arrays.stream(value.split(",")).map(String::trim).mapToDouble(Double::parseDouble).toArray();
    }

    private static boolean[] parseBooleans(String value) {
        if (value.isBlank()) return new boolean[] {};
        String[] parts = value.split(",");
        boolean[] result = new boolean[parts.length];
        for (int i = 0; i < parts.length; i++) result[i] = Boolean.parseBoolean(parts[i].trim());
        return result;
    }

    private static long[] parseLongs(String value) {
        if (value.isBlank()) return new long[] {};
        return Arrays.stream(value.split(",")).map(String::trim).mapToLong(Long::parseLong).toArray();
    }

    private static long secondsToNanos(double seconds) {
        return (long) (seconds * 1_000_000_000.0);
    }

    private static Segment[] parseSequence(String value, int defaultPort, boolean[] defaultButtons, long[] defaultPovs) {
        if (value.isBlank()) return new Segment[] {};
        String[] rawSegments = value.split(";");
        Segment[] result = new Segment[rawSegments.length];
        for (int i = 0; i < rawSegments.length; i++) {
            String[] parts = rawSegments[i].trim().split(":", 5);
            if (parts.length != 2 && parts.length != 5) {
                throw new IllegalArgumentException("Sequence segment must be duration:axes or duration:port:axes:buttons:povs, got " + rawSegments[i]);
            }
            if (parts.length == 2) {
                result[i] = new Segment(Double.parseDouble(parts[0].trim()), defaultPort, parseDoubles(parts[1]), defaultButtons, defaultPovs);
            } else {
                result[i] = new Segment(Double.parseDouble(parts[0].trim()), Integer.parseInt(parts[1].trim()), parseDoubles(parts[2]), parseBooleans(parts[3]), parseLongs(parts[4]));
            }
        }
        return result;
    }

    private static double sequenceDuration(Segment[] sequence) {
        double total = 0.0;
        for (Segment segment : sequence) total += segment.durationSeconds;
        return total;
    }

    private static int[] touchedPorts(int defaultPort, Segment[] sequence) {
        int[] ports = new int[sequence.length + 1];
        int count = 0;
        ports[count++] = defaultPort;
        for (Segment segment : sequence) {
            boolean seen = false;
            for (int i = 0; i < count; i++) {
                if (ports[i] == segment.port) {
                    seen = true;
                    break;
                }
            }
            if (!seen) ports[count++] = segment.port;
        }
        return Arrays.copyOf(ports, count);
    }

    private static class Segment {
        final double durationSeconds;
        final int port;
        final double[] axes;
        final boolean[] buttons;
        final long[] povs;

        Segment(double durationSeconds, int port, double[] axes, boolean[] buttons, long[] povs) {
            this.durationSeconds = durationSeconds;
            this.port = port;
            this.axes = axes;
            this.buttons = buttons;
            this.povs = povs;
        }

        @Override
        public String toString() {
            return String.format("%.3fs:port%d axes=%s buttons=%s povs=%s", durationSeconds, port, Arrays.toString(axes), Arrays.toString(buttons), Arrays.toString(povs));
        }
    }
}
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--alliance", default="Blue1", help="Alliance station, e.g. Red1 or Blue1")
    parser.add_argument("--duration", type=float, help="Teleop enabled duration in seconds")
    parser.add_argument("--port", type=int, default=0, help="Joystick port")
    parser.add_argument("--axes", default="0,0,0,0,0,0", help="Comma-separated joystick axes")
    parser.add_argument(
        "--sequence",
        default="",
        help="Semicolon-separated duration:axes segments, e.g. '1.2:0,-1,0,0,0,0;1.0:-1,0,0,0,0,0'",
    )
    parser.add_argument("--buttons", default="false,false,false,false,false,false,false,false,false,false")
    parser.add_argument("--povs", default="-1")
    parser.add_argument("--connection-timeout", type=float, default=45.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--java-home", help="WPILib JDK; defaults to the discovered WPILib packaged JDK")
    parser.add_argument(
        "--headless-init",
        default=str(Path(__file__).resolve().parents[1] / "references/headless-sim.gradle"),
        help="Gradle init script that disables Sim GUI and real DriverStation HAL extensions",
    )
    args = parser.parse_args()
    if not args.sequence and args.duration is None:
        parser.error("--duration is required unless --sequence is provided")
    duration = args.duration if args.duration is not None else 0.0
    timeout_duration = duration
    if args.sequence:
        timeout_duration = 0.0
        for segment in args.sequence.split(";"):
            if not segment.strip():
                continue
            parts = segment.split(":", 1)
            if len(parts) != 2 or not parts[1].strip():
                parser.error(f"Sequence segment must be duration:... got '{segment}'")
            try:
                timeout_duration += float(parts[0].strip())
            except ValueError:
                parser.error(f"Sequence segment duration must be numeric, got '{parts[0].strip()}'")

    repo = Path(args.repo).resolve()
    root = wpilib_root()
    java_home = Path(args.java_home).expanduser().resolve() if args.java_home else root / "jdk"
    before_log = latest_log(repo)

    sim_command = ["./gradlew", "--init-script", str(Path(args.headless_init).resolve()), "simulateJava"]
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
        with tempfile.TemporaryDirectory(prefix="wpilib-sim-teleop-") as tmpdir:
            classes, cp, native_dir = compile_client(root, Path(tmpdir), "RunTeleopNtClient", JAVA_SOURCE)
            client_command = [
                str(root / "jdk/bin/java"),
                f"-Djava.library.path={native_dir}",
                "-cp",
                os.pathsep.join([str(classes), cp]),
                "RunTeleopNtClient",
                args.host,
                args.alliance,
                str(duration),
                str(args.port),
                args.axes,
                args.buttons,
                args.povs,
                str(args.connection_timeout),
                args.sequence,
            ]
            subprocess.run(client_command, cwd=repo, check=True)

        output, _ = sim.communicate(timeout=max(15.0, timeout_duration + 20.0))
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
