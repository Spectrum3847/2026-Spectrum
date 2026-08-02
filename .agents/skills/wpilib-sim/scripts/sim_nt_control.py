#!/usr/bin/env python3
"""Write generic SimAgentBridge DriverStation and joystick state over NetworkTables."""

from __future__ import annotations

import argparse
import subprocess
import tempfile
from pathlib import Path

from run_auto_sim import classpath, extract_natives, wpilib_root


JAVA_SOURCE = r'''
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;

public class SimNtControlClient {
    public static void main(String[] args) throws Exception {
        String host = args[0];
        String alliance = args[1];
        boolean enabled = Boolean.parseBoolean(args[2]);
        boolean autonomous = Boolean.parseBoolean(args[3]);
        boolean test = Boolean.parseBoolean(args[4]);
        int port = Integer.parseInt(args[5]);
        double[] axes = parseDoubles(args[6]);
        boolean[] buttons = parseBooleans(args[7]);
        long[] povs = parseLongs(args[8]);
        boolean exit = Boolean.parseBoolean(args[9]);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("wpilib-sim-agent-control");
        inst.setServer(host, 5810);
        long deadline = System.nanoTime() + 15_000_000_000L;
        while (!inst.isConnected() && System.nanoTime() < deadline) {
            Thread.sleep(50);
        }
        if (!inst.isConnected()) {
            throw new RuntimeException("Timed out waiting for NT4 connection to " + host + ":5810");
        }

        NetworkTable sim = inst.getTable("SimAgent");
        NetworkTable ds = sim.getSubTable("DriverStation");
        ds.getEntry("AllianceStation").setString(alliance);
        ds.getEntry("DsAttached").setBoolean(true);
        ds.getEntry("FmsAttached").setBoolean(false);
        ds.getEntry("Enabled").setBoolean(enabled);
        ds.getEntry("Autonomous").setBoolean(autonomous);
        ds.getEntry("Test").setBoolean(test);

        if (port >= 0) {
            NetworkTable joystick = sim.getSubTable("Joystick").getSubTable(Integer.toString(port));
            joystick.getEntry("Axes").setDoubleArray(axes);
            joystick.getEntry("Buttons").setBooleanArray(buttons);
            joystick.getEntry("POVs").setIntegerArray(povs);
            joystick.getEntry("IsXbox").setBoolean(true);
            joystick.getEntry("Type").setInteger(1);
            joystick.getEntry("Name").setString("SimAgent Xbox Controller");
        }

        if (exit) {
            sim.getSubTable("Control").getEntry("Exit").setBoolean(true);
        }

        inst.flush();
        Thread.sleep(250);
        inst.stopClient();
        System.out.println("wroteSimAgentState=true");
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
}
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--alliance", default="Red1")
    parser.add_argument("--enabled", action="store_true")
    parser.add_argument("--autonomous", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--port", type=int, default=-1, help="Joystick port to write, or -1 for none")
    parser.add_argument("--axes", default="", help="Comma-separated joystick axes")
    parser.add_argument("--buttons", default="", help="Comma-separated booleans for buttons 1..N")
    parser.add_argument("--povs", default="", help="Comma-separated POV values")
    parser.add_argument("--exit", action="store_true", help="Request clean sim exit")
    args = parser.parse_args()

    root = wpilib_root()
    with tempfile.TemporaryDirectory(prefix="wpilib-sim-control-") as tmpdir:
        tmp = Path(tmpdir)
        source = tmp / "SimNtControlClient.java"
        classes = tmp / "classes"
        native_root = tmp / "native"
        classes.mkdir()
        native_root.mkdir()
        source.write_text(JAVA_SOURCE)
        native_dir = extract_natives(root, native_root)
        cp = classpath(root)
        subprocess.run(
            [str(root / "jdk/bin/javac"), "-cp", cp, "-d", str(classes), str(source)],
            cwd=Path(args.repo).resolve(),
            check=True,
        )
        subprocess.run(
            [
                str(root / "jdk/bin/java"),
                f"-Djava.library.path={native_dir}",
                "-cp",
                f"{classes}:{cp}",
                "SimNtControlClient",
                args.host,
                args.alliance,
                str(args.enabled).lower(),
                str(args.autonomous).lower(),
                str(args.test).lower(),
                str(args.port),
                args.axes,
                args.buttons,
                args.povs,
                str(args.exit).lower(),
            ],
            cwd=Path(args.repo).resolve(),
            check=True,
        )


if __name__ == "__main__":
    main()
