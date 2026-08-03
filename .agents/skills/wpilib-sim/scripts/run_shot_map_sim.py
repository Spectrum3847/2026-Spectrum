#!/usr/bin/env python3
"""Run MapleSim fuel shot checks at fixed shooter-map distances."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from run_auto_sim import classpath, compile_client, extract_natives, latest_log, wpilib_root, wpilib_version


JAVA_SOURCE = r'''
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RunShotMapNtClient {
    private static final double HUB_WIDTH_METERS = Units.inchesToMeters(47.0);
    private static final double[][] SHOOTER_MAP = new double[][] {
        {1.22, Math.toRadians(2.5), Units.rotationsPerMinuteToRadiansPerSecond(2250), 1.0},
        {1.75, Math.toRadians(7.0), Units.rotationsPerMinuteToRadiansPerSecond(2300), 1.0},
        {2.25, Math.toRadians(10.0), Units.rotationsPerMinuteToRadiansPerSecond(2400), 1.0},
        {2.75, Math.toRadians(14.0), Units.rotationsPerMinuteToRadiansPerSecond(2500), 1.0},
        {3.25, Math.toRadians(15.0), Units.rotationsPerMinuteToRadiansPerSecond(2675), 1.1},
        {3.6, Math.toRadians(15.5), Units.rotationsPerMinuteToRadiansPerSecond(2750), 1.1},
        {3.8, Math.toRadians(15.5), Units.rotationsPerMinuteToRadiansPerSecond(2825), 1.1},
        {4.25, Math.toRadians(16.0), Units.rotationsPerMinuteToRadiansPerSecond(2960), 1.2},
        {4.5, Math.toRadians(16.5), Units.rotationsPerMinuteToRadiansPerSecond(3050), 1.2},
        {4.8, Math.toRadians(17.5), Units.rotationsPerMinuteToRadiansPerSecond(3150), 1.2},
        {5.0, Math.toRadians(18.0), Units.rotationsPerMinuteToRadiansPerSecond(3200), 1.3},
        {5.2, Math.toRadians(18.5), Units.rotationsPerMinuteToRadiansPerSecond(3275), 1.3},
        {5.5, Math.toRadians(19.0), Units.rotationsPerMinuteToRadiansPerSecond(3400), 1.3}
    };

    public static void main(String[] args) throws Exception {
        String host = args[0];
        double distance = Double.parseDouble(args[1]);
        double settleSeconds = Double.parseDouble(args[2]);
        double enabledSeconds = Double.parseDouble(args[3]);
        int requestId = Integer.parseInt(args[4]);

        AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        double targetX = field.getTagPose(10).orElseThrow().getX() - HUB_WIDTH_METERS / 2.0;
        double targetY = field.getFieldWidth() / 2.0;
        double robotX = targetX + distance;
        double robotY = targetY;
        double robotHeading = Math.PI;
        double hood = interpolate(distance, 1);
        double flywheel = interpolate(distance, 2);
        double tof = interpolate(distance, 3);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("wpilib-sim-agent-shot-map");
        inst.setServer(host, 5810);
        long deadline = System.nanoTime() + 45_000_000_000L;
        while (!inst.isConnected() && System.nanoTime() < deadline) {
            Thread.sleep(50);
        }
        if (!inst.isConnected()) {
            throw new RuntimeException("Timed out waiting for NT4 connection to " + host + ":5810");
        }

        NetworkTable sim = inst.getTable("SimAgent");
        NetworkTable ds = sim.getSubTable("DriverStation");
        NetworkTable control = sim.getSubTable("Control");
        NetworkTable shot = sim.getSubTable("ShotMapTest");
        ds.getEntry("AllianceStation").setString("Red1");
        ds.getEntry("DsAttached").setBoolean(true);
        ds.getEntry("FmsAttached").setBoolean(true);
        ds.getEntry("Enabled").setBoolean(false);
        ds.getEntry("Autonomous").setBoolean(false);
        ds.getEntry("Test").setBoolean(false);
        control.getEntry("Exit").setBoolean(false);
        control.getEntry("ExitAfterSeconds").setDouble(settleSeconds + enabledSeconds + 12.0);
        inst.flush();

        Thread.sleep((long) (settleSeconds * 1000.0));
        shot.getEntry("Enabled").setBoolean(true);
        shot.getEntry("RequestId").setInteger(requestId);
        shot.getEntry("RobotPose").setDoubleArray(new double[] {robotX, robotY, robotHeading});
        shot.getEntry("FuelCount").setInteger(1);
        shot.getEntry("TurretAngleRad").setDouble(0.0);
        shot.getEntry("HoodAngleRad").setDouble(hood);
        shot.getEntry("FlywheelVelocityRadPerSec").setDouble(flywheel);
        shot.getEntry("FireDelaySeconds").setDouble(1.0);
        shot.getEntry("FeederVelocityRadPerSec").setDouble(100.0);
        shot.getEntry("IndexerVelocityRadPerSec").setDouble(100.0);
        inst.flush();
        Thread.sleep((long) (settleSeconds * 1000.0));
        ds.getEntry("Enabled").setBoolean(true);
        inst.flush();
        Thread.sleep((long) (enabledSeconds * 1000.0));

        ds.getEntry("Enabled").setBoolean(false);
        shot.getEntry("Enabled").setBoolean(false);
        control.getEntry("Exit").setBoolean(true);
        inst.flush();
        Thread.sleep(500);
        inst.stopClient();

        System.out.printf("distance=%.3f%n", distance);
        System.out.printf("robotPose=%.6f,%.6f,%.6f%n", robotX, robotY, robotHeading);
        System.out.printf("target=%.6f,%.6f%n", targetX, targetY);
        System.out.printf("mapHoodRad=%.9f%n", hood);
        System.out.printf("mapFlywheelRadPerSec=%.9f%n", flywheel);
        System.out.printf("mapTimeOfFlight=%.9f%n", tof);
    }

    private static double interpolate(double distance, int column) {
        if (distance <= SHOOTER_MAP[0][0]) return SHOOTER_MAP[0][column];
        for (int i = 1; i < SHOOTER_MAP.length; i++) {
            if (distance <= SHOOTER_MAP[i][0]) {
                double x0 = SHOOTER_MAP[i - 1][0];
                double x1 = SHOOTER_MAP[i][0];
                double t = (distance - x0) / (x1 - x0);
                return SHOOTER_MAP[i - 1][column] + t * (SHOOTER_MAP[i][column] - SHOOTER_MAP[i - 1][column]);
            }
        }
        return SHOOTER_MAP[SHOOTER_MAP.length - 1][column];
    }
}
'''


POSE_PATTERN = re.compile(r"Pose3d\(x=([-0-9.]+),y=([-0-9.]+),z=([-0-9.]+),")
TRANSLATION_PATTERN = re.compile(r"Translation3d\(x=([-0-9.]+),y=([-0-9.]+),z=([-0-9.]+)\)")

HUB_SIDE_MIN_HEIGHT_METERS = 1.435  # Fuel must clear this height to leave the hub side
MAPLESIM_TARGET_TOLERANCE_METERS = 0.7  # Default tolerance when matching simulated shots to the target


def shot_client_classpath(root: Path) -> str:
    """Build the Java classpath for the shot-map sim client (WPILib + sim jars)."""
    version = wpilib_version(root, "edu/wpi/first/ntcore", "ntcore-java")
    base = classpath(root).split(os.pathsep)
    quickbuf_version = wpilib_version(root, "us/hebi/quickbuf", "quickbuf-runtime")
    extra = [
        root / f"maven/edu/wpi/first/wpimath/wpimath-java/{version}/wpimath-java-{version}.jar",
        root / f"maven/edu/wpi/first/apriltag/apriltag-java/{version}/apriltag-java-{version}.jar",
        root / f"maven/edu/wpi/first/wpiunits/wpiunits-java/{version}/wpiunits-java-{version}.jar",
        root / f"maven/us/hebi/quickbuf/quickbuf-runtime/{quickbuf_version}/quickbuf-runtime-{quickbuf_version}.jar",
    ]
    missing = [str(jar) for jar in extra if not jar.exists()]
    if missing:
        raise SystemExit("Missing WPILib Java dependency jars:\n" + "\n".join(missing))
    return os.pathsep.join(base + [str(jar) for jar in extra])


def run_one(
    repo: Path,
    root: Path,
    java_home: Path,
    headless_init: Path,
    distance: float,
    request_id: int,
    settle_seconds: float,
    enabled_seconds: float,
) -> Path:
    """Run a single shot-map simulation trial and return parsed results."""
    before_log = latest_log(repo)
    sim_command = ["./gradlew", "--init-script", str(headless_init), "simulateJava"]
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
        with tempfile.TemporaryDirectory(prefix="wpilib-sim-shot-map-") as tmpdir:
            classes, cp, native_dir = compile_client(
                root, Path(tmpdir), "RunShotMapNtClient", JAVA_SOURCE, shot_client_classpath
            )
            subprocess.run(
                [
                    str(root / "jdk/bin/java"),
                    f"-Djava.library.path={native_dir}",
                    "-cp",
                    os.pathsep.join([str(classes), cp]),
                    "RunShotMapNtClient",
                    "127.0.0.1",
                    str(distance),
                    str(settle_seconds),
                    str(enabled_seconds),
                    str(request_id),
                ],
                cwd=repo,
                check=True,
            )
        sim_timeout = max(90.0, 2.0 * settle_seconds + enabled_seconds + 60.0)
        output, _ = sim.communicate(timeout=sim_timeout)
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
        raise

    if sim.returncode != 0:
        raise SystemExit(f"simulateJava exited with {sim.returncode}")
    after_log = latest_log(repo)
    if after_log is None or after_log == before_log:
        raise SystemExit("No new SimLogs/*.wpilog was created")
    return after_log


def decode(repo: Path, log: Path, topics: list[str]) -> dict:
    """Decode a Java sim-client output line into a dict of typed fields."""
    command = [
        sys.executable,
        str(repo / ".agents/skills/wpilog-decode/scripts/read_wpilog_values.py"),
        "--repo",
        str(repo),
        "--log",
        str(log),
        "--json",
        "--samples",
        "20",
    ]
    for topic in topics:
        command.extend(["--topic", topic])
    result = subprocess.run(command, cwd=repo, check=True, text=True, capture_output=True)
    return json.loads(result.stdout)["topics"]


def latest_number(topics: dict, topic: str) -> float:
    """Return the latest value of a topic as a float, or NaN when it is not numeric."""
    value = topics.get(topic, {}).get("latest", "-")
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def trajectory_points(topics: dict) -> list[tuple[float, float, float]]:
    """Extract trajectory reference points (x, y) from the sim log."""
    samples = topics.get("/RealOutputs/FieldSimulation/FuelShotTrajectory", {}).get("samples", [])
    for sample in reversed(samples):
        value = str(sample.get("value", ""))
        matches = POSE_PATTERN.findall(value)
        if matches:
            return [(float(x), float(y), float(z)) for x, y, z in matches]
    return []


def latest_translation(topics: dict, topic: str) -> tuple[float, float, float]:
    """Extract the latest robot translation from the sim log."""
    value = topics.get(topic, {}).get("latest", "")
    match = TRANSLATION_PATTERN.search(str(value))
    if not match:
        return (math.nan, math.nan, math.nan)
    return tuple(float(component) for component in match.groups())


def latest_pose(topics: dict, topic: str) -> tuple[float, float, float]:
    """Return the x, y, z components of the latest Pose3d value for a topic."""
    value = topics.get(topic, {}).get("latest", "")
    match = POSE_PATTERN.search(str(value))
    if not match:
        return (math.nan, math.nan, math.nan)
    return tuple(float(component) for component in match.groups())


def closest_point_to_target(
    points: list[tuple[float, float, float]],
    target: tuple[float, float, float],
) -> tuple[tuple[float, float, float], float]:
    """Find the trajectory reference point closest to the given robot position."""
    if not points or any(math.isnan(component) for component in target):
        return ((math.nan, math.nan, math.nan), math.nan)
    closest = min(points, key=lambda point: math.dist(point, target))
    return closest, math.dist(closest, target)


def inside_maplesim_target_tolerance(
    point: tuple[float, float, float],
    target: tuple[float, float, float],
    tolerance: float = MAPLESIM_TARGET_TOLERANCE_METERS,
) -> bool:
    """Check whether the robot pose is within the Maplesim target tolerance."""
    if any(math.isnan(component) for component in point + target):
        return False
    return all(abs(point[index] - target[index]) <= tolerance for index in range(3))


def summarize(repo: Path, log: Path, distance: float) -> dict:
    """Summarize the result of a single shot-map simulation trial."""
    topics = decode(
        repo,
        log,
        [
            "/RealOutputs/Simulation/Fuel/ShotLaunchSpeedMetersPerSecond",
            "/RealOutputs/Simulation/Fuel/ShotExpectedTimeOfFlightSeconds",
            "/RealOutputs/Simulation/Fuel/ShotTargetDistanceMeters",
            "/RealOutputs/Simulation/Fuel/ShotTargetPosition",
            "/RealOutputs/Simulation/Fuel/ShooterLaunchPitchDeg",
            "/RealOutputs/Simulation/Fuel/ShotsLaunched",
            "/RealOutputs/Simulation/Fuel/InRobotCount",
            "/RealOutputs/Simulation/Fuel/ReturnedFuelCount",
            "/RealOutputs/Simulation/Fuel/LastReturnPose",
            "/RealOutputs/Simulation/Fuel/LastReturnVelocityMPS",
            "/RealOutputs/FieldSimulation/FuelShotTrajectory",
        ],
    )
    points = trajectory_points(topics)
    max_z = max((point[2] for point in points), default=math.nan)
    final = points[-1] if points else (math.nan, math.nan, math.nan)
    target = latest_translation(topics, "/RealOutputs/Simulation/Fuel/ShotTargetPosition")
    closest, closest_distance = closest_point_to_target(points, target)
    return {
        "distance": distance,
        "log": str(log),
        "launchSpeed": latest_number(topics, "/RealOutputs/Simulation/Fuel/ShotLaunchSpeedMetersPerSecond"),
        "expectedTOF": latest_number(topics, "/RealOutputs/Simulation/Fuel/ShotExpectedTimeOfFlightSeconds"),
        "targetDistance": latest_number(topics, "/RealOutputs/Simulation/Fuel/ShotTargetDistanceMeters"),
        "launchPitchDeg": latest_number(topics, "/RealOutputs/Simulation/Fuel/ShooterLaunchPitchDeg"),
        "shotsLaunched": latest_number(topics, "/RealOutputs/Simulation/Fuel/ShotsLaunched"),
        "inRobotCount": latest_number(topics, "/RealOutputs/Simulation/Fuel/InRobotCount"),
        "returnedFuelCount": latest_number(topics, "/RealOutputs/Simulation/Fuel/ReturnedFuelCount"),
        "lastReturnPose": latest_pose(topics, "/RealOutputs/Simulation/Fuel/LastReturnPose"),
        "lastReturnVelocity": latest_translation(topics, "/RealOutputs/Simulation/Fuel/LastReturnVelocityMPS"),
        "trajectoryPoints": len(points),
        "trajectoryMaxZ": max_z,
        "trajectoryFinal": final,
        "target": target,
        "closestToTarget": closest,
        "closestTargetDistance": closest_distance,
        "clearsHubSide": max_z >= HUB_SIDE_MIN_HEIGHT_METERS,
        "nearTarget": inside_maplesim_target_tolerance(closest, target),
    }


def main() -> None:
    """Parse CLI arguments and run one or more shot-map simulations."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default=".", help="Robot repo root")
    parser.add_argument("--distances", default="1.75,3.25,5.0", help="Comma-separated shooter-map distances in meters")
    parser.add_argument("--settle-seconds", type=float, default=3.0, help="Disabled settle time before each shot")
    parser.add_argument(
        "--enabled-seconds",
        type=float,
        default=4.5,
        help="Enabled observation time after each shot request; keep long enough for scored fuel return",
    )
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
    headless_init = Path(args.headless_init).resolve()
    distances = [float(value.strip()) for value in args.distances.split(",") if value.strip()]

    summaries = []
    request_base = int(time.time())
    for index, distance in enumerate(distances):
        log = run_one(
            repo,
            root,
            java_home,
            headless_init,
            distance,
            request_base + index,
            args.settle_seconds,
            args.enabled_seconds,
        )
        summary = summarize(repo, log, distance)
        summaries.append(summary)
        print(json.dumps(summary, indent=2))

    print("shotMapSummary=" + json.dumps(summaries))


if __name__ == "__main__":
    main()
