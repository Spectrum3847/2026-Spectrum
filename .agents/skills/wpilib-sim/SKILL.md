---
name: wpilib-sim
description: "Use for WPILib robot simulation work: running desktop or headless sim, selecting project-specific sim IO, inspecting live NetworkTables outputs, emulating DriverStation/joystick inputs, and producing WPILOGs for verification without adding committed one-off harnesses. Use wpilog-decode for deep WPILOG topic decoding."
metadata:
  short-description: Run WPILib sim and inspect logs
---

# WPILib Sim

## Core Rules

- Discover the repo's WPILib runtime before running commands. Prefer the packaged JDK at `<wpilib-jdk>`, commonly `~/wpilib/<wpilib-year>/jdk`, and confirm the year/path from GradleRIO config, repo docs, installed WPILib folders, or prior project scripts.
- Prefer the robot's normal WPILib simulation flow and existing telemetry/log outputs.
- Select sim IO with the repo's established mode behavior when one exists. Team 8044 Rebuilt-2026 defaults desktop runs to `SIM` in `Constants.currentMode`; agents can override with `-DrobotMode=SIM` or `-DrobotMode=REPLAY`.
- Inspect behavior through live NetworkTables for "what is happening now" and through `*.wpilog` files for repeatable before/after comparisons. Use `$wpilog-decode` for structured WPILOG decoding.
- Treat AdvantageScope automation as an optional visualization layer. Objective pass/fail checks should still come from WPILOG/NT data unless the user explicitly asks for visual-only output.
- Do not add per-feature Gradle tasks, scenario classes, or committed harness files unless the user explicitly asks for a persistent test harness.
- Do not patch robot source with temporary DriverStation/joystick hooks. Rebuilt-2026 has a permanent sim-only `SimAgentBridge`; use it for automated sim control.
- For automated checks, prefer headless sim: run the normal robot program while disabling GUI/real-driver-station HAL extensions, then inspect NT or WPILOG output.

## Standard Workflow

1. Ground in the repo with `rg`, `git status --short`, and targeted file reads.
2. Discover the WPILib JDK and any required sim-mode override:
   ```sh
   ls ~/wpilib
   rg -n "RobotMode|robotMode|currentMode|SIM|simulation|JAVA_HOME|wpilib" <repo>
   ```
3. Run normal tests before or after risky changes:
   ```sh
   JAVA_HOME=<wpilib-jdk> ./gradlew test
   ```
4. Run normal sim when a human/live tool will drive the robot:
   ```sh
   JAVA_HOME=<wpilib-jdk> ./gradlew simulateJava
   ```
   For explicit agent-controlled mode selection, pass the JVM property:
   ```sh
   JAVA_HOME=<wpilib-jdk> ./gradlew simulateJava -DrobotMode=SIM
   JAVA_HOME=<wpilib-jdk> ./gradlew simulateJava -DrobotMode=REPLAY
   ```
5. Run headless sim when Codex needs to automate checks without opening GUI windows:
   ```sh
   JAVA_HOME=<wpilib-jdk> \
     ./gradlew --init-script .agents/skills/wpilib-sim/references/headless-sim.gradle simulateJava
   ```
6. In headless automated runs, enable the permanent bridge with `JAVA_TOOL_OPTIONS="-DrobotMode=SIM -Dsim.agent.enabled=true"` and drive it through NT using the scripts below.
7. Verify outputs from existing logged topics, preferably IO-layer applied voltage, velocity, position, current, and command/reference topics. Avoid custom test-only outputs unless the user asked for a harness.
8. For log inspection, use `$wpilog-decode`. Compatibility wrappers remain in this skill for older command paths.

## SimAgentBridge

- `SimAgentBridge` is a permanent robot-code hook for agent automation. It is active only when all are true:
  - `RobotBase.isSimulation()`
  - `Constants.getRobotMode() == SIM`
  - JVM property `-Dsim.agent.enabled=true`
- It is inactive on real hardware and replay.
- It reads NT topics under `/SimAgent` and applies them with `DriverStationSim` before `CommandScheduler.run()`, so command triggers see inputs at the same point in the loop as real DS data.
- DriverStation control topics:
  - `/SimAgent/DriverStation/AllianceStation` string: `Red1`, `Red2`, `Red3`, `Blue1`, `Blue2`, `Blue3`, or `Unknown`
  - `/SimAgent/DriverStation/Enabled` boolean
  - `/SimAgent/DriverStation/Autonomous` boolean
  - `/SimAgent/DriverStation/Test` boolean
  - `/SimAgent/DriverStation/DsAttached` boolean
  - `/SimAgent/DriverStation/FmsAttached` boolean
  - `/SimAgent/Control/Exit` boolean for clean shutdown
  - `/SimAgent/Control/ExitAfterSeconds` double timeout from bridge startup
- Joystick topics by port:
  - `/SimAgent/Joystick/<port>/Axes` double array, axis indexes matching WPILib
  - `/SimAgent/Joystick/<port>/Buttons` boolean array, button 1 at index 0
  - `/SimAgent/Joystick/<port>/POVs` integer array
  - `/SimAgent/Joystick/<port>/IsXbox` boolean
  - `/SimAgent/Joystick/<port>/Type` integer
  - `/SimAgent/Joystick/<port>/Name` string
- Status/log outputs include `/RealOutputs/SimAgent/Active`, alliance, enabled/autonomous state, auto-enabled timestamp, exit reason, and observed Auto Chooser selected/active strings.
- Auto selection must go through the real SmartDashboard/NT Auto Chooser:
  - `/SmartDashboard/Auto Chooser/options`
  - `/SmartDashboard/Auto Chooser/selected`
  - `/SmartDashboard/Auto Chooser/active`
  Do not call `autoChooser.select(...)` from agent code.

## Agent Run Commands

- Run a full auto through the real NT Auto Chooser and SimAgentBridge:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_auto_sim.py \
    --repo . \
    --auto <auto-name> \
    --alliance <alliance-station> \
    --duration <auto-duration> \
    --buffer 1
  ```
  The script launches headless `simulateJava`, waits for the real chooser options, writes the selected auto over NT, enables autonomous, disables/exits cleanly, and prints `latestLog=<path>`.
- Write live teleop/joystick state to a running sim:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/sim_nt_control.py \
    --repo . \
    --alliance <alliance-station> \
    --enabled \
    --port 0 \
    --axes "0,0,0,1" \
    --buttons "false,false,false,false"
  ```
  Use `--exit` to request clean shutdown.
- Run a bounded headless teleop simulation with a fixed Xbox joystick state and a clean WPILOG:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_teleop_sim.py \
    --repo . \
    --alliance Blue1 \
    --duration 5 \
    --port 0 \
    --axes "0,-1,0,0,0,0"
  ```

## AdvantageScope Visualization

- For visualization-specific work, use `$advantagescope` when that skill is available. This skill remains responsible for running WPILib sim, choosing robot/log topics, and producing WPILOG/NT data summaries.
- Prefer vanilla AdvantageScope for manual inspection and the Team 8044 AdvantageScope fork only when its opt-in agent hooks are available.
- Detect the fork by checking for documented agent flags or a known fork checkout/build before relying on automation. If only vanilla AdvantageScope is installed, open logs/layouts manually and provide WPILOG summaries instead of promising robust screenshots or recordings.
- Keep robot-specific choices outside AdvantageScope:
  - topic names
  - preferred layouts
  - auto names
  - artifact paths
  - comparison/assertion logic
- The fork's intended flags are:
  ```sh
  AdvantageScope --agent-control
  AdvantageScope --agent-export --log <log.wpilog> --layout <layout.json> --out <output-dir> --start <seconds> --end <seconds> --fps 30
  ```
- Agent control must be local-only, opt-in, and token-protected. Prefer commands like `status`, `openLog`, `applyLayout`, `setTime`, `capture`, `export`, and `close`; do not drive the UI by clicking screen coordinates.
- For video previews, prefer the fork's realtime canvas recording path. Frame-by-frame PNG export is only a debugging fallback because it is much slower than the auto duration.
- Always pair visual artifacts with a data summary from the WPILOG, such as robot mode/type, selected auto, enabled interval, pose movement, final pose, and notable state transitions.
- For auto-review videos, do not start at timestamp `0` unless that is the actual auto start. Derive the start from logged DriverStation state, usually the first timestamp where both `/DriverStation/Autonomous` and `/DriverStation/Enabled` are true, then export the requested game/task auto duration from that point.

## Headless Sim Notes

- Headless means the normal `simulateJava` robot program runs without the WPILib Sim GUI window and without the real Driver Station socket extension.
- Use a temporary init script instead of editing `build.gradle` or changing committed simulation extension defaults.
- Set `JAVA_TOOL_OPTIONS=-Djava.awt.headless=true` only as an extra guard. It is not a substitute for disabling `Sim GUI`; if the GUI extension still loads, Java headless mode can make the run fail early instead of silently opening windows.
- Stop headless sim cleanly after collecting data so WPILOG writers can rename and close log files.

## Joystick Emulation In Sim

- Use `sim_nt_control.py` and `SimAgentBridge` for DriverStation/joystick state. Do not add temporary Java helpers to robot source.
- Prefer emulating real robot inputs and existing NT overrides over adding test-only outputs. If a command is readiness-gated, discover and use the robot's existing readiness inputs, operator overrides, or NT entries needed to pass that gate.
- Verify the emulated input in logs or NT when possible. Useful generic topics include `/DriverStation/Enabled`, `/DriverStation/Autonomous`, `/DriverStation/Joystick0/AxisValues`, `/DriverStation/Joystick0/ButtonValues`, `/DriverStation/Joystick0/Xbox`, and `/DriverStation/Joystick0/Name`.
- After any temporary hook, remove it and run `git diff` to confirm no source changes remain except the user's requested implementation.

## Script Helpers

- `$wpilog-decode` owns WPILOG listing, value summaries, struct decoding, latest-log lookup, and boolean-window lookup. Prefer:
  ```sh
  python3 .agents/skills/wpilog-decode/scripts/read_wpilog_values.py \
    --repo . \
    --log <log.wpilog> \
    --topic <topic>
  ```
- Backward-compatible wrappers remain in this skill:
  - `scripts/find_latest_simlog.py <repo-or-log-dir>`
  - `scripts/list_wpilog_topics.py --repo <repo> [--log <path-or-log-dir>] [--filter <text>]`
  - `scripts/read_wpilog_values.py --repo <repo> --topic <topic> [--topic <topic> ...] [--log <path-or-log-dir>]`
  - `scripts/find_wpilog_window.py --repo <repo> --log <path> --all-true <boolean-topic> [--all-true <boolean-topic> ...] [--duration <seconds>]`
- `scripts/run_auto_sim.py --repo <repo> --auto <chooser-option> --alliance <station> [--duration <seconds>] [--buffer 1]` runs headless sim, selects the auto through NT Auto Chooser, enables autonomous through `SimAgentBridge`, exits cleanly, and prints the latest log path.
- `scripts/run_teleop_sim.py --repo <repo> --alliance <station> --duration <seconds> --axes <comma-separated-axes> [--port 0]` runs headless sim, sets a simulated Xbox joystick through `SimAgentBridge`, enables teleop for the duration, exits cleanly, and prints the latest log path. For Xbox full forward left stick, use `--axes "0,-1,0,0,0,0"`.
- `scripts/run_teleop_sim.py` also supports timed joystick sequences with `--sequence "duration:axes;duration:axes"`. For a rectangle with Xbox joystick 0:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_teleop_sim.py \
    --repo . \
    --alliance Blue1 \
    --port 0 \
    --sequence "1.2:0,-1,0,0,0,0;1.0:-1,0,0,0,0,0;1.2:0,1,0,0,0,0;1.0:1,0,0,0,0,0;0.5:0,0,0,0,0,0"
  ```
- Timed sequences also support per-segment joystick ports, buttons, and POVs with `duration:port:axes:buttons:povs`. Port state persists until changed, which is useful for setting a secondary sim control port while driving with port 0. Example fuel intake/shoot sequence:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_teleop_sim.py \
    --repo . \
    --alliance Blue1 \
    --port 0 \
    --sequence "1.5:0:0,0,0,0,0,0:false,false,false,false,false,false,false,false,false,false:-1;2.0:0:0,-1,0,0,0,0:false,false,false,false,false,false,false,false,false,false:-1;3.0:0:0,0,0,1,0,0:false,false,false,false,false,false,false,false,false,false:-1"
  ```
- `scripts/run_shot_map_sim.py --repo . --distances "1.75,3.25,5.0"` runs fixed red-alliance own-side MapleSim fuel shots through `/SimAgent/ShotMapTest`, using shooter-map hood/flywheel values for each distance. It prints one JSON summary per log with launch speed, expected time of flight, trajectory point count, max trajectory height, closest point to the hub target, MapleSim target tolerance status, and scored-fuel return pose/velocity/count.
- For MapleSim heightmap bump smoke tests, run teleop sequences that first move to a bump lane and then drive across it. A verified crossing sequence is:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_teleop_sim.py \
    --repo . \
    --alliance Blue1 \
    --port 0 \
    --sequence "0.5:0,0,0,0,0,0;0.35:1,0,0,0,0,0;2.8:0,1,0,0,0,0;0.5:0,0,0,0,0,0"
  ```
  A verified stopped-on-bump check is:
  ```sh
  python3 .agents/skills/wpilib-sim/scripts/run_teleop_sim.py \
    --repo . \
    --alliance Blue1 \
    --port 0 \
    --sequence "0.5:0,0,0,0,0,0;0.35:1,0,0,0,0,0;0.85:0,1,0,0,0,0;2.0:0,0,0,0,0,0"
  ```
  Acceptance is not slide-back: `/RealOutputs/FieldSimulation/RobotVelocity` should settle near zero while `/RealOutputs/Simulation/Bump/OnBump`, `ModuleHeights`, `RobotPitchRad`, and/or `RobotRollRad` remain stable and nonzero.
- `scripts/sim_nt_control.py` writes live DriverStation and joystick state to a running `SimAgentBridge` sim.
- Use `$wpilog-decode` for boolean-window lookup. Compatibility command for auto preview exports:
  ```sh
  python3 .agents/skills/wpilog-decode/scripts/find_wpilog_window.py \
    --repo . \
    --log <log.wpilog> \
    --all-true /DriverStation/Autonomous \
    --all-true /DriverStation/Enabled \
    --duration <auto-duration>
  ```

The decode scripts may create temporary Java helpers using the OS temp directory and compile them directly with the discovered WPILib JDK plus local WPILib jars. They do not edit robot source.

## Verification Pattern

- For before/after checks, run the same sim flow twice and compare the same WPILOG topics.
- Prefer logged IO-layer applied motor voltage over supply voltage when judging how hard a motor is working.
- Treat missing, zero-only, or stuck IO-layer sim values as sim IO correctness issues, not as successful behavior changes.
- Keep project-specific topic names and operator inputs in notes for the current task rather than in this general skill.
