---
name: live-robot-telemetry
description: "Use for live real-robot practice telemetry: connecting read-only to NT4, watching odometry and AdvantageKit topics from an active robot, sampling bounded practice windows, and recommending optimizations from live robot data without commanding the robot."
metadata:
  short-description: Analyze live robot NT4 telemetry
---

# Live Robot Telemetry

## Core Rules

- Use this skill when the user wants live data from a real robot while it is practicing, driving, running autos, or being tuned by humans.
- Stay read-only by default. Do not command actuators, set DriverStation state, write joystick values, change auto selection, or write tuning entries while the robot is moving.
- Treat live analysis as observation plus recommendations. If a change is needed, summarize the evidence, propose the code/tuning change, and validate it later with sim, WPILOG replay, or a separate user-approved disabled-robot tuning session.
- Do not interfere with Driver Station, FMS, practice field control, or the drive team. If telemetry looks unsafe, tell the user to disable the robot; do not try to disable it over NT.
- Prefer bounded capture windows. Avoid open-ended live monitors unless the user explicitly asks for one and the command has a clear timeout.
- Real robot logs are written by AdvantageKit to `/U` on the robot. Live data comes from `NT4Publisher`; analyze copied `*.wpilog` files with `$wpilog-decode` after retrieval.

## Connection Workflow

1. Determine the robot IP from the user request or nearby context. Use `10.80.44.2` only when no stronger context exists.
2. Practice robots may use other team-number IPs, for example a practice team number like `9994`; ask or infer the exact IP before connecting if the target is unclear.
3. Connect to NT4 on port `5810`. Do not use mDNS as the default path for this team workflow.
4. If the robot is off, rebooting, or changing batteries and NT4 does not respond, poll about once every 30 seconds until it reconnects or the agent is stopped. Avoid tight retry loops while the radio and roboRIO are coming back up.
5. After a reconnect or deploy, ignore the first 20-30 seconds of loop timing unless the user explicitly asks about boot performance; initialization spikes during that window are expected.
6. Start with topic discovery before sampling:
   ```sh
   python3 .agents/skills/live-robot-telemetry/scripts/list_live_nt_topics.py \
     --host 10.80.44.2 \
     --filter /AdvantageKit/RealOutputs/Swerve
   ```
7. Take a quick status snapshot before a longer capture:
   ```sh
   python3 .agents/skills/live-robot-telemetry/scripts/snapshot_live_nt.py \
     --host 10.80.44.2 \
     --topic /AdvantageKit/RealOutputs/Swerve/Odometry/Robot \
     --topic /AdvantageKit/RealOutputs/Swerve/Actual_Velocity \
     --topic /AdvantageKit/DriverStation/Enabled
   ```
8. Capture a bounded practice window:
   ```sh
   python3 .agents/skills/live-robot-telemetry/scripts/sample_live_nt.py \
     --host 10.80.44.2 \
     --duration 10 \
     --period 0.1 \
     --topic /AdvantageKit/RealOutputs/LoggedRobot/FullCycleMS \
     --topic /AdvantageKit/RealOutputs/LoggedRobot/UserCodeMS \
     --topic /AdvantageKit/RealOutputs/LoopProfiler/Robot/CommandSchedulerMS \
     --topic /AdvantageKit/DriverStation/Enabled \
     --spike-topic /AdvantageKit/RealOutputs/LoggedRobot/FullCycleMS \
     --spike-threshold 20
   ```

## Useful Topics

- Odometry and drive: `/AdvantageKit/RealOutputs/Swerve/Odometry/Robot`, `/AdvantageKit/RealOutputs/Swerve/Actual_Velocity`, `/AdvantageKit/RealOutputs/Swerve/SwerveStates/Rawgyro`, `/AdvantageKit/RealOutputs/Swerve/SwerveStates/Setpoints`, module inputs under `/AdvantageKit/Swerve/Module*`.
- DriverStation state: `/AdvantageKit/DriverStation/Enabled`, `/AdvantageKit/DriverStation/Autonomous`, `/AdvantageKit/DriverStation/Test`, `/AdvantageKit/DriverStation/AllianceStation`, `/AdvantageKit/DriverStation/Joystick0/AxisValues`, `/AdvantageKit/DriverStation/Joystick0/ButtonValues`.
- Loop timing: `/AdvantageKit/RealOutputs/LoggedRobot/FullCycleMS`, `/AdvantageKit/RealOutputs/LoggedRobot/UserCodeMS`, `/AdvantageKit/RealOutputs/LoggedRobot/LogPeriodicMS`, `/AdvantageKit/RealOutputs/Logger/*MS`, and `/AdvantageKit/RealOutputs/LoopProfiler/...MS` when profiling code is deployed.
- Vision: discover `/AdvantageKit/RealOutputs/Vision`, `/AdvantageKit/Vision`, Limelight tables, and accepted/rejected pose topics before assuming names.
- Superstructure: discover `/AdvantageKit/RealOutputs/SuperStructure`, `/AdvantageKit/RealOutputs/Shooter`, `/AdvantageKit/RealOutputs/Intake`, `/AdvantageKit/RealOutputs/Hopper`, `/AdvantageKit/RealOutputs/Turret`, `/AdvantageKit/RealOutputs/States`, and `/AdvantageKit/RealOutputs/DriverDash`.
- If topic names differ, prefer the discovered AdvantageKit names over adding duplicate robot outputs.

## Analysis Workflow

- For odometry, report start/end pose, heading change, approximate path length, velocity range, sudden discontinuities, and whether motion matches the intended drill.
- For heading and gyro issues, compare odometry rotation, raw gyro, commanded heading, and DriverStation mode transitions.
- For velocity tracking, compare requested path or driver behavior with actual velocity, setpoint topics, module states, and current/voltage topics when available.
- For vision, compare accepted observations, camera names, tag counts, timestamps/latency, and pose jumps near observation updates.
- For this robot, do not remove or throttle the per-loop NT flush used to push Limelight robot pose and turret pose data unless the user explicitly approves a replacement. Accurate fast-motion vision depends on those camera inputs being flushed every loop.
- For subsystem optimization, tie recommendations to captured evidence: command state, setpoint, measured response, current draw, voltage demand, and DriverStation mode.
- When the user wants a visual artifact, capture or retrieve a WPILOG first, then use `$advantagescope` for layouts/exports. Live NT snapshots alone are for data summaries, not full AdvantageScope playback.

## Helper Scripts

- `scripts/list_live_nt_topics.py --host <ip> [--port 5810] [--filter text] [--json]` listens for published NT topics and lists remote topic names/types.
- `scripts/snapshot_live_nt.py --host <ip> --topic <topic> ... [--json]` prints one timestamped read of selected topics.
- `scripts/sample_live_nt.py --host <ip> --duration <seconds> --topic <topic> ... [--period 0.1] [--spike-topic <topic>] [--spike-threshold <number>] [--json]` samples selected topics for a bounded window, summarizes first/latest/min/max where possible, and can print raw samples above a spike threshold.
- The scripts compile temporary Java NT4 clients in the OS temp directory using the installed WPILib JDK and jars. They do not edit robot source.

## Handoff Pattern

- Separate observed facts from recommendations. Include the robot IP, capture duration, key topics, DriverStation mode, and any missing topics.
- If recommending code changes, point to the subsystem or behavior implicated by the data and describe the validation run needed afterward.
- If a WPILOG is available, switch to `$wpilog-decode` for structured decoding and `$advantagescope` for visual review.
