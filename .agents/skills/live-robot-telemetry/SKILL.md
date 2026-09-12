---
name: live-robot-telemetry
description: "Use for live real-robot practice telemetry: connecting read-only to NT4, watching odometry and DogLog topics from an active robot, sampling bounded practice windows, and recommending optimizations from live robot data without commanding the robot."
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
- Real robot logs come from **DogLog** (via `frc.spectrumLib.telemetry.Telemetry`) and publish to NT4. Analyze copied `*.wpilog` files with `$wpilog-decode` after retrieval.

## Connection Workflow

1. Determine the robot IP from the user request or nearby context. Use `10.38.47.2` only when no stronger context exists.
2. Practice robots may use other team-number IPs, for example a practice team number like `3847`; ask or infer the exact IP before connecting if the target is unclear.
3. Connect to NT4 on port `5810`. Do not use mDNS as the default path for this team workflow.
4. If the robot is off, rebooting, or changing batteries and NT4 does not respond, poll about once every 30 seconds until it reconnects or the agent is stopped. Avoid tight retry loops while the radio and roboRIO are coming back up.
5. After a reconnect or deploy, ignore the first 20-30 seconds of loop timing unless the user explicitly asks about boot performance; initialization spikes during that window are expected.
6. Start with topic discovery before sampling:

   ```sh
   python3 .agents/skills/live-robot-telemetry/scripts/list_live_nt_topics.py \
     --host 10.38.47.2 \
     --filter Swerve
   ```
7. Take a quick status snapshot before a longer capture:

   ```sh
   python3 .agents/skills/live-robot-telemetry/scripts/snapshot_live_nt.py \
     --host 10.38.47.2 \
     --topic Swerve/State/Pose \
     --topic Swerve/State/MeasuredStates \
     --topic /DriverStation/Enabled
   ```
8. Capture a bounded practice window:

   ```sh
   python3 .agents/skills/live-robot-telemetry/scripts/sample_live_nt.py \
     --host 10.38.47.2 \
     --duration 10 \
     --period 0.1 \
     --topic Swerve/CommandSchedulerMS \
     --topic /DriverStation/Enabled \
     --spike-topic Swerve/CommandSchedulerMS \
     --spike-threshold 20
   ```

## Useful Topics

- Odometry and drive: `Swerve/State/Pose`, `Swerve/State/MeasuredStates`, `Swerve/State/MeasuredSpeeds`, `Swerve/State/TargetStates`, `Swerve/SystemState`, `Swerve/CurrentCommand`, and per-module states under `Swerve/Module*`.
- Motor currents: `Swerve/Currents/DriveStatorCurrent`, `Swerve/Currents/SteerStatorCurrent`, `Swerve/Currents/DriveSupplyCurrent`, `Swerve/Currents/SteerSupplyCurrent`.
- DriverStation state: `/DriverStation/Enabled`, `/DriverStation/Autonomous`, `/DriverStation/Test`, `/DriverStation/AllianceStation`, `/DriverStation/Joystick0/AxisValues`, `/DriverStation/Joystick0/ButtonValues`.
- Command scheduler: `Swerve/CurrentCommand`, `Commands` (init/end lifecycle), `Alerts`, `Sim/SimPose` (SIM pose), `Sim/RobotPose3d` (SIM bump-corrected pose).
- Vision: discover `Vision/*` topics, the `limelight-back/left/right` NT tables, and accepted/rejected pose status before assuming names (Limelight MegaTag pipelines).
- Subsystems: discover the real topic roots before assuming names, this robot has `FuelIntake`, `Launcher`, `IndexerBed`, `IndexerTower`, `Hood`, `Leds`, and the `SuperStructure` orchestrator.
- If topic names differ, prefer the discovered names from `list_live_nt_topics.py` over guessing.

## Analysis Workflow

- For odometry, report start/end pose, heading change, approximate path length, velocity range, sudden discontinuities, and whether motion matches the intended drill.
- For heading and gyro issues, compare odometry rotation, raw gyro, commanded heading, and DriverStation mode transitions.
- For velocity tracking, compare requested path or driver behavior with actual velocity, setpoint topics, module states, and current/voltage topics when available.
- For vision, compare accepted observations, camera names, tag counts, timestamps/latency, and pose jumps near observation updates.
- For this robot, do not remove or throttle the per-loop `setRobotOrientation(...)` heading push to the Limelights unless the user explicitly approves a replacement. Accurate fast-motion MegaTag2 fusion depends on that orientation being flushed every loop (see the vision subsystem in `src/main/java/frc/robot/subsystems/vision`).
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
