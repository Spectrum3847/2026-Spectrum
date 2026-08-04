---
name: advantagekit-replay
description: "Use for AdvantageKit WPILOG replay workflows: pulling real robot logs, running Team 8044 robot code in REPLAY mode, regenerating outputs that were disabled on the real robot, and comparing real vs replay logs to prove outputs are safely replayable."
---

# AdvantageKit Replay

## Core Rules

- Use this skill when validating that real-robot AdvantageKit outputs can be disabled and regenerated from a WPILOG replay.
- Never disable or filter `Logger.processInputs(...)` data. Replayable outputs must be recreated from logged inputs, DriverStation state, constants, and deterministic robot code.
- Keep driver-critical live topics, dashboard topics, tuning inputs, alerts, and match state outputs live unless the user explicitly approves moving them to replay-only.
- For Team 8044 Rebuilt-2026, replay mode is selected in the robot JVM with `-DrobotMode=REPLAY`; `Robot.java` uses `LogFileUtil.findReplayLog()`, so set `AKIT_LOG_PATH=<log.wpilog>` for noninteractive replay.
- Real robot logs are written under `/U`; pull logs only after the robot is disabled or the user says the capture is complete.

## Standard Workflow

1. Confirm the robot build has replayable outputs disabled on-real and enabled in replay through a single global toggle, currently `ReplayableOutput.ENABLE_ON_REAL`.
2. Deploy only while `/AdvantageKit/DriverStation/Enabled` is false.
3. Wait at least 35 seconds after deploy/reconnect before using loop timing.
4. Capture a bounded real robot window, then pull the newest WPILOG:
   ```sh
   python3 .agents/skills/advantagekit-replay/scripts/pull_robot_wpilogs.py \
     --host 10.80.44.2 \
     --out RobotLogs
   ```
5. Run replay locally. The helper uses the existing `$wpilib-sim` headless init script when present, because AdvantageKit replay requires all HAL sim extensions to be disabled.
   ```sh
   python3 .agents/skills/advantagekit-replay/scripts/run_replay.py \
     --repo . \
     --log RobotLogs/<real-log>.wpilog
   ```
6. Compare the real and replay logs:
   ```sh
   python3 .agents/skills/advantagekit-replay/scripts/compare_replay_topics.py \
     --repo . \
     --real RobotLogs/<real-log>.wpilog \
     --replay RobotLogs/<real-log>_REPLAY.wpilog
   ```

## Interpreting Results

- A safe disabled output is absent from the real log and present in the replay log.
- If a gated output is missing from replay, keep it live or fix the replay path before claiming it is safe.
- If an output is present in the real log, it was not actually disabled; check whether it is `@AutoLogOutput`, `processInputs`, or an ungated `recordOutput`.
- Use `$wpilib-sim` scripts for value summaries once topic presence is validated.

## Helper Scripts

- `scripts/pull_robot_wpilogs.py` copies the newest or all `*.wpilog` files from `admin@<host>:/U` into a local directory.
- `scripts/run_replay.py` sets `AKIT_LOG_PATH`, runs `./gradlew simulateJava -DrobotMode=REPLAY`, and prints the expected `_REPLAY.wpilog` path.
- `scripts/compare_replay_topics.py` reads WPILOG topic starts with WPILib `DataLogReader` and checks default always-exercised replayable topics. Pass `--topic` for command-path outputs that only appear when the captured log exercised that command.
