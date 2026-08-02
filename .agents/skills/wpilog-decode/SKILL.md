---
name: wpilog-decode
description: "Use for WPILOG/DataLog inspection and decoding robot telemetry: listing topics, finding log windows, summarizing primitive values, decoding WPILib structs such as Pose2d/Rotation2d/Pose3d/SwerveModuleState, and exporting readable or JSON samples without running simulation."
metadata:
  short-description: Decode WPILOG telemetry
---

# WPILOG Decode

## Core Rules

- Use this skill when reading, listing, summarizing, or decoding `*.wpilog` files.
- Do not edit robot source to inspect telemetry. Decode existing logs with the scripts in this skill.
- Keep simulation launching and joystick/DriverStation control in `$wpilib-sim`; this skill owns log decoding after a log exists.
- Prefer decoded struct topics over adding duplicate scalar log outputs. For example, decode `struct:Rotation2d` instead of adding a separate degrees topic.
- If a repeatable WPILOG decoding workflow or missing common struct is discovered, update this skill and its decoder script.

## Scripts

- List topics:

    ```sh
    python3 .agents/skills/wpilog-decode/scripts/list_wpilog_topics.py \
      --repo . \
      --log <log.wpilog-or-SimLogs> \
      --filter <substring>
    ```

- Summarize selected topics:

    ```sh
    python3 .agents/skills/wpilog-decode/scripts/read_wpilog_values.py \
      --repo . \
      --log <log.wpilog-or-SimLogs> \
      --topic /RealOutputs/Swerve/Odometry/Robot \
      --topic /Swerve/Module0/SteerPosition
    ```

- Print JSON for scripts/comparisons:

    ```sh
    python3 .agents/skills/wpilog-decode/scripts/read_wpilog_values.py \
      --repo . \
      --log <log.wpilog> \
      --json \
      --samples 5 \
      --topic <topic>
    ```

- Find the newest sim log:

    ```sh
    python3 .agents/skills/wpilog-decode/scripts/find_latest_simlog.py .
    ```

- Find a boolean-gated window, such as autonomous enabled:

    ```sh
    python3 .agents/skills/wpilog-decode/scripts/find_wpilog_window.py \
      --repo . \
      --log <log.wpilog> \
      --all-true /DriverStation/Autonomous \
      --all-true /DriverStation/Enabled \
      --duration <seconds>
    ```

## Decoded Types

`read_wpilog_values.py` decodes primitives, primitive arrays, and common WPILib structs, including:

- `Rotation2d`, `Translation2d`, `Pose2d`, `Transform2d`, `Twist2d`
- `Translation3d`, `Rotation3d`, `Pose3d`, `Transform3d`, `Twist3d`
- `ChassisSpeeds`, `SwerveModuleState`, `SwerveModulePosition`

The decoder uses a hardcoded layout registry for common WPILib structs. Dynamic schema parsing is a future enhancement.

## Validation Pattern

- Confirm topic names/types first with `list_wpilog_topics.py`.
- Decode relevant topics with `read_wpilog_values.py`.
- Use `--samples N` when first/latest is not enough to understand movement over time.
- For before/after behavior checks, compare the same topics from both logs and prefer IO-layer applied voltage, velocity, position, current, command/reference topics, and decoded pose/rotation structs.
