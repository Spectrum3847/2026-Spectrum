---
name: create-subsystem
description: "Create a new Java subsystem skeleton following repository conventions (Mechanism + States + registration)."
---

Purpose
Create a minimal, buildable skeleton for a new subsystem named `{{SUBSYSTEM_NAME}}` in `frc.robot`. Follow existing project conventions (mechanism class extending `frc.spectrumLib.mechanism.Mechanism`, an accompanying `*States` class, and wiring in `Robot.java`/`Coordinator.java` if requested).

Inputs (replace or provide when invoking):
- `SUBSYSTEM_NAME` (CamelCase, e.g., `MyArm`)
- `packageFolder` (optional folder under `frc/robot`, e.g., `arm`; default: lowercase of the name)

Steps
1. Find an existing subsystem to use as a pattern (examples: `fuelIntake`, `launcher`, `indexerTower`). Inspect one Mechanism class and its States file.
2. Generate two new files:
   - `src/main/java/frc/robot/{{packageFolder}}/{{SUBSYSTEM_NAME}}.java` — Mechanism subclass skeleton with inner `Config` class and constructor that accepts a `Config` instance.
   - `src/main/java/frc/robot/{{packageFolder}}/{{SUBSYSTEM_NAME}}States.java` — States holder with static methods to map Robot `State` values to commands for this subsystem.
3. Optionally (only when `--wire` flag is provided), produce patches to register the subsystem in `Robot.java` (field + getter) and add a mapping in `Coordinator.java` where appropriate.

Output format (required): Produce an `apply_patch`-style patch only. The patch must add the two new files and any optional edits (Robot/Coordinator) only when `--wire` is requested. Do not modify `BuildConstants.java`.

Skeleton requirements (follow style in repo):
- Use package `frc.robot.<packageFolder>`.
- Class extends `frc.spectrumLib.mechanism.Mechanism` (or `Mechanism`) and includes a public static `Config` inner class with tunable fields.
- Add a minimal constructor and a `periodic()` override (if appropriate).

Example instruction (caller):
"Create subsystem `ClimberArm` in folder `climber` and wire it into Robot (use `--wire`)."

Constraints
- Keep changes minimal. Prefer adding files and small, explicit edits to registration points. Add TODO comments where human review is needed.
