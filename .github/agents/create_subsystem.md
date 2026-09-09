---
name: create-subsystem
description: "Create a new Java subsystem skeleton following repository conventions (Mechanism + inner Config + state machine)."
---

Purpose
Create a minimal, buildable skeleton for a new subsystem named `{{SUBSYSTEM_NAME}}` in `frc.robot.subsystems`. Follow existing project conventions: a single file holding the mechanism class extending `frc.spectrumLib.mechanism.Mechanism`, its inner `*Config` class, and its `WantedState` / `SystemState` machine.

Inputs (replace or provide when invoking):
- `SUBSYSTEM_NAME` (CamelCase, e.g., `MyArm`)
- `packageFolder` (optional folder under `frc/robot/subsystems`, e.g., `arm`; default: lowerCamelCase of the name)

Read first
- `CLAUDE.md` at the repo root for the state-machine pattern and conventions.
- `src/main/java/frc/robot/subsystems/hood/Hood.java` — 232 lines, the reference implementation.

Steps
1. Read `Hood.java` as the pattern. `turret/Turret.java` and `fuelIntake/FuelIntake.java` are further examples.
2. Generate one new file: `src/main/java/frc/robot/subsystems/{{packageFolder}}/{{SUBSYSTEM_NAME}}.java`, containing:
   - `public class {{SUBSYSTEM_NAME}} extends Mechanism`
   - `public static class {{SUBSYSTEM_NAME}}Config extends Config` — every tunable as `@Getter private final`, applied in the constructor through the `config*()` helpers (`configPIDGains`, `configGearRatio`, `configSupplyCurrentLimit`, …). Call `super(name, canId, Rio.CANIVORE)` first.
   - `public enum WantedState` and `public enum SystemState`
   - `setWantedState(WantedState)`, `private SystemState handleStateTransition()`, `private void applyStates()`
   - a constructor taking the config, and a `periodic()` that calls `handleStateTransition()` then `applyStates()`
3. Optionally (only when `--wire` is provided), produce patches to add the config field to `Robot.Config`, the `@Getter private static` subsystem field to `Robot`, and its construction (with a `Timer.delay(canInitDelay)`) in the `Robot` constructor. If the new mechanism has to coordinate with others, add it to the `SuperStructure` constructor as well.

Output format (required): Produce an `apply_patch`-style patch only. The patch must add the new file and any optional wiring edits only when `--wire` is requested. Do not modify `BuildConstants.java`.

Constraints
- Keep changes minimal. Add TODO comments where human review is needed.
- Do not create a companion `*States.java` file — that was the 2025 layout and is not used here.
- Remember each per-robot config in `frc/robot/configs/` may need a `setAttached(...)` call for the new mechanism.
