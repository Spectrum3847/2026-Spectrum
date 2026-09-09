---
name: add-robot-state
description: "Add a new coordinated robot state and wire it into SuperStructure and Robot.configureBindings()."
---

Purpose
- Create a new high-level `WantedSuperState` and wire it into the robot's coordination layer.

Inputs (required):
- `STATE_NAME` — the new state name in `SCREAMING_SNAKE_CASE` (e.g., `MY_CUSTOM_STATE`).
- `behavior` — short description of how the state should behave, i.e. what `WantedState` each mechanism should be put into.
- `bindings` (optional) — the preferred input binding or auton trigger to use for this state.

Read first
- `CLAUDE.md` at the repo root for the state-machine pattern.
- `src/main/java/frc/robot/subsystems/SuperStructure.java` — the file you are editing.

Behavior
1. Add `STATE_NAME` to the `WantedSuperState` enum in `SuperStructure.java`.
2. Add the matching entry to the `CurrentSuperState` enum in the same file.
3. Add a `case` to `handleStateTransitions()` mapping the wanted state to the current state.
4. Add a `case` to `applyStates()` dispatching to a new `private void` method, and write that method — it should call `setWantedState(...)` on each mechanism the state touches. Every mechanism the state affects must be set explicitly; a mechanism left unset keeps whatever the previous state gave it.
5. If `bindings` are provided, bind the trigger in `Robot.configureBindings()` with `superStructure.setStateCommand(WantedSuperState.STATE_NAME)`. Gamepad `Trigger` fields themselves are declared in `pilot/Pilot.java` or `operator/Operator.java`.

Output format
- When making code changes, produce an `apply_patch`-style patch with minimal edits.

Constraints
- Do not modify generated files like `src/main/java/frc/robot/BuildConstants.java`.
- Keep changes narrowly scoped to the two enums, the transition table, the new apply method, and the binding.
- Do not create a `State.java`, `RobotStates.java`, `Coordinator.java`, or `*States.java` file — that was the 2025 layout and is not used here.
