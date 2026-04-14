---
name: add-robot-state
description: "Add a new robot state and wire it into State.java, RobotStates.java, and Coordinator.java."
---

Purpose
- Create a new high-level robot `State` constant and wire the state into the robot state machine.

Inputs (required):
- `STATE_NAME` — the new state name in `SCREAMING_SNAKE_CASE` (e.g., `MY_CUSTOM_STATE`).
- `behavior` — short description of how the state should behave or what subsystem it should activate.
- `bindings` (optional) — the preferred input binding or auton trigger to use for this state.

Behavior
1. Add the new constant to `src/main/java/frc/robot/State.java`.
2. Add or update state-handling logic in `src/main/java/frc/robot/Coordinator.java` to map the new state to subsystem commands.
3. Add robot state transition bindings in `src/main/java/frc/robot/RobotStates.java` for the new state, if an input or auton trigger is provided.
4. If `bindings` are provided, add a comment or placeholder trigger wiring in `RobotStates.java` and document the expected input source.

Output format
- When making code changes, produce an `apply_patch`-style patch with minimal edits.

Constraints
- Do not modify generated files like `src/main/java/frc/robot/BuildConstants.java`.
- Keep changes narrowly scoped to the state enum and coordinating wiring.
