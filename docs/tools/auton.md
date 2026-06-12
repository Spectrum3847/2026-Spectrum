# Autonomous Programming (Auton)

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The first 15 seconds of a match are unattended — the robot runs whatever sequence was selected before the match started. For 2026 we use [PathPlanner](https://pathplanner.dev) for path-following and our own `Auton` class to stitch paths and shoot sequences together.

## How It's Wired

The whole subsystem fits in one file: [`frc.robot.auton.Auton`](../../src/main/java/frc/robot/auton/Auton.java). It owns:

* A `SendableChooser<Command>` that publishes auto names to NetworkTables. Elastic's Pre-Match tab picks this up automatically.
* A handful of `EventTrigger`s with names that match the markers in the `.auto` files — `intake`, `shotPrep`, `shoot`, `clearState`, `unjam`, `poseUpdate`. When PathPlanner crosses one, the matching `Trigger` fires whatever command [`RobotStates`](../../src/main/java/frc/robot/RobotStates.java) has bound to it.
* The `launch()` command, which sets `LAUNCH_WITH_SQUEEZE` for 2.5 seconds while `SwerveStates.autonAimAtTarget()` holds heading on the target. Every auto chains some number of `SpectrumAuton(...)` segments together with `launch()` between them.

`Robot.autonomousInit` calls `Auton.init()`, which schedules the selected command and starts an FPGA timer. `Robot.autonomousExit` calls `printAutoDuration()` so the console shows how long the routine actually took (or how much it had left when teleop took over). The timer trick is borrowed from team 6328 — it makes "did the auto finish in time" answerable at a glance.

## The Routine Catalog

Every entry in the chooser is just a sequence of `SpectrumAuton(pathName, mirrored)` calls glued together with `launch()`. The naming pattern is `<scoring sequence>` where each letter is a goal column (T/B/D). `TBTB`, for example, runs Top → Bottom → Top → Bottom. Each routine has a Left and Right variant — `mirrored = true` flips poses across the field's midline so the same `.auto` file works from both starting positions.

| Group | Entries | Notes |
| --- | --- | --- |
| Headliners | `TBTB`, `TBTT`, `TTTT`, `BBBB` | Four-shot routines, all chained `SpectrumAuton + launch` |
| Optional | `Option TBT`, `Option BBB` | Insert an `OPTIONAL_DELAY` (1.0 s) after the first segment — used when a partner needs the lane to clear |
| 2nd Man | `2nd-TBTB`, `2nd-BBD` | Begin with `SECOND_MAN_DELAY` (1.0 s) so we're not in the way of an ally's first move |
| Fallback | `Do Nothing` | Default chooser entry — never let a missing selection mean an unscheduled robot |

The `withName(...)` suffix `" - Left"` / `" - Right"` is significant: the field visualizer parses the suffix to decide whether to mirror the rendered pose. Don't drop it.

## Paths and Autos in PathPlanner

PathPlanner stores its data in [`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner/):

* `paths/*.path` — single trajectories (waypoints, constraints, rotation targets).
* `autos/*.auto` — sequences of paths and named commands. `PathPlannerAuto("TBTB 1", mirrored)` loads `autos/TBTB 1.auto`.
* `navgrid.json` — the obstacle grid for the pathfinder.
* `settings.json` — robot kinematics PathPlanner uses for trajectory generation. Keep this in sync with the swerve constants.

`frcStaticFileDeploy` ships the whole `deploy/` tree to the roboRIO, so anyone connected to the bot has whatever PathPlanner state matches the deployed code. Editing a path in the PathPlanner app writes the JSON back into the repo; commit that alongside any code changes that depend on it.

## Event Markers

Every meaningful behavior during an auto routine fires from an event marker, not from a hand-coded `waitSeconds(...)`. The flow is:

1. In PathPlanner, drop a marker on the path and name it (`intake`, `shotPrep`, `shoot`, …).
2. `Auton.java` declares a matching `public static final EventTrigger autonIntake = new EventTrigger("intake");` etc.
3. `RobotStates.setupStates()` binds those triggers to whatever robot state should fire — `INTAKE_FUEL`, `TRACK_TARGET`, etc.

The advantage is the auto file stays declarative — "intake from here to here, then shoot" — instead of hardcoding timings that drift the moment the robot accelerates differently.

## Adding a New Auto

1. Open PathPlanner, design the new path(s) and `.auto` file. Make sure event-marker names line up with the existing `EventTrigger` list in `Auton.java` (or add a new trigger and bind it in `RobotStates`).
2. Add a method on `Auton` that returns a `Command`. Follow the existing pattern: `Commands.sequence(SpectrumAuton(...), launch(), SpectrumAuton(...))` and tag it with `.withName("YourAuto Full - " + (mirrored ? "Right" : "Left"))`.
3. Register it in `setupSelectors()` with `pathChooser.addOption("YourAuto Left", yourAuto(false))` plus the mirrored counterpart.
4. Test in sim first (`./gradlew simulateJava` → select the auto from Elastic's chooser). The `Field2d` preview will show the trajectory; verify the mirrored variant ends up where you expect.

## Useful Helpers

* `followSinglePath(name)` runs a single PathPlanner `.path` outside of an `.auto` wrapper. Handy for one-off scripted moves, less useful in competition.
* `pathfindingCommandToPose(x, y, rot, vel, accel)` invokes `AutoBuilder.pathfindToPoseFlipped(...)`, which navigates to a pose using `navgrid.json` for obstacle avoidance. Used during testing more than matches — the on-the-fly planner is slower than running a pre-baked path.

## See Also

[2026 Season Specific](../other-guides/2026-season-specific.md) for the state machine that auton drives. [PathPlanner](../dependencies/pathplanner.md) for the dependency-level details — version, JavaDoc link, and which APIs we lean on.
