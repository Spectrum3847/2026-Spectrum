# Autonomous programming (auton)

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The first twenty seconds of a match run without driver control, so the routine is picked before the match and the robot runs it on its own. We use [PathPlanner](https://pathplanner.dev) for path following and our own `Auton` class to stitch paths and launch sequences together.

## Where the pieces live

[`Auton`](../../src/main/java/frc/robot/auton/Auton.java) owns the chooser and the command factories. The `AutoBuilder` registration lives in [`Swerve`](../../src/main/java/frc/robot/subsystems/swerve/Swerve.java), because that is where the pose and speed suppliers come from. Trigger bindings live in `Robot.configureBindings()`.

The chooser publishes to the dashboard as `Auto Chooser`, which the **Pre-Match** tab in [Elastic](elastic.md) renders. Every routine has a Left and a Right variant served by a single `.auto` file, with a `mirrored` flag flipping the poses across the field midline.

Read `Auton.java` for the current routine list and for the two group delays it applies before the first path. One of those delays exists so we are not in an ally's way on the first move, the other so a partner has time to clear a lane. The chooser defaults to `Do Nothing`, so a bot with no selection still does something legal instead of nothing visible.

## Timing and the console

`Robot.autonomousInit` calls `Auton.init()`, which schedules the selection and stamps the FPGA clock. `Robot.autonomousExit` calls `Auton.exit()`, which prints either how long the auto took or that it was cancelled and at what second. The timer came from [team 6328](https://github.com/6328/MotorMatcher). It is the fastest way to answer "did we finish" after a match, and once the console scrolls it is the only record.

## Event markers

Behaviors fire from PathPlanner event markers rather than hardcoded waits, so the routine stays declarative and the timings do not drift when the robot accelerates differently.

Adding one:

1. Drop a marker on the path in the PathPlanner app and give it a name. It has to match an `EventTrigger` in `Auton.java` exactly, including case. A marker with no matching trigger fails silently.
2. Bind the trigger in `Robot.configureBindings()`. A trigger with no binding also fails silently, so a marker that does nothing is usually a missing binding rather than a bad name.

Two triggers are not what they look like. `autonPoseUpdate` has no binding, because `Vision` reads it to decide when to feed camera estimates to the pose estimator. `autonShoot` is declared and has neither a binding nor a reader, so a `shoot` marker in a path does nothing today.

## Path files

[`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner/) holds the trajectories, the auto sequences that chain them, the nav grid the on-the-fly planner avoids obstacles with, and the robot geometry the PathPlanner app generates trajectories against.

Two things that cost an afternoon if forgotten:

* Editing a path in the PathPlanner app writes JSON back into that tree. Commit it alongside the code change that depends on it, or the next person gets a different auto from the same code.
* Trajectories are generated from the robot geometry in `settings.json`. If you move a module in `SwerveConfig`, update `settings.json` too, or the paths are generated for a drivetrain you no longer have.

`./gradlew deploy` ships the whole `deploy/` tree, so anyone who plugs into the bot gets the paths that match the code.

## Adding a routine

1. Design the path and the `.auto` in the PathPlanner app. Keep marker names aligned with the triggers already in `Auton.java`, and add a new `EventTrigger` there if you need a new name.
2. Add a factory on `Auton` returning a `Command`. The existing routines are a `Commands.sequence` of `SpectrumAuton(...)` calls with `launch()` between them, and `launch()` is what gets the robot to shoot between paths.
3. Register both variants in `setupSelectors()`.
4. Keep the `"... - Left"` or `"... - Right"` suffix on the command name. `Robot.disabledPeriodic` strips that suffix to recover the base path name, and uses it to mirror the `Field2d` preview for a Right start. Drop it and the auto still runs correctly while the field preview silently shows the wrong trajectory.
5. Test in sim with `./gradlew simulateJava` and pick the auto from the chooser. Confirm the preview before you trust it.

## Helpers for bench testing

`Auton.followSinglePath(name)` runs a single `.path` without an `.auto` wrapper, which is handy for a one-off scripted move. `pathfindingCommandToPose(x, y, rot, vel, accel)` plans to a pose on the fly against the nav grid. The on-the-fly planner is slower than a pre-baked path, so do not build a match routine on it.

## See also

[2026 Season Specific](../other-guides/2026-season-specific.md) for the state machine the auton drives. [PathPlanner](../dependencies/pathplanner.md) for the dependency itself, its version, and the APIs we lean on.
