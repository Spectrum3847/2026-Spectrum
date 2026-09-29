# PathPlanner

*Audience: Reference. Assumes you've read [Dependencies overview](overview.md).*

PathPlanner is two pieces: a desktop editor that produces `.path` and `.auto` files, and an on robot library that reads them and drives the swerve. We use both for every autonomous routine. `vendordeps/` holds the pinned JSON.

Trajectories and autos live in [`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner/) and deploy to the roboRIO under `/home/lvuser/deploy/pathplanner/`.

## Where Java meets PathPlanner

Three places, and only three:

* `Swerve.configurePathPlanner()` in [`Swerve.java`](../../src/main/java/frc/robot/subsystems/swerve/Swerve.java) is private and runs from the Swerve constructor. It reseeds the pose to a merely plausible blue start, loads `RobotConfig.fromGUISettings()`, and hands `AutoBuilder` the pose, speed, and control plumbing.
* [`Auton.java`](../../src/main/java/frc/robot/auton/Auton.java) holds the `EventTrigger` constants, the `SendableChooser`, and the command builders that turn an `.auto` name into a command.
* `Robot.disabledInit` schedules the PathPlanner warmups once per session, guarded by the `autonWarmedUp` flag.

The translation and rotation `PIDConstants` in that `AutoBuilder` call get re tuned every year. Do not change them without coordinating with whoever owns auto tuning, because small shifts have outsized effects on whether a path lands on its end pose. The alliance lambda in the same call handles red: paths are authored against blue, so on red PathPlanner mirrors the whole path for you.

## Adding an auto step

1. Drop the event marker in the PathPlanner editor on the path that needs it.
2. Add a matching `public static final EventTrigger` in `Auton.java`. The marker name and the string passed to the constructor have to match exactly.
3. Bind it in `Robot.configureBindings()` to a super state, or to whatever command the step needs. The existing `Auton.autonIntake` style triggers are bound there, next to the driver bindings.

If a trigger never fires, check the marker name first, then check it for trailing whitespace. A trailing space looks identical in the editor and does not match the Java string.

## Mirroring and flipping

These are two different operations and the difference matters.

`Auton.SpectrumAuton(name, mirrored)` is the wrapper that turns an `.auto` name plus a mirror flag into a command, and each chooser option chains several of those together with a `launch()` step between segments. The `mirrored` flag is a within alliance mirror, which is how one `.auto` file becomes both the "Left" and "Right" options. The `withName` suffix on those options is also read by the field visualizer, so keep the " - Left" and " - Right" endings.

Red versus blue is the other operation, and it only happens automatically inside an `.auto` loaded by `PathPlannerAuto`. If you load a path straight from Java instead, nothing flips it. You are on the hook for the red flip and the mirror yourself.

## Warmup

`Robot.disabledInit` schedules `FollowPathCommand.warmupCommand()` and `PathfindingCommand.warmupCommand()`, guarded by `autonWarmedUp`, so the JIT compiles the hot paths before the first auto runs. If you add path loading code that only runs during matches, give it the same treatment. The first `PathPlannerPath.fromPathFile(...)` call is expensive.

## Loading a path from Java

`Auton.followSinglePath(pathName)` is the helper for a one off, such as a recovery routine after a failed pose update. It catches `FileVersionException`, `IOException`, and `ParseException` and returns a `PrintCommand` that says the path failed to load, so a missing or malformed file ends that command rather than killing the auto. `Auton.pathfindingCommandToPose(...)` does the equivalent job against a live pose. Neither currently has callers, so read them before assuming they are wired to something.

## Gotchas

The editor's `settings.json` shadows the robot constants. Its `robotMass`, `robotMOI`, `driveWheelRadius`, `driveGearing`, and `wheelCOF` are what the editor generates trajectories from, and nothing reads them back on the robot. If they drift from `SwerveConfig`, generated paths stop tracking, and the symptom is a bot that is off its line at the end of every path rather than anything the error log will tell you.

`settings.json` also declares a `pathFolders` and `autoFolders` list, but `paths/` and `autos/` are flat on disk. A path saved from the editor can land in one of those folders instead of next to the existing files. Check where a new `.path` ended up before committing it.

`navgrid.json` is editor only. It deploys to the roboRIO and nothing reads it there, but keep it under source control anyway so the editor opens cleanly for everyone.

## Provenance

`Auton.printAutoDuration()`, which reports how long an auto actually took and flags a cancelled run, is ported from team 6328.

## Further reading

[PathPlanner documentation](https://pathplanner.dev/home.html) covers the editor and the on robot library at a concept level, and the [JavaDoc](https://pathplanner.dev/api/java/) is linked into our generated docs. For how an auto runs end to end, see [Auton](../tools/auton.md).
