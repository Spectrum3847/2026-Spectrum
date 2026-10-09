# PathPlannerLib

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

PathPlanner is two pieces: a desktop editor that produces `.path` and `.auto` files, and an on-robot library that reads them and drives the swerve. We use both for every autonomous routine. The version is pinned in `vendordeps/`.

The path and auto files live in [`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner/) and deploy to the roboRIO with the rest of `src/main/deploy`. On the Java side there are three entry points worth knowing: `Swerve.configurePathPlanner()` registers the drivetrain with `AutoBuilder` at boot, [`Auton`](../../src/main/java/frc/robot/auton/Auton.java) declares the event triggers and builds the chooser, and `Robot.disabledInit` schedules the PathPlanner warmups.

## Adding an auto step

Path event markers fire `EventTrigger`s. There are no `NamedCommands` in this codebase. A new auto step is three edits:

1. Drop the event marker on the path in the PathPlanner editor and save.
2. Add a matching `public static final EventTrigger` constant in `Auton.java`, using the same string.
3. Bind it in [`Robot.configureBindings()`](../../src/main/java/frc/robot/Robot.java) with `.onTrue(...)`, usually to a `setStateCommand(...)` for a `WantedSuperState`.

The marker name and the string in the constructor have to match exactly. If your trigger never fires, that is the first thing to check, along with trailing whitespace on the marker name. It looks identical in the editor and does not match in Java.

## The drivetrain controller gains are seasonal

`AutoBuilder.configure` in `Swerve.configurePathPlanner()` carries the translation and rotation `PIDConstants` for the whole robot. They get re-tuned every year, and a small shift has an outsized effect on whether a path lands on its end pose. Do not change them without coordinating with whoever owns auto tuning, and expect to re-tune them if the drivetrain changes underneath.

The same call also passes a `RobotConfig` read from the editor's `settings.json`, and an alliance supplier that decides whether a path needs flipping.

## Mirroring and flipping are different things

The chooser offers one `.auto` file as a left and a right variant, and that works because mirroring and flipping mean opposite things:

* `mirrorPath()` mirrors across the center line of the same alliance. It turns a left-side auto into a right-side one.
* `flipPath()` flips for red versus blue.

Inside an `.auto` file both are automatic, because the editor and the `AutoBuilder` alliance supplier do them for you. If you load a path directly from Java instead of going through a `PathPlannerAuto`, neither one happens on its own and you are responsible for both. `Robot` does this explicitly for the auto preview, which is a working example of the manual path.

## The chooser

`Auton.setupSelectors()` fills a `SendableChooser<Command>` and publishes it to SmartDashboard under `Auto Chooser`, so it shows up on the Elastic pre-match tab. The command for an option comes from `PathPlannerAuto`, with a `mirrored` flag for the left and right pair.

Two things to know before you add an option:

* Auto names in the chooser must match the `.auto` file names exactly, including case. The roboRIO's filesystem is case-sensitive and the Windows sim is not, so a mismatch passes locally and fails on the robot with the auto simply not starting. `Auton.verifyAutoFile` alerts at boot and `Robot.logAutoSelection` writes the outcome to the wpilog.
* `Auton.getAutonomousCommand()` returns a print command rather than null when the chooser has nothing selected, so a missing selection is visible on the Driver Station instead of throwing during the auto period.

## Warmup

`Robot.disabledInit` schedules `FollowPathCommand.warmupCommand()` and `PathfindingCommand.warmupCommand()` once per session, guarded by a flag, so the JIT has compiled the hot paths before the first auto runs. Anything new that is only loaded during a match, such as parsing a path file, deserves a warmup the same way. Path loading is heavy on its first call.

## Loading a path directly from Java

For a one-off, such as a recovery routine or a fallback after a failed pose update, read the file with the library's `PathPlannerPath.fromPathFile(...)`. It declares checked exceptions, so catch them specifically, report through `DriverStation` and `Telemetry`, and make sure a missing or malformed file cannot leave the auto silently doing nothing.

## Gotchas

The editor's `settings.json` shadows the robot constants. If the trajectories were generated against one robot mass or wheel friction and the real swerve assumes another, the paths will not track. Keep them aligned, and re-check `settings.json` after any change to the robot's physical mass or gearing.

`navgrid.json` is editor-only. It deploys to the roboRIO, but nothing on the robot reads it. Keep it under source control anyway so the editor opens cleanly for everyone.

## Further reading

[PathPlanner Documentation](https://pathplanner.dev/home.html) covers the editor and the on-robot library. The [JavaDoc](https://pathplanner.dev/api/java/) is cross-linked from our generated docs. For how an auto runs end to end from a human's point of view, see [Auton](../tools/auton.md).
