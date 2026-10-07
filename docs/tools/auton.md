# Autonomous Programming (Auton)

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The first 15 seconds of a match are unattended; the robot runs whatever sequence was selected before the match started. For 2026 we use [PathPlanner](https://pathplanner.dev) for path-following and our own `Auton` class to stitch paths and shoot sequences together.

## How It's Wired

Most of the routine logic lives in one file: [`frc.robot.auton.Auton`](../../src/main/java/frc/robot/auton/Auton.java). The PathPlanner/`AutoBuilder` registration it relies on lives in [`Swerve.java`](../../src/main/java/frc/robot/subsystems/swerve/Swerve.java) (`configurePathPlanner()`), since that's where the pose/speeds suppliers and drive request live. `Auton` owns:

* A `SendableChooser<Command>` that publishes auto names to NetworkTables. Elastic's Pre-Match tab picks this up automatically.
* A handful of `EventTrigger`s with names that match the markers in the `.auto` files, `intake`, `shotPrep`, `shoot`, `clearState`, `unjam`, `poseUpdate`. When PathPlanner crosses one, the matching `Trigger` fires whatever command has been bound to it in [`Robot.java`](../../src/main/java/frc/robot/Robot.java) (e.g. `Auton.autonIntake.onTrue(...)`).
* Routine building blocks: `routine(fullAutoName, mirrored, steps...)` sequences steps and names the result; `SpectrumAuton(name, mirrored)` is one `.auto` file; `state(...)`, `holdState(...)` and `launch()` are state steps. `launch()` holds `WantedSuperState.AUTON_LAUNCH_WITH_SQUEEZE` for 2.5 seconds, then returns to `IDLE`.

`Robot.autonomousInit` calls `Auton.init()`, which schedules the selected command and starts an FPGA timer. `Robot.autonomousExit` calls `Auton.exit()` so the console shows how long the routine actually took, or that it was cancelled. The timer trick is borrowed from team 6328; it makes "did the auto finish in time" answerable at a glance.

If the dashboard sends a selection the chooser doesn't have (a stale name after an option is renamed), `getAutonomousCommand()` returns a print command instead of null.

## The Routine Catalog

Each chooser entry has a Left and Right variant; `mirrored = true` flips poses across the field's midline so the same `.auto` file works from both starting positions. Most are a single `.auto` file end to end, with the launches fired by event markers.

|         Chooser entry          |         `.auto` file         |                        Notes                        |
|--------------------------------|------------------------------|-----------------------------------------------------|
| Do Nothing                     | none                         | Default entry                                       |
| Double Swipe                   | `OSTBTB FULL`                |                                                     |
| Single Swipe with Depot        | `OSRIPPOFF FULL`             |                                                     |
| 2nd Double Swipe               | `2MANOSTBTB FULL`            | Waits 2 s first so we're out of an ally's way       |
| Center 1 Swipe                 | `OSCENT FULL`                | Keeps launching at a standstill after the path ends |
| Center to Depot                | `OSCENTOT FULL`              |                                                     |
| Single Swipe with Depot Cutoff | `OSRIPOFF CUTOFF`            |                                                     |
| Double Swipe 1 1/2             | `OSRIPOFF DOUBLE SWIPE FULL` |                                                     |

`routine(...)` names each one `"<fullAutoName> - Left"` / `" - Right"`. That suffix is significant: `Robot.disabledPeriodic` strips it to find the `.auto` file for the preview and start-pose check, and the field visualizer reads it to decide whether to mirror. Don't drop it.

## Paths and Autos in PathPlanner

PathPlanner stores its data in [`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner/):

* `paths/*.path`: single trajectories (waypoints, constraints, rotation targets).
* `autos/*.auto`: sequences of paths and named commands. `PathPlannerAuto("TBTB 1", mirrored)` loads `autos/TBTB 1.auto`.
* `navgrid.json`: the obstacle grid for the pathfinder.
* `settings.json`: robot kinematics PathPlanner uses for trajectory generation. Keep this in sync with the swerve constants.

`frcStaticFileDeploy` ships the whole `deploy/` tree to the roboRIO, so anyone connected to the bot has whatever PathPlanner state matches the deployed code. Editing a path in the PathPlanner app writes the JSON back into the repo; commit that alongside any code changes that depend on it.

## Event Markers

Every meaningful behavior during an auto routine fires from an event marker, not from a hand-coded `waitSeconds(...)`. There is no wait in front of the routine either: each `PathPlannerAuto` is built at boot, so PathPlanner has already generated and cached its trajectories, and the first path command starts driving in the first scheduler loop of auto. The one thing that can slow that down is a pose heading more than 30 deg from the path's starting heading, which makes PathPlanner regenerate the trajectory on the spot; the Pre-Match tab's **Pose Seed Confirmed** box (and the matching DS alert) says whether vision has settled the heading before the match starts.

The seed being confirmed says the measurement is trustworthy, not that it agrees with the auto. While disabled `Robot.checkStartPose()` compares the current pose with the selected auto's starting pose (already flipped for red and mirrored for a right start), shows the two differences on the Pre-Match tab as **Start Pose Err (m)** and **Start Heading Err (deg)**, and raises a DS error once they have exceeded 0.5 m or 10 deg for a second. That catches a wrong auto selection, a wrong side, an alliance that has not come through yet, and a bad vision seed, all of which look the same from the path's point of view: it drives to its own start point first. With no cameras the numbers read zero, because the same code placed the robot there. The flow is:

1. In PathPlanner, drop a marker on the path and name it (`intake`, `shotPrep`, `shoot`, …).
2. `Auton.java` declares a matching `public static final EventTrigger autonIntake = new EventTrigger("intake");` etc.
3. `Robot.java` binds those triggers to whatever super-state should fire, e.g. `Auton.autonIntake.onTrue(superStructure.setStateCommand(WantedSuperState.INTAKE_FUEL))`.

The advantage is the auto file stays declarative: "intake from here to here, then shoot", instead of hardcoding timings that drift the moment the robot accelerates differently.

## Adding a New Auto

1. Open PathPlanner, design the new path(s) and `.auto` file. Make sure event-marker names line up with the existing `EventTrigger` list in `Auton.java` (or add a new trigger and bind it in `Robot.configureBindings()`).
2. A one-file auto needs no method: use `single("YourAuto FULL", mirrored)`. For a multi-step one, add a method returning `routine("YourAuto FULL", mirrored, SpectrumAuton(...), launch(), SpectrumAuton(...))`. The first argument must be a `.auto` file covering the whole routine end to end, since the preview and start-pose check load it.
3. Register it in `setupSelectors()` with `pathChooser.addOption("YourAuto Left", ...(false))` plus the mirrored counterpart. `.auto` names are case-sensitive on the rio; a missing one raises a boot alert.
4. Test in sim first (`./gradlew simulateJava` → select the auto from Elastic's chooser). The `Field2d` preview will show the trajectory; verify the mirrored variant ends up where you expect.

## See Also

[2026 Season Specific](../other-guides/2026-season-specific.md) for the state machine that auton drives. [PathPlanner](../dependencies/pathplanner.md) for the dependency-level details, version, JavaDoc link, and which APIs we lean on.
