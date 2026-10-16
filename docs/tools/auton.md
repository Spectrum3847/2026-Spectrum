# Autonomous programming (auton)

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The first 15 seconds of a match are unattended. The robot runs whatever sequence was selected before the match started, so the cost of a bad auto is a whole match spent recovering. We use [PathPlanner](https://pathplanner.dev) for path following and our own `Auton` class to stitch paths and shoot sequences together.

## How the two halves divide

Path following is configured in [`Swerve.java`](../../src/main/java/frc/robot/subsystems/swerve/Swerve.java) under `configurePathPlanner()`, because that is where the pose and speed suppliers and the drive request live. Everything else is in [`frc.robot.auton.Auton`](../../src/main/java/frc/robot/auton/Auton.java): the chooser, the event triggers, and the helpers that build a routine out of path segments and state steps.

`Auton` publishes a `SendableChooser` to NetworkTables under `Auto Chooser`, which Elastic's Pre-Match tab picks up. The list of what is in it is `setupSelectors()`; that is the place to add an entry, and the place to read when you want to know what the robot can run.

`Robot.autonomousInit` calls `Auton.init()`, which schedules the selected command and starts an FPGA timer. `Robot.autonomousExit` calls `Auton.exit()`, which prints how long the routine actually took, or that it was cancelled. That timer is based on team 6328's code, credited in the comment on `Auton.exit()`. It is the thing that makes "did the auto finish in time" answerable at a glance, so keep it.

If the dashboard sends a selection the chooser does not have, which is what a stale name after an option is renamed looks like, `getAutonomousCommand()` returns a print command rather than null. You get a line in the console instead of a robot that does nothing with no explanation.

## The Left and Right naming convention

Every chooser entry has a Left and a Right variant. `mirrored = true` flips poses across the field's midline so one `.auto` file works from both starting positions, and `routine(...)` names the two entries with a `" - Left"` or `" - Right"` suffix.

That suffix is load-bearing, not decoration. `Robot.disabledPeriodic` strips it to find the `.auto` file for the trajectory preview and the start-pose check, and it decides the mirror from the same string. An entry named without the suffix will not preview and will not get its start pose checked. Keep it.

## Paths and autos in PathPlanner

PathPlanner's data lives in [`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner/): trajectories under `paths/`, auto sequences under `autos/`, plus `navgrid.json` and `settings.json`. `settings.json` carries the robot kinematics the pathfinder generates against, so if you change a swerve dimension you have to keep that in step or the generated trajectories are for a robot that does not exist.

`frcStaticFileDeploy` ships the whole `deploy/` tree to the roboRIO, so anyone connected to the bot gets whatever PathPlanner state matches the deployed code. Editing a path in the PathPlanner app writes the JSON back into the repo. Commit that alongside any code change that depends on it, or the next person to pull gets a path that does not match the robot.

## Event markers

Every meaningful behaviour during a routine fires from an event marker rather than a hand-coded `waitSeconds(...)`. A burst of fuel lasts a different length of time every match, so a timing that was right in practice stops being right the moment the robot accelerates differently.

There is no wait in front of the routine either. Each `PathPlannerAuto` is built at boot, so PathPlanner has already generated and cached its trajectories by the time the match starts, and the first path command drives in the first scheduler loop of auto. The one thing that can cost loops there is a seeded heading that disagrees with the path's starting heading, because PathPlanner then regenerates the trajectory on the spot. That is what the Pre-Match tab's **Pose Seed Confirmed** box is for.

Being confirmed says the measurement is trustworthy, not that it agrees with the auto. Those are different questions and the code answers them separately. While disabled, `Robot.checkStartPose()` compares the current pose against the selected auto's starting pose, already flipped for red and mirrored for a right start, puts both differences on the Pre-Match tab as **Start Pose Err (m)** and **Start Heading Err (deg)**, and raises a Driver Station error once they have exceeded their limits for a sustained second. Without a heading seed it refuses to answer at all rather than reporting a perfect zero, because unseeded the code is comparing the start pose against itself.

That check catches a wrong auto selection, a wrong side, an alliance that has not come through yet, and a bad vision seed. They all look identical from the path's point of view: the robot drives to its own start point first.

## Adding a new auto

1. Design the paths and the `.auto` file in the PathPlanner app. Marker names have to match an existing trigger or you have to add one.
2. A one-file auto needs no new method: pass its name to `single(...)`. A multi-step one gets a method that passes the steps to `routine(...)`. The first `.auto` file named has to cover the whole routine end to end, because the trajectory preview and the start-pose check both load it.
3. Register it in `setupSelectors()` with `addOption` for each side. The `.auto` name is case sensitive on the rio and the filesystem there is case sensitive where the Windows sim is not, so a mismatch passes everything you do on a laptop and then does nothing on the field. `Auton.verifyAutoFile` checks at boot and raises an alert, and `Robot.logAutoSelection` writes both the chosen name and whether the file was found to the log, which is the first thing to check when an auto sat still.
4. Test in sim first with `./gradlew simulateJava` and the auto selected from the chooser. The `Field2d` preview shows the trajectory. Run the mirrored variant too and confirm it ends up where you expect, not just that it exists.

## See also

[2026 Season Specific](../other-guides/2026-season-specific.md) for the state machine that auton drives. [PathPlanner](../dependencies/pathplanner.md) for the dependency-level details and version. [Shot Records and Trim Events](shot-log.md) for what the shot columns in a log mean once a routine has run.
