# PathPlanner

*Audience: reference. Assumes you have read [Dependencies overview](overview.md).*

PathPlanner is two things: a desktop editor that writes `.path` and `.auto` files, and a library on the robot that reads them and drives the swerve. We use both for every autonomous routine. The `pathplanner` crate replaces PathPlannerLib. It ports **v2026.1.2** (MIT license, `rust/crates/pathplanner/LICENSE`).

Paths and autos are in `src/main/deploy/pathplanner/`. They deploy to `/home/lvuser/deploy/pathplanner/` on the roboRIO.

## What is ported

Paths, trajectory generation, the holonomic drive controller, `FollowPathCommand`, `PathfindingCommand` with AD* pathfinding, `AutoBuilder`, `PathPlannerAuto`, `NamedCommands`, event triggers and flipping.

What the robot does not use is not ported. The list is in the doc comment at the top of `crates/pathplanner/src/lib.rs`. It includes Choreo trajectories, `PathfindThenFollowPath`, dynamic obstacles, the auto trigger helpers (`isRunning`, `timeElapsed` and others) and the setpoint generator.

## Where the robot meets PathPlanner

Three places:

* `Swerve::configure_path_planner` in `robot/subsystems/swerve/swerve.rs` runs from `Swerve::new`. It moves the pose to a plausible blue start, loads `RobotConfig::from_gui_settings()`, and gives `AutoBuilder::configure` the pose, reset, speed and output closures. The output sends the speeds and per-wheel force feedforwards to the drivetrain. The translation and rotation `PidConstants` are in that call (4.0 and 3.0 now). They get tuned each year. Ask whoever owns auto tuning before you change them, because small changes move where a path ends.
* `robot/auton/auton.rs` holds the `EventTrigger`s, the `SendableChooser` and the builders that turn an `.auto` name into a command.
* `disabled_init` in `robot/robot.rs` schedules the warmup commands once per session, guarded by `auton_warmed_up`.

Paths are drawn for the blue alliance. The alliance closure passed to `AutoBuilder::configure` makes PathPlanner flip them for red.

## Adding an auto step

See [Auton](../tools/auton.md#event-markers). In short: marker in the editor, an `EventTrigger` with the same name in `auton.rs`, a binding in `bind_controls`. If a trigger never fires, check the marker name, including a trailing space that the editor does not show.

## Mirroring and flipping

These are two different things.

`spectrum_auton(name, mirrored)` in `auton.rs` makes a command from an `.auto` name and a mirror flag. A chooser option chains several of them, with `launch()` between. `mirrored` flips within one alliance, which is how one `.auto` file becomes both Left and Right. Keep the ` - Left` and ` - Right` endings on the names, because `disabled_periodic` reads them for the `Field2d` preview.

Blue to red flipping happens when an `.auto` loads through `PathPlannerAuto`. A path you load yourself is not flipped. Do the flip and mirror in your own code.

## Warmup

`disabled_init` schedules `FollowPathCommand::warmup_command()` and `PathfindingCommand::warmup_command()` once. The first use of path code is slow, so warm it up before the first match. If you add path-loading code that first runs in a match, warm it up the same way. `disabled_periodic` also loads the chosen auto's paths and runs the first one briefly while disabled.

## Gotchas

`settings.json` is read on the robot. `RobotConfig::from_gui_settings` loads it for robot mass, moment of inertia, wheel radius, gearing, max speed, motor type and wheel friction. The editor uses the same values to make trajectories. If they differ from the real robot, paths do not track. The symptom is a robot off its line at the end of every path, with nothing in the error log. If the file is missing, `configure_path_planner` reports an error and returns without configuring `AutoBuilder`.

`settings.json` lists `pathFolders` and `autoFolders`, but `paths/` and `autos/` are flat on disk. A path saved in the editor can land in a folder entry instead of next to the existing files. Check where a new `.path` ended up before you commit it.

`navgrid.json` is used by the pathfinding code. Keep it in the repo so the editor opens the same for everyone.

## Provenance

`Auton::print_auto_duration`, which reports how long an auto took, came from team 6328.

## Further reading

[PathPlanner documentation](https://pathplanner.dev/home.html) covers the editor and the library concepts. The Rust names are the Java names in snake_case, with the `PP` prefix written `Pp`. For how an auto runs from start to finish, see [Auton](../tools/auton.md).
