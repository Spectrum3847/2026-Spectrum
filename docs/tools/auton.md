# Autonomous programming (auton)

*Audience: reference. Assumes you have read [2026 season specific](../other-guides/2026-season-specific.md).*

The first 20 seconds of a match run without driver control. You pick the routine before the match and the robot runs it alone. We use [PathPlanner](https://pathplanner.dev) to draw paths. The `pathplanner` crate in this repo follows them, and `Auton` joins paths and launch steps.

## Where the pieces live

* `robot/auton/auton.rs` has `Auton`: the chooser, the routine builders and the event triggers.
* `Swerve::configure_path_planner` in `robot/subsystems/swerve/swerve.rs` hands PathPlanner the pose, speed and drive plumbing, the translation and rotation PID constants and the red-alliance check.
* `bind_controls` in `robot/robot.rs` binds each event trigger to a super state.

The chooser shows on the dashboard as `Auto Chooser`. The **Pre-Match** tab in [Elastic](elastic.md) renders it. The default is `Do Nothing`.

Each routine has a Left and a Right option. One `.auto` file serves both. The Right option passes `mirrored = true`, which flips the poses across the field midline. PathPlanner flips paths for the red alliance on its own, because paths are drawn for blue.

## Timing and the console

`autonomous_init` calls `Auton::init`, which schedules the chosen command and stores the FPGA time. `autonomous_exit` calls `Auton::exit`, which prints how long the auto took, or that it was cancelled and at what second. Once the console scrolls, that line is the only record. The idea came from [team 6328](https://github.com/6328/MotorMatcher).

## Event markers

An event marker on a path starts a behavior. This avoids fixed waits, so timing does not drift when the robot accelerates differently.

The robot reacts to these markers. Each is an `EventTrigger` in `auton.rs`:

| Marker name | Bound in `bind_controls` to |
|---|---|
| `intake` | `WantedSuperState::AutonIntakeFuel` |
| `shotPrep` | `WantedSuperState::AutonTrackTarget` |
| `unjam` | `Unjam`, wait 1 second, then `LaunchWithSqueeze` |
| `clearState` | `WantedSuperState::Idle` |
| `poseUpdate` | Nothing. `Vision` reads it to decide when to fuse camera estimates. |

The paths in the repo use only `intake`, `shotPrep` and `poseUpdate` today. There is no `shoot` trigger. The shot between paths is the `launch()` step in the routine, not a marker.

To add a marker:

1. Put a marker with a name on the path in the PathPlanner app.
2. Add an `EventTrigger` with exactly that name in `auton.rs`, in the `thread_local!` block, with an accessor function.
3. Bind it in `bind_controls`.

A marker with no matching trigger does nothing and prints no error. A trigger with no binding does the same. If a marker does nothing, check the name (case counts, and a trailing space looks the same in the editor), then the binding.

## Path files

`src/main/deploy/pathplanner/` holds the paths, the `.auto` files, `navgrid.json` and `settings.json`. Two rules:

* The PathPlanner app writes JSON into that folder. Add it to the same change as the code that depends on it. Otherwise the next person gets a different auto from the same code.
* The robot reads `settings.json` at startup (`RobotConfig::from_gui_settings`) for mass, wheel radius, gearing, motor type and wheel friction. If you change the drivetrain, change `settings.json` too. If the file is missing or broken, the robot reports an error and drives no paths.

`cargo xtask deploy` copies the whole `deploy/` folder to the roboRIO. In simulation the robot reads the folder in the repo.

## Adding a routine

1. Draw the paths and `.auto` files in the PathPlanner app. Names with a number suffix (`TBTB 1`, `TBTB 2`) are the segments.
2. Add a builder on `Auton`. The existing ones are a `Commands::sequence` of `spectrum_auton(...)` calls with `self.launch()` between them.
3. Add both Left and Right options in `options()`.
4. Name each command with `full_name(...)`. That gives the ` - Left` or ` - Right` ending. `disabled_periodic` in `robot.rs` strips it to find the `.auto` file and mirror the `Field2d` preview. If you drop it, the auto still runs, but the preview shows the wrong path.
5. Run it in [simulation](simulation.md) and check the preview before you trust it.

## Test paths

The folder also holds `1 Meter`, `3 Meter` and `5 Meter` paths and autos for bench tests. The robot does not list them in the chooser.

## See also

[2026 season specific](../other-guides/2026-season-specific.md) for the state machine the auton drives. [PathPlanner](../dependencies/pathplanner.md) for the crate.
