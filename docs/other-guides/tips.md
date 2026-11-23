# Programming tips

*Audience: reference. Assumes you have read [2026 season specific](2026-season-specific.md).*

Practical things that come up often enough to write down.

## Restart rust-analyzer

If VS Code shows red underlines on code that builds, or completion acts strangely, rust-analyzer's cache is probably stale. Open the Command Palette (`Ctrl+Shift+P`) and run `rust-analyzer: Restart server`. If that does not help, run `cargo build` from `rust/` and read the real compiler message. See [Build tools](../tools/build-tools.md).

## Read compiler errors from the top

The Rust compiler's first error is usually the real one. Later errors often come from it. Read the first message to the end. It often says what to change. `cargo clippy` gives the same kind of help for style and common mistakes.

## Coordinate systems

FRC uses a field coordinate system. Positive X points toward the opposing alliance wall. Positive Y points left from the driver's view. Robot heading is in radians, counter-clockwise from the positive X axis. This matters when you write drive code and when you read `Pose2d` values from vision or auton.

If you are unsure which way something points, draw it. A sketch with labeled axes unblocks more sessions than reading does.

## Closures instead of numbers

Many mechanism methods take a closure, `impl Fn() -> f64`, and not a plain `f64`. A closure runs each time it is called. A plain number is fixed when you pass it.

Use a closure for setpoints that can change while a command runs: a launcher speed that follows a distance lookup, a hood angle that follows live vision data, or a value you tune with `TuneValue` (see [PID tuning](../tools/pid-tuning.md#live-tuning-with-tunevalue)). If you pass a fixed number, the value freezes when you pass it, and a "live" value stays still.

The hood is a clear example. `Hood::apply_states` calls `output_for`, which asks the shot calculator for the aim angle every loop in the `AimAtTarget` state. `periodic` calls `apply_states` each loop, so the hood follows the live shot solution. A control request reads its closure once, when you send it. The tracking comes from the loop sending it again.

## Cached values

Every CAN read is a network call. If you read the same motor position three times in one loop from three places, you make three CAN requests and may get three different readings.

`Mechanism` caches the status signals it reads, so a repeated read in one loop reuses the first sample. The loop number comes from `robot_loop::next()` (`spectrum_lib/framework/robot_loop.rs`), which `robot_periodic` calls first thing. Without that call, the cache never refreshes and every reading freezes. `Limelight` works the same way. `Vision::periodic` calls `invalidate()` on each camera at the top of the loop. `CachedDouble` (`spectrum_lib/util/cached_double.rs`) caches any value once per scheduler loop.

For a sensor `Mechanism` does not cache, read it once in the subsystem's `periodic` and store it in a field. Do not scatter CAN reads across command bodies.

## Simulation before robot time

[Simulation](../tools/simulation.md) catches most logic bugs: state changes, command order, PathPlanner paths. Keep time on the real robot for what needs it: tuning gains, setting offsets and hardware tests. Do not develop new features on the robot.

## Before you commit

Branch from `main`, build and test in simulation, run `cargo xtask gate` from `rust/`, merge `main` into your branch, then open a pull request. The commit rules, the shared-computer commit identity and one-feature-per-PR are in [Commits and pull requests](../coding-conventions/commits-pull-requests.md). That page is the one copy. Do not repeat it here.

For logging, the `telemetry` functions and how to read a `.wpilog` are in [Logging](../tools/logging.md). Log more than you think you need, but read that page first. There is already a convention for key names, and following it makes a log readable six weeks later.
