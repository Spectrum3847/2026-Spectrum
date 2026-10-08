# Programming tips

*Audience: Reference. Assumes you've read [2026 season specific](2026-season-specific.md).*

Practical things that come up often enough to be worth writing down.

## Clean your Java workspace

If VS Code is showing red squiggles on code that definitely compiles, or IntelliSense is behaving strangely, the language server's cache is probably stale. Open the Command Palette (`Ctrl+Shift+P`) and run `Java: Clean Language Server Workspace`. If that doesn't do it, run `./gradlew clean build` from the terminal; see [Gradle](../tools/gradle.md) for what that actually does.

## Coordinate systems

FRC uses a field-relative coordinate system where positive X points toward the opposing alliance wall and positive Y points left from the driver's perspective. Robot heading is in radians measured counter-clockwise from the positive X axis. This matters when writing drive commands and when interpreting `Pose2d` values from the vision or auton systems.

If you're confused about which way something points, draw it. A quick sketch on a whiteboard with labeled axes has unblocked many programming sessions faster than any amount of reading.

## `DoubleSupplier` vs. `double`

Most command and trigger factory methods in this codebase take a `DoubleSupplier` instead of a raw `double`. The difference is that a `DoubleSupplier` is evaluated each time it's called, while a plain `double` is captured once when the command is scheduled.

For setpoints that may shift while a command runs, such as shooter speed that tracks a distance lookup, a hood angle that follows live vision data, or a value you're tuning with [`TuneValue`](../tools/pid-tuning.md#live-tuning-with-tunevalue), you want the supplier. If you pass a bare `double`, the command freezes the value at scheduling time and never updates it.

The launcher is the clearest example. Its `applyStates()` reads a wanted RPM out of `ShotCalculator` for the aim state, and sends it as the flywheel target. `Launcher.periodic()` calls `applyStates()` every loop, so the target is recomputed each time and the flywheel keeps following the live shot solution instead of freezing on the distance at the moment the state was entered. The control request itself reads its supplier once, when it is sent; the tracking comes from the loop calling it again.

More on the habit in [Class Generation](../coding-conventions/class-generation.md#methods).

## Cached values

Every CAN read is a network call. If you call `motor.getPosition().getValueAsDouble()` three times in one loop from different parts of the code, you've made three CAN requests and gotten three (potentially different) readings back.

The pattern in `frc.spectrumLib` is to cache reads once per loop. The `Mechanism` base class does this for every status signal it reads: the first getter call in a loop runs one `BaseStatusSignal.refreshAll`, and later calls in the same loop reuse that sample. The loop number comes from [`RobotLoop`](../../src/main/java/frc/spectrumLib/framework/RobotLoop.java), which `Robot.robotPeriodic()` advances first thing. Without that call the cache never refreshes and every reading freezes. `Limelight` works the same way, with `Vision.periodic()` calling `invalidate()` on each camera at the top of the loop.

If you're reading a sensor value that `Mechanism` does not already cache, read it once into a field and have every caller read the field. The mechanism's own `periodic()` is a good place for that refresh: every `Mechanism` registers itself with the scheduler in its constructor, so an override runs once per loop. The base `Mechanism.periodic()` is empty. Either way, don't scatter CAN reads across command bodies.

## Simulation before robot time

Simulation catches the majority of logic bugs. State transitions, command sequencing, PathPlanner paths, most of it is testable without touching physical hardware. The full workflow is in [Simulation](../tools/simulation.md), but the short version: run `Ctrl+Shift+P`, then `WPILib: Simulate Robot Code`, pick `GUI Sim`, and you get Glass plus a Field2d view.

Reserve time on the real robot for things that genuinely require it: tuning gains, calibrating offsets, testing hardware interactions. Don't develop new features on the robot.

## Before you commit

Branch from `main`, develop and test in sim, merge `main` back in, then open a PR. The commit and PR conventions, the shared-machine commit identity mechanism, and the one-feature-per-PR rule are all in [Commits and Pull Requests](../coding-conventions/commits-pull-requests.md). Don't keep a second copy of that here; it changes as the team changes and two copies means one of them is wrong.

For logging, the `Telemetry` API and how to pull signals back out of a `.wpilog` are in [Logging](../tools/logging.md). Log more than you think you need to, but read that page before inventing a pattern: there is already a convention for naming keys and for wrapping commands, and matching it is what makes a log readable six weeks later.
