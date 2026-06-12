# Programming Tips

*Audience: Reference. Assumes you've read [2026 Season Specific](2026-season-specific.md).*

Practical things that come up often enough to be worth writing down.

## Clean Your Java Workspace

If VS Code is showing red squiggles on code that definitely compiles, or IntelliSense is behaving strangely, the language server's cache is probably stale. Open the Command Palette (`Ctrl+Shift+P`) and run `Java: Clean Language Server Workspace`. If that doesn't do it, run `./gradlew clean build` from the terminal — see [Gradle](../tools/gradle.md) for what that actually does.

## Coordinate Systems

FRC uses a field-relative coordinate system where positive X points toward the opposing alliance wall and positive Y points left from the driver's perspective. Robot heading is in radians measured counter-clockwise from the positive X axis. This matters when writing drive commands and when interpreting `Pose2d` values from the vision or auton systems.

If you're confused about which way something points, draw it. A quick sketch on a whiteboard with labeled axes has unblocked many programming sessions faster than any amount of reading.

## `DoubleSupplier` vs. `double`

Most command and trigger factory methods in this codebase take a `DoubleSupplier` instead of a raw `double`. The difference is that a `DoubleSupplier` is evaluated each time it's called, while a plain `double` is captured once when the command is scheduled.

For setpoints that may shift while a command runs — shooter speed that tracks a distance lookup, a hood angle that follows live vision data, or a value you're tuning with [`TuneValue`](../tools/pid-tuning.md#live-tuning-with-tunevalue) — you want the supplier. If you pass a bare `double`, the command freezes the value at scheduling time and never updates it.

The launcher shows this pattern:

```java
// LauncherStates.java
public static void idlePrep() {
    scheduleIfNotRunning(
            launcher.runVelocityTcFocRPM(config::getIdlingRPM).withName("Launcher.idlePrep"));
}
```

`config::getIdlingRPM` is a method reference — a `DoubleSupplier` — so if `idlingRPM` is changed at runtime (say, from a per-robot config override), the running command picks up the new value. More on this in [Class Generation](../coding-conventions/class-generation.md#methods).

## Cached Values

Every CAN read is a network call. If you call `motor.getPosition().getValueAsDouble()` three times in one loop from different parts of the code, you've made three CAN requests and gotten three (potentially different) readings back.

The pattern in `frc.spectrumLib` is to cache reads once per loop. The `Mechanism` base class already does this for position, velocity, voltage, and current using [`CachedDouble`](../../src/main/java/frc/spectrumLib/CachedDouble.java), which is a `SubsystemBase` that clears its cached flag in `periodic()` and recomputes on first access each loop.

If you're reading a sensor value that isn't already cached by `Mechanism`, do it in the subsystem's `periodic()` into a field, and have everything else read the field. Don't scatter CAN reads across command bodies.

## Method Chaining

`Command` and `Trigger` in WPILib return `this` from most modifier methods, so you can write:

```java
myTrigger.whileTrue(
    launcher.runVelocityTcFocRPM(config::getIdlingRPM)
        .andThen(launcher.stopMotor())
        .withTimeout(5.0));
```

This is idiomatic in the codebase — you'll see it everywhere in the `*States` files. Splitting across lines like above is fine; just keep the closing parenthesis aligned with the method call that opened it.

## Simulation Before Robot Time

Simulation catches the majority of logic bugs. State transitions, command sequencing, PathPlanner paths — most of it is testable without touching physical hardware. The full workflow is in [Simulation](../tools/simulation.md), but the short version: run `Ctrl+Shift+P → WPILib: Simulate Robot Code`, pick `GUI Sim`, and you get Glass plus a Field2d view.

Reserve time on the real robot for things that genuinely require it: tuning gains, calibrating offsets, testing hardware interactions. Don't develop new features on the robot.

## Feature Branches

Work on a feature branch, not directly on `main`. The typical flow:

1. Branch from `main` for your feature.
2. Develop and test in simulation.
3. Merge `main` back into your branch before opening a pull request, re-test in sim, then put it on the robot if needed.
4. Open a PR for review by a programming lead before merging.

Small, focused branches have fewer merge conflicts and are much easier to review than multi-week accumulations of changes. See [Commits and Pull Requests](../coding-conventions/commits-pull-requests.md) for commit message conventions.

## Logging is Free

Log more than you think you need to. A voltage reading, a command lifecycle event, a boolean state transition — these are nearly free to log and invaluable after a bad match. See [Logging](../tools/logging.md) for the `Telemetry` API. The pattern used in every `*States` file is:

```java
private static Command log(Command cmd) {
    return Telemetry.log(cmd);
}
```

Wrap command factories in `log(...)` and you automatically get init/end events in the log without changing any other code.
