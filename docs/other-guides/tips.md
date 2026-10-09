# Programming tips

*Audience: Reference. Assumes you've read [2026 Season Specific](2026-season-specific.md).*

Practical habits that come up often enough to be worth writing down. For how the code is put
together, read the code.

## Clean your Java workspace

If VS Code is showing red squiggles on code that definitely compiles, or the suggestions have gone
strange, the language server's cache is probably stale. Open the Command Palette with
`Ctrl+Shift+P` and run `Java: Clean Language Server Workspace`. If that does not do it, run
`./gradlew clean build` from the terminal. See [Gradle](../tools/gradle.md) for what that actually
does.

## Field coordinates

FRC uses a field-relative coordinate system. Positive X points toward the opposing alliance wall,
and positive Y points left from the driver's perspective. Robot heading is in radians, measured
counter-clockwise from the positive X axis. This is the convention WPILib uses everywhere, so it
is worth being fluent in before you write a drive command or read a `Pose2d` value.

When you are not sure which way something points, draw it. A whiteboard sketch with labeled axes
has unblocked more sessions than any amount of reading.

## `DoubleSupplier` vs. `double`

Most of the command and trigger factory methods here take a `DoubleSupplier` rather than a plain
`double`. The difference is when the value is read. A `DoubleSupplier` is asked for its value every
time something needs it. A `double` is a single number, copied when you pass it, and it never
changes afterwards.

For a setpoint that might move while a command is running, you want the supplier. That covers a
speed that follows a distance lookup, an angle that follows live sensor data, or anything an
operator can change. Pass a bare `double` instead and the command is stuck with whatever the
number was at the moment it was scheduled.

```java
// The launcher, in its LAUNCH case.
commandedRPM = ShotCalculator.getInstance().getParameters().flywheelSpeed();
setVelocityRPM(() -> commandedRPM);
```

`setVelocityRPM` takes a `DoubleSupplier`. The `() -> commandedRPM` is that supplier, and it is
re-read by the control request every loop. So as the shot calculator's target moves with the
distance, the motor follows it. Writing `setVelocityRPM(commandedRPM)` would compile, and the
launcher would sit at one fixed speed for the whole match.

More on this in [Class Generation](../coding-conventions/class-generation.md#methods).

## Cached values

Every read from a motor is a network call. If you ask the same motor for the same value three times
in one loop, from three different places, that is three requests and potentially three different
answers.

The pattern in this repo is to read once per loop and share the result. `Mechanism` sets this up
for every motor it owns: it collects all of a motor's status signals and refreshes them with a
single Phoenix call, keyed on the robot loop counter, so every getter in a loop sees the same
sample and the whole robot makes one call per mechanism rather than one per signal.

`CachedDouble` does the same job for any value that is not already covered, by wrapping a
`DoubleSupplier` so it runs at most once per scheduler iteration. It is a `SubsystemBase` whose
`periodic()` clears the cached flag, which is what makes the cache safe: the scheduler polls
triggers before it calls subsystem `periodic()`, so a trigger reading the cache always sees the
current loop's value.

If you are reading a sensor that nothing already caches, read it once into a field and have
everything else read the field. Do not scatter reads across command bodies. There is more on why
this matters in
[Loop Time and CPU Handoff 2026-09-05](loop-time-handoff-2026-09-05.md).

## Start in simulation

Simulation catches most logic bugs. State transitions, command sequencing, path following, all of
it is testable before the robot is involved. See [Simulation](../tools/simulation.md) for the full
workflow. The short version is `Ctrl+Shift+P`, then `WPILib: Simulate Robot Code`, then pick
**GUI Sim**, which gives you the driver station emulator and a field view.

Save robot time for the things that genuinely need the robot: tuning gains, calibrating offsets,
checking hardware interactions. Do not build new features on the robot.

## Other pages

* [Commits and Pull Requests](../coding-conventions/commits-pull-requests.md): branching, commit messages, and pull requests.
* [Logging and Data Analysis](../tools/logging.md): the telemetry API, the log tiers, and reading the log files.
* [Loop Time and CPU Handoff 2026-09-05](loop-time-handoff-2026-09-05.md): what a busy loop costs, measured.
