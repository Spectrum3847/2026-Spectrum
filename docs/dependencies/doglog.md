# DogLog

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

DogLog is the logger that writes our WPILOG files. We do not call it directly. Everything goes through [`frc.spectrumLib.telemetry.Telemetry`](../../src/main/java/frc/spectrumLib/telemetry/Telemetry.java), which extends `DogLog`, implements `Subsystem` so the scheduler runs its per-loop housekeeping, and is the layer essentially all of the robot code calls.

The version is pinned in `vendordeps/`.

## Telemetry is a static facade on purpose

Every entry point on `Telemetry` is static, and no instance methods may be added to it. The class is the single vocabulary for logging; the moment half the codebase calls it one way and the other half calls it the other way, both spellings are wrong forever and the value of having one facade is gone. If you need per-instance state, put it in a separate class that logs through `Telemetry`.

## The start options are a decision, not defaults

`Robot`'s constructor calls `Telemetry.start(...)` once, with a set of flags chosen for a competition robot rather than left at their defaults. Read the call and its comments before changing it, because the two flags that are off are off on purpose: the NetworkTables mirror is off on the robot, and the extra logging categories are off. On the robot we log dashboard keys individually instead; in simulation the mirror is on, which is why every logged key shows up live in AdvantageScope when you sim.

`Telemetry.start` also hands DogLog a `PowerDistribution` instance, so PDH currents land in the log without anyone writing a line for them.

## Logging values

Keys are `Subsystem/Path/Name`, so Elastic and AdvantageScope render them as a tree. Before inventing a new top-level key, look at what already exists in the code and reuse its prefix. Drift here is what makes a log unsearchable a season later.

`Telemetry.log` has overloads for the primitive types, arrays, and WPILib structs, so in practice you just call it and pass the value. Add units to the value where there are any; the logger uses the third argument for exactly that.

## Logging commands

Wrapping a command with `Telemetry.log(...)` records when it was initialized and when it ended under the `Commands` key, which is the fastest way to answer "did that command ever run". Wrap only the outermost command in a group. Wrapping inner ones just produces log spam and no extra information.

## Console output

`Telemetry.print(...)` stamps the time, decides whether to print to the console based on the priority, and logs the line under `Prints` so it survives in the WPILOG. Use it instead of `System.out.println`. A bare print disappears when console capture is off, and it has no timestamp, so two identical messages a match apart are indistinguishable.

There are two priorities. `HIGH` always reaches the console. `NORMAL` reaches the console only when the global priority is also `NORMAL`, which is the setting `Robot` uses in a match. Both always land in the log either way.

## Alerts

`Telemetry` scrapes active `Alert` objects every loop and mirrors anything new into the `Alerts` log key, so the ordinary WPILib pattern of declaring an alert as a static field and flipping it with `set(boolean)` gives you the dashboard, the log, and no plumbing. See [WPILib](wpilib.md).

## Build stamps

`Robot.robotInit` writes a set of `BuildConstants/...` keys into every log. Those come from `BuildConstants.java`, which the `gversion` Gradle task regenerates on every `compileJava`. When you pull up a log a week later trying to work out why the robot misbehaved, those keys are how you tell which build produced it. Never edit that file by hand and never commit it; it is generated.

## Things that have bitten us

Mirroring every logged value to NetworkTables, with a flush per loop, was a full-time job for one of the roboRIO's two cores on 2026-09-05. That is why the mirror is off on the robot. Individual dashboard values still go out through `Telemetry.logDash`, and a `Telemetry/MirrorLogsToNT` switch on SmartDashboard turns the full mirror back on for an AdvantageScope session in the shop. The switch is ignored whenever the FMS is attached, so nobody can flip it mid-match.

`withNtTunables` controls DogLog's own tunable entries. It does not affect `TuneValue` or `SmartDashboard` writes. If you need to prevent match-day tuning, guard the call sites separately.

`Telemetry.Fault` is the shared vocabulary for conditions worth naming rather than describing in free text. Add an entry when you notice yourself logging the same fault from more than one file; the enum is in the source, so read it there for what it currently holds.

## Further reading

The [DogLog JavaDoc](https://javadoc.doglog.dev) is cross-linked from our generated docs, and the [README](https://github.com/jonahsnider/doglog) covers the feature flags we have not touched. For what to log and when, see [Logging](../tools/logging.md).
