# Logging and data analysis

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

A log is the difference between "the indexer stopped at champs" and "the indexer stopped because the CANivore dropped the bus at 2.14 seconds." Everything goes through [`Telemetry`](../../src/main/java/frc/spectrumLib/telemetry/Telemetry.java), a thin wrapper over [DogLog](https://doglog.dev) that keeps call sites short. The library itself is covered on the [DogLog](../dependencies/doglog.md) page.

## What ends up in the file

`Telemetry.start(...)`, called from the `Robot` constructor, sets the whole policy in one place. Seven flags, in the order they are passed:

* `ntMirror` is the starting position of the SmartDashboard switch `Telemetry/MirrorLogsToNT`, which mirrors every logged value to NetworkTables so AdvantageScope can see it live. It starts on in simulation and off on the robot, to save roboRIO CPU. Flip it in Elastic when AdvantageScope needs the full live stream in the shop. It is forced off whenever the FMS is attached. The dashboard gets its values another way, see [What reaches the dashboard](#what-reaches-the-dashboard).
* `captureDs` records SmartDashboard entries in the log, so a value published the old way is still recoverable from the file afterwards. On.
* `captureNt` records the entire NetworkTables tree. Off, because it swamps the log and duplicates what the code already logs. Turn it on only when you specifically need a topic that nothing in our code logs.
* `captureConsole` folds console output into the log, which is how bare prints from CTRE and WPILib survive. On.
* `logExtras` adds PDH currents, CAN bus usage, and radio status. Off by default because it is a lot of data for a diagnostic you rarely run. Worth turning on for a brownout investigation.
* `tunableOnFMS` controls DogLog's own NetworkTables tunables, and it is on, so tunables stay editable with an FMS attached. That is a practice convenience and a match-day hazard.
* `priority` is the lowest priority that still reaches the console.

`Telemetry.start` also hands DogLog a `PowerDistribution` and puts the command scheduler on the dashboard. Leave that scheduler entry alone, it is the fastest way to see what is actually scheduled when something looks stuck.

## Naming keys

Keys are `Subsystem/Name` and they are the file's table of contents. Two rules:

* Lead with the subsystem's own name. `/Launcher/RPM` is findable, `/motorSpeed7` is not.
* Pass the unit as the third argument when there is one. DogLog records it as metadata and AdvantageScope uses it for axis labels, so the plot reads `volts` instead of a bare number.

Log inside `periodic()`. That is the one place you know a value is fresh at loop rate, and it keeps log calls out of command bodies where they get forgotten. Do not log the same value from two places under two names; a season of that makes the logs unsearchable and nothing fixes it retroactively.

## What reaches the dashboard

While the mirror is off, a value is on NetworkTables only if the code publishes it on purpose. `Telemetry` has three tiers:

|                 call                  |                   wpilog                   |      NetworkTables       |                                   use for                                   |
|---------------------------------------|--------------------------------------------|--------------------------|-----------------------------------------------------------------------------|
| `Telemetry.log(key, value)`           | every call (DogLog skips unchanged values) | only through the mirror  | everything                                                                  |
| `Telemetry.logDash(key, value)`       | every call                                 | every fifth loop (10 Hz) | keys the Elastic layout shows                                               |
| `Telemetry.logDashAlways(key, value)` | every call                                 | every call               | dashboard keys logged once or on their own cadence, like `BuildConstants/*` |

`Telemetry.slowLogThisLoop()` is true on the same every-fifth loop; wrap anything that does not need 20 ms resolution in it. The Elastic layout in `src/main/deploy/elastic-layout.json` is the list of keys that have to stay `logDash`.

## Command lifecycle

`Telemetry.log(cmd)` wraps a command so its start and end land in the `Commands` key with the command's own name. Wrap the outermost command in a group, not every step inside it, or the log fills with internal sequence steps.

Every subsystem already logs its running command name and its wanted and current state each loop from its own `periodic()`, so you can see what owns a mechanism without decorating anything. Look for `<Subsystem>/CurrentCommand` in a log rather than adding more wrappers.

## Console prints

`Telemetry.print(...)` timestamps the line, decides whether it reaches the console, and writes it to the `Prints` key so it survives in the file either way. `HIGH` priority always prints. `NORMAL` prints only while the global priority is `NORMAL`, which is what the robot sets, so you can mark a fault loud without turning on console spam for everything else.

Use prints for initialization milestones and faults. Anything you would want to graph belongs in `log`, not `print`. The exception-handling convention is in [Exception Handling](../coding-conventions/exception-handling.md).

## Alerts

`Telemetry.logAlerts()` runs every loop and copies anything new from `SmartDashboard/Alerts` into the log, deduplicated so a flapping alert does not fill the disk. An ordinary WPILib `Alert` shows on the dashboard and lands in the file with no extra plumbing.

## Faults

`Telemetry.Fault` is an enum of named failure modes kept so the codebase has a shared vocabulary instead of free-text strings. There is no `logFault(...)` helper, so a fault is logged as a high-priority print using the enum's name. Add entries as new failure classes show up; the post-match grep is much faster than scanning prose.

## Which build wrote this

The robot stamps its git branch, commit, dirty flag, and build date into the log at startup. A log that does not say which build produced it cannot be compared to anything.

## Tunables

`TuneValue` writes to `SmartDashboard`, so it stays editable when the FMS is attached no matter what the `tunableOnFMS` flag says. If a value must not move during a match, guard or remove the `TuneValue` at the call site. The FMS will not stop it for you.

## Getting logs off the roboRIO

`.wpilog` files are written under `/U/logs/` on the roboRIO. Connect over USB or Ethernet, then:

1. Open the file straight off the robot in [AdvantageScope](https://docs.advantagescope.org) with **File, then Open Log**, or download it first and open the copy.
2. Drag entries from the log's own tree into a plot. AdvantageScope carries the layout you had in the live view into the recorded file, so a plot you set up while watching a mechanism is already set up for the analysis.

For a live session, start AdvantageScope before connecting the driver station and point it at the same robot. It reads NetworkTables directly, so it sees everything `Telemetry` publishes.

## What to log

Motor voltages and currents, sensor readings, setpoints, state transitions, vision estimates, command lifecycle, and anything else you would want to graph the morning after.

Do not log the same value on every iteration of a tight inner loop. DogLog will accept it and the disk will not thank you.

## See also

* [DogLog](../dependencies/doglog.md) for the library, its version, and the option flags in full.
* [Elastic Dashboard](elastic.md), the live view of the same publish stream.
* [Exception Handling](../coding-conventions/exception-handling.md) for when to print at high priority.
