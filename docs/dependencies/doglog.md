# DogLog

*Audience: Reference. Assumes you've read [Dependencies overview](overview.md).*

DogLog writes WPILOG files on the roboRIO and republishes to NetworkTables so dashboards can read live. We extend it with [`Telemetry`](../../src/main/java/frc/spectrumLib/telemetry/Telemetry.java), and that is the layer essentially all code calls. Do not call `DogLog` directly.

`vendordeps/` holds the pinned JSON.

## What Telemetry adds

`Telemetry extends DogLog implements Subsystem` and registers itself, so its `periodic()` runs once a loop. On top of DogLog it gives us four things:

* A static entry point, `Telemetry.start(...)`, called once from the `Robot` constructor. Its arguments configure `DogLogOptions` and set the console print threshold. [Logging](../tools/logging.md) lists what each flag does and why ours are set the way they are. Read that before changing one.
* `Telemetry.print(...)`, which timestamps the line, filters it to the console by priority, and always writes it to the log under the `Prints` key. Use it instead of `System.out.println`, which loses the timestamp and vanishes from the log when console capture is off. `PrintPriority.HIGH` always reaches the console. `PrintPriority.NORMAL` only does when the threshold passed to `start(...)` is `NORMAL`, which is what we set.
* `Telemetry.logAlerts()`, which runs from `periodic()` and copies anything new out of the SmartDashboard alerts table into the log. It compares each alert against the previous poll rather than against everything it has ever seen, so an alert that clears for one poll and then returns is logged again, and the log holds one line per appearance.
* `Telemetry.log(Command)`, which decorates a command so it writes `Init:` and `End:` entries under the `Commands` key. Wrap the outermost command only. Wrapping an inner one just nests log lines for every internal step.

## Log keys

Keys are `Subsystem/Path/Name`, which renders as a tree in Elastic and AdvantageScope. Before inventing a new top level key, grep for the prefix first. A value that ends up under two different top level names is why nobody can find last season's log. `Launcher/RPM` and `Match Data/MatchTime` are the models to follow.

## Build stamps

The `Robot` constructor writes the `BuildConstants/*` keys, sourced from the `BuildConstants.java` that the `gversion` Gradle task regenerates on every compile. Those keys are how you tell which build was on the robot when you are reading a log a week later.

## Faults

`Telemetry.Fault` is a shared vocabulary for known failure modes. It is a catalog: there is no `logFault` helper, so a fault gets logged as a high priority print using the enum name. Add an entry when the same failure is being described by free text in more than one file.

## Things that have bitten us

NetworkTables bandwidth is shared with everything else on the bus. If a match day log gets noisy, log fewer high frequency values first. Turning the full mirror off does not switch NetworkTables off, and it does not silence `logDash` keys or other publishers such as `TuneValue`, so a dashboard fed by those keeps working.

`withNtTunables` reaches DogLog's own tunable entries and nothing else. `TuneValue` publishes straight to SmartDashboard, so the `tunableOnFMS` argument we pass to `start(...)` has no effect on it. Preventing match day tuning means guarding or removing the `TuneValue` call sites, not changing a DogLog option.

Mirroring every logged value to NetworkTables is not free. On the offseason robot on 2026-09-05, the mirror plus a flush every loop was a full-time job for one of the roboRIO's two cores. So the mirror starts off on the robot (on in simulation), is forced off whenever the FMS is attached, and dashboard values are published one by one with `Telemetry.logDash`. The `Telemetry/MirrorLogsToNT` switch on SmartDashboard turns the full mirror on or off in the shop. See [Logging](../tools/logging.md#what-reaches-the-dashboard).

Do not add instance methods to `Telemetry`. It is a static facade on purpose. Once part of the codebase calls `telemetry.log(...)` and the rest calls `Telemetry.log(...)`, both sides are wrong forever.

## Further reading

[DogLog JavaDoc](https://javadoc.doglog.dev) is linked into our generated docs, and the [README](https://github.com/jonahsnider/doglog) covers feature flags we have not touched. For what to log and when, see [Logging](../tools/logging.md).
