# Elastic dashboard

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Elastic](https://github.com/Gold872/elastic-dashboard) is the driver station dashboard we run at practice and at matches. It reads NetworkTables, gives the operator the auto chooser, and renders the field. It ships with the WPILib installer.

## Our layout

The layout is [`src/main/deploy/elastic-layout.json`](../../src/main/deploy/elastic-layout.json). It deploys to the roboRIO with the rest of `src/main/deploy`, but Elastic does not load it on its own. On each laptop, connect to the robot and use **File > Download From Robot** to get the same tabs we do. Edit it in Elastic and save back to that file. It is JSON and the formatter does not touch it, so let Elastic write it rather than cleaning up whitespace by hand.

**Pre-Match** is the tab on screen between matches and it owns the chooser. **Git Status** answers "which build is actually on this robot": it reads the build stamp the robot publishes at startup.

If you add a widget, edit the layout in Elastic and save it back to the file, and make sure the topic it reads is published by a `Telemetry.logDash` or `logDashAlways` call. Plain `Telemetry.log` keys are not on NetworkTables unless the `Telemetry/MirrorLogsToNT` switch is on.

## System health alerts

`SystemLoadMonitor` samples the roboRIO once a second and publishes `System/CpuPercent`, `System/MemAvailableMB`, `System/HeapUsedMB`, `System/Gc/MsPerSecond` and the loop period mean, max and overrun share under `System/Loop/`. It raises Driver Station alerts, which show in every Alerts widget:

|                  alert                   |                                  condition                                  |
|------------------------------------------|-----------------------------------------------------------------------------|
| roboRIO CPU high (warning)               | CPU at or above 85 % for 10 s; clears under 80 %                            |
| Robot loop overrunning (warning)         | more than half the loops over 25 ms for 5 s; clears under a quarter         |
| Robot loop stalled while enabled (error) | one enabled loop over 200 ms; stays up 10 s                                 |
| GC pause while enabled (warning)         | 100 ms or more of collector time in one second while enabled; stays up 10 s |
| roboRIO memory low (warning)             | under 24 MB available for 10 s                                              |

Thresholds are constants at the top of `SystemLoadMonitor`.

## NetworkTables in brief

Elastic talks to the robot over NetworkTables. Anything the robot publishes, such as `SmartDashboard.put*`, Shuffleboard, or our `Telemetry.logDash` (plain `Telemetry.log` reaches NT only through the mirror, see [Logging](logging.md#what-reaches-the-dashboard)), is reachable. Widgets bind to a topic like `/SmartDashboard/Field2d` or `/Robot/Initialized`, which is why our log keys use a `Subsystem/Path/Name` hierarchy. It keeps the topic tree navigable.

The reverse direction works too. The auto chooser writes back over NT to a `SendableChooser`. Live-tunable values use `SmartDashboard.getNumber(...)` wrapped by `TuneValue` (see [PID Tuning](pid-tuning.md)).

## Connecting

Point Elastic at the robot, `roborio-3847-frc.local` for the real bot and `localhost` in sim, then **File, then Open Layout**.

If your copy and the robot's have drifted, the layout menu's **Download from robot** grabs whatever the RIO has deployed. The robot serves the whole `deploy/` directory over HTTP on port 5800, so `http://<rio-ip>:5800/elastic-layout.json` opens in any browser on the network, which saves you when you are not sitting at the driver station laptop.

Pin Elastic to the same monitor position before every match. The operator should never be hunting for a widget mid-match, and a relocated widget at the wrong moment is exactly the kind of small thing that costs points.

## Habits worth keeping

One job per tab, so nothing important is buried under something else.

Consistent colors for booleans. Most of ours are green for true and red for false. Mixed colors on the same screen cost the operator real time, and the habit only works if it is uniform.

Preview the auto on `Field2d` while disabled, before you enable, rather than finding out on the field.

## See also

* [Logging](logging.md) for what `Telemetry` publishes and the key naming convention.
* [PID Tuning](pid-tuning.md) for the live tunables you can edit from here.
