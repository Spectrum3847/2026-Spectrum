# Elastic dashboard

*Audience: reference. Assumes you have read [Setup](../setup.md).*

[Elastic](https://github.com/Gold872/elastic-dashboard) is the driver station dashboard we run at practice and at matches. It reads NetworkTables, shows the auto chooser and draws the field. It comes with the WPILib installer.

## Our layout

The layout is [`src/main/deploy/elastic-layout.json`](../../src/main/deploy/elastic-layout.json). `cargo xtask deploy` copies it to the roboRIO with the rest of `src/main/deploy`, but Elastic does not load it on its own. On each laptop, connect to the robot and use **File > Download From Robot**. Edit the layout in Elastic and save it back to that file. Let Elastic write the JSON. Do not tidy the whitespace by hand.

**Pre-Match** is the tab on screen between matches, and it holds the auto chooser. **Git Status** shows which build is on the robot. It reads the `BuildConstants/*` keys the robot publishes at startup.

If you add a widget, make sure the robot publishes its topic with `telemetry::log_dash` or `log_dash_always`. A plain `telemetry::log` key is not on NetworkTables unless the `Telemetry/MirrorLogsToNT` switch is on.

## System health alerts

`SystemLoadMonitor` samples the roboRIO once a second. It publishes `System/CpuPercent`, `System/MemAvailableMB` and `System/Loop/MeanPeriodMs`, `System/Loop/MaxPeriodMs` and `System/Loop/OverrunPercent`. It raises driver station alerts, which show in every Alerts widget:

| Alert | Condition |
|---|---|
| roboRIO CPU high (warning) | CPU at or above 85 percent for 10 s. Clears under 80 percent. |
| Robot loop overrunning (warning) | More than half the loops over 25 ms for 5 s. Clears under a quarter. |
| Robot loop stalled while enabled (error) | One enabled loop over 200 ms. Stays up 10 s. |
| roboRIO memory low (warning) | Under 24 MB available for 10 s. |

The thresholds are constants at the top of `rust/crates/telemetry/src/system_load_monitor.rs`.

## NetworkTables in brief

Elastic talks to the robot over NetworkTables. Everything the robot publishes is reachable, such as SmartDashboard values and the `log_dash` keys. Widgets bind to a topic like `/SmartDashboard/Field2d`. That is why our log keys use a `Subsystem/Name` form: it keeps the topic tree easy to browse. See [Logging](logging.md#what-reaches-the-dashboard).

The reverse works too. The auto chooser writes its choice back over NetworkTables. Live-tunable values use SmartDashboard numbers through `TuneValue` (see [PID tuning](pid-tuning.md)).

## Connecting

Point Elastic at the robot: `roborio-3847-frc.local` for the real robot, `localhost` for [simulation](simulation.md). Then use **File > Open Layout**.

The robot also serves the whole `deploy/` folder over HTTP on port 5800. `http://<roboRIO address>:5800/elastic-layout.json` opens in any browser on the network. That helps when you are not at the driver station laptop.

Put Elastic in the same screen position before every match. The operator should never hunt for a widget during a match.

## Habits worth keeping

* One job per tab, so nothing important sits under something else.
* The same colors for booleans everywhere. Ours are green for true and red for false.
* Preview the auto on `Field2d` while disabled, before you enable.

## See also

* [Logging](logging.md) for what `telemetry` publishes and how keys are named.
* [PID tuning](pid-tuning.md) for the live tunables you can edit here.
