# Logging and data analysis

*Audience: reference. Assumes you have read [2026 season specific](../other-guides/2026-season-specific.md).*

A log is the difference between "the indexer stopped at champs" and "the indexer stopped because the CANivore dropped the bus at 2.14 seconds." Robot code logs through the free functions in `telemetry::telemetry` (`rust/crates/telemetry/src/telemetry.rs`). They sit on top of a port of [DogLog](https://doglog.dev). See [DogLog](../dependencies/doglog.md) for the crate.

## What ends up in the file

`telemetry::start(...)`, called from `Robot::new`, sets the policy. It takes seven arguments, in this order:

1. `nt_mirror_default`: the starting position of the SmartDashboard switch `Telemetry/MirrorLogsToNT`. The switch copies every logged value to NetworkTables so AdvantageScope sees it live. The robot passes `wpilib::is_simulation()`, so it starts on in simulation and off on the robot, to save roboRIO CPU. Flip it in Elastic when AdvantageScope needs the full live stream in the shop. It stays off whenever the FMS is attached.
2. `capture_ds`: records SmartDashboard entries in the log. On.
3. `capture_nt`: would record the whole NetworkTables tree. The Rust port does not support it. Off.
4. `capture_console`: puts console output in the log. On.
5. `log_extras`: adds PDH currents, CAN bus use and radio status. Off. Turn it on for a brownout investigation.
6. `tunable_on_fms`: keeps DogLog's own tunables editable with an FMS attached. On.
7. `priority`: the lowest print priority that reaches the console. The robot passes `PrintPriority::Normal`.

## Where the file goes

The log manager picks the folder:

* On a robot: a USB stick's `logs` folder if one is plugged in and writable, else `/home/lvuser/logs`.* In simulation: a `logs` folder in the folder you ran from. `cargo xtask sim` runs from `rust/`.

A file starts as `FRC_TBD_<random>.wpilog`. It is renamed `FRC_<UTC time>.wpilog` once the driver station connects, and `FRC_<UTC time>_<event>_<P|Q|E><match>.wpilog` once the FMS sends match info. When free space drops under 50 MB, old logs are deleted, but at least 10 stay.

## Naming keys

Keys are `Subsystem/Name`. Two rules:

* Start with the subsystem's name. `Launcher/RPM` is findable. `motorSpeed7` is not.
* Pass the unit with the `_with_unit` functions. The log stores it, and AdvantageScope uses it for axis labels.

Log inside `periodic`. That is the one place a value is fresh at loop rate. Do not log the same value under two names.

## What reaches the dashboard

With the mirror off, a value is on NetworkTables only if the code publishes it on purpose. There are three tiers:

| Call | wpilog | NetworkTables | Use for |
|---|---|---|---|
| `telemetry::log(key, value)` | every call, skipped when unchanged | only through the mirror | everything |
| `telemetry::log_dash(key, value)` | every call | every fifth loop (10 Hz) | keys the Elastic layout shows |
| `telemetry::log_dash_always(key, value)` | every call | every call | keys logged once or on their own timing, like `BuildConstants/*` |

Each has a `_with_unit` form. `telemetry::slow_log_this_loop()` is true on the same every-fifth loop. Use it for anything that does not need 20 ms resolution. The keys in `src/main/deploy/elastic-layout.json` have to stay `log_dash` or `log_dash_always`.

Other functions: `log_struct`, `log_struct_array`, `log_enum` and `log_state` for non-number values, and `time` / `time_end` to time a block. Enums implement `EnumName` so the log shows their name in capitals, like `INTAKE_FUEL`.

## Per-subsystem keys

Each subsystem logs its wanted state, its system state and its running command every loop from `periodic`. Look for `<Subsystem>/CurrentCommand` in a log to see what owns a mechanism.

## Console prints

`telemetry::print(...)` stamps the line with the FPGA time, prints it to the console if the priority allows, and always writes it to the `Prints` key. `print_with(text, PrintPriority::High)` always reaches the console. A normal print reaches it only when the global priority is `Normal`, which the robot sets.

Use prints for startup steps and faults. Anything you want to graph belongs in `log`.

## Alerts

`Telemetry::periodic` runs every loop as a subsystem. It copies new entries from `SmartDashboard/Alerts` into the `Alerts` key, once per appearance. A WPILib `Alert` shows on the dashboard and lands in the file with no extra work.

## Faults

`Fault` in `telemetry.rs` names three failure modes: `CameraOffline`, `AutoShotTimeoutTriggered` and `Brownout`. Nothing in the robot logs them yet. There is no `log_fault` function.

## Which build wrote this

At startup the robot logs the project name, build date, git SHA, git date, git branch and dirty flag under `BuildConstants/`. A log without them cannot be compared to anything.

## System health

`SystemLoadMonitor` runs once a second and publishes `System/CpuPercent`, `System/MemAvailableMB` and `System/Loop/*` (mean and max loop period, overrun share). It raises alerts, listed on the [Elastic](elastic.md#system-health-alerts) page. Its constants are at the top of `telemetry/src/system_load_monitor.rs`.

## Tunables

`TuneValue` publishes a number to SmartDashboard so you can edit it live. It stays editable with an FMS attached, whatever `tunable_on_fms` says. No robot code uses it yet. See [PID tuning](pid-tuning.md#live-tuning-with-tunevalue).

## Getting logs off the roboRIO

`.wpilog` files are in `/home/lvuser/logs` or on the USB stick. The `robot-log-upload` skill in `.agents/skills` pulls them off the roboRIO and uploads them to the team log site. Open them in [AdvantageScope](https://docs.advantagescope.org) with **File, Open Log**. Drag entries from the tree into a plot.

For a live session, start AdvantageScope and connect it to the robot. It reads NetworkTables directly.

## What to log

Motor voltages and currents, sensor readings, setpoints, state changes and vision results. Do not log the same value many times in one loop.

## See also

* [DogLog](../dependencies/doglog.md) for the crate.
* [Elastic dashboard](elastic.md) for the live view.
* [Exception handling](../coding-conventions/exception-handling.md) for when to print at high priority.
