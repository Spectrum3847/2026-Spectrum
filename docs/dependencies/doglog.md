# DogLog

*Audience: reference. Assumes you have read [Dependencies overview](overview.md).*

DogLog writes `.wpilog` files on the roboRIO and also publishes to NetworkTables so dashboards can read values live. The `telemetry` crate ports **DogLog 2026.5.0** (MIT license, copyright Jonah Snider). It also ports WPILib's log writer and `DataLogManager`, so the logging stack is pure Rust.

Robot code does not call `DogLog` directly. It calls the free functions in `telemetry::telemetry` (`rust/crates/telemetry/src/telemetry.rs`), which wrap it.

## What the crate holds

| File | What |
|---|---|
| `telemetry.rs` | The functions robot code calls: `log`, `log_dash`, `print` and others. Also the `Telemetry` subsystem struct. |
| `dog_log.rs` | The DogLog port: a queue, a log thread, options. |
| `writers.rs`, `extras.rs` | The wpilog and NetworkTables writers, and the extras (PDH, CAN bus, radio). |
| `datalog.rs`, `wpilog.rs`, `data_log_manager.rs` | The wpilog file format and file management. |
| `battery_logger.rs`, `system_load_monitor.rs` | Battery and loop-health monitoring. |
| `tune_value.rs` | `TuneValue`, a number you can edit live. |

## What the wrapper adds

* `telemetry::start(...)`, called once from `Robot::new`. Its arguments set the DogLog options and the console print threshold. [Logging](../tools/logging.md) lists each one and why ours are set the way they are.
* `telemetry::print(...)`, which stamps the line, filters it to the console by priority and always writes it to the `Prints` key. Use it instead of `println!`. A plain `println!` has no stamp, and it is missing from the log when console capture is off.
* `Telemetry::periodic`, which copies new alerts from the SmartDashboard alerts table into the log. The robot registers it as a subsystem, so the scheduler calls it every loop.
* `log_command_init` and `log_command_end`, which write `Init:` and `End:` lines to the `Commands` key.
* The three tiers: `log`, `log_dash` and `log_dash_always`.

## Log keys

Keys are `Subsystem/Path/Name`. They show as a tree in Elastic and AdvantageScope. Before you invent a new top-level key, search the code for the prefix. A value logged under two top-level names is why nobody finds last season's log. `Launcher/RPM` and `Match Data/MatchTime` are good models.

## Build stamps

`Robot::new` writes the `BuildConstants/*` keys. The values come from the `build_constants.rs` that `crates/robot/build.rs` writes on each build. They tell you which build was on the robot when you read a log a week later.

## Faults

`Fault` in `telemetry.rs` is a short list of failure names. Nothing logs them yet. Add an entry when the same failure is described with free text in more than one file.

## Things that have bitten us

NetworkTables bandwidth is shared. If a match log gets noisy, log fewer fast values first. Turning the mirror off does not stop `log_dash` keys or other publishers such as `TuneValue`.

`tunable_on_fms` reaches DogLog's own tunables and nothing else. `TuneValue` writes straight to SmartDashboard, so that flag does not affect it. To stop tuning during a match, remove or guard the `TuneValue` call sites.

Mirroring every value to NetworkTables is not free. On the offseason robot on 2026-09-05, the mirror plus a flush every loop used one of the roboRIO's two cores. So the mirror starts off on the robot, is forced off whenever the FMS is attached, and dashboard values are published one by one with `log_dash`. The `Telemetry/MirrorLogsToNT` switch turns the full mirror on in the shop. See [Logging](../tools/logging.md#what-reaches-the-dashboard).

Not ported: `capture_nt`, which would log every NetworkTables change. It needs ntcore's native data log. The option exists but does nothing.

## Threads

Telemetry functions work from any thread. Log calls queue an entry, and the log thread writes it. The queue holds 5000 entries before it drops new ones. Cutting what you log is the real fix for drops.

## Further reading

The [DogLog README](https://github.com/jonahsnider/doglog) covers features we do not use. For what to log and when, see [Logging](../tools/logging.md).
