# Loop Time and CPU Handoff, 2026-09-05 Evening

*Audience: whoever picks this up next, including a Claude session on the shop laptop with the robot on the network. Everything referenced is in this repo or in the logs release; nothing lives only in a chat. Read [Offseason Handoff 2026-09-05](offseason-handoff-2026-09-05.md) first for the robot's state coming into the day.*

## Where things stand

- The loop overruns come from a saturated roboRIO CPU, not from one slow call. Every Driver Station log from 2026-09-05 shows RIO CPU at 92 to 95 percent, enabled or disabled, and every section of a slow loop stretched together, which is what preemption looks like.
- Three commits landed on `2026-offseason-bot` tonight: `ffb0be3` cuts load on every thread, `c4d7b54` adds system-health alerts and refreshes the Diagnostic tab, `8fb4f29` reverts the one change that misfired (real-time main thread priority).
- The robot was deployed once tonight, before the revert. The Driver Station showed stale swerve odometry signals and `WaitForAll -1003` errors while disabled. Those are pre-existing, at rates matching the daytime logs (see below). They were not caused by the new code, but the priority change was reverted because it can only make them worse.
- **Not yet measured:** CPU, CANivore utilization, loop period and the -1003 rate with `8fb4f29` deployed. That is the first job.
- The shop laptop's working tree may hold an uncommitted change from another agent: `BaseStatusSignal.setUpdateFrequencyForAll(20, moduleCurrentSignals)` in `Swerve`. It is harmless (commit it or drop it) but it does not touch the red errors; that agent's diagnosis attributed them to the Java `refreshAll`, and the location string proves otherwise (see Gotchas).

## What the logs showed

All eight 2026-09-05 robot wpilogs and the Driver Station logs are in the `logs-2026-09-05` release of [Spectrum3847/2026-Robot-Logs](https://github.com/Spectrum3847/2026-Robot-Logs). Pull them with:

```bash
gh release download logs-2026-09-05 -R Spectrum3847/2026-Robot-Logs -p "*.zip"
```

Enabled loop, 18:38 log (medians, p90 in parentheses):

| Section | ms |
|---|---|
| CommandScheduler.run (subsystem periodics and commands) | 15.1 (27.5) |
| Vision.periodic | 4.5 (12.7) |
| Rest of robotPeriodic (BatteryLogger, CANivore status, Field2d) | 2.8 (7.3) |
| SuperStructure.periodic | 0.6 (2.7) |
| Outside robotPeriodic (DS refresh, SmartDashboard) | 3.0 (9.0) |
| Whole loop period | 30.1 (47.1) |

Across the eight logs the enabled period median was 26 to 39 ms and 57 to 93 percent of enabled loops missed 25 ms. Every loop over 300 ms was disabled, inside the scheduler, at 25 to 33 s after boot or on an auto-chooser change: PathPlanner warmup and trajectory generation, harmless. The worst enabled loop was 232 ms. The GC theory from the earlier session did not hold for the big stalls; `-Xlog:gc*` is still on (`/home/lvuser/logs/gc.log`) to settle the 130 to 230 ms enabled episodes, and `SystemLoadMonitor` now logs GC time per second directly.

Other numbers: 1800 to 2800 log records per second (94 per loop in the 18:38 log), CANivore bus utilization 63 to 77 percent, DS "Memory Free" 4 to 9 MB (page cache, not a leak).

## What changed and why

| Change | Where | Why |
|---|---|---|
| DogLog to NetworkTables mirror off; per-key `Telemetry.logDash` for dashboard values; `Telemetry/MirrorLogsToNT` switch, forced off on FMS | `Telemetry` | Every logged value was republished to NT and flushed every 20 ms; a full-time job for one core |
| 10 Hz slow tier (`Telemetry.slowLogThisLoop()`) for currents, temperatures, vision status, battery, shot-calculator outputs while not launching | `Mechanism.logDiagnostics`, `Vision`, `BatteryLogger`, `ShotCalculator` | Cut records per second toward 1000 |
| One `BaseStatusSignal.refreshAll` per mechanism per loop, keyed on the `RobotLoop` counter; `CachedDouble` subsystems removed from `Mechanism` | `Mechanism` | Over a hundred JNI refreshes per loop |
| Mechanism status frames: 100 Hz position/velocity on leaders, 50 Hz output frames kept on leaders with followers, 20 Hz for followers and diagnostics | `Mechanism.configureStatusSignals` | Was 250 Hz on every motor and follower; bus at 63 to 77 percent |
| Swerve odometry stays at 250 Hz. Swerve state logged from `periodic()`, not CTRE's odometry-thread callback; drivetrain state read once per loop (`loopState()`), invalidated on vision fusion and resets; 16 module current signals refreshed together at 10 Hz | `Swerve` | The odometry callback logged under the drivetrain lock and its records were exactly the ones DogLog dropped |
| Phoenix hoot auto-logging off | `Robot` constructor | A large share of 2.2 GB on the SD card; nobody replays them |
| `CANBus.getStatus()` at 1 Hz | `Robot.logCanBusStatus` | CTRE: blocks up to 1 ms; ran every loop |
| Full GC in `disabledInit` and every 60 s while disabled | `Robot` | Serial GC's one long pause happens where it cannot matter |
| `-XX:MaxGCPauseMillis`, `-XX:GCTimeRatio` removed | `build.gradle` | Ignored by SerialGC |
| Vision telemetry at 10 Hz, MegaTag2 parsed only for the turret camera, Limelight scalar NT entries cached | `Vision`, `LimelightHelpers` | Vision.periodic was 4.5 ms median |
| `SystemLoadMonitor`: CPU, memory, GC, heap, loop period once a second under `System/`, with DS alerts | new class, wired first in `robotPeriodic` | Nothing on the dashboard said the CPU was at 93 percent |
| Elastic layout: Diagnostic tab gets RIO CPU graph and loop-health widgets in place of the dead DogLog queue graph; Pre-Match keeps one camera stream | `src/main/deploy/elastic-layout.json` | Three MJPEG streams on the pre-match tab spend field bandwidth |
| Pilot default command no longer re-initializes every second | `Pilot` | Showed in every overrun epoch print |
| **Reverted:** `Threads.setCurrentThreadPriority(true, 99)` around the loop body | `Robot` | See below |

## Pre-existing problems you will see on the Driver Station

**`ERROR -1003 CAN frame not received/too-stale ... ctre::phoenix6::BaseStatusSignal::WaitForAll`** with yellow `1000 CAN message is stale` warnings for talon fx 1, 2, 11, 12, 21, 22, 31, 32 (Position and Velocity) and pigeon 2 0 (Yaw, AngularVelocityZWorld). That is CTRE's native 250 Hz odometry thread timing out. It waits on eighteen signals with a two-period timeout, about 8 ms, and reports whenever frames arrive late because the bus is busy or Phoenix's receive path is short of CPU. Daytime baseline, before any change:

| Session 2026-09-05 | WaitForAll -1003 per minute |
|---|---|
| 11:37 | 149 |
| 11:52 | 49 |
| 12:09 | 66 |
| 13:37 | 18 |
| 15:00 | 4 |
| 15:25 | 60 |
| Tonight, priority change still in | about 15 |

**talon fx 18** (LauncherTower Back, the follower whose power lead was off on 09-05) logged 1189 -1003 errors on its own during the day. Until it is powered, expect its errors to continue and the "Tower Follower" bar on the Power tab plus its Driver Station alert to show it.

**A "Loop time of 0.xs overrun" print at every disable** is the deliberate GC. One per disable is expected; any while enabled is not.

## The real-time priority revert

`Threads.setCurrentThreadPriority(true, 99)` for the loop body (the 6328 pattern) was tried and reverted in `8fb4f29`. With the loop body still 15 to 30 ms long, a SCHED_FIFO main thread owned one core for most of every period and Phoenix's frame dispatch lost its turn on that core. The stale odometry errors were already present at similar rates without it, so it was not the cause, but it cannot help them either. Get the loop under budget by doing less; do not reintroduce the priority.

## Tonight's test, step by step

1. Deploy `8fb4f29` or later. Confirm the Driver Station console is not spamming anything new.
2. Sit disabled for two minutes with the **Diagnostic** tab open. Read RIO CPU (%), Loops over 25 ms (%), Loop Mean (ms), CANivore Bus (%), and the Alerts widget. Count red -1003 lines per minute and compare with the table above.
3. Enable, drive hard, shoot a few cycles. Watch SHOT READY on the Match tab, the follower bars on the Power tab, and whether any of the five new alerts appears (CPU high, loop overrunning, loop stalled, GC pause, memory low).
4. Pull the logs. From the repo, `./gradlew archiveLogs -PdryRun` lists what is on the rio; `./gradlew archiveLogs` uploads to a new release; `-Pdelete` clears the rio afterwards. Driver Station logs are in `C:\Users\Public\Documents\FRC\Log Files` on the DS laptop; zip the day's `.dslog` and `.dsevents` into the same release by hand.
5. Analyze:

```bash
node scripts/looptime.js FRC_2026xxxx_xxxxxx.wpilog
```

```bash
node scripts/dslog.js "C:\Users\Public\Documents\FRC\Log Files" 2026_09_05
```

Targets: RIO CPU under 75 percent, enabled loop period median under 20 ms, CANivore under 55 percent, -1003 under 5 per minute, no enabled loop over 100 ms.

## Decisions still open

- **Odometry rate.** Team preference is 250 Hz. 254, 6328, 1678 and 2910 all run 250 Hz on a CANivore in their 2025 code, but 254 puts the drivebase on its own CANivore and CTRE's own measurement has a bare swerve at 45 percent of a CANivore at 250 Hz. Ours shares one CANivore with thirteen mechanism motors and a CANdle. If the -1003 rate stays above 10 per minute with the bus under 55 percent, 200 Hz (10 ms tolerance, about 7 points less bus) is imperceptible on the field. The structural fix is moving the mechanism motors to the rio CAN bus (14 percent used today; they now run at 100 and 20 Hz) or adding a second CANivore. That is wiring plus the bus name in each mechanism config.
- **Alert thresholds** in `SystemLoadMonitor` (CPU 85 percent for 10 s, half the loops over 25 ms for 5 s, one enabled loop over 200 ms, 100 ms of GC in one second, under 24 MB available) are first guesses. Adjust after a real session.
- **Watchdog** stays at 0.20 s (`SpectrumRobot.LOOP_OVERRUN_WARNING_SECONDS`). Lower it for a diagnostic build only; each print is work.
- **What to log at loop rate.** Turret and launcher values stayed at 50 Hz on purpose. Revisit once records per second is known.

## Gotchas

- The `Scheduler/*` timers are logged in **seconds**. `scripts/looptime.js` converts.
- DogLog skips a record when the value has not changed (`DataLogEntry.update`). Booleans and state strings are free while steady; NaN is never equal to itself and logs every call, which is why estimate ages and heading error are on the slow tier.
- DogLog's default `ntPublish` was already "not on FMS"; the old `Telemetry.start(true, ...)` overrode it to always-on. Now `Telemetry.start(false, ...)`.
- Any new Elastic widget needs a `Telemetry.logDash` or `logDashAlways` behind its topic. The layout in `src/main/deploy/elastic-layout.json` is the list of what must stay live.
- Java `refreshAll` reports errors under `ctre.phoenix6.BaseStatusSignal.refreshAll` (dots, lowercase). `ctre::phoenix6::BaseStatusSignal::WaitForAll` with a C++ stack is the native odometry thread. That distinction settled tonight's misdiagnosis.
- The hoot path in `SwerveConfig` (`./logs/spectrum.hoot`) is a simulation replay file, not a recording location. Hoot recording on the rio is Phoenix auto-logging, now disabled.
- Elastic is Dart: geometry and ranges in the layout JSON must stay doubles (`512.0`, not `512`), divisions and colors ints. Tonight's edits used a node script that round-trips the file byte-identical before changing anything; hand-edit or let Elastic save it, do not run it through a generic JSON formatter.
- Gradle on the machine used tonight picked a JDK 25 from PATH and failed with "Unsupported class file major version 69". Pass the WPILib JDK: `-Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk"` with `JAVA_HOME` set to the same. `--offline` works.
- `logs/FRC_20260905_013127.wpilog` in this repo is a simulation log, not the robot.

## Files touched tonight

`build.gradle`, `Robot`, `SpectrumRobot`, `RobotLoop` (new), `Telemetry`, `SystemLoadMonitor` (new), `BatteryLogger`, `Mechanism`, `Swerve`, `SwerveAlignment`, `Vision`, `VisionLogger`, `LimelightHelpers`, `ShotCalculator`, `SuperStructure`, `Turret`, `Launcher`, `LauncherTower`, `Hood`, `DyeRotor`, `FuelIntake`, `IntakeExtension`, `Pilot`, `elastic-layout.json`, `scripts/looptime.js` and `scripts/dslog.js` (new), and the docs for logging, DogLog, Elastic, Tuner X and the offseason handoff.
