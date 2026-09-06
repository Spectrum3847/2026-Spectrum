# Loop Time and CPU Handoff, 2026-09-05 Evening

*Audience: whoever picks this up next, including a Claude session on the shop laptop with the robot on the network. Everything referenced is in this repo or in the logs release; nothing lives only in a chat. Read [Offseason Handoff 2026-09-05](offseason-handoff-2026-09-05.md) first for the robot's state coming into the day.*

## Where things stand

- The loop overruns come from a saturated roboRIO CPU, not from one slow call. Every Driver Station log from 2026-09-05 shows RIO CPU at 92 to 95 percent, enabled or disabled, and every section of a slow loop stretched together, which is what preemption looks like.
- Three commits landed on `2026-offseason-bot` tonight: `ffb0be3` cuts load on every thread, `c4d7b54` adds system-health alerts and refreshes the Diagnostic tab, `704030d` reverts the one change that misfired (real-time main thread priority). In between, `d5bcc36` from the shop laptop reworked `tools/archive-logs.sh` to name release assets by path and to stop deleting rio files that were never uploaded.
- The robot was deployed once tonight, before the revert. The Driver Station showed stale swerve odometry signals and `WaitForAll -1003` errors while disabled. Those are pre-existing, at rates matching the daytime logs (see below). They were not caused by the new code, but the priority change was reverted because it can only make them worse.
- **Not yet measured:** CPU, CANivore utilization, loop period and the -1003 rate with `704030d` deployed. That is the first job.
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

`Threads.setCurrentThreadPriority(true, 99)` for the loop body (the 6328 pattern) was tried and reverted in `704030d`. With the loop body still 15 to 30 ms long, a SCHED_FIFO main thread owned one core for most of every period and Phoenix's frame dispatch lost its turn on that core. The stale odometry errors were already present at similar rates without it, so it was not the cause, but it cannot help them either. Get the loop under budget by doing less; do not reintroduce the priority.

## Tonight's test, step by step

1. Deploy `704030d` or later. Confirm the Driver Station console is not spamming anything new.
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

## Results from the shop laptop, 2026-09-05 22:00 to 22:20

*Robot on the bench at 10.85.15.2 (team number 8515 in `.wpilib/wpilib_preferences.json`; the 3847 addresses do not answer). Disabled the whole time. Rio clock is UTC, so rio log names are five hours ahead of the Driver Station file names.*

**The -1003 and stale-signal errors are gone with `704030d` deployed.** Deployed `bd8b047` at 22:07:56. Nine minutes disabled: zero `WaitForAll -1003`, zero `1000 CAN message is stale`, zero talon fx 18 errors. Same bench, same wiring, same evening.

What was actually running before that: the jar on the rio was built at 21:27:56, before the revert commit existed, so the 21:28 Driver Station session was the **real-time priority build**. Its numbers, against the old-code session just before it and the fixed build after:

| DS session | Code | RIO CPU med | CANivore bus | Loop body med | -1003 | Stale warnings |
|---|---|---|---|---|---|---|
| 20:53, 7 min | `a9b761e` (old) | 95% | 67% | 14.1 ms | 11 (2/min) | 16 |
| 21:28, 31 min | `c4d7b54` + RT priority 99 | 57% | 46% | 2.8 ms | 116 (18/min early) | 682 |
| 22:07, 9 min | `bd8b047` (revert in) | 62% | 46% | 5 ms warming up | **0** | **0** |

The 21:28 errors were front-loaded: 22 in the first minute, 14, 7, 7, 11, 7, then about one a minute after ten minutes. That is the JIT warming up. Early on the loop body is long and interpreted, a SCHED_FIFO 99 main thread holds a core for all of it, and Phoenix's threads miss their turn. As the body shrank the errors thinned out. Bus utilization was already down to 46 percent in that session, so the bus was not what was tripping the odometry thread; the priority was. The revert fixed it; nothing else changed between the two builds.

**The daytime and 09-04 rates were mostly talon fx 18.** In the 09-04 23:51 session the -1003 errors came every 3.00 s for 40 minutes, 810 of them, and 774 were each followed within half a second by `-10021 Device firmware could not be retrieved` for talon fx 18. Phoenix retried the unpowered follower every three seconds and each retry stalled the frame path long enough to trip the odometry thread's two-period timeout. Talon 18 has power now and neither error has appeared since. Sessions with talon 18 quiet had 0 to 4 -1003s in total.

**Odometry stays at 250 Hz.** No reason to drop to 200 or 150 Hz on this evidence. Revisit only if -1003s come back while enabled with the bus under 55 percent.

### What the rio looks like now (disabled, warm, from `/proc`)

- Two cores. Busy 69 to 73 percent by `/proc/stat` over 30 s, of which user 40, sys 23 to 27, softirq 5 to 7. The Driver Station reports 62 percent median for the same build. The system time is CAN and USB work, not Java.
- About 23,000 context switches a second.
- The CANivore enumerates at **USB Full Speed, 12 Mbps** (`/sys/bus/usb/devices/1-1.2/speed`), behind the rio's internal hub, with 1,100 USB interrupts a second. That is what the device is; it is not a fault, but every CAN frame batch pays a 1 ms USB frame.
- Hottest threads (percent of one core, 10 s sample): main robot thread 35; Phoenix odometry thread (SCHED_RR priority 1) 16; two CANivore transport threads (SCHED_RR 3 and 2) 14 and 11; two unnamed native threads at normal priority 15 and 8; DogLog log thread 11; rio CAN `can_recv` 5; PathPlanner `ADStar Planning` 4. The main thread is at normal priority (policy 0), which confirms the revert is what is running. WPILib's HAL notifier is the one SCHED_FIFO thread, at priority 40, and is idle.
- The main thread at 35 percent with a 3 to 5 ms loop body is more than the body accounts for. The rest is the loop's own overhead outside `robotPeriodic` (DS refresh, SmartDashboard, LiveWindow) plus JIT early on. Worth a look if CPU needs to come down further; not related to the CAN errors.
- Seven loops over 200 ms in the first eight minutes disabled: the deliberate full GC every 60 s while disabled, plus PathPlanner warmup. Expected.

### Still to do

1. Enable and drive with `bd8b047` or later. Everything above is disabled-only; the enabled loop and bus numbers in the tables at the top are still from the old code.
2. Archive tonight's logs: rio `FRC_20260906_014905` (old code, 20:49 to 21:27), `FRC_20260906_022826` (RT-priority build, 21:28 to 22:07), `FRC_20260906_030756` (revert), and the Driver Station files `2026_09_05 20_53_13`, `21_28_03`, `22_07_36`.
3. Commit or drop the uncommitted `setUpdateFrequencyForAll(20, moduleCurrentSignals)` in `Swerve` if it is still in someone's tree; it was not on this laptop.
4. `scripts/dslog.js` counts the `Tracer` epoch prints as loop overruns; its "loop overrun prints" number is high by those.

### Enabled on the cart, 22:54 to 23:47 (two teleop runs, 185 s and 183 s, no stick input, turret belt off)

- **CAN:** two `WaitForAll -1003` in the first run (t=1560 and t=1658 in `FRC_20260906_030756.wpilog`, one with stale Position/Velocity on talon fx 2), none in the second. 0.3 per minute enabled overall. Not correlated with CPU bursts.
- **Load enabled:** CPU 88 to 92 percent busy by both `SystemLoadMonitor` and a `/proc` sampler on the rio (disabled: 65 to 70). CANivore bus 58 percent (disabled 46). Loop period median 20.1 ms, 10 to 13 percent of loops over 25 ms, body median 7.2 ms. The rise is the swerve control frames: with `TELEOP_DRIVE` active Phoenix sends eight TorqueCurrentFOC requests every 4 ms regardless of stick input, and the kernel USB work that comes with them is the roughly 40 percent of one core that no thread owns (`irq/53-e0002000` shows 15 percent, the rest is hardirq/softirq on the `cpu` line).
- **Thread split enabled** (percent of one core, 3 min average): main 22, Phoenix odometry (SCHED_RR 1) 23, CANivore transport threads 10 and 6, two unnamed native threads 9 and 4, DogLog 8, `FRC_NetCommDaemon` 5, eth irq 4.
- **Unexplained once:** in the first run, seven-second stretches at t=1518 and t=1579 (60 s apart, 22 s after enable) where every loop was 30 to 130 ms and every section stretched together, vision 2 to 40 ms, scheduler 4 to 60 ms. Pure contention from something else, not GC (no full collection in this JVM while enabled, `Gc/MsPerSecond` flat). Did not recur in the second run with the sampler watching. If it comes back, `/tmp/sampler.sh` on the rio (one awk pass per second into `/home/lvuser/sample.txt`) will name the thread; parse with the pattern in this session's scripts, watching for the double space after `cpu` in `/proc/stat`.
- **Subsystems behaved as designed:** Launcher `IDLE_PREP` 700 RPM at 2 V, DyeRotor `IDLE_SLOW_INDEX` -20 RPM, turret `AIM_AT_TARGET` from odometry with no tags (1028 degrees of travel with the belt off), hood and tower off, swerve holding on the cart at 4 A drive and 6 A steer stator. Alerts: only gamepads disconnected and pose not vision-seeded. Battery 11.85 V, known low.
- **Next lever if CPU headroom matters:** odometry 250 to 200 Hz in the `Swerve` constructor. Cuts odometry thread, control frames, USB interrupts and bus by a fifth. Not needed for the CAN errors; those were the priority build and talon 18.
