# Loop time and CPU handoff, 2026-09-05 evening

*Audience: Whoever picks this up next, a team student or a mentor with the robot on the network.
Everything referenced is in this repo or in the logs release. Read
[Offseason handoff 2026-09-05](offseason-handoff-2026-09-05.md) first for the robot's state coming
into the day. This is a snapshot of 2026-09-05. Branch `2026-offseason-bot`.*

## Where things stood

* The loop overruns come from a saturated roboRIO CPU, not from one slow call. Every Driver Station
  log from 2026-09-05 shows RIO CPU at 92 to 95 percent, enabled or disabled, and every section of a
  slow loop stretched together, which is what preemption looks like.
* That evening three changes landed in sequence. The first cut load on every thread, the second
  added system-health alerts and refreshed the Diagnostic tab, and the third reverted the one change
  that misfired, which was real-time priority on the main thread. Separately, a change from the shop
  laptop reworked the log archiving script to name release assets by path and to stop deleting rio
  files that were never uploaded.
* The robot was deployed once that evening, before the revert. The Driver Station showed stale swerve
  odometry signals and `WaitForAll -1003` errors while disabled. Those are pre-existing, at rates
  matching the daytime logs (see below). They were not caused by the new code, but the priority
  change was reverted because it can only make them worse.
* **Not yet measured:** CPU, CANivore utilization, loop period, and the -1003 rate with the revert
  deployed. That was the first job, and the "Results from the shop laptop" section below is its
  answer.
* One more thing that was unresolved at the time: the shop laptop's working tree held an uncommitted
  change from another agent, a `setUpdateFrequencyForAll` call on the swerve module current signals.
  It is harmless either way, and it does not touch the red errors. That agent's diagnosis
  attributed the errors to the Java `refreshAll`, and the error's own location string proves
  otherwise (see Gotchas).

## What the logs showed

All eight 2026-09-05 robot wpilogs and the Driver Station logs are in the `logs-2026-09-05` release
of [Spectrum3847/2026-Robot-Logs](https://github.com/Spectrum3847/2026-Robot-Logs). Pull them with:

```bash
gh release download logs-2026-09-05 -R Spectrum3847/2026-Robot-Logs -p "*.zip"
```

Enabled loop, 18:38 log (medians, p90 in parentheses):

|                             Section                             |     ms      |
|-----------------------------------------------------------------|-------------|
| CommandScheduler.run (subsystem periodics and commands)         | 15.1 (27.5) |
| Vision.periodic                                                 | 4.5 (12.7)  |
| Rest of robotPeriodic (BatteryLogger, CANivore status, Field2d) | 2.8 (7.3)   |
| SuperStructure.periodic                                         | 0.6 (2.7)   |
| Outside robotPeriodic (DS refresh, SmartDashboard)              | 3.0 (9.0)   |
| Whole loop period                                               | 30.1 (47.1) |

Across the eight logs the enabled period median was 26 to 39 ms and 57 to 93 percent of enabled
loops missed 25 ms. Every loop over 300 ms was disabled, inside the scheduler, at 25 to 33 s after
boot or on an auto-chooser change: PathPlanner warmup and trajectory generation, harmless. The worst
enabled loop was 232 ms. The GC theory from the earlier session did not hold for the big stalls.
`-Xlog:gc*` is still on (`/home/lvuser/logs/gc.log`) to settle the 130 to 230 ms enabled episodes.

Other numbers: 1800 to 2800 log records per second (94 per loop in the 18:38 log), CANivore bus
utilization 63 to 77 percent, DS "Memory Free" 4 to 9 MB (page cache, not a leak).

## What was cut, and why

|                                                                     What changed                                                                      |                                                            Why                                                            |
|-------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| The log-to-NetworkTables mirror was turned off, and dashboard values publish per key instead                                                          | Every logged value was being republished to NT and flushed every 20 ms, a full-time job for one core                      |
| A slow logging tier, so currents, temperatures, vision status, battery, and shot-calculator output stop publishing at loop rate                       | Cut records per second toward 1000                                                                                        |
| Each mechanism refreshes all of its signals with one Phoenix call per loop, instead of one call per signal per getter                                 | Over a hundred JNI refreshes per loop                                                                                     |
| Mechanism status frames came down hard. Swerve odometry was left alone                                                                                | Status frames had been 250 Hz on every motor and every follower, and the bus was at 63 to 77 percent                      |
| Swerve state is logged from `periodic()` rather than from CTRE's odometry callback, and the drivetrain state is read once per loop                    | The odometry callback logged under the drivetrain lock, and its records were exactly the ones the log writer was dropping |
| Phoenix hoot auto-logging turned off                                                                                                                  | A large share of 2.2 GB on the SD card, and nobody replays them                                                           |
| CAN bus status polled instead of every loop                                                                                                           | CTRE notes it blocks for up to 1 ms                                                                                       |
| A full garbage collection on every disable                                                                                                            | The serial collector's one long pause happens where it cannot matter                                                      |
| The GC tuning flags removed from the build                                                                                                            | Ignored by the serial collector                                                                                           |
| Vision telemetry slowed, MegaTag2 parsed only for the turret camera, Limelight scalar entries cached                                                  | `Vision.periodic` was 4.5 ms median                                                                                       |
| A new system-load monitor logging CPU, memory, GC, heap, and loop period once a second, with Driver Station alerts                                    | Nothing on the dashboard said the CPU was at 93 percent                                                                   |
| The Elastic Diagnostic tab got a CPU graph and loop-health widgets in place of the dead log-queue graph, and the pre-match tab kept one camera stream | Three MJPEG streams on the pre-match tab spend field bandwidth                                                            |
| The pilot's default command stopped re-initializing every second                                                                                      | It showed up in every overrun epoch print                                                                                 |

## Pre-existing problems you will see on the Driver Station

**`ERROR -1003 CAN frame not received/too-stale ... ctre::phoenix6::BaseStatusSignal::WaitForAll`**
with yellow `1000 CAN message is stale` warnings for talon fx 1, 2, 11, 12, 21, 22, 31, 32 (Position
and Velocity) and pigeon 2 0 (Yaw, AngularVelocityZWorld). That is CTRE's native 250 Hz odometry
thread timing out. It waits on eighteen signals with a two-period timeout, about 8 ms, and reports
whenever frames arrive late because the bus is busy or Phoenix's receive path is short of CPU.
Daytime baseline, before any change:

|        Session 2026-09-05         | WaitForAll -1003 per minute |
|-----------------------------------|-----------------------------|
| 11:37                             | 149                         |
| 11:52                             | 49                          |
| 12:09                             | 66                          |
| 13:37                             | 18                          |
| 15:00                             | 4                           |
| 15:25                             | 60                          |
| Tonight, priority change still in | about 15                    |

**talon fx 18**, the launcher tower follower whose power lead was off on 09-05, logged 1189 -1003
errors on its own during the day. Until it is powered, expect its errors to continue, and expect the
"Tower Follower" bar on the Power tab plus its Driver Station alert to show it.

**A "Loop time of 0.xs overrun" print at every disable** is the deliberate garbage collection. One
per disable is expected. Any while enabled is not.

## The real-time priority revert

`Threads.setCurrentThreadPriority(true, 99)` around the loop body, the 6328 pattern, was tried and
reverted that evening. With the loop body still 15 to 30 ms long, a `SCHED_FIFO` main thread owned
one core for most of every period and Phoenix's frame dispatch lost its turn on that core. The stale
odometry errors were already present at similar rates without it, so it was not the cause, but it
cannot help them either. Get the loop under budget by doing less. Do not reintroduce the priority.

## Tonight's test, step by step

1. Deploy the current branch, or later. Confirm the Driver Station console is not spamming anything
   new.
2. Sit disabled for two minutes with the **Diagnostic** tab open. Read RIO CPU (%), Loops over 25 ms
   (%), Loop Mean (ms), CANivore Bus (%), and the Alerts widget. Count red -1003 lines per minute
   and compare with the table above.
3. Enable, drive hard, shoot a few cycles. Watch SHOT READY on the Match tab, the follower bars on
   the Power tab, and whether any of the five new alerts appears (CPU high, loop overrunning, loop
   stalled, GC pause, memory low).
4. Pull the logs. From the repo, `./gradlew archiveLogs -PdryRun` lists what is on the rio,
   `./gradlew archiveLogs` uploads to a new release, and `-Pdelete` clears the rio afterwards.
   Driver Station logs are in `C:\Users\Public\Documents\FRC\Log Files` on the DS laptop. Zip the
   day's `.dslog` and `.dsevents` into the same release by hand.
5. Analyze:

```bash
node scripts/looptime.js FRC_2026xxxx_xxxxxx.wpilog
```

```bash
node scripts/dslog.js "C:\Users\Public\Documents\FRC\Log Files" 2026_09_05
```

Targets: RIO CPU under 75 percent, enabled loop period median under 20 ms, CANivore under 55
percent, -1003 under 5 per minute, no enabled loop over 100 ms.

## Decisions still open

* **Odometry rate.** Team preference is 250 Hz, and after the evening's measurement that decision
  was made: no reason to drop to 200 or 150 Hz on this evidence. The reasoning is under "Results
  from the shop laptop" below. Revisit only if -1003s come back while enabled with the bus under 55
  percent. If they do, the structural fix is moving the mechanism motors to the rio CAN bus (14
  percent used today) or adding a second CANivore, which is wiring plus a bus name in each
  mechanism config.
* **Alert thresholds** in the system-load monitor (CPU 85 percent for 10 s, half the loops over
  25 ms for 5 s, one enabled loop over 200 ms, 100 ms of GC in one second, under 24 MB available)
  are first guesses. Adjust after a real session.
* **Watchdog.** It stays where it is. Lower it for a diagnostic build only, because each print is
  itself work.
* **What to log at loop rate.** Turret and launcher values stayed high on purpose. Revisit once
  records per second is known.

## Gotchas

* The `Scheduler/*` timers are logged in **seconds**. `scripts/looptime.js` converts.
* DogLog skips a record when the value has not changed. Booleans and state strings are free while
  steady. NaN is never equal to itself and logs every call, which is why estimate ages and heading
  error sit on the slow tier.
* DogLog's default `ntPublish` is already "not on FMS", but an explicit `true` passed to
  `Telemetry.start` overrides that. Getting that flag right matters when attached to the FMS.
* Any new Elastic widget needs a `Telemetry.logDash` or `logDashAlways` behind its topic. The layout
  in `src/main/deploy/elastic-layout.json` is the list of what must stay live.
* Java `refreshAll` reports errors under `ctre.phoenix6.BaseStatusSignal.refreshAll` (dots,
  lowercase). `ctre::phoenix6::BaseStatusSignal::WaitForAll` with a C++ stack is the native odometry
  thread. That distinction settled the misdiagnosis that evening.
* The hoot path in `SwerveConfig` (`./logs/spectrum.hoot`) is a simulation replay file, not a
  recording location. Hoot recording on the rio is Phoenix auto-logging.
* Elastic is Dart, so geometry and ranges in the layout JSON must stay doubles (`512.0`, not
  `512`), and divisions and colors must stay ints. The edits that evening used a node script that
  round-trips the file byte-identical before changing anything. Hand-edit it or let Elastic save
  it. Do not run it through a generic JSON formatter.
* Gradle on the machine used that evening picked a JDK 25 from PATH and failed with "Unsupported
  class file major version 69". Pass the WPILib JDK with
  `-Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk"` and set `JAVA_HOME` to the same.
  `--offline` works.
* `logs/FRC_20260905_013127.wpilog` in this repo is a simulation log, not the robot.

## Results from the shop laptop, 2026-09-05 22:00 to 22:20

*Robot on the bench at 10.85.15.2 (team number 8515 in `.wpilib/wpilib_preferences.json`; the 3847
addresses do not answer). Disabled the whole time. The rio clock is UTC, so rio log names are five
hours ahead of the Driver Station file names.*

**The -1003 and stale-signal errors are gone with the revert deployed.** Deployed at 22:07:56. Nine
minutes disabled: zero `WaitForAll -1003`, zero `1000 CAN message is stale`, zero talon fx 18
errors. Same bench, same wiring, same evening.

What was actually running before that: the jar on the rio had been built at 21:27:56, before the
revert existed, so the 21:28 Driver Station session was the **real-time priority build**. Its
numbers, against the old-code session just before it and the fixed build after:

|  DS session   |               Code                | RIO CPU med | CANivore bus |  Loop body med  |       -1003        | Stale warnings |
|---------------|-----------------------------------|-------------|--------------|-----------------|--------------------|----------------|
| 20:53, 7 min  | old code                          | 95%         | 67%          | 14.1 ms         | 11 (2/min)         | 16             |
| 21:28, 31 min | alerts build, plus RT priority 99 | 57%         | 46%          | 2.8 ms          | 116 (18/min early) | 682            |
| 22:07, 9 min  | revert in                         | 62%         | 46%          | 5 ms warming up | **0**              | **0**          |

The 21:28 errors were front-loaded: 22 in the first minute, then 14, 7, 7, 11, 7, then about one a
minute after ten minutes. That is the JIT warming up. Early on the loop body is long and interpreted,
a `SCHED_FIFO` 99 main thread holds a core for all of it, and Phoenix's threads miss their turn. As
the body shrank the errors thinned out. Bus utilization was already down to 46 percent in that
session, so the bus was not what was tripping the odometry thread. The priority was. The revert
fixed it, and nothing else changed between the two builds.

**The daytime and 09-04 rates were mostly talon fx 18.** In the 09-04 23:51 session the -1003 errors
came every 3.00 s for 40 minutes, 810 of them, and 774 were each followed within half a second by
`-10021 Device firmware could not be retrieved` for talon fx 18. Phoenix retried the unpowered
follower every three seconds, and each retry stalled the frame path long enough to trip the odometry
thread's two-period timeout. Talon 18 has power now and neither error has appeared since. Sessions
with talon 18 quiet had 0 to 4 -1003s in total.

**Odometry stays at 250 Hz.** No reason to drop to 200 or 150 Hz on this evidence. Revisit only if
-1003s come back while enabled with the bus under 55 percent.

### What the rio looks like now (disabled, warm, from `/proc`)

* Two cores. Busy 69 to 73 percent by `/proc/stat` over 30 s, of which user 40, sys 23 to 27,
  softirq 5 to 7. The Driver Station reports 62 percent median for the same build. The system time
  is CAN and USB work, not Java.
* About 23,000 context switches a second.
* The CANivore enumerates at **USB Full Speed, 12 Mbps** (`/sys/bus/usb/devices/1-1.2/speed`),
  behind the rio's internal hub, with 1,100 USB interrupts a second. That is what the device is. It
  is not a fault, but every CAN frame batch pays a 1 ms USB frame.
* Hottest threads (percent of one core, 10 s sample): main robot thread 35; Phoenix odometry thread
  (`SCHED_RR` priority 1) 16; two CANivore transport threads (`SCHED_RR` 3 and 2) 14 and 11; two
  unnamed native threads at normal priority 15 and 8; DogLog log thread 11; rio CAN `can_recv` 5;
  PathPlanner `ADStar Planning` 4. The main thread is at normal priority (policy 0), which confirms
  the revert is what is running. WPILib's HAL notifier is the one `SCHED_FIFO` thread, at priority
  40, and is idle.
* The main thread at 35 percent with a 3 to 5 ms loop body is more than the body accounts for. The
  rest is the loop's own overhead outside `robotPeriodic` (DS refresh, SmartDashboard, LiveWindow)
  plus JIT early on. Worth a look if CPU needs to come down further. Not related to the CAN errors.
* Seven loops over 200 ms in the first eight minutes disabled: the deliberate full collection every
  60 s while disabled, plus PathPlanner warmup. Expected.

### Still to do

1. Enable and drive with the revert deployed or later. Everything above is disabled-only. The
   enabled loop and bus numbers at the top of this file are still from the old code.
2. Archive that night's logs: rio `FRC_20260906_014905` (old code, 20:49 to 21:27),
   `FRC_20260906_022826` (priority build, 21:28 to 22:07), `FRC_20260906_030756` (revert), and the
   Driver Station files `2026_09_05 20_53_13`, `21_28_03`, `22_07_36`.
3. Commit or drop the uncommitted `setUpdateFrequencyForAll` on the swerve module current signals if
   it is still in someone's tree. It was not on that laptop.
4. `scripts/dslog.js` counts the `Tracer` epoch prints as loop overruns, so its "loop overrun prints"
   number is high by those.

### Enabled on the cart, 22:54 to 23:47 (two teleop runs, 185 s and 183 s, no stick input, turret belt off)

* **CAN:** two `WaitForAll -1003` in the first run (t=1560 and t=1658 in
  `FRC_20260906_030756.wpilog`, one with stale Position/Velocity on talon fx 2), none in the second.
  0.3 per minute enabled overall. Not correlated with CPU bursts.
* **Load enabled:** CPU 88 to 92 percent busy by both the load monitor and a `/proc` sampler on the
  rio (disabled: 65 to 70). CANivore bus 58 percent (disabled 46). Loop period median 20.1 ms, 10 to
  13 percent of loops over 25 ms, body median 7.2 ms. The rise is the swerve control frames: with
  teleop drive active, Phoenix sends eight `TorqueCurrentFOC` requests every 4 ms regardless of stick
  input, and the kernel USB work that comes with them is the roughly 40 percent of one core that no
  thread owns (`irq/53-e0002000` shows 15 percent, the rest is hardirq/softirq on the `cpu` line).
* **Thread split enabled** (percent of one core, 3 min average): main 22, Phoenix odometry
  (`SCHED_RR` 1) 23, CANivore transport threads 10 and 6, two unnamed native threads 9 and 4, DogLog
  8, `FRC_NetCommDaemon` 5, eth irq 4.
* **Unexplained once:** in the first run, seven-second stretches at t=1518 and t=1579 (60 s apart, 22 s
  after enable) where every loop was 30 to 130 ms and every section stretched together, vision 2 to
  40 ms, scheduler 4 to 60 ms. Pure contention from something else, not GC (no full collection in
  this JVM while enabled, `Gc/MsPerSecond` flat). It did not recur in the second run with the
  sampler watching. If it comes back, `/tmp/sampler.sh` on the rio (one awk pass per second into
  `/home/lvuser/sample.txt`) will name the thread. Parse it with the pattern in that session's
  scripts, watching for the double space after `cpu` in `/proc/stat`.
* **Subsystems behaved as designed:** the launcher held its idle prep speed and voltage, the dye rotor
  idled slowly, the turret tracked from odometry with no tags and covered 1028 degrees of travel with
  the belt off, hood and tower were off, and the swerve held on the cart at 4 A drive and 6 A steer
  stator. Alerts: only gamepads disconnected and pose not vision-seeded. Battery 11.85 V, known low.
* **Next lever if CPU headroom matters:** drop swerve odometry to 200 Hz. That cuts the odometry
  thread, the control frames, the USB interrupts, and the bus by a fifth. It is not needed for the
  CAN errors, because those were the priority build and talon 18.
