# Logging and data analysis

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

Robot logs are the difference between "the elevator stopped working at champs and we don't know why" and "the elevator stopped working at champs, here's the CAN dropout that caused it." We use [DogLog](https://doglog.dev) for the heavy lifting, wrapped by our own [`Telemetry`](../../src/main/java/frc/spectrumLib/telemetry/Telemetry.java) class to keep call sites short and add a few project-specific behaviours.

## What reaches the dashboard

`Telemetry` extends DogLog, registers itself as a `Subsystem` so its `periodic()` runs every loop, and is started from `Robot.java`. A logged value only appears on NetworkTables if the code publishes it on purpose, and there are three tiers. Choosing the right one is the single most useful habit in this file.

`Telemetry.log(key, value)` is the default. The value goes in the wpilog and nowhere else. Most things in the codebase use this.

`Telemetry.logDash(key, value)` is the same, and additionally publishes the value to NetworkTables, but only on the slow tier: every fifth loop, which at a 50 Hz loop is 10 Hz. The slow tier is `Telemetry.SLOW_LOG_EVERY_LOOPS`, and `Telemetry.slowLogThisLoop()` is the predicate for it. Use this for anything an Elastic widget or the robot app reads, and wrap anything that does not need 20 ms resolution in `slowLogThisLoop()`: motor currents, temperatures, camera status, shot-calculator output while not launching.

`Telemetry.logDashAlways(key, value)` publishes on every call. Use it for values logged on their own slower cadence that might never land on a slow-tier tick: once at boot, once a second, the swerve alignment publisher.

Before you add a widget to the Elastic layout, make sure the key it reads is a `logDash` or `logDashAlways` call. The layout in `src/main/deploy/elastic-layout.json` is the list of what has to stay live, and a plain `log` key will not be there.

`Mechanism.logStandard(prefix, dashboardDiagnostics, rpm)` is the per-loop logging every mechanism calls: battery use, the current command, `logDiagnostics(prefix)` for voltage, currents, temperature and connection, and RPM at the rate the `RpmLog` argument picks. The one exception is voltage on a mechanism whose config sets `fastOutputLogging`: that is logged every loop, and its output status frames run at the control rate, so a feedforward fit from the log has a voltage sample for every velocity sample. The launcher, the launcher tower and the turret set it, and all three are mechanisms whose feedforward has been fit from a log.

The `Scheduler/*` loop timers are dashboard keys and are logged in seconds, the same units DogLog's own timers used.

## Mirroring to NetworkTables

Mirroring every logged value to NetworkTables is off on the robot and on in simulation, so an AdvantageScope session in the shop can watch everything while simming. The `Telemetry/MirrorLogsToNT` switch on the SmartDashboard turns the full mirror on, and it is ignored whenever the FMS is attached regardless of the switch. `Telemetry.start` takes the switch's starting position as its first argument, and `Robot.java` passes `RobotBase.isSimulation()`.

Do not turn it on during a match. On 2026-09-05 the mirror plus a NetworkTables flush every loop was a full-time job for one of the roboRIO's two cores. The dashboard gets its values the other way, through `logDash`, one key at a time.

`Telemetry.start` also configures the rest of DogLog's options once, from its remaining arguments: whether Driver Station messages and `System.out` are captured into the log, whether the PDH extras are included, and whether DogLog's own NT tunables are allowed with the FMS attached. The arguments in `Robot.java` are the settings, and they are one line you can read.

Note that the FMS flag governs DogLog's tunables only. [`TuneValue`](../../src/main/java/frc/spectrumLib/telemetry/TuneValue.java) uses `SmartDashboard` directly regardless, so tunables are a practice-only policy either way. See [PID Tuning](pid-tuning.md).

`Telemetry.logAlerts()` runs in `periodic()` and pulls anything published to NetworkTables under `SmartDashboard/Alerts` into the log with deduplication, so a flapping alert does not fill the disk.

## State transitions

State machines log their wanted and system state with `Telemetry.logState(key, state)`, which writes only on the loop a state changes. DogLog skips unchanged values anyway, so the wpilog comes out the same, but nothing is queued while a state holds and every transition keeps its exact loop timestamp.

`Telemetry.logStateDash` also publishes each change to NetworkTables immediately, which `logDash` could otherwise miss between its slow-tier ticks. That is why the super structure's current state uses it.

## Conventions

* Keys are `Subsystem/Name`. The hierarchical paths keep the NetworkTables tree navigable and group cleanly in AdvantageScope.
* Set the unit string when there is one. DogLog records it as metadata and AdvantageScope uses it for axis labels.
* Log inside the subsystem's `periodic()`, not scattered through command bodies. `periodic()` is the one place you know the value updates at the loop rate.
* DogLog handles the common overloads directly, including booleans, strings, arrays and WPILib geometry types. There is no need to convert.
* Do not log the same value many times within one loop. It is wasted disk, and disk on the RIO is finite.

`Telemetry.log(Command)` returns a decorated command that logs its init and its end. Wrap the *outermost* command factory, not every step inside it, or every internal sequence step gets its own pair of lines. You do not need to decorate anything to see which command owns a mechanism: each subsystem logs its own running command every loop.

## Console output

`Telemetry.print(message)` writes to stdout and logs to the same text with an FPGA timestamp. `Telemetry.print(message, PrintPriority.HIGH)` always reaches the console and the Driver Station; the default `NORMAL` priority only prints if the global priority is also `NORMAL`. Either way both land in the log.

Use it sparingly. Anything that should be in the log but does not need to be on a driver's screen, such as a sensor reading or a command transition, should be a `log` rather than a `print`. Reserve prints for initialization milestones and fault events.

`Telemetry.Fault` is a small enum of named conditions declared inside `Telemetry`. It is currently a *catalog* only: the values give the codebase a shared vocabulary for known failure modes, and there is no `logFault(...)` helper. When a known fault fires, print the enum name at high priority. The post-match grep is then much faster than scanning free text.

## Fast triage between matches

When something went wrong in a match and the next one is in ten minutes, do not start in AdvantageScope. Copy the newest log off the RIO and run the triage script from the `fast-log-triage` agent skill:

```sh
.agents/skills/fast-log-triage/scripts/pull_rio_logs.sh -n 1 ./rio-logs
python3 .agents/skills/fast-log-triage/scripts/triage_wpilog.py ./rio-logs
```

It needs only Python 3, no WPILib install and no Java. It makes one pass over the file and prints a ranked list of likely causes with timestamps, what to check on the robot, and what it ruled out. The skill page lists every topic and threshold it uses and maps drive-team symptoms to findings, so read that when you want to know what a finding means rather than what to do about it. Use it, or ask an agent to, before the slower topic-by-topic decoding in the `wpilog-decode` skill.

## Pulling logs off the RIO

`.wpilog` files land in `/U/logs/` on the roboRIO. [AdvantageScope](https://docs.advantagescope.org) can open one directly off the RIO, or download it. For a live session, AdvantageScope reads NetworkTables directly: start it before connecting Elastic, point it at the same robot, and it streams everything DogLog publishes.

## Committed match logs

Real match logs are committed under [`logs/matches/`](../../logs/matches/README.md), so every clone has real data for the robot app's tests, which parse each one, and for the triage script, with no second repository to hunt through. Only logs the FMS renamed with an event and match belong there; the rest of `logs/` stays gitignored. After each event:

```sh
python tools/copy-match-logs.py
```

It scans the usual laptop and DS log folders plus the repo's own `logs/` and `rio-logs/` scratch folders. It refuses anything over 50 MB and reads each copy back before keeping it. Then describe the log in the folder's README, saying what happened and what it is good for testing, and commit. Big logs and the full season's worth go to the [2026-Robot-Logs](https://github.com/Spectrum3847/2026-Robot-Logs) archive with `tools/archive-logs.sh`.

## One row per event

Not everything worth logging is a signal sampled over time. A burst of fuel and an operator's verdict on where it landed are events, and they are logged as one sparse row each rather than as another loop-rate stream. Reading them takes a little care, because DogLog skips a record whose value has not changed.

See [Shot Records and Trim Events](shot-log.md).

## See also

* [Shot Records and Trim Events](shot-log.md), the per-burst and per-trim rows, and how to join them.
* [Elastic Dashboard](elastic.md), the live NetworkTables view that reads from the same publish stream.
* `.agents/skills/fast-log-triage/SKILL.md`, the between-matches triage script, its detectors and thresholds.
* [DogLog](../dependencies/doglog.md) for the library's own API and the meaning of each option flag.
