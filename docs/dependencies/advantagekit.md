# AdvantageKit

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md) and [Logging](../tools/logging.md).*

[AdvantageKit](https://docs.advantagekit.org) is Team 6328's logging framework. It records every input the robot code sees so a log can be replayed through the code later. We run it next to DogLog: DogLog (through `Telemetry`) is still what every subsystem logs to, and AdvantageKit owns the robot loop and records its own inputs.

Version pinned: 26.0.2 ([`vendordeps/AdvantageKit.json`](../../vendordeps/AdvantageKit.json)). The 27.x releases are alphas for the 2027 WPILib beta; don't move to them on a 2026 build.

## Where it shows up

* [`SpectrumRobot`](../../src/main/java/frc/spectrumLib/framework/SpectrumRobot.java) extends `org.littletonrobotics.junction.LoggedRobot` instead of `TimedRobot`. `LoggedRobot` runs the same 20 ms loop but has no `addPeriodic(...)`.
* [`Robot`](../../src/main/java/frc/robot/Robot.java) calls `startLogger()` first thing in its constructor. AdvantageKit has to start before anything reads hardware, or those reads are missing from the log.
* `build.gradle` adds the `akit-autolog` annotation processor (for `@AutoLog` input classes) and the `replayWatch` task.

## Modes

`Robot.MODE` is always `REAL` on the roboRIO. In simulation it is `Robot.SIM_MODE`, which is `SIM` unless you change it.

|   Mode   |                          Receivers                           |
|----------|--------------------------------------------------------------|
| `REAL`   | `WPILOGWriter` to `/home/lvuser/logs` (`akit_*.wpilog`)      |
| `SIM`    | `NT4Publisher`, so AdvantageScope can read it live           |
| `REPLAY` | reads a log, writes `<log>_sim.wpilog` next to it, no timing |

The real robot writes to the folder DogLog uses so `./gradlew archiveLogs` picks both files up. It does not publish AdvantageKit data to NetworkTables: we keep NT traffic to dashboard keys to spare the roboRIO CPU. Add `Logger.addDataReceiver(new NT4Publisher())` to the `REAL` case for a shop session that needs it live.

## Recording values

```java
Logger.recordOutput("Launcher/TargetRPM", targetRpm);
```

Outputs are not replayed; they are recomputed on replay. Inputs need an `@AutoLog` class and `Logger.processInputs(...)` (see [recording inputs](https://docs.advantagekit.org/data-flow/recording-inputs/)). Until a subsystem is split into an IO layer that way, replay only reproduces what AdvantageKit records on its own (Driver Station, joysticks, battery and system stats, loop timing), not the mechanisms.

## Replaying a log

1. Set `SIM_MODE = Mode.REPLAY` in `Robot.java` (don't commit it).
2. Open the log in AdvantageScope, or set `AKIT_LOG_PATH=<log.wpilog>`.
3. `./gradlew simulateJava`. The output lands next to the input as `<name>_sim.wpilog`.

`./gradlew replayWatch` reruns replay whenever the code changes.

## Links

* Docs: <https://docs.advantagekit.org>
* Javadoc: <https://docs.advantagekit.org/javadoc/>
* Releases: <https://github.com/Mechanical-Advantage/AdvantageKit/releases>
