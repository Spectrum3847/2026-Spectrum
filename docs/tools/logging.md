# Logging and Data Analysis

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

Robot logs are the difference between "the elevator stopped working at champs and we don't know why" and "the elevator stopped working at champs, here's the CAN dropout that caused it." We use [DogLog](https://doglog.dev) for the heavy lifting and wrap it with our own [`Telemetry`](../../src/main/java/frc/spectrumLib/Telemetry.java) class to keep call sites short and add a few project-specific behaviors.

## What Telemetry Is

`frc.spectrumLib.Telemetry` extends DogLog, registers itself as a `Subsystem` so its `periodic()` runs every loop, and starts via `Telemetry.start(...)` in `Robot.java`. The start call configures DogLog options once:

```java
// frc.robot.Robot
Telemetry.start(
    /* ntPublish     */ true,
    /* captureDs     */ true,
    /* captureNt     */ false,
    /* captureConsole*/ true,
    /* logExtras     */ false,
    /* tunableOnFMS  */ true,
    /* priority      */ PrintPriority.NORMAL);
```

Each flag maps to a `DogLogOptions` setter — toggling them changes what ends up in the `.wpilog` files on the RIO.

* `ntPublish` mirrors every logged value to NetworkTables so [Elastic](elastic.md) and AdvantageScope can see it live.
* `captureDs` / `captureConsole` snapshot Driver Station messages and `System.out` into the log.
* `logExtras` (PDH currents, CAN utilization, radio status) is currently off — flip to `true` when you want the extra noise for diagnostics.
* `tunableOnFMS` controls DogLog's NT tunables; `TuneValue` uses `SmartDashboard` (NetworkTables) regardless, so treat tunables as a practice-only policy and remove/guard them for competition.

`Telemetry.logAlerts()` runs in `periodic()` and pulls anything published to NetworkTables under `SmartDashboard/Alerts` (errors, warnings, infos) into the log file with deduplication, so a flapping alert doesn't fill the disk.

## Logging Values

Inside any subsystem `periodic()`:

```java
Telemetry.log("Launcher/RPM", getVelocityRPM(), "RPM");
Telemetry.log("Launcher/Voltage", getVoltage(), "volts");
Telemetry.log("Launcher/StatorCurrent", getStatorCurrent(), "amps");
```

That's the convention used in [`Launcher.java`](../../src/main/java/frc/robot/launcher/Launcher.java) and every other subsystem. A few things to notice:

* Keys are `Subsystem/Name`. Hierarchical paths make the NT tree navigable and group cleanly in AdvantageScope.
* The unit string (`"volts"`, `"amps"`, `"deg_C"`, `"RPM"`) is optional but worth setting — DogLog records it as metadata and AdvantageScope uses it on axis labels.
* DogLog handles all the common overloads (`double`, `boolean`, `String`, `Pose2d`, arrays). No need to convert.

For values that change per loop, prefer logging inside `periodic()` over scattering log calls in command bodies — `periodic()` is the one place you know the value updates at the loop rate.

## Logging Commands

`Telemetry.log(Command cmd)` returns a decorated command that logs `Commands: Init: <name>` when scheduled and `Commands: End: <name>` when it ends. Every `*States` file uses it:

```java
// LauncherStates.java
private static Command log(Command cmd) {
    return Telemetry.log(cmd);
}
```

That `log(...)` helper is a static convenience so command factories read `log(intakeFuel())` instead of `Telemetry.log(intakeFuel())`. The convention is shared across `LauncherStates`, `HoodStates`, `IntakeExtensionStates`, `PilotStates`, `IndexerTowerStates`, and so on.

Wrap the *outermost* command factory, not every sub-command — otherwise you get nested log lines for every internal sequence step.

## Console Output

`Telemetry.print(message)` writes to stdout *and* logs to the `Prints` topic with an FPGA timestamp:

```java
Telemetry.print("Launcher Subsystem Initialized");                       // NORMAL priority
Telemetry.print("AUTO_SHOT_TIMEOUT_TRIGGERED", PrintPriority.HIGH);      // always prints
```

The `PrintPriority.NORMAL` / `PrintPriority.HIGH` distinction filters console spam without dropping the log entry. `HIGH` always reaches stdout (and the Driver Station); `NORMAL` only prints if the configured priority is also `NORMAL`. Either way, the message hits the log.

Use it sparingly. Anything that should be in the log but doesn't need to be on a driver's screen — sensor readings, command lifecycle — should be `log(...)`, not `print(...)`. Reserve prints for true initialization milestones and fault events.

## Faults

`Telemetry.Fault` is a small enum of named conditions (`CAMERA_OFFLINE`, `AUTO_SHOT_TIMEOUT_TRIGGERED`, `BROWNOUT`) declared inside `Telemetry`. It's currently a *catalog* — the enum values exist so the codebase has a shared vocabulary for known failure modes, but there's no `logFault(...)` helper yet. When a known fault fires, log it as a high-priority print using the enum name: `Telemetry.print(Fault.CAMERA_OFFLINE.name(), PrintPriority.HIGH)`. Add new entries as new fault classes emerge; the post-match grep is much faster than scanning free-text strings.

## Pulling Logs Off the RIO

`.wpilog` files land in `/U/logs/` on the roboRIO. There's an [AdvantageScope](https://docs.advantagescope.org) tool for downloading and analyzing them; the workflow:

1. Plug in via USB (or Ethernet) to the RIO.
2. `AdvantageScope → File → Open Log...` to load a file directly off the RIO, or download via the AdvantageScope "Get logs" feature.
3. Drag NetworkTables paths from the sidebar into the timeline.

For a live session, AdvantageScope reads NetworkTables directly — start it before connecting Elastic, point it at the same robot, and it streams everything DogLog publishes.

## What to Log, What Not to Log

Log: motor voltages and currents, sensor readings, calculated setpoints, command lifecycle, state transitions, vision pose estimates, anything you'd want to graph after a match.

Don't log: anything inside a tight inner loop on every iteration (DogLog handles per-loop logging but logging the same value 50 times per loop is wasted disk). Don't log secrets — there aren't any in robot code, but the warning lives here as a reminder.

## See Also

* [DogLog dependency page](../dependencies/doglog.md) for version, JavaDoc link, and the option flags themselves.
* [Elastic Dashboard](elastic.md) — the live NetworkTables view that reads from the same publish stream.
