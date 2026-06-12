# DogLog

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

DogLog is a small WPILib logger that writes WPILOG files on the roboRIO and (optionally) re-publishes everything to NetworkTables so dashboards can read it live. We extend it with our own [`Telemetry`](../../src/main/java/frc/spectrumLib/Telemetry.java) class, and that's the layer 99% of the code calls.

Version pinned: 2026.5.0 ([`vendordeps/DogLog.json`](../../vendordeps/DogLog.json)).

## The `Telemetry` Wrapper

`frc.spectrumLib.Telemetry` extends `dev.doglog.DogLog` and implements `Subsystem`, so the scheduler runs its `periodic()` once a loop. `Robot.java` boots it with:

```java
Telemetry.start(
    /* ntPublish      */ true,
    /* captureDs      */ true,
    /* captureNt      */ false,
    /* captureConsole */ true,
    /* logExtras      */ false,
    /* tunableOnFMS   */ true,
    PrintPriority.NORMAL);
```

Translated to `DogLogOptions`, that publishes everything to NetworkTables (so Elastic and SmartDashboard see it), captures DriverStation events and `System.out` into the WPILOG, and leaves NT tunables editable even on FMS. We also hand DogLog a `PowerDistribution` so PDH currents log automatically.

There's one piece outside of `start(...)` worth knowing: `SmartDashboard.putData(CommandScheduler.getInstance())` exposes the running-commands widget. Don't remove that — it's the fastest way to see what's actually scheduled when something looks wrong.

## Logging Values

```java
Telemetry.log("Subsystem/ValueName", value);
Telemetry.log("Subsystem/ValueName", value, "units");
```

Keys use `Subsystem/Path/Name` so both Elastic and AdvantageScope render them as a tree. Look at the existing keys (`Launcher/kP`, `Match Data/MatchTime`, `BuildConstants/GitSHA`) before inventing a new top-level — drift here is what makes logs unsearchable a season later.

DogLog has overloads for doubles, booleans, strings, arrays, and WPILib structs like `Pose2d` and `ChassisSpeeds`. Just call `log` and pass the value.

## Logging Commands

```java
Auton.autonUnjam.onTrue(
    Telemetry.log(
        Commands.sequence(/* ... */)));
```

`Telemetry.log(Command)` wraps a command so it writes `Init:` and `End:` entries to the `Commands` key. Only wrap the outermost command in a group — wrapping inner ones just produces log spam.

## Console Prints

`Telemetry.print(...)` does three things at once: stamps the time, decides whether to print to stdout based on a `PrintPriority`, and logs the line under `Prints` so it survives in the WPILOG even if console capture is off. Use it instead of `System.out.println` — bare prints disappear unless `captureConsole` is on, and they don't get a timestamp.

Two priorities exist:

* `HIGH` always prints to the console.
* `NORMAL` only prints if the global priority is set to NORMAL.

Both always log.

## Alerts

`Telemetry.logAlerts()` runs every periodic tick and scrapes `SmartDashboard/Alerts` for new error/warning/info strings, mirroring anything new into the `Alerts` log key. So the normal WPILib pattern still works:

```java
private static final Alert lowBattery = new Alert("Battery below 12V", AlertType.kWarning);

// later
lowBattery.set(voltage < 12.0);
```

The dashboard shows it, the log captures it, no extra plumbing.

## Build Stamps

`Robot.robotInit` writes `BuildConstants/ProjectName`, `BuildDate`, `GitSHA`, `GitDate`, and `GitBranch`. Those come from the `BuildConstants.java` that the `gversion` Gradle task regenerates on every `compileJava`. When you pull up a log a week later trying to figure out why a robot misbehaved, those five keys are how you know which build was on it.

## Faults

`Telemetry.Fault` is an enum for conditions worth tracking by name rather than by stringly-typed key:

```java
public enum Fault {
    CAMERA_OFFLINE,
    AUTO_SHOT_TIMEOUT_TRIGGERED,
    BROWNOUT,
}
```

Add to this enum when you find yourself logging the same fault from multiple files.

## Things That Have Bitten Us

`ntPublish=true` is convenient but it's not free — NT bandwidth is shared with everything else on the bus. If a match-day log gets noisy, log fewer high-frequency values before reaching for `withNtPublish(false)`.

`withNtTunables(true)` controls DogLog's own tunable entries; it does not affect `TuneValue`/`SmartDashboard` writes. If you want to prevent match-day tuning, remove or guard `TuneValue` call sites separately.

Don't add instance methods to `Telemetry`. It's a static façade on purpose — once half the codebase calls `telemetry.log(...)` and the other half calls `Telemetry.log(...)`, both sides are wrong forever.

## Further Reading

[DogLog JavaDoc](https://javadoc.doglog.dev) is wired into our generated JavaDoc site. The [README](https://github.com/jonahsnider/doglog) covers feature flags we haven't touched. For the bigger picture on what to log and when, see [Logging](../tools/logging.md).
