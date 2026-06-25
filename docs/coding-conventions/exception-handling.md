# Exception Handling

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

Robot code runs in a single process that's responsible for moving a lot of mass around. Crashing during a match is the worst outcome; silently swallowing an exception that explains *why* the robot is misbehaving is a close second. The aim of exception handling here is to fail loudly when something is wrong, recover gracefully when recovery is meaningful, and never hide errors from the log.

## Never Write an Empty Catch

> "Anytime somebody has an empty catch clause they should have a creepy feeling. There are definitely times when it is actually the correct thing to do, but at least you have to think about it." — James Gosling

If you have a reason to catch and not log, write it down in a one-line comment so a future reader sees you thought about it. Otherwise, at minimum:

```java
} catch (IOException e) {
    Telemetry.print("Failed to load auto file: " + e.getMessage(), PrintPriority.HIGH);
    e.printStackTrace();
}
```

Use [`Telemetry.print(..., PrintPriority.HIGH)`](../tools/logging.md) — it ends up in the WPILib log *and* on the Driver Station console. `e.printStackTrace()` adds the full trace to stderr, which is also captured if `captureConsole` is on (it is, see `Telemetry.start(...)`).

## Don't Catch `Exception` Broadly

Catching `Exception` or `Throwable` at the top of a method silences `NullPointerException`, `ClassCastException`, and `OutOfMemoryError` along with whatever you actually wanted to catch. Those generic exceptions almost always mean a real bug — let them propagate so the WPILib robot wrapper can log them and the match continues with whatever it can.

Catch the *specific* exception you're handling:

```java
try {
    path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
} catch (FileVersionException | IOException | ParseException e) {
    // Pattern from frc.robot.auton.Auton.followSinglePath
    e.printStackTrace();
    return new PrintCommand("ERROR LOADING PATH");
}
```

This is the actual pattern in [`Auton.followSinglePath`](../../src/main/java/frc/robot/auton/Auton.java) — three specific exceptions, joined with `|`, fallback to a print command that names the failure mode.

## Prefer Validating Inputs Over Catching `NullPointerException`

`NullPointerException` is almost never the right thing to catch. If a value can be null, check for null:

```java
Command auton = pathChooser.getSelected();
if (auton != null) {
    return auton;
}
return new PrintCommand("*** AUTON COMMAND IS NULL ***");
```

That's how `Auton.getAutonomousCommand()` handles "what if nothing's selected" — explicit check, explicit fallback, no `try { … } catch (NullPointerException)`.

Same logic for `ArrayIndexOutOfBoundsException`: bounds-check before indexing rather than trapping the throw.

## When `try-catch` Is the Right Tool

You want `try-catch` when:

* You're at a boundary with code you don't control — file I/O, network calls, vendor SDK methods that declare checked exceptions.
* The recoverable behavior is meaningful — fall back to a default, retry once, switch to a degraded mode.
* You want to *log and continue* rather than crash. Per-loop sensor reads are a common case: a bad read shouldn't crash the loop, just produce a sentinel value and log the issue.

You don't want `try-catch` when:

* The exception indicates a bug in your code. Let it crash; fix the bug.
* You're catching it just to log and re-throw. Use the underlying logging hook instead, or let it propagate.

## Logging the Exception, Not Just Its Message

`e.getMessage()` alone often loses the chain. Prefer:

```java
Telemetry.print("Vision: failed to decode tag: " + e, PrintPriority.HIGH);
e.printStackTrace();
```

`e.toString()` (which `+ e` calls) includes the exception class name; `printStackTrace()` includes the call site. Both together let someone reading the log six weeks later figure out what happened.

## Faults

For known failure modes, use the named entries in `Telemetry.Fault` (`CAMERA_OFFLINE`, `AUTO_SHOT_TIMEOUT_TRIGGERED`, `BROWNOUT`). The enum is the shared vocabulary; log the fault via `Telemetry.print(Fault.X.name(), PrintPriority.HIGH)` so it's easy to grep after a match. Add new entries as new fault classes emerge — they're cheap and they make post-match analysis much faster than free-text searches through `Prints`.

## See Also

* [Logging](../tools/logging.md) for how `Telemetry.print` and `log` end up in the `.wpilog`, plus the `Telemetry.Fault` catalog.
* [Code Style](code-style.md) for the surrounding conventions.
