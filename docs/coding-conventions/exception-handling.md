# Exception Handling

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

Robot code runs in a single process that's responsible for moving a lot of mass around. Crashing during a match is the worst outcome; silently swallowing an exception that explains *why* the robot is misbehaving is a close second. The aim of exception handling here is to fail loudly when something is wrong, recover gracefully when recovery is meaningful, and never hide errors from the log.

## Never write an empty catch

> "Anytime somebody has an empty catch clause they should have a creepy feeling. There are definitely times when it is actually the correct thing to do, but at least you have to think about it." Attributed to James Gosling.

If you have a reason to catch and not log, write it down in a one-line comment so a future reader sees you thought about it. Otherwise log it and dump the trace:

```java
} catch (IOException e) {
    Telemetry.print("Failed to load auto file: " + e, PrintPriority.HIGH);
    e.printStackTrace();
}
```

Use [`Telemetry.print(..., PrintPriority.HIGH)`](../tools/logging.md); it ends up in the WPILib log *and* on the Driver Station console. `e.printStackTrace()` puts the call site in stderr, which is also captured because `Telemetry.start(...)` is called with console capture on.

## Don't catch `Exception` broadly

Catching `Exception` or `Throwable` at the top of a method silences `NullPointerException`, `ClassCastException`, and `OutOfMemoryError` along with whatever you actually wanted to catch. Those generic exceptions almost always mean a real bug; let them propagate so the WPILib robot wrapper can log them and the match continues with whatever it can.

Catch the *specific* exceptions you're handling and join them with `|`. The pattern to copy is in [`Auton.followSinglePath`](../../src/main/java/frc/robot/auton/Auton.java): the three exceptions that loading a PathPlanner path file can throw, one combined catch, and a fallback that names the failure mode instead of returning null.

## Prefer validating inputs over catching `NullPointerException`

`NullPointerException` is almost never the right thing to catch. If a value can be null, check for null and return an explicit fallback. [`Auton.getAutonomousCommand`](../../src/main/java/frc/robot/auton/Auton.java) is the reference: it checks `pathChooser.getSelected()` for null and returns a `PrintCommand` that says so, rather than letting the null travel.

Same logic for `ArrayIndexOutOfBoundsException`: bounds-check before indexing rather than trapping the throw.

## When `try-catch` is the right tool

You want `try-catch` when:

* You're at a boundary with code you don't control: file I/O, network calls, vendor SDK methods that declare checked exceptions.
* The recoverable behavior is meaningful: fall back to a default, retry once, switch to a degraded mode.
* You want to *log and continue* rather than crash. Per-loop sensor reads are a common case: a bad read shouldn't crash the loop, just produce a sentinel value and log the issue.

You don't want `try-catch` when:

* The exception indicates a bug in your code. Let it crash; fix the bug.
* You're catching it just to log and re-throw. Use the underlying logging hook instead, or let it propagate.

## Log the exception, not just its message

`e.getMessage()` alone often loses the chain. Log `e` itself, which is `e.toString()` and therefore includes the exception class name, and call `e.printStackTrace()` to get the call site. Both together let someone reading the log six weeks later figure out what happened.

## Faults

For known failure modes, add a named entry to the `Telemetry.Fault` enum in [`Telemetry.java`](../../src/main/java/frc/spectrumLib/telemetry/Telemetry.java) and log it with `Telemetry.print(Fault.X.name(), PrintPriority.HIGH)`. The enum is the shared vocabulary, so a fault is easy to grep after a match. New entries are cheap and make post-match analysis far faster than searching free text through `Prints`.

## See also

* [Logging](../tools/logging.md) for how `Telemetry.print` and `log` end up in the `.wpilog`.
* [Code Style](code-style.md) for the surrounding conventions.
