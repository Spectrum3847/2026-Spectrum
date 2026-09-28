# Exception Handling

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

Robot code runs in a single process that's responsible for moving a lot of mass around. Crashing during a match is the worst outcome, and silently swallowing an exception that explains *why* the robot is misbehaving is a close second. The aim here is to fail loudly when something is wrong, recover gracefully when recovery is meaningful, and never hide errors from the log.

## Never write an empty catch

> "Anytime somebody has an empty catch clause they should have a creepy feeling. There are definitely times when it is actually the correct thing to do, but at least you have to think about it." Attributed to James Gosling.

If you have a reason to catch and not log, write that reason down in a one-line comment so a future reader sees you thought about it. Otherwise, at minimum: report through `Telemetry.print` at `HIGH` priority, and print the stack trace.

Both halves matter. `Telemetry.print` ends up in the WPILOG *and* on the Driver Station console, so it survives the match. `e.printStackTrace()` adds the full trace to stderr, and stderr is captured because console capture is on (see `Telemetry.start(...)` in [DogLog](../dependencies/doglog.md)).

## Catch the specific exception

Catching `Exception` or `Throwable` at the top of a method silences `NullPointerException`, `ClassCastException`, and `OutOfMemoryError` along with whatever you actually wanted to catch. Those generic exceptions almost always mean a real bug. Let them propagate so the WPILib robot wrapper can log them and the match continues with whatever still works.

Catch the specific exceptions, join them with `|`, and make the message name the failure mode. `Robot`'s auto preview does this when it loads the path group for the selected auto: it catches `IOException` and `ParseException` around the call and logs that the path planner paths could not be loaded. A missing or malformed file must never leave the auto silently doing nothing.

## Prefer validating inputs over catching NullPointerException

`NullPointerException` is almost never the right thing to catch. If a value can be null, check for null.

`Auton.getAutonomousCommand()` is the model: it takes the chooser's selection and falls back to a print command when there is none. Explicit check, explicit fallback, no `catch (NullPointerException)`, and the failure shows up on the Driver Station instead of throwing during the auto period.

The same logic covers `ArrayIndexOutOfBoundsException`. Bounds-check before indexing rather than trapping the throw.

## When try-catch is the right tool

You want `try-catch` when:

* You are at a boundary with code you do not control: file I/O, network calls, vendor SDK methods that declare checked exceptions.
* The recoverable behavior is meaningful: fall back to a default, retry once, switch to a degraded mode.
* You want to *log and continue* rather than crash. Per-loop sensor reads are a common case: a bad read should produce a sentinel value and a log line, not end the loop.

You do not want `try-catch` when:

* The exception indicates a bug in your code. Let it crash, then fix the bug.
* You are catching it only to log and rethrow. Use the underlying logging hook instead, or let it propagate.

## Log the exception, not just its message

`e.getMessage()` alone often loses the chain. Log the exception object, and print the stack trace.

Concatenating `e` calls `toString()`, which includes the exception class name. `printStackTrace()` includes the call site. Both together let someone reading the log six weeks later work out what happened.

## Faults

For a known failure mode, log the name of the matching `Telemetry.Fault` entry rather than a sentence describing it, so it is easy to grep after a match. The enum is the shared vocabulary, and it is in the source; read it there for what it currently holds and add an entry when a new fault class shows up. Named entries beat free-text searches through `Prints`.

## See Also

* [Logging](../tools/logging.md) for how `Telemetry.print` and `log` end up in the `.wpilog`.
* [Code Style](code-style.md) for the surrounding conventions.
