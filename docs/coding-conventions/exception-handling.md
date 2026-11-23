# Errors and panics

*Audience: reference. Assumes you have read [Code style](code-style.md).*

Robot code moves a lot of mass. A crash in a match is the worst result. Hiding the error that explains a misbehaving robot is almost as bad. The goal is to fail loudly when something is wrong, recover when recovery makes sense, and never hide an error from the log.

Rust has no exceptions. There are two tools:

* **`Result` and `Option`** for things that can fail in normal use. The caller decides what to do.
* **`panic!`** for a bug, an impossible state or a broken setup. A panic unwinds the stack, and the code that catches it reports it.

## What happens on a panic

* **Building the robot.** `wpilib::start_robot` catches a panic while the robot is built. It reports "Unhandled exception instantiating robot" and the message to the driver station, and the program ends.
* **In the loop.** `start_robot` also catches a panic in the robot loop. It reports "Unhandled exception" and the message to the driver station, and the program ends.
* **`guarded` in `robot/robot.rs`.** Robot init, `robot_periodic` and the mode init methods run inside `guarded`. It writes the panic message to `/home/lvuser/crash_tracking.txt` through `CrashTracker` (`spectrum_lib/util/crash_tracker.rs`), then raises the panic again. The file is appended to, never rolled over, and it survives a reboot. Each line has a run ID, the time and the text. In simulation the write fails and prints a note.
* **Notifier threads.** A panic in a `Notifier` callback is reported to the driver station, and that notifier stops.
* **Sim steps.** A panic in a `sim_loop` step is reported as "SimLoop step failed", and the other steps still run.
* **Shutdown hooks.** A panic in one is caught, so the others still run.

So a panic in robot code ends the program. Plan for that. Do not rely on a panic to "continue with whatever works".

## Use `Result` for things that can fail

Return `Result` where failure is normal: loading a file, parsing JSON, creating a device. The `wpilib` and `phoenix6` constructors return `Result`. A getter that fails reports to the driver station and returns zero, and does not panic.

Handle the error where you can do something useful:

```rust
let robot_config = match RobotConfig::from_gui_settings() {
    Ok(config) => config,
    Err(error) => {
        driver_station::report_error(&format!("Failed to load PathPlanner config: {error}"));
        return;
    }
};
```

That is from `Swerve::configure_path_planner`. It reports the failure and goes on without paths. Do not write `let _ = ...` to throw away a `Result` without a reason. If you ignore one on purpose, write a short comment that says why.

## Report where people will see it

* `driver_station::report_error` and `report_warning` show in the driver station console and in the log.
* `HalError::report` sends a HAL error to the driver station.
* `telemetry::print_with(text, PrintPriority::High)` always reaches the console and the `Prints` key. See [Logging](../tools/logging.md).

## Do not use `unwrap` on a value that can fail

`unwrap()` and `expect()` panic when the value is `None` or `Err`. Use them only when failure means a bug, and then prefer `expect` with a message that says what was wrong. The code does this for setup that cannot work without it:

* Reading a context item before it is set panics with `"<name> is not set"` (`context.rs`).
* `Swerve::new` panics with "Could not create the swerve drivetrain" when the device setup fails.

Do not use `unwrap` on user input, on files, on a network call or on a device read. Use `match`, `if let`, `?` or a default.

For indexing, check bounds first or use `.get(i)`, which returns an `Option`.

## Locks and borrows

* A `Mutex` is poisoned when a thread panics while it holds the lock. The code uses `lock().unwrap_or_else(PoisonError::into_inner)` so one panic does not stop every later lock.
* A `RefCell` panics with "already borrowed" if you borrow it twice at once. Keep a borrow inside one method. See the borrow rule in [Code style](code-style.md).
* Dashboard calls from a thread other than the loop thread panic.

## Stopping the robot on purpose

`KillRobotException` (`spectrum_lib/util/exceptions/kill_robot_exception.rs`) is an error type for the case where continuing is unsafe. A caller that must stop the program does `panic!` with it. Nothing in the robot uses it today.

## Faults

`Fault` in `telemetry.rs` names known failure modes (`CameraOffline`, `AutoShotTimeoutTriggered` and `Brownout`). Nothing logs them yet. When you add a fault, log it with `telemetry::print_with` at `High` priority, using the name of the variant, so a search of the log finds it.

## Log the error, not only its message

Log the whole error value with `{error}` or `{error:?}`. `{error}` gives the text. `{error:?}` shows the type and fields. A message with no cause is hard to read six weeks later.

## See also

* [Logging](../tools/logging.md) for how prints and logs reach the `.wpilog`.
* [Code style](code-style.md) for the thread and borrow rules.
