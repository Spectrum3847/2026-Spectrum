# Command Compositions

> Applies to: WPILib 2026
> Source: [Command Compositions](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html)

## Contents

- [Composition Types](#composition-types) — sequence, parallel, race, deadline, repeat
- [Requirements](#requirements) — requirement union behavior
- [End Condition Decorators](#end-condition-decorators) — timeout, until, onlyWhile, onlyIf
- [End Behavior Decorators](#end-behavior-decorators) — finallyDo, handleInterrupt
- [Conditional Commands](#conditional-commands) — either, SelectCommand
- [ProxyCommand](#proxycommand)
- [Disabled Behavior & Cancellation](#disabled-behavior--cancellation)

---

## Overview

Compositions combine commands into a single command object. They can be nested recursively. Prefer factory methods and decorators over subclassing composition groups.

**Critical constraint**: A command instance can only belong to one composition. Passing the same instance to multiple compositions throws an exception.

---

## Composition Types

| Type | Ends when | Factory | Decorator |
| ---- | --------- | ------- | --------- |
| Sequential | All members finish in order | `Commands.sequence(...)` | `.andThen(...)`, `.beforeStarting(...)` |
| Parallel (all) | ALL members finish | `Commands.parallel(...)` | `.alongWith(...)` |
| Parallel race | ANY member finishes | `Commands.race(...)` | `.raceWith(...)` |
| Parallel deadline | The *deadline* command finishes | `Commands.deadline(deadline, ...)` | `.deadlineWith(...)` |
| Repeat | Never (runs until interrupted) | `Commands.repeatingSequence(...)` | `.repeatedly()` |

### Sequential

```java
Commands.sequence(
  intake.runIntakeCommand(1.0).withTimeout(2.0),
  shooter.spinUpCommand(),
  shooter.fireCommand()
)
```

### Parallel (all finish)

```java
Commands.parallel(
  drivetrain.driveCommand(0.5, 0.5),
  intake.runIntakeCommand(1.0)
).withTimeout(3.0)
```

### Parallel race (first to finish wins)

```java
drivetrain.driveCommand(0.5, 0.5).raceWith(new WaitCommand(5.0))
```

### Parallel deadline (primary command defines end)

```java
// Stops spinning intake when drive-to-position completes
drivetrain.driveToPositionCommand(pos).deadlineWith(intake.runIntakeCommand(1.0))
```

---

## Requirements

Compositions automatically inherit the union of all member requirements. A composition touching drivetrain, intake, and shooter reserves all three for its entire duration—even if only one runs at a time (e.g. in a sequence).

---

## End Condition Decorators

| Decorator | Behavior |
| --------- | -------- |
| `.withTimeout(seconds)` | Interrupts command after N seconds |
| `.until(BooleanSupplier)` | Interrupts command when condition becomes true |
| `.onlyWhile(BooleanSupplier)` | Interrupts command when condition becomes false |
| `.onlyIf(BooleanSupplier)` | Skips command entirely if condition is false at schedule time |

---

## End Behavior Decorators

| Decorator | Behavior |
| --------- | -------- |
| `.finallyDo(BooleanConsumer)` | Runs lambda after command ends; boolean = `true` if interrupted |
| `.handleInterrupt(Runnable)` | Runs lambda only on interruption |

---

## Conditional Commands

```java
// Run one of two commands based on a condition
Commands.either(commandIfTrue, commandIfFalse, limitSwitch::get)

// More general: select from a map at runtime
new SelectCommand<>(Map.of(State.A, commandA, State.B, commandB), this::getState)
```

---

## ProxyCommand

Schedules a command through the scheduler rather than directly inside the composition. Prevents the composition from inheriting that command's requirements.

**Use sparingly.** If any command for a subsystem is proxied, all commands for that subsystem within that composition must be proxied to avoid conflicts.

```java
someCommand.asProxy()  // decorator form
```

---

## Disabled Behavior & Cancellation

- A composition runs while disabled only if **all** members have `runsWhenDisabled = true`.
- A composition is `kCancelIncoming` only if **all** members are `kCancelIncoming`; otherwise it is `kCancelSelf`.
