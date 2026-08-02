# Command Scheduler

> Applies to: WPILib 2026
> Source: [The Command Scheduler](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html)

## Overview

`CommandScheduler` is a singleton that manages command lifecycle, subsystem periodic calls, and trigger polling. Access via `CommandScheduler.getInstance()`.

**Required**: Call `CommandScheduler.getInstance().run()` from `robotPeriodic()`. Without this, nothing runs.

---

## Per-Iteration Run Sequence

Each call to `run()` executes in this order:

1. Call `periodic()` on all registered subsystems (+ `simulationPeriodic()` in sim)
2. Poll all registered triggers; schedule bound commands
3. For each scheduled command:
   - Call `execute()`
   - Check `isFinished()`; if true, call `end(false)` and de-schedule
4. Schedule default commands for subsystems with no active command

---

## Scheduling a Command

`schedule(command)` logic:

1. Rejects commands that are part of a composition
2. No-ops if: scheduler is disabled, command is already scheduled, or robot is disabled and command doesn't have `runsWhenDisabled`
3. Checks requirement conflicts with currently running commands
4. Interrupts conflicting commands (calls `end(true)`) before calling `initialize()` on the new command

**Interrupt order**: `end(true)` of the interrupted command completes before `initialize()` of the incoming command.

---

## Default Commands

Register with `subsystem.setDefaultCommand(command)`. Scheduled automatically when the subsystem has no other active command. Must require that subsystem. Restarted (re-initialized) each time it becomes the active command.

---

## Event Callbacks

Hook into scheduler events for logging or telemetry:

| Method | Fires when |
| ------ | ---------- |
| `onCommandInitialize(consumer)` | Command starts |
| `onCommandExecute(consumer)` | Each iteration during execution |
| `onCommandFinish(consumer)` | `isFinished()` returns true |
| `onCommandInterrupt(biConsumer)` | Command is canceled or interrupted |

---

## Control Methods

| Method | Effect |
| ------ | ------ |
| `disable()` | Pauses all scheduling (commands mid-run continue until finished) |
| `enable()` | Resumes scheduling |
| `cancel(command)` | Explicitly stops a running command; calls `end(true)` |

---

## Key Behaviors

- Commands execute in scheduling order within a single iteration—one command's `end()` completes before another's `execute()` runs.
- Subsystem `periodic()` always runs before any command `execute()`.
- Default commands are scheduled at the **end** of the iteration, after command execution.
