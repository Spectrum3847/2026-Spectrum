# Command-Based Architecture

> Applies to: WPILib 2026
> Sources: [Structuring a Command-Based Robot Project](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html) · [Organizing Command-Based Robot Projects](https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html)

## Contents

- [Project Structure](#project-structure) — `Robot`, `RobotContainer`, `Constants`, directory layout
- [Command Definition Patterns](#command-definition-patterns) — pattern selection, factory methods, subclassing
- [Common Anti-Patterns](#common-anti-patterns)

---

## Project Structure

| Class/Dir | Responsibility |
| --------- | -------------- |
| `Main` | Entry point (Java only). Do not modify. |
| `Robot` | Control flow. Keep minimal—declarative paradigm. |
| `RobotContainer` | Subsystems, button bindings, autonomous selection. Most setup lives here. |
| `Constants` | Global constants (speeds, PID gains, ports). Use inner classes per subsystem. |
| `Subsystems/` | User-defined subsystem classes |
| `Commands/` | User-defined command classes |

### Robot Class Essentials

- **Constructor**: Instantiate `RobotContainer`.
- **robotPeriodic()**: Must call `CommandScheduler.getInstance().run()`. See [command-scheduler.md](command-scheduler.md) for scheduler behavior.
- **autonomousInit()**: Schedule command from `robotContainer.getAutonomousCommand()`.
- **teleopInit()**: Cancel any running autonomous command.

Avoid imperative logic in `Robot`. Keep it declarative.

### RobotContainer

- Declare subsystems as **private fields** (not globals). Enables dependency injection and resource management.
- Pass subsystems to commands explicitly. Use `configureBindings()` for button→command mappings.
- `getAutonomousCommand()` returns the selected autonomous routine.

### Constants

- Use `public static final`. Group by subsystem or mode.
- Use `import static Constants.OIConstants.*` to avoid repeating the class name.

---

## Command Definition Patterns

When a command is reused (teleop bindings, autonomous, self-test), avoid duplicating inline definitions. Choose a pattern based on subsystem count and state.

### Pattern Selection

| Use Case | Preferred Pattern | Avoid |
| -------- | ----------------- | ----- |
| Single-subsystem, no internal state | Instance factory method | Inline everywhere; Command subclass |
| Single-subsystem, has internal state (e.g. PID) | Instance factory with captured state | Command subclass (unless logic is complex) |
| Multi-subsystem | Static or non-static command factory | Instance factory (causes circular deps) |
| Stateful, complex logic, multi-subsystem | Subclass `Command` | Inline/factory if it gets messy |
| Composite (sequence/parallel) | Factory method with `sequence`/`parallel` | Subclass `SequentialCommandGroup` (extra file per group) |

### Instance Factory Methods

**When**: Single-subsystem commands only.

```java
// In Intake subsystem
public Command runIntakeCommand(double percent) {
  return this.startEnd(() -> this.set(percent), () -> this.set(0.0));
}
```

Use: `intakeButton.whileTrue(intake.runIntakeCommand(1.0));` or in sequences. **Do not** use for multi-subsystem—causes circular dependencies.

### Static Command Factories

**When**: Multi-subsystem commands (e.g. autonomous routines).

```java
public static Command driveAndIntake(Drivetrain drivetrain, Intake intake) {
  return Commands.sequence(
    Commands.parallel(drivetrain.driveCommand(0.5, 0.5), intake.runIntakeCommand(1.0)).withTimeout(5.0),
    Commands.parallel(drivetrain.stopCommand(), intake.stopCommand())
  );
}
```

### Non-Static Command Factories

**When**: Many multi-subsystem routines; want to avoid repeating subsystem parameters.

```java
public class AutoRoutines {
  private final Drivetrain drivetrain;
  private final Intake intake;

  public AutoRoutines(Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
  }

  public Command driveAndIntake() {
    return Commands.sequence(
      Commands.parallel(drivetrain.driveCommand(0.5, 0.5), intake.runIntakeCommand(1.0)).withTimeout(5.0),
      Commands.parallel(drivetrain.stopCommand(), intake.stopCommand())
    );
  }
}
```

### Capturing State in Factory Methods

**When**: Command needs internal state (e.g. PIDController) but you prefer inline composition. State must be effectively final.

```java
public Command turnToAngle(double targetDegrees) {
  PIDController controller = new PIDController(Constants.kP, 0, 0);
  controller.setPositionTolerance(Constants.kTolerance);
  return run(() -> arcadeDrive(0, -controller.calculate(gyro.getHeading(), targetDegrees)))
      .until(controller::atSetpoint)
      .andThen(runOnce(() -> arcadeDrive(0, 0)));
}
```

### Subclassing Command

**When**: Significant internal state or complex multi-step logic; multi-subsystem; OOP style preferred. Verbose for simple commands—prefer factory methods when possible.

```java
public class TurnToAngleCommand extends Command {
  private final Drivetrain drivetrain;
  private final PIDController controller;
  private final double targetDegrees;

  public TurnToAngleCommand(Drivetrain drivetrain, double targetDegrees) {
    this.drivetrain = drivetrain;
    this.targetDegrees = targetDegrees;
    this.controller = new PIDController(Constants.kP, 0, 0);
    controller.setPositionTolerance(Constants.kTolerance);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.arcadeDrive(0, -controller.calculate(drivetrain.getHeading(), targetDegrees));
  }

  @Override
  public boolean isFinished() { return controller.atSetpoint(); }

  @Override
  public void end(boolean interrupted) { drivetrain.arcadeDrive(0, 0); }
}
```

### Subclassing Command Groups

**When**: Rarely. Prefer inline `Commands.sequence()` / `Commands.parallel()` inside a factory method. Subclassing produces one extra file per group and makes nested groups harder to read without providing meaningful benefits over factory composition.

```java
// Prefer this (factory):
public static Command scoreCoral(Drivetrain drive, Intake intake, Shooter shooter) {
  return Commands.sequence(
    drive.driveToPositionCommand(Constants.kScoringPos),
    intake.feedCommand(),
    shooter.fireCommand()
  );
}

// Avoid this (subclass adds no value here):
public class ScoreCoralCommand extends SequentialCommandGroup {
  public ScoreCoralCommand(Drivetrain drive, Intake intake, Shooter shooter) {
    addCommands(
      drive.driveToPositionCommand(Constants.kScoringPos),
      intake.feedCommand(),
      shooter.fireCommand()
    );
  }
}
```

---

## Common Anti-Patterns

| Anti-pattern | Problem | Fix |
| ------------ | ------- | --- |
| Calling `subsystem.method()` directly in `teleopPeriodic()` | Bypasses scheduler; conflicts with running commands | Bind to a command via trigger/button instead |
| Defining the same inline command in multiple places | Bugs fixed in one place silently persist elsewhere | Extract to an instance factory method on the subsystem |
| Constants scattered across subsystem files | Same port/gain defined in multiple places | Consolidate into `Constants` with inner classes per subsystem |
