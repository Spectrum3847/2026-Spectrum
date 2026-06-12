# WPILib

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

WPILib is the foundation. Pretty much every Java file in the repo imports something from it: `TimedRobot`, the command-based scheduler, the math/geometry types, sensors, sim hooks, SmartDashboard, NetworkTables, the deploy pipeline. If a feature isn't covered here, the [WPILib docs](https://docs.wpilib.org/) probably do.

The command-based extension lives in [`vendordeps/WPILibNewCommands.json`](../../vendordeps/WPILibNewCommands.json). Everything else is pulled in by GradleRIO directly from `build.gradle` — there's no JSON for "WPILib core."

## Packages We Touch Most

`edu.wpi.first.wpilibj2.command` is the heart of the codebase: `Command`, `Subsystem`, `Trigger`, and the `Commands.*` factories. If you're writing new behavior, you start here.

For math: `edu.wpi.first.math.*` gives you `Pose2d`, `Rotation2d`, `ChassisSpeeds`, `MathUtil`, `ProfiledPIDController`, and `Matrix`/`VecBuilder` (which we use to weight vision std-devs). The newer `edu.wpi.first.units.*` package is creeping in too — `Inches.of(...)`, `MetersPerSecond.of(...)` show up in `RobotSim` and the swerve sim. Use it on new code that has natural units; older code mixes raw doubles with `Units.inchesToMeters(...)` and that's fine to match.

For hardware: `edu.wpi.first.wpilibj` has `DriverStation`, `RobotBase`, `Alert`, `Timer`, `Notifier`, `AddressableLED`, and `Mechanism2d`. For dashboards: `smartdashboard.SmartDashboard` and `SendableChooser` (the auto chooser uses both). For the field tag layout: `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)` is the one call that nails our vision system to the 2026 field.

## Subsystems

Don't extend `SubsystemBase` directly for a TalonFX mechanism. Use [`frc.spectrumLib.mechanism.Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java), which gives you config, signal caching, and command factories all in one. The vision system is the lone exception — `VisionSystem` extends `SubsystemBase` because there's no motor to wrap.

## Commands and Triggers

Almost everything in this codebase is glued together by `Trigger`. The canonical example lives in [`RobotStates.setupStates()`](../../src/main/java/frc/robot/RobotStates.java):

```java
pilot.RT.onTrue(
        Commands.either(applyState(State.INTAKE_FUEL), Commands.none(), pilot.LT.negate()));
```

A few habits worth picking up:

Prefer `Commands.either` / `runOnce` / `sequence` over hand-built `InstantCommand` and `SequentialCommandGroup` instances. They read better and compose cleanly with `Trigger.onTrue`/`whileTrue`. Always tag a command with `.withName("...")` — that name is what shows up in DogLog and in the running-commands widget. Use `.ignoringDisable(true)` only when you really need disabled-state behavior; the LED default command is the usual case.

If you're tempted to call `CommandScheduler.getInstance().schedule(...)` from inside a `periodic()` method, bind a trigger instead. Manual scheduling sidesteps `requirements`, and `requirements` is what stops two commands from fighting over the same subsystem.

## Telemetry and Tunables

We use SmartDashboard for two things: live dashboards (`Auto Chooser`, `Mechanism2d` views), and tunable knobs via `SmartDashboard.putNumber` / `getNumber`. The tunable case is wrapped by [`TuneValue`](../../src/main/java/frc/spectrumLib/TuneValue.java), so prefer that over hand-rolled `getNumber` reads.

For anything else — sensor readings, state transitions, fault flags — go through `Telemetry.log(...)` rather than `SmartDashboard.put*`. `Telemetry` routes everything through DogLog with a consistent key format and writes it to the WPILOG so the log survives the match.

## Simulation

`RobotBase.isSimulation()` and `Utils.isSimulation()` (from Phoenix) both work; the file you're editing usually dictates which to use. Override `simulationPeriodic()` on a `SubsystemBase` for sim-only updates — that's where the vision sim runs its `visionSim.update(...)`. For per-mechanism visualization, hand a `Mechanism2d` ligament out of `RobotSim` instead of standing up new widgets per subsystem.

## Alerts

WPILib's `Alert` API integrates automatically with `Telemetry.logAlerts()`. Create an alert once at the top of a class, then flip it on and off:

```java
private static final Alert lowBattery = new Alert("Battery below 12V", AlertType.kWarning);

lowBattery.set(voltage < 12.0);
```

`Rio.java` already uses this pattern for the "unknown RIO" warning at boot — copy that style.

## Things to Watch For

`addVisionMeasurement` only makes sense from a periodic-style update, not from a one-shot command. Calling it sporadically makes the pose estimator drift in unpredictable ways.

`Notifier` versus `addPeriodic`: the swerve sim uses `Notifier` because it needs tight, regular updates. For everything else, `TimedRobot.addPeriodic(...)` is the simpler choice.

`requirements` on commands cancel anything else using that subsystem. That's usually what you want, but be aware of it when chaining a state-change `runOnce` to a real command — if you set requirements on the wrong half, you cancel the wrong thing.

## Further Reading

[WPILib Documentation](https://docs.wpilib.org/) is the concept-level guide; the [JavaDoc](https://github.wpilib.org/allwpilib/docs/release/java/) is wired into our own generated docs. Before writing a new subsystem from scratch, skim the [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) chapter — most of the patterns above will make more sense afterward.
