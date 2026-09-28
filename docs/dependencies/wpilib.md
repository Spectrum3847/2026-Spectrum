# WPILib

*Audience: Reference. Assumes you've read [Dependencies overview](overview.md).*

WPILib is the substrate: the command scheduler, the math types, the simulation hooks, NetworkTables, the deploy pipeline. This page covers only the places where the team has made a decision. Anything not here, read [WPILib docs](https://docs.wpilib.org/).

The command-based extension is the only vendored piece, `vendordeps/WPILibNewCommands.json`. WPILib core comes straight from GradleRIO via `build.gradle`.

## Mechanisms, and the one direct Subsystem

Do not extend `SubsystemBase` for a TalonFX mechanism. Extend [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java), which is already a `Subsystem`, so it lands in the scheduler with no adapter. [`Vision`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) is the deliberate exception: it implements `Subsystem` directly because there is no motor to wrap and nothing for a base class to do.

## Commands and triggers

`Robot.configureBindings()` in [`Robot.java`](../../src/main/java/frc/robot/Robot.java) is the one place operator input becomes commands. It is a few dozen lines and it shows every pattern we use, so read it before adding a binding.

The habits that matter:

* Build commands from the `Commands.*` factories instead of hand rolled `InstantCommand` and `SequentialCommandGroup`. They compose with `onTrue`, `whileTrue`, and `onFalse` without extra wrapping.
* Tag every command with `.withName("...")`. That name is what DogLog prints and what the running commands widget shows. An untagged command logs as a class name, which is useless in a post match log.
* Reserve `.ignoringDisable(true)` for behavior that genuinely has to run while the robot is disabled. The real uses are swerve control requests, super structure state changes, the shot calculator nudges, and the disable time shift timer reset in `Robot.configureBindings()`. Putting it on a mechanism default command means that command keeps fighting for the mechanism while the robot sits disabled.
* Bind a trigger instead of calling `CommandScheduler.schedule(...)` from a `periodic()`. The scheduler does arbitrate between the two commands either way. The reason to bind a trigger is that a command the scheduler did not enqueue cannot stop one that was, so nothing will preempt a long-running command that a periodic method started. A trigger puts the start on the scheduler's own queue where its requirement and interruption rules apply.

## Units

`edu.wpi.first.units` is the newer style and is what new code should use. `Pounds.of(...)`, `Inches.of(...)`, and `Seconds.of(...)` return a `Mass`, `Distance`, or `Time` rather than a bare `double`, so the unit travels with the value and a caller cannot silently read metres as inches. Seven files use it: `SwerveConfig`, `Swerve`, `Robot`, `Pilot`, `SysID`, `MapleSimSwerveDrivetrain`, and `SpectrumLEDs`. `SwerveConfig` is the clearest example, where every module position is declared as `@Getter private Distance frontLeftXPos = Inches.of(wheelBaseInches / 2);` rather than a converted double. Older files still pass raw doubles through `Units.inchesToMeters(...)`, and several current files do both. Mixing the two inside one file is the thing to avoid. Match whatever the file you are editing already does.

## The season field layout

[`Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) pins the tag layout with `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)`. That constant is the only thing tying vision to the current field, so it is the line to change when a new game ships.

## Alerts

Create an `Alert` once as a static field, then call `set(...)` on it. `Telemetry.logAlerts()` scrapes the SmartDashboard alerts table every loop and mirrors anything new into the log, so the dashboard, the log, and the Driver Station all get it with no extra plumbing. [`Rio.java`](../../src/main/java/frc/spectrumLib/hardware/Rio.java) is the model to copy, with one alert per known roboRIO serial plus one for the unknown case.

## Simulation

`Utils.isSimulation()` is Phoenix's check and `RobotBase.isSimulation()` is WPILib's. Both work; use whichever the surrounding file already imports instead of adding a second import for the other.

`MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(...)` carries its own `RobotBase.isReal()` guard, so calling it unconditionally is safe. Mechanisms advance their own motor sim state from `simulationPeriodic()`, and the classes that need a sim object build it in their own `simulationInit()`. Game piece physics is not a WPILib job on this robot; see [Simulation](../tools/simulation.md).

## Things to watch for

`addVisionMeasurement` belongs in a periodic style update, once per measurement with a distinct timestamp. The pose estimator keys measurements by timestamp, so several cameras fusing in one loop is fine and is what `Vision` does. Two calls sharing a timestamp is the problem: the second replaces the correction the first made. A timestamp older than one already processed can also undo later corrections, so a stalled camera that keeps offering a stale reading is worth rejecting rather than passing through.

`Notifier` versus `addPeriodic`: the swerve sim uses a `Notifier` because it needs tight, regular updates. For everything else, `TimedRobot.addPeriodic(...)` is the simpler choice.

Requirements cancel any other command touching the same subsystem, which is usually what you want. It bites on a state-change `runOnce` chained ahead of a real command, such as `setStateCommand(...)` in `SuperStructure`. A `SequentialCommandGroup` holds the union of its children's requirements for the whole sequence, so a requirement on either child reserves that subsystem until the sequence ends, not just while that child runs. Add the requirements for the subsystems the sequence should hold, and no others.

## Further reading

[WPILib documentation](https://docs.wpilib.org/) is the concept level guide, and the [JavaDoc](https://github.wpilib.org/allwpilib/docs/release/java/) is linked into our generated docs. Read the [command based programming chapter](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) before writing a subsystem from scratch.
