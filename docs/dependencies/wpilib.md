# WPILib

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

WPILib is the foundation, and its [online documentation](https://docs.wpilib.org/) is good. This page only records the places where this team made a decision of its own. When your question is "what does this WPILib type do", the answer belongs in the WPILib docs or the [JavaDoc](https://github.wpilib.org/allwpilib/docs/release/java/), not here.

Only New Commands ships as a vendor JSON, in `vendordeps/`. WPILib core comes straight from GradleRIO, so there is no JSON to look at for it.

## Subsystems do not extend SubsystemBase

A TalonFX mechanism extends [`frc.spectrumLib.mechanism.Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java), which implements `Subsystem` itself and hands you the config, the cached sensor reads, and the command factories. Do not extend `SubsystemBase` and rebuild any of that by hand.

The exception is [`Vision`](../../src/main/java/frc/robot/subsystems/vision/Vision.java), which implements `Subsystem` directly because it has no motor to wrap and nothing to configure at boot. That is the only exception. A new subsystem that seems to need one probably has hardware.

## Bindings live in one place

Gamepad triggers are bound to commands in [`Robot.configureBindings()`](../../src/main/java/frc/robot/Robot.java), with a sim-only twin in `configureSimBindings()`. Keep `tools/robot-app/data/controls.json` in step; `node tools/robot-app/scripts/check-drift.mjs` reports drift.

Habits worth picking up:

* Reach for the `Commands.*` factories rather than hand-built `InstantCommand` and `SequentialCommandGroup` instances. They compose with `Trigger.onTrue` and `whileTrue` in a way a raw command group does not.
* Name every command with `.withName(...)`. That string is what shows up in the DogLog `Commands` key and in the running-commands widget, so it is the only handle you have when a command is not behaving the way you expected.
* `.ignoringDisable(true)` is normal in this codebase, not an escape hatch. The state machine, the auton warmup, and the calibration commands (zeroing the turret, shifting the robot into auto mode) are all meant to run while the robot is disabled, because that is when a human is standing at the robot doing setup. Read what your command suppresses before adding the flag.
* Do not call `CommandScheduler.getInstance().schedule(...)` from inside a `periodic()`. Manual scheduling sidesteps command requirements, and requirements are what stop two commands from fighting over the same subsystem.

## Telemetry goes through Telemetry, not SmartDashboard

Sensor readings, state transitions, and fault flags go through `Telemetry.log(...)`, which routes everything to DogLog under a consistent key format and writes it to the WPILOG so it survives the match. `SmartDashboard.put*` is for the two things SmartDashboard is actually for here: publishing the auto chooser, and publishing `Mechanism2d` views. See [DogLog](doglog.md) and [Logging](../tools/logging.md).

## Simulation

Simulation entry points are guarded on either `Utils.isSimulation()` (Phoenix) or `RobotBase.isSimulation()` (WPILib). Both work. Match whichever one the surrounding file already imports so the guard reads like the rest of the file.

A mechanism's own sim object advances its TalonFX sim state each loop. Game-piece physics is not WPILib: fuel spawning, intake pickup, and projectile flight run through [`FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java), driven from [`RobotSim`](../../src/main/java/frc/robot/RobotSim.java). For per-mechanism visualization, take a `Mechanism2d` ligament handed out by `RobotSim` rather than standing up a new widget in each subsystem. See [Simulation](../tools/simulation.md).

## Alerts

`Alert` is how this robot reports a condition a human needs to know about, and `Telemetry` scrapes every active alert into the WPILOG each loop. Create one as a `private static final` field at the top of the class, then flip it with `set(boolean)`. [`Rio.java`](../../src/main/java/frc/spectrumLib/hardware/Rio.java) has the pattern for warning at boot that the roboRIO's type is not one we recognize.

## Pose estimation

`addVisionMeasurement` belongs in a periodic-style update, not in a one-shot command. Feeding the pose estimator sporadically makes its drift unpredictable, and the symptom is a trajectory that looks fine when replayed and wrong on the field. See [Vision Systems](../tools/vision.md).

## Further reading

[Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) is the chapter worth reading before you build a new subsystem from scratch. Most of the habits above fall out of it.
