# MapleSim

*Audience: Reference. Assumes you've read [Dependencies overview](overview.md).*

MapleSim is a physics based simulation library under `org.ironmaple.simulation` that models the swerve drivetrain and the current season's arena. On this robot it does exactly one thing: the drivetrain. Game piece physics, intake, flight, and scoring are not MapleSim. Those run through [`FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java) as `RobotSim.ballSim`, published under the `Sim/Fuel` key. If you went looking for fuel physics in MapleSim, that is why you did not find it. See [Simulation](../tools/simulation.md) for that side.

`vendordeps/` holds the pinned JSON.

## Where it plugs in

[`MapleSimSwerveDrivetrain`](../../src/main/java/frc/spectrumLib/swerve/MapleSimSwerveDrivetrain.java) wraps `SwerveDriveSimulation` plus a per module `SwerveModuleSimulation` and owns the Pigeon 2 sim state. Its constructor also takes over MapleSim's global state: it overrides the simulation timing and installs a fresh `Arena2026Rebuilt`. That arena is a MapleSim season specific import, so it is the line that breaks every offseason when the new game ships. Wheel slip and weight transfer come from here, not from WPILib's own sim.

## Guarding sim code

`Swerve` constructs the drivetrain sim only inside a `Utils.isSimulation()` branch in its constructor, so `mapleSimSwerveDrivetrain` stays null on a real roboRIO. Guard new sim entry points the same way, using whichever `isSimulation()` the surrounding file already imports.

`MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)` is also called from `Swerve`'s constructor, and it has its own `RobotBase.isReal()` guard, so calling it unconditionally is safe. It zeroes the encoder offset and clears the drive, steer, and encoder inversions, then substitutes sim tuned gains, gear ratio, friction voltages, and steer inertia. It has to do that, because an inverted drive config upsets the drive PID and a non zero CANcoder offset upsets module state optimization. Skip it and the sim robot misbehaves in ways that look like a physics problem but are really a config problem.

## Hookups worth knowing

In sim, `Swerve.getRobotPose()` returns `mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose()`. `Swerve.resetPose(...)` calls `setSimulationWorldPose(pose)` on that same object, which is how an auto seeds its pose during simulation.

The sim tick is a WPILib `Notifier` in `Swerve` (`simNotifier`), started at `config.getSimLoopPeriod()`, which is 5 ms in `SwerveConfig`. It runs faster than real time on purpose, so the PID gains written for the real robot behave sanely in sim. Do not slow it down without re checking how the gains respond.

## Constants that track reality

`Swerve.startSimThread()` passes the robot mass, the two bumper dimensions, the drive and steer motor models, and the wheel coefficient of friction as literals into the `MapleSimSwerveDrivetrain` constructor. If the real robot's weight, bumper size, or motor count changes, those are the literals to update. Nothing cross checks them against `SwerveConfig`, so a mismatch shows up as sim quietly diverging from the robot rather than as a failure.

## Further reading

[MapleSim JavaDoc](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/) is linked into our generated docs, and the [README](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) has examples plus the physics knobs we have not touched. For the broader sim workflow, see [Simulation](../tools/simulation.md).
