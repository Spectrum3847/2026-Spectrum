# MapleSim (IronMaple)

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

MapleSim is a physics-based simulation library — `org.ironmaple.simulation.*` — that models the swerve drivetrain and the per-season arena. It plugs into WPILib's simulation loop, and the result is a GUI sim that's a lot closer to reality than a stock WPILib one. (Game-piece physics — intake, flight, scoring — is *not* MapleSim on this robot; that runs through [`FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java), covered in [Simulation](../tools/simulation.md).)

Vendor JSON: [`vendordeps/maple-sim.json`](../../vendordeps/maple-sim.json).

## What We Sim

The swerve drivetrain runs through [`MapleSimSwerveDrivetrain`](../../src/main/java/frc/spectrumLib/swerve/MapleSimSwerveDrivetrain.java), which wraps `SwerveDriveSimulation` + `SwerveModuleSimulation`. That's where wheel slip and weight transfer come from.

The 2026 field comes from MapleSim's season-specific arena, `Arena2026Rebuilt`, constructed inside `MapleSimSwerveDrivetrain`. It gets replaced every year when the new game ships, so plan to revisit it in the offseason.

Game pieces are *not* MapleSim on this robot — fuel spawning, intake pickup, and projectile flight all run through [`FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java) (`RobotSim.ballSim`, published to `Sim/Fuel`). See [Simulation](../tools/simulation.md) for that side.

## Guarding Sim Code

MapleSim only stands up under simulation, so its entry points are gated on `Utils.isSimulation()` (or `RobotBase.isSimulation()` — both work; pick whichever matches the surrounding file's imports). The drivetrain sim (`mapleSimSwerveDrivetrain`) is only constructed on that path, so it stays `null` on the RIO.

## Drivetrain Hookup

`MapleSimSwerveDrivetrain` is the glue. A few specifics worth knowing:

`MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)` adjusts module constants to physically plausible sim values. Skip this and the sim robot skitters because its wheels are too stiff.

`mapleSimDrive.getSimulatedDriveTrainPose()` is what `Swerve.getRobotPose()` returns in sim. `mapleSimDrive.setSimulationWorldPose(pose)` is the teleport call `Swerve.resetPose(...)` uses when an auto seeds the pose.

The sim thread is a WPILib `Notifier` (`simNotifier`) ticking at `config.getSimLoopPeriod()`. Don't slow it down without checking physics behavior — MapleSim integrates dynamics on every tick.

## Constants That Need to Track Reality

`Swerve.startSimThread()` hard-codes a few values that have to stay in sync with the real robot:

```java
Pounds.of(115),       // robot weight
Inches.of(30),        // bumper length
Inches.of(30),        // bumper width
DCMotor.getKrakenX60Foc(1),
```

If the real robot changes weight or motor count, change these too or sim diverges from reality.

## Further Reading

[MapleSim JavaDoc](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/) is linked into our generated docs. The [README](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) has examples and the physics knobs we haven't touched. For the broader sim workflow, see [Simulation](../tools/simulation.md).
