# MapleSim (IronMaple)

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

MapleSim is a physics-based simulation library (`org.ironmaple.simulation.*`) that models the swerve drivetrain. It plugs into WPILib's simulation loop, and the result is a sim where wheel slip and weight transfer actually show up.

Game-piece physics is *not* MapleSim on this robot. Fuel spawning, intake pickup, and projectile flight run through [`FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java), which depends only on WPILib. See [Simulation](../tools/simulation.md) for that side.

The version is pinned in `vendordeps/`.

## What MapleSim covers here

The drivetrain runs through [`MapleSimSwerveDrivetrain`](../../src/main/java/frc/spectrumLib/swerve/MapleSimSwerveDrivetrain.java), which wraps MapleSim's swerve drive and module simulation classes. The field comes from MapleSim's season-specific arena, constructed inside that class. It is replaced every year when the new game ships, so expect to revisit it in the offseason.

## Guarding sim code

MapleSim only stands up under simulation, so its entry points are gated on `Utils.isSimulation()`. The drivetrain sim is only constructed on that path, which is why the field on `Swerve` is `null` on the roboRIO. Keep the guard. Code that reaches for a null sim outside simulation is a robot that boots fine and crashes the first time somebody runs it on real hardware.

## Drivetrain hookup specifics

`MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules)` rewrites the module constants to physically plausible sim values. Skip it and the sim robot skitters, because the real module constants describe a drivetrain that is far too stiff.

The simulated drive train pose is what `Swerve` returns for its robot pose in sim, and setting the simulation world pose is how `Swerve.resetPose(...)` teleports the robot when an auto seeds its start. Read the two sides of that pair in `Swerve` before you add a sim branch; the pose has to agree in both directions or a respawn puts the robot somewhere else.

The sim runs on a WPILib `Notifier` ticking at the loop period from the swerve config, not on the scheduler. MapleSim integrates dynamics on every tick, so slowing it down changes the physics rather than just the frame rate.

## Constants that have to track reality

`Swerve.startSimThread()` builds the drivetrain in one constructor call, and the literals in that call are the robot's mass, its bumper length and width, the drive and steer motor models, and the wheel coefficient of friction. They are easy to miss because they read like tuning, and they are not: if the real robot changes weight or motor count, change them in the same commit or the sim quietly diverges from the machine it is supposed to stand in for.

Each one already carries a trailing comment naming what it is, so find the `MapleSimSwerveDrivetrain` constructor inside `startSimThread()` rather than a table here. A second copy of these numbers in prose is exactly the kind of thing that goes stale and then lies.

## Further reading

The [MapleSim JavaDoc](https://shenzhen-robotics-alliance.github.io/maple-sim/javadocs/) is cross-linked from our generated docs, and the [README](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) has examples and the physics knobs we have not touched. For the broader sim workflow, see [Simulation](../tools/simulation.md).
