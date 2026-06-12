# Simulation

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

Running the robot code without a robot. Worth doing every time you push: it catches state-machine bugs, wiring mistakes between subsystems, and PathPlanner trajectories that look fine on paper but collide with field elements.

## Launching the Sim

The fast path is in VSCode: `Ctrl+Shift+P → WPILib: Simulate Robot Code`. Gradle builds, then prompts for `GUI Sim` or `Use Driver Station`. Pick `GUI Sim` for typical iteration — it launches `Glass`, which gives you joysticks, Field2d, and NetworkTables in one window.

From the terminal: `./gradlew simulateJava` does the same thing.

Both routes have `wpi.sim.addGui().defaultEnabled = true` and `wpi.sim.addDriverstation()` from [`build.gradle`](../../build.gradle) wired up — Glass and the simulated DS come up by default. Elastic will also connect to `localhost` if you point it there.

## RobotSim — Our Side-View Drawing

[`frc.robot.RobotSim`](../../src/main/java/frc/robot/RobotSim.java) builds a `Mechanism2d` published to `SmartDashboard/Sim/LeftView`. Drag that into Glass and you get a 2D side-view of the robot rendered from `MechanismLigament2d` segments. Right now it draws an outline; adding subsystem-specific ligaments (hood angle, intake extension position) is how you make it actually useful.

The pattern from existing subsystems: instantiate a `frc.spectrumLib.sim.ArmSim` / `LinearSim` / `RollerSim` in the subsystem's constructor, route it to update its angle/position/velocity from the motor's `getSimState()`, and append it onto `RobotSim.leftView`. The sim classes do the math to map motor rotations into the visualization.

| Helper | What it draws |
| --- | --- |
| `ArmSim` | A pivoting ligament — hood, shooter pivot, arm. |
| `LinearSim` | A telescoping/sliding ligament — elevator, intake extension. |
| `RollerSim` | A spinning indicator with direction + relative speed — intake roller, indexer wheels, launcher flywheel. |

These came from Team 604's sample project and were adapted; the principle of "always move the root/origin to change display position" (commented at the top of `RobotSim.java`) is the most useful thing to remember.

## MapleSim Physics

[MapleSim (IronMaple)](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) handles the physics — drivetrain dynamics, game-piece interaction, projectile flight. [`RobotSim.intakeSimulation`](../../src/main/java/frc/robot/RobotSim.java) wires up an over-the-bumper fuel intake:

```java
IntakeSimulation.OverTheBumperIntake(
    "Fuel",
    Robot.getSwerve().getMapleSimSwerveDrivetrain().mapleSimDrive,
    Inches.of(29),
    Inches.of(12),
    IntakeSimulation.IntakeSide.FRONT,
    80); // max game pieces
```

When the intake command starts, `mapleSimIntakeFuel()` calls `startIntake()` on the simulated intake; the moment a simulated fuel piece is in range, MapleSim transfers it to the robot. `mapleSimLaunchFuel()` reads the held-piece count and creates `RebuiltFuelOnFly` projectiles using the actual `ShotCalculator` outputs — same code path as match-day, so a shot that lands in sim should also land on the real field if the calibration is right.

The trajectory is published to NetworkTables under `SimShot/FuelProjectileSuccessfulShot` (or `…UnsuccessfulShot`). Drag those into Glass's `Field2d` overlay to see shots in flight.

## What Simulation Catches (and Doesn't)

It catches:

* State-machine bugs — a `Coordinator` state that forgets to set a default mechanism back, command interruptions, race conditions in `setupStates()`.
* PathPlanner trajectories that look fine in the editor but the chassis can't actually drive (over-aggressive velocities, infeasible turn angles).
* Auto chooser plumbing — Elastic's chooser, auton command names, mirror flag.
* Vision pose math, when paired with PhotonVisionSim (we don't currently wire that up, but the hook is there).

It doesn't catch:

* Mechanical fit — your hood can't actually swing through the intake.
* PID feel — gains that move a sim mechanism smoothly may fight a real one with friction.
* CAN bus saturation, brownouts, anything power-related.

The sim is for *logic* validation. Mechanical and tuning validation happens on the real robot.

## When the Sim Lies

A few things to check first when a sim result doesn't match reality:

* `Robot.isSimulation()` and `Utils.isSimulation()` are not interchangeable everywhere — the `RobotBase`/Phoenix versions differ. Our code reaches for Phoenix's `Utils.isSimulation()` in `RobotSim` because we need it to gate Phoenix sim-state calls.
* `mapleSimDrive` is built from the per-robot swerve config. If you tweaked module positions in code but didn't redeploy/rebuild before running sim, MapleSim is still simulating yesterday's drivetrain.
* `IntakeSimulation.OverTheBumperIntake` height/width are in `Inches`. Mixing units silently produces a sub-millimeter intake that catches nothing.

## See Also

* [MapleSim dependency page](../dependencies/maple-sim.md) for the version we run.
* [PathPlanner](../dependencies/pathplanner.md) for trajectory generation, which feeds into the sim swerve.
* WPILib's [simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html) for `Mechanism2d` and `Field2d` basics.
