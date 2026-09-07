# Simulation

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

Running the robot code without a robot. Worth doing every time you push: it catches state-machine bugs, wiring mistakes between subsystems, and PathPlanner trajectories that look fine on paper but collide with field elements.

## Launching the Sim

The fast path is in VSCode: `Ctrl+Shift+P → WPILib: Simulate Robot Code`. Gradle builds, then prompts for `GUI Sim` or `Use Driver Station`. Pick `GUI Sim` for typical iteration, it launches `Glass`, which gives you joysticks, Field2d, and NetworkTables in one window.

From the terminal: `./gradlew simulateJava` does the same thing.

Both routes have `wpi.sim.addGui().defaultEnabled = true` and `wpi.sim.addDriverstation()` from [`build.gradle`](../../build.gradle) wired up, Glass and the simulated DS come up by default. Elastic will also connect to `localhost` if you point it there.

## RobotSim, Our Side-View Drawing

[`frc.robot.RobotSim`](../../src/main/java/frc/robot/RobotSim.java) builds a `Mechanism2d` published to `SmartDashboard/Sim/LeftView`. Drag that into Glass and you get a 2D side-view of the robot rendered from `MechanismLigament2d` segments. Right now it draws an outline; adding subsystem-specific ligaments (hood angle, intake extension position) is how you make it actually useful.

The pattern from existing subsystems: instantiate a `frc.spectrumLib.sim.ArmSim` / `LinearSim` / `RollerSim` in the subsystem's constructor, route it to update its angle/position/velocity from the motor's `getSimState()`, and append it onto `RobotSim.leftView`. The sim classes do the math to map motor rotations into the visualization.

|   Helper    |                                              What it draws                                               |
|-------------|----------------------------------------------------------------------------------------------------------|
| `ArmSim`    | A pivoting ligament, hood, shooter pivot, arm.                                                          |
| `LinearSim` | A telescoping/sliding ligament, elevator, intake extension.                                             |
| `RollerSim` | A spinning indicator with direction + relative speed, intake roller, indexer wheels, launcher flywheel. |

These came from Team 604's sample project and were adapted; the principle of "always move the root/origin to change display position" (commented at the top of `RobotSim.java`) is the most useful thing to remember.

## Fuel Physics

Game-piece physics — spawning, intake pickup, and projectile flight — runs through [`frc.rebuilt.FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java), owned by `RobotSim` as its `ballSim` field and publishing to NetworkTables under `Sim/Fuel`. (MapleSim still simulates the swerve *drivetrain* via [`MapleSimSwerveDrivetrain`](../../src/main/java/frc/spectrumLib/swerve/MapleSimSwerveDrivetrain.java), but no longer the game pieces.)

`RobotSim` sets it up in its constructor:

```java
ballSim = new FuelPhysicsSim("Sim/Fuel");
ballSim.enable();
ballSim.placeFieldBalls();   // spawns all the game pieces
configBallSimRobot();        // registers the robot's intake zone
```

`configBallSimRobot()` calls `ballSim.addIntakeZone(...)`, so a fuel piece that enters the robot's intake box while the intake is active is picked up (tracked by `ballSim.getTotalIntaked()`). `ballSimLaunchFuel()` fires held fuel: it reads the intaked count and calls `ballSim.launchBall(launcherPose, launchVelocity, spin)`, where the pose and velocity come from the actual [`ShotCalculator`](../../src/main/java/frc/rebuilt/ShotCalculator.java) outputs, the same code path as match-day, so a shot that lands in sim should also land on the real field if the calibration is right.

`FuelPhysicsSim` carries its own drag/gravity/Magnus integrator, so it does not depend on MapleSim for projectile flight. Drag `Sim/Fuel` into Glass's `Field2d`/`Field3d` overlay to see the balls in flight.

## What Simulation Catches (and Doesn't)

It catches:

* State-machine bugs, a `SuperStructure` super-state that forgets to set a mechanism back, command interruptions, race conditions between subsystem state machines.
* PathPlanner trajectories that look fine in the editor but the chassis can't actually drive (over-aggressive velocities, infeasible turn angles).
* Auto chooser plumbing, Elastic's chooser, auton command names, mirror flag.
* Fuel intake/launch logic, `FuelPhysicsSim` reacts to the same intake and shooter commands the real robot runs.

It doesn't catch:

* Mechanical fit, your hood can't actually swing through the intake.
* PID feel, gains that move a sim mechanism smoothly may fight a real one with friction.
* CAN bus saturation, brownouts, anything power-related.

The sim is for *logic* validation. Mechanical and tuning validation happens on the real robot.

## When the Sim Lies

A few things to check first when a sim result doesn't match reality:

* `Robot.isSimulation()` and `Utils.isSimulation()` are not interchangeable everywhere, the `RobotBase`/Phoenix versions differ. Our code reaches for Phoenix's `Utils.isSimulation()` in `RobotSim` because we need it to gate Phoenix sim-state calls.
* `mapleSimDrive` is built from the per-robot swerve config. If you tweaked module positions in code but didn't redeploy/rebuild before running sim, MapleSim is still simulating yesterday's drivetrain.
* The `FuelPhysicsSim` intake zone in `configBallSimRobot()` is in meters. A units mixup silently produces an intake box that catches nothing (or everything).

## See Also

* [MapleSim dependency page](../dependencies/maple-sim.md) for the version we run.
* [PathPlanner](../dependencies/pathplanner.md) for trajectory generation, which feeds into the sim swerve.
* WPILib's [simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html) for `Mechanism2d` and `Field2d` basics.
