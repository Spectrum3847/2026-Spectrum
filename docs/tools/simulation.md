# Simulation

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

Running the robot code without a robot. Worth doing every time you push: it catches state-machine bugs, wiring mistakes between subsystems, and PathPlanner trajectories that look fine on paper but collide with field elements.

## Launching the Sim

The fast path is in VSCode: `Ctrl+Shift+P → WPILib: Simulate Robot Code`. Gradle builds, then prompts for `GUI Sim` or `Use Driver Station`. Pick `GUI Sim` for typical iteration: it launches `Glass`, which gives you joysticks, Field2d, and NetworkTables in one window.

From the terminal: `./gradlew simulateJava` does the same thing.

Both routes have `wpi.sim.addGui().defaultEnabled = true` and `wpi.sim.addDriverstation()` from [`build.gradle`](../../build.gradle) wired up: Glass and the simulated DS come up by default. Elastic will also connect to `localhost` if you point it there.

## RobotSim: Our Side-View Drawing

[`frc.robot.RobotSim`](../../src/main/java/frc/robot/RobotSim.java) builds a `Mechanism2d` published to `SmartDashboard/Sim/LeftView`. Drag that into Glass and you get a 2D side-view of the robot rendered from `MechanismLigament2d` segments. Right now it draws an outline; adding subsystem-specific ligaments (hood angle, intake extension position) is how you make it actually useful.

The pattern from existing subsystems: instantiate a `frc.spectrumLib.sim.ArmSim` / `LinearSim` / `RollerSim` in the subsystem's constructor, route it to update its angle/position/velocity from the motor's `getSimState()`, and append it onto `RobotSim.leftView`. The sim classes do the math to map motor rotations into the visualization.

| Helper | What it draws |
| --- | --- |
| `ArmSim` | A pivoting ligament: hood, shooter pivot, arm. |
| `LinearSim` | A telescoping/sliding ligament: elevator, intake extension. |
| `RollerSim` | A spinning indicator with direction + relative speed: intake roller, indexer wheels, launcher flywheel. |

They mimic the behavior of the subsystem with a custom Sim class in different subsystem folders.

These came from Team 604's sample project and were adapted; the principle of "always move the root/origin to change display position" (commented at the top of `RobotSim.java`) is the most useful thing to remember.

## AdvantageScope

We use AdvantageScope as a simulation tool in addition to a log analysis tool. Its main job during sim is the **3D Field** view: a better way to watch the robot drive around the field than staring at raw pose numbers.

Start the robot sim first (`./gradlew simulateJava` or `WPILib: Simulate Robot Code`), then open AdvantageScope and connect to `localhost`. The robot publishes [`Robot.field2d`](../../src/main/java/frc/robot/Robot.java) to `SmartDashboard/Field2d`, and `robotPeriodic()` updates it from `swerve.getRobotPose()`. In sim, that pose comes from the drivetrain simulation, so the robot model in AdvantageScope is showing the simulated drivetrain, not a fake hand-entered pose.

For the usual 3D field setup:

1. Open AdvantageScope's **3D Field** tab.
2. Set the source to the live NetworkTables connection.
3. Drag `SmartDashboard/Field2d/Robot` into the robot pose slot.
4. If an auto is selected, drag `SmartDashboard/Field2d/Auto Routine` in as a trajectory/poses object so the planned path and simulated robot motion can be compared.

You can also visualize flying fuel projectiles: `RobotSim.createFuelProjectile()` publishes them to NetworkTables under `Sim/Fuel/Positions`, and you can drag those entries into the 3D field to watch shots in flight.

That view is especially useful for autos. Select the auto in Elastic or Glass, enable the simulated Driver Station, and watch whether the robot starts in the right place, follows the expected route, and ends with the correct heading. If the AdvantageScope robot jumps, drives mirrored from what you expected, or misses the drawn path, check pose reset first: autos seed pose through the swerve reset path, and in sim that also calls the drivetrain sim's world pose reset.

AdvantageScope can also replay the same data from a `.wpilog` after the run. That makes the workflow: test the auto in sim, use 3D Field live while it runs, then open the saved log if you need to scrub frame-by-frame through the exact moment the path or pose estimate went sideways.

## What Simulation Catches (and Doesn't)

It catches:

* State-machine bugs: a `Coordinator` state that forgets to set a default mechanism back, command interruptions, race conditions in `setupStates()`.
* PathPlanner trajectories that look fine in the editor but the chassis can't actually drive (over-aggressive velocities, infeasible turn angles).
* Auto chooser plumbing: Elastic's chooser, auton command names, mirror flag.
* Vision pose math, when paired with PhotonVisionSim (we don't currently wire that up, but the hook is there).

It doesn't catch:

* Mechanical fit: your hood can't actually swing through the intake.
* PID feel: gains that move a sim mechanism smoothly may fight a real one with friction.
* CAN bus saturation, brownouts, anything power-related.

The sim is for *logic* validation. Mechanical and tuning validation happens on the real robot.

## When the Sim Lies

A few things to check first when a sim result doesn't match reality:

* `Robot.isSimulation()` and `Utils.isSimulation()` are not interchangeable everywhere: the `RobotBase`/Phoenix versions differ. Our code reaches for Phoenix's `Utils.isSimulation()` in `RobotSim` because we need it to gate Phoenix sim-state calls.
* The drivetrain sim is built from the per-robot swerve config. If you tweaked module positions in code but didn't redeploy/rebuild before running sim, the sim is still simulating yesterday's drivetrain.
* `IntakeSimulation.OverTheBumperIntake` height/width are in `Inches`. Mixing units silently produces a sub-millimeter intake that catches nothing.

## See Also

* [PathPlanner](../dependencies/pathplanner.md) for trajectory generation, which feeds into the sim swerve.
* WPILib's [simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html) for `Mechanism2d` and `Field2d` basics.
