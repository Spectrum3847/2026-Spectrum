# Simulation

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

Running the robot code without a robot. Worth doing every time you push. It catches state machine bugs, wiring mistakes between subsystems, and PathPlanner trajectories that look fine on paper but that the chassis cannot actually drive.

## Launching the sim

The fast path is in VSCode: `Ctrl+Shift+P`, then `WPILib: Simulate Robot Code`. It builds, then asks for `GUI Sim` or `Use Driver Station`. Pick `GUI Sim` for normal iteration. It launches Glass, which gives you a joystick view, Field2d and NetworkTables in one window, and the simulated Driver Station comes up alongside it. `build.gradle` decides which of those are enabled by default; Elastic will also connect to `localhost`.

From the terminal, `./gradlew simulateJava` does the same thing.

## The side view

[`RobotSim`](../../src/main/java/frc/robot/RobotSim.java) builds a `Mechanism2d` published to `SmartDashboard/Sim/LeftView`. Drag that into Glass and you get a 2-D side view of the robot, drawn from `MechanismLigament2d` segments.

Right now it draws an outline, both for the side view and the top view. That is the boring part and it is also the part that is worth finishing: adding a subsystem's own ligament is what makes the drawing actually useful for spotting a mechanism in the wrong place.

The pattern to follow is the sim helpers in `frc.spectrumLib.sim`, which cover a pivoting arm, a sliding axis and a spinning roller. Instantiate one in the subsystem's constructor, let it advance from the motor's `getSimState()`, and append its root onto the view in `RobotSim`. They do the arithmetic that maps motor rotations into a drawn pose.

The one thing worth remembering from that code is the convention that the root, not the segment, is what you move to change where something appears. Moving a root moves everything appended under it, which is what you want when a subassembly sits on a pivot.

## Fuel physics

Game piece physics, spawning, intake pickup and projectile flight run through [`FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java), owned by `RobotSim` as its `ballSim` field and publishing under `Sim/Fuel`. It carries its own drag, gravity and Magnus integrator, so projectile flight does not depend on MapleSim.

The wiring is in `RobotSim`'s constructor and `configBallSimRobot()`. `RobotSim` constructs the sim, enables it, spawns the field's fuel, and registers the robot's intake zone. Fuel that enters the intake box while the intake is active gets picked up, which `ballSim.getTotalIntaked()` reports. `ballSimLaunchFuel()` fires held fuel, and the launch pose and velocity come from the real [`ShotCalculator`](../../src/main/java/frc/rebuilt/ShotCalculator.java), the same code path as match day. That is the point: a shot that lands in sim should land on the real field if the calibration is right, so when it does not, the calibration is what is suspect.

Drag `Sim/Fuel` into Glass's field view to watch the balls in flight.

## What simulation catches, and what it does not

It catches state machine bugs, a super state that forgets to set a mechanism back, command interruptions and races between subsystem state machines. It catches auto chooser plumbing, command names and the mirror flag. It catches PathPlanner trajectories the editor was happy with and the chassis cannot follow. It catches fuel intake and launch logic, because the fuel sim reacts to the same intake and shooter commands the real robot runs.

It does not catch mechanical fit, whether the hood can actually swing through the intake. It does not catch PID feel, because gains that move a sim mechanism smoothly may fight a real one with friction and load. It catches nothing power related: no CAN bus saturation, no brownouts.

The sim is for validating logic. Mechanical and tuning validation happens on the real robot.

## When the sim lies

* **Two different `isSimulation` calls, and they are not interchangeable.** The code uses WPILib's `RobotBase.isSimulation()` in most places and CTRE's `Utils.isSimulation()` in `RobotSim`, because that one gates Phoenix sim-state calls. There is no `Robot.isSimulation()`; if you reach for it, you are writing a method that does not exist. Check which import you have before you trust a branch you wrote from memory.

* **The drivetrain MapleSim is simulating is built from the per-robot swerve config.** Change module positions in code and rebuild before you conclude anything about handling. A sim that has not been rebuilt is simulating yesterday's drivetrain, and it will be perfectly consistent and completely wrong.

* **The fuel sim's intake zone is in metres.** A units mixup produces an intake box that silently catches nothing, or everything, and both look like a logic bug in the intake.

## See also

* [MapleSim](../dependencies/maple-sim.md) for the version we run and what it does and does not model.
* [PathPlanner](../dependencies/pathplanner.md) for trajectory generation, which feeds the sim swerve.
* WPILib's [simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html) for `Mechanism2d` and the field views.
