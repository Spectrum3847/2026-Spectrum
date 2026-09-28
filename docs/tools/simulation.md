# Simulation

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

Running the robot code with no robot. Worth doing on every change: it catches state machine bugs, wiring mistakes between subsystems, and trajectories that look fine in the editor and cannot actually be driven.

## Launching

In VSCode, `Ctrl+Shift+P` then **WPILib: Simulate Robot Code**. It asks for `GUI Sim` or `Use Driver Station`; take `GUI Sim` while iterating. From a terminal, `./gradlew simulateJava` does the same thing.

`build.gradle` turns Glass and a simulated Driver Station on by default, so you get joysticks, an FMS panel, and a field view without configuring anything.

The saved Glass layout is [`simgui-window.json`](../../simgui-window.json) in the repo root, and it already docks the FMS panel, the joysticks, `Field2d`, the running-commands widget, the alerts list, the auto chooser, and both robot views. Two problems with it:

* The window is saved at 2256 by 1415. On a smaller laptop the panels overlap and you cannot read any of them. Drag them once and save, and the layout is yours.
* It has a docked window for `/SmartDashboard/Sim/TopView`. Nothing publishes that topic, so the panel is permanently blank. `RobotSim` only puts `Sim/LeftView` on the dashboard. Close it or ignore it.

## Fuel physics

Game pieces are simulated by [`frc.rebuilt.FuelPhysicsSim`](../../src/main/java/frc/rebuilt/FuelPhysicsSim.java), owned by [`RobotSim`](../../src/main/java/frc/robot/RobotSim.java) and published under `Sim/Fuel`. The drivetrain is MapleSim, wired in through [`MapleSimSwerveDrivetrain`](../../src/main/java/frc/spectrumLib/swerve/MapleSimSwerveDrivetrain.java), which is adapted from the [MapleSim CTRE swerve template](https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/templates/CTRE%20Swerve%20with%20maple-sim/src/main/java/frc/robot/utils/simulation/MapleSimSwerveDrivetrain.java). Game pieces are not MapleSim's job, they are `FuelPhysicsSim`'s, and it carries its own drag, gravity, and Magnus integrator rather than leaning on MapleSim.

`RobotSim` builds the ball sim in its constructor and registers the robot footprint and the intake zone in `configBallSimRobot()`. The intake zone is a box in metres around the intake, and it only catches fuel while the super state is the intake state. Fuel is only on the field because `placeFieldBalls()` ran, and `Robot.autonomousInit` clears and replaces the balls when a sim auto starts.

Launching is the part worth believing. `ballSimLaunchFuel()` takes its position and exit speed from `ShotCalculator`, the same calculation match day uses, so a shot that lands in sim lands on the field if the calibration is right. `Robot.configureSimBindings()` wires the launch to the super state reaching a launch state, so a real launch during a sim auto throws real fuel.

Two things to check when fuel misbehaves:

* The intake zone is in metres. A units mistake gives you a box that catches nothing, or everything.
* MapleSim builds the drivetrain simulation once, at construction, from the same `SwerveConfig` the robot uses. Change a module position and the running sim is still yesterday's robot until you restart it.

## The robot drawing

`RobotSim` publishes a `Mechanism2d` as `Sim/LeftView` and currently draws a rectangle outline. To make it useful, append ligaments to the `MechanismRoot2d` in `drawSideRobot()` and move `RobotSim.origin` to reposition the whole layout, since that is what frames the camera.

`frc.spectrumLib.sim` has three mechanism simulators worth knowing: `ArmSim` for a joint, `LinearSim` for a sliding stage, and `RollerSim` for a spinning roller. Each takes a motor's `getSimState()` and drives both the WPILib physics sim and its own ligament, so a subsystem that constructs one gets a moving drawing without writing drawing code. They are `Mount` and `Mountable`, so one can attach to the tip of another.

## What simulation does and does not catch

It catches:

* Super state bugs, a state that forgets to return a mechanism to rest, command interruptions, races between subsystem state machines.
* Trajectories the editor accepted and the chassis cannot drive.
* The auto chooser, the command names, and the mirror flag. The `Field2d` preview while disabled draws the routine the chooser will actually run.
* Fuel intake and launch, since the ball sim reacts to the same commands the robot runs.

It does not catch:

* Mechanical fit. Your hood can swing through the intake in sim.
* Gains. A mechanism that moves smoothly in sim fights a real one with friction.
* Anything on the CAN bus: saturation, brownouts, power.

## When the sim lies

`Utils.isSimulation()` from Phoenix and `RobotBase.isSimulation()` from WPILib are different functions and this codebase uses both. `Robot` and `RobotSim` reach for the Phoenix one because it is what gates calls into TalonFX sim state. Using the wrong one means either sim state never updates in sim, or a real robot blocks waiting on a sim device that is not there, and both present as "the mechanism does not move."

## See also

* [MapleSim](../dependencies/maple-sim.md) for the drivetrain side and the version we run.
* [PathPlanner](../dependencies/pathplanner.md) for trajectory generation, which feeds the sim swerve.
* WPILib's [simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html) for `Mechanism2d` and `Field2d` basics.
