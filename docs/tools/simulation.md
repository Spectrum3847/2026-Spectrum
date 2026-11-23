# Simulation

*Audience: reference. Assumes you have read [Setup](../setup.md).*

Simulation runs the robot code with no robot. Use it on every change. It catches state machine bugs, wiring mistakes between subsystems, and paths the drivetrain cannot drive.

## Launching

From `rust/`:

```text
cargo xtask sim
```

This needs the natives from `cargo xtask fetch-natives`. See [Cargo xtask](cargo-xtask.md). The robot runs the same program as on the roboRIO. WPILib's HAL detects simulation, so `wpilib::is_simulation()` is true.

This opens WPILib's Sim GUI, like the Java sim did. Close the window to stop the robot. For a run with no window, for example over SSH, use `cargo xtask sim --no-gui`.

You can also connect a dashboard to `localhost`: [Elastic](elastic.md) for the driver view, or [AdvantageScope](https://docs.advantagescope.org) for the field and logs. The robot publishes `Sim/Fuel`, `Sim/SimPose`, `Sim/RobotPose3d`, `Sim/Components` and the `Sim/LeftView` mechanism drawing. The AdvantageScope robot models are in `advantagescope-custom-assets/`.

## Enabling the robot

The robot starts disabled. In the Sim GUI, use the System Joysticks and Robot State windows: pick the mode (Teleoperated, Autonomous, Practice or Test), then click Enable. Drag a controller onto a joystick slot to drive.

To use the FRC Driver Station program on Windows, or another tool that speaks the simulated driver station protocol, add `--ds`:

```text
cargo xtask sim --ds
cargo xtask sim --no-gui --ds
```

`--ds` loads WPILib's `halsim_ds_socket` extension, which opens the socket those programs talk to. Use `--no-gui --ds` when the driver station program owns the joysticks and the mode, so two programs do not both claim them. The FRC Driver Station program is untested against this sim. [Setup](../setup.md#enabling-the-robot-in-simulation) lists what is tested.

With the robot enabled, choose an auto in the `Auto Chooser` on the dashboard. While disabled, the `Field2d` view shows the path of the chosen auto.

## Logs and deploy files in simulation

* The robot reads PathPlanner files and the Elastic layout from the nearest `src/main/deploy` above the folder you run in. Set `SPECTRUM_DEPLOY_DIR` to use a different folder.
* Log files go to a `logs` folder in the folder you ran from. `cargo xtask sim` runs from `rust/`, so look in `rust/logs`. See [Logging](logging.md).

## Fuel physics

Game pieces are simulated by `FuelPhysicsSim` (`rust/crates/robot/src/rebuilt/fuel_physics_sim.rs`). `RobotSim` (`robot/robot_sim.rs`) owns it and publishes it under `Sim/Fuel`. The drivetrain is a separate port of maple-sim. See [maple-sim](../dependencies/maple-sim.md).

`RobotSim` places the field fuel at startup and again when an auto starts. It registers the robot footprint and the intake zone in `config_ball_sim_robot`. Launching uses `ShotCalculator`, the same calculation as on a real robot. `configure_sim_bindings` in `robot.rs` starts a launch while the super state is one of the launch states.

When fuel misbehaves, check units first. The intake zone is in meters. A units mistake gives a zone that catches nothing, or everything.

## Mechanism drawings

`spectrum_lib/sim` has three mechanism simulators: `ArmSim` for a joint, `LinearSim` for a sliding stage and `RollerSim` for a roller. Each reads the subsystem's `TalonFX`, runs the physics and draws on `Sim/LeftView`. A shared 200 Hz notifier in `sim_loop.rs` steps them. The robot loop calls `sim_loop::draw_all` once per period, because dashboard objects work on the loop thread only.

A subsystem builds its sim in `simulation_init`. See `hood.rs` for a short example. If a mechanism sits at its limit and never moves, its travel runs against the motor's positive direction. Use `set_reversed_linkage(true)` on its sim config.

## What simulation catches

* Super state bugs: a state that does not return a mechanism to rest, command interruptions, races between state machines.
* Paths the editor accepted and the drivetrain cannot drive.
* The auto chooser, command names and mirror flag.
* Fuel intake and launch.

## What it does not catch

* Mechanical fit. A hood can swing through the intake in sim.
* Gains. A mechanism that moves smoothly in sim fights friction on a real robot.
* CAN bus load, brownouts and power.

## When the sim lies

Two functions answer "is this a simulation": `wpilib::is_simulation()` and `phoenix6::utils::is_simulation()`. The robot uses the Phoenix one where it gates calls into TalonFX sim state, such as the sim setup in `robot.rs` and the swerve sim in `swerve.rs`. Use the one the surrounding file uses. The wrong one makes a mechanism not move in sim.

## See also

* [maple-sim](../dependencies/maple-sim.md) for the drivetrain side.
* [PathPlanner](../dependencies/pathplanner.md) for paths.
* WPILib's [simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html).
