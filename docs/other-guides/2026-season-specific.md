# 2026 season specific: REBUILT

*Audience: reference. Assumes you have read [Setup](../setup.md).*

This page tells you where things live and which file owns which decision. It also lists the robot's states and controls, because students ask for them first. The code is the source of truth. If this page and the code differ, trust the code and fix the page.

## Where things live

All robot code is in `rust/crates/robot/src/`.

* `robot/robot.rs` builds everything in `Robot::init`, picks the config, and holds `bind_controls`, where gamepad buttons become commands. It also has the mode methods (`autonomous_init`, `teleop_init` and so on).
* `robot/subsystems/` has one file per mechanism: `fuel_intake`, `intake_extension`, `indexer_bed`, `indexer_tower`, `launcher`, `hood`, `leds` (not used yet), `vision` and `swerve/`. `super_structure.rs` is the orchestrator. See [Adding a subsystem](../coding-conventions/adding-a-subsystem.md).
* `robot/pilot/` and `robot/operator/` hold the gamepad classes. They expose `Trigger`s and hold no behavior.
* `robot/configs/` holds one function per robot.
* `robot/auton/` holds the auto chooser and the PathPlanner event triggers.
* `robot/robot_sim.rs` and `rebuilt/` hold game logic and simulation. `ShotCalculator` and `field.rs` run on the real robot, so treat them as robot code. `FuelPhysicsSim` and `RobotBumpSim` are simulation only.
* `context.rs` holds robot-wide objects (`swerve`, `super_structure`, `hood` and others). Each is set once during init and read through a function. Reading one before it is set panics with its name.
* `spectrum_lib/` is the reusable layer: `Mechanism`, gamepads, hardware wrappers, LEDs, vision, sims. A change here changes every robot, so take more care than with a per-robot config.

Other crates (`wpilib`, `commands`, `wpimath`, `phoenix6`, `swerve`, `pathplanner`, `telemetry`) are libraries. See [Dependencies overview](../dependencies/overview.md).

## Where a behavior belongs

* A move that needs several mechanisms to change together is a new `WantedSuperState` in `super_structure.rs`. Add it to `WantedSuperState`, `CurrentSuperState`, `handle_state_transitions` and `plan_for`.
* A button that starts a state that already exists is a binding in `bind_controls`, calling `targets.set_state(...)`.

## The state machine

`WantedSuperState` is what a button or auto asks for. `handle_state_transitions` turns it into a `CurrentSuperState`. `plan_for` maps that state to a `Plan`, which is the wanted state of the swerve and each mechanism. `SuperStructure::apply_states` sends the plan out every loop.

| State | What it does |
|---|---|
| `Idle` | Mechanisms off or at rest, launcher in idle prep, hood home. Teleop drive. While disabled or in auto, the swerve idles. |
| `IntakeFuel` | Extends the intake, runs the intake and the bed slowly. |
| `TrackTarget` | Drive aims at the target, launcher and hood aim. Intake extension is `ConditionalExtend`. |
| `LaunchWithSqueeze` | Aims and launches with the intake held out for 1 second, then closes it slowly. |
| `LaunchWithSqueezeWithNoDelay` | Same, but the intake closes right away. |
| `LaunchWithoutSqueeze` | Same, with the intake held out the whole time. |
| `LaunchWithBrake` | Locks the wheels in an X, launches. No pilot binding uses it today. |
| `AutonIntakeFuel`, `AutonTrackTarget` | Like the teleop states, but the swerve is left to the path follower. |
| `Unjam` | Runs the indexers in reverse. Launcher off. |
| `Eject` | Runs the intake out and the indexers in reverse. Launcher off. |
| `ForceHome` | Retracts the intake fully. Launcher off. |

While the drive aims at the target in the launch states, the pilot's translation speed drops to 10 percent. `plan_for` has the exact settings for every state.

Field zone triggers, such as `robot_in_neutral_zone`, are methods on `SuperStructure`.

## Controls

Read `bind_controls` in `robot/robot.rs` for the current bindings. The pilot is gamepad port 0 and the operator is port 1.

Pilot:

| Input | Action |
|---|---|
| Left trigger | `IntakeFuel` while held |
| Right trigger | `LaunchWithSqueeze` while held |
| Both triggers | `LaunchWithoutSqueeze` |
| Left trigger released while right is held | `LaunchWithSqueezeWithNoDelay` |
| Right trigger released while left is held | `IntakeFuel` |
| Both triggers released | `Idle` |
| Left trigger and left bumper | `Eject` |
| X | `TrackTarget` while held |
| A | `Unjam` while held |
| Select | `ForceHome` while held |
| Left bumper and D-pad | Set the field-centric forward direction (up, left, down, right) |
| A while disabled | Coast the intake extension and hood |
| B while disabled | Brake the intake extension and hood |

Operator:

| Input | Action |
|---|---|
| Left bumper and Y | Reset the intake extension position to its maximum, with a rumble. Works while disabled. |
| Select | `ForceHome` while held |
| D-pad up and down | Raise or lower the hood angle offset by 0.1 |
| D-pad left and right | Raise or lower the drive angle offset by 1 |
| A or B while disabled | Coast or brake, as for the pilot |

## Per-robot configs

The same program runs on several robots. At startup `config_for(Rio::id())` picks a config function by roboRIO serial. `spectrum_lib/hardware/rio.rs` holds the serial numbers.

* `PHOTON2026` uses `photon2026`.
* `PM2026` uses `pm2026`.
* Anything else, including the simulator, uses `fm2026`.
* `am2026` and `xm2026` exist, but nothing selects them. Editing them changes no robot.

Two gotchas:

* A serial that matches no robot falls back to `fm2026`. A reflashed or replaced roboRIO looks like a working robot until you deploy to it. If a robot behaves like the wrong one, check `rio.rs` first.
* Each config sets `attached` for each mechanism. A mechanism is built on every robot, but an unattached one makes no motors and its reads return zero. Swerve encoder offsets are per-robot facts. CAN IDs are in each subsystem's config struct. A change in the wrong config breaks a different robot and still compiles.

## Autos

Autos are PathPlanner files in `src/main/deploy/pathplanner/`. `Auton` builds the chooser from them. Event markers on paths (`intake`, `shotPrep`, `unjam`, `clearState`, `poseUpdate`) trigger states. See [Auton](../tools/auton.md).

## Vision

Three Limelights (back, left, right) do AprilTag pose estimation. `Vision` decides which estimate to trust and passes it to the swerve pose estimator. Game piece detection and QuestNav are not integrated. See [Vision](../tools/vision.md).

## Hub shifts

In REBUILT each alliance's hub switches between active and inactive during teleop. This is a game rule, so the code uses it without stating it. `rebuilt/shift_helpers.rs` holds the schedule: shift start and end times and which alliance is active in each. `bind_controls` calls `shift_helpers::initialize` when teleop, auto or disabled starts, so the shift clock restarts from zero. If a shift-aware behavior reads wrong, check that reset first.

The first active alliance comes from the FMS game message, not from alliance color. With no FMS, as in practice, the code assumes the opponent is active first, so the schedule can be wrong in practice.

## Shot calculation

`ShotCalculator` (`rebuilt/shot_calculator.rs`) computes the drive angle, hood angle and launcher speed for the current pose and velocity. The operator's D-pad nudges the hood and drive offsets. A `Hub Model Chooser` shows on the dashboard.
