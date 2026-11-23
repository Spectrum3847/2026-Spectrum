# WPILib

*Audience: reference. Assumes you have read [Dependencies overview](overview.md).*

Three crates replace WPILib. They port WPILib v2026.2.1 (BSD-3 license, see `WPILib-License.md` at the repo root).

| Crate | What it holds |
|---|---|
| `wpilib` | The HAL over WPILib's C API, driver station, timers, notifiers, the timed robot loop, joysticks, PWM, power distribution, NetworkTables, dashboards (SmartDashboard, `SendableChooser`, `Field2d`, `Mechanism2d`, `Alert`), the deploy folder, a small web server and shutdown hooks. |
| `commands` | The command-based framework: scheduler, subsystems, triggers, command groups, `SysIdRoutine`. |
| `wpimath` | Geometry, kinematics, the pose estimator, PID, filters, DC motor models and mechanism sims. A subset only. |

Port only what the robot uses. If you need a WPILib class that is missing, add it to the right crate with tests, and add it to `rust/THIRD_PARTY.md`.

## How it differs from Java

* **Natives load at runtime.** `wpilib` finds `wpiHal`, `ntcore` and the rest by name through the library path. A missing library gives a clear error that names the file. `cargo xtask fetch-natives` downloads them.
* **Errors go to the driver station.** Where Java throws, a getter reports to the driver station and returns zero. Constructors return `Result`.
* **One loop thread.** The robot loop, the scheduler and dashboards run on one thread. Code on other threads must not touch them. See [Exception handling](../coding-conventions/exception-handling.md) and the rules in `AGENTS.md`.
* **`start_robot`.** `main.rs` calls `wpilib::start_robot(robot::robot::robot::new_timed_robot)`. It starts the HAL, builds the robot, runs the loop, and reports a panic to the driver station.
* **Time and mode.** `commands` does not depend on `wpilib`. `new_timed_robot` hands it WPILib's clock and the robot mode through `commands::env`.

## Mechanisms and subsystems

Motor mechanisms hold a `Mechanism` (`spectrum_lib/mechanism/mechanism.rs`), which owns the TalonFX. Each subsystem, such as `Hood`, implements the `Subsystem` trait and registers itself with `CommandScheduler::get_instance().register_subsystem(...)`. `Vision` implements `Subsystem` directly because it has no motor.

## Commands and triggers

`bind_controls` in `robot/robot.rs` is the one place gamepad input becomes commands. Read it before you add a binding. Habits that matter:

* Build commands with the `Commands::` factories (`sequence`, `parallel`, `run_once`, `wait_seconds`, `either` and others).
* Give a command a name with `.with_name("...")`. That name shows in the log and in the running commands widget.
* Use `.ignoring_disable(true)` only for behavior that must run while disabled, such as swerve control requests, coast and brake mode, and the shot offsets.
* Bind a trigger rather than scheduling from a `periodic`. A trigger puts the start in the scheduler's own queue, where its requirement and interruption rules apply.

## Units

The `wpimath` crate uses SI `f64`: meters, radians, seconds. Robot code keeps the units the Java used (RPM, rotations, degrees) and puts the unit in the name, such as `hood_angle_degrees` or `position_kp`. Do not mix units inside one function. Convert at the edge with the helpers in `wpimath::units`, such as `inches_to_meters`.

## The season field layout

`rebuilt/field.rs` holds the field sizes and key points. Change it when a new game ships. The robot code does not load an AprilTag layout. Check the field map set on each Limelight.

## Not covered here

WPILib's own [docs](https://docs.wpilib.org/) describe the concepts. The Rust crates keep the Java names in snake_case, so those docs map over.
