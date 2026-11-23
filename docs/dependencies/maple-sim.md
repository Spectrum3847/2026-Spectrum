# maple-sim

*Audience: reference. Assumes you have read [Dependencies overview](overview.md).*

maple-sim is a physics library that models a swerve drivetrain and the season's arena. This repo ports the parts the robot uses from **maple-sim v0.4.0-beta** (MIT license, Team 5516 "Iron Maple") and the part of dyn4j 5.0.2 it runs on (BSD-3). The port is in `rust/crates/robot/src/spectrum_lib/swerve/`. Sources and licenses are in `rust/THIRD_PARTY.md`.

On this robot maple-sim does one job: the drivetrain. Game piece physics are not maple-sim. They run in `FuelPhysicsSim` (`rebuilt/fuel_physics_sim.rs`), owned by `RobotSim`, and publish under `Sim/Fuel`. See [Simulation](../tools/simulation.md).

## Files

| File | What |
|---|---|
| `maple_sim.rs` | The maple-sim classes: motor and battery sims, the swerve module and drivetrain sim, the gyro sim, and the arena with the 2026 field obstacles. |
| `physics.rs` | The rigid body physics: shapes, contacts and the solver. |
| `maple_sim_swerve_drivetrain.rs` | `MapleSimSwerveDrivetrain`: connects the drivetrain sim to the Phoenix sim states of the real swerve modules and the Pigeon 2. |

`rebuilt/robot_bump_sim.rs` models the field bump for the robot pose.

## Where it plugs in

`Swerve::start_sim_thread` in `robot/subsystems/swerve/swerve.rs` builds `MapleSimSwerveDrivetrain` and a sim notifier, only in simulation. Wheel slip and weight transfer come from here, not from WPILib's own sim.

`regulate_module_constants_for_simulation` in `swerve.rs` runs when `Swerve::new` builds the modules. It clears the encoder offset and the drive, steer and encoder inversions in simulation, and sets sim gains, friction voltages and steer inertia. It has to. An inverted drive config upsets the drive PID, and a non-zero CANcoder offset upsets module state optimization. Without it the sim robot misbehaves in ways that look like physics but are config.

## Hookups to know

In simulation `Swerve::get_robot_pose` reads the pose from the drivetrain sim, and `reset_pose` sets the sim world pose. That is how an auto seeds its pose in simulation.

The sim tick is a `Notifier` that runs every `sim_loop_period` in `SwerveConfig`, which is 5 ms. It runs faster than the robot loop on purpose, so the gains written for the real robot behave in simulation. Check how the gains respond before you slow it down.

## Constants that track the robot

`start_sim_thread` passes the robot mass (115 lb), the bumper size (30 in by 30 in), the motor models and the wheel friction as numbers into the `MapleSimSwerveDrivetrain` constructor. If the real mass, bumpers or motor count change, update those numbers. Nothing checks them against `SwerveConfig`, so a mismatch shows as a sim that drifts from the robot, not as a failure.

## Each new season

The arena (`Arena2026Rebuilt` field obstacles in `maple_sim.rs`) is specific to the 2026 game. It needs a new version when the game changes.

## Further reading

The [maple-sim repository](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) has examples and the physics settings we do not use. See [Simulation](../tools/simulation.md) for the whole sim workflow.
