# CTRE Phoenix 6

*Audience: reference. Assumes you have read [Dependencies overview](overview.md).*

Phoenix 6 is CTRE's API for TalonFX, CANcoder, Pigeon 2 and CANdle. Two crates replace CTRE's Java API:

* `phoenix6` calls CTRE's C library (`c_ctre_phoenix6_*`) and mirrors the Java API of version **26.3.0**. It has `TalonFX`, `CANcoder`, `Pigeon2`, `CANdle`, `CanBus`, status signals, control requests, configs and the sim states.
* `swerve` is the swerve drivetrain: `SwerveDrivetrain`, `SwerveModule`, the swerve requests (`FieldCentric`, `ApplyRobotSpeeds`, `SwerveDriveBrake` and others), and a Phoenix PID controller.

## Why the version is pinned

CTRE does not document the C layer as a public API. `phoenix6` copies the function signatures from CTRE's shipped headers, and its tables (signal numbers, enums, status codes, config defaults) are generated from the 26.3.0 Java API. A different Phoenix version can break both. Change the version only on purpose. See [Dependencies overview](overview.md#updating-a-version).

CTRE ships its swerve code compiled, as C++ templates and a JNI library that Rust cannot call. The `swerve` crate rebuilds that behavior from CTRE's headers, Java sources, docs and by measuring CTRE's simulation. Never disassemble CTRE binaries. CTRE's license forbids it. The facts we measured are in `AGENTS.md` under "Rules for the code".

CTRE's code stays under the CTRE license. See `rust/THIRD_PARTY.md`.

## Every mechanism goes through Mechanism

Do not call `TalonFX::new` in subsystem code. Hold a `Mechanism` (`spectrum_lib/mechanism/mechanism.rs`) and give it a `Config`. `talon_fx_factory.rs` builds the Talon and its followers from that config. The base code gives you:

* The leader and follower Talons, made from your `Config`.
* Cached reads. Position, velocity, voltage and current are read at most once per loop, so repeating a read does not hit the bus again.
* Methods for each control mode we use (`set_mm_position_foc`, `set_velocity_torque_current_foc`, `set_voltage_output` and more), and triggers such as `at_rotations`, `above_velocity_rpm` and `at_current`.
* An `attached` flag. When it is false the mechanism builds no hardware and reads return 0.

Read `mechanism.rs` before you add to it. If you write the same boilerplate in a second subsystem, it belongs there.

## Control requests

Torque current FOC is the default where it helps, and it needs Phoenix Pro. `VoltageOut` and duty cycle requests are for when you do not want FOC. The request a method sends is chosen inside `Mechanism`, so read it there.

PID gains live in slots 0, 1 and 2. The slot is set on the request. See [PID tuning](../tools/pid-tuning.md).

## Units

At the Phoenix boundary the units are Phoenix's own: rotations (`*_rot`), rotations per second (`*_rps`), volts and amps. Robot code keeps RPM and degrees, with the unit in the name.

## Simulation

The CTRE simulation libraries (`libCTRE_Sim*`) are downloaded by `cargo xtask fetch-natives` on desktop platforms. In simulation, creating a device starts the enable feeder that Java's `AutoFeedEnable` runs. `phoenix6::sim` has the sim states the drivetrain simulation writes to.

## Encoders and the CAN bus

The swerve modules use `CANcoder` through the `swerve` crate. `SpectrumCANcoder` (`spectrum_lib/hardware/spectrum_cancoder.rs`) exists, but no subsystem uses it today.

`CanBus::new(Rio::CANIVORE)` finds the first CANivore. `Rio::RIO_CANBUS` is the roboRIO's own bus.

## See also

* [Phoenix Tuner X](../tools/phoenix-tuner-x.md) for device setup.
* [PID tuning](../tools/pid-tuning.md).
* CTRE's [Phoenix 6 documentation](https://v6.docs.ctr-electronics.com/).
