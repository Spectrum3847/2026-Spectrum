# Class Generation and Method Building

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

Conventions for designing classes and methods. Most of what follows is reinforced throughout `frc.spectrumLib`. If you are not sure how to structure something new, copy what an existing subsystem already does.

## Subsystem layout

Every mechanism in `frc.robot.subsystems` is one file holding the subsystem plus its config inner class, and that file follows the order in [Code Style](code-style.md). A mechanism extends `Mechanism` (from `frc.spectrumLib.mechanism`) and owns the motors and sensors. The `Config` inner class holds every tunable value: gear ratios, current limits, voltages, target poses, as `@Getter` fields. Per-robot config files (`OM2026` is what the robot runs) select which mechanisms exist on that robot with `setAttached(true/false)` and supply robot-specific calibration such as the swerve's encoder offsets, so a single codebase covers different physical robots.

A mechanism config is an immutable value holder. There are no setters on it, because nothing should be able to change a launcher's gear ratio after it has been built. That is the opposite of the config objects assembled at a call site, which use chained setters (via `@Accessors(chain = true)`, see [Project Lombok](project-lombok.md)) so they read like a builder. `Limelight.LimelightConfig` is the example in the codebase. If a class genuinely warrants different construction shapes, a builder is still better than a pile of overloads; if you do end up with several constructors, chain them through `this(...)` so the initialization logic lives in exactly one place.

Instead of a separate command-factory class, each subsystem drives itself with an in-class state machine: `WantedState` and `SystemState` enums, a `setWantedState(...)` entry point, a `handleStateTransition()` that maps wanted to system state, and an `applyStates()` called from `periodic()` that issues the motor request. The orchestrator [`SuperStructure`](../../src/main/java/frc/robot/subsystems/SuperStructure.java) sits above them: its `setWantedSuperState(...)`, or the `setStateCommand(...)` wrapper around it, fans a single robot-level intent out to each subsystem's `setWantedState(...)`. Gamepad bindings and `Auton` talk to `SuperStructure`, not to the mechanisms directly.

Stick to this layout for new subsystems unless there is a concrete reason not to.

## Constructors

One constructor, taking a `Config`. It calls `super(config)` and then wires up the subsystem's own extra sensors and triggers. It does not construct a motor, because `Mechanism` already did that from the config; see [CTRE Phoenix 6](../dependencies/phoenix6.md) for why that is not ours to do.

## Methods

Keep them single-purpose. A subsystem method that both computes a setpoint *and* drives the motor is hard to test and hard to override per-robot. Pull the math into a small helper, often a `DoubleSupplier`, and let the command factory just schedule things.

For long `if` chains: if you are past about three conditions, extract them. A `switch` on an enum reads better than nested `if`s once a pattern emerges, and a subsystem's `handleStateTransition()` is the template for that in this codebase. And for anything checked every loop that might run a command, return a `Trigger` (via the at/above/below helpers on `Mechanism`) rather than a raw `boolean`. The scheduler handles re-evaluation; you do not have to.

Use parameters generously. Explicit parameters make methods readable and testable. But avoid the boolean-flag pattern: a method that takes `boolean reverse` is usually two methods with clearer names.

For setpoints that may change while a command is running (joystick input, target distance, tunable dashboard values), accept a `DoubleSupplier` rather than a fixed `double`. The reasoning is in [Tips](../other-guides/tips.md#doublesupplier-vs-double).

A note on streams: they are fine for one-shot setup code. Inside a `periodic()` they allocate every loop, which adds up. Plain `for` loops in hot paths.

## Documentation

Every `public` method a subsystem (or `SuperStructure`) exposes is part of the API the rest of the robot consumes, and deserves at least a one-line JavaDoc. `Config` fields should be documented with their units (`rotations`, `meters`, `volts`) so per-robot configs do not drift on what a number means.

The bigger picture on comments is in [Documentation and Comments](documentation-and-comments.md).
