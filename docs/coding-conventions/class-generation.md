# Class Generation and Method Building

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

Conventions for designing classes and methods. Most of what follows is reinforced throughout `frc.spectrumLib`. If you're not sure how to structure something new, copy what an existing subsystem already does.

## Subsystem layout

Every mechanism in `frc.robot.subsystems` is one file holding the subsystem plus its state machine and its config:

```
launcher/
  Launcher.java                // the subsystem
    Launcher.LauncherConfig    // inner class: every tunable value
    Launcher.WantedState       // inner enums: what a caller asked for
    Launcher.SystemState       // inner enum: what the mechanism is doing about it
```

The subsystem class extends `Mechanism` (`frc.spectrumLib.mechanism`) and owns the motors and sensors. The `Config` inner class holds every tunable value, gear ratios, current limits, voltages, and target poses, as `@Getter private final` fields with no setters, because those values are constants. It also carries the `attached` flag. Each per-robot config file under `frc/robot/configs` sets that flag per mechanism and supplies robot-specific calibration such as encoder offsets, so one codebase covers several physical robots.

Instead of a separate command-factory class, each subsystem drives itself with an in-class state machine: `setWantedState(WantedState)` is the entry point, `handleStateTransition()` maps wanted to system, and `applyStates()` turns the system state into a motor request. The mechanism's own `periodic()` is what chains the last two, so a state change only takes effect on the loop after the one that set it.

The orchestrator, [`SuperStructure`](../../src/main/java/frc/robot/subsystems/SuperStructure.java), sits above them. Its `setWantedSuperState(WantedSuperState)` (with `setStateCommand(...)` as the command wrapper bindings use) fans one robot-level intent out to every mechanism's `setWantedState(...)` in the `apply*` methods its own `applyStates()` dispatches to. Gamepad bindings and `Auton` talk to `SuperStructure`, never to a mechanism directly.

Worth knowing when you are new to the tree: a `Mechanism` implements `Subsystem` but does not extend `SubsystemBase`, so it never registers itself with the scheduler. `SuperStructure` is the one registered `SubsystemBase` among the robot's own classes, and it is what the scheduler ticks. If you add a mechanism and find that your `periodic()` is never called, this is why, and the fix is to call it from `SuperStructure`, not to change the base class.

Stick to this layout for new subsystems unless there's a concrete reason not to.

## Constructors

Java lets you have a dozen overloaded constructors. We mostly don't. The settled pattern is one constructor per class that takes its `Config`, passes it to `super(...)`, and then wires up motors, encoders, and triggers.

A mechanism `Config` is an immutable value holder: `@Getter private final` fields, no setters. A config that gets assembled at a call site instead uses chained setters, which is `@Accessors(chain = true)` on the class. See [Project Lombok](project-lombok.md). `Limelight.LimelightConfig` is the worked example. For pure-value classes that genuinely warrant different construction shapes, constructing from inches versus meters for instance, a builder still beats a pile of overloads. If you do end up with multiple constructors, chain them through `this(...)` so the initialization logic lives in exactly one place.

## Methods

Keep them single-purpose. A subsystem method that both computes a setpoint *and* drives the motor is hard to test and hard to override per-robot. Pull the math into a small helper, often a `DoubleSupplier`, and let the caller just schedule things.

For long `if` chains: if you're past about three conditions, extract them. A `switch` on an enum reads better than nested `if`s once a pattern emerges, and the `switch` over `WantedState` inside a `handleStateTransition()` is a decent template. For anything checked every loop that might run a command, return a `Trigger` rather than a raw `boolean`, using the `at*`, `above*`, and `below*` helpers on `Mechanism`. The scheduler re-evaluates triggers for you, so you don't have to.

Use parameters generously. Explicit parameters make methods readable and testable. But avoid the boolean-flag pattern: a method that takes `boolean reverse` is usually two methods with clearer names, such as `forward()` and `reverse()`.

For setpoints that may change while a command is running (joystick input, target distance, tunable dashboard values), accept a `DoubleSupplier` rather than a fixed `double`. The reasoning is in [Tips](../other-guides/tips.md#doublesupplier-vs-double).

A note on streams: they're fine for one-shot setup code. Inside a `periodic()` they allocate every loop, which adds up. Use plain `for` loops in hot paths.

## Documentation

Every `public` method a subsystem (or `SuperStructure`) exposes is part of the API the rest of the robot consumes, and deserves at least a one-line JavaDoc. `Config` fields should be documented with their units (`rotations`, `meters`, `volts`) so per-robot configs don't drift on what a number means.

The bigger picture on comments is in [Documentation and Comments](documentation-and-comments.md).
