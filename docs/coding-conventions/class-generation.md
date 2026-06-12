# Class Generation and Method Building

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

Conventions for designing classes and methods. Most of what follows is reinforced throughout `frc.spectrumLib` — if you're not sure how to structure something new, copy what an existing subsystem like `Launcher` or `Hood` already does.

## Subsystem Layout

Every mechanism in `frc.robot` is three pieces in one folder:

```
launcher/
  Launcher.java         // the subsystem itself
  Launcher.LauncherConfig (inner class)
  LauncherStates.java   // commands + triggers the rest of the robot uses
```

The subsystem class extends `Mechanism` (from `frc.spectrumLib.mechanism`) and owns the motors and sensors. The `Config` inner class holds every tunable value — gear ratios, current limits, voltages, target poses — annotated with `@Getter`/`@Setter`. Each per-robot config file (`FM2026`, `PM2026`, …) mutates those defaults during construction so a single codebase covers different physical robots.

The `*States` class is `public final` with a private constructor, exposing only `public static` command factories (`intakeFuel()`, `aimAtTarget()`, …). The rest of the robot — `Coordinator`, `RobotStates`, gamepad bindings — talks to the subsystem through `*States`, never directly. That indirection is what makes the trigger graph work and what lets us swap robot configs without rewriting the call sites.

Stick to this layout for new subsystems unless there's a concrete reason not to.

## Constructors

Java lets you have a dozen overloaded constructors. We mostly don't. The pattern that's settled across the codebase is "one constructor that takes a `Config`":

```java
public Launcher(LauncherConfig config) {
    super(config);
    // wire up motors, encoders, triggers
}
```

The `Config` itself uses chained setters (via `@Accessors(chain = true)` — see [Project Lombok](project-lombok.md)) so the call site reads like a builder. For pure-value classes that genuinely warrant different construction shapes (constructing from inches versus meters, say), a builder is still preferred over a pile of overloads. If you do end up with multiple constructors, chain them through `this(...)` so the actual initialization logic lives in exactly one place.

## Methods

Keep them single-purpose. A subsystem method that both computes a setpoint *and* drives the motor is hard to test and hard to override per-robot. Pull the math into a small helper — often a `DoubleSupplier` — and let the command factory just schedule things.

For long `if` chains: if you're past about three conditions, extract them. A `switch` on an enum reads better than nested `if`s once a pattern emerges — `frc.robot.State.isReadyState` is a decent template for that. And for anything that's checked every loop and might run a command, return a `Trigger` (via the `At/Above/Below` helpers on `Mechanism`) instead of a raw `boolean`. The scheduler handles re-evaluation; you don't have to.

Use parameters generously. Explicit parameters make methods readable and testable. But avoid the boolean-flag pattern — a method that takes `boolean reverse` is usually two methods with clearer names (`forward()` and `reverse()`).

For setpoints that may change while a command is running (joystick input, target distance, tunable dashboard values), accept a `DoubleSupplier` rather than a fixed `double`. The reasoning is in [Tips](../other-guides/tips.md#doublesupplier-vs-double).

A note on streams: they're fine for one-shot setup code. Inside a `periodic()` they allocate every loop, which adds up. Plain `for` loops in hot paths.

## Documentation

Every `public` method on a `*States` class is part of the API the rest of the robot consumes. They deserve at least a one-line JavaDoc. `Config` fields should be documented with their units (`rotations`, `meters`, `volts`) so per-robot configs don't drift on what a number means.

The bigger picture on comments is in [Documentation and Comments](documentation-and-comments.md).
