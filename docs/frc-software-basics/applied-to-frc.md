# Applied to FRC

*Audience: New programmers. Assumes you've read [Formatting Code & Comments](formatting-code.md).*

The basics docs in this section cover Java as a language. This page is the bridge: it maps what you've learned to what actually shows up (or doesn't) in robot code.

## The Command-Based Framework Replaces Most Loops

The biggest shift from general Java to FRC Java is that WPILib's command-based framework handles repetition for you. `teleop` and `autonomous` aren't loops you write — the scheduler calls `periodic()` on every subsystem, then runs whatever commands are scheduled, 50 times a second.

That means `while` loops doing continuous robot control are almost never the right tool. Instead of:

```java
while (unjamButtonHeld) {
    runUnjam();
}
```

you write:

```java
pilot.AButton.onTrue(superStructure.setStateCommand(WantedSuperState.UNJAM));
```

The `Trigger.whileTrue()` call handles the "keep doing this while the condition holds" logic. When the trigger goes false, the command ends automatically. No loop, no manual state management.

## What You'll Actually Use

**If/else and logic operators** — everywhere. Conditions gate command scheduling, check sensor state, and drive branching in subsystem logic. `Launcher.java` checks `isAttached()` before configuring motors; `Swerve.java` checks `isSimulation()` to decide which drivetrain to initialize.

**Classes and objects** — the entire robot is structured around them. Each mechanism is a class. `SuperStructure` and every subsystem — all classes. See [Class Generation](../coding-conventions/class-generation.md) for how they're organized.

**Enums** — heavily used. `SuperStructure` defines the top-level robot states (`WantedSuperState`: `IDLE`, `TRACK_TARGET`, `INTAKE_FUEL`, etc.). A `switch` on that enum drives `handleStateTransitions()`. When you see a mechanism that has multiple named modes (`WantedState`/`SystemState`), those modes are an enum too.

**Standard `for` loops** — used in specific places where you need to touch every element of a fixed array. `Swerve` iterates all four swerve modules by index; `Vision.java` iterates all three Limelights with an enhanced `for`. See [Loops](loops.md) and [Arrays](arrays.md).

**Lambdas and method references** — used constantly. Command factories take `DoubleSupplier` instead of `double` so setpoints can be live values. `config::getIdlingRPM` is a method reference; `() -> config.getIdlingRPM()` is an equivalent lambda. See [Classes, Methods, and Objects](classes-methods-objects.md).

## What's Rare

`while` loops and `do-while` loops appear occasionally in utility and setup code, but almost never in subsystem periodic logic or command bodies. If you're reaching for one in a command, there's usually a trigger or command composition that fits better.

Raw arrays are present but tend to live at the edges of the system — collecting module positions, storing April tag IDs, passing data to WPILib APIs that expect arrays. For anything that grows or shrinks, the codebase uses `List` or lets WPILib handle it.

---

*Previous: [Formatting Code & Comments](formatting-code.md) — Up next: the [reference docs](../index.md#i-already-know-how-to-program--show-me-the-reference) on tools, dependencies, and conventions.*
