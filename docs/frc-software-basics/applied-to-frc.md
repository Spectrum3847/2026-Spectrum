# Applied to FRC

*Audience: New programmers. Assumes you've read [Code formatting and comments](formatting-code.md).*

The rest of this section teaches Java as a language. This page is the bridge: it maps what you
have learned onto what actually shows up in robot code, and onto one thing that deliberately does
not.

The names on this page are generic. They stand in for a mechanism, not for anything in this repo.

## The scheduler does the repeating for you

This is the biggest change coming from general Java. WPILib's command-based framework handles
repetition on your behalf. Teleop and autonomous are not loops you write. The scheduler runs 50
times a second, and each time it calls `periodic()` on every subsystem you registered, then runs
whatever commands are currently scheduled.

That means a `while` loop doing continuous robot control is almost never the right tool. Say you
wanted a mechanism to keep driving forward while a button is held. The instinct from plain Java is
this:

```java
// Do not put this in periodic(). The loop never returns.
while (fastForward.get()) {
    drive(1.0);
}
```

Two things go wrong. Nothing after the loop ever runs, including the rest of `periodic()`. And
because the scheduler is what calls `periodic()`, the whole robot stops being scheduled. The
command-based version is:

```java
// `fastForward` is a Trigger, and `drive` is a method that returns a Command.
fastForward.whileTrue(drive(1.0));
```

`whileTrue` handles the "keep doing this while the condition holds" part. When the button is
released the command ends on its own. No loop, and nothing for you to reset.

This is the single most useful thing on this page. If you reach for a `while` loop in a command or
in a subsystem's `periodic()`, there is almost always a trigger or a command composition that fits
better.

## What you will actually use

**If, else, and the logic operators**, everywhere. Conditions gate command scheduling, check
sensor readings, and choose between branches inside a mechanism's state machine.

**Classes and objects.** The whole robot is built out of them. Every mechanism is a class, and one
orchestrator class holds them and decides what each one should be doing. See
[Class Generation](../coding-conventions/class-generation.md) for how a mechanism is laid out in
this repo.

**Enums, heavily.** A mechanism with several named modes usually holds them in an enum, and a
`switch` on that enum drives what happens in each mode. The compiler checks that you covered every
value, which is most of the reason to use an enum over a string or a number.

**Plain `for` loops**, in the places where you need to touch every value in a fixed size array.
Use the enhanced form when you do not need the index. See [Loops](loops.md) and
[Arrays and enums](arrays.md).

**Lambdas and method references**, constantly. Anything that holds a setpoint which may need to
change while a command runs takes a supplier rather than a plain number, so the command re-reads it
every loop instead of freezing the value it was given. `config::getLimit` and `() -> config.getLimit()`
mean the same thing. See [Classes, methods, and objects](classes-methods-objects.md).

## What is rare

`while` and `do-while` loops turn up in utility and setup code, but almost never in a command body
or in a subsystem's `periodic()`. If you want one there, look for a trigger first.

Raw arrays show up at the edges, collecting a fixed set of readings or handing data to a WPILib
call that wants an array. For a collection that grows or shrinks, use a `List` or let WPILib hold
it.

---

*Previous: [Code formatting and comments](formatting-code.md), Up next: the [reference docs](../index.md#i-already-know-how-to-program-show-me-the-reference) on tools, dependencies, and conventions.*
