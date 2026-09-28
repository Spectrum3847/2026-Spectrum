# Applied to FRC

*Audience: New programmers. Assumes you've read [Formatting code and comments](formatting-code.md).*

The lessons in this folder are about Java, and Java is the same everywhere. This page is the bridge to the one framework on top of it. WPILib does not change the way a language does, so what is written here stays true.

## The scheduler owns the loop

This is the single biggest difference between Java on a laptop and Java on a robot, and it is worth understanding before you write a subsystem.

In a normal Java program, you write the loop. In a command-based robot, you do not. WPILib starts your program and calls your code on a schedule it owns. Each call runs `periodic()` on every subsystem, then runs whatever commands are currently scheduled. You never write that loop. `TimedRobot`'s default period is 20 ms, and this project keeps it: `SpectrumRobot` calls the no-argument `super()`, so the robot runs 50 loops a second. The 0.20 in its constructor is not a loop interval. It goes to `CommandScheduler.setPeriod` and to `Watchdog.setTimeout`, and WPILib documents `setPeriod` as setting the expected period of the main loop, which makes it the watchdog's tolerance for a loop that overruns. Read 0.20 as a warning threshold, and assume fifty loops a second.

So a subsystem describes a thing, and its `periodic()` describes one frame of it. Setpoints get applied, sensors get read, and whatever the driver asked for takes effect. Then the frame is over.

```java
public class Elevator extends SubsystemBase {
    private final Motor motor = new Motor(7);
    private double setpoint = 0;

    @Override
    public void periodic() {
        motor.setVelocity(setpoint);
    }

    public void setSetpoint(double rpm) {
        setpoint = rpm;
    }
}
```

Notice what is not here. There is no loop. This method runs, and then it is done, and 20 milliseconds later it runs again. Everything the elevator does happens inside those 20 ms slices. The `Motor` class above is a stand-in for whatever library reads your motor controller.

## Why a while loop is wrong in periodic

The hand-rolled version is the mistake everyone makes first. It looks obvious, and it is a bug.

```java
// do not do this
public void periodic() {
    while (buttonHeld) {
        motor.set(0.4);
    }
}
```

This does not run slowly. It runs the inner `while` as fast as the processor can, and nothing else ever gets a turn. Your other subsystems stop being called, the driver station stops hearing from the robot, and the match ends. Java does re-evaluate `buttonHeld` at the top of every iteration, but if that value is maintained by a WPILib trigger, then the loop is exactly what stops the scheduler from polling that trigger, so the read you get can be a stale one. A release that happens while the thread is stuck inside is never seen.

The same code is correct when you never write the loop yourself, because the scheduler ends the command for you. A trigger bound to a button runs a command while the button is held and ends it the moment the button is released.

```java
driver.leftBumper().whileTrue(Commands.run(elevator::moveUp));
```

`whileTrue` is the whole abstraction: "keep doing this while that is true". The scheduler checks the trigger, starts the command on a false-to-true edge, and the command's `end()` runs on the true-to-false edge. You get the loop without writing one, and the loop cannot be faster than the scheduler. One caveat worth knowing early: the trigger controls the command's lifetime, not its repeat count. A one-shot command that finishes on its own while the button is still held is not started again, so a mechanism that has to keep moving wants a looping command, not a one-shot one.

The same idea covers most cases where you would have reached for `while`. `onTrue` runs something once when a condition becomes true. `onFalse` runs it when the condition goes away. `until` ends a command as soon as a condition becomes true. Triggers also combine, so either of two drivers' buttons can hold the same command, and requiring two buttons at once is a couple of lines instead of state variables and a timer.

## What you will use most

**Conditionals and logic operators.** They gate everything. Whether a command should start, whether a subsystem should act, whether a value passed a limit. The short-circuit rules from the logic lesson matter more here than anywhere, because guarding a sensor read with `if (sensor != null && sensor.isValid())` is the difference between a warning and a crash at a competition.

**Classes and objects.** Each mechanism is a class, you create one object of it, and that object holds the hardware and the current state for the whole match. Other code asks the object to do things through its `public` methods. See [Class Generation](../coding-conventions/class-generation.md) for how we lay those classes out.

**Enums.** Anything with a fixed set of named modes is an enum, and a `switch` over an enum is how you respond to them. A `switch` expression with no `default` has to cover every value, so adding a mode later is a compile error at each site that needs updating. A `switch` statement can leave values out, and the ones you left out do nothing.

**Loops over fixed arrays.** Needed where you touch every element of something that never changes size, such as a set of positions or a table of constants. See [Loops](loops.md) and [Arrays and enums](arrays.md).

**Lambdas and method references.** Where a value can change while a command is running, pass something that fetches the value rather than the value itself, so a setpoint you are still tuning takes effect without restarting anything. WPILib only does this where its own signature asks for a supplier. Most of its command factories take a `Runnable` or a plain value, and the way to get a live read there is to let the runnable read the value itself. The pattern then spreads to your own methods, which is where the supplier overload is worth having.

```java
// runMotor is a method of ours, not a WPILib one. Give it both overloads.
runMotor(() -> tunedSetpoint);  // asks for the value every time it is called
runMotor(tunedSetpoint);       // asks once, and that is the number it uses
```

The lambda is `() -> tunedSetpoint`. It looks like a value but it is a function, and the difference above is the whole reason the supplier form exists. `config::getSetpoint` is the same thing written shorter. See [Programming tips](../other-guides/tips.md#doublesupplier-vs-double) and [Classes, methods, and objects](classes-methods-objects.md).

## What is rare

`while` and `do-while` loops inside `periodic()` or inside a command body. If you are reaching for one, there is almost always a trigger or a command composition that does the same job, and it will be correct by construction.

Raw arrays. They show up at the edges, where a WPILib API expects an array or you are collecting values for a log. Inside a subsystem, reach for a `List` instead.

---

*Previous: [Formatting code and comments](formatting-code.md), Up next: the [reference docs](../index.md#i-already-know-how-to-program-show-me-the-reference) on tools, dependencies, and conventions.*
