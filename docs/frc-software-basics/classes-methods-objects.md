# Classes, Methods, and Objects

*Audience: New programmers. Assumes you've read [Loops](loops.md).*

A class is a blueprint. An object is a specific instance built from that blueprint. In this codebase, every mechanism on the robot is a class: `Launcher`, `IndexerBed`, `Hood`, and so on. When `Robot.java` starts up, it constructs one object of each class, which owns that mechanism's motors and state for the whole match.

For how we structure those classes and their companion `*States` files, see [Class Generation](../coding-conventions/class-generation.md).

## Methods

A method is a named block of code the class can run. In Java, calling a method is "invoking" it.

```java
someObject.doSomething(parameter1, parameter2);
```

If there are no parameters, the parentheses are still required:

```java
launcher.stopMotor();
```

The general form of a method declaration:

```java
accessModifier returnType methodName(parameterType parameterName) {
    // body
    return value;  // omit this line if returnType is void
}
```

A method that doesn't return anything declares `void`. One that returns a `boolean` declares `boolean`, and so on. A method can return any type including object types — `LauncherStates.aimingAtTarget()` returns a `Trigger`, for example.

## Scope and Access Modifiers

`private` means only code inside this class can see this variable or method. `public` means anything can. No modifier at all (package-private) means only code in the same package can.

The convention in this codebase: fields are `private`, methods on `*States` classes are `public static`. That keeps internal mechanism data hidden while exposing a clean API to `Coordinator` and `RobotStates`.

```java
public class IndexerBed extends Mechanism {
    @Getter @Setter private double indexerVoltageOut = 8;  // private field, accessed via getter
    // ...
}
```

```java
public class LauncherStates {
    private static Launcher launcher = Robot.getLauncher();  // private — internal

    public static Command launchFuel() {                      // public — used by Coordinator
        return launcher.runTorqueFOC(config::getLauncherTorqueCurrent);
    }
}
```

## Static vs. Non-Static

A `static` method or field belongs to the class itself, not to any particular instance. You call it with the class name, not an object name:

```java
LauncherStates.launchFuel();  // static method on LauncherStates
Math.abs(-5);                  // static method on Math
```

Non-static (instance) methods and fields belong to a specific object. You call them on the object:

```java
launcher.stopMotor();          // instance method on a specific Launcher object
```

`*States` classes are all static because there's only ever one launcher, one indexer, etc. The static reference to the mechanism object (`private static Launcher launcher = Robot.getLauncher()`) is initialized once when `Robot.java` constructs everything.

## Constructors

A constructor runs once when an object is created. It sets up the object's initial state. Its name matches the class name and it has no return type:

```java
public class IndexerBed extends Mechanism {
    public IndexerBed(IndexerBedConfig config) {
        super(config);
        // motor setup, encoder wiring, etc.
    }
}
```

`super(config)` calls the parent class (`Mechanism`) constructor before doing any `IndexerBed`-specific setup.

## Lambdas and Method References

When a method expects a function as a parameter — like a `DoubleSupplier` or `Command` factory — you can pass a lambda rather than writing a whole named method:

```java
// lambda: () -> body
launcher.runVelocityTcFocRPM(() -> config.getIdlingRPM())

// method reference: shorthand when the lambda just calls one method
launcher.runVelocityTcFocRPM(config::getIdlingRPM)
```

Both are equivalent. The method reference form reads more clearly when there's nothing else in the lambda body. Lambdas are how commands in this codebase stay connected to live config values — if `idlingRPM` changes at runtime (through a `TuneValue`, say), the command sees the new value because it re-evaluates the supplier each loop.

This is covered further in [Tips](../other-guides/tips.md#doublesupplier-vs-double).

---

*Previous: [Loops](loops.md) — Next: [Formatting Code & Comments](formatting-code.md)*
