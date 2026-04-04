# Classes, Methods, and Objects

## Methods

Methods are actions that objects can perform. In Java, using a method is called "invoking." Parameters are values that are passed into a method.

### Invoking a Method

```java
class/object.doSomething(parameters);
```
This calls a method with the parameters passed in. If a method takes no parameters, a blank pair of parentheses is used.

### General Method Format

```java
scope returnType methodName(parameters) {
    // Method Body
    return value; // (if not void)
}
```

You don't need to call the class/object name for the method if the method is in the same scope where it is being called.

### Lambdas

Lambda expressions allow methods to be passed in as parameters.

We use them when creating Commands and whenever we call config values in Commands as Suppliers.

```java
public static Command move(DoubleSupplier percent) {
    return moveToPercentage(percent);
}

// Using a lambda expression
move(() -> config.getPercent());
// Or using a method reference (shorthand for lambda if method only calls one thing)
move(config::getPercent);
```

`() -> config.getPercent()` is equivalent to defining an anonymous method:
```java
public double someName() {
    return config.getPercent();
}
```
Lambdas are especially useful for returning config values of a mechanism when using Getter/Setter methods.

## Scope

Access modifiers are added to variables and methods to control access.

*   `private`: Restricts variable/method access to the declaring class.
*   `public`: Variables/methods can be accessed from anywhere.
*   No modifier (package-private): Variables/methods can be accessed only within the same package.
*   Other access modifiers like `protected` are also used.

We typically keep variables `private` and most methods `public`.

### Scope Example

**`Elevator.java`**
```java
public class Elevator {
    public void extendArm() {
        // ...
        // getGoals(); // cannot access goals variable from Intake
    }
}
```

**`Intake.java`**
```java
public class Intake {
    private int goals;

    public Intake() {
        goals = 5;
    }

    public int getGoals() {
        return goals;
    }
}
```

## Return Type

Returning is sending back a value from a method.

*   In Java, methods can only return one value.
*   If the method does not return anything, `void` is used in the header.
*   The data type that is returned by the method must be specified in the header (e.g., `String`, `int`, `boolean`). Data types also include class objects as return types.

## Classes and Objects

*   **Class**: A template/blueprint for objects.
    *   File names must match the name of the class.
    *   Usage of Encapsulation: Each mechanism on the robot is a class and contains all the variables and commands we can call on the mechanism.
*   **Objects**: Instances of a class. An object is like a cookie and a class is like a cookie cutter.

### Generalization: Car Example

**`Car.java`**
```java
public class Car {
    private int position;

    // Constructor: defines how to initialize the class
    public Car(int pos) {
        position = pos;
    }

    // Method: can change or access fields
    public void run() {
        position = position + 2;
    }
}
```

## Static and Non-Static

### Static

The `static` keyword means that the method or field belongs to the **class** itself, not to specific **objects** of the class.

*   Static methods and attributes are called with the name of the class they belong to (e.g., `ClassName.staticMethod()`).
*   Made for shared data, constants, and behaviors shared between multiple subsystems (e.g., tracking the number of users).

### Non-Static

There is no specific keyword for non-static; its absence indicates non-static.

*   Non-static methods and attributes are called with the name of the **instance created/object** (e.g., `objectName.nonStaticMethod()`).
*   Use non-static when data is unique to the specific instance of a class (e.g., tracking the score of each user).

### Robot Example: Intake Subsystem

**`Intake.java`**
```java
public class Intake {
    private int position;

    public Intake(int pos) {
        position = pos;
    }

    public void run() {
        position = position + 2;
    }
}
```

**`IntakeStates.java`**
```java
public class IntakeStates {
    private static Intake intake = Robot.getIntake(); // Static reference to the Intake instance
    private static int speed;

    public static void run() {
        intake.run(); // Calls the non-static run() method on the static intake object
    }
}
```
*In `IntakeStates.java`, `Robot.getIntake()` (assumed to be a static method) provides a shared instance of `Intake`. The `run()` method in `IntakeStates` is `static` but operates on the `intake` object, which is an instance, demonstrating interaction between static and non-static elements.*
