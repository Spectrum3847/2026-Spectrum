# Loops

*Audience: New programmers. Assumes you've read [Arrays & Enums](arrays.md).*

A loop runs a block of code repeatedly — either a fixed number of times or until some condition changes. Read [Applied to FRC](applied-to-frc.md) first for context on when loops appear in robot code versus when the command-based framework handles repetition for you.

## For Loop

Use a `for` loop when you know how many iterations you need. The three parts of the header are: initialize a counter, state the condition that keeps it running, and update the counter each iteration.

```java
for (int i = 0; i < 4; i++) {
    // runs with i = 0, 1, 2, 3
}
```

In `SwerveStates`, this pattern reads back position data from all four swerve modules:

```java
double[] positions = new double[4];
for (int i = 0; i < 4; i++) {
    positions[i] = swerve.getModule(i).getCachedPosition().distanceMeters / wheelRadiusGuess;
}
```

### Enhanced For Loop

When you want to visit every element in an array or collection without tracking an index yourself, the enhanced `for` loop is cleaner:

```java
for (ElementType element : collection) {
    // use element
}
```

`Vision.java` uses this to update all three Limelights in one pass:

```java
for (Limelight limelight : allLimelights) {
    limelight.setRobotOrientation(yaw);
}
```

`BatteryLogger` uses it to sum current draw across subsystems:

```java
for (double amp : amps) totalAmps += Math.abs(amp);
```

Use the enhanced form when you don't need the index. Use the indexed form when you do — for example, reading `positions[i]` alongside `state.positions[i]` in the same loop.

## While Loop

A `while` loop runs as long as a condition stays `true`. You'd use it when you don't know the iteration count up front.

```java
while (condition) {
    // body
}
```

In command-based robot code, true `while` loops doing I/O or sensor polling inside `periodic()` are uncommon — the scheduler's 20 ms loop handles that for you. But the concept shows up in setup code or utility methods where you're waiting on a result before continuing.

```java
boolean targetFound = false;
while (!targetFound) {
    targetFound = checkSensorForTarget();
}
```

One risk: if the condition never becomes `false`, you have an infinite loop, and the program hangs. Always make sure something inside the loop can make the condition change.

## Do-While Loop

Like a `while`, but the body runs once before the condition is checked. That guarantees at least one execution.

```java
int targetPos = 0;
do {
    targetPos = calculatePos();
} while (targetPos != 0);
```

This is uncommon in the codebase — only reach for it when you need that "run at least once" guarantee.

## Variables Inside vs. Outside Loops

Where you declare a variable determines how it persists across iterations.

```java
int x = 0;
for (int i = 0; i < 4; i++) {
    x += i;   // x accumulates: 0, 1, 3, 6
}
// x is 6 after the loop
```

```java
for (int i = 0; i < 4; i++) {
    int y = i;   // y is brand new each iteration: 0, 1, 2, 3
    // y disappears when this brace closes
}
```

Declare outside the loop when you need to use the value after it finishes (like `positions[]` in the swerve example above). Declare inside when each iteration is independent.

## Common Loop Errors

An infinite loop — one whose condition is always `true` — will freeze the robot or the simulation. The most common cause is a counter that moves the wrong direction:

```java
// infinite loop: i starts at 24, increments, condition i > 6 is always true
for (int i = 24; i > 6; i++) { ... }
```

Syntax errors the compiler will catch: forgetting `int` before the counter variable, missing semicolons in the `for` header, or unmatched braces.

---

*Previous: [Arrays & Enums](arrays.md) — Next: [Classes, Methods, & Objects](classes-methods-objects.md)*
