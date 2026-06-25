# Arrays

*Audience: New programmers. Assumes you've read [Logic-Based Operators & Strings](logic-operators.md).*

An array holds a fixed number of values of the same type, indexed starting at zero. You declare the type, then the name, with `[]` after the type to signal it's an array.

```java
int[] x = {3, 8, 4, 7};
// x[0] is 3, x[1] is 8, x[2] is 4, x[3] is 7
```

The size is fixed at creation. Once you create an `int[4]`, it holds exactly four ints for its lifetime. If you need a resizable list, Java has `ArrayList` — but for most robot code, fixed arrays are fine.

## Declaring and Initializing

You can declare an array and fill it in one line (as above), or declare it first and initialize separately:

```java
double[] positions = new double[4];   // four doubles, all initialized to 0.0
positions[0] = 1.5;
positions[1] = 2.3;
// etc.
```

In `Vision.java`, the April tag IDs for each alliance are stored as int arrays:

```java
int[] blueTags = {18, 19, 20, 21, 24, 25, 26, 27};
int[] redTags  = {2, 3, 4, 5, 8, 9, 10, 11, 12};
```

Arrays of objects work the same way. `Vision.java` groups its three Limelights:

```java
allLimelights = new Limelight[] {backLL, leftLL, rightLL};
```

Once that array exists, a single enhanced `for` loop can update all three:

```java
for (Limelight limelight : allLimelights) {
    limelight.setRobotOrientation(yaw);
}
```

`SwerveStates` uses a `double[4]` to collect per-module positions for wheel-radius characterization:

```java
double[] positions = new double[4];
for (int i = 0; i < 4; i++) {
    positions[i] = swerve.getModule(i).getCachedPosition().distanceMeters / wheelRadiusGuess;
}
```

See [Loops](loops.md) for more on the `for` patterns used with arrays.

## Enums

Enums define a fixed set of named constants, and they're far more common in this codebase than raw arrays. Every mechanism's operating modes are expressed as enum values. `State.java` defines the top-level robot states:

```java
public enum State {
    IDLE,
    INTAKE_FUEL,
    TRACK_TARGET,
    LAUNCH_WITH_SQUEEZE,
    LAUNCH_WITHOUT_SQUEEZE,
    UNJAM,
    // ...
}
```

Enums are useful wherever you'd otherwise use a magic string or integer to represent a category. A `switch` on an enum is cleaner than a chain of `if/else` comparisons, and the compiler catches typos because only declared values exist:

```java
switch (state) {
    case TRACK_TARGET -> true;
    default -> false;
}
```

For how this pattern fits into subsystem design, see [Class Generation](../coding-conventions/class-generation.md).

## Array Errors

`ArrayIndexOutOfBoundsException` — you accessed an index outside the valid range. If the array has four elements, valid indices are 0 through 3. Accessing index 4 (or any negative index) throws this at runtime.

`NullPointerException` — you declared an array of objects but never initialized the individual elements. An uninitialized slot holds `null`, and calling a method on `null` throws immediately.

---

*Previous: [Logic-Based Operators & Strings](logic-operators.md) — Next: [Loops](loops.md)*
