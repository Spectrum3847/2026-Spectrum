# Formatting Code

*Audience: New programmers. Assumes you've read [Classes, Methods, & Objects](classes-methods-objects.md).*

## Java Basics

Java files end in `.java` and the filename must match the public class name inside. Execution in a standard Java program starts from `main()`, but in a WPILib robot project `Robot.java` is the entry point — WPILib calls `robotInit()`, `teleopPeriodic()`, and so on rather than a `main()` you write yourself.

Java uses braces `{}` to group code into blocks. Indentation is cosmetic — the compiler ignores whitespace and relies on braces to know what belongs where. A method body is everything between its opening and closing brace; an `if` block is everything between its pair of braces.

```java
void moveRobot() {
    moveForward();   // inside moveRobot
}                    // end of moveRobot

int a = 5;
int b = 1;
if (a > b) {
    System.out.println("a greater than b");
} else {
    System.out.println("a not greater than b");
}
```

Every statement ends with a semicolon. Missing one is a compile error — Java won't try to guess where statements end the way Python does.

## Spotless Formatting

You don't have to think much about code style manually. Spotless reformats every `.java` file automatically when you build. The full rules are in [Gradle](../tools/gradle.md), but the short version: run `./gradlew spotlessApply` if CI complains about formatting, and the tool fixes it for you.

If you have a block of code that needs to stay hand-aligned (a matrix of numbers, for example), wrap it:

```java
// spotless:off
double[][] matrix = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
};
// spotless:on
```

## Comments

Single-line comments start with `//`. Anything after those two slashes on the same line is ignored by the compiler.

```java
double idlingRPM = 700;  // rotations per minute
```

JavaDoc comments use `/** ... */` and attach documentation to classes and methods. Any `public` method — especially on a `*States` class — should have at least a one-line JavaDoc so tooling and teammates can read what it does without opening the implementation:

```java
/**
 * Schedules the launcher velocity command only if it isn't already running.
 *
 * @param command the command to schedule
 */
public static void scheduleIfNotRunning(Command command) { ... }
```

Block comments (`/* ... */`) exist but are less common than `//` in this codebase. Use `//` for inline notes and JavaDoc `/** */` for public API documentation.

---

*Previous: [Classes, Methods, & Objects](classes-methods-objects.md) — Next: [Applied to FRC](applied-to-frc.md)*
