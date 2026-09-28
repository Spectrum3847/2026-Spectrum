# Formatting code and comments

*Audience: New programmers. Assumes you've read [Classes, methods, and objects](classes-methods-objects.md).*

You do not have to think about indentation or line length. A formatter runs as part of every build and rewrites your files to match the team style, so save your file, run the build, and read the result. [Code Style](../coding-conventions/code-style.md) has the naming rules, the file layout, and how to tell the formatter to leave a block alone.

What the formatter cannot do is write the comment that explains why the code is the way it is. That part is yours.

## Line comments

Everything after two slashes on a line is ignored by the compiler. That makes it the right tool for a short note to the next reader.

```java
double taxRate = 0.0825;  // a published rate, change it in one place only
```

Use it to explain a choice, not to narrate the code. `double taxRate = 0.0825;  // set the tax rate` tells a reader nothing they cannot see. The comment earns its place when the answer is not in the code: why this number, why here, why not the obvious approach.

```java
// Subtract a small bias so the reading settles at zero instead of jittering around it.
double corrected = raw - 0.05;
```

Avoid comments about history. A note that says what used to be here goes stale the next time someone changes it, and then it is worse than nothing because someone trusts it.

## JavaDoc comments

A JavaDoc comment is a block comment that starts with `/**` and ends with `*/`. It attaches to the class, method, or field below it, and it is what your editor shows when you hover.

```java
/**
 * Moves the object toward a target position.
 *
 * @param target the position to move toward
 * @return the distance still left to travel
 */
public double moveToward(double target) {
    double remaining = target - position;
    double step = Math.signum(remaining) * Math.min(1, Math.abs(remaining));
    position += step;
    return Math.abs(remaining) - Math.abs(step);
}
```

The opening `/**` must be immediately above the thing it documents, and every line in the body starts with an asterisk. The tags carry the details: `@param` for each input, `@return` for the result, and `@throws` for what can go wrong. Put the units in the description, so a reader never has to guess whether that number is metres or feet. What goes in the body is the part a reader cannot get from the signature, not an announcement of what the method is about to do. In the example above, `Math.min(1, ...)` is what stops the last step from carrying the position past the target when the target is less than one unit away, and the returned value is measured after the move, so it is the distance actually left.

Put JavaDoc on anything `public` that other code calls, and on any field where the units or the valid range are not obvious from the name. [Documentation and Comments](../coding-conventions/documentation-and-comments.md) has the rest of our rules on when a comment helps and when it is just noise.

## Block comments

A plain `/* ... */` comment spans several lines and is not attached to anything in particular. JavaDoc grew out of this and replaced it for anything that documents code.

```java
/*
 * The layout below mirrors the order of the fields above it.
 * Keep them together or the reader loses track.
 */
```

Use `//` for notes and `/** */` for documentation. That is nearly always the right split.

---

*Previous: [Classes, methods, and objects](classes-methods-objects.md). Next: [Applied to FRC](applied-to-frc.md)*
