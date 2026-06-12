# Variables & Arithmetic

*Audience: New programmers. Assumes you've completed [Setup](../setup.md).*

Java is a statically typed language, which means every variable has a fixed type declared up front, and you end each statement with a semicolon. If you come from Python, both of those things feel unusual at first.

```java
int a = 5;   // declare type, name, and initial value
a = 3;       // reassign — fine
```

These will not compile:

```java
a = "Java";      // type mismatch: a is an int, not a String
a = 3            // missing semicolon
String a = "Java"; // 'a' is already declared in this scope
int a = 3;         // same problem
```

## Primitive Types

The types you'll encounter most often in this codebase:

| Type | What it holds | Example |
| --- | --- | --- |
| `int` | whole numbers | `int canID = 8;` |
| `double` | decimal numbers | `double idlingRPM = 700;` |
| `boolean` | `true` or `false` | `boolean isAttached = true;` |
| `String` | text (not technically primitive, but used everywhere) | `String name = "IndexerBed";` |
| `byte` | small signed integer, −128 to 127 | rare in robot code |

You'll also see `final` (the value can't be reassigned) and `static` (belongs to the class rather than a specific instance) used as modifiers. Both are covered in [Classes, Methods, and Objects](classes-methods-objects.md).

In the `Launcher.LauncherConfig` class, for instance, the config fields look like this:

```java
@Getter @Setter private double idlingRPM = 700;
@Getter @Setter private double slowLaunchSpeed = 400;
@Getter @Setter private double autoTrenchLaunch = 1800;
```

All `double`s because RPM values have decimal precision. Changing them to `int` would silently truncate fractions.

## Arithmetic Operators

These require two operands — one on each side.

| Operator | Meaning | Example |
| --- | --- | --- |
| `+` | addition | `a + b` |
| `-` | subtraction | `a - b` |
| `*` | multiplication | `a * b` |
| `/` | division | `a / b` |
| `%` | modulus (remainder) | `a % b` |

A quick demonstration with concrete values:

```java
int a = 5;
int b = 2;

int result = a + b; // 7
result = a - b;     // 3
result = a * b;     // 10
result = a / b;     // 2  — integer division, decimal is truncated
result = a % b;     // 1  — remainder of 5 ÷ 2
```

Integer division truncates, it doesn't round. `3 / 2` gives `1`, not `1.5`. If you need the decimal, at least one side has to be a `double`:

```java
double result = 3.0 / 2;   // 1.5
```

In real robot code you see this with RPM error checks:

```java
double targetRPM = params.flywheelSpeed();
double currentRPM = getVelocityRPM();
double errorRPM = currentRPM - targetRPM;

return Math.abs(errorRPM) < config.getOnTargetToleranceRPM();
```

Operator precedence follows PEMDAS. Parentheses first, then multiplication and division (left to right), then addition and subtraction. Java has no exponent operator — use `Math.pow(base, exponent)` instead.

## Unary Operators

These work on a single value.

- `++` increments by 1 (`a++` or `++a`)
- `--` decrements by 1
- `!` negates a boolean — `!true` is `false`
- `-` flips the sign — `-a` where `a` is `5` gives `-5`

The distinction between `a++` and `++a` matters when the expression is used in an assignment (`int x = a++` vs `int x = ++a`), but in a standalone statement like a `for` loop counter they're equivalent.

## Compound Assignment Operators

Shortcuts that modify a variable in place:

```java
int a = 5;
a += 2;  // a is now 7
a -= 2;  // a is now 5
a *= 2;  // a is now 10
a /= 2;  // a is now 5
a %= 2;  // a is now 1
```

## The Math Class

`Math` is a built-in static class for operations that don't have a symbol:

```java
Math.sqrt(4)     // 2.0  — square root
Math.pow(4, 2)   // 16.0 — exponentiation
Math.abs(-4)     // 4    — absolute value
Math.PI          // 3.141592653589793
```

`BatteryLogger` uses `Math.abs` to sum up current draw across subsystems regardless of direction:

```java
for (double amp : amps) totalAmps += Math.abs(amp);
```

---

*Previous: [Setup](../setup.md) — Next: [Logic-Based Operators & Strings](logic-operators.md)*
