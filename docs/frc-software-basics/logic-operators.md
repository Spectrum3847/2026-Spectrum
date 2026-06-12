# Logic-Based Operators

*Audience: New programmers. Assumes you've read [Variables & Arithmetic](variables-arithmetic.md).*

## Comparison Operators

These compare two values and return a `boolean`.

| Operator | Meaning |
| --- | --- |
| `>` | greater than |
| `>=` | greater than or equal to |
| `<` | less than |
| `<=` | less than or equal to |
| `==` | equal to |
| `!=` | not equal to |

`5 > 4` is `true`. `5 >= 6` is `false`. `5 != 5` is `false`. Those work exactly as you'd expect.

One gotcha: `==` checks value equality for primitives, but for objects (like `String`) it checks whether both variables point to the same object in memory, not whether they contain the same text. For strings, use `.equals()`:

```java
String s1 = "Hello World";
String s2 = "Hello World";
s1.equals(s2);  // true — compare content
s1 == s2;       // may be false — compares memory address
```

## Logical Operators

`&&` (AND) returns `true` only when both sides are `true`. `||` (OR) returns `true` when either side is `true`.

```java
(5 == 5) && (4 == 4)   // true  — both sides true
(5 == 5) || (5 == 4)   // true  — left side is true
(5 == 4) && (5 == 5)   // false — left side is false
(5 == 4) || (5 == 3)   // false — both sides false
```

In robot code you see these constantly in trigger compositions:

```java
pilot.LT.and(pilot.RT).onTrue(applyState(State.LAUNCH_WITHOUT_SQUEEZE));
pilot.LT.or(pilot.RT).onFalse(applyState(State.IDLE));
```

`&&`/`||` are short-circuit operators. For `&&`, if the left side is `false`, Java doesn't evaluate the right side. For `||`, if the left side is `true`, it stops. That matters when the right side has a side effect or could throw an exception.

## Conditional Statements

### If

Runs a block when the condition is `true`. Nothing happens if it's `false`.

```java
if (launcher.isAttached()) {
    launcher.configPIDGains(kP, kI, kD);
}
```

### If-Else

Picks one of two paths.

```java
if (isSimulation()) {
    initializeSimDrivetrain();
} else {
    initializeRealDrivetrain();
}
```

### Else-If

Chains multiple conditions. Java evaluates them top to bottom and takes the first branch that matches.

```java
if (state == State.TRACK_TARGET) {
    aimAtTarget();
} else if (state == State.INTAKE_FUEL) {
    runIntake();
} else {
    neutral();
}
```

Once any branch runs, the rest are skipped. That's different from writing three separate `if` statements, which would all be evaluated independently.

### Switch

When you're branching on a single enum or integer value, a `switch` often reads more clearly than a stack of `else if`. The codebase uses Java's modern arrow-syntax switch expressions in `State.java`:

```java
private static BooleanSupplier isReadyState(State state) {
    return () ->
            switch (state) {
                case TRACK_TARGET -> true;
                default -> false;
            };
}
```

The arrow form doesn't fall through between cases, so you don't need `break` statements. The older colon-syntax switch does fall through unless you `break` explicitly — that's a common source of bugs if you're used to the newer form.

## Strings

`String` isn't a primitive type but it's used constantly. It's immutable — operations return a new string rather than modifying the original.

```java
String name = "Spectrum";
```

Common operations:

```java
String s = "Hello ";
String s1 = s + "World";        // concatenation: "Hello World"

String str = "IndexerBed";
str.length();                    // 10
str.charAt(0);                   // 'I'
str.substring(0, 7);             // "Indexer"
str.contains("Bed");             // true
str.toUpperCase();               // "INDEXERBED"
str.toLowerCase();               // "indexerbed"

"Hello".equals("Hello");         // true
"Hello".equals("hello");         // false — case-sensitive
"Hello".toLowerCase().equals("hello".toLowerCase());  // true
```

In log messages you'll often see string concatenation with `+` to attach a variable value:

```java
Telemetry.print(getName() + " Subsystem Initialized");
```

---

*Previous: [Variables & Arithmetic](variables-arithmetic.md) — Next: [Arrays & Enums](arrays.md)*
