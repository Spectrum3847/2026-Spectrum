# Logic operators and strings

*Audience: New programmers. Assumes you've read [Variables and arithmetic](variables-arithmetic.md).*

The examples on this page are invented, not taken from this robot's code.

## Comparing values

A comparison takes two values and gives you back a `boolean`.

```java
boolean bigger = 5 > 4;      // true
boolean tooBig = 5 >= 6;     // false
boolean smaller = 4 < 3;     // false
boolean notBigger = 4 <= 4;  // true
boolean same = 5 == 5;       // true
boolean different = 5 != 4;  // true
```

`==` works the way you expect on numbers. On objects it does not, and that is the one beginner bug
worth memorizing. For an object, `==` asks "are these two variables pointing at the same thing in
memory", not "do these hold the same value".

```java
String a = new String("Hello");
String b = new String("Hello");

a == b;       // false, two separate objects
a.equals(b);  // true, the same text
```

Using two string literals is the confusing case. Java is allowed to make them the same object, so
`==` might happen to be `true`, and might not be. Never depend on it.

```java
String x = "Hello";
String y = "Hello";

x.equals(y);   // true, always
x == y;        // true or false, depending on the run
```

For text, use `.equals()`.

## Combining conditions

`&&` is "and". It is `true` only when both sides are `true`. `||` is "or". It is `true` when at
least one side is `true`. A single `!` in front flips a `boolean`.

```java
boolean both = (5 == 5) && (4 == 4);       // true
boolean neither = (5 == 4) && (5 == 5);    // false
boolean either = (5 == 5) || (5 == 4);     // true
boolean notEither = (5 == 4) || (5 == 3);  // false
boolean flipped = !true;                   // false
```

Both `&&` and `||` stop early. With `&&`, if the left side is `false`, Java never looks at the
right side. With `||`, if the left side is `true`, it never looks at the right side either. That
gives you a way to guard the risky half of a condition:

```java
if (name != null && name.length() > 3) {
    // the null check runs first, so length() is only called on real text
}
```

Without the short circuit, `name.length()` would run on a `null` and throw.

## Branching

### If

Runs a block when the condition is `true`, and does nothing when it is `false`.

```java
double balance = 100.0;

if (balance > 0) {
    System.out.println("in the black");
}
```

### If and else

Chooses one of two paths.

```java
if (balance > 0) {
    System.out.println("in the black");
} else {
    System.out.println("overdrawn");
}
```

### Else if

Chooses one of three or more paths. Java reads the conditions top to bottom and runs the first one
that matches.

```java
int score = 42;

if (score >= 90) {
    System.out.println("excellent");
} else if (score >= 60) {
    System.out.println("passing");
} else {
    System.out.println("failing");
}
```

With `score` at 42 that prints `failing`, because 42 fails both tests.

Once one branch runs, the rest are skipped. That is different from writing the conditions as
separate `if` statements, which are each tested on their own, so more than one of them can run.

```java
int score = 95;

if (score >= 60) { ... }   // runs
if (score >= 90) { ... }   // also runs
```

### Switch

A `switch` picks a branch from one value. With the arrow form shown here, each case returns its own
result and there is no falling through, so no `break` is needed.

```java
enum Light {
    RED,
    YELLOW,
    GREEN
}

String advice(Light light) {
    return switch (light) {
        case RED -> "stop";
        case YELLOW -> "slow down";
        case GREEN -> "go";
    };
}

advice(Light.GREEN);   // "go"
```

The older colon form, which uses `break` instead of `->`, does fall through to the next case unless
you break out. If you have seen the arrow form first, the colon form is a common source of bugs.

## Strings

A `String` holds text. It is not a primitive type, and it is immutable, which means it cannot be
changed. Every operation gives you a brand new string and leaves the original alone.

```java
String a = "Hello";
String b = a.toUpperCase();

a;   // still "Hello"
b;   // "HELLO"
```

Joining text with `+` works, and you will use it constantly to build messages:

```java
String name = "Ada";
String greeting = "Hello, " + name;   // "Hello, Ada"

System.out.println("Opened an account for " + name);
```

You will not need the whole catalogue of `String` methods. A free Java reference has it. These
three cover most of what you need at first:

```java
String name = "Ada";

name.length();               // 3
name.substring(0, 1);        // "A"
name.equalsIgnoreCase("ada") // true
```

Text comparison is case sensitive, so if you need to ignore case, say so:

```java
"Hello".equals("hello");                  // false
"Hello".equalsIgnoreCase("hello");        // true
"Hello".toLowerCase().equals("hello");    // true
```

---

*Previous: [Variables and arithmetic](variables-arithmetic.md). Next: [Arrays and enums](arrays.md)*
