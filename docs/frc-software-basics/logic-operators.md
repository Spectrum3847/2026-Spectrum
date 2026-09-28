# Logic operators and strings

*Audience: New programmers. Assumes you've read [Variables and arithmetic](variables-arithmetic.md).*

A comparison is an expression that evaluates to `true` or `false`. `>` is greater than, `<` is less than, `>=` and `<=` include the equal case, `==` is equal, and `!=` is not equal.

```java
int a = 5;
int b = 3;

a > b    // true
a >= 5   // true
b == 3   // true
a != b   // true
```

## Comparing strings

Here is the gotcha that catches nearly everyone. `==` on two numbers compares their values. On two objects, including strings, it compares whether the two variables point at the same object in memory. Two strings holding the same text are two separate objects, so `==` answers a question about memory rather than about text, and the answer can be `false`.

```java
String s1 = new String("Hello World");  // its own object
String s2 = "Hello World";              // a different object, same text

s1 == s2;       // false, and it is meant to be false
s1.equals(s2);  // true, this compares the text
```

Note that the two string literals above are not a safe example. Written as `String s2 = "Hello World";` after an identical literal, the compiler merges them into one object, `==` would answer `true`, and the example would quietly teach the opposite lesson. That shortcut applies to literals only, which is why the example above builds the first string explicitly. Use `.equals()` for text and you never have to think about any of it.

If one side might be `null`, check for that first, because calling a method on `null` crashes the program. [NullPointerException](arrays.md#two-errors-you-will-hit) is covered there.

## Combining conditions

`&&` is and. It is `true` only when both sides are `true`. `||` is or. It is `true` when at least one side is `true`. `!` in front of a whole condition flips it.

```java
if (age >= 16 && hasPermit) {
    // both must be true
}
```

Both operators short-circuit, which means Java stops as soon as it knows the answer. With `&&`, a `false` left side means the right side is never evaluated. With `||`, a `true` left side means the same. This is not just a speed trick. It is also what lets you guard a risky expression.

```java
String name = null;

if (name != null && name.length() > 3) {  // safe, the right side never runs if name is null
    // ...
}
```

Written the other way around, `name.length()` runs first and throws, because the `&&` has no reason to stop yet.

## Conditionals

An `if` runs its block when the condition is `true`, and does nothing when it is `false`.

```java
if (balance < 0) {
    System.out.println("Overdrawn");
}
```

Add an `else` to handle the other case.

```java
if (balance < 0) {
    System.out.println("Overdrawn");
} else {
    System.out.println("In the black");
}
```

For more than two cases, chain `else if`. Java reads the chain from the top and takes the first branch whose condition is true, then skips the rest.

```java
if (score >= 90) {
    grade = "A";
} else if (score >= 80) {
    grade = "B";
} else {
    grade = "C";
}
```

Gotcha: one chain, first match wins. Three separate `if` statements are three independent questions, and all three can run.

## Switch

When you are branching on a single value, a `switch` can read better than a long chain of `else if`. The arrow form below assigns one value per case and needs no `break`, because a case cannot fall through into the next one.

```java
char letter = 'B';
String section;

switch (letter) {
    case 'A' -> section = "first";
    case 'B' -> section = "second";
    case 'C' -> section = "third";
    default -> section = "unknown";
}
```

The older colon form, where each case ends with `break`, does fall through to the next case when you forget the `break`. That is a common bug, and one reason the arrow form replaced it. Java also lets a `switch` expression produce the value directly, which removes the temp variable in the example above.

## Strings

A `String` holds text. It is immutable, which means no method can change it. Every operation that looks like it edits a string actually builds a new one and hands it back.

```java
String first = "Hello";
String greeting = first + ", " + "team";  // "Hello, team"
String loud = greeting.toUpperCase();     // a new string, greeting is unchanged
```

Use `+` to glue text and values together. The value gets converted to text for you.

```java
int count = 3;
System.out.println("Items left: " + count);  // Items left: 3
```

`.equals()` is case-sensitive, so `"Hello".equals("hello")` is `false`. If you want to ignore case, convert both sides with `.toLowerCase()` first. You will reach for `.length()`, `.substring()`, and `.contains()` often; the [Java API docs](https://docs.oracle.com/en/java/javase/17/docs/api/) list the rest.

---

*Previous: [Variables and arithmetic](variables-arithmetic.md). Next: [Arrays and enums](arrays.md)*
