# Variables and arithmetic

*Audience: New programmers. Assumes you've completed [Setup](../setup.md).*

A variable is a named box for a value. Java is statically typed, which means you say what type goes in the box before you put anything in it, and every statement ends with a semicolon. If you are coming from Python, both of those feel odd at first.

```java
int count = 5;       // type, name, initial value
count = 3;           // reassign, fine
count = count + 1;  // read the value, use it, write a new one back
```

The type is locked in when you declare. These four lines do not compile:

```java
count = "five";      // a String cannot go in an int box
count = 3            // missing semicolon
int count = 1;       // count already exists in this scope
int count;           // same problem, a second name for one box
```

The last two lines are the same mistake. A name can only be declared once in a given scope, which is the rule beginners coming from Python miss first.

## The types you will actually use

Most code you write needs four types.

```java
int total = 12;               // whole numbers
double price = 4.99;          // decimal numbers
boolean isEmpty = true;       // true or false
String label = "north gate";  // text
```

`String` is not technically a primitive, but you will use it as much as the other three, so it belongs on the list.

The rule of thumb for picking: use `int` for counts and indexes, and use `double` for anything you measured. Measurements have decimals in them, and `int` throws the decimal part away.

`final` means the value can never be reassigned after it is set. Use it for values that are true for the whole program.

```java
final int MAX_ITEMS = 10;
MAX_ITEMS = 11;  // does not compile
```

`static` is the other modifier you will meet early. It means the field or method belongs to the class rather than to one object. [Classes, methods, and objects](classes-methods-objects.md) covers it properly.

## Arithmetic

Arithmetic works the way it does on paper. `+`, `-`, and `*` need no explanation. `/` divides, and `%` gives you the remainder.

```java
int a = 5;
int b = 2;

int result = a + b;  // 7
result = a - b;      // 3
result = a * b;      // 10
result = a / b;      // 2
result = a % b;      // 1, because 5 divided by 2 leaves a remainder of 1
```

Here is the first gotcha, and you will hit it on your first real bug. When both sides of a division are `int`, the answer is an `int` too, and the decimal part is dropped. It truncates, it does not round.

```java
int wrong = 3 / 2;      // 1, not 1.5
double right = 3.0 / 2;  // 1.5
```

If either side is a `double`, the whole expression is done in `double`. A common way to write this is to put the decimal on one side.

There is no `**` operator for powers. Use `Math.pow(4, 2)`, which gives `16.0`. `Math.abs(-5)` gives `5`. `Math` is a built-in class full of helpers like these, and you call them with `Math.` in front.

Precedence follows the usual order: parentheses first, then `*` and `/`, then `+` and `-`. When in doubt, add parentheses. They cost nothing and they tell the next reader what you meant.

## Unary operators

These take one value instead of two.

```java
int a = 5;
a++;      // a is 6
a--;      // a is 5
a = -a;   // a is -5
boolean b = !true;  // b is false
```

`!` only works on booleans. It flips `true` to `false` and back.

There is a difference between `a++` and `++a`, but only when you use the result in the same statement.

```java
int a = 5;
int x = a++;  // x is 5, then a becomes 6
int y = ++a;  // a becomes 7 first, y is 7
```

As a whole statement on its own line, `a++` and `++a` do the same thing. Loop counters are that case, so either is fine there.

## Compound assignment

These shorten "add to the variable I already have."

```java
int a = 5;
a += 2;  // a is 7
a -= 2;  // a is 5
a *= 2;  // a is 10
a /= 2;  // a is 5
a %= 2;  // a is 1
```

The left side is a variable you already declared, never a literal. `5 += 2` does not compile.

---

*Previous: [Setup](../setup.md). Next: [Logic operators and strings](logic-operators.md)*
