# Variables and arithmetic

*Audience: New programmers. Assumes you've completed [Setup](../setup.md).*

Java is statically typed, which means every variable has a type you write down up front, and every
statement ends with a semicolon. If you have used Python, both of those feel odd at first.

The examples on this page are invented. They are not taken from this robot's code, and they are not
about robots. Learn the language here; read the robot's own code when you want to know what the
robot does.

```java
double balance = 250.00;   // type, name, and starting value
balance = 300.00;          // giving it a new value is fine
```

These will not compile:

```java
balance = "three hundred";  // balance is a double, "three hundred" is a String
balance = 300.00            // missing semicolon
```

And this will not compile either, because `balance` is already declared in the same scope:

```java
double balance = 250.00;
double balance = 300.00;   // 'balance' is already declared in this scope
```

A scope is a stretch of code between a pair of braces. You may reuse a name once the old block has
closed. See [Classes, methods, and objects](classes-methods-objects.md) for what lives inside one.

## The types you will use most

`int` holds whole numbers. `double` holds numbers with a decimal point. `boolean` holds `true` or
`false`. `String` holds text, and is not technically a primitive, but you will use it constantly.

```java
int deposits = 3;
double fee = 2.50;
boolean isOverdrawn = false;
String owner = "Ada";
```

`final` means the value can never change after the first assignment. Use it for anything that is a
fixed rule rather than a running total.

```java
final double MONTHLY_FEE = 2.50;
double total = MONTHLY_FEE * 4;   // 10.0
```

`static` means a field or method belongs to the class rather than to one object. Both `final` and
`static` are covered in [Classes, methods, and objects](classes-methods-objects.md).

## Arithmetic

`+` adds, `-` subtracts, `*` multiplies, `/` divides, and `%` gives the remainder after division.
Here is each one with real values:

```java
int a = 5;
int b = 2;

int result = a + b;   // 7
result = a - b;       // 3
result = a * b;       // 10
result = a / b;       // 2
result = a % b;       // 1
```

Dividing two `int`s throws away the decimal part. It does not round.

```java
int whole = 3 / 2;        // 1, not 2 and not 1.5
double exact = 3.0 / 2;   // 1.5
```

If at least one side of the `/` is a `double`, you get a `double` back. This is the single most
common surprise for people new to the language, so check it whenever a number comes out short.

A `double` is how you get a decimal out of any calculation:

```java
double width = 3.0;
double height = 2.0;
double area = width * height;   // 6.0
```

Order of operations is the same as in math class. Parentheses first, then multiplication and
division from left to right, then addition and subtraction from left to right. Java has no
exponent symbol, so use `Math.pow`:

```java
Math.pow(2, 10)   // 1024.0
```

## Unary operators

These act on one value. `++` adds one, `--` subtracts one, `!` flips a `boolean`, and a lone `-`
flips a number's sign.

```java
int count = 5;
count++;             // 6
count--;             // 5
int negative = -count;   // -5
boolean isReady = true;
boolean isNotReady = !isReady;   // false
```

`count++` and `++count` do the same thing to `count`, but they can hand back different values, so
they behave differently when the result is used. Inside a `for` loop's counter they are
interchangeable, and that is the only place you will see this.

## Compound assignment

These shorten "change the variable in place". Each one does the operation, then stores the result
back in the same variable.

```java
int a = 5;
a += 2;   // 7
a -= 2;   // 5
a *= 2;   // 10
a /= 2;   // 5
a %= 2;   // 1
```

---

*Previous: [Setup](../setup.md). Next: [Logic operators and strings](logic-operators.md)*
