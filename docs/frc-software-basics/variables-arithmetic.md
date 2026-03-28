# Variables & Arithmetic

Java is statically typed and uses semicolons.

**Things that Work:**
```java
int a = 5;
a = 3;
```

**Things that Don't:**
```java
a = "Java"; // Type mismatch
a = 3    // Missing semicolon
String a = "Java"; // Already declared
int a = 3;   // Already declared
```

Python is dynamically typed and does not use semicolons.

**Things that Work:**
```python
a = 5
a = "Hello"
```

**Things that Don't Work:**
```python
a = 5; # Semicolon not used in Python
```

## Semicolons and Variable Types

### Java
*   Each program statement terminates with a semicolon.
*   Statically typed.
*   Java ignores whitespaces.

### Python
*   Does not use semicolons.
*   Dynamically typed.
*   Python relies on whitespaces for code structure.

## Java Primitive Types

*   `int`: integer numbers
*   `double`: decimal numbers
*   `boolean`: either `true` or `false`
*   `String`: represents text or sequences of characters. Enclosed in double quotes.
    *   *Disclaimer: not a primitive type but very common.*
*   `byte`: can represent a number, text, or other symbol.

**Other keywords we use:**
*   `implements`
*   `final`: means the variable's value cannot be changed.
*   `static`

## Arithmetic (Binary) Operators

"Binary" - these operators require two operands or values to function.

*   `+` **Addition**: The sum of two (or more) numbers.
*   `-` **Subtraction**: The difference between two numbers.
*   `*` **Multiplication**: The product of two (or more) numbers.
*   `/` **Division**: The quotient of the division of two numbers.
*   `%` **Modulus**: The remainder of the division of two numbers.

## Mathematical Precedence (PEMDAS)

1.  **P**arentheses
2.  **E**xponents (not an operator in Java, use the `Math` class)
3.  **M**ultiplication and **D**ivision
4.  **A**ddition and **S**ubtraction

## Arithmetic (Binary) Operator Examples

```java
int a = 5;
int b = 2;
int result;

result = a + b; // result equals 7
result = a - b; // result equals 3
result = a * b; // result equals 10
result = a / b; // result equals 2 (int division - normal division but the decimals are removed)
result = a % b; // result equals 1
```

**Truncation in Integer Division:**
When performing integer division, Java truncates the decimal part.
*   `int x = 3 / 2; // x will be 1 (truncates from 1.5)`
*   `int x = 1 / 2; // x will be 0 (truncates from 0.5)`

## Unary Operators

*   `++` **Increment Operator**: Increases the operand by 1. Can be placed before variables (`++a`) or after (`a++`).
*   `--` **Decrement Operator**: Decreases the operand by 1.
*   `!` **NOT Operator**: Reverses the state of the operand (e.g., `!false` = `true`).
*   `-` **Unary minus**: Multiplies the operand by -1.

## Unary Operator Examples

```java
int a = 5;
boolean c = true;
int result;

a++;
System.out.println(a); // prints 6

a--;
System.out.println(a); // prints 5

System.out.println(!c); // prints false

System.out.println(-a); // prints -5
```

## Compound Assignment Operators

Shortcuts for applying an arithmetic operation to a variable and assigning that new value to itself.

*   `+=`: `a = a + 2` is the same as `a += 2`
*   `-=`: `a = a - 2` is the same as `a -= 2`
*   `*=`: `a = a * 2` is the same as `a *= 2`
*   `/=`: `a = a / 2` is the same as `a /= 2`
*   `%=`: `a = a % 2` is the same as `a %= 2`

## Compound Assignment Examples

```java
int a = 5;

a += 2; // a equals 7
a -= 2; // a equals 5
a *= 2; // a equals 10
a /= 2; // a equals 5
a %= 2; // a equals 1
```

## Math Class

The `Math` class is a built-in, static, Java class that handles a plethora of mathematical operations.

*   `Math.sqrt(int or double)`: Returns the square root of the input as a `double`.
*   `Math.pow(base, exponent)`: Returns the exponentiation of a number as a `double`.
*   `Math.abs(int or double)`: Returns the absolute (positive) value of a number as the type received (`int` or `double`).
*   `Math.PI`: Returns the value of pi.

## Math Class Examples

```java
int a = 4;

System.out.println(Math.sqrt(a));   // prints 2.0
System.out.println(Math.pow(a, 2)); // prints 16.0
System.out.println(Math.abs(-a));   // prints 4
```
