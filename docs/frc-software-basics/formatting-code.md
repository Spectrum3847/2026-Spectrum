# Code formatting and comments

*Audience: New programmers. Assumes you've read [Classes, methods, and objects](classes-methods-objects.md).*

The examples on this page are invented, not taken from this robot's code.

## The shape of a Java file

A file is named after the class it holds. `BankAccount.java` holds `class BankAccount`, and you may
only have one `public` class per file.

In a plain Java program, execution starts at a method called `main`. Robot projects are different,
and [Applied to FRC](applied-to-frc.md) covers how those start.

Curly braces group lines of code into a block. A method body is everything between one pair, and
so is the body of an `if`. The compiler uses the braces, not the indentation, to work out what
belongs where, so indentation is for people.

```java
void greet() {
    System.out.println("hello");
}

int a = 5;
int b = 1;
if (a > b) {
    System.out.println("a is greater than b");
} else {
    System.out.println("a is not greater than b");
}
```

Every statement ends with a semicolon. Leave one off and it is a compile error. Java will not guess
where a statement ends.

You do not need to format code by hand. A tool does it for you on every build. For the rules this
project uses, and the reasoning behind them, see
[Code Style](../coding-conventions/code-style.md).

## Comments

A comment is a note for the reader. The compiler skips it entirely, so a comment can say anything
and change nothing about what the program does.

**A single line comment** starts with two slashes. Everything after them on that line is a comment.

```java
double interestRate = 0.02;   // two hundredths, written as a decimal
```

**A block comment** starts with a slash and a star and ends with a star and a slash. It can run
over several lines. You will not see many of these.

```java
/*
 * Everything between the markers is a comment,
 * including the line breaks.
 */
```

**A Javadoc comment** also uses two stars, `/**` and `*/`, and is the one that matters here. It
attaches to the class or method below it, and it is what other people and your editor read
instead of the code. Use it on anything public that someone else will call.

```java
/**
 * Moves money out of this account.
 *
 * @param amount how much to take out, in dollars
 * @return true if the withdrawal went through
 */
public boolean withdraw(double amount) {
    if (amount > balance) {
        return false;
    }
    balance -= amount;
    return true;
}
```

Three parts carry most of the meaning:

* The first line says what the method does, in one sentence.
* Each `@param` line names one parameter and says what it is.
* `@return` says what comes back.

Say what the code does not already say. A comment that repeats the line underneath it is noise,
and a comment that explains why something is written an unusual way saves the next person an
afternoon. There is more on this in
[Documentation and Comments](../coding-conventions/documentation-and-comments.md).

---

*Previous: [Classes, methods, and objects](classes-methods-objects.md). Next: [Applied to FRC](applied-to-frc.md)*
