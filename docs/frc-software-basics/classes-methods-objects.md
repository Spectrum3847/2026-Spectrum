# Classes, methods, and objects

*Audience: New programmers. Assumes you've read [Loops](loops.md).*

The examples on this page are invented, not taken from this robot's code. A class is a blueprint,
and an object is one thing built from that blueprint. You write the blueprint once, then make as
many objects from it as you need.

## A class and an object

```java
public class BankAccount {
    private final String owner;
    private double balance;

    public BankAccount(String owner) {
        this.owner = owner;
        this.balance = 0.0;
    }

    public void deposit(double amount) {
        balance += amount;
    }

    public boolean withdraw(double amount) {
        if (amount > balance) {
            return false;
        }
        balance -= amount;
        return true;
    }

    public double getBalance() {
        return balance;
    }
}
```

`new BankAccount("Ada")` builds one object from that blueprint. The two words it holds, the owner
and the balance, are called fields. The things it can do, `deposit`, `withdraw` and `getBalance`,
are called methods.

```java
BankAccount ada = new BankAccount("Ada");

ada.deposit(100.00);
ada.deposit(50.00);
ada.getBalance();      // 150.0
ada.withdraw(200.00);  // false, not enough money
ada.getBalance();      // 150.0, unchanged
ada.withdraw(50.00);   // true
ada.getBalance();      // 100.0
```

Two objects built from the same class keep separate values. A second `BankAccount` starts at zero
however much the first one holds.

## Methods

Calling a method means writing the object, a dot, the method name, and parentheses. The parentheses
are required even when the method takes nothing, so `ada.getBalance()` and never `ada.getBalance`.

A method that does nothing back says `void` and has no `return` line. A method that hands back a
value says what type that value is, and uses `return` to hand it over. In the class above,
`deposit` returns nothing (`void`), `withdraw` returns a `boolean`, and `getBalance` returns a
`double`.

## Where a variable is visible

A variable or method can be marked so that other code is not allowed to touch it directly.

* `private` means only code inside this class can see it.
* no keyword at all means only code in the same package can see it.
* `public` means anything can.

The usual arrangement is to keep fields `private` and hand out values through a getter method, as
`getBalance()` does. Then you can change how a value is stored, or check it before returning it,
without breaking anything that used the class.

## Static and not static

A `static` field or method belongs to the class itself, so there is one copy of it shared by every
object. Everything above is not static, which means each `BankAccount` object has its own copy and
you reach it through an object.

```java
public class BankAccount {
    private static int accountsCreated = 0;

    private final String owner;
    private double balance;

    public BankAccount(String owner) {
        this.owner = owner;
        this.balance = 0.0;
        accountsCreated++;
    }

    public double getBalance() {
        return balance;
    }

    public static int getAccountsCreated() {
        return accountsCreated;
    }
}
```

```java
BankAccount ada = new BankAccount("Ada");
BankAccount alan = new BankAccount("Alan");

ada.getBalance();                  // 0.0
BankAccount.getAccountsCreated();  // 2, one counter for the whole class
```

Notice the difference in how they are called. `ada.getBalance()` goes through an object, because
the balance belongs to that object. `BankAccount.getAccountsCreated()` names the class, because
the counter belongs to the class. Java will let you write `ada.getAccountsCreated()`, but it reads
as a mistake, so write it the clear way.

## Constructors

The constructor runs once, when the object is created, and sets up its starting values. Its name is
the class name, it has no return type, and its parameter list is what `new` matches against.

`new BankAccount("Ada")` and `new BankAccount()` are only both allowed if you wrote two
constructors. Writing no constructor at all is fine too, and Java gives you one that takes nothing
and leaves every field at its default.

A class can extend another class to add to it. `super(...)` calls the parent constructor, and it
has to be the first line:

```java
public class SavingsAccount extends BankAccount {
    private final double rate;

    public SavingsAccount(String owner, double rate) {
        super(owner);
        this.rate = rate;
    }
}
```

## Lambdas and method references

Sometimes you want to hand a piece of code to a method as if it were a value, so the method can run
it later. The shortest form is a lambda, which is a chunk of code in parentheses with an arrow
before the body.

```java
import java.util.function.DoubleSupplier;

Portfolio p = new Portfolio();

DoubleSupplier a = () -> p.getCash();
```

`p::getCash` is a method reference, and it means exactly the same thing as the lambda above. Use
whichever is easier to read; the method reference is shorter when the code is just one call.

```java
import java.util.function.DoubleSupplier;

class Portfolio {
    double cash = 100.0;

    double getCash() {
        return cash;
    }

    void deposit(double amount) {
        cash += amount;
    }
}

Portfolio p = new Portfolio();

DoubleSupplier a = () -> p.getCash();
DoubleSupplier b = p::getCash;

System.out.println(a.getAsDouble());   // 100.0
p.deposit(50.0);
System.out.println(b.getAsDouble());   // 150.0
```

Both suppliers ask the portfolio when they are called, so both see 150.0 after the deposit. If you
had read the value into a `double` first and passed that, it would stay 100.0 forever. This is why
code that wants a live number asks for a supplier instead. There is more on that in
[Programming Tips](../other-guides/tips.md#doublesupplier-vs-double).

---

*Previous: [Loops](loops.md). Next: [Code formatting and comments](formatting-code.md)*
