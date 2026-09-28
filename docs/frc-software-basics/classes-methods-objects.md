# Classes, methods, and objects

*Audience: New programmers. Assumes you've read [Loops](loops.md).*

A class is a blueprint that describes what something is and what it can do. An object is one real thing built from that blueprint. Here is a whole class. It is short on purpose, and most real classes are about this size.

```java
public class Rectangle {
    private static int countMade = 0;  // one number, shared by every Rectangle

    private final double width;
    private final double height;

    public Rectangle(double width, double height) {
        this.width = width;
        this.height = height;
        countMade++;
    }

    public static int countMade() {
        return countMade;
    }

    public double area() {
        return width * height;
    }
}
```

Creating an object from the class is `new`, and the values in the parentheses go to the constructor.

```java
Rectangle card = new Rectangle(3, 5);
card.area();  // 15.0
```

Every time you write `new Rectangle`, you get a separate object with its own `width` and `height`. Changing one does not touch the other. Objects of the same class share code, and each object has its own copy of the instance fields. A `static` field is the exception, because there is one copy per class rather than one per object, which is what `countMade` is.

## Methods

A method is a named block of code that the class can run. The constructor at the top of that class runs once, when the object is created. `area()` is a method you call on an object you already have.

Look at the declaration of `area()`. `public` decides who can call it, `double` is the type it hands back, `area` is the name, and the empty parentheses mean it takes no inputs. A method that returns nothing is written `void`, which means it cannot return a value. It may still have a bare `return;` to leave early.

Methods take inputs, and those are called parameters. A method can also return a value, and the return type says what kind.

```java
public double scaledBy(double factor) {
    return width * height * factor;
}
```

Gotcha: a method that says it returns something must return something on every path. If the compiler thinks you can fall off the end without a `return`, it refuses to compile.

## Access modifiers

Every field and method has an access modifier, and picking the narrowest one that works is the habit to build.

- `private` means only code inside this class can see it. This is the default you want.
- No modifier means only code in the same package can see it.
- `public` means anything can see it. Reserve it for the calls other classes need to make.

In the `Rectangle` above, `width` and `height` are `private` on purpose. Nothing outside the class can overwrite them or read them directly, so the only way to get an area is to ask for it through a method. That is the whole idea: keep the data private, expose a small set of `public` methods, and other code cannot put the object into a state you did not allow.

```java
Rectangle card = new Rectangle(3, 5);
card.width = 99;  // does not compile, width is private
card.area();      // still 15.0, so nothing can sneak in and change it
```

## static

A `static` field or method belongs to the class, not to any one object. You call it with the class name.

```java
Rectangle.countMade();  // a static method on the class, same count for every Rectangle
```

Compare that with `card.area()`, which belongs to one specific object and is called on it.

```java
Rectangle a = new Rectangle(2, 2);
Rectangle b = new Rectangle(4, 4);

a.area();  // 4.0, a's data
b.area();  // 16.0, b's data
```

The practical difference shows up in state. A `static` field is one shared value that every object of that class sees and can change. An ordinary field is one value per object. So a `static` field suits a count or a shared setting. It does not suit anything that describes a particular object. In the `Rectangle` example above, `width` and `height` describe one rectangle, so each object has its own, while `countMade` is a tally of every rectangle ever made, so all of them share the one number.

`final` on a field means it is set once and never again, which is a good default for anything that describes the object. `countMade` in the example above still changes, so it is not `final`.

## Constructors

A constructor runs once, when you write `new`. Its name matches the class name, it has no return type, and it is where an object gets its initial state. Writing `new Rectangle(3, 5)` calls the constructor with `width` set to 3 and `height` set to 5, so that is the first thing that happens to a new object, and the only way to make one.

```java
public Point(int x, int y) {
    this.x = x;
    this.y = y;
}
```

`this.x = x` is worth a second look. The left side is the object's field, because of `this.`. The right side is the parameter, which is just a local name. The parameters are named the same as the fields on purpose, so `this` is what tells them apart.

If a class extends another, call `super()` first to run the parent's constructor, then do this class's own setup. `super()` goes above any other statement in the constructor body.

## Lambdas and method references

Java can pass a function around as a value, the same way it passes a number. The way you write one inline is a lambda: parentheses for the inputs, an arrow, then the body.

```java
Runnable task = () -> System.out.println("done");
task.run();  // prints done
```

A method reference is a shorter form for the common case where the function you are passing is just one existing method with no extra work in the body. `this::methodName` means "call my method", and it is equivalent to the lambda `() -> methodName()`.

```java
Runnable first = () -> System.out.println("done");
Runnable second = this::printDone;  // same thing, if printDone takes nothing
```

Use a lambda when the body is a couple of lines, and a method reference when it is one call. Do not invent a named method just to have something to reference.

WPILib APIs lean on this heavily. Any method that takes a supplier, so that it can ask for a value later instead of taking one now, takes a lambda or a method reference. When you meet a parameter whose name ends in `Supplier`, that is what you are looking at. See [Programming tips](../other-guides/tips.md#doublesupplier-vs-double).

Once you are comfortable with classes, [Class Generation](../coding-conventions/class-generation.md) covers how we lay out the classes in this codebase.

---

*Previous: [Loops](loops.md). Next: [Formatting code and comments](formatting-code.md)*
