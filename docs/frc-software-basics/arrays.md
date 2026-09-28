# Arrays and enums

*Audience: New programmers. Assumes you've read [Logic operators and strings](logic-operators.md).*

An array holds a fixed number of values, all of the same type, and you reach them by position. Counting starts at zero, so the first element is at index 0.

```java
int[] scores = {88, 92, 79};

scores[0]  // 88
scores[2]  // 79
scores.length  // 3
```

The size never changes. Once you build an array of three, it holds exactly three values for as long as it exists. That is the trade: arrays are fast and simple, but you must know the count up front. When you do not, use an `ArrayList`, which grows and shrinks as you add and remove.

## Two ways to build one

The version above puts the values in with the declaration. The other way is to create an empty array of a given size and fill it in afterwards. Every element starts at zero, so an empty array of `double` holds four `0.0` values until you write to them.

```java
double[] readings = new double[4];  // 0.0, 0.0, 0.0, 0.0

readings[0] = 1.5;
readings[1] = 2.3;
```

An array can hold objects too, not just numbers. The variable is declared as an array, and the elements are objects. Here `Point` is a made-up class with an `x` and a `y` field, so each element is one location.

```java
Point[] corners = {new Point(0, 0), new Point(10, 0), new Point(10, 10)};
```

Because the elements are objects, you can loop over the array and call methods on each one. See [Loops](loops.md) for the two loop forms.

## Enums

An enum is a type made of a fixed set of named values. Use one whenever a thing has a small, known list of modes, instead of a magic number or a magic string.

```java
public enum Shape {
    CIRCLE,
    SQUARE,
    TRIANGLE
}
```

A variable of that type holds exactly one of the three values, and nothing else compiles.

```java
Shape current = Shape.CIRCLE;
```

The payoff is that the compiler catches your typos, and a `switch` can require that you handle every case. With a `switch` expression like this, leaving out a case is a compile error, so a new mode cannot be added and quietly ignored.

```java
int sides = switch (current) {
    case CIRCLE -> 0;
    case SQUARE -> 4;
    case TRIANGLE -> 3;
};
```

## Two errors you will hit

**ArrayIndexOutOfBoundsException.** You asked for an index that does not exist. An array of three is valid at 0, 1, and 2 only. Index 3 or index -1 throws, and it throws while the program is running, not while it compiles.

```java
int[] scores = {88, 92, 79};
scores[3] = 100;  // throws, there is no fourth element
```

**NullPointerException.** You have a variable that points at nothing, and you used it anyway. This most often comes from an array of objects where you never filled in every slot. An empty slot holds `null`, and calling a method on `null` throws immediately.

```java
Point[] corners = new Point[3];  // all three are null
corners[0] = new Point(1, 1);
corners[0].x;      // fine
corners[1].x;      // throws, that slot was never filled in
```

The habit that prevents it: either fill every slot at creation, or check for `null` before you use it.

---

*Previous: [Logic operators and strings](logic-operators.md). Next: [Loops](loops.md)*
