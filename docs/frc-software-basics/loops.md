# Loops

*Audience: New programmers. Assumes you've read [Arrays and enums](arrays.md).*

The examples on this page are invented, not taken from this robot's code. For when a loop does and
does not belong on a robot, read [Applied to FRC](applied-to-frc.md).

## For loop

Use a `for` loop when you know how many times you want to run. The header has three parts, in
order: set up a counter, state the condition that keeps the loop running, and update the counter.

```java
for (int i = 0; i < 4; i++) {
    // runs with i = 0, then 1, then 2, then 3
}
```

The condition is checked before every pass, so `i < 4` stops the loop before `i` reaches 4.

## Enhanced for loop

The enhanced form walks over every value in an array or collection without an index. See
[Arrays and enums](arrays.md) for more.

```java
double[] readings = {1.5, 2.5, 3.0};

for (double reading : readings) {
    System.out.println(reading);
}
```

Use the enhanced form when you do not need the index, and the plain form when you do.

## While loop

A `while` loop runs as long as its condition stays `true`. Use it when you do not know the number
of passes up front.

```java
boolean sensorReady = false;
while (!sensorReady) {
    sensorReady = checkSensor();
}
```

The loop only ends if something inside it can change the condition. If `checkSensor()` always
returned `false`, this would run forever and the program would never get past it.

## Moving toward a target

A `while` loop is the natural shape for "keep going until you get there", and the condition has to
be re-checked on every pass so the loop stops exactly on the target.

```java
int position = 0;
int target = 10;

while (position != target) {
    if (position < target) {
        position++;
    } else {
        position--;
    }
}
// position is 10
```

This works only because the step is exactly 1. Make the step 3 and the loop overshoots, then
turns around, then overshoots again, and never ends:

```java
int position = 0;
int target = 10;

while (position != target) {
    if (position < target) {
        position += 3;
    } else {
        position -= 3;
    }
}
// position never reaches 10. It goes 0, 3, 6, 9, 12, then 9, then 12, forever.
```

When a step can overshoot, compare with `>=` or `<=` instead of `!=`, so the loop stops as soon as
it is close enough.

## Do-while loop

The body runs once before the condition is checked, so the loop always runs at least one time.

```java
int count = 0;
do {
    count++;
} while (count < 3);
// count is 3
```

Reach for this only when you specifically need that guarantee. A plain `for` loop is clearer in
most cases.

## Where you declare a variable decides how long it lives

A variable written inside a loop body is a brand new variable on every pass, and it is gone when
that pass ends. A variable written before the loop is created once and keeps its value across
passes.

```java
int total = 0;
for (int i = 0; i < 4; i++) {
    total += i;   // total goes 0, then 1, then 3, then 6
}
// total is 6
```

```java
for (int i = 0; i < 4; i++) {
    int doubled = i * 2;   // 0, then 2, then 4, then 6, discarded each time
}
```

Declare a variable before the loop when you need the result after the loop finishes. Declare it
inside when each pass is independent.

## Common loop errors

An infinite loop is one whose condition never turns `false`. The classic version is a counter that
moves the wrong way:

```java
// Never ends. i starts at 24 and only counts up, so i > 6 is true forever.
for (int i = 24; i > 6; i++) {
}
```

The other loop mistakes are worth knowing because the compiler catches all of them: forgetting
`int` in front of the counter, missing a semicolon in the header, and unmatched braces.

---

*Previous: [Arrays and enums](arrays.md). Next: [Classes, methods, and objects](classes-methods-objects.md)*
