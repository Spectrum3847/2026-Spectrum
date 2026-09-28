# Loops

*Audience: New programmers. Assumes you've read [Arrays and enums](arrays.md).*

A loop runs the same block of code more than once. You use a `for` loop when you know how many times, and a `while` loop when you do not.

Read [Applied to FRC](applied-to-frc.md) when you reach the `while` loop. On a robot, a `while` loop that waits for something is almost always the wrong tool, and knowing why is worth learning before you need it.

## The for loop

A `for` loop has three parts in the header, separated by semicolons, and they all matter.

```java
for (int i = 0; i < 3; i++) {
    System.out.println("i is " + i);
}
```

1. `int i = 0` runs once, before anything else. This is where the counter is born.
2. `i < 3` is checked before every pass. When it is false, the loop stops and the program carries on after the closing brace.
3. `i++` runs at the end of every pass. This is what changes the condition so the loop can end.

That means the body runs with `i` equal to 0, 1, and 2. The value 3 is never used inside the loop, because the check happens first. Getting that boundary right is the whole game with `for` loops.

`i` stops existing when the loop ends, because it was declared inside the header. See [Inside or outside the loop](#inside-or-outside-the-loop) below.

## The enhanced for loop

When you want to visit every element of an array or list and do not care what position you are at, the enhanced form is shorter and harder to get wrong.

```java
int[] scores = {88, 92, 79};
int total = 0;

for (int score : scores) {
    total += score;
}
```

The part before the colon is a variable for one element, and the part after is the thing being walked. The loop visits every element, in order, once each.

Use the enhanced form when you do not need the position. Use the plain form when you do, for example when you are comparing two arrays element by element.

```java
for (int i = 0; i < scores.length; i++) {
    System.out.println("student " + i + " scored " + scores[i]);
}
```

Prefer `scores.length` over a hardcoded number. If someone adds a fourth score later, the loop still does the right thing.

## The while loop

A `while` loop runs as long as its condition stays `true`. Use it when you cannot say up front how many passes you need.

```java
boolean ready = false;

while (!ready) {
    ready = checkIfReady();
}
```

Here is the risk. If the condition never becomes `false`, the loop never ends, the program stops responding, and you have to kill it. So be able to point at the line inside the loop that can change the condition, and be sure it eventually will. A `while` loop with no way out is the shape of every infinite loop bug you will ever write.

## The do-while loop

This one runs the body once, then checks the condition, so the body always executes at least one time.

```java
int result;

do {
    result = readSensor();
} while (result < 0);  // keep going until the reading is a real number
```

Reach for it when the first pass is not optional. It is rare, and when you are unsure, a plain `while` is the better default.

## Inside or outside the loop

Where you declare a variable decides whether it survives the loop. Declare it before the loop and it keeps its value across passes and still exists afterwards. Declare it inside and you get a brand new one each pass, and it is gone once the brace closes.

```java
int total = 0;
for (int i = 0; i < 4; i++) {
    total += i;
}
// total is 6 here

for (int i = 0; i < 4; i++) {
    int copy = i;  // a new copy each pass: 0, 1, 2, 3
}
// copy does not exist here
```

So: declare outside when you need the answer after the loop finishes, or when each pass has to build on the last one. Declare inside when each pass is independent and you do not care afterwards.

## Common loop errors

The classic infinite loop is a counter that moves the wrong way. The condition is true at the start, the counter never approaches it, and the program hangs.

```java
for (int i = 0; i > -5; i--) {  // true at the start, and the update walks away from it
    // this never ends
}
```

The same thing happens if you write `i < 5` but leave out `i++` entirely, because nothing about the loop ever changes.

One more, specific to loops over arrays: using `<=` instead of `<` on the index reads one past the end and throws `ArrayIndexOutOfBoundsException`. `i < scores.length` is correct, `i <= scores.length` is not.

The compiler catches the mechanical mistakes: a missing `int` before the counter, a missing semicolon in the header, or mismatched braces. It cannot catch a loop that runs the wrong number of times, so that part is on you to test.

---

*Previous: [Arrays and enums](arrays.md). Next: [Classes, methods, and objects](classes-methods-objects.md)*
