# Loops

## What are Loops?

Loops iterate through a set number of times or items. They can be used to repeatedly perform a task or calculate values.

## Types of Loops

*   `while-loop`
*   `for-loop` (including enhanced for-loop)
*   `do-while loop`

## While Loop

A `while` loop repeatedly executes a block of code as long as its condition remains `true`.

*   Used mainly when the number of iterations is not defined beforehand.
*   Example: a robot iterating to a non-defined target position.

```java
boolean targetValue = false;
// Loop continues as long as targetValue is false
while (!targetValue) {
    boolean foundTarget = checkSensorForTarget(); // Assume this method checks for the target
    // Code within loop
    if (foundTarget) {
        // If target is found, set targetValue to true to stop the loop
        targetValue = foundTarget;
    }
}
```

## For Loop

A `for` loop is typically used when the number of iterations is known.

```java
for (int i = 6; i < 24; i++) {
    System.out.println(i);
}
```

**Two variations:**
*   **Standard for loop**: `for(int i=0; i < maxIndex; i++)`
*   **Enhanced for loop**: `for(index : (given array))`

## Enhanced For Loop

Used specifically for iterating through all entities within an array or collection.

**General Makeup:**
*   `Datatype` of elements in the array
*   `index/variable` to hold each element during iteration
*   `Array` or collection being iterated

```java
int [] variables = {8, 5, 1, 5};
for (int variable : variables) {
    // Will print out each int in the array
    System.out.print(variable);
}

// Output:
// 8515
```

## Inside vs. Outside the Loop

Variables behave differently depending on whether they are initialized outside versus inside the loop.

```java
// Variable 'x' initialized outside the loop
int x = 0;
System.out.println("Outside the loop");
for(int i = 0; i < 4; i++) {
    x += i;
    // x values: 0 (0+0), 1 (0+1), 3 (1+2), 6 (3+3)
}
System.out.println("Final x: " + x); // prints 6

System.out.println("Inside the Loop");
for(int i = 0; i < 4; i++) {
    // Variable 'y' initialized inside the loop (new 'y' each iteration)
    int y = i;
    System.out.println(y);
    // y values: 0, 1, 2, 3
}
```

## Do-While Loop

Similar to a `while` loop, but guarantees that the loop body executes at least once before the condition is checked.

*   Only used in situations where one singular repetition at the start is necessary to calculate or set a value.

```java
int targetPos = 0;
do {
    targetPos = calculatePos(); // Assume this method calculates a position
} while (targetPos != 0);
// The loop will run at least once, then continue as long as targetPos is not 0.
```

*Note: `do-while` loops are less commonly used compared to `for` and `while` loops, but it's good to be aware of their existence and purpose.*

## Loop Errors

Errors occur in loops when either:

*   **Faulty logic in the loop heading**: For instance, the loop never terminates (infinite loop).
    ```java
    // Infinite loop example: condition 'i > 6' will always be true if i starts at 24 and increments
    // for (i = 24; i > 6; i++) {
    //    System.out.println(i);
    // }
    ```
*   **Syntax errors**:
    *   Not matching curly braces.
    *   Forgetting to define the variable you are looping through.
    *   Missing semicolons in `for` loop headers.

    ```java
    // Example of syntax errors:
    // for (i = 6; i < 24; i++) { // 'i' not defined
    //    System.out.println(i);
    // }

    // for (int i = 6; i < 24; i++) {
    //    System.out.println(i) // Missing semicolon
    // }
    ```
