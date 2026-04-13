# Arrays

## What are Arrays?

Arrays are lists/containers that can store multiple elements of the same data type.

*   Can store variables, integers, bytes, booleans, etc.
*   Also can store objects (covered in the next section).
*   Limited only by the initialized amount in the array and its data type.

```java
// Main.java
public static void main(String [] args) {
    int [] x = {3, 8, 4, 7};
    System.out.print(x[0]);
    System.out.print(x[1]);
    System.out.print(x[2]);
    System.out.print(x[3]);
}

// Output:
// 3847
```

## Initializing Arrays

Arrays are useful in storing multiple variables at once.

*   The array can be created and initialized with its length.
*   They can only store the same initialized data type within each array.
*   There are no arrays with "negative" length/size.
*   Each stored value is at an "index" value.
*   The first index value—or the start of the list—is at "0".

```java
// Main.java
public static void main(String [] args) {
    int [] x;

    // initializes the array to length "4"
    x = new int[4];

    // stores the value 4 at index "0"
    x[0] = 4;
}
```

## Enums

Enums (enumerations) are used to define a fixed number of named constants.

*   Makes code clearer.
*   Useful when you need to access one variable out of a set.
*   Good usage for switch cases.

```java
public class EnumExample {
    enum Day {
        MONDAY, TUESDAY, WEDNESDAY,
        THURSDAY, FRIDAY, SATURDAY,
        SUNDAY
    }
    public static void main(String[] args) {
        Day today = Day.MONDAY;
        switch (today) {
            case MONDAY:
                System.out.println("Start of the week!");
                break;
            case SUNDAY:
                System.out.println("It's the weekend!");
                break;
            default:
                System.out.println("A regular weekday.");
        }
    }
}
```

## Array Specific Errors

*   `ArrayIndexOutOfBoundsException`: Occurs when you try to access an array element using an index that is outside the valid range (e.g., negative index or an index greater than or equal to the array size).
*   `NullPointerException`: Occurs when you try to access members of an object that is `null`. If an array of objects is declared but not initialized, its elements will be `null` by default.
