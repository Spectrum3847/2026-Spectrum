# Logic-Based Operators

## Comparison Operators

*   `>`, `>=`: greater than or greater than or equal to
*   `<`, `<=`: less than or less than or equal to
*   `!=`: not equal to
*   `==`: equals

**Examples:**
*   `5 > 4` = `true`
*   `5 >= 6` = `false`
*   `5 < 6` = `true`
*   `5 <= 4` = `false`
*   `5 != 4` = `true`
*   `5 != 5` = `false`
*   `5 == 5` = `true`
*   `5 == 4` = `false`

## Logical Operators

*   `&&` **AND**: if both sides of the operator are true, will return `true`.
*   `||` **OR**: if either side of the operator is true, will return `true`.

**Examples:**
*   `((5 == 5) && (4 == 4))` = `true`
*   `((5 == 5) || (5 == 4))` = `true`
*   `((5 == 4) && (5 == 5))` = `false`
*   `((5 == 4) || (5 == 3))` = `false`

## Conditional Statements

*   **One-way selection**: If-statements
*   **Two-way selection**: If-else statements
*   **Multi-way selection**: Else-if statements, Switch statements

### One-Way Selection (If-statements)

```java
public static void main(String[] args) {
    boolean name = true;
    if (name){
        System.out.println("J.R.R.");
    }
    System.out.println("Tolkien");
}
// Output:
// J.R.R.
// Tolkien
```

### Two-Way Selection (If-else statements)

```java
public static void main(String[] args) {
    boolean a = false;
    if (a) {
        System.out.println("Never ");
    } else  {
        System.out.println("gonna give");
    }
    System.out.println(" you up");
}
// Output:
// gonna give
// you up
```

### Multi-Way Selection (Else-if statements)

```java
public static void main(String[] args) {
    boolean a = false;
    boolean b = true;
    if (a) {
        System.out.println("Never ");
    } else if (b) {
        System.out.println("gonna give");
    } else {
        System.out.println(" you up");
    }
}
// Output:
// gonna give
```

## Strings

### Initialization

```java
String s = "Hello World";
```

### String Operations

Assume `String s = "Hello ";` and `String s1 = "Hello World";` for examples below.

*   **Concatenation**:
    ```java
    String s = "Hello ";
    String s1 = s + "World";
    System.out.println(s1); // This would print "Hello World"
    ```

*   **Length**: Returns the number of characters in the string.
    ```java
    String str = "Java";
    System.out.println(str.length()); // This would print 4
    ```

*   **charAt(index)**: Returns the character at the specified index.
    ```java
    String str = "Spectrum";
    System.out.println(str.charAt(1)); // This would print 'p'
    ```

*   **substring(beginIndex, endIndex)**: Returns a new string that is a substring of this string. The substring begins at the specified `beginIndex` and extends to the character at `endIndex - 1`.
    ```java
    String str = "Hello World";
    System.out.println(str.substring(1, 4)); // This would print 'ell'
    ```

*   **contains(CharSequence s)**: Checks to see if a given string can be found within the original string.
    ```java
    String str = "Hello World";
    boolean b = str.contains("he"); // Case-sensitive, so this will be false for "Hello World"
    System.out.println(b); // Prints false
    // To make it case-insensitive, convert both to lower case:
    // boolean b = str.toLowerCase().contains("he".toLowerCase());
    ```

*   **equals(Object anotherString)**: Checks to see if two strings have equal values.
    ```java
    String s1 = "Hello World";
    String s2 = "Hello World";
    System.out.println(s1.equals(s2)); // Prints true
    ```

*   **toUpperCase()** and **toLowerCase()**: Returns a new string with all characters converted to uppercase or lowercase.
    ```java
    String str = "Hello World";
    String sUpper = str.toUpperCase();
    System.out.println(sUpper); // Prints 'HELLO WORLD'
    ```
