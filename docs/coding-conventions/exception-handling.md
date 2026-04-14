# Exception Handling

Proper exception handling is crucial for creating robust and reliable robot code. It prevents unexpected crashes and allows the robot to recover gracefully from errors.

## No Empty Catch Clauses

Never use empty catch clauses.

> "Anytime somebody has an empty catch clause they should have a creepy feeling. There are definitely times when it is actually the correct thing to do, but at least you have to think about it. In Java you can't escape the creepy feeling." — James Gosling

An empty catch block hides errors, making debugging extremely difficult. At a minimum, log the exception or print its stack trace.

## Don't Catch Generic Exceptions

Avoid catching `Exception` or `Throwable` generically, especially in application-level code.

*   **Danger**: Catching generic exceptions can inadvertently suppress critical errors, including runtime exceptions like `NullPointerException` or `ClassCastException`, which should typically cause program termination or be handled more specifically.
*   **Specificity**: Catch the most specific exception possible (e.g., `IOException`, `TimeoutException`) to handle only the errors you anticipate and can meaningfully recover from.

## Array Exceptions

Commonly, exceptions like `NullPointerException` or `ArrayIndexOutOfBoundsException` can cause the robot to break if not handled.

### Prevention Strategies

1.  **Conditional Checks**: Use `if` statements to check for potential array-related issues *before* accessing elements.
    *   **Example**: Check if an `index` is within bounds (`index >= 0 && index < array.length`) before accessing `array[index]`.
    *   **Example**: Check if an object is `null` (`if (myObject != null)`) before calling methods on it.

2.  **`try-catch` Blocks**: Use `try-catch` blocks as a fail-safe mechanism in robot code, particularly for operations that might fail unpredictably (e.g., sensor readings, network communication).

    ```java
    try {
        // Code that might throw an exception
        int value = mySensor.getValue();
        // ...
    } catch (SpecificException e) {
        // Handle the specific exception, e.g., log it and use a default value
        System.err.println("Error reading sensor: " + e.getMessage());
        // telemetry.log("SensorError", e.getMessage());
    } catch (AnotherSpecificException e) {
        // Handle other specific exceptions
    } catch (Exception e) { // Only catch generic Exception as a last resort, and always log it!
        System.err.println("An unexpected error occurred: " + e.getMessage());
        e.printStackTrace(); // Always print stack trace for generic exceptions
    }
    ```
