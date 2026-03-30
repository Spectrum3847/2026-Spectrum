# Programming Tips and Best Practices

This section covers various tips and best practices that can help improve your Java and FRC programming experience.

## Java Clean Workspace

If you are encountering strange Java errors that you suspect are not genuine code issues, sometimes cleaning and reloading the Java Workspace can resolve them.

*   **How to Clean**: In VS Code, open the Command Palette (`Ctrl + p`) and type `>java Clean` or run `./gradlew clean` in the terminal to initiate the cleaning process. This rebuilds the Java language server's cache.

## Coordinate Systems

Understanding coordinate systems is fundamental in FRC, especially for robot motion and vision.

*   **Spectrum Coordinate System Poster**: Refer to this visual aid for our team's specific coordinate system conventions.
*   **Coordinate System Document**: A detailed document explaining the coordinate systems used in our robot code and how they relate to the field.

## `DoubleSupplier` vs. `double`

For most commands and trigger factories, using `DoubleSupplier` as arguments is preferred over a raw `double`.

*   **Dynamic Values**: `DoubleSupplier` allows values to be updated as they are running (e.g., fetching a sensor reading or a potentiometer value), which makes tuning easier and more flexible.
*   **Static Values**: A `double` provides a fixed, static value that won't change after the command is scheduled.

## Cached Values

For any sensor updates that have to go over the CAN bus (e.g., getting motor rotations, velocity, or CAN sensor distances), it's a best practice to cache these values.

*   **Efficiency**: Check the value once per periodic loop within a subsystem's `periodic()` method.
*   **Consistency**: Save this value to an object (e.g., a member variable) that can then be read by other methods throughout the robot's control loop. This prevents multiple, potentially expensive, CAN bus calls within a single loop cycle and ensures all parts of the code are working with the same, most recent, sensor data.

## Method Chaining

We utilize method chaining extensively in our codebase.

*   **Fluid API**: Methods in `Triggers` and `Commands` often return the `this` object (the instance itself).
*   **Conciseness**: This allows you to chain multiple method calls together in a single line, creating a more fluent and readable API.

    ```java
    // Example of method chaining
    myTrigger.whenTrue(myCommand.andThen(anotherCommand).withTimeout(5.0));
    ```

## Spectrum 2026 Code Goals

Our development goals for the 2026 season code include:

*   **Accessibility**: Make it easier for more people to work on our code.
*   **Modularity**: Organize code so small chunks can be worked on individually.
*   **Build Tools**: Better utilize build tools (Spotless and SpotBugs) to help reduce errors.
*   **Simulation**: Improve our use of WPILib Simulation and AdvantageScope.
*   **Merge Conflicts**: Reduce the amount of merge conflicts when committing (as they can be confusing and cause errors).
*   **Codebase Consolidation**: Reduce lines of code in `Robot.java` where possible, moving more logic to `SpectrumLib`.
*   **Robot Agnostic**: Allow a single codebase to run on multiple robots, even with very different configurations.
*   **Command Groups**: Reduce the number of large Command Groups, better using `Triggers` to represent state.
*   **CTRE & PathPlanner**: Integrate CTRE 2026 Swerve and PathPlanner (possibly Choreo too).
*   **LEDs**: Move to 2026 WPILib LED and `LEDBuffer`.
*   **Logging**: Improve logging (DogLog).

Feel free to open a pull request with any changes!
