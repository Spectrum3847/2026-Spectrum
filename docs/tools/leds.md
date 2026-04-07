# LEDs

## `SpectrumLEDs.java` in `SpectrumLib`

The `SpectrumLEDs.java` class, located in our `SpectrumLib`, is where we implement and manage new LED animations for the robot.

*   **Subsystem**: It functions as a subsystem and uses an inner `SpectrumLEDs.Config` class for its configuration.
*   **Multiple Instances**: It allows for the creation of multiple instances, each with an `LEDBufferView`. This enables commanding specific parts of an LED strip independently.

## Default Command

The LED subsystem supports multiple default commands.

*   A trigger can be set for when the DefaultCommand is running.
*   Other commands can then be initiated based on the robot's mode (e.g., Teleop, Disabled, Autonomous).

## Creating New LED Commands

Creating new LED commands typically involves defining specific LED patterns or animations and then creating commands to activate them. Since `SpectrumLEDs.java` is our custom library, new animations would be added there.

1.  **Define LED Patterns/Animations**: In `SpectrumLEDs.java`, you would add new methods or classes that define the visual behavior of the LEDs (e.g., `blink(color)`, `rainbowEffect()`, `solid(color)`). These patterns often manipulate an `LEDBuffer` or `LEDBufferView`.
2.  **Create Commands**: Create `InstantCommand` or `RunCommand` wrappers that call these LED pattern methods.
    ```java
    // Example: Command to set LEDs to solid green
    public static Command setLedsGreen() {
        return Commands.instant(() -> Robot.leds.setSolidColor(Color.kGreen));
    }

    // Example: Command to start a rainbow effect
    public static Command startRainbowEffect() {
        return Commands.runOnce(() -> Robot.leds.startRainbow());
    }
    ```
3.  **Bind to Triggers**: These commands can then be bound to various triggers (buttons, robot states, sensor inputs) in `RobotStates.java` or directly within a subsystem's `setupStates()` method.
    ```java
    // Example: Pilot's A button sets LEDs to green
    Robot.pilot.getGamepad().a().onTrue(setLedsGreen());
    ```
This approach allows for modular and testable control over the robot's lighting.
