# Applied to FRC (First Robotics Competition)

This section highlights common Java constructs and how they are typically used (or not used) within the context of FRC programming.

## What's Used and What Isn't?

### Example: Command Chaining

We often use WPILib's command-based programming structure, which allows for concise command scheduling.

**We write:**
```java
stationIntaking.whileTrue(coral.toggleToTrue(), algae.setFalse());
```

**Instead of:**
```java
while(stationIntaking == true) {
    coral = true;
    algae = false;
}
```
This demonstrates how higher-level APIs abstract away traditional loop structures for FRC-specific tasks.

### Frequently Used

*   **If and else statements**: For conditional logic (e.g., `if (buttonPressed) { ... } else { ... }`).
*   **Logic operators and arithmetic**: Essential for all calculations and decision-making.
*   **Classes, objects, etc.**: The foundation of our object-oriented robot code, with subsystems being prime examples of classes.

### Moderately Used

*   **Enums**: Useful for defining states (e.g., robot states, mechanism positions).
*   **Direct for loops**: Can be used for specific iterative tasks, but often abstracted by command-based programming or stream APIs.

### Barely Used

*   **Arrays**: While fundamental, complex data structures in FRC often leverage more dynamic collections or specialized WPILib constructs.
*   **While loops**: Less common for continuous robot control due to the event-driven nature of FRC's command-based framework; however, they can be used for specific blocking operations or custom sensor polling where appropriate.
