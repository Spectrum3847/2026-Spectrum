# Simulation

Robot simulation is a crucial tool for developing and testing robot code without needing physical access to the robot.

## Simulation Library

Our simulation library allows for the creation of various virtual mechanisms and their integration into a side view of the robot.

*   **Mechanism Types**: Able to make `Roller`, `Linear`, and `Arm` simulations.
*   **Assembly**: These simulations can be attached together to mimic the movement of the real robot (e.g., rollers mounted to elevators, arms mounted to each other).
*   **Roller Sim**: Specifically displays the direction and relative velocity of the roller.

## Simulation Details (Based on 604's Sample Project)

Our simulation capabilities are inspired by Team 604's sample project, particularly their use of `PhotonVisionSim` for simulating AprilTags and cameras.

*   **Mechanism Drawing**: Creates `LinearSim`, `ArmSim`, and `RollerSim` classes that can be used to draw various mechanisms.
*   **Motion Combination**: Able to combine the motion of multiple mechanisms in a `Mechanism2D` (e.g., rollers mounted to elevators/arms, arms/elevators mounted to each other).
*   **Multiple Types**: Can draw multiple types of elevators, arms, rollers, etc.

## Benefits of Simulation

Simulation offers numerous advantages for FRC teams:

*   **Early Coding**: Allows programmers to start coding early in the season, even before the physical robot is complete.
*   **Accessibility**: Enables more team members to work on the robot's code concurrently.
*   **Logic Error Detection**: Helps catch logic errors in the code before they can affect the physical robot, preventing potential damage or wasted time.
*   **Convenience**: Provides a convenient way to test and debug code; however, it is not a replacement for testing on the actual robot.

*Note: This was highlighted during the 2024 December Open House.*

## Subsystem Simulation

Our codebase incorporates robust subsystem simulation capabilities, allowing for comprehensive testing and development in a virtual environment.

*   **`RobotSim.java`**: This class (located in `frc.robot`) orchestrates the overall robot simulation, integrating various simulated components.
*   **Season-Specific Simulation**: We leverage `IronMaple`'s season-specific simulation features, such as:
    *   `org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly`: Likely simulates game pieces ("fuel") for the REBUILT 2026 game.
    *   `org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt`: Provides a simulated field environment specific to the 2026 REBUILT game.
*   **Integration**: Subsystem simulations are integrated with the main `RobotSim` to provide realistic behavior for mechanisms like intakes, launchers, and turrets, interacting with the simulated game elements and field. This allows for early tuning and validation of complex robot logic.
