# Autonomous Programming (Auton)

## What is Auton?

Autonomous mode (Auton) refers to the first 15 seconds of an FRC game during which the robot operates solely based on pre-programmed instructions, without human driver input.

## Why Have an Auton?

*   **Scoring**: Auton actions are typically worth more points than actions performed during Teleop (driver-controlled) period.
*   **Strategy**: Different autonomous strategies can earn significant points early in the match and can disrupt opponents' plans.

## How Does Auton Work?

We primarily use **PathPlanner Software** to design and implement our autonomous routines.

*   **Paths with "Event Markers"**: These paths are designed with specific points where commands are triggered.

## PathPlanner Basics

"Autons" is the catch-all term for robot movements in autonomous mode.

### Composition

An autonomous routine is composed of:
*   **Multiple Paths**: Sequences of robot movement.
*   **Commands**: These commands run in between path segments (e.g., `wait`, `score`).

### Paths

The basic form of moving the robot. Paths consist of:
*   **Waypoints**: Specific locations the robot should pass through.
*   **Event markers**: Triggers for commands to execute at certain points along the path.
*   **Starting States**: Initial conditions for the robot's pose and configuration.

### Auton Commands

Auton often has its own subset of commands tailored for autonomous actions. These commands are connected to fit the robot's needs in autonomous rather than teleop.

## Connecting Your Auton

Within the auton file:
*   **Connect Paths**: Define the sequence of paths the robot should follow.
*   **Use Wait Commands**: Incorporate pauses when necessary.
*   **Connecting to SmartDashboard**: Integrate with SmartDashboard for selection and monitoring.

## Alternative Usages

*   Other teams have used PathPlanner to automate controls in moving to specific game piece locations (e.g., the reef to score in a past season).
*   The Madtown Auto (from the 2024 season) previously used a camera to determine if a note was in the path before going toward it.
