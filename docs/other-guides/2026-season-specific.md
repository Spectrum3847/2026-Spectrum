# 2026 Season Specific Documentation: REBUILT

This section provides documentation specifically relevant to the 2026 FRC game, **REBUILT**, and our robot's architecture and control schemes for this season.

## 2026 FRC Code Structure

### Subsystems

Subsystems represent any part of the robot with an output.

*   **Examples**: Mechanisms (FuelIntake, IndexerBed, Launcher), swerve drive, LEDs, controller rumble, Vision.
*   **Mechanisms**: These are subsystems that typically involve a motor (e.g., TalonFX, Kraken). Our `Mechanism` class wraps many motor-related methods for easier setup.
*   **Swerve**: A specialized drivetrain using Krakens. It handles movement and orientation as part of the overall drivetrain, distinct from game-specific mechanisms. It is fundamental for navigation and offensive/defensive play in REBUILT.
    *   Examples: `swerve`, `fuelIntake`, `indexerBed`, `indexerTower`, `intakeExtension`, `launcher`, `turretRotationalPivot`, `leds`, `operator`, `pilot`, `vision`.
*   **Structure**: Subsystems are usually organized into a folder containing 2-3 classes:
    *   **Main Class**: The primary class defining the subsystem's functionality.
    *   **Config Class**: Used to set up the subsystem and allow for configuration adjustments between different robots running our codebase.
    *   **States Class**: Contains commands that can be run to perform actions, often bound to triggers, allowing for complex behaviors.

### Configurations for Multiple Robots (XM, PM, FM)

Our codebase supports running on different physical robot setups for the 2026 season through distinct configuration classes. This allows us to adapt to variations in hardware or testing environments.

*   **`Robot.Config`**: The base class for all robot configurations, defining common parameters and subsystem configurations.
*   **`Rio.id`**: The robot's unique ID (`Rio.id`) determines which specific configuration (e.g., `XM2026`, `PM2026`, `FM2026`) is loaded at startup.
*   **Configuration Classes**:
    *   **`XM2026` (Experimental Machine)**: Used for experimental robot setups. It may have specific encoder offsets or mechanism attachment states tailored for testing new designs or components.
    *   **`PM2026` (Practice Machine)**: Configured for practice robots, often matching the competition robot as closely as possible but potentially with minor adjustments for wear and tear.
    *   **`FM2026` (Field Machine/Competition Machine)**: The configuration intended for the competition robot, with precise calibration values.
*   **`setAttached(boolean attached)`**: Each configuration can specify whether a particular mechanism (subsystem) is physically present and operational on that robot. This prevents errors and optimizes resource usage by only initializing active mechanisms.

### States/Triggers

#### Commands

Commands represent atomic actions the robot can take.

*   **Execution**: Commands run when scheduled, continuing until they are interrupted or their end condition is met. They encapsulate specific robot behaviors.
*   **Examples**: `swerve.drive()`, `fuelIntake.intake()`, `launcher.shoot()`, `turret.aim()`, `leds.blink()`.

#### Triggers

Triggers are conditions that, when met, can initiate commands or state changes.

*   **Examples**: A button press on a gamepad, a sensor detecting a game piece, the robot reaching a specific field position, or a timer expiring.
*   **Boolean Value**: All triggers evaluate to a `true` or `false` condition.
*   **Command Binding**: You can schedule commands based on the condition of a trigger (`whileTrue`, `onTrue`, `whileFalse`, `onFalse`).
    *   `pilot.X` (true if button X is pressed)
    *   `indexerBed.hasFuel()` (true if fuel is detected in the indexer bed).
*   **`SpectrumState`**: A special trigger that holds its own boolean value, commanded true or false. Used for conditions not easily represented by hardware (buttons, encoders) but derived from complex logic.
*   **Placement**: Triggers should normally be created as a `public static final` variable in the `SubsystemStates` file or the `RobotStates` file for consistent access.
*   **Logging**: Each command sent by a trigger should be logged (e.g., using `Telemetry.log(Command)` in the `states.java` file's log method) for debugging and analysis.

#### Subsystem State Conventions

*   **Default Command**: Set up a default command for each subsystem (e.g., a `stop()` or `hold()` method) to ensure safe and predictable behavior when no other command is active.
*   **`setupStates()`**: This method (found in `Robot.java` and `RobotStates.java`) is where triggers are bound to commands, defining the robot's reactive behaviors.
*   **`Coordinator.java`**: The `Coordinator` class plays a central role in managing high-level robot states by calling specific actions on multiple subsystems simultaneously based on a defined `State` enum. This simplifies complex inter-subsystem behaviors.
    *   **Example from `Coordinator`**: In an `INTAKE_FUEL` state, the `FuelIntakeStates` might be set to `intakeFuel()`, `IntakeExtensionStates` to `fullExtend()`, while `LauncherStates` remain `idlePrep()`.

### Mechanism

Our `Mechanism` class serves as a robust wrapper for motor controllers (e.g., Phoenix 6 TalonFX, Kraken).

*   **Wrapper**: Simplifies interaction with motor controllers.
*   **Config File**: Allows for easy configuration of the motor's behavior and parameters.
*   **Followers**: Supports unlimited permanent follower motors for synchronized movement.
*   **Configurable per Robot**: Configuration can be adjusted for each robot type loaded.
*   **Constructor Injection**: Configuration (`Config`) is passed into the constructor, allowing dynamic setup after identifying the robot type.
*   **Default Command Generators**: Provides utilities for generating commands for various operational modes (e.g., `MotionMagicTCFOC`, `VelocityFOC`).
*   **Trigger Generators**: Generates triggers based on motor position and velocity setpoints (`At`, `Above`, `Below` position and velocity), useful for autonomous and closed-loop control.

### Pose Estimation for REBUILT

*   **Integration with Swerve and Vision**: Pose estimation combines data from the Swerve drivetrain's odometry and the Vision system's AprilTag detections to provide a highly accurate estimate of the robot's position and orientation on the REBUILT field.

## 2026 Controls Meeting

### Code Plan

*   **Mechanism Groups**: Code is logically organized into mechanism groups (e.g., Drivetrain covers swerve, vision, auton paths; Manipulator covers FuelIntake, Launcher, Indexer).
*   **Collaboration**: Different team members can specialize and contribute to different areas concurrently.
*   **Simulation First**: The majority of code development and testing should occur in simulation using WPILib Simulation and AdvantageScope. This catches logical errors early and prevents damage to physical hardware.
*   **Robot Time**: Time on the physical robot should be primarily reserved for tuning, calibration, and final verification, not for developing new features.
*   **Feature Branches**: Use Git feature branches for each new feature or significant addition.
    *   **Workflow**:
        1.  Develop and test in simulation on your feature branch.
        2.  Merge `main` into your feature branch and re-test thoroughly in simulation to catch integration issues.
        3.  Test on the physical robot only if the changes involve potential hardware interactions or require real-world feedback.
        4.  Create a pull request for code review by a programming lead before merging into `main`.

### Controls Areas for REBUILT

*   **Swerve Drive**: Implementation and control of the swerve modules for advanced robot movement and field positioning in REBUILT.
*   **Robot Mechanisms**: `FuelIntake`, `IndexerBed`, `IndexerTower`, `IntakeExtension`, `Launcher`, `TurretRotationalPivot` for interacting with game pieces.
*   **Vision**: `Limelight`, object recognition (`PhotonVision` on Orange Pi), and `QuestNav` for targeting and localization using AprilTags and game pieces on the REBUILT field.
*   **Driver Controls**: Implementation of the `Operator` and `Pilot` gamepads for intuitive control of robot movement and mechanisms.
*   **Simulation**: Integration with WPILib Simulation and `IronMaple` for virtual testing.
*   **Logging and Data Analysis**: `DogLog` and `AdvantageScope` for capturing, reviewing, and analyzing robot performance data.
*   **LEDs**: `CANdle` LEDs for status indication and visual feedback.
*   **Dashboard and Feedback**: `Elastic Dashboard` for real-time telemetry and control.

## 2026 Robot States (Game REBUILT)

These are high-level states the robot can be in for the 2026 game, **REBUILT**, reflecting strategic and operational modes.

*   **`IDLE`**: Robot is ready but not actively performing a task. Subsystems are in a neutral or home position.
*   **`INTAKE_FUEL`**: Actively collecting game pieces (fuel). This involves extending the intake, running the intake motor, and potentially indexing.
*   **`TURRET_TRACK`**: The turret is tracking a target (e.g., AprilTag, goal) but not yet launching. Intake and indexing may be neutral or conditionally extended.
*   **`TURRET_TRACK_WITH_LAUNCH`**: Turret is tracking, and the fuel intake/indexer/launcher are actively preparing or executing a launch sequence.
*   **`UNJAM`**: Recovery state for clearing jammed game pieces from the intake or indexer.
*   **`FORCE_HOME`**: Forces all relevant mechanisms to their fully retracted or home positions.
*   **`CUSTOM_SPEED_TURRET_LAUNCH`**: Launches fuel with a custom speed, typically used for specific shot distances or scenarios.
*   **`TEST_INFINITE_LAUNCH`**: A testing state to continuously launch fuel, primarily for calibration and tuning.
*   **`TEST_IDLE`**: A neutral state for testing individual subsystems or functionalities without interference.
*   **`COAST`**: Motors are set to coast mode (free-spinning).
*   **`BRAKE`**: Motors are set to brake mode (actively resist movement when idle).
*   **Location States**:
    *   Left or Right Side of the field?
    *   Nearest Goal Side?
    *   Opponent's Side of the Field?
    *   Aligned to April Tag on Goal?

### 2026 Vision (Game REBUILT)

*   **Hardware**: Typically utilizes 1-2x Limelight 4s (main vision cameras) and potentially 0-1x Limelight 3s (for additional vision tasks).
*   **`QuestNav`**: Possible integration of `QuestNav` for advanced navigation and localization, potentially used for logging even if not for primary control.
*   **`PhotonVision`**: Potential use of an Orange Pi with `PhotonVision` for object recognition (e.g., detecting game pieces).
*   **AprilTag Field Layout**: `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)` confirms the use of the official 2026 REBUILT field AprilTag layout for accurate localization.

## 2026 Controls Layout (Game REBUILT)

This section details the intended interaction between the operator and pilot to control the robot for the 2026 game, **REBUILT**.

### Concepts

*   **Operator Staging**: The operator sets the robot's next action or desired state. This includes decisions about motion that keeps the center of gravity low or within the robot's frame.
*   **Pilot Action Button**: The pilot's "Action Button" triggers the robot to perform the staged action.
    *   **Press**: Alignment and game piece manipulation begins (potentially waiting for alignment to finish).
    *   **Release**: The actual action (scoring/intaking) occurs.
*   **Robot Autonomy**: The robot leverages its location, angle, and movement to assist in actions (e.g., choosing scoring side, aligning to nearest tag).

### Fuel/Game Piece Mode (Example)

*   **Operator Staging (`LB` held)**: Operator holds `LB` to enter a specific game piece scoring/manipulation mode.
    *   **Dpad Left/Right**: Chooses specific scoring locations or orientations.
    *   **`A, B, X, Y` buttons**: Selects different scoring levels or game piece handling sub-modes.
    *   Releasing `LB` returns to a home state.
*   **Pilot Action (`RB` pressed)**: Pilot presses `RB` to move the robot to an "Action Ready" position (e.g., aligns to a scoring goal, positions manipulator).
*   **Pilot Action Release**: Releasing `RB` initiates the scoring action. The robot will autonomously choose optimal scoring parameters (e.g., front/back) based on its pose/angle/AprilTags.

### Intaking (Example)

*   **Pilot - LeftTrigger**: Initiates Human Player intake.
*   **Operator or Pilot Button**: Extends intake to ground or specific height.
*   **Pilot - RightTrigger**: Initiates ground intake of game pieces.
*   **Combined Trigger Presses**: For specific, contextual intake actions (e.g., `LB + LeftTrigger` for a specialized ground intake).

### Handoffs

*   Operator buttons are typically responsible for managing the transfer of game pieces between different robot mechanisms (e.g., from intake to indexer, from indexer to launcher).

### Operator Controls (Examples)

*   Mechanisms Home/Retract
*   Climb Sequence Initiation

### Auto-Score Diagram (Conceptual Flow)

**Pilot pressing auto-score? (Staging)**
*   Robot aligns to goal.
*   **Pilot pressing action?**
    *   **Close to the goal?**
        *   **Yes**: Manipulators in prep position. "Auto-score Mode" on.
        *   **No**: Nothing happens.
    *   **Cancel "Action Prep"**

**Action State**
*   **Aligned to the goal?**
    *   **Yes**: "Auto-score Mode" on. Pilot can control scoring with action button.
    *   **No**: Nothing happens.
*   **Manipulators in prep position?**
    *   **Yes**: "Auto-score Mode" on. Pilot can control scoring with action button.
    *   **No**: Nothing happens.

### RobotStates (`frc.robot.State.java` and `frc.robot.RobotStates.java`)

As identified in the codebase, the robot utilizes high-level states to drive overall behavior.

*   `frc.robot.State.java`: Defines an enum (`State`) used by `Coordinator.java` to manage simultaneous actions across multiple subsystems. These are typically command-level states.
*   `frc.robot.RobotStates.java`: Defines high-level robot operational modes (also likely an enum) and provides a `setupStates()` method to bind triggers to specific commands or `Coordinator` states. This class represents broader robot objectives like "Intaking" or "Scoring".
