# 2026 Spectrum Robot Code

Robot code for Spectrum 3847's FRC 2026 season.

## Description

This is the working code base for our robots in the 2026 REBUILT FRC Competition. The code is constantly being worked on, so watch out for bugs and issues.

### Features

* Mechanism class wraps all the FalconFX configurations and control mode calls so they are easier to use.
* Heavy use of Triggers as states for activating commands. This all but eliminates the need for complex multi-mechanism command groups.
* Simulation classes allow us to build a good representation of the complete robot in sim so more of the code can be tested without the robot.
* CachedDouble: Allows a value to only be updated once per periodic loop. This is useful for CANbus calls to sensors or motors, etc.
* LED classes that use the CTRE CANdle for animations, easily have multiple strips on one port, has priorities for when to override animations that are currently running, different animations for each control mode, etc.

### Build Tools and Extensions

* **Spotless:** Autoformats are code on each build so that we keep a consistent format across programmers.
* **SpotBugs**: Catches some common bugs in the software such as using = instead of == in a conditional, etc.
* **Lomok:** Annotations to create Getter and Setter methods. This allows for less boilerplate code.
* **Error Lens:** Highlights errors and makes them easier to see and fix.
* **Git Config User Profiles:** Allows multiple programmers to share the same computers and commit under their own names.
* **Git Lens:** lets you see who committed changes and more
* **SpellRight:** Spell Check for VSCode

### Dependencies

* WPILib 2026
* CTRE Phoenix 6 (Using there swerve control code)
* PathPlanner
* PhotonLib (For vision simulation)

## Project Structure

* Robot = In season robot code, we have configuration files to be able to run our code base on multiple robots at once. Heavily uses WPILib commands and triggers.
* [`SpectrumLib`](src/main/java/frc/spectrumLib) = Code that we try to reuse year to year.
* Our goal is to make our software easy for multiple people to contribute too. Each subsystem should be able to be modified on it's own without needing to understand the rest of the robot code.

```text
src/
├── main/                               # Main source code
│   ├── java/                           # Java source files
│   │   └── frc/                        # All FRC application code
│   │       ├── robot/                  # Main robot application and subsystems
│   │       │   ├── auton/              # Autonomous routines and PathPlanner integration
│   │       │   ├── configs/            # Robot-specific hardware configs (FM2026, XM2026, PM2026, AM2026)
│   │       │   ├── swerve/             # Swerve drive subsystem and controllers
│   │       │   ├── vision/             # PhotonVision and Limelight vision subsystem
│   │       │   ├── launcher/           # Fuel launcher mechanism
│   │       │   ├── indexerTower/       # Vertical fuel indexer mechanism
│   │       │   ├── indexerBed/         # Horizontal fuel indexer mechanism
│   │       │   ├── fuelIntake/         # Ground intake mechanism
│   │       │   ├── intakeExtension/    # Intake arm extension mechanism
│   │       │   ├── turretRotationalPivot/ # Turret rotation mechanism
│   │       │   ├── leds/               # CANdle LED control and animation
│   │       │   ├── pilot/              # Pilot gamepad bindings and commands
│   │       │   └── operator/           # Operator gamepad bindings and commands
│   │       ├── spectrumLib/            # Reusable Spectrum team utilities (year-to-year code)
│   │       │   ├── gamepads/           # Gamepad abstraction layer
│   │       │   ├── leds/               # LED management utilities
│   │       │   ├── mechanism/          # Motor and mechanism base classes
│   │       │   ├── sim/                # Physics simulation helpers
│   │       │   ├── talonFX/            # TalonFX motor factory and wrappers
│   │       │   ├── util/               # Utility classes (conversions, CAN IDs, crash tracking)
│   │       │   │   └── exceptions/     # Custom exception classes
│   │       │   └── vision/             # Vision utilities (Limelight helpers)
│   │       └── rebuilt/                # 2026 game-specific field and targeting helpers
│   │           ├── offsets/            # Home offsets and calibration data
│   │           └── targetFactories/    # Target factory implementations
│   └── deploy/                         # Files deployed to RoboRIO
│       └── pathplanner/                # PathPlanner autonomous paths and settings
│           ├── paths/                  # Individual path trajectory files
│           └── autos/                  # Autonomous routine configurations
└── vendordeps/                         # Vendor dependency JSON files (WPILib, CTRE, PathPlanner, etc.)
```

#### View the online JavaDoc [here](https://spectrum3847.github.io/2026-Spectrum/javadoc).
