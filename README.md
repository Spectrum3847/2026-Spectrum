# 2026 Spectrum Robot Code

Robot code for Spectrum 3847's FRC 2026 season.

## Description

This is the working code base for our robots in the 2026 REBUILT FRC Competition. The code is constantly being worked on, so watch out for bugs and issues.

### Features

* Mechanism class wraps all the TalonFX configurations and control mode calls so they are easier to use.
* Heavy use of Triggers as states for activating commands. This all but eliminates the need for complex multi-mechanism command groups.
* Simulation classes allow us to build a good representation of the complete robot in sim so more of the code can be tested without the robot.
* CachedDouble: Allows a value to only be updated once per periodic loop. This is useful for CANbus calls to sensors or motors, etc.
* LED classes that use the CTRE CANdle for animations, easily have multiple strips on one port, has priorities for when to override animations that are currently running, different animations for each control mode, etc.

### Build Tools and Extensions

* **Spotless:** Autoformats are code on each build so that we keep a consistent format across programmers.
* **SpotBugs**: Catches some common bugs in the software such as using = instead of == in a conditional, etc.
* **Lombok:** Annotations to create Getter and Setter methods. This allows for less boilerplate code.
* **Error Lens:** Highlights errors and makes them easier to see and fix.
* **Git Config User Profiles:** Allows multiple programmers to share the same computers and commit under their own names.
* **Git Lens:** lets you see who committed changes and more
* **SpellRight:** Spell Check for VSCode

### Dependencies

* WPILib 2026
* CTRE Phoenix 6 (using their swerve control code)
* PathPlanner
* DogLog (logging)
* MapleSim (drivetrain simulation)

## Project Structure

* `frc/robot` is in-season robot code. Configuration files let one codebase run on several robots. It leans on WPILib commands and triggers.
* [`SpectrumLib`](src/main/java/frc/spectrumLib) is code we aim to reuse year to year.
* Each subsystem can be modified independently without needing to understand the rest of the robot code.

The source tree is the authority on what exists and where. Below is the shape of the repo, one level
below the split above, so you know which directory to open:

```text
src/main/java/frc
├── robot/          in-season code
│   ├── auton/      autonomous routines and PathPlanner integration
│   ├── configs/    per-robot hardware config, one class per robot
│   ├── pilot/      pilot gamepad bindings
│   ├── operator/   operator gamepad bindings
│   └── subsystems/ the orchestrator plus one folder per mechanism
├── spectrumLib/    reusable utilities
└── rebuilt/        game-specific field, sim, and targeting helpers

src/main/deploy/pathplanner/   paths and autos deployed to the RoboRIO
vendordeps/                    vendor dependency JSON files
```

### Documentation

[`docs/`](docs/index.md) holds our conventions, workflows, and hard-won gotchas, and is written for
people. It deliberately does not restate what the code does, because a copy of the code in prose goes
stale and then misleads. If you want to know what a class or a state does, read the class.

#### View the online JavaDoc [here](https://spectrum3847.github.io/2026-Spectrum).
