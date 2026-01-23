# 2026-Spectrum

Repo for Spectrum's 2026 robots (Team 3847).

## Overview
This repository contains the robot code for FRC Team 3847 (Spectrum) for the 2026 season. The project is built using the WPILib framework and follows a command-based architecture. It leverages a custom library, `spectrumLib`, for shared utilities and abstractions.

### Key Features
- **Swerve Drive**: Powered by CTRE Phoenix 6.
- **Autonomous**: Integrated with PathPlanner for complex path following and auto routines.
- **Vision**: Uses PhotonLib for vision processing.
- **Logging**: Uses DogLog for telemetry and logging.
- **Subsystems**: Includes Swerve, Fuel Intake, Turret (Rotational & Hood), Intake Extension, Indexer, Launcher, and LEDs.

## Requirements
- **Java Development Kit (JDK) 17**: Required for WPILib 2026.
- **WPILib 2026**: The project uses GradleRIO version `2026.2.1`.
- **Frc-characterization / SysId**: (Optional) For robot characterization.

## Stack
- **Language**: Java 17
- **Framework**: WPILib (Command-Based)
- **Package Manager**: Gradle (via GradleRIO)
- **Vendor Libraries**:
  - CTRE Phoenix 6
  - PathPlannerLib
  - PhotonLib
  - DogLog
  - WPILib New Commands

## Setup & Run Commands

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/Spectrum3847/2026-Spectrum.git
   ```
2. Open the project in VS Code (with WPILib extension) or IntelliJ IDEA.

### Common Gradle Tasks
- **Build**: `./gradlew build`
- **Deploy to RoboRIO**: `./gradlew deploy`
- **Run Simulation**: `./gradlew simulateJava`
- **Format Code**: `./gradlew spotlessApply`
- **Run Static Analysis**: `./gradlew spotbugsMain`
- **Generate Javadoc**: `./gradlew javadoc`

## Project Structure
- `src/main/java/frc/robot`: Main robot logic.
  - `auton/`: Autonomous mode definitions and commands.
  - `configs/`: Configuration sets for different robots (e.g., FM2026).
  - `swerve/`, `fuelIntake/`, `launcher/`, etc.: Subsystem-specific implementations.
- `src/main/java/frc/spectrumLib`: Shared library code used across multiple robots.
- `src/main/deploy`: Files deployed to the RoboRIO (PathPlanner paths, autos, etc.).
- `vendordeps`: JSON files defining vendor library dependencies.

## Environment Variables
- `TEAM_NUMBER`: (Optional) Can be overridden via command line if not using the default `3847`.

## Scripts
- `gradlew`: Gradle wrapper script for running tasks.
- `gradlew.bat`: Windows version of the Gradle wrapper.

## Tests
The project uses JUnit 5 for testing.
- **Run Tests**: `./gradlew test`
*Note: Unit tests are currently under development and may be minimal or absent.*

## License
This project is licensed under the MIT License - see the `LICENSE` file (or `WPILib-License.md` for WPILib components) for details. 
*TODO: Verify if a specific 2026 license file should be added.*
