# 2026-Spectrum

Repo for Spectrum's 2026 robots.

## Overview
This repository contains the robot code for Spectrum for the 2026 season. The project is built using the WPILib. It uses a custom library, `spectrumLib`, for shared utilities.

### Key Features
- **Swerve Drive**: Powered by CTRE Phoenix 6.
- **Autonomous**: Integrated with PathPlanner for complex path following and auto routines.
- **Vision**: Uses PhotonLib for vision processing.
- **Logging**: Uses DogLog for telemetry and logging.
- **Subsystems**: Includes Swerve, Fuel Intake, Turret (Rotational & Hood), Intake Extension, Indexer, Launcher, and LEDs.

## Requirements
- **Java Development Kit (JDK) 17**
- **WPILib 2026**

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

## Setup

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/Spectrum3847/2026-Spectrum.git
   ```
2. Open the project in WPILib VS Code.

## Project Structure
- `src/main/java/frc/robot`: Main robot logic.
  - `auton/`: Autonomous mode definitions and commands.
  - `configs/`: Configuration sets for different robots (e.g., FM2026).
  - `swerve/`, `fuelIntake/`, `launcher/`, etc.: Subsystem-specific implementations.
- `src/main/java/frc/spectrumLib`: Shared library code used across multiple robots.
- `src/main/deploy`: Files deployed to the RoboRIO (PathPlanner paths, autos, etc.).
- `vendordeps`: JSON files defining vendor library dependencies.
