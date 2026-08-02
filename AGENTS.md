# Agent Instructions

## Project Overview

This is **FRC Team Spectrum 3847's robot code for the 2026 season** (REBUILT game).
It is a **Java 17** project built with **GradleRIO 2026.2.1** and **WPILib 2026**.
The robot is a swerve-drive robot with a fuel launcher, turret, indexer, intake, vision, LEDs, and climb.

---

## Build & Development

### Java 17

This repo uses Java 17. If Java 17 is not the default Java version, check if the host has sdkman, and use that. The fallback is to download an archive of Java 17 from Eclipse Temurin to a temp directory.

### Build Command

```bash
./gradlew build
```

This compiles the code, runs Spotless formatting (auto-applies fixes), generates `BuildConstants.java`,
and runs any unit tests.

### Simulate

```bash
./gradlew simulateJava
```

### Deploy to Robot

```bash
./gradlew deploy
```

### Run Tests Only

```bash
./gradlew test
```

### Code Formatting (Critical)

- **Spotless runs automatically on every build** and auto-applies formatting fixes.
- Format: **Google Java Format AOSP** (Android Open Source Project style, 4-space indent).
- Style guide: <https://source.android.com/docs/setup/contribute/code-style#java-style-rules>
- Spotless also formats `.gradle` (Greclipse), `.xml` (EclipseWTP, 2-space indent), `.md`, and `.gitignore`.
- Spotless can also be run directly: `./gradlew spotlessApply`
- **DO NOT** use standard Google Java Format style — use **AOSP** (4-space indent, not 2-space).
- Line endings must be **UNIX (LF)** — Windows CRLF will be auto-converted.

### Known Build Issues & Workarounds

- **`BuildConstants.java` is auto-generated** by the `gversion` plugin at
  `src/main/java/frc/robot/BuildConstants.java`. Do not edit it manually and do not commit it.
- **Spotless** may fail on first run if there are formatting violations — re-run `./gradlew build`
  after it auto-applies fixes, or run `./gradlew spotlessApply` first.
- **SpotBugs** runs on every build. If it fails, check
  `build/reports/spotbugs.html` for details. The exclude filter is `excludeFilter-spotbugs.xml`.
- **ErrorProne** is enabled as an annotation processor — compiler warnings may be elevated to errors.

### CI

GitHub Actions (`.github/workflows/main.yml`) runs `./gradlew build` on every push/PR to `main` using the `wpilib/roborio-cross-ubuntu:2025-24.04` Docker container.

---

## Repository Structure

```
src
├── main: main source code
│   ├── java: Java source files
│   │   └── frc: all FRC application code
│   │       ├── robot: main robot application and subsystems
│   │       │   ├── auton: autonomous routines and PathPlanner integration
│   │       │   ├── configs: robot-specific hardware configs (FM2026, XM2026, PM2026, AM2026, PHOTON2026)
│   │       │   ├── swerve: swerve drive subsystem and controllers
│   │       │   ├── vision: PhotonVision and Limelight vision subsystem
│   │       │   ├── launcher: fuel launcher mechanism
│   │       │   ├── indexerTower: vertical fuel indexer mechanism
│   │       │   ├── indexerBed: horizontal fuel indexer mechanism
│   │       │   ├── fuelIntake: ground intake mechanism
│   │       │   ├── intakeExtension: intake arm extension mechanism
│   │       │   ├── hood: launcher hood pivot mechanism
│   │       │   ├── leds: CANdle LED control and animation
│   │       │   ├── pilot: pilot gamepad bindings and commands
│   │       │   └── operator: operator gamepad bindings and commands
│   │       ├── spectrumLib: reusable Spectrum team utilities (year-to-year code)
│   │       │   ├── gamepads: gamepad abstraction layer
│   │       │   ├── leds: LED management utilities
│   │       │   ├── mechanism: motor and mechanism base classes
│   │       │   ├── sim: physics simulation helpers
│   │       │   ├── swerve: shared swerve helpers (MapleSim integration, SysID)
│   │       │   ├── talonFX: TalonFX motor factory and wrappers
│   │       │   ├── util: utility classes (conversions, CAN IDs, crash tracking)
│   │       │   │   └── exceptions: custom exception classes
│   │       │   └── vision: vision utilities (Limelight helpers)
│   │       └── rebuilt: 2026 game-specific field and targeting helpers
│   │           ├── launchingMaps: distance/angle lookup maps for launcher tuning
│   │           ├── offsets: home offsets and calibration data
│   │           └── targetFactories: target factory implementations
│   └── deploy: files deployed to RoboRIO
│       └── pathplanner: PathPlanner autonomous paths and settings
│           ├── paths: individual path trajectory files
│           └── autos: autonomous routine configurations
└── vendordeps: vendor dependency JSON files (WPILib, CTRE, PathPlanner, etc.)
```

---

## Architecture Patterns

### 1. Robot State Machine

The entire robot is controlled through a **high-level `State` enum** (`State.java`) with 16 states
e.g., `IDLE`, `INTAKE_FUEL`, `LAUNCHER_TRACK`, `LAUNCER_TRACK_WITH_LAUNCH`, `UNJAM`,
`FORCE_HOME`, etc.).

- **`RobotStates.java`**: Sets up WPILib `Trigger`s that fire when the active robot state changes.
  `setupStates()` is called at the start of each robot mode (teleop, auton, test).
- **`Coordinator.java`**: Maps each `State` to specific subsystem commands. This is the central
  orchestration layer — edit this when adding new robot behaviors.

> Note: on branch `2026-offseason-bot`, we are testing a new State Machine architecture that we will adopt in the future.

### 2. Subsystem Pattern

Each subsystem follows this structure:

```
SubsystemName/
├── SubsystemName.java        # Extends Mechanism or SubsystemBase; motor config, hardware init
└── SubsystemNameStates.java  # State machine: defines commands/states for this subsystem
```

- All motor-based subsystems extend `frc.spectrumLib.Mechanism` (wraps TalonFX via Phoenix 6).
- All subsystems implement `frc.spectrumLib.SpectrumSubsystem`.
- Each subsystem has an inner `Config` class for hardware configuration (CAN IDs, offsets, PID, etc.).

### 3. Robot Configurations

Four physical robots are supported via config classes in `frc/robot/configs/`:

| Class | Robot | Notes |
|-------|-------|-------|
| `FM2026` | Final Machine 2026 | Primary competition robot |
| `XM2026` | Experimental Machine 2026 | Development robot |
| `PM2026` | Practice Machine 2026 | Practice robot |
| `AM2026` | Additional Machine 2026 | Alternate robot |

The correct config is selected automatically at runtime based on the **RoboRIO serial number**
mapped in `frc.spectrumLib.Rio`. Encoder offsets, CAN IDs, and mechanism configs differ per robot.

### 4. Telemetry / Logging

- Use **DogLog** (`frc.spectrumLib.Telemetry`) for all telemetry.
- `TuneValue` allows runtime parameter adjustment via NetworkTables (useful for tuning PID, etc.).
- `BuildConstants.java` (auto-generated) is logged on robot init.

### 5. Gamepad Bindings

- `Pilot.java` and `Operator.java` extend `frc.spectrumLib.gamepads.Gamepad`.
- Button/axis bindings are set up in `PilotStates.java` / `OperatorStates.java` and trigger
  state transitions via WPILib `Trigger`s.

### 6. Autonomous

- Autonomous routines use **PathPlanner** (paths and autos in `src/main/deploy/pathplanner/`).
- `Auton.java` registers named commands with PathPlanner and manages routine selection.
- Available autos: `Do Nothing`, `Neutral Zone - Left Trench Start`,
  `Neutral Zone - Right Trench Start`, `Taxi + Preload`.

### 7. Vision

- Dual camera support via **PhotonVision** and **Limelight**.
- `Vision.java` + `VisionSystem.java` handle pose estimation and target tracking.
- `TagProperties.java` and `Field.java` define the AprilTag layout.

### 8. Swerve Drive

- Uses **CTR Electronics CTRE Swerve API** via Phoenix 6.
- Controllers: `RotationController`, `TranslationXController`, `TranslationYController`,
  `TagCenterAlignController`, `TagDistanceAlignController`.
- Full simulation via **Maple Sim** (`MapleSimSwerveDrivetrain.java`).

---

## Key Dependencies (vendordeps/)

| Library | Purpose |
|---------|---------|
| `WPILibNewCommands.json` | WPILib command-based framework (vendored copy with Spectrum patches) |
| `PathplannerLib-2026.1.2.json` | Path following & autonomous |
| `Phoenix6-26.3.0.json` | CTR Talon FX motor controllers & CTRE Swerve |
| `maple-sim.json` | Physics-based swerve simulation |
| `DogLog.json` | Telemetry and logging |
| `photonlib.json` | PhotonVision camera integration |

> **Note**: WPILib's `Trigger` class is **vendored** (modified copy) at
> `src/main/java/edu/wpi/first/wpilibj2/command/button/Trigger.java` — the default start
> condition is set to `false` instead of WPILib's default. Do not replace with the upstream version.

---

## Code Style Conventions

- **Java 17** with `var` usage where it improves readability.
- **Google Java Format AOSP** (4-space indent) — enforced by Spotless on every build.
- **Lombok** (`@Getter`, `@Setter`, etc.) is available via `io.freefair.lombok`.
- Use descriptive state names in `State.java` — prefer `LAUNCER_TRACK_WITH_LAUNCH` over abbreviations.
- Configuration constants belong in the subsystem's inner `Config` class, not in a global constants file.
- CAN device IDs are managed via `frc.spectrumLib.util.CanDeviceId`.
- Unit conversions go through `frc.spectrumLib.util.Conversions`.

---

## Testing

- Tests use **JUnit 5** (`org.junit.jupiter`).
- Test files go in `src/test/java/`.
- Run: `./gradlew test`
- Robot hardware tests require simulation or physical hardware (most tests are unit-level).

---

## Simulation

- Run: `./gradlew simulateJava`
- Simulation uses Maple Sim for swerve physics.
- `RobotSim.java` initializes simulation-specific components.
- GUI config: `simgui.json`, `simgui-ds.json`, `simgui-window.json` (in repo root).

---

## Important Notes

1. **Always run `./gradlew build` after making Java changes** — Spotless will auto-format, and
   SpotBugs + ErrorProne will catch issues. If the build fails due to formatting, re-run it.
2. **Do not manually edit `BuildConstants.java`** — it is regenerated on every build.
3. **New subsystems** should follow the `SubsystemName.java` + `SubsystemNameStates.java` pattern
   and be registered in `Robot.java` (init) and `Coordinator.java` (state mappings).
4. **New robot states** go in `State.java`; their subsystem command mappings go in `Coordinator.java`;
   their trigger setup goes in `RobotStates.java`.
5. **New PathPlanner paths** go in `src/main/deploy/pathplanner/paths/`; autos go in
   `src/main/deploy/pathplanner/autos/`; name commands must be registered in `Auton.java`.
6. **Hardware config per robot** (CAN IDs, encoder offsets) belongs in `configs/FM2026.java`,
   `XM2026.java`, etc. — not hardcoded in subsystem files.
7. **Line endings must be LF (UNIX)**. Windows users: ensure Git is configured with
   `core.autocrlf=false` or `core.autocrlf=input`.
8. **When you discover or are told new repository information, update this file** — Agents should
  append concise, factual notes here recording what was learned, the source (e.g., command output,
  log, or user message), and the date. Do not add secrets, credentials, or sensitive data.
