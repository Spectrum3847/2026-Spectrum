# Copilot Instructions for 2026-Spectrum

## Project Overview

This is **FRC Team Spectrum 3847's robot code for the 2026 season** (REBUILT game).
It is a **Java 17** project built with **GradleRIO 2026.2.1** and **WPILib 2026**.
The robot is a swerve-drive robot with a fuel launcher, turret, indexer, intake, vision, LEDs, and climb.

---

## Build & Development

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

GitHub Actions (`.github/workflows/main.yml`) runs `./gradlew build` on every push/PR to `main`
or `photon` using the `wpilib/roborio-cross-ubuntu:2025-24.04` Docker container.

---

## Repository Structure

```
src/main/java/frc/
├── robot/              # Main robot application code
│   ├── Robot.java      # Entry point — subsystem init, mode handlers (teleop/auton/test/sim)
│   ├── RobotContainer.java  # (not present — see Robot.java + Coordinator.java)
│   ├── RobotStates.java     # High-level trigger/state setup called each mode
│   ├── Coordinator.java     # Maps robot states → subsystem commands
│   ├── State.java           # Robot state enum (21 states)
│   ├── RobotSim.java        # Simulation integration
│   ├── Main.java            # WPILib entry point
│   ├── auton/          # Autonomous routines (PathPlanner integration)
│   ├── configs/        # Robot-specific hardware configs (FM2026, XM2026, PM2026, AM2026)
│   ├── swerve/         # Swerve drive subsystem + controllers
│   ├── vision/         # PhotonVision / Limelight vision subsystem
│   ├── launcher/       # Fuel launcher mechanism
│   ├── indexerTower/   # Vertical fuel indexer
│   ├── indexerBed/     # Horizontal fuel indexer
│   ├── fuelIntake/     # Ground intake
│   ├── intakeExtension/# Intake arm extension
│   ├── turretRotationalPivot/  # Turret rotation mechanism
│   ├── leds/           # CANdle LED control
│   ├── pilot/          # Pilot gamepad bindings
│   └── operator/       # Operator gamepad bindings
├── spectrumLib/        # Shared Spectrum team utilities (base classes, wrappers)
│   ├── SpectrumRobot.java      # Base robot class (extends TimedRobot)
│   ├── SpectrumSubsystem.java  # Base subsystem interface
│   ├── Mechanism.java          # Base class for TalonFX-driven mechanisms
│   ├── Rio.java                # Maps RoboRIO serial numbers to robot identity
│   ├── Telemetry.java          # Centralized DogLog telemetry
│   ├── TuneValue.java          # Runtime-tunable parameter
│   ├── gamepads/Gamepad.java   # Gamepad abstraction
│   ├── leds/                   # LED management
│   ├── vision/                 # Limelight helpers
│   ├── talonFX/                # TalonFX motor factory
│   ├── sim/                    # Physics sim helpers (Arm, Linear, Roller)
│   └── util/                   # Conversions, curves, CAN IDs, crash tracking
└── rebuilt/            # Field/targeting helpers
    ├── ShotCalculator.java     # Trajectory calculations
    ├── Field.java              # Field layout (AprilTags, zones)
    ├── FieldHelpers.java
    ├── Zones.java              # Game zone definitions
    ├── ShiftHelpers.java       # Match timing
    ├── TagProperties.java
    ├── targetFactories/        # Hub/Feed target factories
    └── offsets/HomeOffsets.java

src/main/deploy/
└── pathplanner/        # PathPlanner paths (.path), autos (.auto), settings, navgrid
```

---

## Architecture Patterns

### 1. Robot State Machine

The entire robot is controlled through a **high-level `State` enum** (`State.java`) with 21 states
(e.g., `IDLE`, `INTAKE_FUEL`, `TURRET_TRACK`, `TURRET_TRACK_WITH_LAUNCH`, `L1_CLIMB_PREP`, `UNJAM`,
`FORCE_HOME`, etc.).

- **`RobotStates.java`**: Sets up WPILib `Trigger`s that fire when the active robot state changes.
  `setupStates()` is called at the start of each robot mode (teleop, auton, test).
- **`Coordinator.java`**: Maps each `State` to specific subsystem commands. This is the central
  orchestration layer — edit this when adding new robot behaviors.

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

**NOTE** There is a Photon Machine, only present in the photon branch.

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
- Available autos: `Depot`, `Taxi - Preload`, `Neutral Zone - Left Start`,
  `Neutral Zone - Right Start`, `1 Meter`, `3 Meter`, `5 Meter`.

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
| `Phoenix6-26.1.1.json` | CTR Talon FX motor controllers & CTRE Swerve |
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
- Use descriptive state names in `State.java` — prefer `TURRET_TRACK_WITH_LAUNCH` over abbreviations.
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

## Subagents & Subagent Templates

Subagents are small, focused prompt templates used by Copilot-run subagents (or via `runSubagent`) to perform repeatable repository tasks (searches, scaffolding, small patches, audits). We keep templates as separate Markdown files so they are discoverable and easily updated.

- Location: `.github/subagents/` (templates: `.github/subagents/templates/`).
- Usage: When a user request maps to an existing template, prefer invoking that subagent. If no template exists for a recurring task, propose creating one and ask before applying any changes.
- Adding a new subagent template:
  1. Add the new Markdown template to `.github/subagents/templates/` with YAML frontmatter including `name` and `description`.
  2. Add a one-line entry to `.github/subagents/README.md` describing the template and example usage.
  3. Update this file (`.github/copilot-instructions.md`) with a short bullet referencing the new template (path + purpose).
  4. Use `apply_patch` for edits and include a one-line rationale for each patch hunk. Do not edit generated files like `src/main/java/frc/robot/BuildConstants.java`.
- Permissions & behavior:
  - Copilot is allowed to create and edit subagent template files and to update `copilot-instructions.md` when new agents are added, but must never add secrets or sensitive data.
  - For code changes produced by subagents, produce `apply_patch` patches and do not automatically commit without human review.
  - Keep subagent templates minimal, with clear inputs/outputs and explicit instructions for `apply_patch` output when code changes are expected.

## Important Notes for Agents

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
