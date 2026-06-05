# 2026 Spectrum Robot Code

Robot code for Spectrum 3847's FRC 2026 season — [REBUILT](https://www.firstinspires.org/robotics/frc/game-and-season).

## Description

This is the working codebase for our robots in the 2026 REBUILT FRC Competition. The code is actively developed, so expect ongoing changes. The architecture is built around explicit state machines in every subsystem, coordinated by a central `SuperStructure`, and backed by a high-fidelity physics simulation for off-robot development.

---

## Key Features

- **WantedState / SystemState machines** — Every subsystem owns its own dual-enum state machine. `WantedState` is set from outside; `SystemState` is resolved internally each loop. All motor commands live in private state-method calls, keeping subsystem logic self-contained.
- **SuperStructure** — A top-level coordinator that fans out `WantedState` assignments to every subsystem in a single call, eliminating the need for multi-mechanism command groups in most cases.
- **Polynomial shot model** — `ShotCalculator` uses a polynomial virtual-target model to compute hood angle and flywheel RPM from any field position in real time.
- **High-fidelity simulation** — Full projectile physics (drag, Magnus effect, field-collision detection) via `FuelPhysicsSim`; 3D bump-ramp contact physics via `RobotBumpSim`; articulating Mechanism2d representations of every mechanism.
- **`Mechanism` base class** — Wraps all TalonFX configuration, control-mode calls, soft limits, current limits, and sim hooks behind a clean API. All hardware access is gated by `isAttached()` so the same code runs in sim and on metal.
- **DogLog telemetry** — Structured `.wpilog` file logging with optional NT publishing; subsystem current/power/energy tracked per loop via `BatteryLogger`.
- **Three-Limelight vision** — MegaTag1 and MegaTag2 pose fusion with per-measurement confidence tiers, spin-rate rejection, field-boundary rejection, and automatic camera selection.

---

## Architecture

`SuperStructure` acts as the single point of control for the robot. It holds a `WantedSuperState` and a `CurrentSuperState`, resolves which subsystem states to apply each loop, and calls `setWantedState()` on every subsystem accordingly. This keeps button bindings simple — callers set one super state and the rest is handled automatically.

Each subsystem follows the same pattern:

1. External caller sets `WantedState` via `setWantedState()`.
2. `periodic()` calls `handleStateTransition()` → resolves `SystemState`.
3. `periodic()` calls `applyStates()` → dispatches motor commands for the current `SystemState`.

---

## Project Structure

```text
src
└── main
    └── java
        └── frc
            ├── robot                        Main robot application
            │   ├── Robot.java               Robot lifecycle, binding setup
            │   ├── RobotSim.java            Mechanism2d simulation canvas
            │   ├── pilot
            │   │   └── Pilot.java           Pilot gamepad bindings and axis curves
            │   ├── operator
            │   │   └── Operator.java        Operator gamepad bindings
            │   ├── auton
            │   │   └── Auton.java           PathPlanner autonomous routines and triggers
            │   └── subsystems               All robot subsystems
            │       ├── SuperStructure.java  Cross-subsystem state coordinator
            │       ├── swerve
            │       │   ├── Swerve.java      Swerve drive (CTRE Phoenix + Maple-Sim)
            │       │   └── SwerveConfig.java  Module constants, PIDs, CAN IDs
            │       ├── vision
            │       │   └── Vision.java      Three-Limelight pose fusion subsystem
            │       ├── launcher
            │       │   └── Launcher.java    Flywheel launcher (1 leader, 3 followers)
            │       ├── indexerTower
            │       │   └── IndexerTower.java  Vertical fuel indexer
            │       ├── indexerBed
            │       │   └── IndexerBed.java  Horizontal fuel indexer
            │       ├── fuelIntake
            │       │   └── FuelIntake.java  Ground intake rollers
            │       ├── intakeExtension
            │       │   └── IntakeExtension.java  Intake extension
            │       ├── hood
            │       │   └── Hood.java        Adjustable hood
            │       └── leds
            │           └── CANdleLeds.java  CANdle LED control (stub)
            │
            ├── rebuilt                      2026 REBUILT game-specific code
            │   ├── Field.java               Field geometry constants
            │   ├── FieldHelpers.java        Alliance-aware coordinate helpers
            │   ├── Zones.java               Named field zone definitions
            │   ├── ShiftHelpers.java        Alliance-origin coordinate shifting
            │   ├── ShotCalculator.java      Polynomial virtual-target shot model
            │   ├── FuelPhysicsSim.java      Projectile physics sim (drag, Magnus, collisions)
            │   ├── RobotBumpSim.java        3D bump-ramp contact physics for swerve sim
            │   ├── targetFactories          Dynamic aiming target factories
            │   │   ├── HubTargetFactory.java
            │   │   └── FeedTargetFactory.java
            │   └── launchingMaps            Field-position → shot-parameter maps
            │       ├── HomeMap.java
            │       └── AndyMarkMap.java
            │
            └── spectrumLib                  Reusable Spectrum team library
                ├── framework                Robot/subsystem base classes
                ├── hardware                 TalonFX, CANcoder, servo, RIO identity
                ├── telemetry                DogLog wrapper, BatteryLogger, TuneValue
                ├── util                     Utilities, curves, CAN IDs, triggers
                ├── mechanism                Abstract TalonFX mechanism base class
                ├── gamepads                 Xbox controller abstraction
                ├── leds                     AddressableLED pattern library
                ├── sim                      Mechanism2d sim helpers (arm, roller, linear)
                ├── swerve                   Maple-Sim bridge, SysID routines
                └── vision                   Limelight helpers and VisionLogger
```

See [`src/main/java/frc/spectrumLib/README.md`](src/main/java/frc/spectrumLib/README.md) for full SpectrumLib documentation.

---

## Hardware Overview

| Subsystem | Motor IDs | CAN Bus | Notes |
|---|---|---|---|
| Swerve FL | Drive 1, Steer 2, Encoder 3 | CANivore | Front-left module |
| Swerve FR | Drive 11, Steer 12, Encoder 13 | CANivore | Front-right module |
| Swerve BL | Drive 21, Steer 22, Encoder 23 | CANivore | Back-left module |
| Swerve BR | Drive 31, Steer 32, Encoder 33 | CANivore | Back-right module |
| Hood | 15 | CANivore | Motion Magic position |
| Fuel Intake | 5 (lead), 6 (follower) | RIO CAN | Torque-current FOC |
| IntakeExtension | 7 | CANivore | Motion Magic position |
| IndexerBed | 8 (lead), 9 (follower) | CANivore | Velocity Torque-current FOC |
| IndexerTower | 51 (lead), 52 (follower) | CANivore | Velocity Torque-current FOC |
| Launcher | 46 (lead), 47/48/49 (followers) | CANivore | Velocity Torque-current FOC |
| Pigeon 2 | 0 | CANivore | IMU |

---

## Build Tools and Extensions

| Tool | Purpose |
|---|---|
| **Spotless** | Auto-formats code on every build for consistent style across programmers |
| **SpotBugs** | Static analysis — catches common bugs (wrong operator, null dereference, etc.) |
| **Lombok** | `@Getter` / `@Setter` annotations to eliminate boilerplate |
| **DogLog** | Structured `.wpilog` file logging with optional NetworkTables publishing |
| **Error Lens** | Inline error/warning highlighting in VS Code |
| **Git Config User Profiles** | Multiple programmers can commit under their own names on shared machines |
| **GitLens** | Blame, history, and diff tooling in VS Code |
| **SpellRight** | Spell-check in VS Code |

---

## Dependencies

| Dependency | Purpose |
|---|---|
| [WPILib 2026](https://github.com/wpilibsuite/allwpilib) | Core robot framework |
| [CTRE Phoenix 6](https://pro.docs.ctr-electronics.com/en/stable/) | TalonFX, CANcoder, Pigeon 2, swerve control |
| [PathPlanner](https://github.com/mjansen4857/pathplanner) | Autonomous path generation and following |
| [DogLog](https://github.com/jonahsnider/doglog) | `.wpilog` file logging and NT telemetry |
| [Maple-Sim](https://github.com/Shenzhen-Robotics-Alliance/Maple-Sim) | High-fidelity swerve drive simulation *(sim only)* |
| [Limelight](https://docs.limelightvision.io/) | AprilTag vision — MegaTag1 and MegaTag2 pose fusion |
| [Lombok](https://projectlombok.org/) | Compile-time `@Getter` / `@Setter` generation |

---

#### View the online JavaDoc [here](https://spectrum3847.github.io/2026-Spectrum).
