# 2026 Season Specific Documentation: REBUILT

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

This page is the orientation tour for the 2026 FRC game (**REBUILT**) — the subsystems we have, the state machine that drives them, the controls layout, and the vision setup. It's the page to read first if you've just cloned the repo and want to understand what does what.

## Subsystems

Each physical thing the robot does lives in its own subsystem folder under `src/main/java/frc/robot/subsystems/`:

`swerve`, `fuelIntake`, `indexerBed`, `indexerTower`, `intakeExtension`, `launcher`, `hood`, `vision`, `leds` — plus `SuperStructure.java`, the orchestrator that sits above them. The gamepads (`pilot`, `operator`) live one level up under `src/main/java/frc/robot/`.

Anything with a `periodic()` lifecycle and state to manage gets the same shape.

Each subsystem is one file: the subsystem class (extending `Mechanism` for anything motor-backed), an inner `*Config` class holding every tunable as `@Getter private final` fields, and inner `WantedState`/`SystemState` enums driving its state machine. There is no separate `*States` command-factory class — the subsystem drives itself via `setWantedState(...)` + `handleStateTransition()` + `applyStates()`.

The full structural conventions live in [Class Generation](../coding-conventions/class-generation.md); don't reinvent the layout when adding a new subsystem.

## Per-Robot Configurations

We build multiple physical robots each season and run the same code on all of them. The `Rio.id` field, looked up from the RoboRIO serial number in `frc.spectrumLib.hardware.Rio`, decides which configuration is loaded at startup. All configs live under `src/main/java/frc/robot/configs`:

|    Config    |                  Bot                  |                                                                Use                                                                |
|--------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------|
| `FM2026`     | Final Machine — the competition robot | Precise calibration, the bot we travel with.                                                                                      |
| `PM2026`     | Practice Machine                      | Mirrors competition, minor wear-and-tear tweaks.                                                                                  |
| `XM2026`     | Experimental Machine                  | In-season experimentation and prototyping. Encoder offsets and attachment flags vary. (Off-season work gets its own `OM` config.) |
| `AM2026`     | Alpha Machine                         | Earlier prototype, used pre-build.                                                                                                |
| `PHOTON2026` | Photon's machine                      | The robot run by Photon, our sister team.                                                                                         |

Each config can mark a mechanism present or absent via `setAttached(boolean)` so a bot without the launcher doesn't try to initialize one.

## States and Triggers

Each subsystem exposes a `setWantedState(<Subsystem>.WantedState)` entry point and runs its own `WantedState`/`SystemState` machine internally. Triggers are conditions that fire commands — a `pilot.X` press, a sensor reading, a `SpectrumState` another subsystem flipped.

The high-level orchestrator is [`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java). It maps the `WantedSuperState` enum (below) to a coordinated configuration across every mechanism. `setWantedSuperState(WantedSuperState)` is the entry point (with `setStateCommand(...)` as the command wrapper used by bindings) — when `INTAKE_FUEL` fires, `SuperStructure` fans that intent out to each subsystem's `setWantedState(...)`: the fuel intake runs, the indexer bed slow-indexes, the extension extends, and the launcher/hood hold their prep/aim states.

## Pose Estimation

Swerve odometry and Limelight MegaTag readings feed a WPILib `SwerveDrivePoseEstimator`. The filtering, weighting, and which Limelight to trust live in [`Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) — read [Vision](../tools/vision.md) for the full integration scheme.

## 2026 Robot States

These are the entries in `SuperStructure.WantedSuperState`, applied by `setWantedSuperState(...)`. Each one drives a coordinated setup across launcher, hood, fuel intake, indexer bed/tower, and intake extension.

|                State                |                             What it does                              |
|-------------------------------------|-----------------------------------------------------------------------|
| `IDLE`                              | Ready, neutral. Subsystems home.                                      |
| `INTAKE_FUEL`                       | Active fuel collection — intake runs, bed indexes, extension extends. |
| `TRACK_TARGET`                      | Launcher + hood aim while the robot is free to drive.                 |
| `LAUNCH_WITH_SQUEEZE`               | Aim + launch with the delayed-close "squeeze" sequence.               |
| `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY` | Squeeze launch without the delayed close.                             |
| `LAUNCH_WITHOUT_SQUEEZE`            | Aim + launch while the intake stays extended.                         |
| `LAUNCH_WITH_BRAKE`                 | Launch while holding the drivetrain in brake.                         |
| `AUTON_TRACK_TARGET`                | Auton-mode aim.                                                       |
| `AUTON_INTAKE_FUEL`                 | Auton-mode fuel collection.                                           |
| `UNJAM`                             | Clear jammed fuel from intake or indexer.                             |
| `EJECT`                             | Spit fuel back out.                                                   |
| `FORCE_HOME`                        | Drive every mechanism to its home position.                           |

`CurrentSuperState` mirrors these — `handleStateTransition()` maps the wanted state to the current one each loop.

A few field-location triggers live on `SuperStructure` itself rather than in the enum: `robotInNeutralZone()`, `robotInEnemyZone()`, `robotInFeedZone()`, `robotInScoreZone()` (which delegate to the swerve pose).

## Vision Hardware

Three Limelight 4s — back, left, right — for AprilTag-based pose estimation. `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)` is the seasonal map. Game-piece detection is not currently wired up; QuestNav is on the radar but unintegrated. Details in [Vision](../tools/vision.md).

## Controls Layout

The pilot drives and runs the fuel cycle; the operator handles offset trims and mechanism resets. Every binding lives in [`Robot.configureBindings()`](../../src/main/java/frc/robot/Robot.java); the `Pilot`/`Operator` classes just expose the button `Trigger`s (`LT`, `RT`, `XButton`, …). This section is the summary, that method is the truth.

### Pilot — fuel cycle (the triggers)

`LT` and `RT` drive the core intake/launch state machine:

* `LT` held alone — `INTAKE_FUEL`.
* `RT` held alone — `LAUNCH_WITH_SQUEEZE`.
* Both held — `LAUNCH_WITHOUT_SQUEEZE`.
* Release `LT` while `RT` is still held — `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY`.
* Release `RT` while `LT` is still held — back to `INTAKE_FUEL`.
* Release both — `IDLE`.

### Pilot — everything else

* Left stick — field-relative translation; right stick — rotation (exponential curves, deadzone in `Pilot`'s config).
* `X` (hold) — `TRACK_TARGET` (launcher + hood aim while driving); release → `IDLE`.
* `A` (hold) — `UNJAM`; release → `IDLE`.
* `LT + LB` — `EJECT`; release → `IDLE`.
* `Select` — `FORCE_HOME`; release → `IDLE`.
* `LB + Dpad` (up/left/down/right) — reorient the robot heading forward/left/back/right.
* While disabled: `A` → coast mechanisms, `B` → brake mechanisms.

### Operator

* `Dpad Down/Up` — hood-angle offset trim (−/+, via `ShotCalculator`).
* `Dpad Right/Left` — drive-angle offset trim (−/+, via `ShotCalculator`).
* `Select` — `FORCE_HOME`; release → `IDLE`.
* `LB + Y` — reset the intake-extension position to max (with a rumble confirmation).
* While disabled: `A` → coast mechanisms, `B` → brake mechanisms.

### Hub shifts

REBUILT alternates each alliance's hub between active and inactive during teleop. `Robot.configureBindings()` calls [`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java)`::initialize` on teleop/auto/disable transitions so shift-aware logic knows where the match clock is.

## Where Robot State Lives

* [`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java) — the `WantedSuperState`/`CurrentSuperState` enums and the `handleStateTransition()`/`applyStates()` logic that fans a super-state out across every mechanism. This is where a coordinated multi-mechanism move belongs.
* [`Robot.java`](../../src/main/java/frc/robot/Robot.java) — `configureBindings()` wires gamepad triggers and `Auton` event triggers to `superStructure.setStateCommand(...)`. A single trigger that fires one existing state goes here.

If you're adding a behavior that's a coordinated multi-mechanism move, add a `WantedSuperState` and handle it in `SuperStructure`. If you're adding a single trigger that fires an existing state, just bind it in `Robot.configureBindings()`.
