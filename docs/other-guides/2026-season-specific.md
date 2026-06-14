# 2026 Season Specific Documentation: REBUILT

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

This page is the orientation tour for the 2026 FRC game (**REBUILT**) — the subsystems we have, the state machine that drives them, the controls layout, and the vision setup. It's the page to read first if you've just cloned the repo and want to understand what does what.

## Subsystems

Each physical thing the robot does lives in its own subsystem folder under `src/main/java/frc/robot/`:

`swerve`, `fuelIntake`, `indexerBed`, `indexerTower`, `intakeExtension`, `launcher`, `hood`, `vision`, `leds`, `pilot`, `operator`.

The "subsystem" name covers everything from "drives motors" (launcher, hood) to "reads controllers" (pilot, operator) to "renders status lights" (leds). Anything with a `periodic()` lifecycle and state to manage gets the same shape.

Most subsystems follow a three-file layout:

* `*.java` — the subsystem class itself, extending `Mechanism` for anything motor-backed.
* `*.Config` (inner class) — every tunable, declared with `@Getter @Setter` so per-robot configs can override defaults.
* `*States.java` — the public API of command factories the rest of the robot uses.

The full structural conventions live in [Class Generation](../coding-conventions/class-generation.md); don't reinvent the layout when adding a new subsystem.

## Per-Robot Configurations

We build multiple physical robots each season and run the same code on all of them. The `Rio.id` field, looked up from the RoboRIO serial number in `frc.spectrumLib.Rio`, decides which configuration is loaded at startup. All configs live under `src/main/java/frc/robot/configs`:

| Config | Bot | Use |
| --- | --- | --- |
| `FM2026` | Final Machine — the competition robot | Precise calibration, the bot we travel with. |
| `PM2026` | Practice Machine | Mirrors competition, minor wear-and-tear tweaks. |
| `XM2026` | Experimental Machine | In-season experimentation and prototyping. Encoder offsets and attachment flags vary. (Off-season work gets its own `OM` config.) |
| `AM2026` | Alpha Machine | Earlier prototype, used pre-build. |
| `PHOTON2026` | Photon's machine | The robot run by Photon, our sister team. |

Each config can mark a mechanism present or absent via `setAttached(boolean)` so a bot without the launcher doesn't try to initialize one.

## States and Triggers

Commands are atomic actions: `swerve.drive()`, `fuelIntake.intake()`, `launcher.shoot()`, `hood.aimAtTarget()`. They run when scheduled and stop on interruption or completion — standard WPILib command-based stuff.

Triggers are conditions that fire commands. A `pilot.X` button press, a sensor reading, a `SpectrumState` that another subsystem flipped. Triggers go in `*States` files as `public static final` so they're reachable from anywhere. Every command bound to a trigger should be wrapped in `log(...)` so the lifecycle ends up in the WPILib log — see [Logging](../tools/logging.md).

The high-level orchestrator is [`Coordinator.java`](../../src/main/java/frc/robot/Coordinator.java). It maps the `State` enum (below) to a coordinated configuration across every mechanism. `applyRobotState(State)` is the entry point — when `INTAKE_FUEL` fires, the Coordinator schedules `FuelIntakeStates.intakeFuel()`, `IndexerBedStates.slowIndex()`, `IntakeExtensionStates.fullExtend()`, and keeps `LauncherStates.idlePrep()` + `HoodStates.home()` for the upper mechanisms.

## Pose Estimation

Swerve odometry and Limelight MegaTag readings feed a WPILib `SwerveDrivePoseEstimator`. The filtering, weighting, and which Limelight to trust live in [`Vision.java`](../../src/main/java/frc/robot/vision/Vision.java) — read [Vision](../tools/vision.md) for the full integration scheme.

## 2026 Robot States

These are the entries in `frc.robot.State`, applied by `Coordinator.applyRobotState(...)`. Each one drives a coordinated setup across launcher, hood, fuel intake, indexer bed/tower, and intake extension.

| State | What it does |
| --- | --- |
| `IDLE` | Ready, neutral. Subsystems home. |
| `INTAKE_FUEL` | Active fuel collection — intake runs, bed slow-indexes, extension fully extends. |
| `SNAKE_INTAKE` | Variant intake when threading fuel through the indexer chain. |
| `TRACK_TARGET` | Launcher + hood aim while the robot is free to drive. Extension extends conditionally. |
| `TRACK_TARGET_WITH_NO_SWERVE` | Same as `TRACK_TARGET` but swerve is held still for auton shot prep. |
| `LAUNCH_WITH_SQUEEZE` | Aim + launch with the delayed-close "squeeze" sequence. |
| `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY` | Squeeze launch without the delayed close. |
| `LAUNCH_WITHOUT_SQUEEZE` | Aim + launch while the intake stays fully extended. |
| `AUTON_TRACK_TARGET` | Auton-mode aim profiles. |
| `AUTON_LAUNCH_WITH_SQUEEZE` | Auton launch with squeeze. |
| `CUSTOM_SPEED_TURRET_LAUNCH` | Tuning-only state: launch at a manually-specified speed. |
| `UNJAM` | Clear jammed fuel from intake or indexer. |
| `FORCE_HOME` | Drive every mechanism to its home position. |
| `TEST_INFINITE_LAUNCH` | Calibration: continuously launch. |
| `TEST_IDLE` | Calibration: neutral, isolate one subsystem. |
| `COAST` / `BRAKE` | Motor neutral-mode overrides. |

`State.getNext()` advances `TRACK_TARGET` → `LAUNCH_WITH_SQUEEZE` for the scoring sequence map.

A few triggers in [`RobotStates.java`](../../src/main/java/frc/robot/RobotStates.java) are derived from field location rather than enum entries: `robotInNeutralZone`, `robotInEnemyZone`, `robotInFeedZone`, `robotInScoreZone`, plus `launcherOnTarget` from the launcher's aim state.

## Vision Hardware

Three Limelight 4s — back, left, right — for AprilTag-based pose estimation. `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)` is the seasonal map. PhotonVision on an Orange Pi remains an option for game-piece detection (not currently wired up); QuestNav is on the radar but unintegrated. Details in [Vision](../tools/vision.md).

## Controls Layout

The pilot drives and runs the fuel cycle; the operator handles unjams, manual overrides, and shot-offset trims. The actual bindings live in [`RobotStates.setupStates()`](../../src/main/java/frc/robot/RobotStates.java), [`PilotStates.setStates()`](../../src/main/java/frc/robot/pilot/PilotStates.java), and [`OperatorStates.setStates()`](../../src/main/java/frc/robot/operator/OperatorStates.java) — this section is the summary, those files are the truth.

### Pilot — fuel cycle (the triggers)

`RT` and `LT` drive the core intake/launch state machine:

* `RT` held alone — `INTAKE_FUEL`.
* `LT` held alone — `LAUNCH_WITH_SQUEEZE`.
* Both held — `LAUNCH_WITHOUT_SQUEEZE`.
* Release `RT` while `LT` is still held — `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY`.
* Release `LT` while `RT` is still held — back to `INTAKE_FUEL`.
* Release both — `IDLE`.

Slow mode engages automatically while the robot is in `LAUNCH_WITH_SQUEEZE`.

### Pilot — everything else

* Left stick — field-relative translation; right stick — rotation (exponential curves, deadzone in `PilotConfig`).
* `X` (hold) — `TRACK_TARGET` (launcher + hood aim while driving).
* `A` (hold) — `UNJAM`.
* `B` (hold) — slow intake-extension close.
* `Y` (hold) — run the selected auton's launch routine.
* `Start` (hold) — `CUSTOM_SPEED_TURRET_LAUNCH` (tuning).
* `Dpad Up/Down` — hood-angle offset trim; `Dpad Left/Right` — drive-angle offset trim (`ShotCalculator`).
* `LB` is the function (`fn`) modifier: `LB + Dpad` reorients the robot heading (up/down/left/right), `LB + RT` ejects fuel, `LB + Select` forces every mechanism home.
* `Select` — clear state to `IDLE`.
* While disabled: `A`/`B` toggle coast/brake, `LB + Select` resets pose to vision.

### Operator

* `A` — aim launcher at target.
* `B` — unjam indexer tower; `X` — unjam indexer bed.
* `RB` — fully retract the intake extension.
* `RT` / `LT` — intake extension manual voltage out (+/−).
* `Dpad` — same `ShotCalculator` hood/drive offset trims as the pilot.
* `LB + Y` — reset intake-extension position.
* Test mode: `A`/`B` full extend/retract, `X` — `TEST_INFINITE_LAUNCH`.

### Hub shifts

REBUILT alternates each alliance's hub between active and inactive during teleop. `RobotStates` restarts the [`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java) timer on auto/teleop/disable transitions so shift-aware triggers know where the match clock is.

## Where Robot State Lives

Two files, two roles:

* [`State.java`](../../src/main/java/frc/robot/State.java) — the enum and the scoring-sequence map. Used by `Coordinator` to apply simultaneous mechanism configurations.
* [`RobotStates.java`](../../src/main/java/frc/robot/RobotStates.java) — defines higher-level triggers and runs `setupStates()` which binds them. Field-derived triggers (alliance side, nearest goal, AprilTag alignment) live here.

If you're adding a behavior that's a coordinated multi-mechanism move, extend `State` and update `Coordinator`. If you're adding a single trigger that fires one existing command, just add it in `RobotStates`.
