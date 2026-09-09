# 2026 Season Specific Documentation: REBUILT

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

This page is the orientation tour for the 2026 FRC game (**REBUILT**) — the subsystems we have, the state machine that drives them, the controls layout, and the vision setup. It's the page to read first if you've just cloned the repo and want to understand what does what.

## Subsystems

Each physical thing the robot does lives in its own folder under `src/main/java/frc/robot/subsystems/`:

`swerve`, `fuelIntake`, `intakeExtension`, `dyeRotor`, `turret`, `launcher`, `hood`, `vision`, `leds`.

The gamepad classes are siblings one level up: `frc/robot/pilot/Pilot.java` and `frc/robot/operator/Operator.java`.

Most subsystems are a **single file** containing three things:

* The subsystem class, extending [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java) for anything motor-backed.
* An inner `*Config` class holding every tunable as `@Getter private final`, applied through the `config*()` helpers in its constructor.
* The state machine — see below. There is no separate `*States` file; that was the 2025 layout.

`launcher/` holds two mechanisms (`Launcher.java` for the flywheel, `LauncherTower.java` for the tower) and `swerve/` splits out `SwerveAlignment.java` and `SwerveConfig.java`. Everything else is one file per folder.

The full structural conventions live in [Class Generation](../coding-conventions/class-generation.md); don't reinvent the layout when adding a new subsystem.

## Per-Robot Configurations

We build multiple physical robots each season and run the same code on all of them. The `Rio.id` field, looked up from the RoboRIO serial number in [`Rio.java`](../../src/main/java/frc/spectrumLib/hardware/Rio.java), decides which configuration is loaded at startup. All configs live under `src/main/java/frc/robot/configs` and extend `Robot.Config`:

| Config | Bot | Use |
| --- | --- | --- |
| `OM2026` | Offseason Machine | The bot currently being run. **The only config the `switch` in `Robot()` can reach today** — it is the `default` arm and there are no others. |
| `FM2026` | Final Machine — the competition robot | Precise calibration, the bot we travel with. |
| `PM2026` | Practice Machine | Mirrors competition, minor wear-and-tear tweaks. |
| `XM2026` | Experimental Machine | In-season experimentation and prototyping. |
| `AM2026` | Alpha Machine | Earlier prototype, used pre-build. |
| `PHOTON2026` | Photon's machine | The robot run by Photon, our sister team. |

Each config sets swerve encoder offsets and marks a mechanism present or absent via `setAttached(boolean)` so a bot without the launcher doesn't try to initialize one. When you bring another robot back online, add its `Rio.id` case to the `switch` in the `Robot` constructor — the config class alone is not enough.

## The State Machine

This is the central pattern in the codebase. Learn it once and every subsystem reads the same way.

**Each mechanism owns two enums.** `WantedState` is what a caller asks for; `SystemState` is what the mechanism has decided to actually do. The only public entry point is `setWantedState(...)`. Each loop, `periodic()` runs `handleStateTransition()` (Wanted → System, usually 1:1, but this is where a mechanism can refuse or defer) and then `applyStates()`, a `switch` on `SystemState` that writes motor output. [`Hood.java`](../../src/main/java/frc/robot/subsystems/hood/Hood.java) is 232 lines and is the cheapest place to read the whole pattern.

**[`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java) is the coordinator.** It has the same shape one level up — `WantedSuperState` and `CurrentSuperState` — and its `periodic()` calls `setWantedState(...)` on every mechanism it owns. Subsystems never call each other. If two mechanisms have to agree about something, that logic belongs in `SuperStructure` and nowhere else.

**Triggers wire hardware to states.** `Pilot` and `Operator` expose `public final Trigger` fields composed from the `Gamepad` base triggers (`LB.and(selectButton)`, `AButton.and(disabled)`). They are bound to `superStructure.setStateCommand(...)` in [`Robot.configureBindings()`](../../src/main/java/frc/robot/Robot.java) — that method is the single place controls are attached to behavior. Commands bound to a trigger should be wrapped in `Telemetry.log(...)` so the lifecycle ends up in the WPILib log — see [Logging](../tools/logging.md).

## Pose Estimation

Swerve odometry and Limelight MegaTag readings feed a WPILib `SwerveDrivePoseEstimator`. The filtering, weighting, and which Limelight to trust live in [`Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) — read [Vision](../tools/vision.md) for the full integration scheme.

## 2026 Robot States

These are the entries in `SuperStructure.WantedSuperState`. Each one drives a coordinated setup across launcher, launcher tower, turret, hood, fuel intake, dye rotor, and intake extension.

| State | What it does |
| --- | --- |
| `IDLE` | Ready, neutral. Subsystems home. |
| `INTAKE_FUEL` | Active fuel collection — intake runs, extension extends. |
| `TRACK_TARGET` | Turret + hood aim while the robot is free to drive. |
| `LAUNCH_WITH_SQUEEZE` | Aim + launch with the delayed-close "squeeze" sequence. |
| `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY` | Squeeze launch without the delayed close. |
| `LAUNCH_WITHOUT_SQUEEZE` | Aim + launch while the intake stays fully extended. |
| `LAUNCH_WITH_BRAKE` | Launch while the drivetrain is held still. |
| `AUTON_INTAKE_FUEL` | Auton-mode intake profile. |
| `AUTON_TRACK_TARGET` | Auton-mode aim profile. |
| `AUTON_LAUNCH_WITH_SQUEEZE` | Auton launch with squeeze. |
| `AUTON_LAUNCH_WITHOUT_SQUEEZE` | Auton launch, intake left extended. |
| `UNJAM` | Clear jammed fuel from intake or rotor. |
| `KICKER_UNJAM` | Same as `UNJAM`, but the intake kicker keeps running forward. |
| `FORCE_HOME` | Drive every mechanism to its home position. |

`CurrentSuperState` mirrors these and adds `AUTON_IDLE`, which the coordinator resolves to itself rather than being requestable.

Four triggers on `SuperStructure` are derived from field location rather than the enum: `robotInNeutralZone()`, `robotInEnemyZone()`, `robotInFeedZone()`, `robotInScoreZone()`.

## Vision Hardware

Three Limelight 4s — back, left, right — for AprilTag-based pose estimation. `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)` is the seasonal map. PhotonVision on an Orange Pi remains an option for game-piece detection (not currently wired up); QuestNav is on the radar but unintegrated. Details in [Vision](../tools/vision.md).

## Controls Layout

The pilot drives and runs the fuel cycle; the operator handles unjams, feed direction, and shot trims. The bindings are all in [`Robot.configureBindings()`](../../src/main/java/frc/robot/Robot.java); the trigger names come from [`Pilot.java`](../../src/main/java/frc/robot/pilot/Pilot.java) and [`Operator.java`](../../src/main/java/frc/robot/operator/Operator.java). This section is the summary, those files are the truth.

### Pilot — fuel cycle (the triggers)

`LT` and `RT` drive the core intake/launch state machine:

* `LT` held alone — `INTAKE_FUEL`.
* `RT` held alone — `LAUNCH_WITH_SQUEEZE`.
* Both held — `LAUNCH_WITHOUT_SQUEEZE` (intake stays extended).
* Release `LT` while `RT` is still held — `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY`.
* Release `RT` while `LT` is still held — back to `INTAKE_FUEL`.
* Release both — `IDLE`.
* `LB + RT` — `LAUNCH_WITH_BRAKE`.

### Pilot — everything else

* Left stick — field-relative translation; right stick — rotation (exponential curves, deadzone in `PilotConfig`).
* `X` (hold) — `TRACK_TARGET`. Releasing it returns to `IDLE` unless a launch is already in progress.
* `A` (hold) — `UNJAM`.
* `B` (hold) — `KICKER_UNJAM`.
* `LB` is the function (`fn`) modifier: `LB + Dpad` reorients the robot heading, `LB + Select` resets pose to vision.
* `Select` alone — `FORCE_HOME`, releasing to `IDLE`.
* While disabled: `A` puts the intake extension and turret in coast.

### Operator

* `Y` (hold) — feed override: feeds regardless of the shot-readiness gates.
* `LB` / `RB` — feed left / feed right; releasing either returns to the default feed target.
* `Dpad Up`/`Down` — hood-angle trim; `Dpad Left`/`Right` — turret-angle trim (`ShotCalculator`). Each press is also the shot-outcome signal — see [Shot Records and Trim Events](../tools/shot-log.md).
* `Start + Select` — reset all shot trims.
* While disabled: `A` coast; `B` zeroes the turret (point it away from the intake by hand first).

## Hub shifts

REBUILT alternates each alliance's hub between active and inactive during teleop. `Robot.configureBindings()` restarts the [`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java) timer on auto and disabled transitions so shift-aware triggers know where the match clock is.

## Where Robot State Lives

One file: [`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java). It holds the `WantedSuperState` / `CurrentSuperState` enums, the transition table, the per-state `apply*()` methods, and the field-derived triggers.

If you're adding a coordinated multi-mechanism move, add a `WantedSuperState` entry, a `CurrentSuperState` entry, a case in `handleStateTransitions()`, and an `apply*()` method. If you're adding a control that fires an existing state, add the `Trigger` to `Pilot` or `Operator` and bind it in `Robot.configureBindings()`.
