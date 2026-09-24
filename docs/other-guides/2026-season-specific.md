# 2026 Season Specific Documentation: REBUILT

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

This page is the orientation tour for the 2026 FRC game (**REBUILT**), the subsystems we have, the state machine that drives them, the controls layout, and the vision setup. It's the page to read first if you've just cloned the repo and want to understand what does what.

## Subsystems

Each physical thing the robot does lives in its own subsystem folder under `src/main/java/frc/robot/subsystems/`:

`swerve`, `fuelIntake`, `indexerBed`, `indexerTower`, `intakeExtension`, `launcher`, `hood`, `vision`, `leds`, plus `SuperStructure.java`, the orchestrator that sits above them. The gamepads (`pilot`, `operator`) live one level up under `src/main/java/frc/robot/`.

Anything with a `periodic()` lifecycle and state to manage gets the same shape.

Each subsystem is one file: the subsystem class (extending `Mechanism` for anything motor-backed), an inner `*Config` class holding every tunable as `@Getter private final` fields, and inner `WantedState`/`SystemState` enums driving its state machine. There is no separate `*States` command-factory class, the subsystem drives itself via `setWantedState(...)` + `handleStateTransition()` + `applyStates()`.

The full structural conventions live in [Class Generation](../coding-conventions/class-generation.md); don't reinvent the layout when adding a new subsystem.

### Turret zero at boot

Phoenix 6 seeds a Talon FX's position register from the rotor's **absolute** position at power-on,
not from zero ("The Talon FX and CANcoder sensors are always initialized to their absolute position
in Phoenix 6", CTRE's migration guide). One rotor turn is 360/39.78 = 9.05 deg of turret, so the
turret powers on reading somewhere in that 9 deg band depending on where the rotor magnet stopped --
never 0, and a power cycle does not clear it.

`Turret.seedFromZeroReference()` undoes that at code start: the rotor's absolute position is
repeatable for a given turret angle, so subtracting the measured `ZERO_REFERENCE_DEGREES` (5.9765625
deg on PM_2026, 2026-09-19) makes the parked zero read 0. The correction is only unique within one
rotor turn, so **park the turret within +/-4.5 deg of zero before a power cycle** (aim for 2) or it
snaps to the wrong turn. What it came up reading is logged as `Turret/BootPositionDegrees` and
printed.

The seed runs only after a real power cycle. A code restart or a roboRIO reboot leaves the Talon's
count intact, and re-seeding then would throw a good zero away. `decideBootZero` separates the two
cases, logged as `Turret/BootDecision`:

* A raw reading outside the band a power-on can produce (-4.5 to +9.05 deg) cannot be a power
  cycle: the count is **kept**.
* Inside that band, the reading is compared with the last position the previous code wrote to
  `/home/lvuser/turret-position.txt` (rewritten whenever the turret moves more than 0.5 deg and on
  every disable). A match within 0.25 deg is a code restart with the turret parked there: **kept**.
* Anything else is a power cycle: **seeded** from the reference. If the seeded angle lands within
  1 deg of the +/-4.5 wrap edge an alert says so, because the turret could equally have been
  parked on the other side and now reads 9 deg wrong (Chezy Q17 booted 0.3 deg from that edge).

Operator B while disabled still declares the current position zero and forces the file to be
rewritten. The one case this cannot tell apart is a power cycle whose rotor absolute lands within
0.25 deg of where the turret was left; parking on zero makes that rare.

That constant is a property of where the motor sits on the belt. It survives power cycles and
deploys but not a skipped tooth, a re-tension, or a motor swap -- re-measure it after any of those
by parking the turret at zero and reading `Position` off the Talon in Tuner X. Operator B while
disabled remains the manual override.

### Turret reading and output guards

The turret has no absolute reference, so a bad angle is indistinguishable from a real one until it
does damage. Two guards sit between the encoder and the motor, both in
[`Turret.java`](../../src/main/java/frc/robot/subsystems/turret/Turret.java):

* **Impossible-step guard.** A reported angle that moves further between two samples than the
  turret could have is held out, and the last good angle is what the aim, the soft-limit
  arithmetic, the shot gate, travel and Vision's zero chaser all see. The budget is 15 deg plus 1.5
  times the travel the velocity signal accounts for over the time that actually elapsed (clamped to
  20 to 250 ms), so a slow loop during a fast slew widens it rather than tripping it. Until Chezy
  Q50 (2026-09-19) the 15 deg stood alone and assumed a 20 ms loop; with the loop running 30 to 80
  ms and the turret at 1 rot/s it fired five times on real motion and "re-framed" the turret by 50
  to 108 deg each time. A step that repeats itself for 10 loops (200 ms) is believed after all --
  the encoder really has been re-framed -- and says so on the console and in an alert. Logged as
  `Turret/PositionSuspect`, `Turret/PositionStepBudgetDegrees`, `Turret/PositionStepsRejected`,
  `Turret/PositionStepsAccepted`.
* **Stall latch.** Pinned at 95% of the stator ceiling and not turning for 1 s cuts the turret's
  output and raises an alert. It is not stuck there: a command pointing to the other side of where
  it sits releases the latch and drives immediately, as do `OFF` and an operator-B re-zero. Logged
  as `Turret/StallLatched`, `Turret/StallLatchCount`.

A reported angle outside the configured travel (-216 to +180 deg) raises its own alert and
`Turret/AngleOutsideEnvelope`: the mechanism cannot be there, so the zero has slipped, the soft
limits are in the wrong frame, and every shot leaves by the same error. Re-zero before trusting one.

Both guards came out of the 2026-09-19 pit log, where the reported angle stepped 289 deg in one loop
and the turret then held 80 A stator against a hard stop for 6.8 s.

### Turret long moves: unwrap and set shots

The turret has 381 deg of travel, so for a 21 deg band of target angles two windings are reachable.
`resolveTurretAngle` picks the nearest one, and when the nearest is within 10 deg of a soft limit it
commits to the other winding (`unwrapping = true`) and slews a full turn under Motion Magic. Tracking
itself (`AIM_AT_TARGET`, `AIM_SWEEP`) is unprofiled `PositionVoltage`, so the Motion Magic numbers
(`mmCruiseVelocity` 1 rot/s, `mmAcceleration` 2 rot/s2) only ever apply to these long moves; the
knobs for how hard the turret accelerates while aiming are the 80 A stator limit and the 6 V ceiling.

Two things changed on 2026-09-19 after Chezy Q36, where the belt is believed to have slipped:

* **The unwrap latch only clears on arrival** (within `unwrapExitMargin`, 45 deg). It used to also
  clear the moment only one winding was reachable, which a moving target does within a few loops of
  the unwrap starting, and the rest of the turn then ran unprofiled: 21 of the 26 full-turn flips in
  Q36 went 650 to 700 deg/s into the far stop at 70 to 98 A stator (460 to 500 deg/s at 6 V in Q24).
* **Set shots resolve their winding and profile the long way.** `FIXED_ANGLE` used to command the
  raw -180 under `PositionVoltage`, which from +138 deg is 318 deg the long way instead of 42. It now
  goes through `resolveTurretAngle`, and a move over `longMoveDegrees` (90) is latched as an unwrap:
  Motion Magic, `Turret/ReadyToShoot` false until it arrives. Short moves keep `PositionVoltage`.

The peak voltage went 6 to 8 V for Q36 only and is back at 6. Full-turn flips are visible in a log
as a `Turret/CommandedDegrees` step of about 340 deg; `Turret/Unwrapping` should now stay true for
the whole of one.

### Shot map: the near-shot RPM drop

Practice-field shooting on the evening of 2026-09-19, at the fitted 365 RPM per m/s, had every hub
shot from about tower radius (3.2 m) inward landing long and the far shots landing. Taking 1.5 deg
of hood out fixed the near shots and dropped the far ones short: the model's hood is worth about
0.2 m per degree at 2 m and 0.4 m per degree at 3.5 m, so a flat hood trim can never fix one end
of the range without breaking the other. The team's choice is to leave the hood alone and take the
range out of the near shots with exit speed, which also keeps the near shot lower.

`ShotCalculator.nearShotRpmDrop` takes RPM off the hub-shot flywheel command by distance: a fixed
shape (`NEAR_SHOT_DROP_SHAPE`, 1.0 at 3.0 m and inside, zero from 3.75 m out, rising to 1.7 at
1.5 m because speed is a weak knob at an 84 deg launch) times one size, `ShotCalc/NearShotRpmDrop`
on the dashboard, 150 RPM at boot and clamped 0 to 400. It applies to tracked hub shots (at the
converged lookahead distance) and to set shots (at their fixed range), not to feed shots. The size
is a best guess converted from that 1.5 deg of hood and is expected to move by 50 to 100 RPM; zero
disables it. `ShotCalc/NearShotRpmDropApplied` logs the RPM actually removed on every launch loop.

The 390 RPM per m/s coupling drafted after Chezy Q45 was dropped without being deployed: it adds
range at every distance, most far out, which was the wrong shape for this problem.

### Dye rotor: feed auto-unjam

[`DyeRotor.java`](../../src/main/java/frc/robot/subsystems/dyeRotor/DyeRotor.java) reverses itself
when the feed jams. While `INDEX_MAX` is wanted, rotor stator current above
`DyeRotor/AutoUnjamAmps` (tunable, 55 A at boot) held without a break for 0.4 s puts the system
state in the existing `UNJAM` for 0.25 s (rotor -100 RPM, feeder -1000 RPM), then feeding resumes
and the timers start over. The first 0.4 s after feeding starts, or restarts after a reverse, is
not watched: the rotor spinning up against a packed bed draws jam-level current on its own. The
check lives in `handleStateTransition()`, so `applyStates()` and the rest of the robot see only the
resulting `UNJAM`; any wanted state other than `INDEX_MAX` clears it. Same software-only shape as
the idle stall check, so no current-limit config writes. Logged as `DyeRotor/AutoUnjamActive`,
`DyeRotor/AutoUnjamCount`.

## Per-Robot Configurations

We build multiple physical robots each season and run the same code on all of them. The `Rio.id` field, looked up from the RoboRIO serial number in `frc.spectrumLib.hardware.Rio`, decides which configuration is loaded at startup. All configs live under `src/main/java/frc/robot/configs`:

|    Config    |                 Bot                  |                                                                Use                                                                |
|--------------|--------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------|
| `FM2026`     | Final Machine, the competition robot | Precise calibration, the bot we travel with.                                                                                      |
| `PM2026`     | Practice Machine                     | Mirrors competition, minor wear-and-tear tweaks.                                                                                  |
| `XM2026`     | Experimental Machine                 | In-season experimentation and prototyping. Encoder offsets and attachment flags vary. (Off-season work gets its own `OM` config.) |
| `AM2026`     | Alpha Machine                        | Earlier prototype, used pre-build.                                                                                                |
| `PHOTON2026` | Photon's machine                     | The robot run by Photon, our sister team.                                                                                         |

Each config can mark a mechanism present or absent via `setAttached(boolean)` so a bot without the launcher doesn't try to initialize one.

## States and Triggers

Each subsystem exposes a `setWantedState(<Subsystem>.WantedState)` entry point and runs its own `WantedState`/`SystemState` machine internally. Triggers are conditions that fire commands, such as a `pilot.X` press, a sensor reading, or a `SpectrumState` another subsystem flipped.

The high-level orchestrator is [`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java). It maps the `WantedSuperState` enum (below) to a coordinated configuration across every mechanism. `setWantedSuperState(WantedSuperState)` is the entry point (with `setStateCommand(...)` as the command wrapper used by bindings); when `INTAKE_FUEL` fires, `SuperStructure` fans that intent out to each subsystem's `setWantedState(...)`: the fuel intake runs, the indexer bed slow-indexes, the extension extends, and the launcher/hood hold their prep/aim states.

## Pose Estimation

Swerve odometry and Limelight MegaTag readings feed a WPILib `SwerveDrivePoseEstimator`. The filtering, weighting, and which Limelight to trust live in [`Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java); read [Vision](../tools/vision.md) for the full integration scheme.

## 2026 Robot States

These are the entries in `SuperStructure.WantedSuperState`, applied by `setWantedSuperState(...)`. Each one drives a coordinated setup across launcher, hood, fuel intake, indexer bed/tower, and intake extension.

|                State                |                             What it does                             |
|-------------------------------------|----------------------------------------------------------------------|
| `IDLE`                              | Ready, neutral. Subsystems home.                                     |
| `INTAKE_FUEL`                       | Active fuel collection, intake runs, bed indexes, extension extends. |
| `TRACK_TARGET`                      | Launcher + hood aim while the robot is free to drive.                |
| `LAUNCH_WITH_SQUEEZE`               | Aim + launch with the delayed-close "squeeze" sequence.              |
| `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY` | Squeeze launch without the delayed close.                            |
| `LAUNCH_WITHOUT_SQUEEZE`            | Aim + launch while the intake stays extended.                        |
| `LAUNCH_WITH_BRAKE`                 | Launch while holding the drivetrain in brake.                        |
| `AUTON_TRACK_TARGET`                | Auton-mode aim.                                                      |
| `AUTON_INTAKE_FUEL`                 | Auton-mode fuel collection.                                          |
| `UNJAM`                             | Clear jammed fuel from intake or indexer.                            |
| `EJECT`                             | Spit fuel back out.                                                  |
| `FORCE_HOME`                        | Drive every mechanism to its home position.                          |
| `TEST_TURRET_FOLLOW_TAG`            | Test-mode pit check: turret follows any tag the turret camera sees.  |
| `TEST_TURRET_SWEEP`                 | Test-mode pit check: turret runs soft limit to soft limit and back.  |
| `TEST_TURRET_ZERO`                  | Test-mode pit check: turret returns to its zero.                     |
| `TEST_TURRET_STOP`                  | Test mode at rest: everything off, turret held where it stands.      |

`CurrentSuperState` mirrors these; `handleStateTransition()` maps the wanted state to the current one each loop.

A few field-location triggers live on `SuperStructure` itself rather than in the enum: `robotInNeutralZone()`, `robotInEnemyZone()`, `robotInFeedZone()`, `robotInScoreZone()` (which delegate to the swerve pose).

## Vision Hardware

Three Limelight 4s, back, left, right, for AprilTag-based pose estimation. `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)` is the seasonal map. Game-piece detection is not currently wired up; QuestNav is on the radar but unintegrated. Details in [Vision](../tools/vision.md).

## Controls Layout

The pilot drives and runs the fuel cycle; the operator handles offset trims and mechanism resets. Every binding lives in [`Robot.configureBindings()`](../../src/main/java/frc/robot/Robot.java); the `Pilot`/`Operator` classes just expose the button `Trigger`s (`LT`, `RT`, `XButton`, …). This section is the summary, that method is the truth.

### Pilot, fuel cycle (the triggers)

`LT` and `RT` drive the core intake/launch state machine:

* `LT` held alone, `INTAKE_FUEL`.
* `RT` held alone, `LAUNCH_WITH_SQUEEZE`.
* Both held, `LAUNCH_WITHOUT_SQUEEZE`.
* Release `LT` while `RT` is still held, `LAUNCH_WITH_SQUEEZE_WITH_NO_DELAY`.
* Release `RT` while `LT` is still held, back to `INTAKE_FUEL`.
* Release both, `IDLE`.

### Pilot, everything else

* Left stick: field-relative translation; right stick: rotation (exponential curves, deadzone in `Pilot`'s config).
* `X` (hold), `TRACK_TARGET` (launcher + hood aim while driving); release → `IDLE`.
* `A` (hold), `UNJAM`; release → `IDLE`.
* `LT + LB`: `EJECT`; release → `IDLE`.
* `Select`: `FORCE_HOME`; release → `IDLE`.
* `LB + Dpad` (up/left/down/right), reorient the robot heading forward/left/back/right.
* `LB + A` / `LB + Y` / `LB + X` / `LB + B` (hold), `SET_SHOT` from the tower / hub face / left
  trench / right trench. Fixed shots for when the pose is gone: no pose is read, the turret goes to
  the spot's angle, and the hood and flywheel come off the live shot map (hub model, near-shot RPM
  drop, hood trim) at the spot's range, so they move with the map instead of being retyped. Park
  with the **intake away from the hub** for the tower and both trenches (turret at zero, no half
  turn to wait for) and **intake against the hub** for the hub face (turret at -180). Ranges and
  the reference hood/RPM at each spot are in `ShotCalculator.SetShot`. Release → `IDLE`.
* While disabled: `A` → coast the intake extension and turret. (`brakeB` is declared in `Pilot.java` but bound to nothing, so `B` does nothing here.)

### Pilot, test mode (turret pit checks)

Three held-to-run turret checks on the bare D-pad, live only while the Driver Station is in Test. All three are pose-independent — no alliance, no tag map, no `ShotCalculator` — so they run on a cart. Nothing else on the robot moves: intake, rotor, tower and flywheel are off and the hood goes home.

* `Dpad Up` (hold), `TEST_TURRET_FOLLOW_TAG`: the turret points at whatever AprilTag the turret Limelight sees, closing the camera's own `tx` bearing. Checks that the camera, the turret zero and the gearbox agree on direction — confirmed working on `FRC_20260919_180624`, where `tx` converged to under 0.5°. With no tag in view it holds its last command. Clamped to the soft limits.
* `Dpad Left` (hold), `TEST_TURRET_SWEEP`: runs to one soft limit, then the other, and keeps going. The travel check — watch `Turret/PositionDegrees` at each end and `Turret/TravelTotalDeg` against `Vision/TurretZero/SlipDegPerKiloDegTravel` for belt slip. A leg that stalls or runs over 20 s turns around instead of pushing.
* `Dpad Down` (hold), `TEST_TURRET_ZERO`: back to zero, through the same output path the robot uses to sit at zero in a match.

Release any of them and the robot goes to `TEST_TURRET_STOP` — the turret stops where it stands, rather than falling back to `IDLE`, which would aim at the target. Entering test mode starts there too, and `testExit()` resets to `IDLE`.

Test mode is not a reduced mode: `robotPeriodic()` runs in every mode, so Vision, `SuperStructure`, the `CommandScheduler` and every subsystem `periodic()` — and with them all the DogLog keys — behave exactly as in teleop. Current limits, soft limits and the stall cut-out come from the motor config applied at construction and are never changed per mode. The one thing that would break that is WPILib enabling LiveWindow in test, which disables the `CommandScheduler`; it defaults off and nothing calls `enableLiveWindowInTest(true)`. Leave it that way.

Both moving checks drive the motor the same way `AIM_AT_TARGET` does — `commandPosition`, i.e. `PositionVoltage` in gain slot 0 at the ±6 V ceiling — not Motion Magic. That is deliberate and it was measured: both were written on Motion Magic first, and on `FRC_20260919_180624` (163.2–177.0 s) the follow check sat pinned at 89.7 °/s, exactly the then 0.25 rot/s `mmCruiseVelocity` (1 rot/s since 2026-09-19), drawing no more than 2.51 V of its 6 V, while `AIM_AT_TARGET` in the P8 match log runs p90 135 °/s, p99 385 °/s and uses the full 6.11 V. The profile was discarding more than half the authority the mechanism had. The one cost is that the sweep now crosses the travel at teleop speed and reaches its turnaround quickly; teleop's own full-travel move (the cable unwrap) is profiled precisely because that move is not a tracking move.

Note that the bare `A`, `B`, `X`, `Select` and trigger bindings have no mode gate, so they are also live in test mode. The test checks are on the D-pad precisely because nothing else claims it.

New log keys: `Turret/Test/FollowTagInView`, `Turret/Test/FollowTagTxDeg`, `Turret/Test/SweepTowardMax`.

### Operator

* `Dpad Down/Up`: range trim (via `ShotCalculator`): each press moves the hood 0.25° and the
  flywheel 2 % of model RPM together; up adds range (the shot fell short), down takes it away
  (the shot went long). The hood trim persists on the rio across power cycles. The flywheel trim
  is gone (Chezy Q11 only); a near-long, far-fine bias is the near-shot RPM drop's job, above.
* `Dpad Right/Left`: turret-angle offset trim (+/−1°, via `ShotCalculator`). Session-only since
  2026-09-19: it starts at zero every boot and is never stored.
* `Start + Select`: zero all three trims, including the stored copies.
* `X` (held): let vision trim and re-home the turret zero. Off unless held, since 2026-09-19; see [vision](../tools/vision.md#the-turret-camera-and-the-turret-zero). Released, the zero is whatever operator-B set and vision only logs what it would have changed.
* `Select`: `FORCE_HOME`; release → `IDLE`.
* `LB + Y`: reset the intake-extension position to max (with a rumble confirmation).
* While disabled: `A` → coast the intake extension and turret, `B` → declare the turret's current position its zero (`Turret.zeroTurretCommand()`), which also releases a turret stall latch.

### Hub shifts

REBUILT alternates each alliance's hub between active and inactive during teleop. `Robot.configureBindings()` calls [`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java)`::initialize` on teleop/auto/disable transitions so shift-aware logic knows where the match clock is.

## Where Robot State Lives

* [`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java): the `WantedSuperState`/`CurrentSuperState` enums and the `handleStateTransition()`/`applyStates()` logic that fans a super-state out across every mechanism. This is where a coordinated multi-mechanism move belongs.
* [`Robot.java`](../../src/main/java/frc/robot/Robot.java): `configureBindings()` wires gamepad triggers and `Auton` event triggers to `superStructure.setStateCommand(...)`. A single trigger that fires one existing state goes here.

If you're adding a behavior that's a coordinated multi-mechanism move, add a `WantedSuperState` and handle it in `SuperStructure`. If you're adding a single trigger that fires an existing state, just bind it in `Robot.configureBindings()`.
