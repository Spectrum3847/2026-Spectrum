# 2026 season specific: REBUILT

*Audience: Reference for team students and mentors. Assumes you've read [Setup](../setup.md).*

This page is for the things a single file cannot tell you: where a decision belongs, and the few
operating rules the code cannot state for itself. For what any class, method, or state actually
does, read the class.

## Where things live

Every pointer below is a place to start reading, not a summary of what you will find there.

* **The orchestrator** is [`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java).
  Its `WantedSuperState` enum is the list of coordinated robot behaviors, and its
  `handleStateTransitions()` and `applyStates()` are where one wanted state becomes a setting on
  every mechanism. Start here to understand any multi-mechanism move.
* **Each mechanism** is one folder under
  [`subsystems/`](../../src/main/java/frc/robot/subsystems) holding one file: the mechanism, an inner
  `*Config` class with its tunables, and inner `WantedState` and `SystemState` enums. The mechanism
  drives itself, so a caller sets a wanted state and the mechanism decides what that means. See
  [Class Generation](../coding-conventions/class-generation.md) before adding one.
* **Per-robot hardware config** lives in
  [`configs/`](../../src/main/java/frc/robot/configs), never in a subsystem file. Each class covers
  one physical robot. `Robot.java` picks the class from the roboRIO serial at startup, and
  `frc.spectrumLib.hardware.Rio` holds those serials. Read the `switch` in `Robot.java` to see
  which one this branch actually runs.
* **Bindings** are all in `Robot.configureBindings()` in
  [`Robot.java`](../../src/main/java/frc/robot/Robot.java). The `Pilot` and `Operator` classes
  expose the buttons and nothing else.
* **Game-specific helpers** for 2026 are in
  [`src/main/java/frc/rebuilt`](../../src/main/java/frc/rebuilt): the field, the shot model, shift
  timing, and the physics simulation.
* **Vision** is [`Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java), which
  owns the three cameras and fuses their estimates into the pose estimator. The seasonal AprilTag
  map is chosen in [`Field.java`](../../src/main/java/frc/rebuilt/Field.java), not in the vision
  subsystem. Read [Vision](../tools/vision.md) for the integration scheme.
* **Autos** are registered in
  [`Auton.java`](../../src/main/java/frc/robot/auton/Auton.java), and the paths and auto files live
  under [`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner). See
  [Auton](../tools/auton.md).

## Where a new behavior goes

There is one rule, and it is the only thing on this page you have to memorize.

* If the behavior is a **coordinated move across several mechanisms**, add a value to
  `WantedSuperState` and handle it in `SuperStructure`. That is the only place that knows about more
  than one mechanism at a time.
* If the behavior is **one input firing a state that already exists**, bind it in
  `Robot.configureBindings()` and change nothing else.

A mechanism that needs to notice something on its own, such as a jam or a stall, belongs inside that
mechanism, not in the orchestrator. The orchestrator is for moves a human asked for.

## Operating rules you cannot read off the code

**Hub shifts.** REBUILT alternates each alliance's hub between active and inactive during teleop, so
"which hub is live" is a game rule, not a robot behavior.
[`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java) tracks the match clock, and
`Robot.configureBindings()` re-initializes it on every teleop, autonomous, and disabled transition.
Any shift-aware logic depends on that being called; do not add shift logic that reads the clock
without it.

**Test mode is not a reduced mode.** `robotPeriodic()` runs in every mode, so vision, the
orchestrator, the scheduler, and every mechanism's `periodic()` behave in test exactly as they do in
teleop. Nothing is quieter in test. The one thing that would break this is WPILib enabling LiveWindow
in test, which disables the command scheduler. It defaults off and nothing turns it on here. Leave it
that way.

**The turret zero depends on a physical pin.** The turret has no absolute position reference, so the
code recovers its zero at boot from the motor's rotor angle. That is only correct if the turret was
pinned at its zero before the power cycle. A code restart keeps the count the code already had, so
the seed runs on a real power cycle only. The consequences, the alerts, and the manual override are
all documented on `Turret.seedFromZeroReference()`. The one-line version for the pit: if the
turret was not pinned, re-zero by hand while disabled.

**An angle outside the turret's travel is not a reading, it is a fault.** The mechanism cannot
physically be there, so a reported angle out there means the zero has slipped. Re-zero before you
trust any shot taken since.

## Shot map: the near-shot RPM drop

Practice shooting on 2026-09-19 found that shots close to the hub landed long while far shots landed
correct. Correcting it with the hood was a dead end: the hood's worth in range grows with distance,
so a flat trim fixes one end of the range by breaking the other. The team's answer was to leave the
hood alone and take the range out of the near shots with exit speed instead, which also keeps those
shots flatter.

That correction is `ShotCalculator.nearShotRpmDrop`, and the dashboard knob that sets its size is
the `ShotCalc/NearShotRpmDrop` tunable, logged as `ShotCalc/NearShotRpmDropApplied` on every launch
loop. It applies to tracked hub shots and to set shots, not to feed shots. Zero disables it.

The measured reasoning, the numbers, and what was tried before this are in
[Tuning and Calibration Handoff 2026-09-08](tuning-calibration-handoff-2026-09-08.md). The shape of
the data it produces is in [Shot Records and Trim Events](../tools/shot-log.md).

## Dye rotor: feed auto-unjam

A jammed feed is detected in software, from rotor stator current held above a threshold for a
debounce period. The one thing that is not a jam is the rotor spinning up against a packed bed, so
the check stays disarmed for a moment after feeding starts or restarts, and every timer restarts
after a reverse. It is the same shape as the idle stall check, so it needs no current-limit config
writes.

The logic is in `DyeRotor.indexMaxWithAutoUnjam()` and is reached from `handleStateTransition()`, so
`applyStates()` and the rest of the robot only ever see the resulting state. The thresholds are
DogLog tunables under `DyeRotor/`, and the state is logged as `DyeRotor/AutoUnjamActive` and
`DyeRotor/AutoUnjamCount`.
