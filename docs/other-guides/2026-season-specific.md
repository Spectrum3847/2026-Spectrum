# 2026 season specific: REBUILT

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

Orientation for the 2026 game. It tells you where things live and which file owns which decision, and it points at the code for everything else. It deliberately does not list the states, the controls, or the per-robot configs, because those live in source and a copy here would be a copy that lies to you within a week.

## Where things live

Under `src/main/java/frc/robot/`:

* `subsystems/` holds one folder per mechanism, each with a single file that carries the mechanism, its inner `Config` class, and its inner `WantedState` and `SystemState` enums. The layout and the reasoning behind it are in [Class Generation](../coding-conventions/class-generation.md).
* `subsystems/SuperStructure.java` is the orchestrator, and the one registered `SubsystemBase` among the robot's own classes, so it is what the scheduler ticks. The mechanisms below it are plain `Subsystem` objects, not `SubsystemBase`, so they never register themselves.
* `pilot/` and `operator/` hold the gamepad classes. They expose `Trigger`s and nothing else; they hold no behavior.
* `configs/` holds the per-robot config classes.
* `auton/` holds the auto chooser and the named commands the paths call back into.
* `Robot.java` constructs everything, owns the hardware CAN bus, and holds the binding wiring.
* `RobotSim.java` and the classes under `src/main/java/frc/rebuilt/` are game math and simulation support. `ShotCalculator` and `Field` are called on the real robot, so treat those as robot code. `FuelPhysicsSim` is simulation only and is reached through `RobotSim`.

Under `src/main/java/frc/spectrumLib/` is the reusable layer: the `mechanism` base class, gamepads, hardware wrappers, telemetry, LEDs, swerve, vision, and sim configs. If something there changes, it changes for every robot, so treat it with more care than a per-robot config.

## Where a behavior belongs

This is the judgment call the code cannot teach:

* A coordinated move across several mechanisms is a new entry in the `WantedSuperState` enum in [`SuperStructure.java`](../../src/main/java/frc/robot/subsystems/SuperStructure.java), handled in its `handleStateTransition()` and applied in `applyStates()`. A new super-state is the answer whenever two or more mechanisms have to change together.
* A single trigger that fires a state that already exists is a binding in [`Robot.configureBindings()`](../../src/main/java/frc/robot/Robot.java), calling `superStructure.setStateCommand(...)`.

Where to read the current answers:

* **The list of robot-level states** is the `WantedSuperState` enum in `SuperStructure.java`. The transition table is that class's `handleStateTransition()`, and the fan-out to each mechanism is in the `apply*` methods it dispatches to. Field-location triggers such as `robotInNeutralZone()` are methods on the same class.
* **Which button fires which state** is in `Robot.configureBindings()`, and only there. `Pilot` and `Operator` name the triggers and hold no behavior.

## Per-robot configs

The same code runs on several physical robots. At startup the `Robot()` constructor switches on `Rio.id` to pick a config class; `Rio.id` resolves the RoboRIO serial number against the `Rio` enum in [`Rio.java`](../../src/main/java/frc/spectrumLib/hardware/Rio.java).

Two gotchas a reader will not guess:

* **A serial that matches no entry resolves to `Rio.UNKNOWN`,** and `Robot()` falls back to the competition config. So a reflashed or replaced RoboRIO looks exactly like a working robot right up until you deploy to it. If a bot behaves like the wrong robot, check `Rio` first.
* **Not every config class in `configs/` is wired into that switch.** A class can exist for a robot you are no longer running. Grep `Robot.java` for the config you are about to edit, and if it isn't in the switch, editing it changes nothing at runtime.

Every config class marks each mechanism present or absent with `setAttached(boolean)`. The mechanism object is still constructed on every robot; the flag is what stops it creating motors and running per-loop work for hardware that is not there, and what makes the sensor getters return zero instead of reading a device that does not exist. Encoder offsets and CAN IDs are per-robot facts. Changing one in the wrong config class silently breaks a different robot and will not fail to compile.

## Vision

Three Limelights, back, left, and right, doing AprilTag pose estimation. [`Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) decides which candidate estimate to trust and builds a `VisionFieldPoseEstimate` (pose, capture timestamp, and per-axis standard deviations), then hands it to the swerve's `addVisionMeasurement(...)`. The tag layout it loads is the seasonal field map, so it changes with the game each year. Game-piece detection and QuestNav are not integrated, so tag tracking is all that runs today.

Which estimate gets used, and when, is the part worth reading before you change anything, and it is in `Vision.java`. The fusion scheme, the ambiguity rejection, and resetting pose from vision are covered in [Vision](../tools/vision.md).

## Hub shifts

REBUILT alternates each alliance's hub between active and inactive during teleop. This is a competition rule, so the code implements it but cannot state it. The schedule lives in [`ShiftHelpers.java`](../../src/main/java/frc/rebuilt/ShiftHelpers.java), as two parallel arrays of shift start and end times, one active and one inactive, with the rotation chosen by which alliance is active first. `autoEndTime` in that same class is the length of the autonomous period, another rule the code encodes rather than states.

`Robot.configureBindings()` calls `ShiftHelpers.initialize()` on the teleop, auto, and disabled transitions, which is what keeps the shift clock lined up with the match. That only works because the timer restarts from zero on each of those edges; if a shift-aware behavior reads wrong, check that the clock was reset before you look at the mechanism.

Which alliance is active first comes from the FMS game message rather than from alliance color. That is an FMS convention, and it is why the schedule can be wrong in a practice session with no FMS: the fallback assumes the opponent is active first.
