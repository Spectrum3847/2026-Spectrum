# PathPlannerLib

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

PathPlanner is two pieces: a desktop editor that produces `.path` and `.auto` files, and an on-robot library that reads them and drives the swerve through them. We use both for every autonomous routine.

Version 2026.1.2, pinned in [`vendordeps/PathplannerLib-2026.1.2.json`](../../vendordeps/PathplannerLib-2026.1.2.json).

## What's in the Repo

The path and auto files live in [`src/main/deploy/pathplanner/`](../../src/main/deploy/pathplanner/): `paths/` for individual trajectories, `autos/` for sequences, plus `settings.json` and `navgrid.json` for the editor. They deploy to the roboRIO under `/home/lvuser/deploy/pathplanner/` automatically.

The Java side has three entry points:

* `Swerve.configurePathPlanner()` registers the drivetrain with `AutoBuilder` at boot.
* `frc.robot.auton.Auton` defines the auto routines, the event triggers, and the `SendableChooser` exposed to the driver station.
* `Robot.robotInit` runs the PathPlanner warmups so the first auto doesn't hitch.

## AutoBuilder Wiring

`Swerve.configurePathPlanner()` is where PathPlanner meets the swerve drive:

```java
AutoBuilder.configure(
        this::getRobotPose,
        this::resetPose,
        this::getCurrentRobotChassisSpeeds,
        (speeds, feedforwards) -> setControl(
                AutoRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())),
        new PPHolonomicDriveController(
                new PIDConstants(4, 0, 0),  // translation
                new PIDConstants(3, 0, 0)), // rotation
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
```

Those PID constants get re-tuned every year. Don't change them without coordinating with whoever owns auto tuning — small shifts have outsized effects on whether a path lands on its end pose.

## Event Triggers

Path event markers fire `EventTrigger`s. They're declared once at the top of `Auton.java`:

```java
public static final EventTrigger autonIntake     = new EventTrigger("intake");
public static final EventTrigger autonShotPrep   = new EventTrigger("shotPrep");
public static final EventTrigger autonShoot      = new EventTrigger("shoot");
public static final EventTrigger autonClearState = new EventTrigger("clearState");
public static final EventTrigger autonUnjam      = new EventTrigger("unjam");
public static final EventTrigger autonPoseUpdate = new EventTrigger("poseUpdate");
```

…and then bound to robot states in `RobotStates.setupStates()`:

```java
Auton.autonIntake.onTrue(applyState(State.INTAKE_FUEL));
Auton.autonShotPrep.onTrue(applyState(State.TRACK_TARGET_WITH_NO_SWERVE));
Auton.autonShoot.onTrue(applyState(State.LAUNCH_WITH_SQUEEZE));
```

Adding a new auto step is a three-step recipe: drop the event marker in the editor, add a matching `EventTrigger` constant in `Auton.java`, and bind it to a `State` (or whatever command you want) in `RobotStates`. The marker name and the string passed to `new EventTrigger(...)` have to match exactly — if your trigger isn't firing, that's the first thing to double-check.

## The Auto Chooser

`Auton.setupSelectors()` builds a `SendableChooser<Command>` and publishes it to SmartDashboard so it surfaces on Elastic's Pre-Match tab:

```java
pathChooser.addOption("TBTB Left",  TBTB(false));
pathChooser.addOption("TBTB Right", TBTB(true));
// ...
SmartDashboard.putData("Auto Chooser", pathChooser);
```

Each option ultimately returns `new PathPlannerAuto(autoName, mirrored)`. The `mirrored` flag is how a single `.auto` file becomes both the "Left" and "Right" variants — `PathPlannerPath.mirrorPath()` mirrors across the center line of *the same* alliance, while `PathPlannerPath.flipPath()` flips for red versus blue (which the alliance lambda in `AutoBuilder` does for you inside an `.auto`).

If you're loading a path directly from Java instead of through an `.auto`, the alliance flip is *not* automatic. You're on the hook for `flipPath()` and `mirrorPath()` yourself.

## Warmup

`Robot.robotInit` calls `FollowPathCommand.warmupCommand()` and `PathfindingCommand.warmupCommand()` so the JIT has compiled the hot paths before the first auto runs. If you add new path-loading code that's only used in matches, schedule a warmup at boot for it too — `PathPlannerPath.fromPathFile(...)` is heavy on the first call.

## Loading a Path From Java

For the occasional one-off (a recovery routine, a fallback after a failed pose update), `PathPlannerPath.fromPathFile("MyPath")` is what you want. It throws `IOException`, `ParseException`, and `FileVersionException` — catch them all and report through `DriverStation.reportError` and `Telemetry.print` so a missing or malformed file doesn't silently kill the auto.

## Gotchas

The editor's `settings.json` shadows the robot constants. If your generated trajectories assume one robot mass and the real swerve assumes another, paths won't track. Keep them aligned.

`navgrid.json` is editor-only. It deploys, but nothing reads it on the robot. Keep it under source control so the editor opens cleanly for everyone.

Event marker names with trailing whitespace are a recurring footgun. They look identical in the editor and don't match in Java.

## Further Reading

[PathPlanner Documentation](https://pathplanner.dev/home.html) covers the editor and on-robot library at a concept level. The [JavaDoc](https://pathplanner.dev/api/java/) is linked into our generated docs. For our higher-level "how an auto runs end-to-end" view, see [Auton](../tools/auton.md).
