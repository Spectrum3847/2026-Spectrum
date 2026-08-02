# AdvantageKit Output Logging

## What Is Output Logging?

Output logging records computed values (not raw sensor inputs) so they can be viewed in AdvantageScope or regenerated during replay. Unlike inputs — which are fixed sensor readings — outputs are recalculated each time code runs, enabling retroactive analysis.

```java
// In periodic() or commands:
Logger.recordOutput("Drive/LeftVelocityError", setpoint - inputs.leftVelocityRadPerSec);
Logger.recordOutput("Odometry/Robot", poseEstimator.getEstimatedPosition());
Logger.recordOutput("Superstructure/State", currentState.toString());
Logger.recordOutput("Mechanism2d/Arm", mechanism2d);
```

## Replay-Based Output Generation

The key power of AdvantageKit: **replay generates new output data from old logs without changing robot behavior.**

### How It Works

1. Robot runs on field → inputs (sensor data) are logged
2. After the match, add `Logger.recordOutput(...)` calls for values you wish you had logged
3. Replay the log through the updated code
4. Laptop re-runs the same code with identical inputs → new outputs appear in the replayed log

### Example

A robot logs odometry but not target distance. Post-match, add:

```java
Logger.recordOutput("TargetDistanceMeters",
    poseEstimator.getEstimatedPosition()
        .getTranslation()
        .getDistance(FieldConstants.hubCenter));
```

Replay produces `TargetDistanceMeters` for the full match — retroactively.

## Version Control Integration

AdvantageKit stores Git metadata (commit hash, branch, dirty state) in every log file. To replay accurately:

1. Check out the exact commit recorded in the log
2. Add new `Logger.recordOutput()` calls
3. Run replay

This guarantees the same logic that ran on the robot is used for replay, so odometry and other verified outputs still match — confirming the new outputs are trustworthy.

## Supported Types

All types support single values, 1D arrays, and 2D arrays unless noted.

| Category | Types | Notes |
| ---------- | ------- | ------- |
| Primitives | `boolean`, `int`, `long`, `float`, `double`, `String` | — |
| WPILib structs | `Translation2d/3d`, `Pose2d/3d`, `Rotation2d/3d`, `SwerveModuleState`, etc. | Preferred over protobuf — no delay |
| Protobuf | Any WPILib protobuf type | **⚠️ First log can take >100ms** — log once while disabled |
| Records | Custom `record` classes (fields: primitives, enums, structs, nested records) | **⚠️ Same first-log delay as protobuf** — log while disabled; no array fields |
| Enums | Any enum | Logged as `name()` string |
| Colors | WPILib `Color` | Logged as hex triplet string |
| Mechanisms | `LoggedMechanism2d` | Output only |
| Suppliers | `BooleanSupplier`, `IntSupplier`, `LongSupplier`, `DoubleSupplier` | Output only |

**Protobuf / Record first-use pattern** — call once during `disabledInit()` to absorb the delay:

```java
@Override
public void disabledInit() {
    Logger.recordOutput("Odometry/Robot", new Pose2d()); // warm up struct schema
}
```

## Syntax Variants

**Varargs for arrays** — pass multiple values without constructing an array:

```java
Logger.recordOutput("Drive/ModuleStates", stateFL, stateFR, stateBL, stateBR);
```

**Units metadata** — attach unit info for correct AdvantageScope visualization:

```java
Logger.recordOutput("Arm/AngleRad", angleRad, "radians");
Logger.recordOutput("Drive/SpeedMps", speed, MetersPerSecond);  // WPILib unit constant
Logger.recordOutput("Drive/SpeedMps", MetersPerSecond.of(speed)); // Measure type
```

## @AutoLogOutput Annotation

Annotate fields or methods to log them automatically each cycle instead of calling `Logger.recordOutput()` manually.

```java
public class Drive extends SubsystemBase {
    @AutoLogOutput
    private Pose2d pose = new Pose2d();                    // key: "Drive/Pose"

    @AutoLogOutput(key = "Drive/CustomKey")
    private double speed = 0.0;                            // explicit key

    @AutoLogOutput(unit = "radians")
    private double angleRad = 0.0;                         // with unit metadata
}

// Dynamic keys using field references (value freezes after first loop cycle):
public class SwerveModule {
    private final int index;

    @AutoLogOutput(key = "Drive/Module{index}/Speed")
    private double speed = 0.0;
}
```

Supports private fields and methods, all data types including arrays and structs.

### Configuration Requirements

Classes using `@AutoLogOutput` must be reachable from `Robot` via recursive field search within the first loop cycle. This covers most typical subsystem patterns.

For classes outside the standard scan (e.g., library code in `frc.lib` or standalone objects):

```java
// In robotInit(), before Logger.start():
AutoLogOutputManager.addPackage("frc.lib");       // scan entire package
AutoLogOutputManager.addObject(myExternalObject); // scan specific instance
```

Use `Logger.recordOutput()` directly for classes that cannot meet these requirements.

## Naming Conventions

Use `/`-separated hierarchical keys matching subsystem names for clean AdvantageScope organization:

```java
Logger.recordOutput("Drive/LeftVelocityRadPerSec", ...);
Logger.recordOutput("Drive/RightVelocityRadPerSec", ...);
Logger.recordOutput("Odometry/RobotPose", ...);
Logger.recordOutput("Vision/EstimatedPose", ...);
Logger.recordOutput("Arm/SetpointAngleRad", ...);
```

## What to Log

Prioritize values that are expensive to recompute or hard to infer post-match:

- Setpoints vs. measured values (control error)
- Pose estimates and vision corrections
- State machine states
- Calculated feed-forward / PID outputs
- Any value you've had to debug blind after a match
