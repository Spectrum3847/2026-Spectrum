# AdvantageKit Recording Inputs

## IO Interface Pattern (Detail)

The IO layer separates hardware interaction from control logic, enabling logging and replay.

### @AutoLog Annotation

Annotate the inputs inner class with `@AutoLog` — the annotation processor generates a `<ClassName>AutoLogged` class that implements `LoggableInputs` with automatic `toLog()` / `fromLog()` serialization.

**Always instantiate the generated `AutoLogged` class, not the original:**

```java
public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    default void updateInputs(FlywheelIOInputs inputs) {}
    default void setVoltage(double volts) {}
}

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    // ✓ Use the generated AutoLogged class
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }
}
```

### Supported Input Types

All [supported types](https://docs.advantagekit.org/data-flow/supported-types) are valid as input fields **except mechanism states**. Nested loggable inputs classes are permitted as fields.

### Units in Input Fields

Two approaches — pick one consistently:

**Naming convention** (simpler):
```java
public double velocityRadPerSec = 0.0;
public double positionMeters = 0.0;
```

**Measure objects** (type-safe):
```java
public Distance position = Meters.of(0.0);
public LinearVelocity velocity = MetersPerSecond.of(0.0);
```

`Measure` values in inputs are always logged in **base units** (meters for distance, regardless of the unit used in code), ensuring consistent replay across IO implementations.

### IO Implementation Structure

```
FlywheelIO          ← interface (defines contract)
FlywheelIOSparkMax  ← real hardware implementation
FlywheelIOSim       ← physics simulation implementation
```

In `RobotContainer`:

```java
Flywheel flywheel = Robot.isReal()
    ? new Flywheel(new FlywheelIOSparkMax())
    : new Flywheel(new FlywheelIOSim());
```

For replay (no hardware, no sim physics — inputs come from the log):

```java
new Flywheel(new FlywheelIO() {}) // anonymous no-op implementation
```

---

## Dashboard Inputs

Direct NetworkTables access (`SmartDashboard.getNumber()`, `SmartDashboard.getString()`, etc.) **will not work correctly in replay** — these values are non-deterministic and not logged.

### Logged Replacements

| Instead of | Use |
|-----------|-----|
| `SendableChooser<T>` | `LoggedDashboardChooser<T>` |
| `SmartDashboard.getNumber()` | `LoggedNetworkNumber` |
| `SmartDashboard.getString()` | `LoggedNetworkString` |
| `SmartDashboard.getBoolean()` | `LoggedNetworkBoolean` |

### Auto Chooser Example

```java
private final LoggedDashboardChooser<Command> autoChooser =
    new LoggedDashboardChooser<>("Auto Routine");

public RobotContainer() {
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Score And Drive", new ScoreAndDriveAuto());
}

public Command getAutonomousCommand() {
    return autoChooser.get();
}
```

### PathPlanner Integration

`LoggedDashboardChooser` accepts an existing `SendableChooser`, so it is compatible with `AutoBuilder.buildAutoChooser()`:

```java
private final LoggedDashboardChooser<Command> autoChooser =
    new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
```

### Coprocessor / Vision Data

NetworkTables data from coprocessors (e.g., PhotonVision, Limelight) must also go through an IO layer. Treat the coprocessor as a hardware device — read NT values inside `updateInputs()` so they are logged and replayable. See the AKit vision template project for a complete example.

### Tuning Values

Publish tuning values under the `/Tuning` NT path when using the logged network classes — AdvantageScope supports live tuning via this path.
