# AdvantageKit Common Issues

## Non-Deterministic Data Sources

Replay works by re-running identical code with identical logged inputs. Any data source that isn't logged as an input will produce different values during replay, breaking log comparison.

### Problem Sources

| Source | Problem | Fix |
| ------ | ------- | --- |
| `Timer.getFPGATimestamp()` | Raw FPGA time — not deterministic | Use `Timer.getTimestamp()` |
| NetworkTables / dashboard inputs | Values change outside the logging cycle | Log as inputs through IO layer |
| YAGSL, Phoenix 6 swerve libraries | Bypass IO abstraction, call hardware directly | Use AKit's swerve template instead |
| `Math.random()` / random number generators | Non-deterministic by nature | Log seed or generated values as inputs |
| Iterating over `HashMap` / unordered collections | Iteration order varies across JVM runs | Use `LinkedHashMap` or sorted collections |
| Filesystem reads | File contents may change | Log file data as inputs |
| Driver Station data accessed before `Logger.start()` | Not yet deterministic | Defer all DS access until after `Logger.start()` |

---

## Multithreading

Robot code logic must be single-threaded for reliable replay. Thread timing varies between hardware and simulation runs, so multi-threaded execution cannot be deterministically reproduced.

### Guidelines

**Before adding a thread, consider alternatives:**

- 50 Hz main loop is sufficient for most control needs
- Use motor controller closed-loop features (PID on the controller) rather than a separate thread

**If a thread is truly necessary:**

- Restrict it entirely to the IO implementation layer
- The thread may collect sensor data internally and expose it through `updateInputs()` — this syncs to the main loop
- Never call `Logger.recordOutput()` or `Logger.processInputs()` from a background thread

### What Not To Do

```java
// BAD: Logging from a background thread
new Thread(() -> {
    while (true) {
        Logger.recordOutput("Sensor/Value", sensor.get()); // NOT thread-safe
    }
}).start();

// GOOD: Thread stays inside IO, values sync through inputs
public class SensorIOReal implements SensorIO {
    private volatile double cachedValue = 0.0;

    public SensorIOReal() {
        new Thread(() -> {
            while (true) {
                cachedValue = sensor.get(); // thread-safe field update
            }
        }).start();
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.value = cachedValue; // read on main thread, logged safely
    }
}
```

---

## Uninitialized Inputs

### Logger.start() Must Come First

AdvantageKit does not log anything until `Logger.start()` is called. Driver Station data and FPGA timestamps accessed before this point are non-deterministic and must not be used by robot logic.

**Correct initialization order:**

```java
public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start(); // must be before any subsystem construction

        robotContainer = new RobotContainer(); // safe — Logger is running
    }
}
```

**Anti-pattern to avoid:**

```java
public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer = new RobotContainer(); // BAD: constructed before Logger.start()
    ...
}
```

### Subsystem Constructors Cannot Read Inputs

`io.updateInputs(inputs)` is only called during `periodic()`. Inputs are zero/default until the first loop cycle runs.

```java
// BAD: reading inputs in constructor
public class Arm extends SubsystemBase {
    public Arm(ArmIO io) {
        io.updateInputs(inputs); // called once but not logged; stale next cycle
        Logger.processInputs("Arm", inputs);
        double initialAngle = inputs.angleRad; // unreliable
    }
}

// GOOD: defer input-dependent logic to first periodic() or an explicit init call
public class Arm extends SubsystemBase {
    private boolean initialized = false;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        if (!initialized) {
            // safe to use inputs.angleRad here
            initialized = true;
        }
    }
}
```
