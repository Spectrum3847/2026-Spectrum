# CTRE Phoenix 6

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

Phoenix 6 is CTRE's API for the Kraken X60 / Falcon 500 (TalonFX), CANcoder, Pigeon 2, and CANdle. It's a clean rewrite of the older Phoenix v5 — same vendor, totally different classes. We're all-in on Phoenix 6; nothing in this repo still uses v5.

Pinned to 26.1.3 in [`vendordeps/Phoenix6-26.1.3.json`](../../vendordeps/Phoenix6-26.1.3.json).

## Where It Shows Up

Every powered mechanism in `frc.robot` runs on a TalonFX through Phoenix 6. The swerve drive is built on CTRE's swerve generator output (`SwerveDrivetrain`, `SwerveModule`, `SwerveRequest`) — see [`Swerve.java`](../../src/main/java/frc/robot/swerve/Swerve.java), which has a comment at the top pointing to the CTRE example it was forked from. The Pigeon 2 IMU is configured through `Pigeon2Configuration`, and CANcoders go through our `SpectrumCANcoder` wrapper.

The low-level signal reader behind every cached value in `Mechanism` is `BaseStatusSignal`. You'll occasionally see it imported directly — that's almost always a hot path that wanted to refresh several signals at once.

## TalonFX → `Mechanism`

Never `new TalonFX(...)` in subsystem code. Subclass [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java) and let `TalonFXFactory` build the motor from your `Config`:

```java
public class Launcher extends Mechanism {
    public Launcher(LauncherConfig config) {
        super(config);
        // motors, followers, and signals are wired up by the parent
    }
}
```

The parent class handles a lot for you. It configures the leader (and any followers) from your `Config` — current limits, neutral mode, gear ratio, inversion. It caches `positionRotations`, `velocityRPS`, `voltage`, and `statorCurrent` so reads in the same loop don't re-hit the bus. It exposes command factories for every control mode we use (`moveToRotations`, `moveToDegrees`, `moveToPercentage`, `runVelocityTcFocRPM`, `runVoltage`, `runPercentage`, …). And it gives you triggers like `atPosition`, `abovePosition`, and `atVelocity` so you can bind to a mechanism's state instead of polling it.

If you find yourself writing the same boilerplate twice, it probably belongs in `Mechanism`, not in your subsystem.

## Control Modes

Torque-current FOC is the default for anything that benefits — and most things do. The control requests `Mechanism` exposes are:

* `MotionMagicTorqueCurrentFOC` for profiled position moves (hood, indexers).
* `VelocityTorqueCurrentFOC` for closed-loop velocity (launcher wheels, fuel intake rollers).
* `VoltageOut` / `DutyCycleOut` as escape hatches when you specifically don't want FOC.
* `Follower`, which gets wired up automatically when your `Config` declares follower IDs.

PID gains live in `Slot0Configs` (with `Slot1` and `Slot2` available if you need multiple sets). `Mechanism.config.configPIDGains(kP, kI, kD)` configures Slot 0; use `Mechanism.config.configPIDGains(slot, kP, kI, kD)` to configure Slot 1/2. See [PID Tuning](../tools/pid-tuning.md) for the live-tuning workflow with `TuneValue`.

## CANcoders and the CAN Bus

CANcoders go through [`SpectrumCANcoder`](../../src/main/java/frc/spectrumLib/SpectrumCANcoder.java), configured by `SpectrumCANcoderConfig`. Absolute offsets belong in the relevant `*2026.java` config file, not in the mechanism — that way each physical robot can carry its own zero point. The "find the right offset" workflow lives in [Phoenix Tuner X](../tools/phoenix-tuner-x.md).

For routing: `Rio.CANIVORE` is the magic string `"*"` (use the first CANivore bus found), and `Rio.RIO_CANBUS` is the roboRIO's built-in bus. Specify devices with `CanDeviceId(id, bus)` from `frc.spectrumLib.util` — don't hand-roll CAN IDs in subsystem code. And keep in mind Phoenix licensing is per device, per season; if a Talon stubbornly refuses to FOC, check Phoenix Tuner X first.

## Status Signals

Phoenix surfaces data via `StatusSignal<Double>`. Reading a signal doesn't hit the bus — the bus is updated in the background, and `.getValue()` just returns the latest cached sample. The catch is that you need to call `refresh` to update the cache, and you should do it *once* per loop: `Mechanism.refreshSignals()` batches the calls via `BaseStatusSignal.refreshAll(...)` inside `periodic()`. Calling it again from the same loop is a small but real waste.

For one-off reads outside the periodic loop (say, during init), `signal.refresh().getValue()` is fine.

## Gotchas

`StatusCode` is *advisory*. Configuring a device returns one, but Phoenix doesn't throw if it's an error — you have to check `isError()` yourself. Log failed config attempts with `Telemetry.print` and an alert so they don't slip past.

`optimizeBusUtilization()` matters. Phoenix defaults to publishing every signal at a reasonable rate; on a busy CAN bus, the unused ones still add up. `Mechanism` calls it for you — keep that.

The CANdle imports in [`leds/CANdleLeds.java`](../../src/main/java/frc/robot/leds/CANdleLeds.java) are currently commented out while we sort out the WPILib `AddressableLED` migration (see [LEDs](../tools/leds.md)). When you reintroduce CANdle, paste the imports back from the comments — don't reach for the Phoenix v5 class with the same name.

PID gain slots are independent. `configPIDGains(kP, kI, kD)` configures Slot 0 only; configure Slot 1/2 explicitly if you use them.

## Further Reading

The [Phoenix 6 JavaDoc](https://api.ctr-electronics.com/phoenix6/latest/java/) is linked into our own generated docs. CTRE's [Phoenix 6 online docs](https://v6.docs.ctr-electronics.com/) cover FOC, control requests, and signal rates at a concept level. If you're tracing through `Swerve.java` and wondering where a piece of code came from, the [Phoenix-Examples](https://github.com/CrossTheRoadElec/Phoenix6-Examples) repo is the upstream we forked.
