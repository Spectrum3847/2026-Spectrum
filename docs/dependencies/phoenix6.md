# CTRE Phoenix 6

*Audience: Reference. Assumes you've read [Dependencies overview](overview.md).*

Phoenix 6 is CTRE's API for TalonFX, CANcoder, Pigeon 2, and CANdle. It is a clean break from v5: same vendor, different classes. Nothing in this repo uses v5.

`vendordeps/` holds the pinned JSON. We do not keep a version number anywhere else.

The swerve drive is built on CTRE's own `SwerveDrivetrain` and `SwerveModule` classes with `SwerveRequest` control requests, and the Pigeon 2 IMU is configured through `Pigeon2Configuration`. Both live in [`SwerveConfig.java`](../../src/main/java/frc/robot/subsystems/swerve/SwerveConfig.java).

## Every mechanism goes through Mechanism

Never `new TalonFX(...)` in subsystem code. Extend [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java) and hand it a `Config`; [`TalonFXFactory`](../../src/main/java/frc/spectrumLib/hardware/TalonFXFactory.java) builds the Talon and any followers from that config. What the base class gives you:

* The leader and follower Talons, created and configured from your `Config`.
* Cached reads. `getPositionRotations()`, `getVelocityRPM()`, `getVoltage()`, and `getStatorCurrent()` each compute at most once per scheduler loop, so repeating a read inside one loop does not re hit the bus.
* Command factories for every control mode we use, plus `Trigger`s such as `atRotations`, `aboveVelocityRPM`, and `atCurrent`, so you can bind to a mechanism's state instead of polling it.

Read `Mechanism.java` before adding anything to it. If you are about to write the same boilerplate in a second subsystem, it belongs in the base class.

## Control requests

Torque current FOC is the default for anything that benefits, and most things do. Profiled position moves and closed loop speed moves are the two cases worth knowing by heart; `VoltageOut` and `DutyCycleOut` are escape hatches for when you specifically do not want FOC. Which request a given factory sends is decided inside `Mechanism`, not at the call site, so read it there rather than guessing from a subsystem.

PID gains live in slot configs. `Mechanism.config.configPIDGains(kP, kI, kD)` writes slot 0 and `configPIDGains(slot, kP, kI, kD)` writes slot 1 or 2. The slots are independent, so gains you set on slot 0 do nothing when the active request is on slot 1. See [PID tuning](../tools/pid-tuning.md) for the live tuning workflow with `TuneValue`.

## Encoders and the CAN bus

The swerve uses Phoenix `CANcoder` and `CANcoderConfiguration` directly. `frc.spectrumLib.hardware.SpectrumCANcoder` exists in the library, but no subsystem uses it, so do not go hunting for a wrapper call when you wire an encoder.

Encoder offsets are per physical robot, not per season. Each `*2026.java` config in [`src/main/java/frc/robot/configs/`](../../src/main/java/frc/robot/configs/) calls `swerve.configEncoderOffsets(...)`, and `Rio` picks which config class to build from the roboRIO serial. The workflow for finding a new offset is in [Phoenix Tuner X](../tools/phoenix-tuner-x.md).

`Rio.CANIVORE` is the string `"*"`, which asks Phoenix for the first CANivore bus it finds, and `Rio.RIO_CANBUS` is the roboRIO's built in bus. Pass devices as `CanDeviceId` from `frc.spectrumLib.util` rather than hand rolling an id and bus pair inside a subsystem. That class is ported from 254's 2023 code.

## Status signals

Reading a `StatusSignal` does not hit the bus. Phoenix refreshes signals in the background and `getValue()` returns the last sample. `Mechanism` refreshes all of its signals with one `BaseStatusSignal.refreshAll` per loop, gated on `RobotLoop.count()`, so repeated getter calls inside a loop are free. Its constructor sets each signal's update rate and then calls `optimizeBusUtilization()`: position and velocity at 100 Hz, output signals at 50 Hz on a leader with followers and 20 Hz otherwise (100 Hz with `fastOutputLogging`), currents at 20 Hz, temperature at 4 Hz, and every follower signal at 20 Hz. For a one off read outside the loop, such as during init, `signal.refresh().getValue()` is fine.

Config calls go through `CanConfigBudget`. Once failed config calls have used 3 seconds in total, later calls get one attempt each and an alert is raised, so a dead CAN bus cannot stretch boot past a minute.

## Gotchas

`StatusCode` is advisory. A config call returns one and Phoenix does not throw when it is an error, so you have to check `isError()` yourself. A device that silently refused its config looks exactly like a device that took it, right up until the gains are wrong at an event. Log failed config attempts with `Telemetry.print` and raise an `Alert`.

Phoenix licensing does not carry over between seasons. A Talon that refuses to FOC, or reports a licensing fault on boot, is often just unlicensed. Check Phoenix Tuner X before you start chasing gains.

`optimizeBusUtilization()` is not optional. Without it Phoenix publishes every signal at its default rate, and a signal nobody reads still costs bus time on a busy robot. `Mechanism` calls it; keep it that way.

The Phoenix 5 and Phoenix 6 CANdle classes share a name, so the import is the tell for which is which. The LED strip runs on a Phoenix 6 CANdle through [`Leds.java`](../../src/main/java/frc/robot/subsystems/leds/Leds.java); see [LEDs](../tools/leds.md).

## Further reading

The [Phoenix 6 JavaDoc](https://api.ctr-electronics.com/phoenix6/latest/java/) is linked into our generated docs, and [Phoenix 6 online docs](https://v6.docs.ctr-electronics.com/) cover FOC, control requests, and signal rates at a concept level.
