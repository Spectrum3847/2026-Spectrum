# CTRE Phoenix 6

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

Phoenix 6 is CTRE's API for the TalonFX, CANcoder, Pigeon 2, and CANdle. This repo is Phoenix 6 only. Phoenix v5 is a clean break with different class names, and nothing here uses it. The version is pinned in `vendordeps/`.

## Subsystems do not construct motors

Never call `new TalonFX(...)` in subsystem code. Subclass [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java) and let it build the motor from your `Config` through [`TalonFXFactory`](../../src/main/java/frc/spectrumLib/hardware/TalonFXFactory.java), which supplies the team defaults for neutral mode, inversion, deadband, and current limit. A subsystem constructor should be wiring its own extra sensors and triggers, and nothing else.

If you find yourself writing the same motor boilerplate twice, it belongs in `Mechanism` rather than in your subsystem. That is how the swerve, the launcher tower, and the rest of the drivetrain all ended up sharing one set of cached reads and one set of command factories.

The swerve drive is the exception that proves the rule: it is built on CTRE's own `SwerveDrivetrain` generator output rather than on `Mechanism`, and [`Swerve.java`](../../src/main/java/frc/robot/subsystems/swerve/Swerve.java) carries a header comment pointing at the upstream [Phoenix6-Examples](https://github.com/CrossTheRoadElec/Phoenix6-Examples) file it was forked from. Follow that link before changing it, so the diff stays comparable to upstream.

## CAN routing

Devices are named with `CanDeviceId` from `frc.spectrumLib.util`, never with a hand-rolled CAN ID string. For the bus, use the constants in [`Rio`](../../src/main/java/frc/spectrumLib/hardware/Rio.java): `Rio.CANIVORE` is the wildcard that picks the first CANivore bus found, and `Rio.RIO_CANBUS` is the roboRIO's built-in bus.

CANcoder offsets belong in the per-robot `*2026.java` config file, not in the mechanism. That way each physical robot carries its own zero point, and the swerve alignment tool has one place to write. See [Phoenix Tuner X](../tools/phoenix-tuner-x.md) for finding the offset and [Swerve Alignment](../tools/swerve-alignment.md) for writing it into the code.

## StatusCode is advisory

This is the single most important thing to know about the API. A Phoenix configuration call returns a `StatusCode` and does not throw when the device is missing or the bus is dead. Nothing in our code will notice on your behalf. Check the result yourself, and report a failure through `Telemetry` and an `Alert` so it does not slip past. [`CanConfigBudget`](../../src/main/java/frc/spectrumLib/hardware/CanConfigBudget.java) is where we do that, and its class comment explains the failure it was written for.

The same advisory rule applies to `optimizeBusUtilization()` and the per-signal rate calls. They are never worth the boot time on a bus that is not answering, which is why `Mechanism` and `Swerve` both check `CanConfigBudget.exhausted()` before spending them.

## Signal reads in a hot loop

Reading a signal does not necessarily hit the CAN bus, and this is where the default hurts. A Phoenix getter such as `getStatorCurrent()` refreshes its own signal on every call, and each refresh is a JNI call. `Mechanism` avoids that by batching every signal the mechanism needs into one `BaseStatusSignal.refreshAll` per scheduler loop, so several reads in one loop see the same sample for the price of one.

The consequence for subsystem code: do not call a Phoenix getter on a hot path and assume it is free. If you need a value that is genuinely fresh outside the loop, such as during init, one `refresh()` is fine.

## Gain slots are independent

`configPIDGains(kP, kI, kD)` writes slot 0 and nothing else. The slot-taking overload writes the slot you name. Slots 1 and 2 do not inherit from slot 0, so a mechanism that expects a second set of gains has to configure it explicitly. See [PID Tuning](../tools/pid-tuning.md) for the live-tuning workflow.

## Licensing is per device and per season

Phoenix licensing does not carry over between seasons, and it is per device. If a Talon refuses to run FOC or Tuner X will not claim it, check the license in Tuner X before you go looking at the code.

## Further reading

The [Phoenix 6 JavaDoc](https://api.ctr-electronics.com/phoenix6/latest/java/) is cross-linked from our own generated docs, and CTRE's [online docs](https://v6.docs.ctr-electronics.com/) cover control requests and signal rates at the concept level.
