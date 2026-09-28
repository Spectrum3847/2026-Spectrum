# Phoenix Tuner X

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Phoenix Tuner X](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/index.html) is CTRE's tool for talking to TalonFX, CANcoder, CANivore, and Pigeon devices over the same link the Driver Station uses. It is the fastest way to see what the CAN bus is actually doing, and the only way to read a raw encoder value before the robot code gets to it.

## First run

* **License every Phoenix 6 device** with the team's CTRE account before anything else. An unlicensed device reports values but refuses gain and configuration writes, and the symptom is Tuner X appearing not to save. Re-apply the license every season; they do not carry over.
* **Pick the right bus.** `Rio.CANIVORE` is a wildcard that resolves to the first CANivore the system reports, and `Rio.RIO_CANBUS` is the roboRIO's own port. Swerve and most mechanisms are on the CANivore, and a few are not: the fuel intake is on the RIO bus. Tuner X shows both. Configuring a device through the wrong bus is the single most common wasted afternoon here.
* **Update firmware** to match the Phoenix release in `vendordeps/`. Update CANivores and Pigeons through Tuner X too. Mismatched firmware produces motion that is subtly and intermittently wrong and reads as a tuning problem.

## Naming devices

CAN IDs must be unique across all device types on a bus, so a TalonFX and a CANcoder cannot both be 5. This repo pairs the id with its bus in [`CanDeviceId`](../../src/main/java/frc/spectrumLib/util/CanDeviceId.java), adapted from [Team 254's 2023 code](https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java). Each subsystem's `*Config` class passes its own name and ID up to its superclass constructor, and the per-robot overrides live in the `*2026.java` config classes.

Name the device in Tuner X exactly what the code passes as the name. Casing is part of it, and a mismatch presents as a connection failure rather than as a typo.

## Swerve encoder offsets

Do this once a season, and again after a hard crash or any work that moves a module.

1. **Square the modules.** Drop the aligners on, bevel inward, and confirm by eye that all four point the same way. Every reading below is measured from this.
2. **Read the raw value.** In Tuner X, open each module's CANcoder and look at **Absolute Position No Offset**. That is the reading before any magnet offset in the device config is applied.
3. **Put it in the code.** The fields are `frontLeftEncoderOffset`, `frontRightEncoderOffset`, `backLeftEncoderOffset`, and `backRightEncoderOffset` in [`SwerveConfig`](../../src/main/java/frc/robot/subsystems/swerve/SwerveConfig.java). A robot's actual offsets are set by `swerve.configEncoderOffsets(...)` in its `*2026.java` class under `src/main/java/frc/robot/configs/`, so edit that file, not the defaults. Check the sign against the modules you actually have, and check whether the robot you are on already stores its readings offset by half a turn: one of them does and the defaults do not.
4. **Deploy and re-check.** With the offset applied, **Absolute Position** should sit near zero. A few millionths is fine. A reading around 0.34 means the sign is wrong or you copied from the wrong module.

Keep the offsets in code rather than in Tuner X. An offset belongs to a physical robot, and the per-robot config classes are what turn a swapped module into a config change instead of a laptop with one device configured and three not.

## Plotter

The fastest way to know whether a tuning change helped.

1. Open the device and switch to the **Plot** tab.
2. Add `Closed Loop Reference` and the matching `Position` or `Velocity`.
3. Hit record, run the mechanism, and watch the two lines against each other.

[PID Tuning](pid-tuning.md) covers what to do with what you see.

## Self-test

**Self-Test Snapshot** dumps firmware version, active config, faults, sticky faults, temperatures, and supply voltage. Run it before you ask anyone for help and save it, because it is what a CTRE Discord answer needs. It is also worth reading later when something was intermittent and you no longer remember.

## Hoot logs

Phoenix 6 records `.hoot` logs to the CANivore's onboard storage, at whatever path the `CANBus` was constructed with. Ours is built in `SwerveConfig.canBus` and points under `./logs/`. Pull them in Tuner X with **Log Extractor** after a match.

They are higher resolution than NetworkTables and they survive a dropped radio, which is what you want when the NT capture came out incomplete.

## When something looks wrong

* **A device vanished from the bus.** Usually a CAN ID collision after somebody reassigned a device. Tuner X shows the duplicate in red.
* **Configs will not stick.** Seasonal license expired, or never applied.
* **The motor refuses to spin.** Read the sticky faults before anything else. `Hardware Failure`, `Over Voltage`, and `Boot during enable` all show there long before the robot side shows symptoms.

## See also

* [PID Tuning](pid-tuning.md), the workflow that uses the plotter.
* [Phoenix 6](../dependencies/phoenix6.md) for the version and the control requests.
* CTRE's [Phoenix 6 documentation](https://v6.docs.ctr-electronics.com/en/latest/) for everything not covered here.
