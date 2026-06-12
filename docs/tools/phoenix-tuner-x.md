# Phoenix Tuner X

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Phoenix Tuner X](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/index.html) is CTRE's all-in-one tool for talking to TalonFX motors, CANcoders, CANivores, and Pigeons. It runs against the robot over USB or the same network as the Driver Station and is the fastest way to debug anything that lives on the CAN bus.

This page is what to actually do with it — not a feature tour.

## First-Run Setup

* **License each device**. Open Tuner X, sign in with the team's CTRE account, and apply the seasonal license to every Phoenix 6 device. An unlicensed device will report values but won't accept gain or configuration writes. Re-do this every season — licenses don't carry over.
* **Pick the right CAN bus**. We run swerve and most mechanism motors on the CANivore (`Rio.CANIVORE`, see [`SwerveConfig.java`](../../src/main/java/frc/robot/swerve/SwerveConfig.java)). Other devices may sit on the RIO's built-in `rio` bus. Tuner X shows both — make sure you're configuring the right one.
* **Update firmware**. Phoenix 6 motor firmware ships with each Phoenix release. Mismatched firmware is the silent cause of half the "the motor moved a tick weird" mysteries. Update CANivores and Pigeons through Tuner X too.

## Device IDs and Names

Every device on a CAN bus needs a unique ID — including across types (a TalonFX and a CANcoder cannot share ID 5 on the same bus). Spectrum's convention:

* IDs are wrapped in [`frc.spectrumLib.util.CanDeviceId`](../../src/main/java/frc/spectrumLib/util/CanDeviceId.java); the per-subsystem `*Config` classes own the actual ID assignments (`LauncherConfig.MOTOR_ID = 30`, etc.).
* The Tuner X device name should match the robot-code constant. If the launcher motor is `LauncherConfig.MOTOR_ID = 30` in code, name the device `launcher` in Tuner X. This makes the live device list line up with the code, which speeds up debugging during a frantic match-day setup.

## Swerve Offset Procedure

This is the one ritual everyone needs at least once a season, and probably more after a hard crash.

1. **Square the modules.** Drop the physical swerve aligner blocks on, bevel inward, and confirm visually that all four wheels point the same way.
2. **In Tuner X**, open each module's CANcoder. Look at *Absolute Position No Offset* — that's the raw reading without anything from `MagnetSensorConfigs.MagnetOffset` applied.
3. **Copy the raw value** to the matching field in the swerve constants/config that the per-robot `*2026` file applies. Mind the sign — depending on the module's mechanical orientation, you may need the negative of the reading.
4. **Deploy and re-check.** With the new offsets in code, *Absolute Position* (with offset) should read close to zero. A reading like `0.0000024` is fine; `0.34` means the sign is wrong or you copied from the wrong module.

Don't apply offsets *inside* Tuner X (via the device's MagnetOffset field) for swerve — keep them in code so the per-robot config files stay the source of truth. Different robots will have different offsets; that's exactly what `FM2026` / `PM2026` / `XM2026` exist to capture.

## Plotter

Live-plotting setpoint against process variable is by far the fastest way to know whether a tuning change helped or made things worse. From Tuner X:

* Open the device, switch to the **Plot** tab.
* Add signals — typically `Closed Loop Reference` and the matching `Position` / `Velocity`.
* Hit Record, run the mechanism, watch the two lines.

The [PID Tuning](pid-tuning.md) doc walks through what to look for; this is the surface that makes those judgments fast.

## Self-Test

The **Self-Test Snapshot** dumps everything the device knows about itself: firmware version, configs, faults, sticky faults, temperatures, supply voltage. If something is misbehaving and you have ten seconds, run a self-test and save the snapshot — it's exactly what you'd want when asking for help on the CTRE Discord or by replaying it later.

## Hoot Logs

Phoenix 6 records `.hoot` logs to the CANivore's onboard storage. [`SwerveConfig.java`](../../src/main/java/frc/robot/swerve/SwerveConfig.java) configures the path:

```java
canBus = new CANBus(Rio.CANIVORE, "./logs/spectrum.hoot");
```

After a match, pull the logs in Tuner X via **Log Extractor**. The signals are higher-resolution than what NetworkTables sees and survive even if the radio drops — useful for post-match analysis when the NT capture is incomplete.

## When Something Looks Wrong

Common quick checks before going deeper:

* **A device disappeared from the bus** — most often a CAN ID collision after someone reassigned a device. Tuner X will show duplicates with red highlighting.
* **Configs aren't sticking** — license expired for the season, or the device is unlicensed. Re-apply the seasonal license.
* **Motor refuses to spin** — check the device for sticky faults. `Hardware Failure`, `Over Voltage`, `Boot during enable` all show up here before the symptoms are obvious from the robot side.

## See Also

* [PID Tuning](pid-tuning.md) — the workflow that actually uses the plotter.
* [Phoenix 6 dependency page](../dependencies/phoenix6.md) — version, JavaDoc cross-link.
* CTRE's [Phoenix 6 docs](https://v6.docs.ctr-electronics.com/en/latest/) for everything not covered here.
