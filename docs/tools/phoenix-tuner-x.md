# Phoenix Tuner X

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Phoenix Tuner X](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/index.html) is CTRE's tool for talking to TalonFX motors, CANcoders, CANivores and Pigeons. It runs against the robot over USB or over the same network as the Driver Station, and it is the fastest way to debug anything that lives on the CAN bus.

This page is what to do with it, not a feature tour. CTRE's own documentation is the reference for what each control and field means.

## First run

* **License every device.** Open Tuner X, sign in with the team's CTRE account, and apply the seasonal license to every Phoenix 6 device. An unlicensed device reports values but will not accept gain or configuration writes, which looks exactly like a permissions problem and is not. Re-do this every season. Licenses do not carry over.
* **Check which CAN bus you are on.** Most of this robot runs on the CANivore, whose bus name constant is `Rio.CANIVORE` in `frc.spectrumLib.hardware.Rio`. Some devices may sit on the RIO's built-in `rio` bus instead. Tuner X shows both, so make sure you are configuring the one you meant.
* **Update firmware.** Phoenix 6 motor firmware ships with each Phoenix release. Mismatched firmware is the silent cause of a large share of "the motor moved a tick weird" mysteries. Update CANivores and Pigeons through Tuner X too.

## Device IDs and names

Every device on a CAN bus needs a unique ID, and the ID space is shared across device types. A TalonFX and a CANcoder cannot both be 5 on the same bus.

IDs are wrapped in `frc.spectrumLib.util.CanDeviceId` and assigned in each subsystem's own `*Config` class, in the first argument list of its `Config` constructor. That constructor call is where the device ID, the bus and the name the code logs under are all set, so it is the one place to look for either.

Worth the habit: name the device in Tuner X after the same string the code passes as the config's first argument. The code does not write the name onto the device, so this is purely so a human reading Tuner X and a human reading the subsystem are looking at the same word. When something is misbehaving, that match is often the whole investigation.

## Swerve offset procedure

> **Use [the swerve alignment page](swerve-alignment.md) instead.** It reads the same numbers, does the sign arithmetic, warns you when a wheel is backwards, and writes the offsets into the config file for you. The procedure below is the fallback for when a device will not report at all.

This is the one ritual everyone needs at least once a season, and more often after a hard crash.

1. **Square the modules.** Drop each MK5n module's alignment pin through the top plate into the azimuth gear, with the bevel gears all facing the same side, and confirm nothing rotates when you nudge it. Do not reach for a straight edge. This drivetrain is not a rectangle, so there is no pair of wheels you can lay one across; the reasoning is on the [alignment page](swerve-alignment.md#why-theres-no-straight-edge-in-this-procedure).
2. **In Tuner X**, open each module's CANcoder and look at *Absolute Position No Offset*, which is the raw reading with nothing from `MagnetOffset` applied.
3. **Copy the raw value** into the offset field for that module, negating it if the module's mechanical orientation needs it.
4. **Deploy and re-check.** With the new offsets in code, *Absolute Position* with the offset applied should read close to zero. A reading like a few millionths is fine. A reading near a tenth of a turn means the sign is wrong or you copied from the wrong module.

Do not apply swerve offsets inside Tuner X, through the device's `MagnetOffset` field. Keep them in code so the per-robot config files stay the source of truth. Different robots have different offsets, and the unused `*2026` config classes exist to capture them.

## Plotter

Live-plotting a setpoint against a measured value is the fastest way to know whether a tuning change helped. Open the device, go to the **Plot** tab, add the signals, hit Record, run the mechanism and watch the two lines. For closed loops the pair to add is `Closed Loop Reference` and the matching `Position` or `Velocity`.

[PID Tuning](pid-tuning.md) covers what to look for once the lines are moving.

## Self-test

**Self-Test Snapshot** dumps everything the device knows about itself: firmware version, configs, faults, sticky faults, temperatures, supply voltage. If something is misbehaving and you have ten seconds, run one and save the snapshot. It is exactly what you want when asking for help on the CTRE Discord, and it is exactly what you want when replaying the problem a week later.

## Hoot logs

Phoenix 6 can record `.hoot` signal logs for every CAN device. The robot turns automatic recording off, in `Robot`'s constructor, because on 2026-09-05 the hoot files were a large share of 2.2 GB of logs on the RIO's SD card, written alongside the wpilog on a CPU that was already saturated, and nobody was replaying them.

To capture one on purpose, call `SignalLogger.start()` and `stop()` around what you want, or start it from Tuner X, and pull it afterwards with **Log Extractor**. The signals are higher resolution than the wpilog and worth having when tuning a mechanism.

The hoot path in `SwerveConfig` is not a recording location. The `CANBus` there is constructed with a filename that names a hoot file to *replay* in simulation. It does nothing on the robot, and it is easy to misread as somewhere logs are being written.

## When something looks wrong

* **A device disappeared from the bus.** Most often a CAN ID collision after someone reassigned a device. Tuner X shows duplicates with red highlighting.
* **Configs are not sticking.** Usually an expired or missing license. Re-apply the seasonal license before you go looking for a code bug.
* **A motor refuses to spin.** Check the device's sticky faults. `Hardware Failure`, `Over Voltage` and `Boot during enable` all show up here well before they are obvious from the robot side.

## See also

* [Swerve Alignment](swerve-alignment.md), the page that replaces the manual offset ritual above.
* [PID Tuning](pid-tuning.md), the workflow that uses the plotter.
* [Phoenix 6](../dependencies/phoenix6.md) for the version we vendor and its JavaDoc link.
* CTRE's [Phoenix 6 docs](https://v6.docs.ctr-electronics.com/en/latest/) for everything not covered here.
