# Phoenix Tuner X

*Audience: reference. Assumes you have read [Setup](../setup.md).*

[Phoenix Tuner X](https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/index.html) is CTRE's tool for talking to TalonFX, CANcoder, CANivore and Pigeon devices over the same link the driver station uses. It is the fastest way to see what the CAN bus is doing, and the only way to read a raw encoder value before the robot code gets to it.

## First run

* **License every Phoenix 6 device** with the team's CTRE account first. An unlicensed device reports values but refuses gain and config writes. The symptom is Tuner X that seems not to save. Apply the license again every season.
* **Pick the right bus.** `Rio::CANIVORE` is a wildcard for the first CANivore the system reports. `Rio::RIO_CANBUS` is the roboRIO's own port. Swerve and most mechanisms are on the CANivore. The fuel intake is on the roboRIO bus. Tuner X shows both. Configuring a device through the wrong bus wastes many afternoons.
* **Update firmware** to match the Phoenix version the robot uses (26.3.0, set in `rust/xtask/src/fetch.rs`). Update CANivores and Pigeons in Tuner X too. Mismatched firmware gives motion that is slightly and sometimes wrong, and looks like a tuning problem.

## Naming devices

CAN IDs must be unique across all device types on a bus. A TalonFX and a CANcoder cannot both be 5. The robot pairs an ID with its bus in `CanDeviceId` (`spectrum_lib/util/can_device_id.rs`), adapted from [Team 254's 2023 code](https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java). Each subsystem's config struct sets its own name, ID and bus. Swerve IDs are in `SwerveConfig`. A robot that differs changes values in its function in `robot/configs/`.

Name the device in Tuner X the same as the name in the code. Casing counts. A mismatch looks like a connection failure, not a typo.

## Swerve encoder offsets

Do this once a season, and again after a hard crash or work that moves a module.

1. **Square the modules.** Put the aligners on, bevel gear inward, and check by eye that all four point the same way.
2. **Read the raw value.** In Tuner X, open each module's CANcoder and read **Absolute Position No Offset**. That is the value before any magnet offset in the device config.
3. **Put it in the code.** The fields are `front_left_encoder_offset_rot`, `front_right_encoder_offset_rot`, `back_left_encoder_offset_rot` and `back_right_encoder_offset_rot` in `SwerveConfig` (`robot/subsystems/swerve/swerve_config.rs`). A robot sets its own values with `config_encoder_offsets(...)` in its file in `robot/configs/`, so edit that file and leave the defaults. Check the sign against your modules. `pm2026` and `xm2026` store their readings shifted by half a turn (`+ 0.5` or `- 0.5`). The other configs do not.
4. **Deploy and check.** With the offset applied, **Absolute Position** should be near zero. A few millionths is fine. A value near 0.34 means the sign is wrong, or you copied from the wrong module.

Keep offsets in code, not in Tuner X. An offset belongs to a physical robot. Per-robot config files turn a swapped module into a config change, not a laptop with one device set and three not.

## Plotter

The fastest way to see if a tuning change helped.

1. Open the device and go to the **Plot** tab.
2. Add `Closed Loop Reference` and the matching `Position` or `Velocity`.
3. Record, run the mechanism, and compare the two lines.

[PID tuning](pid-tuning.md) says what to do with what you see.

## Self-test

**Self-Test Snapshot** shows firmware version, active config, faults, sticky faults, temperatures and supply voltage. Run it before you ask anyone for help, and save it. A CTRE support answer needs it. It also helps later, when a problem came and went and nobody remembers the details.

## Hoot logs

Phoenix 6 can record `.hoot` signal logs. Extract them in Tuner X with **Log Extractor**. They have more detail than NetworkTables and survive a dropped radio. The `phoenix6` crate has a `signal_logger` module (`start`, `stop`, `set_path`). The robot code does not call it, so check the logging settings in Tuner X before a run you want recorded. We have not confirmed what is recorded by default.

`SwerveConfig` has a `hoot_file_path` of `./logs/spectrum.hoot`. At startup `Swerve::new` builds the CAN bus with that path, which loads the file for replay, and reports a warning if it cannot. That is for replay, not for recording.

## When something looks wrong

* **A device vanished from the bus.** Usually a CAN ID clash after someone changed a device. Tuner X shows the duplicate in red.
* **Configs will not stick.** The seasonal license expired or was never applied.
* **The motor refuses to spin.** Read the sticky faults first. `Hardware Failure`, `Over Voltage` and `Boot during enable` show there long before the robot code shows symptoms.

## See also

* [PID tuning](pid-tuning.md) for the workflow that uses the plotter.
* [Phoenix 6](../dependencies/phoenix6.md) for the crate and the control requests.
* CTRE's [Phoenix 6 documentation](https://v6.docs.ctr-electronics.com/en/latest/).
