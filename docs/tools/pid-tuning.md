# PID tuning

*Audience: reference. Assumes basic motor control and that you have read [Phoenix Tuner X](phoenix-tuner-x.md).*

CTRE's [Phoenix 6 closed loop guide](https://v6.docs.ctr-electronics.com/en/stable/) explains what the gains mean and how slots work. This page is the workflow we use, and the traps in this code base.

## The trap: units follow the control type

A `Mechanism` (`spectrum_lib/mechanism/mechanism.rs`) drives its TalonFX with one of two kinds of control request. The same gain means different things in each. With a voltage request, gains are in volts, so `kV` is volts per rotation per second. With a torque current FOC request, gains are in amps. `set_mm_position_foc` and `set_velocity_torque_current_foc` use torque current.

If you move a mechanism from a voltage request to an FOC request without tuning again, the gains look fine and behave nothing like the old ones. Check which request a mechanism uses before you trust or copy its gains.

Tune feedforward first. PID then only fixes what feedforward missed.

## The trap: slots

A TalonFX holds gains in three slots, 0, 1 and 2. The slot a request uses is set on the request, not in the config. If you set slot 0 while the running request uses slot 1, nothing happens and nothing reports an error.

* `config_pid_gains(kp, ki, kd)` writes slot 0. `config_pid_gains_slot(slot, kp, ki, kd)` writes the slot you name.
* `config_feed_forward_gains(ks, kv, ka, kg)` writes slot 0. `config_feed_forward_gains_slot(slot, ...)` names a slot.
* A slot other than 0, 1 or 2 reports a warning to the driver station and changes nothing.

One case to know: `mm_position_voltage_slot` in `Config` starts on slot 1, while `config_pid_gains` writes slot 0. If a position loop acts as if the gains were never applied, check the slot first.

## Where the gains live

Defaults are in each subsystem's config struct, such as `HoodConfig` in `robot/subsystems/hood.rs`. A robot with different hardware changes them in its config function in `robot/configs/`. Do not change a default to fix one robot. That changes every robot.

## Live tuning with TuneValue

`TuneValue` (`telemetry/src/tune_value.rs`) publishes a number to SmartDashboard so you can edit it in [Elastic](elastic.md) without a deploy. Nothing reads it back on its own. Call `update()` each loop, or pass `supplier()` to code that reads it when it runs. A plain `f64` freezes the value when the command is created, which is why a "live" tunable sometimes is not live.

No robot code uses `TuneValue` yet. The gains are set in the config structs, then deployed again.

Two rules when you use it. Pick the gains, then remove the `TuneValue`. It stays editable with an FMS attached, and a value that changes during a match is worse than a wrong one. And keep one tunable per mechanism in one place.

## The order that converges fastest

1. Set `kI` and `kD` to zero. Set feedforward (`kS`, `kV`, `kG`) if you know the physics.
2. Raise `kP` until the response is fast and starts to oscillate.
3. Add `kD` until the oscillation stops. Back off if it turns into fast chatter.
4. Add `kI` only if a steady error remains. Most velocity loops never need it.
5. Check again under load. Gains that look good with no load usually need a small change.

Velocity loops want a small `kP`, a real `kV` and no `kI`. Position loops want `kP`, `kD`, and `kG` when gravity matters. They should follow a Motion Magic profile, not a step.

## Proving a change helped

Use the [Phoenix Tuner X](phoenix-tuner-x.md) plotter. Plot `Closed Loop Reference` against `Position` or `Velocity`, record, and run the mechanism.

`SysId` in `spectrum_lib/swerve/sys_id.rs` has SysId routines for the swerve (translation, rotation and steer). Use it as the model if you need to characterize another mechanism. The `commands` crate has the `SysIdRoutine` they use. See WPILib's [SysId docs](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html) for what the output constants mean.

## See also

* [Phoenix Tuner X](phoenix-tuner-x.md) for the plotter and device setup.
* [Phoenix 6](../dependencies/phoenix6.md) for the control requests.
