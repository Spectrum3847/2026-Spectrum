# PID tuning

*Audience: Reference. Assumes basic motor control and that you've read [Phoenix Tuner X](phoenix-tuner-x.md).*

CTRE's [Phoenix 6 closed loop guide](https://v6.docs.ctr-electronics.com/en/stable/) is the reference for what the gains mean and how slots work. This page is the workflow we use and the traps that are specific to this codebase.

## The trap: units follow the control type

A [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java) drives its TalonFX through one of two families of control request, and the same gain number means different things in each. Under a voltage request the gains are in volts, so `kV` is volts per radian per second. Under the `TorqueCurrentFOC` requests, which is what `setMMPositionFoc` and `setVelocityTorqueCurrentFOC` use, the gains are in amps. `configFeedForwardGains` says as much in its own documentation.

The consequence: move a mechanism from a voltage request to an FOC request without re-tuning and you get gains that look plausible on paper and behave nothing like the old ones. Check which request a mechanism is configured for before you trust its gains or copy them somewhere else.

For controllable mechanisms, tune feedforward first. PID then only has to correct what feedforward got wrong.

## The trap: slots

Gains live on the motor controller in three independent slots, and the slot a request uses is carried on the request object, not on the config. Setting slot 0 while the running request sits on slot 1 does nothing at all, with no error. `config.configPIDGains(kP, kI, kD)` writes slot 0 and `config.configPIDGains(slot, kP, kI, kD)` writes the one you name; the same pair exists for feedforward as `configFeedForwardGains`. A slot outside 0, 1, or 2 reports a warning and changes nothing.

One concrete case to know: `Config.mmPositionVoltageSlot` is constructed on slot 1 while the no-argument `configPIDGains` writes slot 0. When a position loop misbehaves in a way that looks like the gains were never applied, check which slot the request is actually on before you re-tune.

## Where the gains live

Defaults are in each subsystem's inner `*Config` class. A per-robot override goes in the matching class under `src/main/java/frc/robot/configs/`, which runs before the subsystem is constructed. Do not edit a default in a `*Config` class to fix one robot, that breaks the other three.

## Live tuning with `TuneValue`

[`TuneValue`](../../src/main/java/frc/spectrumLib/telemetry/TuneValue.java) publishes a double to SmartDashboard so it can be edited from [Elastic](elastic.md) without a redeploy. Nothing reads it back on its own: call `update()` in `periodic()` to pull the current value, or pass `getSupplier()` into a command factory and let it be read when the command runs. Passing a bare `double` freezes the value at scheduling time instead, which is the usual reason a "live" tunable is not live.

Nothing in the codebase uses a `TuneValue` yet. The class is there and works; the gains are currently set in the `*Config` classes and re-deployed.

Two rules when you start using it. Settle the gains and then delete the `TuneValue`, because it stays editable with an FMS attached and a value moving under you mid-match is worse than a value that is merely wrong. And keep one tunable per mechanism in one place so you can find them all later.

## The order that converges fastest

1. Zero out `kI` and `kD`. Set feedforward (`kS`, `kV`, `kG`) if the mechanism has the physics for it.
2. Raise `kP` until the response is fast and it starts to oscillate.
3. Add `kD` until the oscillation is gone, backing off when it turns into high frequency chatter instead.
4. Add `kI` only if there is steady state error left. Most velocity loops never need any.
5. Re-check under load. Gains that look good unloaded usually need a nudge once the mechanism is doing real work.

As a rough split: velocity loops want a small `kP`, a real `kV`, and no `kI`. Position loops want `kP`, `kD`, and `kG` when gravity is in the way, and they should be chasing a MotionMagic profile rather than a step input.

## Proving a change helped

The [Phoenix Tuner X](phoenix-tuner-x.md) plotter is the fastest way to see it: plot `Closed Loop Reference` against the matching `Position` or `Velocity`, hit record, and run the mechanism. Ten seconds of watching the two lines against each other beats ten minutes of guessing.

[`frc.spectrumLib.swerve.SysID`](../../src/main/java/frc/spectrumLib/swerve/SysID.java) is a working SysId routine for the swerve. Use it as the template if you ever need to characterize another mechanism, and read WPILib's [SysId documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html) for what the output constants mean.

## See also

* [Phoenix Tuner X](phoenix-tuner-x.md), which is where the plotter and the device configuration live.
* [Phoenix 6](../dependencies/phoenix6.md) for the control requests and the request types.
