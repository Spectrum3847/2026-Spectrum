# PID Tuning

*Audience: Reference. Assumes basic motor-control concepts and that you've read [Phoenix Tuner X](phoenix-tuner-x.md).*

## The Short Version

PID drives a measured value toward a target. The three terms:

* **kP** responds to the current error. Bigger kP pulls harder; too big and the system oscillates.
* **kI** accumulates past error. It kills off steady-state error (a flywheel that always lags 1% behind target, say). Almost always small — too much causes windup.
* **kD** responds to how fast the error is changing. It damps overshoot and ringing.

PID is one piece of the controller. On a Phoenix 6 TalonFX, the slot also takes feedforward terms:

* **kS** — voltage needed to *just* start moving (static friction).
* **kV** — volts per unit of target velocity.
* **kA** — volts per unit of target acceleration.
* **kG** — constant voltage to hold against gravity (arms, elevators).

For controllable mechanisms, tune feedforward first. PID then only has to correct what feedforward got wrong.

## The Loop, Slightly More Carefully

The pieces in motion:

* **Setpoint** — what you want (a position, velocity, or angle).
* **Process variable** — what the sensor actually reads.
* **Error** — setpoint minus process variable.
* **Output** — voltage (or torque current) the controller produces from current, past, and predicted error.

## Where PID Lives in the Code

We run PID *on the motor controller*, not on the roboRIO. Phoenix 6 TalonFX motors do this natively, with up to three gain slots (`Slot0`, `Slot1`, `Slot2`). The [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java) wrapper exposes helpers for setting gains per slot (defaults to Slot 0):

    mechanism.config.configPIDGains(kP, kI, kD);          // slot 0
    mechanism.config.configPIDGains(slot, kP, kI, kD);    // pick a slot (0/1/2)

Defaults for each mechanism live in its inner `*Config` class (`LauncherConfig`, `HoodConfig`, etc.). To override per-robot, mutate them inside the matching `*2026.java` config before the subsystem is constructed.

WPILib-side PID — `ProfiledPIDController` for chassis rotation, for example — is constructed directly in the subsystem and can be re-tuned in `periodic()` if a `TuneValue` is hooked up.

## Live Tuning with `TuneValue`

[`TuneValue`](../../src/main/java/frc/spectrumLib/TuneValue.java) wraps SmartDashboard's `putNumber`/`getNumber` so a value can be edited live from [Elastic](elastic.md) without redeploying:

```java
private final TuneValue kP = new TuneValue("Launcher/kP", 0.25);

@Override
public void periodic() {
    config.configPIDGains(kP.update(), 0.0, 0.0);
}
```

`getSupplier()` returns a `DoubleSupplier`, so you can pass the tunable straight into a command factory. Don't take `TuneValue`s to competition unless you mean to — pull them or hide them behind a debug flag once gains are settled.

## A Workflow That Works

You're looking for gains that hit the target quickly, don't overshoot, don't oscillate, and don't sit short of the setpoint. The order that tends to converge fastest:

1. Set kI and kD to zero. Set feedforward (kS, kV, kG) if applicable.
2. Crank kP until the response is fast but the system starts oscillating.
3. Add kD to damp the oscillation. Keep increasing until oscillation is gone or kD itself starts causing high-frequency chatter, then back off a touch.
4. Only add kI if there's persistent steady-state error. Most velocity loops never need any.
5. Validate under load. Gains that look great unloaded often need a nudge once the mechanism is actually doing work.

A few rules of thumb by loop type:

Velocity loops (flywheels, drive wheels) usually need a small kP and a substantial kV feedforward. kI is almost always zero.

Position loops (arms, hoods) usually need kP, kD, and kG. Drive them through MotionMagic profiles so the controller is chasing a smooth trajectory, not a step input.

## What Helps

CTRE's Phoenix 6 closed-loop guide for slot configuration, units, and gain conventions on TalonFX.

WPILib's docs for the WPILib-side controllers (`ProfiledPIDController`, etc.) and for SysId, which produces feedforward constants from a system characterization run.

The Phoenix Tuner X plotter ([Phoenix Tuner X](phoenix-tuner-x.md)) — live-plotting setpoint against process variable is by far the fastest way to know whether a tweak helped or made things worse.

A SysId routine for the swerve drive already lives in [`frc.spectrumLib.swerve.SysID`](../../src/main/java/frc/spectrumLib/swerve/SysID.java). It's a useful template if you ever need to characterize another mechanism the same way.
