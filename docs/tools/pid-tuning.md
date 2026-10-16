# PID tuning

*Audience: Reference. Assumes basic motor-control concepts and that you've read [Phoenix Tuner X](phoenix-tuner-x.md).*

CTRE documents what the gains mean and in what units far better than we can, so this page is not that. It covers where the gains live in this codebase, the one thing about them that bites people, and an order of work that converges.

## Where the gains live

PID runs on the motor controller, not on the roboRIO. The TalonFX does it natively, with up to three gain slots. The [`Mechanism`](../../src/main/java/frc/spectrumLib/mechanism/Mechanism.java) wrapper exposes `config.configPIDGains(kP, kI, kD)` for slot 0 and a slot-taking overload for the others.

Defaults for each mechanism live in its own inner config class, and per-robot overrides go in the matching `*2026.java` config file before the subsystem is constructed. So the chain is: default in the mechanism's config, mutation in the robot's config file, gains programmed onto the device at construction. Read the inner config class to find the current numbers.

## The units trap

The units of a gain depend on the control request, not on the mechanism. Under a voltage request the feedforward terms are volts. Under the torque-current FOC requests this codebase uses for its FOC position and velocity loops, the same terms are amps.

The practical consequence: do not copy gains across a mechanism that has changed request type. Numbers that were right become wrong by a factor of the battery voltage, and the symptom is a mechanism that saturates its stator limit at low battery or will not break away at high battery. The `configPIDGains` javadoc says the same thing per parameter.

The other half of the trap is that on the FOC requests, gains are for Phoenix Pro.

## An order of work that converges

You are looking for gains that hit the target quickly, do not overshoot, do not oscillate, and do not sit short of the setpoint.

1. Set kI and kD to zero. Set feedforward if it applies.
2. Raise kP until the response is fast but the system starts oscillating.
3. Add kD to damp the oscillation. Keep going until the oscillation is gone or kD itself starts causing high-frequency chatter, then back off a touch.
4. Add kI only if there is persistent steady-state error. Most velocity loops never need any.
5. Validate under load. Gains that look good unloaded usually need a nudge once the mechanism is actually doing work.

For controllable mechanisms, tune feedforward first. PID then only has to correct what feedforward got wrong, and a loop that is already close does not need much gain to finish the job.

By loop type, as a starting point rather than a rule:

* **Velocity loops** (flywheels, drive wheels) want a small kP and a substantial kV feedforward. kI is almost always zero.
* **Position loops** (arms, hoods) want kP, kD and gravity compensation, driven through a motion profile so the controller is chasing a smooth trajectory rather than a step input.

## Live tuning with `TuneValue`

[`TuneValue`](../../src/main/java/frc/spectrumLib/telemetry/TuneValue.java) is a number published to `SmartDashboard` and read back on demand, so a gain can be edited live from [Elastic](elastic.md) without a redeploy. `update()` refreshes the cached value, and `getSupplier()` hands back a `DoubleSupplier` you can put straight into a command factory, so a tunable setpoint stays live for the whole time a command runs.

**Nothing in this codebase currently uses it.** The class is there, the wiring works, and the class is unused. That is deliberate for now, and it is worth knowing before you go looking for an example to copy. Gains are set in config and tuned through the plotter, not through the dashboard.

If you do adopt it, do not take tunables to competition unless you mean to. `SmartDashboard` publishes regardless of the FMS, so a gain someone left editable at a shop bench is a gain someone can change at an event. Hide them behind a flag or remove them once the gains are settled. The DogLog `tunableOnFMS` flag does not cover this; see [Logging and Data Analysis](logging.md).

## What helps

* The Phoenix 6 closed-loop guide for slot configuration, units, and gain conventions on TalonFX. That is the reference to read, not this page.
* WPILib's docs for the WPILib-side controllers, and for SysId, which produces feedforward constants from a system characterization run. A SysId routine for the swerve drive already exists in [`frc.spectrumLib.swerve.SysID`](../../src/main/java/frc/spectrumLib/swerve/SysID.java) and is a useful template if you ever need to characterize another mechanism the same way.
* The [Phoenix Tuner X](phoenix-tuner-x.md) plotter, which is the fastest way to know whether a tweak helped or made things worse. Plot the closed-loop reference against the matching position or velocity signal and watch the two lines.
