# LEDs

*Audience: reference. Assumes you have read [2026 season specific](../other-guides/2026-season-specific.md).*

The LED strip is a Phoenix 6 CANdle driving an external strip. `SpectrumLEDs` in `spectrum_lib/leds/spectrum_leds.rs` wraps the CANdle and holds the pattern library. `Leds` in `robot/subsystems/leds.rs` is the robot's small config on top of it.

## Not wired up

`Robot::init` in `robot.rs` never builds a `Leds`, and nothing binds a pattern to a gamepad or a state. The subsystem, its config and the pattern library exist, but they have not run on a robot. Treat them as untested. No LED lights today. To light something, build `Leds` in `Robot::init` and bind patterns. Expect to debug the CANdle before the patterns.

## Three things that waste time

**The CANdle has 8 onboard LEDs before your strip starts.** `Config::new` sets `start_idx` to 8 for that reason. A strip on the CANdle output is indexed from 8. If you address from 0, the first eight cells show the onboard LEDs. A pattern that looks shifted by eight LEDs is this, not a wiring fault.

**Sharing a CANdle means sharing animation slots.** Several `SpectrumLEDs` can drive different ranges of one CANdle. Make one with `Config::with_shared_candle`. Each needs its own `animation_slot` (0 to 7), because the firmware animations write to the slot and two instances on one slot overwrite each other.

**Firmware and software patterns are different.** A firmware pattern hands the CANdle an animation, and the board draws it. A software pattern writes colors from the robot loop. `set_pattern` clears the animation slots when you switch from a firmware animation to a software solid color. If a pattern looks stuck, or two fight, check whether the kind changed without ending the first.

## Using patterns

`SpectrumLEDs` has a method for each pattern: `solid`, `blink`, `breathe`, `rainbow`, `scrolling_rainbow`, `chase`, `bounce`, `fire`, `rgb_cycle`, `stripe`, `gradient`, `edges`, `ombre`, `wave`, `countdown` and `switch_countdown`. Read the signatures first. Speed and duration arguments differ between them, and that is the usual reason a new pattern looks wrong.

`set_pattern(&pattern, priority)` returns a `Command`. It applies the pattern while it runs and keeps running while the robot is disabled, so status lights work on the bench.

## Priorities

`set_pattern` holds its priority while its command runs, and sets it to -1 when the command ends. `check_priority(n)` returns a `Trigger` that is true while the running pattern's priority is at or below `n`. Bind a low-priority pattern while that trigger is true, and a high-priority one with no condition. The high-priority pattern then takes the strip, and the low one gives it up.

`SpectrumLEDs` installs a default command. `Leds::new` replaces it with a purple breathe at priority -1. If a pattern does not show, check whether the default is winning. `default_trigger` is true while the default command is the one running.

Shift-aware patterns read the clock from `rebuilt/shift_helpers.rs`. `bind_controls` calls `shift_helpers::initialize` when teleop, auto or disabled starts, so the countdown starts at zero.

## See also

* CTRE's [Phoenix 6 documentation](https://v6.docs.ctr-electronics.com/) for the animation controls.
* [2026 season specific](../other-guides/2026-season-specific.md) for how subsystems are wired in.
