# LEDs

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

Our LED strip is a Phoenix 6 CANdle driving an external strip. [`SpectrumLEDs`](../../src/main/java/frc/spectrumLib/leds/SpectrumLEDs.java) in `frc.spectrumLib.leds` wraps the CANdle and the pattern library, and [`Leds`](../../src/main/java/frc/robot/subsystems/leds/Leds.java) is the robot's thin config on top of it.

## Not wired up

`Robot.java` declares a `Leds` field and never constructs one, and nothing binds a pattern to a gamepad or a state. The subsystem, its config, and the whole pattern library are written and ready, but they have never run on a robot, so treat them as untested. No LED is lit today. Do not assume LEDs are working. If you are asked to light something up, constructing `Leds` and binding patterns is the first step, not a detail, and expect to debug the CANdle before you debug the patterns.

## Three things that will waste your time

**The CANdle has 8 onboard LEDs before your strip begins.** `Config`'s device constructor sets `startIdx` to 8 for exactly that reason. A strip wired to the CANdle's output is indexed from 8, so the first eight cells of anything you address from 0 are invisible. A pattern that looks shifted by eight LEDs is this, not a wiring fault.

**Sharing a CANdle means sharing animation slots.** Several `SpectrumLEDs` instances can address different LED ranges of one physical CANdle, which is how you would light two strips off one board. They each need a distinct `animationSlot`, because the firmware animations write to the slot and two instances on the same slot overwrite each other.

**Firmware and software patterns are different mechanisms.** A firmware pattern hands the CANdle an animation and the board renders it with no help from robot code. A software pattern writes colors from the running loop. `setPattern` tracks which kind is running and clears the animation slot when you switch from one to the other. If a pattern looks stuck, or two patterns fight, check whether something changed kind without ending first.

## Using patterns

`SpectrumLEDs` has a factory for each pattern: `solid`, `blink`, `breathe`, `rainbow`, `scrollingRainbow`, `chase`, `bounce`, `fire`, `rgbCycle`, `stripe`, `gradient`, `edges`, `ombre`, `wave`, `countdown`, and `switchCountdown`. Read the signatures in `SpectrumLEDs.java` before using one. The speed and duration arguments are not uniform across them, and a mismatch there is the usual reason a new pattern looks wrong on the first try.

`setPattern(pattern, priority)` returns a `Command` that reapplies the pattern every loop and keeps running while the robot is disabled, which is what you want for status lights that need to be visible on the bench.

## Priorities

Priority is how patterns take turns without you writing an arbiter. `setPattern` holds the number it was given for as long as the command runs and resets it to -1 when the command ends, and `checkPriority(n)` returns a `Trigger` that is true while the running pattern's priority is at or below `n`. Bind a low-priority command `whileTrue` that trigger and a high-priority one unconditionally, and the high-priority pattern takes the strip while the low-priority one gives it up.

`SpectrumLEDs` installs a default command, and `Leds` replaces it with a purple breathe. If a pattern is not showing, the first thing to check is whether the default is winning: `defaultTrigger` is true whenever the default command is the one running.

Shift-aware patterns read the clock from [`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java), which `Robot.java` re-initializes on every enable so a practice reboot does not leave the countdown offset.

## See also

* CTRE's Phoenix 6 [CANdle](https://v6.docs.ctr-electronics.com/) documentation for the underlying animation controls.
* [2026 Season Specific](../other-guides/2026-season-specific.md) for how subsystems get wired into the robot.
