# LEDs

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

**The robot's LED subsystem is not running.** [`Leds.java`](../../src/main/java/frc/robot/subsystems/leds/Leds.java) is commented out from top to bottom, and nothing in `Robot.java` constructs a `Leds`. Do not assume LEDs work, and do not write a binding against `Leds` expecting it to exist.

The library underneath it is complete and is what you would build on. This page covers the library's design and what bringing `Leds` back involves.

## The library: SpectrumLEDs

[`frc.spectrumLib.leds.SpectrumLEDs`](../../src/main/java/frc/spectrumLib/leds/SpectrumLEDs.java) wraps a Phoenix 6 `CANdle` directly, not WPILib's `AddressableLED` stack. That choice is what makes the hardware animations available at all: patterns either become a CANdle animation running on the device, or they are written per LED from the roboRIO each loop. Blink, breathe and rainbow go to the device. Gradients, ombre, countdown and anything reading a supplier have to be per-LED writes.

Read the pattern factories in that file for the list. What is worth knowing without reading them is that they all return the same `CANdlePattern` type, so a pattern from one of the hardware factories and one of the software factories are interchangeable as far as `setPattern` is concerned.

`SpectrumLEDs` implements `Subsystem`, so it gets `periodic()` and a default command. Its `Config` has two constructors and the difference matters: one takes a device ID and a bus and the instance owns the hardware, the other takes an existing `CANdle` plus a start index and a count and the instance addresses only that slice of a shared strip, applying no hardware configuration of its own. Use the second one if you ever split one physical strip between two subsystems.

The `Config` also carries an `attached` flag. A zone that is not physically connected can be left unattached, which is how a config that exists for the offseason can still be constructed in season.

## Driving patterns from commands

`setPattern(pattern, priority)` returns a command that applies the pattern every loop for as long as it runs, and holds that priority while it does.

Four things about that command are worth knowing before you build on it:

* **It runs while the robot is disabled.** That is deliberate. Status lights are the one thing you want lit while the robot is waiting on the field.

* **Priority is how one pattern preempts another.** `checkPriority(int)` returns a `Trigger` that is true when the currently-held priority is at or below the number you pass, which is the gate for a higher-priority command to take over. Endgame strobe over alliance breathing is the intended shape.

* **Priority is released when the command ends**, not when you think you are done with it, and it is released through a `finallyDo` so an interrupted command releases it too. A pattern left holding a high priority will block everything below it indefinitely.

* **Switching from a hardware animation to a software pattern clears the animation slot first**, and only this instance's own slot, so other instances sharing the same CANdle keep animating. If you skip that clear you get the ghost of the old animation under the new colours, which looks like the pattern factory is wrong.

## Bringing Leds back

The commented-out `Leds` is a complete, current-looking file, which makes it easy to assume it works. It has not been compiled since it was commented out, so treat it as a sketch.

* The hardware it describes is a CANdle on the CANivore with a 20-LED external strip, addressed from an LED index that skips the CANdle's own onboard LEDs, with loss-of-signal behaviour set to disable the LEDs.
* It sets a breathing default command in its constructor and logs its own command and animation state each loop.
* Wiring it in means constructing it in `Robot.java` and passing it to `SuperStructure`, then binding status triggers to `setPattern` commands at the right priorities, the same way any other subsystem is wired. Match-clock patterns get their timing from [`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java).
* Compile and deploy before you trust any of it. The LEDs are a CAN device, and an LED subsystem is a perfectly good way to find out the CAN bus has a spare.

## See also

* CTRE's Phoenix 6 [CANdle](https://v6.docs.ctr-electronics.com/) documentation for the animation controls and the config fields.
* [Elastic Dashboard](elastic.md), since a status LED and a dashboard widget are usually the same requirement and one of them has to be chosen.
