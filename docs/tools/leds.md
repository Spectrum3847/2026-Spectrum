# LEDs

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The robot's LEDs are live: [`Leds`](../../src/main/java/frc/robot/subsystems/leds/Leds.java) (`frc.robot.subsystems.leds`) extends `SpectrumLEDs` and drives a Phoenix 6 CANdle — device ID 1 on the CANivore, a 20-LED RGB external strip.

## Library: `SpectrumLEDs`

[`frc.spectrumLib.leds.SpectrumLEDs`](../../src/main/java/frc/spectrumLib/leds/SpectrumLEDs.java) is our wrapper around a **Phoenix 6 `CANdle`** (not the WPILib `AddressableLED` stack). It `implements Subsystem` and owns:

* A `CANdle` (device ID + CAN bus, set by `Config`).
* A `CANdleConfiguration` for strip type, brightness, and loss-of-signal behavior.
* An animation slot the CANdle uses for hardware animations (strobe, fade, rainbow, …).

The constructor takes a `Config` either by device id + LED count (it owns the CANdle) or by sharing an existing `CANdle` with a start index and count (a sub-view that addresses a slice of the same physical strip without owning the hardware).

## Patterns

`SpectrumLEDs` ships with pattern factories that return `CANdlePattern` objects (some backed by hardware CANdle animations, some by per-LED color writes):

| Method | What you get |
| --- | --- |
| `solid(color)` | A static color. |
| `blink(color, onTime)` | On for `onTime` seconds, off for the same. |
| `breathe(color, period)` | Smooth fade in/out across `period` seconds. |
| `rainbow()` / `scrollingRainbow()` | Full rainbow, optionally scrolling at 0.25 m/s along the strip. |
| `gradient(colors...)` | Continuous gradient between an arbitrary number of colors. |
| `stripe(percent, c1, c2)` | First `percent` of strip in `c1`, rest in `c2`. |
| `chase(color, percent, speed)` | A moving block of `color` covering `percent` of the strip, scrolling at `speed` Hz. |
| `bounce(color, duration)` | A lit cell with two trails of dimmer color bouncing across the strip. |
| `ombre(start, end)` / `wave(c1, c2, len, dur)` | Color transitions implemented inline because WPILib's built-ins don't quite cover them. |
| `countdown(startSupplier, duration)` | Strip starts full, turns off back-to-front over `duration`, color fades yellow → red. |
| `switchCountdown(startColor)` | 2026-specific: alliance-shift countdown that flips between alliance colors and purple. |
| `edges(color, length)` | `length` LEDs lit at each end, rest off. |

Hardware-animation patterns (blink, breathe, rainbow) are driven by the CANdle's own animation engine via Phoenix 6 controls (`StrobeAnimation`, `SingleFadeAnimation`, `RainbowAnimation`, …); the color/gradient patterns are written per-LED each loop.

## Driving Patterns from Commands

`setPattern(pattern, priority)` returns a `Command` that applies the pattern every loop. It calls `.ignoringDisable(true)`, so LED commands keep running while the robot is disabled — exactly what you want for status lights.

```java
public Command idleLights() {
    return leds.setPattern(leds.breathe(purple, 2.0), 1);
}
```

The `priority` slot is an integer. `checkPriority(int)` returns a `Trigger` so a higher-priority animation (endgame strobe) can preempt a lower-priority one (alliance breathing) cleanly through a `Trigger` chain.

## Wiring It Into the Robot

`Leds` is a normal subsystem: bind status triggers (auto mode, alliance shift, endgame, "about to shift") to `leds.setPattern(...)` commands at their priorities, the same way any other subsystem is wired in `Robot.java`. The [`ShiftHelpers`](../../src/main/java/frc/rebuilt/ShiftHelpers.java) match-clock windows are what the shift-aware animations key off of.

## See Also

* CTRE's Phoenix 6 [CANdle](https://v6.docs.ctr-electronics.com/) docs for the underlying animation controls.
* `Subsystem` for the lifecycle hooks the scheduler runs each loop.
