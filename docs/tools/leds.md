# LEDs

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

> **State of play (2026):** [`CANdleLeds.java`](../../src/main/java/frc/robot/leds/CANdleLeds.java) and [`LedStates.java`](../../src/main/java/frc/robot/leds/LedStates.java) are currently commented out on this branch; LED revival work is happening on the off-season branch. The Phoenix 6 CANdle remains the hardware driver — the WPILib `AddressableLED`-based `SpectrumLEDs` library below is a separate tool that may get integrated alongside it, not a replacement.

## Library: `SpectrumLEDs`

[`frc.spectrumLib.leds.SpectrumLEDs`](../../src/main/java/frc/spectrumLib/leds/SpectrumLEDs.java) is our wrapper around WPILib's addressable-LED stack. It implements `SpectrumSubsystem`, owns:

* An `AddressableLED` (PWM port, set by `Config.port`).
* An `AddressableLEDBuffer` of fixed length.
* An `AddressableLEDBufferView` — a windowed slice of the buffer so multiple `SpectrumLEDs` instances can drive different sections of the same physical strip independently.

The constructor takes a `Config` either by buffer size (in which case it allocates its own LED + buffer and is the "main view") or by sharing an existing buffer with a start/end index (a sub-view that doesn't own the hardware). Only the main view actually pushes data to the strip in `periodic()`, so creating sub-views is free.

## Patterns

`SpectrumLEDs` ships with pattern factories that return WPILib `LEDPattern` objects:

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

The patterns are pure WPILib `LEDPattern` objects, so anything from the [WPILib LED docs](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html) composes — `pattern.reversed()`, `pattern.atBrightness(...)`, `pattern.scrollAtAbsoluteSpeed(...)` all work.

## Driving Patterns from Commands

`setPattern(pattern, priority)` returns a `Command` that applies the pattern every loop. It calls `.ignoringDisable(true)`, so LED commands keep running while the robot is disabled — exactly what you want for status lights.

```java
public Command idleLights() {
    return leds.setPattern(leds.breathe(Color.kPurple, 2.0), 1);
}
```

The `priority` slot is an integer. `commandPriority` is exposed via `checkPriority(int)` so a higher-priority animation (endgame strobe) can preempt a lower-priority one (alliance breathing) cleanly through a `Trigger` chain. The previous `LedStates.java` used this to layer match-time triggers — that file is the reference for the pattern, even though it's currently commented out.

## When the New Wiring Lands

The plan is to revive `CANdleLeds.java` — the Phoenix 6 `CANdle`-backed subsystem — with `LedStates.java` doing what it did before: binding `Trigger`s — auto mode, transition, alliance shift, endgame, "about to shift" — to LED animations of varying priority. The CANdle stays; integrating `SpectrumLEDs` patterns alongside it is a possibility, not the goal. Whoever picks the work back up should:

1. Uncomment and update `CANdleLeds.java` (CANdle on the CANivore bus, 20 LEDs, RGB strip).
2. Wire `setupDefaultCommand()` and `setupStates()` through to `LedStates`.
3. Restore the `Trigger`s in `LedStates.bindTriggers()` (alliance, match-time windows, auto/teleop). The commented-out file has the time windows already worked out for 2026's shift cadence.

## See Also

* WPILib's [`AddressableLED`](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html) docs for the underlying API.
* `frc.spectrumLib.SpectrumSubsystem` for the lifecycle hooks (`setupStates`, `setupDefaultCommand`) every subsystem in this codebase implements.
