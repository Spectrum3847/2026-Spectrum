# Swerve Alignment

*Audience: Anyone squaring up the drivetrain. Assumes you've read [Setup](../setup.md).*

Zeroing the swerve modules used to mean reading four numbers off [Phoenix Tuner X](phoenix-tuner-x.md) and hand-copying them into a config file, getting the sign right, and hoping nobody pointed a wheel backwards. The app in `tools/swerve-align` does the reading, the arithmetic, and the sanity checks, then writes the numbers into the code for you.

It only ever edits four numbers in one file. It never writes anything to the robot.

## Why there's no straight edge in this procedure

The usual FRC trick — lay a long bar against the two wheels down one side of the robot — assumes the modules sit on a rectangle. Ours don't:

* The front modules are at x = 10.25", the rear at x = −7.368" ([`SwerveConfig.java`](../../src/main/java/frc/robot/subsystems/swerve/SwerveConfig.java)).
* The front track is 13.375" per side, the rear 13.25". A bar laid down one side spans two wheels that are ⅛" apart laterally, so it can't sit flat against both.
* The rear modules mount to the angled plates of the hexagonal frame, so the nearest frame rail isn't parallel to the robot's fore-aft axis either. There's nothing local to square against.

So don't try. **MK5n modules have their own alignment hole** through the top plate into the azimuth gear — drop the pin in and the module is mechanically locked pointing straight ahead. That references the module to itself, which is exactly what you want when the frame geometry can't be trusted as a reference.

Pin each module, capture, pull the pins. That's the whole method.

## Starting it

Any of these:

* Double-click **`tools/swerve-align/align-swerve.bat`**
* `node tools/swerve-align/server.js`
* `./gradlew alignSwerve`

```bash
node tools/swerve-align/server.js
```

All three open <http://127.0.0.1:5817> in a browser. Leave the console window open — closing it stops the app.

**To put it on the desktop:** right-click `align-swerve.bat` → **Send to** → **Desktop (create shortcut)**. Rename the shortcut to something like "Align Swerve".

## What it needs

* The **robot powered on and connected** to your laptop, and running code built from this branch. The app talks to the robot over NetworkTables; it will tell you if the robot's offsets don't match the source you're about to edit.
* The **robot disabled.** Aligning an enabled robot is both dangerous and pointless — the steer motors will fight the pins.

## The procedure

1. **Pin every module you're aligning.** Pin through the top plate into the azimuth gear, then nudge each wheel — if it moves at all, the pin isn't seated.
2. **Check the bevel gears all face the same side of the robot.** The pin locks rotation, not which way round the module was assembled. This is the one thing that survives pinning and still produces a 180° error.
3. **Tick the checklist** in step 1 of the app.
4. **Pick which modules to align** in step 3. Default is all four. Untick the ones you haven't pinned — after swapping a single module, for instance. Anything unticked keeps the offset it already has in the code, untouched.
5. **Look at step 2.** Each module shows the angle the robot currently believes it's at, plus its raw encoder reading. If a card is red, that module isn't reporting — fix that first.
6. **Click Capture.** The app averages half a second of readings and shows a table: what's in the source now, what it would write, how far that moved, and a verdict.
7. **Read the verdicts.** See below. Anything flagged needs a decision from you before the Write button unlocks.
8. **Click Write offsets.** It updates `swerve.configEncoderOffsets(...)` in `src/main/java/frc/robot/configs/OM2026.java` and nothing else.
9. **Deploy**, then come back to the app: with the pins still in, every module you captured should now read close to 0°.
10. **Pull all the pins.** Enabling with a pin in will break something.
11. **Enable and drive slowly straight forward.** If a wheel fights or the robot crabs, that module is a half turn out. This is the only test that actually proves the alignment.
12. **Commit** the change, or `git checkout -- src/main/java/frc/robot/configs/OM2026.java` to throw it away.

## Reading the verdicts

| Verdict | What it means |
| --- | --- |
| **Already aligned** | Under 3° of change. The module was fine. |
| **Normal correction** | 3–30°. What a routine touch-up looks like. |
| **Wheel likely backwards** | About 180° of change. Either this module is pinned backwards, or the offset in the code was taken with it backwards. |
| **Unexpected change** | A big jump that isn't a half turn. Usually a pin that isn't seated, or a module wired to a different CANcoder than the code thinks. |
| **Not captured** | You didn't select this module. Its offset is left alone. |

The two flagged verdicts block the Write button until you say what to do about them.

For a **backwards** module you get two choices:

* *"The pin is in and this wheel really is pointed forward"* — the module is fine and the old offset was wrong. The new value is used as measured.
* *"This module is pinned backwards"* — it went in the wrong way round, most likely a flipped bevel. The app adds a half turn and writes the corrected value, so you don't have to physically redo it. Fix the bevel before a competition anyway; a module running backwards from its neighbours is a drivetrain problem waiting to happen.

Re-pinning and capturing again is always a valid third option, and usually the one to prefer.

### If everything says "backwards"

That's its own banner, because it means something bigger: either the whole robot is pointed backwards right now, or every offset in the code was taken with the wheels backwards. Both happened to us — the Aug 2 and Aug 20 2026 alignments differ by roughly 180° on all four modules (see [the 2026 offseason handoff](../other-guides/offseason-handoff-2026-09-05.md)). Work out which end of the robot is the front before writing anything.

### "The robot is not running this code"

The magnet offset programmed into the CANcoders doesn't match what's in the source file. Deploy the current code first. If you align against a stale deploy, the difference between the two gets folded into the new offsets and you end up worse off than you started.

## How it works

`SwerveAlignment.java` publishes each module's raw encoder data to NetworkTables at 20 Hz:

```
/Robot/Swerve/Align/<FrontLeft|FrontRight|BackLeft|BackRight>/
    AbsoluteRotations   what the robot thinks the module angle is, offset applied
    RawRotations        the same reading with the offset backed out
    AppliedOffset       the magnet offset actually programmed into the CANcoder
    ConfigOffset        the offset compiled into the robot code
    SteerVelocity       used to refuse a capture while a wheel is still moving
    Connected
    EncoderId
/Robot/Swerve/Align/Timestamp
```

These are visible in AdvantageScope too, which on its own beats opening Tuner X to check a module.

`RawRotations` is the same number Tuner X calls *Absolute Position No Offset*. The offset that makes "forward" read zero is simply its negative:

```
newOffset = wrap(-rawRotations)
```

wrapped into `[-0.5, 0.5)` rotations and rounded to 1/4096, the CANcoder's own resolution.

The browser talks NetworkTables to the robot directly. The small Node server only serves the page and reads/writes the config file; it binds to `127.0.0.1` so nothing on the pit network can ask it to edit your source. Implementation notes are in [`tools/swerve-align/README.md`](../../tools/swerve-align/README.md).

## When to fall back to Tuner X

The [Phoenix Tuner X](phoenix-tuner-x.md) procedure still works and is worth knowing. Reach for it when the app can't help: a CANcoder that won't report at all, a device that needs licensing or a firmware update, or a bus that isn't enumerating.
