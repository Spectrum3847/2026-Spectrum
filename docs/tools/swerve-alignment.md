# Swerve alignment

*Audience: Anyone squaring up the drivetrain. Assumes you've read [Setup](../setup.md).*

Zeroing the swerve modules used to mean reading four numbers off [Phoenix Tuner X](phoenix-tuner-x.md) and hand-copying them into a config file, getting the sign right, and hoping nobody pointed a wheel backwards. The **Swerve Align** page of the [robot app](../../tools/robot-app/README.md) does the reading, the arithmetic and the sanity checks, then writes the numbers into the code for you.

It only ever edits the offset call in one config file. It never writes anything to the robot.

## Why there's no straight edge in this procedure

The usual FRC trick is to lay a long bar against the two wheels down one side of the robot, on the
assumption that the modules sit on a rectangle. Ours do not.

* The front and rear modules sit at different fore-aft positions, so the four wheels are not corners
  of anything. Those positions are the module x and y fields in
  [`SwerveConfig.java`](../../src/main/java/frc/robot/subsystems/swerve/SwerveConfig.java).
* The front track and the rear track are not the same width, and the difference is bigger than a
  machined part tolerates. A bar laid down one side spans two wheels that are slightly apart
  laterally, so it cannot sit flat against both.
* The rear modules mount to the angled plates of the hexagonal frame, so the nearest frame rail is
  not parallel to the robot's fore-aft axis either. There is nothing local to square against.

So do not try. **MK5n modules have their own alignment hole** through the top plate into the azimuth
gear. Drop the pin in and the module is mechanically locked pointing straight ahead. That references
the module to itself, which is exactly what you want when the frame geometry cannot be trusted as a
reference.

Pin each module, capture, pull the pins. That is the whole method.

If you ever re-measure the frame and find the four modules *are* on a rectangle, the straight-edge
method becomes available and is cheaper. Check the module positions in `SwerveConfig` rather than
assuming this page is still true.

## Starting it

Any of these:

* Double-click `tools/robot-app/align-swerve.bat`
* `./gradlew alignSwerve`
* `cd tools/robot-app && npm run align`

All three open <http://localhost:5801/pages/swerve-align/> in a browser. Leave the console window
open, because closing it stops the app.

The first run installs dependencies, which needs internet once. After that it works offline. The
batch file does that for you.

**To put it on the desktop:** right-click `align-swerve.bat`, then **Send to**, then **Desktop
(create shortcut)**. Rename the shortcut to something like "Align Swerve".

Alignment is one page of the robot app. The nav bar at the top also has the pilot and operator
control maps, log sync, and the power and CAN-bus analysis pages. `./gradlew robotApp` opens it on
the home page instead.

## What it needs

* The **robot powered on and connected** to your laptop, and running code built from this branch.
  The app talks to the robot over NetworkTables, and it will tell you if the offsets the robot is
  running do not match the source you are about to edit.
* The **robot disabled.** Aligning an enabled robot is both dangerous and pointless, because the
  steer motors will fight the pins.

## The procedure

1. **Pin every module you are aligning.** Pin through the top plate into the azimuth gear, then nudge
   each wheel. If it moves at all, the pin is not seated.
2. **Check the bevel gears all face the same side of the robot.** The pin locks rotation, not which
   way round the module was assembled. This is the one thing that survives pinning and still
   produces a half-turn error.
3. **Tick the checklist** in step 1 of the app.
4. **Pick which modules to align** in step 3. The default is all four. Untick the ones you have not
   pinned, after swapping a single module for instance. Anything unticked keeps the offset it
   already has in the code, untouched.
5. **Look at step 2.** Each module shows the angle the robot currently believes it is at, plus its
   raw encoder reading. A red card means that module is not reporting. Fix that first.
6. **Click Capture.** The app averages a short window of readings and shows a table: what is in the
   source now, what it would write, how far that moved, and a verdict.
7. **Read the verdicts**, below. Anything flagged needs a decision from you before the Write button
   unlocks.
8. **Click Write offsets.** It updates the `swerve.configEncoderOffsets(...)` call in
   `src/main/java/frc/robot/configs/OM2026.java` and nothing else. Which file that is comes from the
   app's config, so if `Robot.java` ever selects a different robot, change the app's target rather
   than editing the file by hand afterwards.
9. **Deploy**, then come back to the app. With the pins still in, every module you captured should
   now read close to zero.
10. **Pull all the pins.** Enabling with a pin in will break something.
11. **Enable and drive slowly straight forward.** If a wheel fights or the robot crabs, that module
    is a half turn out. This is the only test that actually proves the alignment.
12. **Commit** the change, or `git checkout -- src/main/java/frc/robot/configs/OM2026.java` to throw
    it away.

## Reading the verdicts

Four outcomes cover almost every capture, and the app decides between them from how far the offset
moved. The thresholds it uses are constants at the top of the alignment page's script, so read them
there if you want the exact numbers.

* **Already aligned.** The offset barely moved, so the module was fine and you do not need to deploy
  for it.
* **Normal correction.** A modest move, which is what a routine touch-up looks like.
* **Wheel likely backwards.** About a half turn, which means one of two very different things: the
  module is pinned backwards right now, or the offset in the code was taken with it backwards. You
  get a choice between those two readings, and the app adds a half turn for you if you confirm the
  pin went in the wrong way round. Fix the bevel before a competition anyway. A module running
  backwards from its neighbours is a drivetrain problem waiting to happen.
* **Unexpected change.** A big move that is not a half turn. Usually a pin that is not seated, or a
  module wired to a different CANcoder than the code thinks.

The two flagged verdicts block the Write button until you say what to do about them. Re-pinning and
capturing again is always a valid answer, and usually the one to prefer.

A module you did not select shows as not captured, and its offset is left alone.

### If everything says backwards

That gets its own banner, because it means something bigger than one module: either the whole robot
is pointed backwards right now, or every offset in the code was taken with the wheels backwards.
Both have happened here. The Aug 2 and Aug 20 2026 alignments differ by roughly a half turn on all
four modules; see the
[2026 offseason handoff](../other-guides/offseason-handoff-2026-09-05.md). Work out which end of the
robot is the front before writing anything.

### "The robot is not running this code"

The magnet offset programmed into the CANcoders does not match what is in the source file. Deploy the
current code first. If you align against a stale deploy, the difference between the two gets folded
into the new offsets and you end up worse off than you started.

## How it works

[`SwerveAlignment`](../../src/main/java/frc/robot/subsystems/swerve/SwerveAlignment.java) exists
because CTRE's swerve telemetry only reports module angles *after* the magnet offset has been
applied, so the number you need is not recoverable from a normal log or dashboard. It publishes, per
module, the offset-applied position, the magnet offset actually programmed into the device, and the
difference between them, plus a connection flag and the encoder ID. The keys hang off
`SwerveAlignment.KEY_PREFIX`. Read that class for the exact set.

Those keys are visible in AdvantageScope too, which on its own beats opening Tuner X to check a
module.

The arithmetic the app applies is the one Tuner X shows as *Absolute Position No Offset*, negated and
wrapped into a half turn. The wrapped value is rounded to the CANcoder's own resolution, because
writing more precision than the device can measure just moves the error somewhere less visible.

The browser talks NetworkTables to the robot directly. The Node server only serves the page and
reads and writes the config file, and it binds to `127.0.0.1` so nothing on the pit network can ask it
to edit your source. The parsing and writing are in `tools/robot-app/server/lib/swerve-config.js`,
covered by `tools/robot-app/test/swerve-config.test.mjs`. Implementation notes are in
[`tools/robot-app/README.md`](../../tools/robot-app/README.md).

## When to fall back to Tuner X

The [Phoenix Tuner X](phoenix-tuner-x.md) procedure still works and is worth knowing. Reach for it
when the app cannot help: a CANcoder that will not report at all, a device that needs licensing or a
firmware update, or a bus that is not enumerating.
