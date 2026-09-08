# Shot Records and Trim Events

*Audience: Reference. Assumes you can read a wpilog. Read [Logging and Data Analysis](logging.md)
first if `Telemetry.log` and the slow tier are new to you. Landed 2026-09-08; item 3.1 of the
[Tuning and Calibration Handoff](../other-guides/tuning-calibration-handoff-2026-09-08.md).*

Before this existed, a practice session produced no record of where a shot went. The only trace of
an afternoon of tuning was that someone remembered "3 to 4 feet past the hub centre" and edited
`hoodOffsetDeg` down a degree. The -4, then -5, in the git log for that constant is what a whole
day of shooting reduced to.

Now every burst writes a row saying what was aimed, and every operator trim press writes a row
saying how it went. Pair the two and a practice session is a dataset.

## The outcome signal is the D-pad

There are no "made" and "missed" buttons, and adding them would be redundant. The operator already
tells the robot what happened: a shot going long gets the hood trimmed down, a shot falling short
gets it trimmed up, and a shot that missed to one side gets the turret nudged. A shot nobody
corrects is a shot that went in.

So the D-pad *is* the outcome log. That has two consequences worth knowing:

* **A made shot is the absence of a row.** To count makes, count bursts with no trim press behind
  them, not rows in the trim stream.
* **The verdict is the operator's, not the robot's.** It is a judgement made from behind a driver
  station, about a ball that landed a second and a half ago, possibly after a burst of five. Treat
  it as a noisy label, which is exactly what a distance-binned fit is good at absorbing.

## `ShotCalc/Shot/*` — one row per burst

Written by `ShotCalculator.recordShot(boolean)`, called from `SuperStructure.updateFeedGate()` on
the rising edge of `feedGateOpen`. That is the first loop fuel is allowed into the flywheel, and so
the last loop on which the aim was still a prediction rather than a result.

| Key | Meaning |
| --- | --- |
| `Index` | Bursts since boot. The pairing key. The one key here that is also on NetworkTables, so the operator can see it counting up. |
| `TimestampSeconds` | FPGA time. |
| `MatchTimeSeconds` | Match clock, for lining a row up against video. |
| `DistanceMeters` | `distanceNoLookahead`: the real distance to the target, not the shoot-on-move virtual one. Bin on this. |
| `LookaheadDistanceMeters` | The virtual-target distance the polynomial was actually evaluated at. |
| `WantedRPM`, `ActualRPM` | Flywheel commanded and measured. |
| `WantedHoodDeg`, `ActualHoodDeg` | Hood commanded (trim included) and measured. |
| `TurretErrorDeg` | Measured minus commanded, matching `Turret/TrackingErrorDegrees`. `Turret/PositionError` is logged with the *opposite* sign (`Turret.java` 309), so do not mix the two. Which way in the world a positive value points is not documented on the turret; read it as a magnitude unless you have checked. |
| `ExitSpeedMs`, `TimeOfFlightSeconds` | Straight off the polynomial. |
| `RadialVelocityMs`, `TangentialVelocityMs` | Launcher velocity components. The model is a surface in distance and radial velocity, so a row without these cannot be checked against the fit. |
| `Model`, `HoodModelOffsetDeg` | Which fit was in use and its own hood trim. This is `FEED_MODEL` on a feed shot, unlike `ShotCalc/HubPolyModel`, which always names the hub model. |
| `HoodTrimDeg`, `TurretTrimDeg` | The operator's live trims at the moment of the shot. |
| `FeedShot`, `InRange`, `PoseTrusted` | Whether it was a feed shot, whether the distance was inside the fit's range, and whether vision had accepted an estimate recently enough to believe the distance at all. |
| `Pose` | Robot pose. |

**Balls per burst are not in the row.** Count them afterwards from the dips in `Launcher/RPM`,
which is kept at loop rate for exactly this (`Launcher.java` 203).

**The turret-zero split is not in the row either.** `Vision/TurretZero/PoseHeadingErrorDeg` and
`TurretOnlyErrorDeg` are already logged at 10 Hz and join on time. Copying them in would let two
copies of the same number drift apart. Join, don't duplicate — and do join them, because a shot
taken while the trim was absorbing a pose heading error is a shot whose turret error means
something different.

## `ShotCalc/Trim/*` — one row per press

Written by `ShotCalculator.nudgeTrim()`, so a row exists for the D-pad and for the Start+Select
reset, and for nothing else.

| Key | Meaning |
| --- | --- |
| `Index` | Presses since boot. |
| `TimestampSeconds` | FPGA time. |
| `Axis` | `Hood` or `Turret`. |
| `DeltaDeg` | How far the trim actually moved. Zero when it was already at `MAX_TRIM_DEG`. |
| `ValueDeg` | The trim's new value. |
| `Verdict` | `Short`, `Long`, `MissedCW`, `MissedCCW`, `AtLimit`, or `Reset`. |
| `ShotIndex` | The burst this press is judging, or -1 if there has not been one. |
| `SecondsSinceShot` | How old that burst is. Infinite if there has not been one. |
| `ShotDistanceMeters` | That burst's distance, denormalised onto the row so a distance-binned fit needs no join at all. |

`Verdict` is named for where the ball went, not which way the trim moved: hood up means the ball
fell **short**, and a counter-clockwise turret correction means the ball landed **clockwise** of
the target. The turret verdicts are safe to read that way because `TURRET_ANGLE_OFFSET` is added to
a field-relative `Rotation2d`, where positive is counter-clockwise by WPILib convention — unlike
`TurretErrorDeg` above, which lives in the mechanism's own undocumented frame.

`AtLimit` is a press that changed nothing because the trim was already capped. It still gets a row.
Dropping it would quietly bias the dataset toward whichever direction still had room.

### How stale is too stale?

`SecondsSinceShot` is logged rather than thresholded, because how long an operator takes to judge a
shot is a fact about your operator, not about this code. Look at the distribution before picking a
cut-off. A press in the pit with no burst behind it carries index -1 and an infinite age, and
should be dropped.

## Reading the rows

Both streams are wpilog-only, sparse, and one row per event. DogLog skips a record whose value has
not changed, so a burst at the same distance with the same model writes `Index` and
`TimestampSeconds` and little else.

**So do not expect every key to be present at every row.** Read a row by taking each key's last
value at or before that row's timestamp. `valueAt(series, t)` in
`tools/robot-app/client/lib/analyze-turret.js` does exactly that, and is already tested against
change-only series.

A minimal join, in the shape the Shooting page will want:

```js
import { valueAt } from "../../lib/analyze-turret.js";

// `model` is a LogModel; its ch() adds the /Robot/ prefix these keys are logged under.
const distance = model.ch("ShotCalc/Trim/ShotDistanceMeters");
const age = model.ch("ShotCalc/Trim/SecondsSinceShot");
const fit = model.ch("ShotCalc/Shot/Model");

// One row per trim press, carrying the burst it judged.
const rows = model.ch("ShotCalc/Trim/Verdict").map(([t, verdict]) => ({
    t,
    verdict,
    distance: valueAt(distance, t),
    secondsSinceShot: valueAt(age, t),
    fit: valueAt(fit, t),
}));
```

Note what that join does *not* need: the shot row itself. `ShotDistanceMeters` is denormalised onto
the trim row precisely so a distance-binned fit is one pass over one channel. Reach for
`ShotCalc/Shot/*` when you want the aim behind the verdict — wanted against actual RPM, the turret
error, the pose.

`Long` at 2.3 m and `Short` at 3.1 m in the same session is not a contradiction; it is the shape a
single `hoodOffsetDeg` cannot represent, and the argument for the distance-indexed correction table
sketched in `HubTargetFactory`.

## The trims now persist

`HOOD_ANGLE_OFFSET` and `TURRET_ANGLE_OFFSET` are stored with WPILib `Preferences` under
`ShotHoodTrimDeg` and `ShotTurretTrimDeg`, written inside the trim commands and read once by
`ShotCalculator.loadPersistedTrims()` during robot construction. A redeploy no longer zeroes them.

`Preferences` lives in the rio's flash, so **a trim survives a power cycle too**. That is the point
and it is also the risk: a trim dialled in against a shop ceiling last week silently applies at the
next event. Three things guard against it.

1. A boot print at `HIGH` priority whenever either trim is non-zero, naming both values.
2. **Operator Start+Select** zeroes both and clears the stored values. Works enabled or disabled.
3. Both trims are capped at `MAX_TRIM_DEG`, 10 degrees, on load and on every press. Ten degrees of
   hood is about ten feet of range, so the cap is not there to stop the operator; it is there so a
   corrupt or hand-edited preference cannot swing the turret ten degrees off target unnoticed. The
   hood is clamped again downstream against its soft limits. The turret trim is not, which is the
   axis this actually protects.

There is deliberately **no expiry**. An expiry that zeroed a trim after some minutes disabled would
surprise the operator in the middle of a session they thought was still calibrated, which is worse
than a stale trim the boot print announced.

Both trims are on the dashboard, on the Shooting tab, next to `Shots Logged` — which is `Index`,
and which counting up is how you know records are being written at all.

## What this is for

The correction this data feeds back into the model is item 3.2 of the
[handoff](../other-guides/tuning-calibration-handoff-2026-09-08.md): a Shooting page that bakes the
operator's trim into the model's `hoodOffsetDeg` with one button, and fits the marks into a
distance-indexed hood and RPM correction table. Until that exists, the rows accumulate and the
trims persist, which is already the difference between an afternoon of shooting and a day of it.

One reading is worth doing by hand now, from a single session: a long or short bias that is
**constant across distances** is an exit-speed error, so `RPM_PER_MPS` or `MPS_FACTOR` is wrong; one
that **grows with distance** is a launch-angle error, so the polynomial's angle surface or
`hoodOffsetDeg` is wrong. Neither of those two speed constants has ever been measured on this robot.

## See Also

* [Logging and Data Analysis](logging.md) — the tiers, and why loop-rate NetworkTables traffic is
  not allowed.
* [Elastic Dashboard](elastic.md) — the Shooting tab these keys surface on.
* [Tuning and Calibration Handoff](../other-guides/tuning-calibration-handoff-2026-09-08.md) —
  where this fits, and what consumes it next.
