# Shot records and trim events

*Audience: Reference. Assumes you can read a wpilog. Read [Logging and Data Analysis](logging.md)
first if `Telemetry.log` and the slow tier are new to you. Landed 2026-09-08; item 3.1 of the
[Tuning and Calibration Handoff](../other-guides/tuning-calibration-handoff-2026-09-08.md).*

Before this existed, a practice session produced no record of where a shot went. The only trace of
an afternoon of tuning was that someone remembered "3 to 4 feet past the hub centre" and edited the
hood calibration down a degree. The -4, then -5, in the git log for that number is what a whole day
of shooting reduced to.

Now every burst writes a row saying what was aimed, and every operator trim press writes a row
saying how it went. Pair the two and a practice session is a dataset.

## Where the rows come from

Both streams are written by `ShotCalculator`. The per-burst row is `ShotCalculator.recordShot`, which
`SuperStructure.updateFeedGate()` calls on the rising edge of the feed gate opening. That is the
first loop fuel is allowed into the flywheel, and therefore the last loop on which the aim was still
a prediction rather than a result. The per-press row is written inside `ShotCalculator.nudgeTrim`,
so a row exists for the D-pad and for the Start plus Select reset and for nothing else.

Read those two methods for the columns themselves. They are short, commented, and authoritative.
This page is about what the columns mean once you have them, which is the part a list of names
cannot tell you.

## The outcome signal is the D-pad

There are no "made" and "missed" buttons, and adding them would be redundant. The operator already
tells the robot what happened: a shot going long gets the hood trimmed down, a shot falling short
gets it trimmed up, and a shot that missed to one side gets the turret nudged. A shot nobody
corrects is a shot that went in.

So the D-pad *is* the outcome log. Two consequences worth carrying into any analysis:

* **A made shot is the absence of a row.** To count makes, count bursts with no trim press behind
  them, not rows in the trim stream. A naive count of verdicts silently scores every made shot as
  a miss.
* **The verdict is the operator's, not the robot's.** It is a judgement made from behind a driver
  station about a ball that landed a second and a half ago, possibly after a burst of five. Treat
  it as a noisy label, which is exactly what a distance-binned fit is good at absorbing.

## Reading the shot row

The distance column is the real distance to the target, not the shoot-on-move virtual one, and it is
the one to bin on. The virtual distance is logged alongside it because it is what the polynomial was
actually evaluated at, and comparing the two is how you tell a fit that is being extrapolated from
one that is not.

The model column names which fit was in use, and a feed shot reports the feed model rather than the
hub model, so a session that mixed feed and hub shooting has two populations in one column. Read
the separate hub-model key when you need to know the hub fit specifically.

Three things that are deliberately *not* in the row:

* **Balls per burst.** Count them afterwards from the dips in the launcher RPM, which is kept at
  loop rate for exactly this. A five-ball burst and a one-ball burst produce identical rows.
* **The turret zero split.** The vision turret-zero keys are already logged on the slow tier and join
  on time. Joining beats copying, because two copies of the same number drift apart. And do join
  them, because a shot taken while the trim was absorbing a pose heading error is a shot whose
  turret error means something different from one taken against a settled pose.
* **A made or missed flag.** See the previous section.

One sign trap: the shot row's turret error is measured minus commanded, matching the super
structure's tracking error, while the turret subsystem's own position error is logged with the
opposite sign. Do not mix the two. Which way in the world a positive value points is not documented
on the turret, so read it as a magnitude unless you have checked it yourself.

## Reading the trim row

`Verdict` is named for where the ball went, not which way the trim moved. Hood up means the ball
fell **short**, and a counter-clockwise turret correction means the ball landed **clockwise** of
the target. The turret verdicts are safe to read that way because the turret offset is added to a
field-relative rotation, where positive is counter-clockwise by WPILib convention. The hood verdict
is a straight reading of "which way did the operator have to push it".

The press is joined to the burst it judges by shot index, and the age of that burst is logged rather
than thresholded, because how long an operator takes to judge a shot is a fact about your operator,
not about this code. Look at the distribution before choosing a cut-off.

Two rows that look like noise and are not:

* **A press at the cap still gets a row.** It is a press that changed nothing because the trim was
  already at its limit. Dropping those quietly biases the dataset toward whichever direction still
  had room, which is exactly the bias a distance-binned fit would then report as a model error.
* **A press in the pit with no burst behind it** carries a negative shot index and an infinite age.
  Drop it.

## Reading the rows in a wpilog

Both streams are wpilog-only, sparse, and one row per event. DogLog skips a record whose value has
not changed, so a burst at the same distance with the same model writes the index and the timestamp
and little else.

**So do not expect every key to be present at every row.** Read a row by taking each key's last
value at or before that row's timestamp. `valueAt(series, t)` in the robot app's
`client/lib/analyze-turret.js` does exactly that, and is already tested against change-only series.

A minimal join, in the shape a distance-binned fit wants:

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

Note what that join does *not* need: the shot row itself. The distance is denormalised onto the trim
row precisely so a distance-binned fit is one pass over one channel with no join at all. Reach for
the shot row when you want the aim behind the verdict, wanted against actual RPM, the turret error,
the pose.

`Long` at 2.3 m and `Short` at 3.1 m in the same session is not a contradiction. It is the shape a
single hood calibration constant cannot represent, and it is the argument for the
distance-indexed correction table sketched in `HubTargetFactory`.

## The hood trim persists

The hood trim is stored with WPILib `Preferences` under a hood-specific key, written inside the
trim commands and read once by `ShotCalculator.loadPersistedTrims()` during robot construction. A
redeploy no longer zeroes it. The `Preferences` file lives in the RIO's flash, so a trim survives a
power cycle too.

That is the point and it is also the risk. A trim dialled in against a shop ceiling last week
silently applies at the next event. Three things guard against it:

1. A boot print at high priority whenever the trim is non-zero, naming the value.
2. **Operator Start plus Select** zeroes both trims and clears the stored value. It works enabled or
   disabled.
3. The trim is capped at `ShotCalculator.MAX_TRIM_DEG` on load and on every press. Ten degrees of
   hood is about ten feet of range, so the cap is not there to stop the operator. It is there so a
   corrupt or hand-edited preference cannot swing the shot ten degrees off target unnoticed. The
   hood is clamped again downstream against its soft limits.

There is deliberately **no expiry**. An expiry that zeroed a trim after some minutes disabled would
surprise the operator in the middle of a session they thought was still calibrated, which is worse
than a stale trim that the boot print announced.

In simulation every trim is zero instead, via `ShotCalculator.zeroTrimsForSimulation()`. Both
operator trims start at zero, because the sim's `Preferences` file on the laptop would otherwise
bring back whatever the last sim session nudged, and each shot model's own hood calibration is
switched off, because it corrects how the real robot's shots land and the simulated ball flies the
fitted model. Nudges still work for the rest of the sim session.

**The turret trim is not stored.** On 2026-09-19 that was changed deliberately: Chezy QM4 booted
with the maximum turret trim sitting in flash, left there by an operator pressing D-pad right at a
turret that was parked waiting for a pose. A turret trim corrects one match's pose error, so it
starts at zero every boot, and `loadPersistedTrims()` deletes any stored copy an older build left
behind. Do not reintroduce persistence for it without thinking about how a trim gets set before the
match has a pose.

Both trims are on the dashboard on the shooting tab, next to a counter of shots logged, which is the
burst index. That counter counting up is how you know records are being written at all.

## One reading worth doing by hand

The correction this data feeds back into the model is item 3.2 of the
[handoff](../other-guides/tuning-calibration-handoff-2026-09-08.md): a shooting page that bakes the
operator's trim into the model's hood calibration with one button, and fits the marks into a
distance-indexed hood and RPM correction table. Until that exists, the rows accumulate and the trim
persists, which is already the difference between an afternoon of shooting and a day of it.

From a single session, one reading is worth doing by hand. A long or short bias that is **constant
across distances** is an exit-speed error, so the model's RPM per m/s constant is wrong. One that
**grows with distance** is a launch-angle error, so the polynomial's angle surface or the hood
calibration is wrong. Neither of the two speed constants has ever been measured on this robot. A
bias at **one end of the range only**, which is the shape seen on 2026-09-19, is what the
distance-indexed near-shot RPM drop exists for. The drop actually applied is logged every shot, and
the knob that sets it is a DogLog tunable keyed under `ShotCalc/NearShotRpmDrop`. See the
[season guide](../other-guides/2026-season-specific.md#shot-map-the-near-shot-rpm-drop).

## See also

* [Logging and Data Analysis](logging.md), the tiers, and why loop-rate NetworkTables traffic is
  not allowed.
* [Elastic Dashboard](elastic.md), the shooting tab these keys surface on.
* [Tuning and Calibration Handoff](../other-guides/tuning-calibration-handoff-2026-09-08.md), where
  this fits and what consumes it next.
