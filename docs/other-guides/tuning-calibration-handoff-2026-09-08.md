# Tuning and calibration handoff: shot map, trims, and motor gains

*Audience: Whoever picks this work up next, a team student or a mentor. Written 2026-09-08 on
branch `2026-offseason-bot`. Section 3 is the work list. Each item is written so you can pick it up
with no context beyond this repo. Read sections 1 and 2 first, they are short. This is a snapshot:
the member names were accurate on 2026-09-08 and the code has moved since, so treat them as "look
near here".*

## 1. Where calibration stands

The robot app (`tools/robot-app`) closes the loop on two calibrations, and the robot code closes it
on a third. All three have the same shape: **measure on the robot, propose, write the number where
the code reads it, read it back, and have a check that notices when the two drift apart.**

|                       Calibration                       |                          Measured by                          |                                     Written to                                      |   Landed   |
|---------------------------------------------------------|---------------------------------------------------------------|-------------------------------------------------------------------------------------|------------|
| Swerve CANcoder offsets                                 | Swerve Align page, over NT4 from the browser                  | `swerve.configEncoderOffsets(...)` in `src/main/java/frc/robot/configs/OM2026.java` | August     |
| Camera roll, pitch, height; exposure, gain, black level | Cameras page, over each Limelight's HTTP API from the browser | the three camera config chains in `Vision.java`, and the camera's saved pipeline    | 2026-09-07 |
| Turret zero                                             | Turret camera against the pose heading, in `Vision.java`      | the turret encoder, live: a trim for slip, a one-step re-home for gross error       | 2026-09-06 |

Three calibrations are still done by editing Java, redeploying, and watching balls land.

### 1.1 The shot map is three pasted polynomials

`src/main/java/frc/rebuilt/ShotCalculator.java` holds three model records: the full-field hub fit,
the shop fit, and the feed fit. Each is a degree-3 surface in normalised distance and radial
velocity: ten coefficients for exit speed, ten for launch angle, ten for time of flight, plus the
normalisation means and deviations. **The tool and the data that produced these coefficients are
not in the repo**, and no current session knows where they are. They cannot be regenerated or
refit today. That is the single largest risk in this file.

The only calibration knob on a model is its `hoodOffsetDeg`, edited by hand. The last week of the
git log for that one number is a sequence of model switches and one-degree hood trims, and the
current sequence ends at -5 degrees. Every one of those was justified by someone watching where
balls landed, in words like "3 to 4 feet past the hub centre". **No log records where a shot
went.**

Two other constants convert the model to this robot: the RPM per m/s of exit speed, and its
multiplier. Neither has been measured on this robot.

`src/main/java/frc/rebuilt/targetFactories/HubTargetFactory.java` carries two
`InterpolatingTreeMap`s, `heightMap` and `distanceOffsetMap`, each seeded with a single zero entry.
They are the skeleton of a distance-indexed correction table that was never filled in.

### 1.2 The operator's trims persist, and every burst leaves a record

**Update 2026-09-19.** The turret trim is session-only again, starting at zero every boot and never
stored, after Chezy QM4 booted with +10 degrees in flash. A flywheel trim was added after QM4 and
removed after Q11 the same day: the operator walked it to both caps inside one match, and it made
the hood presses unreadable. The stored copy is deleted on load. See
`docs/tools/shot-log.md`.

**Done 2026-09-08.** This section originally described the trims evaporating on every deploy, and
item 3.1 below fixed it. Both trims are now stored with WPILib `Preferences` under
`ShotHoodTrimDeg` and `ShotTurretTrimDeg`, read once during robot construction, written inside each
trim command, capped, and cleared by an operator chord. A non-zero trim prints at `HIGH` priority
at boot. They survive a power cycle as well as a redeploy, which was the deliberate choice (section
4, question 3). The boot print and the reset chord are the mitigation, not an expiry.

Every burst now writes one row under the shot-calculator shot keys on the rising edge of the feed
gate, and every operator D-pad press writes one under the trim keys, carrying the distance of the
burst it is judging. There are no separate short, made, and long buttons. The D-pad already is the
outcome signal, and a made shot is the absence of a press (section 4, question 2). The full schema
and the join recipe are in `docs/tools/shot-log.md`.

What is still by hand is folding a settled trim into the model's `hoodOffsetDeg`. That is item 3.2.

`tools/robot-app/data/controls.json` said the hood step was 0.1 degrees until 2026-09-08, days after
the code had moved to 0.25. Fixed, but note the gap it showed: the drift check verifies that
bindings exist, not what their descriptions say.

### 1.3 Motor gains mean a redeploy per iteration

Every gain is a `final` in a mechanism's inner `*Config` class, applied once in the constructor. So
changing a gain is a code edit, a build, and a deploy. There is no faster path today, and that is
what item 3.3 is for.

Live tuning does exist. `Telemetry` extends DogLog, so `Telemetry.tunable(key, default)` hands back a
NetworkTables `DoubleSubscriber`, and DogLog 2026.5.0 also offers an `onChange` callback. Several
values use it, covering the feeder's speeds, the intake's agitation timings, the squeeze delay, and
the two shot-calculator speed constants. **None of them is a gain.** The older `TuneValue` class
that `docs/tools/pid-tuning.md` describes is not referenced anywhere in robot code.

Note the flag: the telemetry startup call passes `tunableOnFMS = true`, so tunables are read from
NetworkTables even with the FMS attached. That is deliberate and it stays. A value may need
changing pre-match while already attached, nobody touches the laptop keyboard during a match, and
an accidental mid-match edit is not a real risk. Do not add an FMS guard or a master enable to the
gains work.

The 2026-09-05 handoff (section 3.9) says the turret's `positionKv` is about double what a Kraken
through 39.78:1 needs, and to tune it from a log of commanded against measured turret velocity.
Nobody has, because that is a by-hand log analysis. Item 3.4 is the automated version.

### 1.4 What the logs carry

The set of loop-rate log keys changes as the robot changes, and
[Logging and Data Analysis](../tools/logging.md) is where the current set is written down. One fact
from this date still matters for the work below: the turret, the launcher, and the tower log their
applied voltage on every loop, and the hood does not, so kA is fittable for those three from any
log after 2026-09-08 and not for the hood.

## 2. Ground rules for anyone working here

**Build with the WPILib JDK.** A JDK 25 on PATH breaks Gradle with "Unsupported class file major
version 69". Use:

```bash
export JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; ./gradlew.bat build --offline -Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk"
```

Spotless runs on compile and reformats Java, Gradle, XML, and Markdown, so run the build before
committing and the formatter's changes are part of your commit. A full build is about 35 s.

**The app's split is strict.** The browser talks to the robot (NT4 via `client/lib/nt4.js`, and
Limelight HTTP via `client/lib/limelight.js`). The server talks to the source tree, and it binds to
`127.0.0.1` in `server/index.js` precisely because it can edit robot code. Never let a route write
to disk from anything the pit network can reach.

**Java writers are narrow.** `server/lib/swerve-config.js` and `server/lib/vision-config.js` each
understand exactly one code shape, replace numeric literals in place, leave every comment alone,
emit lines the way the AOSP formatter would so `spotlessCheck` stays green, add a provenance comment
naming the robot app and the date, and read the file back after writing so the UI shows what is on
disk. Do the same. Round-trip tests live in `test/*.test.mjs` and run against the real source file.

**Everything the app mirrors from the Java goes through the drift check.** `scripts/check-drift.mjs`,
surfaced at `/api/drift` and as a banner on every page. It reports. It never fails a build. If you
add a data file that mirrors Java constants, add a check for it.

**Logging has tiers, and the CPU is the constraint.** `Telemetry.log` goes to the wpilog only.
`Telemetry.logDash` also publishes to NetworkTables, at the slow tier.
[Loop time and CPU handoff 2026-09-05](loop-time-handoff-2026-09-05.md) explains why loop-rate
NetworkTables traffic is not allowed. New tunables and dashboard keys should be few and slow.

**Adding a page.** See "Adding a page" in `tools/robot-app/README.md`: a directory under
`client/pages/` with `index.html` and `main.js` becomes a Vite entry, and you add a nav entry to
`PAGES` in `client/lib/ui.js`. `npm test` runs the analysis maths. `npm run check` adds the drift
check.

**Logs.** Synced logs live in a sibling clone of `Spectrum3847/2026-Robot-Logs`, and the Logs page
pulls them off the robot. `client/lib/wpilog.js` parses in the browser or in Node.
`client/lib/log-model.js` normalises across log eras and finds enabled windows. `scripts/wpilog.js`
at the repo root is a dependency-free dump tool.

**Commits.** Imperative subject. The body says why, with the numbers that justified it. See
`docs/coding-conventions/commits-pull-requests.md`.

## 3. Work items, in recommended order

Each item stands alone. 3.1 makes every later item better because it is the data. 3.6 is a refactor
that 3.2 and 3.3 both want, so do it first if both are going to happen, otherwise fold it into
whichever comes first.

### 3.1 Persist the trims and record every shot (done 2026-09-08)

**What landed.** Both trims are read from `Preferences` during robot construction, and the nudge and
reset commands clamp, write flash, and log on every press. The reset chord is Start+Select on the
operator, live in every mode. Every burst writes one row, called from the orchestrator's feed-gate
update on the rising edge. Both streams are wpilog only, with one deliberate exception: the shot
index goes out over NetworkTables through `logDashAlways`, so the operator can watch bursts count up
on the Shooting tab and know records are being written. The turret trim moved from the wpilog-only
logger to the dashboard logger for the same reason. A trim that persists has to be readable before a
match.

**The mark buttons were dropped, deliberately.** The three-button plan above was answered with a
better signal: the operator already says what happened by trimming, so a hood-down press is "that
one went long" and a hood-up press is "that one fell short". Every press is logged with its
verdict, the index and age of the burst it is judging, and that burst's distance denormalised onto
the row, so a distance-binned fit is one pass over one channel. A made shot is the absence of a
press, which means makes are counted as bursts with no trim behind them. No new operator buttons
were spent on outcomes.

**The schema, the join recipe, and the argument for all of it** are in `docs/tools/shot-log.md`,
linked from `docs/index.md` and from `logging.md`.

**What 3.2 still needs from a human.** Nothing in the code. Take a practice log, join the two
streams, and the first reading worth doing by hand is in that doc's last section: a long or short
bias constant across distances is an exit-speed error, and one that grows with distance is an angle
error.

**Not verified on hardware.** Everything below the compile boundary was untested until someone
deployed: that `Preferences` survives a power cycle on that rio, that the boot print appears, and
that the gate's rising edge fires once per burst rather than several times if the readiness signal
chatters. Watch the shot index against the launcher RPM dips on the first practice log. If the
index climbs faster than the bursts, the gate needs a debounce on the closing edge, not on the
opening one.

### 3.2 A shooting page: trims to Java, outcomes to a correction table

**Goal.** The operator's trim becomes the model's trim with one button, and the shot records become
a distance-indexed correction instead of a single constant.

**Today.** Trims are hand-copied into the model's hood offset (1.1). `HubTargetFactory` has two
empty tables (1.1).

**Build.**

1. Server: `server/lib/shot-config.js`, or the shared rewriter of 3.6, that parses the model
   definitions in `ShotCalculator.java` and can rewrite the hood offset, the two speed constants,
   and which fit `WANTED_HUB_MODEL` names. Routes `GET /api/shot/target` and `POST /api/shot/apply`.
   Same rules as the other writers.
2. Client: `client/pages/shooting/`. Live over NT4: the two trims, the live model, distance, and
   wanted against actual RPM and hood. A "Bake trims into model" button that adds the hood trim to
   the live model's offset, writes, and tells the operator to zero the trim, or does it over NT if
   3.3's publish path exists.
3. From logs, once 3.1 data exists: a scatter of marks against distance, per model. Fit a correction
   per distance bin and propose a table of distance to hood correction and RPM correction.
4. Robot: a distance-indexed correction layer on top of the polynomial, one `InterpolatingTreeMap`
   per model, applied where the hood angle and flywheel speed are assembled in `ShotCalculator`.
   Written by the page as `put(d, v)` lines in a static block, the way `HubTargetFactory` already
   sketches it. Keep the polynomial. It carries the shoot-on-move shape that a table cannot.

**Acceptance.** Round-trip tests against the real `ShotCalculator.java`. Bake a trim, `spotlessCheck`
is green, the robot reads the new offset after deploy. The model switch works from the page. The
correction table is empty by default and the model shoots exactly as before when it is.

**Gotchas.** The hood offset is the last positional argument of a record constructor with three
array literals before it, so the parser must count brackets, not commas. Per-model trims exist
because each fit is wrong in its own way, and that is documented in the code near the model
definitions. Never write one trim to all three.

**Size.** Two to three days.

### 3.3 A gains page: live tunables with write-back

**Goal.** Tune a loop without redeploying, and when it is right, write the gains into the config
class with one button.

**Build.**

1. Robot: for the turret, the hood, and the launcher, register each gain as a DogLog tunable with an
   `onChange` that calls the gain, feedforward, or Motion Magic setter and then
   `Mechanism.applyTalonConfig` for the leader and each follower. No FMS guard and no master enable:
   tunables are meant to be editable pre-match while attached (1.3). Keys like `Tuning/Turret/kP`. Do
   not expose current limits from here. They are safety, not tuning.
2. Client: `client/pages/gains/`. One card per mechanism: gain fields bound to the tunables over
   NT4 (the read-only `nt4.js` needs a publish path. Add it deliberately, for `Tuning/*` topics
   only), a live plot of commanded against measured from the loop-rate keys in 1.4, and a step
   table: detect steps in the commanded angle or commanded speed and report rise time, overshoot,
   settle time, and steady-state error per step.
3. Server: rewrite the `final` gain fields in each `*Config` class, using the shared rewriter from
   3.6. Route `POST /api/gains/apply`. Provenance comment. Read back.
4. Mirror the gains into `data/robot-profile.json` and extend `checkLimits` in
   `scripts/check-drift.mjs` to verify them, so the Turret and Power pages can show them and the
   banner catches a hand edit.
5. Rewrite `docs/tools/pid-tuning.md`. As of 2026-09-08 it documents `TuneValue`, which nothing
   uses. Describe the DogLog tunables and this page instead.

**Acceptance.** Change kP on the page while enabled in the shop, watch the step table change, write
to Java, `spotlessCheck` green, redeploy, same behaviour.

**Gotchas.** `applyTalonConfig` is a blocking CAN transaction. Apply on change only, never per loop.
Followers need the same slot gains. The step detector must ignore turret unwrap slews. The Turret
page already excludes them, in `client/lib/analyze-turret.js`.

**Size.** Three days.

### 3.4 Characterize: fit feedforward from logs already on disk

**Goal.** kS, kV, and kA for any mechanism from any practice log, with no SysId routine and no
dedicated run.

**Build.** `client/pages/characterize/`, or a tab on Gains. Pick a log and a mechanism. Over the
enabled windows, least-squares fit `u = kS * sign(v) + kV * v + kA * a`, where `u` is the applied
voltage, `v` the measured velocity, and `a` its derivative. Report the three with residual RMS and
the fraction of samples above the noise floor, and flag when the mechanism never exceeded a few
percent of its range, because then there is no kV information in that log. Run it across every log
in the manifest and plot kV over time. Drift means belt wear or battery.

**Gotchas.**

* The turret, launcher, and tower log voltage on every loop since 2026-09-08, so kA is fittable for
  them from any log after that date. Older logs, and the hood, have voltage only at the slow tier.
  Interpolate it onto the fast timestamps for kS and kV, and do not report kA.
* Units must match the slot. The turret is Motion Magic voltage, so volts per rot/s of mechanism
  velocity after the sensor ratio. Read `Mechanism` and check which request each setter sends before
  choosing volts or amps. Several setters are torque-current.
* The turret's commanded rot/s log key is the *wanted* rate, not a measurement. Use the measured
  velocity key. On older logs, differentiate the position key.
* The turret's kV is expected to come out near 5 V per rot/s (09-05 handoff, 3.9). If the fit says
  10, the handoff was wrong, not the fit. Record the result here either way.

**Acceptance.** A synthetic log with known kS and kV (extend `test/helpers/make-log.mjs`) comes
back within a few percent. Run on the 2026-09-06 and 09-07 logs and write the turret result into
section 5 of this document.

**Size.** One to two days.

### 3.5 Shot model regression in sim

**Goal.** Before a model or trim change goes to the field, a test says where the shots land.

**Today.** `RobotSim.createSimBallLaunch` fires `ShotCalculator` parameters into `FuelPhysicsSim`,
and the sim scores the hub through its own scoring counters. `src/test/java/frc/rebuilt/` already
has a `ShotCalculatorTest` and a `FuelPhysicsSimTest`.

**Build.** A JUnit test, or a Gradle task if it is slow, that for each model and ten distances
across the model's range, stationary and at plus and minus 2 m/s radial, computes the parameters,
launches a ball in the sim, steps physics until the ball sleeps, and reports the landing offset from
the hub centre. Assert a tolerance for the ceiling model, which is known to score. Report only for
the others until 3.1 data says what the tolerance should be.

**Gotchas.** The parameters read the swerve and the orchestrator, so the test needs those stubbed,
or it needs a variant that takes a pose and speeds directly. The sim's drag and Magnus constants
are team 5962's, not measured on our fuel, so a systematic sim-against-field offset is expected. It
is itself worth recording once 3.1 gives field data.

**Size.** One to two days.

### 3.6 Shared Java literal rewriter

The app has the same pipeline three times over, for swerve, for vision, and soon for shot and
gains: parse a Java file, find a call or a field, replace a numeric literal in place, wrap the line
the way the formatter would, add provenance, read back. Factor it into `server/lib/java-rewrite.js`,
with `findClassBody(source, name)`, `findField(body, name)`, `findCall(source, name)`,
`splitArguments(text)`, `replaceLiteral(source, span, text)`, `wrapForFormatter(line, indent)`, and
`provenance(prefix, date)`. `check-drift.mjs` already has a `classBody` helper and a `fieldValue`
helper that should share it. Port `vision-config.js` onto it under its existing tests before
writing anything new on top.

**Size.** One day.

### 3.7 Smaller items

* **Hood zero.** The encoder zero sits a fraction of a degree below the hard stop, which is why
  `Hood.homeRestToleranceDegrees` exists. A homing routine (drive to the stop at low current, zero,
  back off) on a disabled-mode button would remove that workaround and the intake extension's
  zero-at-max chord. Model it on the turret's own zero command.
* **The exit-speed constants.** The conversion from the model's exit speed to flywheel RPM has
  never been measured on this robot. Once 3.1 gives outcomes: a long or short bias that is constant
  across distances is a speed error, and one that grows with distance is an angle error. The
  shooting page can say which.
* **Find the fit tool.** Ask whoever built the polynomial models where the fitting notebook and the
  input data are. If they exist, they belong in `tools/shot-fit/` with the data. If they do not,
  the correction table in 3.2 is the only way to move the model, and this document should say so.

## 4. Open questions for the humans

1. Where are the polynomial fit script and its data? (3.7)
2. ~~Which operator buttons may the shot marks take?~~ **Answered 2026-09-08: none.** No marks. The
   D-pad trim presses are the outcome signal, since a shot going long already gets the hood trimmed
   down and one falling short gets it trimmed up. A made shot is the absence of a press.
   Start+Select, two buttons nothing else used, became the trim reset.
3. ~~Should trims persist across power cycles or only across deploys?~~ **Answered 2026-09-08:
   across power cycles, and then the settled value gets folded into the code (3.2).** Plain
   `Preferences`, with a boot print and the Start+Select reset as the mitigation for a stale trim.
   No expiry. Zeroing a trim after some minutes disabled would surprise the operator in the middle
   of a session they thought was still calibrated.
4. Is a publish path from the app to the robot over NT4 acceptable? `nt4.js` is deliberately
   read-only today. 3.3 needs to write `Tuning/*`. The alternative is typing values into Elastic,
   which works but loses the step table's context.

## 5. Measurements worth remembering

* **Model hood sensitivity:** about 1 degree of hood per foot of range near 2.5 m. This is why a
  flat hood trim cannot fix both ends of the range at once.
* **Shots taken so far:** 2.2 to 2.6 m, with the hood at 18 to 20 degrees (09-05 handoff, 3.1b).
* **Flywheel droop during a burst** held within a 200 RPM window after about 0.25 s of spin-up from
  650 to 2600 RPM. That is what set the planned feeder start tolerance.

The tuning constants themselves live in `ShotCalculator` and in each mechanism's `*Config` class.
Read them there.
