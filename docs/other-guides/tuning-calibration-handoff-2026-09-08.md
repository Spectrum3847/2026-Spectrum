# Tuning and calibration handoff: shot map, trims and motor gains

*Audience: whoever picks up this work next, human or Claude session. Written 2026-09-08 on branch
`2026-offseason-bot` at `6ffd115`, the day after the Cameras page landed. Section 3 is the work
list. Each item is written so a fresh agent can take it with no context beyond this repo. Read
sections 1 and 2 first; they are short. Line numbers are as of `6ffd115` and will drift, so treat
them as "look near here".*

## 1. Where calibration stands

The robot app (`tools/robot-app`) closes the loop on two calibrations, and the robot code closes it
on a third. All three have the same shape: **measure on the robot, propose, write the number where
the code reads it, read it back, and have a check that notices when the two drift apart.**

| Calibration | Measured by | Written to | Landed |
| --- | --- | --- | --- |
| Swerve CANcoder offsets | Swerve Align page, over NT4 from the browser | `swerve.configEncoderOffsets(...)` in `src/main/java/frc/robot/configs/OM2026.java` | August |
| Camera roll, pitch, height; exposure, gain, black level | Cameras page, over each Limelight's HTTP API from the browser | the three `LimelightConfig` chains in `src/main/java/frc/robot/subsystems/vision/Vision.java`, and the camera's saved pipeline | 2026-09-07 |
| Turret zero | Turret camera against the pose heading, in `Vision.java` | the turret encoder, live: a trim for slip, a one-step re-home for gross error | 2026-09-06 |

Three calibrations are still done by editing Java, redeploying, and watching balls land.

### 1.1 The shot map is three pasted polynomials

`src/main/java/frc/rebuilt/ShotCalculator.java` holds three `PolyModel` records (record at line
161): the full-field hub fit `HUB_MODEL` (177), the shop fit `CEILING_3M_HUB_MODEL` (230) and
`FEED_MODEL` (281). Each is a degree-3 surface in normalised distance and radial velocity: ten
coefficients for exit speed, ten for launch angle, ten for time of flight, plus the normalisation
means and deviations. **The tool and the data that produced these coefficients are not in the
repo**, and no current session knows where they are. They cannot be regenerated or refit today.

The only calibration knob is `hoodOffsetDeg`, the last field of each record, edited by hand. The
last week of the git log for that one number, oldest first:

| Commit | Change |
| --- | --- |
| `8a18787` | Use the full-field hub model instead of the shop ceiling fit |
| `d8ac503` | Go back to the 3 m ceiling shot model |
| `4f29410` | Give each shot model its own hood trim; hub gets -4 deg |
| `eb3d300` | Switch back to the full-field hub model to test its -4 deg trim |
| `abdc95c` | Trim the hub hood offset one more degree, to -5 |

Every one of those was justified by someone watching where balls landed ("3 to 4 feet past the hub
centre"). No log records where a shot went.

Two other constants convert the model to this robot: `RPM_PER_MPS` = 365 (line 124) and
`MPS_FACTOR` = 1 (121). Neither has been measured on this robot.

`src/main/java/frc/rebuilt/targetFactories/HubTargetFactory.java` (lines 15 and 23) carries two
`InterpolatingTreeMap`s, `heightMap` and `distanceOffsetMap`, each seeded with a single zero entry.
They are the skeleton of a distance-indexed correction table that was never filled in.

### 1.2 The operator's trims persist, and every burst leaves a record

**Done 2026-09-08.** This section described the trims evaporating on every deploy; item 3.1 below
fixed it. `HOOD_ANGLE_OFFSET` and `TURRET_ANGLE_OFFSET` are stored with WPILib `Preferences` under
`ShotHoodTrimDeg` and `ShotTurretTrimDeg`, read once by `ShotCalculator.loadPersistedTrims()`
during robot construction, written inside each trim command, capped at `MAX_TRIM_DEG` (10 deg), and
cleared by an operator Start+Select chord. A non-zero trim prints at `HIGH` priority at boot. They
survive a power cycle as well as a redeploy, which was the deliberate choice (section 4, question
3); the boot print and the reset chord are the mitigation, not an expiry.

Every burst now writes one row under `ShotCalc/Shot/*` on the rising edge of the feed gate, and
every operator D-pad press writes one under `ShotCalc/Trim/*` carrying the distance of the burst it
is judging. There are no separate short/made/long buttons: the D-pad already is the outcome signal,
and a made shot is the absence of a press (section 4, question 2). Full schema and the join recipe
are in `docs/tools/shot-log.md`.

What is still by hand is folding a settled trim into the model's `hoodOffsetDeg`. That is item 3.2.

`tools/robot-app/data/controls.json` said the hood step was 0.1 deg until 2026-09-08, days after
the code had moved to 0.25 (`HOOD_OFFSET_STEP_DEG`, line 86). Fixed, but note the gap it showed:
the drift check verifies that bindings exist, not what their descriptions say.

### 1.3 Motor gains mean a redeploy per iteration

Every gain is a `final` in a mechanism's inner `*Config` class and is applied once in the
constructor:

| Mechanism | Gains | Control request |
| --- | --- | --- |
| Turret | `positionKp` 800, `positionKi` 100, `positionKv` 10, `positionKs` 0.6; MotionMagic cruise 0.25, accel 0.5 (`Turret.java` 67-79, applied 98-100) | `setMMPosition` sends `MotionMagicVoltage` (`Mechanism.java` 1336-1341), so kV is volts per rot/s; its Javadoc has said so since 2026-09-08 |
| Hood | `positionKp/Ki/Kd`, `positionKs/Kv/Ka/Kg`, MotionMagic cruise/accel/jerk (`Hood.java` 66-70); peak 3 V | `setPosition` |
| Launcher | `velocityKp` 0.5, `velocityKv` 0.1425, `velocityKs` 0 (`Launcher.java` 47-49, applied 64-65) | `setVelocityRPM` |

Live tuning exists but is not used for gains. `Telemetry extends DogLog`
(`src/main/java/frc/spectrumLib/telemetry/Telemetry.java` 44), so
`Telemetry.tunable(key, default)` returns an NT `DoubleSubscriber`, and DogLog 2026.5.0 also has
`tunable(key, default, DoubleConsumer onChange)`. Six values use it today (`Launcher/OnTheFlySpeed`,
`DyeRotor/IndexMaxFeederRPM`, three `IntakeExtension/Agitate*`, `SuperStructure/SecondsToSqueeze`).
None is a gain. The older `TuneValue` class that `docs/tools/pid-tuning.md` describes is not
referenced anywhere in robot code.

Note the flag: `Telemetry.start(false, true, false, true, false, true, ...)` in `Robot.java` 145
passes `tunableOnFMS = true`, so tunables are read from NT even with the FMS attached. That is
deliberate and stays: a value may need changing pre-match while already attached to the FMS, and
nobody touches the laptop keyboard during a match, so an accidental mid-match edit is not a real
risk. Do not add an FMS guard or a master enable to the gains work.

The 2026-09-05 handoff (section 3.9) says turret `positionKv` is about double what a Kraken through
39.78:1 needs and to "tune from a log of commanded vs measured turret velocity". Nobody has, because
that is a by-hand log analysis.

### 1.4 What the logs already carry

Every `Mechanism` logs voltage, stator current, supply current, temperature and connectivity under
`<Name>/*` at the 10 Hz slow tier (`Mechanism.logDiagnostics`). Since 2026-09-08 a config flag,
`Config.fastOutputLogging`, moves a mechanism's output status frames to 100 Hz and logs its
`<Name>/Voltage` on every loop; the turret, launcher and launcher tower set it, so their applied
voltage lines up sample for sample with their velocity. The hood does not, and its voltage stays at
10 Hz. The aimed mechanisms log at loop rate (50 Hz): `Turret/CommandedDegrees`,
`Turret/PositionDegrees`, `Turret/PositionError`, `Turret/CommandedRotPerSec` (the wanted rate)
and `Turret/VelocityRotPerSec` (the measured one); `Hood/CommandedDegrees`,
`Hood/PositionDegrees`, `Hood/RPM`; `Launcher/CommandedRPM`, `Launcher/RPM`;
`LauncherTower/CommandedRPM`, `LauncherTower/RPM`. `ShotCalc/*` logs
fifteen keys at loop rate while launching and 10 Hz otherwise (`ShotCalculator.java` around
505-531), including distance, wanted RPM, wanted hood, exit speed, time of flight, model name and
both trims. `SuperStructure/ShotReady/*` logs the feed gate and every input to it. Since 2026-09-08 the
outcome is recorded too, as two sparse per-event streams rather than another sampled signal:
`ShotCalc/Shot/*`, one row on each rising edge of the feed gate, and `ShotCalc/Trim/*`, one row per
operator trim press with the distance of the burst it judges. See `docs/tools/shot-log.md`.

## 2. Ground rules for anyone working here

**Build with the WPILib JDK.** A JDK 25 on PATH breaks Gradle with "Unsupported class file major
version 69". Use:

```bash
export JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; ./gradlew.bat build --offline -Dorg.gradle.java.home="C:/Users/Public/wpilib/2026/jdk"
```

Spotless runs on compile and reformats Java, Gradle, XML and Markdown, so run the build before
committing and the formatter's changes are part of your commit. A full build is about 35 s.

**The app's split is strict.** The browser talks to the robot (NT4 via `client/lib/nt4.js`,
Limelight HTTP via `client/lib/limelight.js`); the server talks to the source tree. The server
binds to `127.0.0.1` (`server/index.js` 147) precisely because it can edit robot code. Never let
a route write to disk from anything the pit network can reach.

**Java writers are narrow.** `server/lib/swerve-config.js` and `server/lib/vision-config.js` each
understand exactly one code shape, replace numeric literals in place, leave every comment alone,
emit lines the way `googleJavaFormat().aosp()` would so `spotlessCheck` stays green, add a
`// ... by the Spectrum robot app on <date>` provenance comment, and read the file back after
writing so the UI shows what is on disk. Do the same. Round-trip tests live in `test/*.test.mjs`
and run against the real source file.

**Everything the app mirrors from the Java goes through the drift check.**
`scripts/check-drift.mjs`, surfaced at `/api/drift` and as a banner on every page. It reports; it
never fails a build. If you add a data file that mirrors Java constants, add a check for it.

**Logging has tiers, and the CPU is the constraint.** `Telemetry.log` goes to the wpilog only;
`Telemetry.logDash` also publishes to NT, at the 10 Hz slow tier. The 2026-09-05 loop-time handoff
(`loop-time-handoff-2026-09-05.md`) explains why loop-rate NT traffic is not allowed. New tunables
and dashboard keys should be few and slow.

**Adding a page.** `tools/robot-app/README.md`, "Adding a page": a directory under `client/pages/`
with `index.html` and `main.js` becomes a Vite entry; add a nav entry to `PAGES` in
`client/lib/ui.js`. `npm test` runs the analysis maths; `npm run check` adds the drift check.

**Logs.** Synced logs live in a sibling clone of `Spectrum3847/2026-Robot-Logs`; the Logs page
pulls them off the robot. `client/lib/wpilog.js` parses in the browser or Node;
`client/lib/log-model.js` normalises across log eras and finds enabled windows. `scripts/wpilog.js`
at the repo root is a dependency-free dump tool.

**Commits.** Imperative subject; the body says why, with the numbers that justified it. See
`docs/coding-conventions/commits-pull-requests.md`.

## 3. Work items, in recommended order

Each item stands alone. 3.1 makes every later item better because it is the data. 3.6 is a refactor
that 3.2 and 3.3 both want; do it first if both are going to happen, otherwise fold it into
whichever comes first.

### 3.1 Persist the trims and record every shot — DONE 2026-09-08

**What landed.** `ShotCalculator.loadPersistedTrims()` reads both trims from `Preferences` during
robot construction; `nudgeTrim()` clamps, writes flash and logs on every press; `resetTrimsCommand()`
zeroes both, bound to `operator.resetShotTrims_StartSelect` (Start+Select), live in every mode.
`ShotCalculator.recordShot(poseTrusted)` writes one row per burst, called from
`SuperStructure.updateFeedGate()` on the rising edge of `feedGateOpen`. Both streams are
`Telemetry.log`, wpilog only, with one deliberate exception: `ShotCalc/Shot/Index` goes out over
NetworkTables via `logDashAlways` so the operator can watch bursts count up on the Shooting tab and
know records are being written. `ShotCalc/TurretAngleOffsetDegrees` moved from `log` to `logDash`
for the same reason: a trim that persists has to be readable before a match.

**The mark buttons were dropped, deliberately.** The three-button plan above was answered with a
better signal: the operator already says what happened by trimming, so a hood-down press is "that
one went long" and a hood-up press is "that one fell short". Every press is logged under
`ShotCalc/Trim/*` with its verdict, the index and age of the burst it is judging, and that burst's
distance denormalised onto the row so a distance-binned fit is one pass over one channel. A made
shot is the absence of a press, which means makes are counted as bursts with no trim behind them.
No new operator buttons were spent on outcomes.

**Schema, the join recipe, and the argument for all of it** are in `docs/tools/shot-log.md`, linked
from `docs/index.md` and from `logging.md`.

**What 3.2 still needs from a human.** Nothing in the code. Take a practice log, join the two
streams, and the first reading worth doing by hand is in the doc's last section: a long/short bias
constant across distances is an exit-speed error (`RPM_PER_MPS`, `MPS_FACTOR`), one that grows with
distance is an angle error.

**Not verified on hardware.** Everything below the compile boundary is untested until someone
deploys: that `Preferences` survives a power cycle on this rio, that the boot print appears, and
that the gate's rising edge fires once per burst rather than several times if `keepReady` chatters.
Watch `ShotCalc/Shot/Index` against the `Launcher/RPM` dips on the first practice log; if the index
climbs faster than the bursts, the gate needs a debounce on the closing edge, not on the opening
one.

### 3.2 A Shooting page: trims to Java, outcomes to a correction table

**Goal.** The operator's trim becomes the model's trim with one button, and the shot records become
a distance-indexed correction instead of a single constant.

**Today.** Trims are hand-copied into `hoodOffsetDeg` (1.2). `HubTargetFactory` has two empty
tables (1.1).

**Build.**

1. Server: `server/lib/shot-config.js` (or the shared rewriter of 3.6) that parses the three
   `PolyModel` calls in `ShotCalculator.java` and can rewrite `hoodOffsetDeg`, `RPM_PER_MPS`,
   `MPS_FACTOR` and which model `WANTED_HUB_MODEL` names. Routes `GET /api/shot/target` and
   `POST /api/shot/apply`. Same rules as the other writers.
2. Client: `client/pages/shooting/`. Live over NT4: the two trims, `ShotCalc/HubPolyModel`,
   distance, wanted against actual RPM and hood. A "Bake trims into model" button that adds
   `HOOD_ANGLE_OFFSET` to the live model's `hoodOffsetDeg`, writes, and tells the operator to zero
   the trim (or does it over NT if 3.3's publish path exists).
3. From logs, once 3.1 data exists: a scatter of marks against distance, per model. Fit a
   correction per distance bin and propose a table of distance to hood correction and RPM
   correction.
4. Robot: a distance-indexed correction layer on top of the polynomial, one `InterpolatingTreeMap`
   per model, applied where the hood angle and flywheel speed are assembled
   (`ShotCalculator.java` around 481-490). Written by the page as `put(d, v)` lines in a static
   block, the way `HubTargetFactory` already sketches it. Keep the polynomial; it carries the
   shoot-on-move shape that a table cannot.

**Acceptance.** Round-trip tests against the real `ShotCalculator.java`. Bake a trim, `spotlessCheck`
is green, the robot reads the new offset after deploy. The model switch works from the page. The
correction table is empty by default and the model shoots exactly as before when it is.

**Gotchas.** `hoodOffsetDeg` is the last positional argument of a record constructor with three
array literals before it; the parser must count brackets, not commas. Per-model trims exist because
each fit is wrong in its own way (comment at `ShotCalculator.java` 155-159); never write one trim
to all three.

**Size.** Two to three days.

### 3.3 A Gains page: live tunables with write-back

**Goal.** Tune a loop without redeploying; when it is right, write the gains into the config class
with one button.

**Build.**

1. Robot: for turret, hood and launcher, register each gain as
   `Telemetry.tunable(key, default, onChange)` with `onChange` calling `configPIDGains`,
   `configFeedForwardGains` or `configMotionMagic` and then `applyTalonConfig(motor)`
   (`Mechanism.java` 1784) for the leader and each follower. No FMS guard and no master enable:
   tunables are meant to be editable pre-match while attached (1.3). Keys like
   `Tuning/Turret/kP`. Do not expose current limits from here; they are safety, not tuning.
2. Client: `client/pages/gains/`. One card per mechanism: gain fields bound to the tunables over
   NT4 (the read-only `nt4.js` needs a publish path; add it deliberately, for `Tuning/*` topics
   only), a live plot of commanded against measured from the loop-rate keys in 1.4, and a step
   table: detect steps in `CommandedDegrees` or `CommandedRPM` and report rise time, overshoot,
   settle time and steady-state error per step.
3. Server: rewrite the `final` gain fields in each `*Config` class (the shared rewriter, 3.6).
   Route `POST /api/gains/apply`. Provenance comment. Read back.
4. Mirror the gains into `data/robot-profile.json` and extend `checkLimits` in
   `scripts/check-drift.mjs` (line 136) to verify them, so the Turret and Power pages can show them
   and the banner catches a hand edit.
5. Rewrite `docs/tools/pid-tuning.md`: it documents `TuneValue`, which nothing uses. Describe the
   DogLog tunables and this page instead.

**Acceptance.** Change kP on the page while enabled in the shop, watch the step table change,
write to Java, `spotlessCheck` green, redeploy, same behaviour.

**Gotchas.** `applyTalonConfig` is a blocking CAN transaction; apply on change only, never per loop.
Followers need the same slot gains. The step detector must ignore turret unwrap slews; the Turret
page already excludes them (`client/lib/analyze-turret.js`).

**Size.** Three days.

### 3.4 Characterize: fit feedforward from logs already on disk

**Goal.** kS, kV and kA for any mechanism from any practice log, with no SysId routine and no
dedicated run.

**Build.** `client/pages/characterize/`, or a tab on Gains. Pick a log and a mechanism. Over the
enabled windows, least-squares fit `u = kS * sign(v) + kV * v + kA * a`, where `u` is the applied
voltage, `v` the measured velocity and `a` its derivative. Report the three with residual RMS and
the fraction of samples above the noise floor, and flag when the mechanism never exceeded a few
percent of its range (no kV information in that log). Run it across every log in the manifest and
plot kV over time; drift means belt wear or battery.

**Gotchas.**

- Turret, launcher and tower log voltage on every loop since 2026-09-08 (`fastOutputLogging`,
  1.4), so kA is fittable for them from any log after that date. Older logs, and the hood, have
  voltage at 10 Hz: interpolate it onto the fast timestamps for kS and kV and do not report kA.
- Units must match the slot. Turret is `MotionMagicVoltage`, so volts per rot/s of mechanism
  velocity after `sensorToMechanismRatio` 39.78. Check `Mechanism.java` 1140-1400 for the request
  each setter sends before choosing volts or amps; several setters are torque-current.
- `Turret/CommandedRotPerSec` is the wanted rate (`mechOmegaRotPerSec`, `Turret.java` 411), not a
  measurement. Use `Turret/VelocityRotPerSec`, logged since 2026-09-08; on older logs,
  differentiate `Turret/PositionDegrees`.
- Turret kV is expected to come out near 5 V per rot/s (09-05 handoff, 3.9). If the fit says 10,
  the handoff was wrong, not the fit; record the result here either way.

**Acceptance.** A synthetic log with known kS and kV (extend `test/helpers/make-log.mjs`) comes
back within a few percent. Run on the 2026-09-06 and 09-07 logs and write the turret result into
section 5 of this document.

**Size.** One to two days.

### 3.5 Shot model regression in sim

**Goal.** Before a model or trim change goes to the field, a test says where the shots land.

**Today.** `RobotSim.createSimBallLaunch` (`src/main/java/frc/robot/RobotSim.java` 218-266) fires
`ShotCalculator` parameters into `FuelPhysicsSim`, and the sim scores the hub (`getTotalScored`,
`getBlueHub`, `FuelPhysicsSim.java` 2555-2611). `src/test/java/frc/rebuilt/ShotCalculatorTest.java`
and `FuelPhysicsSimTest.java` exist.

**Build.** A JUnit test, or a Gradle task if it is slow, that for each model and ten distances
across `distMin..distMax`, stationary and at plus and minus 2 m/s radial, computes parameters,
launches a ball in `FuelPhysicsSim`, steps physics until the ball sleeps, and reports the landing
offset from the hub centre. Assert a tolerance for the ceiling model, which is known to score;
report only for the others until 3.1 data says what the tolerance should be.

**Gotchas.** `getParameters()` reads `Robot.getSwerve()` and `Robot.getSuperStructure()`; the test
needs those stubbed, or a variant that takes a pose and speeds. The sim's drag and Magnus constants
are team 5962's, not measured on our fuel; a systematic sim-against-field offset is expected and is
itself worth recording once 3.1 gives field data.

**Size.** One to two days.

### 3.6 Shared Java literal rewriter

The app has the same pipeline three times over (swerve, vision, and soon shot and gains): parse a
Java file, find a call or a field, replace a numeric literal in place, wrap the line the way the
formatter would, add provenance, read back. Factor it into `server/lib/java-rewrite.js`:
`findClassBody(source, name)`, `findField(body, name)`, `findCall(source, name)`,
`splitArguments(text)`, `replaceLiteral(source, span, text)`, `wrapForFormatter(line, indent)`,
`provenance(prefix, date)`. `check-drift.mjs` already has `classBody` and `fieldValue` (lines 117
and 131) that should share it. Port `vision-config.js` onto it under its existing tests before
writing anything new on top.

**Size.** One day.

### 3.7 Smaller items

- **Hood zero.** The encoder zero sits a fraction of a degree below the hard stop, which is why
  `homeRestToleranceDegrees` exists (`Hood.java` 34). A homing routine (drive to the stop at low
  current, zero, back off) on a disabled-mode button would remove that workaround and the intake
  extension's `zeroAtMax` chord. Model it on `Turret.zeroTurretCommand()`.
- **`RPM_PER_MPS` and `MPS_FACTOR`.** The exit-speed calibration has never been measured on this
  robot. Once 3.1 gives outcomes: a long/short bias that is constant across distances is a speed
  error; one that grows with distance is an angle error. The Shooting page can say which.
- **Find the fit tool.** Ask whoever built the polynomial models where the fitting notebook and
  the input data are. If they exist, they belong in `tools/shot-fit/` with the data. If they do
  not, the correction table in 3.2 is the only way to move the model, and this document should say
  so.

## 4. Open questions for the humans

1. Where are the polynomial fit script and its data? (3.7)
2. ~~Which operator buttons may the shot marks take?~~ **Answered 2026-09-08: none.** No marks.
   The D-pad trim presses are the outcome signal, since a shot going long already gets the hood
   trimmed down and one falling short gets it trimmed up. A made shot is the absence of a press.
   Start+Select, two buttons nothing else used, became the trim reset.
3. ~~Should trims persist across power cycles or only across deploys?~~ **Answered 2026-09-08:
   across power cycles, and then the settled value gets folded into the code (3.2).** Plain
   `Preferences`, with a boot print and the Start+Select reset as the mitigation for a stale trim.
   No expiry: zeroing a trim after some minutes disabled would surprise the operator in the middle
   of a session they thought was still calibrated.
4. Is a publish path from the app to the robot over NT4 acceptable? `nt4.js` is deliberately
   read-only today. 3.3 needs to write `Tuning/*`. The alternative is typing values into Elastic,
   which works but loses the step table's context.

## 5. Numbers worth remembering

| Item | Value | Source |
| --- | --- | --- |
| Hub model hood trim | -5.0 deg | `ShotCalculator.java` near 225; commit `abdc95c` |
| Hood trim per D-pad press | 0.25 deg, about a quarter foot of range at 2.5 m | `HOOD_OFFSET_STEP_DEG`, line 86 |
| Model hood sensitivity | about 1 deg per foot of range near 2.5 m | comment near `ShotCalculator.java` 225 |
| Exit speed to flywheel | 365 RPM per m/s | `RPM_PER_MPS`, line 124 |
| Shots seen so far | 2.2 to 2.6 m, hood 18 to 20 deg | 09-05 handoff, 3.1b |
| Turret kV | 10 V per rot/s in code; about 5 expected | `Turret.java` 73; 09-05 handoff, 3.9 |
| Launcher on-target window | 200 RPM | `Launcher.java` 51 |
| Log rates | diagnostics 10 Hz; aimed-mechanism position and command 50 Hz; turret, launcher and tower voltage 50 Hz | `Mechanism.logDiagnostics`; section 1.4 |
| DogLog | 2026.5.0, has `tunable(key, default, onChange)` | Gradle cache |
| `tunableOnFMS` | true, deliberately; no guard wanted | `Robot.java` 145 |
| Trim cap | 10 deg either axis, on load and on every press | `MAX_TRIM_DEG`, `ShotCalculator.java` |
| Trim storage | `Preferences`, keys `ShotHoodTrimDeg` and `ShotTurretTrimDeg`; survives a power cycle | `ShotCalculator.loadPersistedTrims()` |
| Trim reset | operator Start+Select, live in every mode | `Robot.java` 311 |
| Shot record | one row per burst on the feed gate's rising edge; `Index` is the only key on NT | `ShotCalc/Shot/*`, `docs/tools/shot-log.md` |
