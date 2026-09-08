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

### 1.2 The operator's trims evaporate

`HOOD_ANGLE_OFFSET` and `TURRET_ANGLE_OFFSET` (`ShotCalculator.java` 76 and 89) are
`public static double` fields, bound to the operator D-pad in `src/main/java/frc/robot/Robot.java`
lines 300-303. They live in RAM. A code restart puts them back to zero, so whatever the operator
dialled in during a session is gone at the next deploy unless someone remembers the number and
folds it into `hoodOffsetDeg` by hand. That is the -4, then -5, in the table above. Both trims are
logged (`ShotCalc/HoodAngleOffsetDegrees`, `ShotCalc/TurretAngleOffsetDegrees`), so the wpilogs
do hold them.

`tools/robot-app/data/controls.json` (line 288) still says the hood step is 0.1 deg; the code moved
to 0.25 (`HOOD_OFFSET_STEP_DEG`, line 86). The drift check verifies that bindings exist, not what
their descriptions say.

### 1.3 Motor gains mean a redeploy per iteration

Every gain is a `final` in a mechanism's inner `*Config` class and is applied once in the
constructor:

| Mechanism | Gains | Control request |
| --- | --- | --- |
| Turret | `positionKp` 800, `positionKi` 100, `positionKv` 10, `positionKs` 0.6; MotionMagic cruise 0.25, accel 0.5 (`Turret.java` 67-79, applied 98-100) | `setMMPosition` sends `MotionMagicVoltage` (`Mechanism.java` 1336-1341), so kV is volts per rot/s despite the comment above it saying current |
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
passes `tunableOnFMS = true`, so tunables are read from NT even with the FMS attached. Anything
that makes gains tunable has to add its own guard.

The 2026-09-05 handoff (section 3.9) says turret `positionKv` is about double what a Kraken through
39.78:1 needs and to "tune from a log of commanded vs measured turret velocity". Nobody has, because
that is a by-hand log analysis.

### 1.4 What the logs already carry

Every `Mechanism` logs voltage, stator current, supply current, temperature and connectivity under
`<Name>/*` at the 10 Hz slow tier (`Mechanism.logDiagnostics`, lines 807-831). The aimed
mechanisms log at loop rate (50 Hz): `Turret/CommandedDegrees`, `Turret/PositionDegrees`,
`Turret/PositionError`, `Turret/CommandedRotPerSec`; `Hood/CommandedDegrees`,
`Hood/PositionDegrees`, `Hood/RPM`; `Launcher/CommandedRPM`, `Launcher/RPM`. `ShotCalc/*` logs
fifteen keys at loop rate while launching and 10 Hz otherwise (`ShotCalculator.java` around
505-531), including distance, wanted RPM, wanted hood, exit speed, time of flight, model name and
both trims. `SuperStructure/ShotReady/*` logs the feed gate and every input to it. What is missing
is any record of the outcome.

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

### 3.1 Persist the trims and record every shot

**Goal.** A deploy no longer zeroes the operator's trims, and every burst leaves a record in the log
that says what was aimed and, when the operator says so, where it went.

**Build.**

1. Persist `HOOD_ANGLE_OFFSET` and `TURRET_ANGLE_OFFSET` with WPILib `Preferences`
   (`edu.wpi.first.wpilibj.Preferences`): read at class init, write inside the four
   `increase*`/`decrease*` commands (`ShotCalculator.java` 91-107). Add a way to reset both to zero
   (a chord, or a disabled-only button), and `Telemetry.print` at boot whenever a persisted trim is
   non-zero so nobody is surprised by it.
2. On the rising edge of `feedGateOpen` (`SuperStructure.java` 337), log one shot record: FPGA
   time, `ShotCalc/DistanceNoLookahead`, `ShotCalc/FlywheelSpeedRPM`, `ShotCalc/HoodAngleDeg`,
   actual `Launcher/RPM`, actual `Hood/PositionDegrees`, `Turret/PositionError`,
   `ShotCalc/HubPolyModel`, both trims, `ShotCalc/FeedShot`, `ShotReady/PoseTrusted` and the pose.
   A struct under `ShotCalc/Shot/*`, or a set of keys written once per burst; the point is one row
   per burst, not another loop-rate stream. Balls per burst can be counted afterwards from the dips
   in `Launcher/RPM`, which is kept at loop rate for exactly this reason (`Launcher.java` 203).
3. Three operator marks: short, made, long. Log each as `ShotCalc/Shot/Mark` with the time; the
   app pairs a mark with the most recent burst. Pick buttons that are free in `data/controls.json`
   and ask the operator before deciding. Update `controls.json`, and fix the 0.1 to 0.25 step text
   while there.

**Acceptance.** Deploy twice with a non-zero trim set in between; the trim survives. A practice log
contains one record per burst plus the marks. `npm run check` is clean. `docs/tools/elastic.md`
and the Shooting tab of `src/main/deploy/elastic-layout.json` mention the marks.

**Gotchas.** `Preferences` writes go through NT to the RIO's flash; write in the command, never in
periodic. Keep the shot record off NT (plain `Telemetry.log`).

**Size.** Half a day.

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
   (`Mechanism.java` 1784) for the leader and each follower. Guard with a `Tuning/Enabled`
   boolean tunable **and** `!DriverStation.isFMSAttached()`, because `tunableOnFMS` is `true`
   today (1.3). Keys like `Tuning/Turret/kP`. Do not expose current limits from here; they are
   safety, not tuning.
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
write to Java, `spotlessCheck` green, redeploy, same behaviour. With the FMS attached, or
`Tuning/Enabled` false, NT edits do nothing.

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

- Voltage is at 10 Hz, position and velocity at 50 Hz (1.4). Interpolate voltage onto the fast
  timestamps for kS and kV. kA from a 10 Hz voltage is not credible: either report only kS and kV,
  or add a loop-rate `<Name>/Voltage` log while the mechanism is tracking or launching (log only,
  not NT).
- Units must match the slot. Turret is `MotionMagicVoltage`, so volts per rot/s of mechanism
  velocity after `sensorToMechanismRatio` 39.78. Check `Mechanism.java` 1140-1400 for the request
  each setter sends before choosing volts or amps; several setters are torque-current.
- The turret's logged rate is the commanded feedforward (`Turret/CommandedRotPerSec` is
  `mechOmegaRotPerSec`, the wanted rate, `Turret.java` 411). Differentiate `Turret/PositionDegrees`
  for the measurement.
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
- **`controls.json` step text** (1.2). Fix with 3.1.

## 4. Open questions for the humans

1. Where are the polynomial fit script and its data? (3.7)
2. Which operator buttons may the shot marks take? (3.1)
3. Should trims persist across power cycles (which `Preferences` does) or only across deploys?
   Across power cycles means a stale trim from last week silently applies at the next event; the
   boot-time print in 3.1 is the mitigation. The alternative is a reset in `disabledInit` after
   some minutes disabled.
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
| Log rates | diagnostics 10 Hz; aimed-mechanism position and command 50 Hz | `Mechanism.logDiagnostics`; section 1.4 |
| DogLog | 2026.5.0, has `tunable(key, default, onChange)` | Gradle cache |
| `tunableOnFMS` | true | `Robot.java` 145 |
