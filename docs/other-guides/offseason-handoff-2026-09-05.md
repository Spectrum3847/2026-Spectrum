# Offseason handoff: 2026-09-04 bench session and open items

*Audience: Whoever picks up the robot code next, a team student or a mentor. Written the night
before the first full-field practice day. This is a snapshot of one session; several items below
were closed later in the season, and each says so. Everything is on branch `2026-offseason-bot`.*

## 1. What changed on 2026-09-04

Each of these was a decision, and the reason is the part worth keeping.

* **Flipped both swerve drive-side inversions.** Robot moved opposite to every command since the
  Aug 20 swerve re-alignment, which shifted all four CANcoder zeros by 180 degrees. Wheel omega
  disagreed in sign with the gyro, and wheel translation disagreed with camera motion.
* **Disabled seeding now uses MegaTag1 only, the turret camera is rejected when its heading
  disagrees with the gyro by more than 5 degrees, stale estimates are rejected, and each camera logs
  its age and whether it integrated.** Bench logs showed the turret camera flipping the pose a metre
  every loop while disabled, one camera accepting estimates that never moved the pose, and a first
  enable with a 90 degree heading error that nothing could fix.
* **The turret stopped zeroing its encoder on code start, and operator B while disabled does it at
  the current position instead.** Every deploy had been silently re-zeroing the turret wherever it
  pointed.
* **Rear camera hostnames corrected.** The code addressed `limelight-back-*`, so the robot never saw
  the rear cameras and they never got a heading.
* **IMU mode is re-sent to every camera periodically.** A camera booting after code start used to
  stay in mode 0.
* **Camera mount values taken from CAD, including the rear yaws.** The old values had axes swapped
  and one camera placed at the front.
* **Shot-readiness predicates are logged, not yet gating.** Stage one of feeder gating.
* **Vision heading is no longer fused while enabled.** A 2025 carry-over fused MegaTag1 heading at
  0.1 degrees standard deviation, injecting camera yaw jitter straight into the turret.
* **Vision and the orchestrator now run from the robot loop, before the scheduler.** This removes
  latency in the aim chain, so a state change reaches the mechanisms in the same loop.
* **Limelight reads cached per loop, one NetworkTables flush, key caching in the battery logger,
  small per-loop trims.** Loop-overrun reduction.
* **The hood stopped pushing into its hard stop at home.** It stalled at 75 A stator whenever
  enabled and idle, and gained 19 C in 30 s.

## 2. Operating procedure that the code now assumes

* **The Driver Station alliance must match where the driver stands.** Blue for the blue-side shop
  mock. Red negates the sticks, and also makes the turret aim at a *feed* point from the shop
  position.
* **Turret zero.** Power the robot on with the turret facing away from the intake, or point it there
  by hand and press **operator B while disabled**. Check `Turret/PositionDegrees` reads near 0, and
  check `Vision/TurretLL/HeadingErrorDeg` reads near 0 once the turret camera sees tags. That key
  *is* the turret zero error, so a steady offset means the zero is off by that many degrees.

  *Update, later in the season.* The boot zero now assumes the turret was **pinned** at its zero
  before the power cycle, and it only re-seeds after a real power cycle rather than after every
  code start. Operator B while disabled is still the manual override. See
  [2026 Season Specific](2026-season-specific.md#operating-rules-you-cannot-read-off-the-code).

* **Before enabling, look at Field2d.** With tags in view the heading is seeded while disabled. If
  the robot is drawn pointing the wrong way, the reset chord recovers from the turret camera. Avoid
  the reorient binding unless there are no tags, because the direction it points is where the
  *intake* ends up from the driver's view. A gross heading error also self-corrects about a second
  after stopping in front of two tags.

* **Do not reorient while driving fine.** The Aug drive complaints were the inverted drivetrain, not
  the heading.

## 3. Open items, in recommended order

### 3.1 Hood does not always move when close and shooting (reported 09-04)

Two separate things were found in the logs.

* **Stall at home (fixed in code).** In every enabled window the hood sat in home at 0.3 degrees and
  -2.3 V with about 75 A stator continuously. The encoder zero sits a fraction of a degree below the
  hard stop, so position 0 can never be reached. Motor temperature rose 27 to 46 C in 32 s idle, and
  reached 58 C in the shooting session. Home now cuts output within a tolerance of zero, set by
  `Hood.homeRestToleranceDegrees`.
* **Dead motor during shots (hardware, watch it).** In the 04:25 shooting log (`FRC_20260905_042536`)
  the hood was commanded to 18 to 20 degrees in 11 launch windows. In 9 of them it reported
  **exactly 0 V and 0 A** and did not move, while turret and flywheel ran normally. In the other 2 it
  hit 3.07 V and swung 19 degrees in half a second, so the motor, the gains, and the voltage ceiling
  are all adequate when the command reaches it. Zero output with a position request pending, on one
  device only, is a device that is not receiving or executing control frames. The same night the DS
  console showed "CAN message is stale" and "CAN frame not received" errors, and an earlier log
  (`_041205`) shows the CANivore transmit error counter climbing to 248 of the 255 that causes
  bus-off. The team found and fixed a CAN hardware fault. Every mechanism's periodic now logs a
  `<Name>/MotorConnected` key. If the hood ever reads 0 V while commanded again, check that key
  first. If it reads true and the hood still does not move, the next suspects are the sticky
  `BootDuringEnable` fault and the CANivore utilization below.
* **CANivore bus utilization is 66 to 81 percent** in every log. CTRE recommends staying well under
  that. High utilization delays frames and produces exactly the stale-frame warnings seen. Every
  mechanism was setting eight status signals to 250 Hz on the leader and each follower, which with
  ten mechanisms plus the swerve modules is most of that load. Dropping voltage, currents, duty
  cycle, and temperature to a low rate and keeping only position and velocity fast would roughly
  halve it. Do this early, it may be the whole story behind the intermittent hood.
* **The hood's voltage ceiling** is low. It moved the hood fine in the two live windows, so leave it
  unless a log shows saturation at the ceiling with the hood lagging its command.

### 3.1b Other things the shooting log showed

* **Loop overruns are real.** The robot-periodic timer ran 15 to 19 ms median, 32 to 44 ms at the
  95th percentile, with a worst case of 0.55 and 0.92 s. Over 5 percent of loops overrun. See 3.8.
* **The turret unwrapped mid-shot.** At 101 s the turret went from -206 to +153 degrees, a 360 degree
  slew, while launching without squeeze, because the target crossed the travel seam at robot-front.
  Anything fed during that slew goes anywhere. Feeder gating (3.3) would have held it. The turret's
  readiness check is already false while unwrapping.
* **The readiness gate was previewed.** Over 366 launching loops the gate would have held the feed
  for 321, almost entirely because the hood was not at angle (the dead hood above) and the flywheel
  had not reached speed in the first 0.3 s. Flywheel spin-up from 650 to 2600 RPM took about 0.25 s
  and held within the 200 RPM window during bursts, so the planned start tolerance is fine.
* **Shots were all at 2.2 to 2.6 m** with a commanded hood of 18 to 20 degrees from the 3 m ceiling
  model.

### 3.2 Switch the hub shot model to the full-field fit

There are two hub fits in `ShotCalculator`: the low-ceiling shop fit and the full-field fit. On a
real field the full-field fit is the one to use, and it is a one-line change to
`ShotCalculator.WANTED_HUB_MODEL`. Do it before shooting on the field.

*Closed after this snapshot. `WANTED_HUB_MODEL` named the low-ceiling shop fit when this was
written.*

### 3.3 Feeder gating, stage two

Stage one logs everything under the shot-readiness keys. The plan is to hold the dye rotor and the
launcher tower in their staging states until the debounced composite is true, with a looser "keep
feeding" window (turret within about 6 degrees, flywheel not more than about 25 percent below
target) so flywheel droop during a burst does not starve the feed. Add an operator override to
ignore the gates. Confirm first that the slow-index state does not push balls into the flywheel. If
it does, the hold state has to be off instead. Set the hold tolerances from a log of launcher RPM
during a burst.

### 3.4 Turret camera latency compensation

As of that date the turret camera was told its live mount transform every loop, but applies it to an
image taken 20 to 40 ms earlier. At 90 degrees per second of slew that is 2 to 4 degrees of yaw and
15 to 25 cm of pose error. The plan was to give the camera a fixed turret-frame transform once,
switch it to MegaTag1, keep a `TimeInterpolatableBuffer` of turret angle on the Rio fed from the
turret's position signal with its own timestamp, and de-rotate each estimate with the angle at its
capture time. Reject frames when the measured turret rate at capture exceeds about 60 degrees per
second. This also makes the Limelight yaw-sign question moot and gives a clean vision-based turret
zero check.

### 3.5 Swerve module alignment

The Aug 20 offsets differ from the Aug 2 offsets by 175 to 186 degrees per module, so one of the two
alignments has modules up to about 6 degrees off, which scrubs and drifts odometry. Re-align
carefully with all wheels straight and bevels on one side. If the team prefers CTRE's template
convention, revert the two inversion flags **and** re-align with bevels on the template's side. Do
not do only one of the two.

### 3.6 Camera calibration on a real field

* The left camera's MegaTag1 heading read a consistent 9 to 12 degrees higher than the right camera
  in every shop session. It could be its mount yaw, or the shop tower tag placement. On the field,
  park where the left camera sees two hub tags and compare its reported pose heading to the gyro.
  Adjust its yaw in the web UI by the difference, raising it if the camera reads high.
* Rear camera values entered on 09-04: left camera forward -0.282 m, right -0.317 m, up 0.433 m; right
  camera forward -0.256 m, right +0.338 m, up 0.443 m. Both roll 180, pitch 60, yaw +135 and -135,
  with the image orientation set to Upside-Down. Cross-camera heading agreement to 0.02 degrees
  confirmed that combination is correct. Do not change the roll or the orientation flag
  independently.
* Update the two rear cameras to Limelight OS 2026.1. At that date the turret camera and one rear
  camera were on 2026.0. 2026.1 fixes MegaTag2 emitting field-centre poses when it has no solution.
  Back up the settings first, because flashing wipes the camera.

### 3.7 Left camera estimates that never moved the pose

In the first 09-04 session the left camera reported "Stable integration" for 12 s while disabled and
the pose never moved. It became diagnosable through the per-camera estimate age and integration keys.
In the last session its age was 0.03 to 0.07 s, so it may have been a one-off. Watch it.

### 3.8 Loop time

Measured across all eight 2026-09-05 robot logs and the Driver Station logs:

* The roboRIO CPU sat at 92 to 95 percent in every DS log, enabled or disabled. The enabled loop
  period median was 26 to 39 ms and 57 to 93 percent of enabled loops missed 25 ms. That is
  starvation, not one slow call: every section of the loop stretched together.
* Of an enabled loop (18:38 log, medians): the scheduler 15 ms, vision 4.5 ms, the rest of the robot
  loop 2.8 ms, the orchestrator 0.6 ms, and work outside the robot loop 3 ms.
* Every loop over 300 ms was while disabled, inside the scheduler, 25 to 33 s after boot or on an
  auto-chooser change: PathPlanner warmup and trajectory generation, harmless. The worst enabled loop
  was 232 ms. The GC theory did not hold for the big stalls, and `-Xlog:gc*` was left on to settle
  the 130 to 230 ms enabled episodes.
* The scheduler's own timers are logged in seconds, not milliseconds.

This was worked in full that same evening, and the results are in
[Loop time and CPU handoff 2026-09-05](loop-time-handoff-2026-09-05.md). Read that file rather than
this paragraph. In particular, the real-time main thread priority that was in flight at the time of
writing was reverted before the end of that evening, so do not reintroduce it.

### 3.9 Smaller items

* The turret's `positionKv` is roughly double what a Kraken through 39.78:1 needs. Feedforward
  overdrives while tracking a moving target. Tune it from a log of commanded against measured
  turret velocity.
* The turret's `positionKi` with a voltage cap can wind up when the target crosses the seam at
  robot-front.
* Rear cameras at 60 degrees pitch only see hub tags within about 2 m. 2910 runs one camera at 15
  degrees and sees tags out to 6 m. Worth a mount discussion.
* Limelight exposure of 4.7 ms is long for a moving robot. Shorten it and raise gain if far tags
  drop out.

## 4. Reading logs without AdvantageScope

`scripts/wpilog.js` is a dependency-free Node reader for `.wpilog` files (Node 18+).

```bash
node scripts/wpilog.js path/to/FRC_x.wpilog list
node scripts/wpilog.js path/to/FRC_x.wpilog dump DS:enabled /Robot/Swerve/State/Pose
```

DogLog keys are prefixed `/Robot/`. In Git Bash set `MSYS_NO_PATHCONV=1`, or the `/Robot/...`
names get rewritten as Windows paths. Logs live on the roboRIO under `/home/lvuser/logs` (or
`/U/logs` with a USB stick):

```bash
scp lvuser@10.85.15.2:/home/lvuser/logs/*.wpilog .
```

The cross-checks that found the bugs that session, and that are still worth running:

* Wheel-derived swerve omega against the rate of change of the pose heading. They must agree in
  sign.
* Wheel field velocity against the change in any camera's reported pose over the same second. They
  must agree in direction.
* Each camera's reported heading against the pose heading. The turret camera's difference is the
  turret zero error.

## 5. Numbers worth remembering

|                   Item                    |                                Value                                 |       Source        |
|-------------------------------------------|----------------------------------------------------------------------|---------------------|
| Robot network team number                 | 8515, with the Limelights at 10.85.15.x                              | DS and camera IPs   |
| Turret zero                               | facing away from the intake; the code offset is 180 from robot front | CAD                 |
| Turret pivot to camera                    | 0.138 m along the look direction, camera 18.632 in up                | CAD and measured    |
| Hub target                                | blue hub centre (4.63, 4.03) m                                       | shot-calculator log |
| Turret camera heading error seen on bench | -4.8 to -6 degrees, steady                                           | turret camera log   |
