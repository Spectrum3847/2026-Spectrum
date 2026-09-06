# Offseason handoff: 2026-09-04 bench session and open items

*Audience: whoever picks up the robot code next, human or Claude session. Written the night before the
first full-field practice day. Everything below is on branch `2026-offseason-bot`.*

## 1. What changed on 2026-09-04 (newest first)

| Commit | Change | Why |
| --- | --- | --- |
| `f7a774e` | Flipped both swerve drive-side inversions in `SwerveConfig` | Robot moved opposite to every command since the Aug 20 swerve re-alignment shifted all four CANcoder zeros by 180 deg. Wheel omega disagreed in sign with the gyro; wheel translation disagreed with camera motion. |
| `340ab90` | Disabled seeding uses MegaTag1 only; turret camera rejected when its heading disagrees with the gyro by more than 5 deg; stale-estimate rejection; per-camera age / integrated / MT2 telemetry; gross-heading safety net while enabled | Bench logs showed the turret camera (turret zero 69 deg off) flipping the pose a metre every loop while disabled, one camera accepting estimates that never moved the pose, and a first enable with a 90 deg heading error nothing could fix. |
| `5013586` | Turret no longer zeros its encoder on code start; operator **B while disabled** zeros it at the current position | Every deploy silently re-zeroed the turret wherever it pointed. |
| `e02a1e0` | Rear camera hostnames are `limelight-left` and `limelight-right` | Code addressed `limelight-back-*`, so the robot never saw the rear cameras and they never got a heading. |
| `71dc7a7` | IMU mode re-sent to every camera every 2 s | A camera booting after code start stayed in mode 0. |
| `e216a64`, `2d9b10c` | Camera mount values from CAD; rear yaws +135 / -135 | Old values had axes swapped and one camera placed at the front. |
| `dcb88dc` | Shot-readiness predicates logged under `SuperStructure/ShotReady/*`, no gating yet | Stage one of feeder gating. |
| `616e663` | Vision heading no longer fused while enabled | 2025 carry-over fused MT1 heading at 0.1 deg std, injecting camera yaw jitter straight into the turret. |
| `78ac4c8` | `Vision.periodic()` and `SuperStructure.periodic()` run from `Robot.robotPeriodic()` before the scheduler | Removes 20 ms of latency in the aim chain; state changes reach mechanisms the same loop. |
| `a3645d4`, `51216e8`, `70a48c3` | Limelight reads cached once per loop, single NT flush, BatteryLogger key caching, small per-loop trims | Loop-overrun reduction. |
| (this doc) | Hood stops pushing into its hard stop at home | Stalled at 75 A stator whenever enabled and idle; +19 C in 30 s. |

## 2. Operating procedure that the code now assumes

- **Driver Station alliance must match where the driver stands.** Blue for the blue-side shop mock. Red
  negates the sticks and also makes the turret aim at a *feed* point from the shop position.
- **Turret zero.** Power the robot on with the turret facing away from the intake, or point it there by
  hand and press **operator B while disabled**. Check `Turret/PositionDegrees` reads near 0, and check
  `Vision/TurretLL/HeadingErrorDeg` reads near 0 once the turret camera sees tags. That key *is* the
  turret zero error; a steady offset means the zero is off by that many degrees.
- **Before enabling, look at Field2d.** With tags in view the heading is seeded while disabled. If the
  robot is drawn pointing the wrong way, LB+Select resets from the turret camera. Avoid the LB+D-pad
  reorient unless there are no tags; the D-pad direction is where the *intake* points from the driver's
  view. A gross heading error also self-corrects about a second after stopping in front of two tags.
- **Do not reorient while driving fine.** The Aug drive complaints were the inverted drivetrain, not the
  heading.

## 3. Open items, in recommended order

### 3.1 Hood does not always move when close and shooting (reported 09-04)
Two separate things were found in the logs.

- **Stall at home (fixed in code).** In every enabled window the hood sat in HOME at 0.3 deg with
  -2.3 V and ~75 A stator continuously: the encoder zero sits a fraction of a degree below the hard
  stop, so position 0 can never be reached. Motor temp rose 27 to 46 C in 32 s idle, and reached 58 C in
  the shooting session. HOME now cuts output within 1 deg of zero (`Hood.homeRestToleranceDegrees`).
- **Dead motor during shots (hardware, watch it).** In the 04:25 shooting log (`FRC_20260905_042536`)
  the hood was commanded to 18 to 20 deg in 11 launch windows. In 9 of them it reported **exactly 0 V
  and 0 A** and did not move, while turret and flywheel ran normally. In the other 2 it hit 3.07 V and
  swung 19 deg in half a second, so the motor, gains and 3 V peak are adequate when the command reaches
  it. Zero output with a position request pending, on one device only, is a device that is not receiving
  or executing control frames. The same night the DS console showed "CAN message is stale" and "CAN
  frame not received" errors and an earlier log (`_041205`) shows the CANivore transmit error counter
  climbing to 248 of the 255 that causes bus-off. The team found and fixed a CAN hardware fault. Every
  mechanism periodic now logs `<Name>/MotorConnected`; if the hood ever reads 0 V while commanded again,
  check that key first. If it is true and the hood still does not move, next suspects are the sticky
  `BootDuringEnable` fault and the CANivore utilization below.
- **CANivore bus utilization is 66 to 81 percent** in every log. CTRE recommends staying well under
  that; high utilization delays frames and produces exactly the stale-frame warnings seen. Every
  `Mechanism` sets eight status signals to 250 Hz on the leader and each follower (`Mechanism`
  constructor), which with ten mechanisms plus the swerve modules is most of that load. Dropping
  voltage, currents, duty cycle and temperature to 20 to 50 Hz and keeping only position and velocity
  fast would roughly halve it. Do this early; it may be the whole story behind the intermittent hood.
- **Peak voltage** is 3 V. It moved the hood fine in the two live windows, so leave it unless a log
  shows saturation at 3 V with the hood lagging its command.

### 3.1b Other things the shooting log showed
- **Loop overruns are real.** `Scheduler/robotPeriodic` median 15 to 19 ms, 95th percentile 32 to
  44 ms, worst 0.55 and 0.92 s. Over 5 percent of loops overrun. See 3.8.
- **Turret unwrapped mid-shot.** At 101 s the turret went from -206 deg to +153 deg, a 360 deg slew,
  during `LAUNCH_WITHOUT_SQUEEZE`, because the target crossed the travel seam at robot-front. Balls fed
  during that slew go anywhere. Feeder gating (3.3) would have held them; the turret's `isReadyToShoot`
  is already false while unwrapping.
- **Readiness gate preview.** Over 366 launching loops the gate would have held the feed for 321, almost
  entirely because `HoodAtAngle` was false (the dead hood above) and the flywheel had not reached speed
  in the first 0.3 s. Flywheel spin-up from 650 to 2600 RPM took about 0.25 s and held within the 200 RPM
  window during bursts, so the planned start tolerance is fine.
- **Shots were all at 2.2 to 2.6 m** with a commanded hood of 18 to 20 deg from the 3 m ceiling model.

### 3.2 Switch the hub shot model to the full-field fit
`ShotCalculator.WANTED_HUB_MODEL` is `CEILING_3M_HUB_MODEL`, the low-ceiling shop fit. On a real field
use `HUB_MODEL`. One line, do it before shooting on the field.

### 3.3 Feeder gating, stage two
Stage one logs everything under `SuperStructure/ShotReady/*`. Plan: hold the dye rotor and launcher
tower in their staging states (`IDLE_SLOW_INDEX`, `SLOW_INDEX`) until the debounced composite is true,
with a looser "keep feeding" window (turret within ~6 deg, flywheel not more than ~25% below target) so
flywheel droop during a burst does not starve the feed. Add an operator override to ignore the gates.
Confirm first that `SLOW_INDEX` does not push balls into the flywheel; if it does, the hold state must
be `OFF`. Set the hold tolerances from a log of `Launcher/RPM` during a burst.

### 3.4 Turret camera latency compensation
Today the turret camera is told its live mount transform every loop, but applies it to an image taken
20 to 40 ms earlier; at 90 deg/s slew that is 2 to 4 deg of yaw and 15 to 25 cm of pose error. Plan:
give the camera a fixed turret-frame transform once (0.138 m forward along look direction, 18.632 in up,
pitch 60, yaw 0), switch it to MegaTag1, keep a `TimeInterpolatableBuffer` of turret angle on the Rio
fed from the turret's position signal with its own timestamp, and de-rotate each estimate with the angle
at its capture time. Reject frames when the measured turret rate at capture exceeds ~60 deg/s. This also
makes the Limelight yaw-sign question moot and gives a clean vision-based turret zero check.

### 3.5 Swerve module alignment
The Aug 20 offsets differ from the Aug 2 offsets by 175 to 186 deg per module, so one of the two
alignments has modules up to ~6 deg off, which scrubs and drifts odometry. Re-align carefully with all
wheels straight and bevels on one side. If the team prefers CTRE's template convention, revert the two
inversion flags **and** re-align with bevels on the template's side; do not do only one of the two.

### 3.6 Camera calibration on a real field
- `limelight-left` MegaTag1 heading read a consistent 9 to 12 deg higher than the right camera in every
  shop session. Could be its mount yaw or the shop tower tag placement. On the field, park where the left
  camera sees two hub tags and compare `Vision/BackLeftLL/MT1Pose` heading to the gyro; adjust its LL Yaw
  in the web UI by the difference (raise it if the camera reads high).
- Rear camera UI values entered 09-04: left forward -0.282 m, right -0.317 m, up 0.433 m; right camera
  forward -0.256 m, right +0.338 m, up 0.443 m; both roll 180, pitch 60, yaw +135 / -135, with the
  image orientation set to Upside-Down. Cross-camera heading agreement to 0.02 deg confirmed this
  combination is correct; do not change roll or the orientation flag independently.
- Update the two rear cameras to Limelight OS 2026.1 (the turret and one rear camera are on 2026.0).
  2026.1 fixes MegaTag2 emitting field-centre poses when it has no solution. Back up settings first;
  flashing wipes the camera.

### 3.7 Left camera estimates that never moved the pose
In the first 09-04 session the left camera reported "Stable integration" for 12 s while disabled and the
pose never moved. Now diagnosable: `Vision/BackLeftLL/EstimateAgeSeconds` and `IntegratedThisLoop`. In
the last session its age was 0.03 to 0.07 s, so it may have been a one-off; watch it.

### 3.8 Loop time
Measured across all eight 2026-09-05 robot logs and the Driver Station logs (evening of 2026-09-05):

- The roboRIO CPU sat at 92 to 95 percent in every DS log, enabled or disabled. Enabled loop period
  median was 26 to 39 ms and 57 to 93 percent of enabled loops missed 25 ms. That is starvation, not
  one slow call: every section of the loop stretched together.
- Of an enabled loop (18:38 log, medians): `CommandScheduler.run` 15 ms, `Vision.periodic` 4.5 ms,
  the rest of `robotPeriodic` 2.8 ms, `SuperStructure` 0.6 ms, outside `robotPeriodic` 3 ms.
- Every loop over 300 ms was while disabled, inside the scheduler, 25 to 33 s after boot or on an
  auto-chooser change: PathPlanner warmup and trajectory generation, harmless. Worst enabled loop was
  232 ms. The GC theory did not hold for the big stalls; `-Xlog:gc*` is still on to settle the
  130 to 230 ms enabled episodes.
- The `Scheduler/*` timers are logged in seconds.

What changed in response (same evening): the log-to-NetworkTables mirror is off (dashboard keys go
through `Telemetry.logDash`), Phoenix hoot auto-logging is off, mechanism status frames dropped from
250 Hz to 100 Hz for leaders and 20 Hz for followers and diagnostics (odometry stays at 250 Hz), every
mechanism refreshes its signals with one `refreshAll` per loop, `CANBus.getStatus()` runs at 1 Hz,
swerve state is logged from the main loop instead of the odometry callback, the drivetrain state is
read once per loop, vision telemetry runs at 10 Hz with MegaTag2 parsed only for the turret camera,
the main thread runs at real-time priority for the loop body, and a full GC runs on every disable.
Compare the DS CPU trace and the `Scheduler/robotPeriodic` distribution against the 2026-09-05 logs.

### 3.9 Smaller items
- Turret `positionKv` is 10 V per rot/s; a Kraken through 39.78:1 needs about 5. Feedforward overdrives
  while tracking a moving target. Tune from a log of commanded vs measured turret velocity.
- Turret `positionKi` 100 with a 6 V cap can wind up when the target crosses the seam at robot-front.
- Rear cameras at 60 deg pitch only see hub tags within ~2 m. 2910 runs one camera at 15 deg and sees
  tags out to 6 m. Worth a mount discussion.
- Limelight exposure 4.7 ms is long for a moving robot; shorten and raise gain if far tags drop out.

## 4. Reading logs without AdvantageScope

`scripts/wpilog.js` is a dependency-free Node reader for `.wpilog` files (Node 18+).

```bash
node scripts/wpilog.js path/to/FRC_x.wpilog list
node scripts/wpilog.js path/to/FRC_x.wpilog dump DS:enabled /Robot/Swerve/State/Pose
```

DogLog keys are prefixed `/Robot/`. In Git Bash set `MSYS_NO_PATHCONV=1` or the `/Robot/...` names get
rewritten as Windows paths. Logs live on the roboRIO under `/home/lvuser/logs` (or `/U/logs` with a USB
stick):

```bash
scp lvuser@10.85.15.2:/home/lvuser/logs/*.wpilog .
```

Useful cross-checks that found today's bugs: wheel-derived `Swerve/State/MeasuredSpeeds.omega` versus
the rate of change of `Swerve/State/Pose` heading (must agree in sign); wheel field velocity versus the
change in any camera's `MT1Pose` over the same second (must agree in direction); each camera's MT1
heading versus the pose heading (turret camera difference is the turret zero error).

## 5. Numbers worth remembering

| Item | Value | Source |
| --- | --- | --- |
| Robot network team number | 8515 (Limelights at 10.85.15.x) | DS / camera IPs |
| Turret zero | facing away from intake; code offset 180 from robot front | CAD, `TurretConfig` |
| Turret pivot to camera | 0.138 m along look direction, camera 18.632 in up | CAD / measured |
| Hub target | blue hub centre (4.63, 4.03) m | `ShotCalc/Target` |
| Turret camera heading error seen on bench | -4.8 to -6 deg steady | `Vision/TurretLL/HeadingErrorDeg` |
