# Vision Systems

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The robot uses three Limelights for AprilTag-based pose estimation. Each one publishes its own MegaTag estimates; the [`Vision`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) subsystem decides which to trust, when to fuse them, and at what standard deviation to feed each measurement into the swerve pose estimator.

## The Hardware

Three Limelight 4s, named for where they sit on the bot:

| Limelight  |      NT name       |                                     Where                                     |                                                              Notes                                                              |
|------------|--------------------|-------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------|
| Back-left  | `limelight-left`   | Rear-left corner panel, upside down, looking out over the corner (yaw +135).  | Chassis camera. IMU mode 1.                                                                                                     |
| Back-right | `limelight-right`  | Rear-right corner panel, upside down, looking out over the corner (yaw -135). | Chassis camera. IMU mode 1.                                                                                                     |
| Turret     | `limelight-turret` | On the turret, upright, facing the robot rear at turret zero.                 | Told it has no planar offset and no yaw; the robot composes the robot pose from the turret angle at the frame time. IMU mode 0. |

The NT names are also the cameras' hostnames, so `limelight-left.local` and friends reach them on the robot network.

## Where the mount numbers live

Each camera's robot-relative pose is a `LimelightConfig` in [`Vision.VisionConfig`](../../src/main/java/frc/robot/subsystems/vision/Vision.java): `withTranslation(forward, right, up)` in metres and `withRotation(roll, pitch, yaw)` in degrees, in exactly the order and sign the Limelight web UI uses. Pitch is positive with the camera tilted up; an upside-down camera has roll 180.

**The Java is the source of truth, not the camera.** `Vision.sendCameraSettings()` writes those six numbers to each chassis camera over NetworkTables every couple of seconds, so whatever is typed into a camera's web UI survives about that long. Change the mount, change the Java. The turret camera is different: it is sent zero forward, zero right and zero yaw with its real height, roll and pitch, because its planar offset and yaw change with the turret and are applied on the robot instead (next section).

## The turret camera

A camera on the turret has a robot-to-camera transform that changes every loop. The camera's solve is for the frame it captured, and the turret angle that belongs with it is the one the turret had *then*, not when the solve arrives 30 to 60 ms later. At 90 deg/s those differ by several degrees, and until 2026-09-15 the robot pushed the live transform to the camera every loop, so the camera applied a transform several degrees stale to every frame taken while the turret was moving and the reported pose moved by range times the lag.

Now the turret camera is told it sits on the robot centre with no yaw, so its `botpose` is its own floor-projected position and heading. `Turret` keeps a two-second `TimeInterpolatableBuffer` of its latency-compensated angle against FPGA time (`Turret.getAngleAt`), and `Vision.solveTurretCamera()` composes the robot pose from the botpose, the turret angle at the frame's timestamp and the gyro heading at that time. The maths is in [`TurretCameraGeometry`](../../src/main/java/frc/robot/subsystems/vision/TurretCameraGeometry.java), which is a pure class with a unit test proving the inverse undoes the forward model. The gyro heading, not the camera's, un-rotates the 0.138 m lever arm from camera to robot centre, so the recovered translation does not inherit the camera's single-tag heading noise; at that arm length even a 5 deg error is 12 mm.

The heading error that drives the turret zero trim is computed the same way, camera-implied robot heading minus gyro heading at the frame time, so it no longer contains the transform lag either. The turret angle history is stored net of zero corrections and the current correction total is added back on lookup, so a frame captured just before a trim step is read in the corrected zero.

A frame whose timestamp has no turret angle in the history is rejected (`No Turret Angle At Frame Rejection`, logged at `Vision/TurretLL/AngleMissingAtFrame`), not solved with a guess. 341 and 581 both arrived at this same design in 2026; 341's fallback to an identity transform on a missing sample is the one part not copied.

`Vision/TurretRioTransform` on the dashboard turns this off and restores the old scheme (push the live transform, fuse the camera's MegaTag2) for comparison at an event. Flipping it re-sends the camera's mount immediately. `Vision/TurretLL/RioTransform` in the log says which mode was active. Note that in the default mode `Vision/TurretLL/MT1Pose` is the camera's floor pose, not a robot pose; `Vision/TurretLL/RobotPose` is the composed robot pose, and it is what the Field2d turret marker shows.

The robot app's Cameras page compares the turret camera's mount against `Vision.java` and will show forward and yaw as zero on the camera; that is correct for this camera. Height, roll and pitch still have to match, and the Measure mount procedure still applies to them.

The MegaTag pose maths is only as good as this transform. On 2026-09-07 all three cameras turned out to be mounted at about 30 deg above horizontal while the code said 60 -- the angle had been read off the wrong side of a 90-degree bracket -- and every pose the cameras reported was displaced accordingly. The fix, and the way to keep it fixed, is the robot app's Cameras page.

## Calibrating the cameras

Open the [robot app](../../tools/robot-app/README.md) (`./gradlew robotApp`, or double-click `tools/robot-app/start.bat`) and go to **Cameras**. It needs the laptop on the robot network; the robot code does not have to be running, though the page will tell you when it is.

Each camera gets a card with its stream, its health (fps, temperature, tags in view) and a **mount table** with three columns: what `Vision.java` says, what the camera has saved in its pipeline, and what the camera is currently using. A red cell means the camera is not running what the code says, which usually means the code has not been deployed since the value changed.

Under that, live, is the camera's own **accelerometer** reading of its pitch against the configured value. This works with no tag in view and is the quickest sanity check there is: if it is more than a degree off, something is wrong with either the bracket or the number.

### Measuring a mount

1. Park the robot on a flat floor with at least one AprilTag in the camera's view, at a few metres. Nobody touches it.
2. Tick the three checklist items on the card and press **Measure mount**. It samples for five seconds.
3. Read the table. Two independent methods are shown:
   * **Accelerometer**: pitch (and a roll magnitude) from gravity in the camera frame. No height.
   * **Tag solve**, per tag and combined: pitch, roll and height from the camera's pose in the tag's frame. Field tags hang vertically at heights the app knows from the 2026 field layout, so the camera's optical axis against the tag's vertical is its pitch and its offset below the tag centre is its height.
     The two should agree on pitch to about a degree. If they don't, the robot is not flat, the tag is not vertical, or a tag solve is being fooled -- look before writing.
4. The **proposal** lists pitch, roll and height against what is in the Java, with the differences that matter pre-ticked. Press **Write to Vision.java**. By default the same values are also saved into the camera's pipeline so it boots correctly before the code has pushed anything.
5. **Deploy.** The robot only pushes what it was built with.

What it will not do: forward, right and yaw. A robot sitting still cannot measure where it is, so those stay CAD. The **pose agreement** table at the top of the page is the check on them: two cameras that both see tags solve the robot's pose through their own mounts, and if they disagree by more than a few centimetres, one of the mounts is wrong in a way this page cannot fix.

### Tuning exposure, gain and black level

The **Auto-tune image** button sweeps exposure, then sensor gain, then black level on the running pipeline, holding each setting for a second and a half and scoring how steadily the camera detects the tags in view: detection rate, tag count, ambiguity, corner jitter and pose jitter, with a slight preference for shorter exposure because less exposure is less motion blur once the robot moves. The candidate lists are editable. Nothing is saved until you press **Apply and save to camera**; the original settings come back the moment the sweep ends.

Do it with tags in view at a realistic range under the lighting the robot will actually see.

### The yaw sign

Every camera reads its mount yaw back with the opposite sign to what was set: set -135, read +135. It is a reporting convention, not an error -- the back-right and turret cameras, with independent mounts, agreed on the robot's field pose to 5 cm while showing it -- and the page compares yaw by magnitude for that reason.

## MegaTag 1 vs. MegaTag 2

Both are Limelight pipelines that estimate the robot pose from AprilTag detections. The difference matters:

* **MegaTag 1 (MT1)** publishes a full `Pose3d` derived from camera intrinsics + tag geometry. It includes rotation, but a single-tag MT1 pose has high yaw ambiguity (you can't tell which way a flat square is facing from one camera frame).

* **MegaTag 2 (MT2)** publishes a `Pose2d` and *requires* the robot's heading (we feed it from the gyro via `setRobotOrientation`). Because the yaw comes from the gyro, MT2 is much more stable.

While **disabled**, the best chassis camera's MT1 seeds the pose, translation and heading. While it does, `Vision` watches for the seed to *hold*: `seedConfirmLoops` consecutive seeded loops from two or more tags with the camera's heading inside `seedConfirmSpreadDeg`. When that happens `Vision/PoseSeedConfirmed` goes true (there is a progress counter at `Vision/SeedConfirmProgress`), and from then on the chassis cameras fuse **MT2** translation while enabled. Until then they fuse MT1 translation, which does not depend on the pushed heading being right. The gross-heading correction also confirms the seed, since it has just put a multi-tag heading in the pose.

The reason for the switch: MT1's translation moves with its own heading solve, and at two tags that heading has a 15 deg tail, which at three metres is a quarter metre sideways. MT2 pins heading to the gyro and solves translation alone, so it is far steadier on the move, but it is worthless if the pushed heading is wrong, which is exactly the case before seeding.

`Vision/ChassisUseMT2` on the dashboard turns the switch off, which falls the chassis cameras back to MT1 for the whole enabled period. It exists so the two can be compared at an event without a deploy; `Vision/ChassisSource` in the log says which one every estimate came from.

While **enabled**, both chassis cameras' translations (MT2 or MT1 as above) and the turret camera's composed MegaTag1 translation (see [The turret camera](#the-turret-camera)) are fused, never heading -- the gyro owns heading during a match -- unless the gross-heading safety net fires (`checkGrossHeadingError`, for a boot heading that is 90 or 180 deg out). The thresholds and the reasoning behind each are documented at length in `VisionConfig`.

**When nothing seeded the pose before the match** (all three Chezy quals on 2026-09-19: no camera saw a tag from the starting position), the pose is whatever `Robot.disabledPeriodic` placed on the selected auto's start, and `Robot` tells `Vision` so (`notePlacedAtAutoStart`, logged as `Vision/Placement/HeadingAssumed`). That heading is the working assumption and there are two ways out of it while enabled:

* `seedWhileEnabled`: the first fresh multi-tag solve from the best chassis camera while the robot is slow re-seeds heading and translation; failing that, a multi-tag turret-camera solve does, with the caveat that its heading carries the turret zero error (`Vision/SeededFromTurretOnly`).
* `checkPlacementHeading`: no stillness needed. Any multi-tag solve, chassis or turret camera, whose heading agrees with the placed heading within `placementAgreeDeg` (8 deg) for `placementDecideFrames` (10) camera frames *confirms* it, since agreement does not move the pose. Ten steady chassis-camera frames that disagree *refute* it and re-seed from that camera. The turret camera never refutes, because its disagreement could equally be the turret zero. Counters: `Vision/Placement/{Chassis,Turret}AgreeFrames`, `ChassisDisagreeFrames`, `ConfirmCount`, `RefuteCount`. In Q17 the turret camera held two to four tags for the first four seconds of auto while the robot drove; this would have confirmed the heading then instead of twenty seconds into teleop.

Either way `Vision/PoseHeadingSeeded` goes true, which unlocks the turret zero trim and the start-pose check; `PoseSeedConfirmed` still needs the confirmation run above before the chassis cameras switch to MT2.

## How Estimates Flow Into the Pose Estimator

`Vision.periodic()` runs before the command scheduler each loop. It publishes the turret-rotated camera transform and the robot heading to every camera, flushes NetworkTables once, then runs the disabled or enabled update and logs.

Only the best chassis camera (most tags, then largest target) seeds while disabled, and only its MegaTag1 heading feeds the gross and consensus heading corrections. While enabled every chassis camera that passes the gates is fused; the estimator weights each by the standard deviation its tier assigns, so a one-tag camera on one corner does not drown out a three-tag camera on the other.

Each estimate goes through a common rejection gate before it is fused: no target, too old, outside the field, target too small, spinning too fast, or (for the turret camera) slewing too fast, no turret angle recorded for the frame time, or an implied robot heading that disagrees with the gyro by more than a few degrees -- which, because that heading is built from the turret encoder, means the turret zero is off. MegaTag1 estimates are additionally rejected when the 3-D solve tilts the robot more than 5 deg or lifts it more than `maxZErrorMeters` off the carpet, either of which means a bad solve or a bad mount transform. Survivors are fused with standard deviations chosen per estimate from tag count and target size.

Two of those gates look at history, not just the current loop:

* **Yaw rate** rejects on the *peak* chassis yaw rate over the last `yawRateLookbackSeconds` (0.3 s), because a frame that arrives just after a spin stops was captured during it. The camera stamps MegaTag2 with whatever heading the robot last pushed, so a spin turns latency into heading error and heading error into translation error.
* **Turret slew** uses `Turret.getSlewOmegaRotPerSec()`, the larger of the commanded and the measured turret velocity. The commanded value alone reads zero while `IDLE` slews the turret home at cruise, and a gate that trusted it was passing frames whose pushed mount transform lagged the image by several degrees. The turret zero trim's "turret still" test uses the same measurement.

## The turret camera and the turret zero

The turret has no absolute reference, so its zero is wherever it pointed at power-on. The turret camera measures the error directly: the robot heading it implies is built from the turret encoder, so a steady disagreement between that and the pose heading *is* the encoder error. `Vision` trims the zero a fraction of a degree at a time for slip, and re-homes it in one step when the error is gross and steady. The robot app's **Turret** page shows the history.

**Both halves are off unless the operator holds `X`** (since 2026-09-19). `Vision.setTurretZeroCorrectionEnable()` takes a `BooleanSupplier`, bound in `Robot.configureBindings()` to `operator.visionTurretFixX`; released, `correctTurretZero()` returns before it reads anything and the zero stays whatever operator-B hand-zeroed it to. `Vision/TurretZero/CorrectionEnabled` logs the state and a console line prints on each edge.

Why it is opt-in: on Chezy Q24 the re-home fired at teleop+108.3 s and moved the zero 52.4 deg in one step off a 2-tag solve. `Turret`'s position guard saw the resulting `setPosition` as a >15 deg one-loop step, held it the usual 10 loops and then believed it (`131.0 deg became 169.2 deg`), so the soft limits and every aim after it were in the new frame. The drive team's finding was that the belt had not slipped -- which matches the trim's own diagnostic at teleop+93.9 s, "absorbing a pose heading error, not slip" -- so the servo was correcting the turret for an error that was the pose's. Holding `X` restores the old behaviour when someone has decided the zero really is wrong.

Crossing the enable in either direction drops the trim filter, the sample streak, the slip window, the re-home vote and the divergence latch (`resetTurretZeroServoState`): they describe a stretch the servo was watching, and after a gap in which it was not allowed to act, none of them describe now.

## Adjusting Limelight Settings

The camera owns its pipeline (AprilTag family, decimation, exposure, gain, the field map); the robot owns the mount pose, the IMU mode and the pipeline index. Exposure, gain and black level are best set from the Cameras page as above; everything else is in the camera's web UI on port 5801. Upload the seasonal AprilTag map before practice.

## See Also

* [Robot App](../../tools/robot-app/README.md) -- the Cameras page, and how it is allowed to edit `Vision.java`.
* [Auton](auton.md) -- `autoUpdatePose` is one of the triggers that enables vision integration during teleop or auton.
* [Phoenix Tuner X](phoenix-tuner-x.md) -- gyro calibration, which feeds MT2.
