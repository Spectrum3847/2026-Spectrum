# Vision systems

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

Three Limelight 4s do AprilTag pose estimation. Each publishes its own MegaTag estimates, and [`Vision`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) decides which to trust, when to fuse them, and at what standard deviation to feed each measurement to the swerve pose estimator.

This page is the reasoning: why the scheme is shaped the way it is, what broke, and how to calibrate from the robot app. The thresholds themselves are constants in `VisionConfig`, each with a comment explaining the number, and they move. Read them there rather than here.

## The hardware, and where its numbers live

Three Limelight 4s, named for where they sit: two on the rear corners, upside down and looking out over each corner, and one on the turret, upright and facing the robot's rear at turret zero. Their NT names, hostnames, mount poses, IMU modes and field layout are all in `VisionConfig` inside `Vision.java`.

Two things that are easy to get wrong:

* **The NT name is also the camera's hostname.** The name a camera is configured with in the code is the name you reach it by on the robot network. If they drift apart, the code configures a camera that is not the one answering.

* **The Java is the source of truth for the mount, not the camera.** `Vision.sendCameraSettings()` writes the mount pose to each chassis camera over NetworkTables every couple of seconds, so anything typed into a camera's own web UI survives about that long. Change the mount in the Java. The turret camera is the deliberate exception, described next.

## The turret camera

A camera on the turret has a robot-to-camera transform that changes every loop. The camera's solve is for the frame it captured, and the turret angle that belongs with it is the one the turret had *then*, not the angle when the solve arrives 30 to 60 ms later. At 90 deg/s those differ by several degrees.

Until 2026-09-15 the robot pushed the live transform to the camera every loop, so the camera applied a several-degree-stale transform to every frame taken while the turret was moving, and the reported pose moved by range times the lag.

The fix is to stop pushing a moving transform. The turret camera is told it sits on the robot centre with no yaw, so what it reports is its own floor-projected position and heading. The robot composes the robot pose from that, the turret angle at the frame's timestamp, and the gyro heading at that time. `Turret` keeps a two-second history of its latency-compensated angle against FPGA time, and `Turret.getAngleAt` is what a solve looks up. The maths lives in [`TurretCameraGeometry`](../../src/main/java/frc/robot/subsystems/vision/TurretCameraGeometry.java), a pure class with a unit test proving the inverse undoes the forward model.

The detail that matters: the lever arm from camera to robot centre is un-rotated by the gyro heading, not by the camera's. So the recovered translation does not inherit the camera's single-tag heading noise, and at that arm length even a five-degree heading error is about 12 mm.

A frame whose timestamp has no turret angle in the history is rejected and labelled on the camera as a missing-angle rejection, not solved with a guess. Teams 341 and 581 both arrived at this same design in 2026; 341's fallback to an identity transform on a missing sample is the one part we did not copy.

`Vision/TurretRioTransform` on the dashboard turns this off and restores the old scheme for comparison at an event. Flipping it re-sends the camera's mount immediately. `Vision/TurretLL/RioTransform` in the log says which mode was active. Note that in the default mode the turret camera's own `MT1Pose` topic is its floor pose, not a robot pose. `Vision/TurretLL/RobotPose` is the composed robot pose, and that is the one the Field2d turret marker shows and the one to compare against the chassis cameras.

The robot app's Cameras page will show the turret camera's forward and yaw as zero, because that is what the code pushes. That is correct. Height, roll and pitch still have to match, and the measure mount procedure below still applies to them.

The pose maths is only as good as the mount transform. On 2026-09-07 all three cameras turned out to be mounted about 30 degrees above horizontal while the code said 60. The angle had been read off the wrong side of a 90-degree bracket, and every pose the cameras reported was displaced accordingly. The robot app's Cameras page is the fix, and the thing that keeps it fixed.

## Calibrating the cameras

Open the [robot app](../../tools/robot-app/README.md) with `./gradlew robotApp`, or double-click `tools/robot-app/start.bat`, and go to the Cameras page. It needs the laptop on the robot network. The robot code does not have to be running, though the page will tell you whether it is.

Each camera gets a card with its stream, its health, and a mount table with three columns: what the Java says, what the camera has saved in its pipeline, and what the camera is currently using. A red cell means the camera is not running what the code says, which usually means the code has not been deployed since the value changed.

Under that, live, is the camera's own accelerometer reading of its pitch against the configured value. This works with no tag in view and it is the quickest sanity check there is. If it is more than a degree off, the bracket or the number is wrong.

### Measuring a mount

1. Park the robot on a flat floor with at least one AprilTag in the camera's view, at a few metres. Nobody touches it.
2. Tick the three checklist items on the card and press **Measure mount**. It samples for five seconds.
3. Read the table. Two independent methods are shown. The accelerometer gives pitch, and a roll magnitude, from gravity in the camera frame, with no height involved. The tag solve gives pitch, roll and height from the camera's pose in the tag's frame, per tag and combined: field tags hang vertically at heights the app knows from the 2026 field layout, so the camera's optical axis against the tag's vertical is its pitch, and its offset below the tag centre is its height. The two should agree on pitch to about a degree. If they do not, the robot is not flat, the tag is not vertical, or a tag solve is being fooled. Look before you write.
4. The **proposal** lists pitch, roll and height against what is in the Java, with the differences that matter pre-ticked. Press **Write to Vision.java**. By default the same values are also saved into the camera's pipeline so it boots correctly before the code has pushed anything.
5. **Deploy.** The robot only pushes what it was built with.

What it will not do: forward, right and yaw. A robot sitting still cannot measure where it is, so those stay CAD. The **pose agreement** table at the top of the page is the check on them. Two cameras that both see tags solve the robot's pose through their own mounts, and if they disagree by more than a few centimetres, one of the mounts is wrong in a way this page cannot fix.

### Tuning exposure, gain and black level

**Auto-tune image** sweeps exposure, then sensor gain, then black level on the running pipeline, holding each setting for a second and a half and scoring how steadily the camera detects the tags in view: detection rate, tag count, ambiguity, corner jitter and pose jitter, with a slight preference for shorter exposure because less exposure is less motion blur once the robot moves. The candidate lists are editable. Nothing is saved until you press **Apply and save to camera**, and the original settings come back when the sweep ends.

Do it with tags in view at a realistic range under the lighting the robot will actually see.

### The yaw sign

Every camera reads its mount yaw back with the opposite sign to what was set: set -135, read +135. It is a reporting convention, not an error. The back-right and turret cameras, with independent mounts, agreed on the robot's field pose to within 5 cm while showing it, and the page compares yaw by magnitude for that reason.

## Why the chassis cameras switch between MT1 and MT2

Both pipelines estimate the robot pose from AprilTag detections, and the difference between them is the reason the vision scheme is shaped the way it is.

MT1 derives a full 3-D pose from camera intrinsics and tag geometry, and it does not need the robot's heading. That independence is worth a lot, but a single-tag MT1 pose has high yaw ambiguity, and even a two-tag heading solve has a long tail, around 15 degrees. At three metres that tail is a quarter metre of sideways error.

MT2 requires the robot's heading, which we feed it from the gyro every loop. That makes it far steadier on the move, because the heading is pinned and only translation is solved. It is also worthless if the pushed heading is wrong, which is exactly the situation before the pose has been seeded.

So the scheme is a switch:

* **While disabled**, only the best chassis camera seeds, and only its MT1 heading feeds the heading corrections. `Vision` watches for that seed to *hold*, over a run of consecutive seeded loops from enough tags with the camera's heading inside a spread limit. When it does, `Vision/PoseSeedConfirmed` goes true, with a progress counter at `Vision/SeedConfirmProgress`. The gross heading correction also confirms the seed, since it has just put a multi-tag heading in the pose. `seedConfirmLoops` and `seedConfirmSpreadDeg` in `VisionConfig` carry the reasoning for their values.

* **While enabled**, the chassis cameras fuse MT2 translation once the seed is confirmed, and MT1 translation until then, because MT1 does not depend on the pushed heading being right. `Vision/ChassisUseMT2` turns the switch off and falls them back to MT1 for the whole enabled period, so the two can be compared at an event without a deploy. `Vision/ChassisSource` in the log says which one every estimate came from.

Heading is never fused during a match. The gyro owns it, except for a gross heading safety net for a boot heading that is grossly wrong.

## When nothing seeded the pose before the match

All three Chezy quals on 2026-09-19 had no camera seeing a tag from the starting position. The pose was therefore whatever `Robot.disabledPeriodic` placed on the selected auto's start, and `Robot` told `Vision` so through `notePlacedAtAutoStart`, which logs `Vision/Placement/HeadingAssumed`. That heading is the working assumption, and there are two ways out of it while enabled.

The first is `seedWhileEnabled`: the first fresh multi-tag solve from the best chassis camera while the robot is slow re-seeds heading and translation. Failing that, a multi-tag turret camera solve does, with the caveat that its heading carries the turret zero error, which `Vision/SeededFromTurretOnly` reports.

The second is `checkPlacementHeading`, and it needs no stillness at all. Any multi-tag solve, chassis or turret camera, whose heading agrees with the placed heading within a tolerance, for a run of camera frames, *confirms* it, because agreement does not move the pose. A steady run of chassis camera frames that disagree instead *refutes* it and re-seeds from that camera. The turret camera never refutes, because its disagreement could equally be the turret zero. The agree, disagree, confirm and refute counters are all under `Vision/Placement/`. In Q17 the turret camera held two to four tags for the first four seconds of auto while the robot drove, and this would have confirmed the heading then instead of twenty seconds into teleop.

Either way `Vision/PoseHeadingSeeded` goes true, which unlocks the turret zero trim and the start-pose check. `PoseSeedConfirmed` still needs the confirmation run described above before the chassis cameras switch to MT2.

## How estimates reach the pose estimator

`Vision.periodic()` runs before the command scheduler each loop. It publishes the robot heading to every camera, flushes NetworkTables once, then runs the disabled or enabled update and logs.

Only the best chassis camera (most tags, then largest target) seeds while disabled. While enabled every chassis camera that passes the gates is fused, along with the turret camera's composed translation. The estimator weights each by the standard deviation its tier assigns, so a one-tag camera on one corner does not drown out a three-tag camera on the other.

Every estimate goes through a common rejection gate before it is fused: no target, too old, outside the field, target too small, spinning too fast, an implied robot heading that disagrees with the gyro by more than a few degrees, and for the turret camera a slew that is too fast, no turret angle at the frame time, or a rejected frame. A 3-D solve that tilts the robot more than a few degrees or lifts it off the carpet is rejected too, because that means a bad solve or a bad mount transform. The threshold names and their justifications are in `VisionConfig`.

Two of those gates look at history rather than the current loop, and both exist because a single loop is not enough to judge motion.

* **Yaw rate** rejects on the *peak* chassis yaw rate over a lookback window, not the current value, because a frame that arrives just after a spin stops was captured during it. The camera stamps MT2 with whatever heading the robot last pushed, so a spin turns latency into heading error and heading error into translation error.
* **Turret slew** uses `Turret.getSlewOmegaRotPerSec()`, the larger of the commanded and the measured turret velocity. The commanded value alone reads zero while `IDLE` slews the turret home at cruise, so a gate that trusted only the command was passing frames whose pushed mount transform lagged the image by several degrees. The turret zero trim's "turret still" test uses the same measurement.

## The turret camera and the turret zero

The turret has no absolute reference, so its zero is wherever it pointed at power-on. The turret camera measures the error directly: the robot heading it implies is built from the turret encoder, so a steady disagreement between that and the pose heading *is* the encoder error. `Vision` trims the zero a fraction of a degree at a time for slip, and re-homes it in one step when the error is gross and steady. The robot app's Turret page shows the history.

Both halves are off unless the operator holds `X`, which has been the case since 2026-09-19. `Vision.setTurretZeroCorrectionEnable()` takes a `BooleanSupplier` bound in `Robot.configureBindings()` to `operator.visionTurretFixX`. Released, the correction routine returns before it reads anything, and the zero stays whatever operator B hand-zeroed it to. `Vision/TurretZero/CorrectionEnabled` logs the state and a console line prints on each edge.

Why it is opt-in: on Chezy Q24 the re-home fired at teleop + 108.3 s and moved the zero 52.4 degrees in one step, off a two-tag solve. `Turret`'s position guard saw the resulting `setPosition` as a one-loop step larger than `MAX_POSITION_STEP_DEGREES`, held it for `POSITION_STEP_CONFIRM_LOOPS`, and then believed it, so the soft limits and every aim after it were in the new frame. The drive team's finding was that the belt had not slipped, which matches the trim's own diagnostic at teleop + 93.9 s: absorbing a pose heading error, not slip. The servo was correcting the turret for an error that was the pose's. Holding `X` restores the old behaviour once someone has decided the zero really is wrong.

The heading error driving that trim is computed the same way as the pose composition, camera-implied robot heading minus gyro heading at the frame time, so it no longer contains the transform lag either. The turret angle history is stored net of zero corrections, and the current correction total is added back on lookup, so a frame captured just before a trim step is read in the corrected zero.

Crossing the enable in either direction drops the trim filter, the sample streak, the slip window, the re-home vote and the divergence latch. They describe a stretch the servo was watching, and after a gap in which it was not allowed to act, none of them describe now.

## Adjusting Limelight settings

The camera owns its pipeline: AprilTag family, decimation, exposure, gain, the field map. The robot owns the mount pose, the IMU mode and the pipeline index. Exposure, gain and black level are best set from the Cameras page as above. Everything else is in the camera's web UI on port 5801. Upload the seasonal AprilTag map before practice.

## See also

* [Robot App](../../tools/robot-app/README.md), the Cameras and Turret pages, and how the app is allowed to edit the vision code.
* [Auton](auton.md), since the pose-update marker is one of the triggers that turns vision integration on during a routine.
* [Phoenix Tuner X](phoenix-tuner-x.md), for gyro calibration, which feeds MT2.
