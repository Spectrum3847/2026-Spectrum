# Vision systems

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

Three Limelights feed the swerve pose estimator. [`Vision`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) decides which camera to trust, which of the two MegaTag pipelines to use in the current mode, and which single measurement to hand the estimator each loop.

## Mounting

Camera names, mount transforms, pipeline numbers, and the default measurement sigmas are all in `Vision.VisionConfig`. Two conventions there are easy to get wrong and hard to debug:

* `withTranslation(x, y, z)` is robot frame **metres**.
* `withRotation(roll, pitch, yaw)` is **degrees**.

The mount transform is the whole accuracy budget. Get it wrong and the tags resolve to the wrong place on the field, so the estimator blends a good odometry estimate with a bad vision one. That presents as a robot that drifts, not as a vision bug, and you will chase the wrong thing for a day. Re-measure after any CAD change.

The AprilTag field layout is loaded in the `Vision` constructor from WPILib's `AprilTagFields`, so there is nothing to upload to the Limelights. Pipeline contents, exposure, gain, and the AprilTag family are the Limelight's own settings on its web UI, and are the one part of this system that is not in the repo.

## Which estimate gets used

The rule lives in `Vision.periodic()` and the two update methods it calls, and it comes down to this. MegaTag1's heading comes from tag geometry and stays usable while the robot is moving. MegaTag2's heading comes from the Limelight's own IMU and drifts as soon as the robot moves, so MT2 is only fused while the robot is disabled, and its rotational standard deviation is set to `VisionConfig.kLargeVariance`, which tells the estimator to ignore that dimension entirely.

Measurements can be rejected, and the rejection thresholds are the literals inside `rejectionCheck(...)` and `getMT1VisionEstimate(...)` in `Vision.java`. The confidence tiers that pick each measurement's sigmas are the `sendValidStatus` branches in those same two methods. Read them there rather than trusting a number written down anywhere else, including here.

Two behaviors worth knowing because they are silent:

* A rejected measurement returns null and the caller ignores it. Nothing is logged as an error, so the only symptom of a badly tuned threshold is "vision is not updating." `VisionLogger` publishes the accept or reject reason per camera, and that is where to look.
* `getBestLimelight()` falls back to the back camera when no camera sees a tag, so the reject reason you get is always that one camera's. When all three go blind, the log blames the back camera and nothing tells you the others are fine.

## Resetting pose to vision

`resetPoseToVision()` snaps the estimator onto a vision pose with near-zero covariance, so it jumps instead of blending. It is not a measurement. The no-argument overload does not expose whether the reset was applied, so call the four-argument form if you need to know, and log it when you do, because a rejected reset leaves the robot where it was and looks like the reset not working.

Use it when the robot has been pushed and nobody knows where it is, and before an auto when the gyro is fresh but the field-frame pose is not.

## Post-match video

`teleopExit` asks every camera to rewind-capture 165 seconds, which is the longest buffer a Limelight keeps, but only when a FMS is attached. At practice with no FMS it never fires, so the footage of a bad match is not there when you go back for it. Pull the video before you leave the field either way.

## Not integrated

Two things have been discussed and deliberately have no code. Do not assume either works.

* **Fuel detection.** No detection pipeline runs, and there is nothing in this repo for one. A neural detector on the Limelights or a separate coprocessor would both be new work.
* **QuestNav.** Meta Quest inside-out tracking as an extra pose source. It would feed the same `addVisionMeasurement` call with a different sigma profile, and nothing has been written.

## See also

* [Auton](auton.md): `autonPoseUpdate` is one of the conditions that enables vision integration while enabled.
* [Phoenix Tuner X](phoenix-tuner-x.md): gyro calibration, which feeds MT2.
