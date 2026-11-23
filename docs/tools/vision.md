# Vision

*Audience: reference. Assumes you have read [2026 season specific](../other-guides/2026-season-specific.md).*

Three Limelights (back, left, right) feed the swerve pose estimator. `Vision` in `robot/subsystems/vision.rs` picks the best camera, decides whether to trust its estimate and hands one measurement to the estimator each loop. The Limelight calls are in `spectrum_lib/vision/`. `limelight_helpers.rs` is a port of the Limelight Java helper library, v1.14.

## Mounting

Camera names, mount transforms, pipeline numbers and default standard deviations are in `VisionConfig`. Two units are easy to mix up:

* `with_translation(x, y, z)` is robot frame meters.
* `with_rotation(roll, pitch, yaw)` is degrees.

The mount transform decides how accurate the estimate is. A wrong mount puts tags in the wrong place on the field, and the estimator blends good odometry with bad vision. That looks like a robot that drifts, not like a vision bug. Measure again after any CAD change.

Pipeline contents, exposure, gain and the AprilTag family are settings on the Limelight web page. They are not in this repo.

## Which estimate is used

The rules are pure functions in `vision.rs`, so tests run them without a camera: `rejection_check`, `mt1_decision`, `mt2_decision` and `reset_pose_to_vision_check`. Read them for the current thresholds, and do not trust a number copied into a doc.

* Each loop `get_best_limelight` picks the camera with the highest tag count plus target size. With no tags in view it picks the back camera.
* While **disabled**, both MegaTag1 and MegaTag2 estimates are fused. This pre-seeds the pose before the match.
* While **enabled**, only MegaTag1 is fused, and only in teleop, or in auton while the `poseUpdate` marker is active or an auton launch is running.
* MegaTag2 takes its heading from the pose that `Vision` sends to the camera. The MegaTag2 estimate ignores its own heading: its rotation standard deviation is `k_large_variance`, which tells the estimator to skip that dimension.
* The shared rejection gate drops a pose outside the field, a capture older than `k_max_time_delta_seconds` (0.1 s), a spin rate of 1.6 rad/s or more, and a target area of 0.025 percent or less.

Two silent behaviors:

* A rejected measurement is dropped with no error. The only symptom is "vision is not updating". `VisionLogger` publishes the accept or reject reason for each camera. Look there.
* When no camera sees a tag, the log blames the back camera, because it is the fallback. That does not mean the others are fine.

## Resetting pose to vision

`reset_pose_to_vision` snaps the estimator onto a vision pose with near-zero standard deviations. It does not blend. The no-argument form does not tell you if the reset was applied. Use `reset_pose_to_vision_with`, which returns a `bool`, when you need to know. A rejected reset leaves the robot where it was and logs the reason under `Vision/PoseReset/Rejection`.

## Post-match video

On `teleop_exit`, when the FMS is attached, `Vision` asks every camera to save the last 165 seconds. That is the longest buffer a Limelight keeps. With no FMS it never fires, so pull the video yourself before you leave the field.

## Not integrated

* Fuel detection. No detection pipeline runs.
* QuestNav. Nothing is written for it.

## See also

* [Auton](auton.md): the `poseUpdate` marker lets `Vision` fuse estimates during auton.
* [Phoenix Tuner X](phoenix-tuner-x.md): gyro calibration, which MegaTag2 depends on.
