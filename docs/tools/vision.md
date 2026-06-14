# Vision Systems

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The robot uses three Limelights for AprilTag-based pose estimation. Each one publishes its own MegaTag estimates; the [`Vision`](../../src/main/java/frc/robot/vision/Vision.java) subsystem decides which to trust, when to fuse them, and at what standard deviation to feed each measurement into the swerve pose estimator.

## The Hardware

Three [Limelight 4](https://limelightvision.io)s, named for where they sit on the bot:

| Limelight | NT name | Notes |
| --- | --- | --- |
| Back | `limelight-back` | Wide rear view, mounted high. |
| Left | `limelight-left` | Side view for tags at oblique angles. |
| Right | `limelight-right` | Mirror of left. |

The 3D mounting transforms are in [`Vision.VisionConfig`](../../src/main/java/frc/robot/vision/Vision.java) — `withTranslation(x, y, z)` is robot-frame meters, `withRotation(roll, pitch, yaw)` is degrees (some call sites currently wrap values in `Math.toRadians(...)`, which converts degrees → radians and is therefore incorrect for this API). Update these when CAD changes; the MegaTag pose math is only as good as the camera-to-robot transform you give it.

## MegaTag 1 vs. MegaTag 2

Both are Limelight pipelines that estimate the robot pose from AprilTag detections. The difference matters:

* **MegaTag 1 (MT1)** publishes a full `Pose3d` derived from camera intrinsics + tag geometry. It includes rotation, but a single-tag MT1 pose has high yaw ambiguity (you can't tell which way a flat square is facing from one camera frame).
* **MegaTag 2 (MT2)** publishes a `Pose2d` and *requires* the robot's heading (we feed it from the gyro via `setRobotOrientation`). Because the yaw comes from the gyro, MT2 is much more stable. We log MT2 only when the robot is disabled — when enabled, the gyro is the trusted heading source and MT2 wouldn't add new information.

Our integration scheme reflects that:

* While **disabled**, both MT1 and MT2 from the best Limelight are integrated (good for pre-match auto-zeroing).
* While **enabled** (teleop, auton-launching, or when `RobotStates.autoUpdatePose` is asserted), only MT1 is integrated.

## How Estimates Flow Into the Pose Estimator

`Vision.periodic()` calls `setLimeLightOrientation()` (pushes gyro yaw to each LL), then `disabledLimelightUpdates()` and `enabledLimelightUpdates()` based on the current mode.

Each estimate goes through `getMT1VisionEstimate(...)` or `getMT2VisionEstimate(...)`. Both run a series of rejections before producing a `VisionFieldPoseEstimate`:

1. No target in view → reject.
2. Tag ambiguity > 0.9 → reject (Limelight is unsure which orientation the tag is actually in).
3. Pose is outside the field → reject.
4. Yaw rate > 1.6 rad/s → reject (motion blur kills tag precision).
5. Target apparent size ≤ 0.025 → reject (too far away to matter).
6. MT1 only: roll/pitch > 5 rad → reject (the field is flat; if MT1 thinks we're tilted, it's wrong).

If a measurement survives, it's tagged with standard deviations based on confidence. The full ladder lives in the source, but the shape is:

| Situation | xy std | θ std (MT1) |
| --- | --- | --- |
| Stationary + large target | 0.1 m | 0.1 rad |
| Multi-tag + large target | 0.1 m | 0.1 rad |
| Multi-tag + medium target | 0.25 m | 8 rad |
| Close, large target | 0.5 m | huge (don't fuse) |
| Stable, low ambiguity | 1.5 m | huge |

`integrateSingleEstimate(...)` then calls `swerve.addVisionMeasurement(pose, timestamp, stdDevs)`. The pose estimator weighs that against odometry by the inverse of the stds — small std means "trust this a lot."

## Choosing the Best Limelight

`getBestLimelight()` ranks the three by `tagCountInView + targetSize` and returns the winner. We only integrate from one Limelight per loop. This is deliberate — multi-camera fusion is implemented (`fuseEstimates`, `integrateMultipleEstimates`) but currently unused. The fusion math comes from team 254's 2025 codebase and uses inverse-variance weighting plus an odometry-based projection to align timestamps; it's there if/when we decide single-camera integration leaves accuracy on the table.

## Resetting Pose

`resetPoseToVision()` is a hard reset rather than a normal measurement. It runs a stricter set of checks (no out-of-field, no in-air, no >5 rad tilt) and then calls `addVisionMeasurement` with extremely tight stds (0.00001). This is the move when:

* The robot has been pushed (driver "I have no idea where I am" reset).
* Initial pose at auto start, when the gyro is fresh but the field-frame pose is unknown.

## Adjusting Limelight Settings

The Limelight itself has the source-of-truth pipeline configuration (exposure, gain, AprilTag family, decimation, etc.). The robot code only:

* Picks which pipeline to use (`backTagPipeline`, `leftTagPipeline`, `rightTagPipeline` — all 0 currently).
* Toggles the LED via `blinkLimelights()` / `solidLimelight()` for visual identification during setup.

Everything else — pipeline contents, camera intrinsics calibration, AprilTag map — is set on the Limelight web UI. Upload the seasonal AprilTag map (`AprilTagFields.k2026RebuiltWelded`) before practice.

## Game-Piece Detection (Future Work)

We don't currently run a fuel-detection pipeline. PhotonVision on an Orange Pi was a 2025 experiment for that; the option to revive it for 2026 is open. The plumbing — a `frc.spectrumLib.vision.PhotonVision` wrapper, similar to `Limelight` — is in `spectrumLib` but not wired into a `Vision` config.

## QuestNav

Mentioned in older drafts of this doc but not currently integrated. The plan, if it lands, is to use a Meta Quest's inside-out tracking as an additional pose source — the math integrates cleanly into the same `addVisionMeasurement(...)` pipeline, just with a different sigma profile. No code exists for it in `2026-Spectrum` yet.

## See Also

* [PhotonVision dependency page](../dependencies/photonvision.md) — version, JavaDoc link.
* [Auton](auton.md) — `autoUpdatePose` is one of the triggers that enables vision integration during teleop or auton.
* [Phoenix Tuner X](phoenix-tuner-x.md) — gyro calibration, which feeds MT2.
