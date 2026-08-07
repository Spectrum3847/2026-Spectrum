---
name: frc-vision
description: >
  Spectrum 3847 FRC vision work in Java — Limelight MegaTag AprilTag pose estimation fused into the
  swerve pose estimator. Use when implementing or reviewing AprilTag detection, pose estimation,
  vision-aided odometry, or vision rejection/std-dev tuning. Triggers on: Limelight, MegaTag,
  AprilTags, pose estimation, Vision subsystem, addVisionMeasurement, SwerveDrivePoseEstimator, or
  any vision-related robot code task.
metadata:
  short-description: Limelight MegaTag vision + pose fusion
---

# FRC Vision (Java) — Spectrum 3847

> Vision runs on **Limelight** hardware. Verify class/method names against `src/main/java` before
> trusting external examples.

## Our Setup

- **Subsystem:** [`frc.robot.subsystems.vision.Vision`](../../../src/main/java/frc/robot/subsystems/vision/Vision.java) `implements Subsystem`. It manages three Limelights — `backLL`, `leftLL`, `rightLL` (hostnames `limelight-back/left/right`) — held in `allLimelights`, and fuses them into the swerve `SwerveDrivePoseEstimator`.
- **Wrapper:** [`frc.spectrumLib.vision.Limelight`](../../../src/main/java/frc/spectrumLib/vision/Limelight.java) wraps the vendored `LimelightHelpers`. Real methods: `getMegaTag1_Pose3d()`, `getMegaTag2_Pose2d()`, `getMegaTag1_PoseEstimate()`, `getMegaTag1PoseTimestamp()`, `getTagCountInView()`, `getRawFiducial()` (can return null), `setRobotOrientation(degrees[, rate])`, `sendValidStatus()/sendInvalidStatus()`.
- **Field layout:** WPILib `AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)` — not PhotonLib.

## Each Loop (`Vision.periodic()`)

1. Push robot heading to every Limelight via `setRobotOrientation(...)` so MegaTag2 IMU fusion stays accurate. Do not throttle this — fast-motion accuracy depends on flushing it each loop.
2. Pick the best chassis camera (`getBestLimelight()`, ranked over `swerveLimelights` by tag count + target size).
3. Run the MT1 rejection pipeline (and MT2 while disabled) on the selected camera and add accepted estimates via `addVisionMeasurement(pose, timestamp, stdDevs)`.
4. Log per-camera status through `VisionLogger`.

## What to Check When Editing

- **Timebase:** pose timestamps are converted with Phoenix `Utils.fpgaToCurrentTime(...)` to match the estimator's timebase — keep both operands in the same timebase.
- **Rejection criteria** (`VisionConfig`): stale timestamp (`kMaxTimeDeltaSeconds`), rotation-speed gate, roll/pitch tilt (in degrees, via `Math.toRadians(5)`), out-of-field, tag count/ambiguity/target-size tiers.
- **Std-dev ladder:** `degStds` overrides should only ever *widen* (`Math.max`) toward `kLargeVariance` to discard heading — never narrow a discarded heading back into the fusion. MT2 heading is always discarded; MT1 supplies heading.
- **`resetPoseToVision`** validates both `botpose3D` and the MT2 `megaPose` (out-of-field/height/tilt) before snapping with a tight 0.00001 std-dev.
- **Null safety:** `getRawFiducial()` can return null; guard before dereferencing.
