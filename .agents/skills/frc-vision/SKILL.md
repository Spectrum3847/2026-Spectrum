---
name: frc-vision
description: >
  FRC vision processing best practices for Java — PhotonVision, Limelight, and WPILib pose estimation.
  Use when implementing or reviewing AprilTag detection, pose estimation, vision-aided odometry, or
  vision-based targeting. Triggers on: PhotonVision, Limelight, AprilTags, pose estimation,
  VisionSubsystem, addVisionMeasurement, PoseEstimator, or any vision-related robot code task.
---

# FRC Vision (Java)

## Overview

Covers FRC vision in Java: PhotonVision, Limelight, and WPILib pose estimation.

- **PhotonVision** — AprilTag pipelines on supported cameras (Orange Pi / co-processor), `PhotonCamera` reads, multi-tag results.
- **Limelight** — MegaTag 1/2 pipelines, `LimelightHelpers`, pipeline selection, LED control.
- **WPILib pose estimation** — `SwerveDrivePoseEstimator` / `PoseEstimator`, `addVisionMeasurement(pose, timestamp, stdDevs)`, standard-deviation tuning, vision-vs-odometry weighting, and fusion gotchas (latency, double-counting).

When implementing or reviewing, check: camera-to-robot transform, rejection criteria (ambiguity, distance, age), a sensible std-dev ladder, and whether vision feeds targeting (turret) or only localization.
