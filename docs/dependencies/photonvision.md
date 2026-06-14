# PhotonVision (PhotonLib)

*Audience: Reference. Assumes you've read [Dependencies Overview](overview.md).*

PhotonVision is the vision coprocessor stack — typically an Orange Pi or a Limelight running PhotonVision firmware — that handles AprilTag detection and game-piece recognition. PhotonLib is its on-robot Java client: `PhotonCamera`, pose helpers, and a sim API.

Vendor JSON: [`vendordeps/photonlib.json`](../../vendordeps/photonlib.json).

## What's Wired Up Today

The real-robot vision pipeline currently goes through Limelights, wrapped by [`Limelight.java`](../../src/main/java/frc/spectrumLib/vision/Limelight.java) (covered in [Vision](../tools/vision.md)). PhotonLib is in the codebase for two reasons.

First, there's a placeholder `PhotonCamera` in [`VisionSystem.java`](../../src/main/java/frc/robot/vision/VisionSystem.java) waiting for an Orange Pi camera to be named:

```java
private final PhotonCamera camera = new PhotonCamera("cameraName");
```

The constructor opens a NetworkTables subscription with that name. The string is literally `"cameraName"` right now — rename it the moment real hardware exists, otherwise the logs fill with "no camera found" warnings.

Second, and more importantly today, PhotonLib's sim API is what makes AprilTags show up in the WPILib simulator:

```java
SimCameraProperties props = new SimCameraProperties();
props.setCalibError(0.25, 0.08);
props.setFPS(20.0);
props.setAvgLatencyMs(35.0);
props.setLatencyStdDevMs(5.0);
```

Those numbers should mirror whatever the real coprocessor measures. Optimistic FPS and latency produce a sim pose that's smoother than reality, which hides exactly the tuning issues sim is supposed to surface.

## How the Sim Hooks In

`VisionSystem` holds one `VisionSystemSim` and adds the field tag layout to it:

```java
AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
visionSim.addAprilTags(tagLayout);
```

`AprilTagFieldLayout` is WPILib — but PhotonLib's sim relies on it. Don't substitute a hand-built layout unless you're explicitly testing a non-standard field; the WPILib enum tracks the official FRC release.

The simulated robot pose is fed in every loop:

```java
@Override
public void simulationPeriodic() {
    visionSim.update(getSimPose.getPose2d());
}
```

`getSimPose` is `swerve::getRobotPose`, passed in from `Robot.java`. If you ever construct `VisionSystem` somewhere else, pass an equivalent pose supplier or the sim goes silent.

The commented-out blocks in `VisionSystem.java` show what real-camera wiring looks like: `PhotonCameraSim`, `enableDrawWireframe`, `visionSim.addCamera(...)`. Uncomment those when you commit to a camera name. `enableDrawWireframe(true)` is great for debugging but expensive to render — leave it off for normal sim runs.

## The Real-Robot Loop, For When We Get There

When a real camera is plumbed in, the read loop looks roughly like this:

1. `PhotonCamera.getAllUnreadResults()` returns one or more `PhotonPipelineResult`s.
2. For an AprilTag pipeline, pull `getMultiTagResult()` and turn it into a `Pose2d` + timestamp.
3. Feed it to the swerve estimator via `addVisionMeasurement(pose, timestamp, stdDevs)`.

Use a `Matrix<N3, N1>` for the std-devs — small numbers for confident multi-tag fixes, bigger numbers when only one tag is visible. If PhotonVision and Limelight both end up publishing vision measurements, give them distinct std-dev matrices so the estimator weights them appropriately.

## Further Reading

[PhotonVision JavaDoc](https://javadocs.photonvision.org/release/) is linked into our generated docs. The [Simulation Guide](https://docs.photonvision.org/en/latest/docs/simulation/) is the source the comment at the top of `VisionSystem.java` points to. For the broader vision strategy, see [Vision](../tools/vision.md).
