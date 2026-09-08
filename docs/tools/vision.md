# Vision Systems

*Audience: Reference. Assumes you've read [2026 Season Specific](../other-guides/2026-season-specific.md).*

The robot uses three Limelights for AprilTag-based pose estimation. Each one publishes its own MegaTag estimates; the [`Vision`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) subsystem decides which to trust, when to fuse them, and at what standard deviation to feed each measurement into the swerve pose estimator.

## The Hardware

Three Limelight 4s, named for where they sit on the bot:

| Limelight | NT name | Where | Notes |
| --- | --- | --- | --- |
| Back-left | `limelight-left` | Rear-left corner panel, upside down, looking out over the corner (yaw +135). | Chassis camera. IMU mode 1. |
| Back-right | `limelight-right` | Rear-right corner panel, upside down, looking out over the corner (yaw -135). | Chassis camera. IMU mode 1. |
| Turret | `limelight-turret` | On the turret, upright, facing the robot rear at turret zero. | Its mount yaw is the live turret angle, pushed every loop. IMU mode 0. |

The NT names are also the cameras' hostnames, so `limelight-left.local` and friends reach them on the robot network.

## Where the mount numbers live

Each camera's robot-relative pose is a `LimelightConfig` in [`Vision.VisionConfig`](../../src/main/java/frc/robot/subsystems/vision/Vision.java): `withTranslation(forward, right, up)` in metres and `withRotation(roll, pitch, yaw)` in degrees, in exactly the order and sign the Limelight web UI uses. Pitch is positive with the camera tilted up; an upside-down camera has roll 180.

**The Java is the source of truth, not the camera.** `Vision.sendCameraSettings()` writes those six numbers to each chassis camera over NetworkTables every couple of seconds (and `updateTurretCameraPose()` writes the turret's every loop), so whatever is typed into a camera's web UI survives about that long. Change the mount, change the Java.

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

While **disabled**, the best chassis camera's MT1 seeds the pose, translation and heading. While **enabled**, that camera's MT1 translation and the turret camera's MT2 translation are fused, never heading -- the gyro owns heading during a match -- unless the gross-heading safety net fires (`checkGrossHeadingError`, for a boot heading that is 90 or 180 deg out). The thresholds and the reasoning behind each are documented at length in `VisionConfig`.

## How Estimates Flow Into the Pose Estimator

`Vision.periodic()` runs before the command scheduler each loop. It publishes the turret-rotated camera transform and the robot heading to every camera, flushes NetworkTables once, then runs the disabled or enabled update and logs.

Each estimate goes through a common rejection gate before it is fused: no target, too old, outside the field, moving too fast, target too small, or (for the turret camera) a MegaTag1 heading that disagrees with the gyro by more than a few degrees -- which, because its mount is built from the turret encoder, means the turret zero is off. Survivors are fused with standard deviations chosen per estimate from tag count and target size.

## The turret camera and the turret zero

The turret has no absolute reference, so its zero is wherever it pointed at power-on. The turret camera measures the error directly: its mount transform is built from the turret encoder, so a steady disagreement between its MegaTag1 heading and the pose heading *is* the encoder error. `Vision` trims the zero a fraction of a degree at a time for slip, and re-homes it in one step when the error is gross and steady. The robot app's **Turret** page shows the history.

## Adjusting Limelight Settings

The camera owns its pipeline (AprilTag family, decimation, exposure, gain, the field map); the robot owns the mount pose, the IMU mode and the pipeline index. Exposure, gain and black level are best set from the Cameras page as above; everything else is in the camera's web UI on port 5801. Upload the seasonal AprilTag map before practice.

## See Also

* [Robot App](../../tools/robot-app/README.md) -- the Cameras page, and how it is allowed to edit `Vision.java`.
* [Auton](auton.md) -- `autoUpdatePose` is one of the triggers that enables vision integration during teleop or auton.
* [Phoenix Tuner X](phoenix-tuner-x.md) -- gyro calibration, which feeds MT2.
