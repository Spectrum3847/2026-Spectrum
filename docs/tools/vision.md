# Vision Systems

## What is Vision?

Vision systems on an FRC robot typically involve cameras and specialized software to interpret visual data from the field.

*   **Cameras**: Used to capture images of the field.
*   **AprilTags**: These are like QR codes placed strategically on the field. The robot uses them to:
    *   **Localize Itself**: Determine its precise position and orientation on the field (known points on the field map).
*   **Game Piece Detection**: Cameras can be trained to find other objects like game pieces (e.g., notes, retro-reflective tape).

## Why Vision?

*   **Automating Tasks**: Essential for tasks that require precise targeting or object interaction.
    *   Turning to score.
    *   Detecting game pieces.

## LimeLight Configuration (LimeLight Configs)

LimeLights are popular FRC cameras. Each LimeLight has its own pipeline configuration.

### Software (Main) Configs:

*   **AprilTags**: Used in relation to vision for robot localization.
    *   Current field map needs to be uploaded to the LimeLight.
*   **Color Detection**: Used to detect different game pieces (e.g., notes in 2024).
*   **TA (Target Area)**: Defines the AprilTag's area seen by a camera.
*   **Exposure**: Camera exposure settings.
*   **Sensor Gain**: Camera sensor gain settings.

### Physical Config:

*   Based on the camera's physical position on the robot, which depends on the CAD design.

## Vision (LimeLight)

### Setting Physical Configuration

Two primary options for setting the physical configuration of the LimeLight:

1.  **Download your physical config onto the LimeLight**: Directly configure the camera via its web interface.
2.  **Setting the config in a software file**: Define the configuration parameters within the robot code.

### LimeLight Software Configs - Values

*   **MegaPose**: A class dedicated to storing pose values.
    *   **MegaTag1**: Represents raw rotational data, often more ambiguous.
    *   **MegaTag2**: With LL4 (LimeLight 4), cameras include their own gyros for more precise positional rotation.
*   **Exposure** and **Sensor Gain**: Control the image quality and brightness from the camera.

## Vision Alignment

Types of Alignment:
*   **Tag Area**: Alignment based on the detected area and relative position of AprilTags, primarily used for coarse or direct alignment to a tag.
*   **QuestNav**: This likely refers to an alignment strategy leveraging QuestNav's advanced pose estimation for more precise or global field alignment.

### Pose Alignment

Pose alignment methods in vision systems involve using camera data (e.g., AprilTag detections) to correct and refine the robot's estimated position (pose).

*   **Purpose**: To ensure the robot knows its exact location and orientation on the field, which is critical for accurate autonomous routines and precise mechanism control (e.g., shooting, game piece manipulation).
*   **Method**: By comparing the robot's current estimated pose with its pose as determined by vision targets, the system can calculate an error and adjust the robot's pose estimate.
*   **Integration**: Our `VisionSystem` takes `swerve::getRobotPose`, indicating that vision measurements are integrated into the swerve drive's odometry for continuous pose correction.

## MegaTags (LimeLights)

*   **MegaTag 1 (MT1)**: Rotations are generally more ambiguous as it's a more "raw data" value compared to MT2.
*   **MegaTag 2 (MT2)**: With LL4 (LimeLight 4), cameras have their own gyros that provide their own positional rotation, leading to more accurate pose estimates.

## QuestNav Pose

QuestNav Pose refers to the robot's estimated position and orientation as determined by the QuestNav system. It aims to provide a highly accurate and robust pose estimate by potentially fusing data from multiple sensors beyond just LimeLight.

## Pose Alignment (Vision) - Adding Vision Measurements

*   **Purpose**: Using a given MegaTag value to create a position that acts as a constant for a robot pose.
*   **Implementation**: Must be done using `Periodic` methods (e.g., in a subsystem's `periodic()` method).
*   **WPILib Command (2026)**: `addVisionMeasurement(PoseMeters, timestampSeconds, visionMeasurementStdDevs)`
    *   `PoseMeters`: Defines the Vision's pose estimation.
    *   `timestampSeconds`: When the measurement was taken.
    *   `visionMeasurementStdDevs`: Standard deviations for the vision measurement, reflecting confidence in the measurement.

## Why QuestNav?

QuestNav is a system or approach likely developed to enhance navigation and pose estimation beyond standard WPILib functionalities, potentially by integrating various sensor inputs and advanced algorithms.

*   **Advanced Navigation**: QuestNav aims to provide more robust and accurate pose estimation, particularly in complex field environments or when standard AprilTag detections might be sparse.
*   **Logging and Analysis**: Even if not used for direct robot control, QuestNav can be invaluable for logging detailed pose data, enabling offline analysis and debugging of navigation performance. This helps identify discrepancies and areas for improvement in the robot's localization.
