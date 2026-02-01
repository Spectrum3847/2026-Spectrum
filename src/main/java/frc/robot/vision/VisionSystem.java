// See: https://docs.photonvision.org/en/latest/docs/simulation/simulation.html
package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSystem extends SubsystemBase {
    private final PhotonCamera frontCam = new PhotonCamera("frontCam");
    private final PhotonCamera backCam = new PhotonCamera("backCam");
    private final VisionSystemSim visionSim = new VisionSystemSim("main");
    private final Pose2dSupplier getSimPose;

    Transform3d robotToFrontCamera =
            new Transform3d(
                    new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(22)),
                    new Rotation3d(0, Math.toRadians(-15), 0));
    Transform3d robotToBackCamera =
            new Transform3d(
                    new Translation3d(Units.inchesToMeters(-10), 0, Units.inchesToMeters(22)),
                    new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180)));

    @FunctionalInterface
    public interface Pose2dSupplier {
        Pose2d getPose2d();
    }

    public VisionSystem(Pose2dSupplier getSimPose) {
        this.getSimPose = getSimPose;

        // Setup simulated camera properties
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 960, Rotation2d.fromDegrees(82));
        props.setCalibError(0.25, 0.08);
        props.setFPS(40);
        props.setAvgLatencyMs(35.0);
        props.setLatencyStdDevMs(5.0);

        // Setup simulated camera
        PhotonCameraSim cameraSimFront = new PhotonCameraSim(frontCam, props);
        PhotonCameraSim cameraSimBack = new PhotonCameraSim(backCam, props);
        // Draw field wireframe in simulated camera view
        cameraSimFront.enableDrawWireframe(true);
        cameraSimBack.enableDrawWireframe(true);

        // // Add simulated camera to vision sim
        visionSim.addCamera(cameraSimFront, robotToFrontCamera);
        visionSim.addCamera(cameraSimBack, robotToBackCamera);

        // Add AprilTags to vision sim
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        visionSim.addAprilTags(tagLayout);
    }

    @Override
    public void simulationPeriodic() {
        // Update the vision system with the simulated robot pose
        visionSim.update(getSimPose.getPose2d());
    }
}
