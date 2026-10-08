package frc.spectrumLib.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.spectrumLib.vision.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Verifies the per-loop NetworkTables snapshot in {@link Limelight}: values are read once per loop,
 * held until {@link Limelight#invalidate()}, and detached cameras never touch NT.
 *
 * <p>Uses the default NetworkTables instance in-process (no server). Every test uses a unique table
 * name and never closes the default instance, because {@link LimelightHelpers} caches entry handles
 * for the life of the JVM.
 */
public class LimelightCacheTest {

    private static final double EPS = 1e-9;

    private String tableName;
    private NetworkTable table;
    private final List<AutoCloseable> publishers = new ArrayList<>();

    @BeforeEach
    void setUp() {
        tableName = "limelight-test-" + UUID.randomUUID();
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    @AfterEach
    void tearDown() throws Exception {
        for (AutoCloseable publisher : publishers) {
            publisher.close();
        }
        publishers.clear();
    }

    // ---- fixtures -------------------------------------------------------------------------------

    /**
     * Builds a botpose array: [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgDist,
     * avgArea, then 7 values per fiducial].
     */
    private static double[] botpose(
            double x, double y, double yawDeg, double latencyMs, double[]... fiducials) {
        double[] out = new double[11 + 7 * fiducials.length];
        out[0] = x;
        out[1] = y;
        out[5] = yawDeg;
        out[6] = latencyMs;
        out[7] = fiducials.length;
        for (int i = 0; i < fiducials.length; i++) {
            System.arraycopy(fiducials[i], 0, out, 11 + 7 * i, 7);
        }
        return out;
    }

    /** One fiducial: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity. */
    private static double[] fiducial(int id, double ambiguity) {
        return new double[] {id, 0.1, 0.2, 1.5, 2.0, 2.5, ambiguity};
    }

    private DoubleArrayPublisher arrayPublisher(String key) {
        DoubleArrayPublisher publisher = table.getDoubleArrayTopic(key).publish();
        publishers.add(publisher);
        return publisher;
    }

    private DoublePublisher doublePublisher(String key) {
        DoublePublisher publisher = table.getDoubleTopic(key).publish();
        publishers.add(publisher);
        return publisher;
    }

    // ---- tests ----------------------------------------------------------------------------------

    @Test
    @DisplayName("MT1 getters hold one sample until invalidate()")
    void mt1CacheHoldsUntilInvalidate() {
        DoubleArrayPublisher pose = arrayPublisher("botpose_wpiblue");
        DoublePublisher tv = doublePublisher("tv");
        DoublePublisher ta = doublePublisher("ta");
        Limelight limelight = new Limelight(tableName);

        pose.set(botpose(1.0, 2.0, 30.0, 20.0, fiducial(7, 0.1)), 1_000_000L);
        tv.set(1.0);
        ta.set(3.5);

        assertEquals(1.0, limelight.getMegaTag1_Pose3d().getX(), EPS);
        assertEquals(1, limelight.getTagCountInView(), EPS);
        assertEquals(7, limelight.getRawFiducial()[0].id);
        assertEquals(1.0 - 0.020, limelight.getMegaTag1PoseTimestamp(), EPS);
        assertTrue(limelight.targetInView());
        assertEquals(3.5, limelight.getTargetSize(), EPS);
        assertTrue(limelight.isCameraConnected());

        // Publish a different frame: nothing should change until the snapshot is cleared.
        pose.set(botpose(5.0, 6.0, 90.0, 10.0, fiducial(3, 0.2), fiducial(4, 0.3)), 2_000_000L);
        tv.set(0.0);
        ta.set(0.5);

        assertEquals(1.0, limelight.getMegaTag1_Pose3d().getX(), EPS);
        assertEquals(1, limelight.getTagCountInView(), EPS);
        assertEquals(7, limelight.getRawFiducial()[0].id);
        assertEquals(1.0 - 0.020, limelight.getMegaTag1PoseTimestamp(), EPS);
        assertTrue(limelight.targetInView());
        assertEquals(3.5, limelight.getTargetSize(), EPS);

        limelight.invalidate();

        assertEquals(5.0, limelight.getMegaTag1_Pose3d().getX(), EPS);
        assertEquals(2, limelight.getTagCountInView(), EPS);
        assertEquals(3, limelight.getRawFiducial()[0].id);
        assertEquals(4, limelight.getRawFiducial()[1].id);
        assertEquals(2.0 - 0.010, limelight.getMegaTag1PoseTimestamp(), EPS);
        assertFalse(limelight.targetInView());
        assertEquals(0.5, limelight.getTargetSize(), EPS);
    }

    @Test
    @DisplayName("All MT1 getters describe the same sample even if the topic changes mid-loop")
    void mt1GettersShareOneSample() {
        DoubleArrayPublisher pose = arrayPublisher("botpose_wpiblue");
        Limelight limelight = new Limelight(tableName);

        pose.set(botpose(1.0, 2.0, 30.0, 20.0, fiducial(7, 0.1)), 1_000_000L);
        Pose3d pose3d = limelight.getMegaTag1_Pose3d(); // first read snapshots the array

        pose.set(botpose(5.0, 6.0, 90.0, 10.0, fiducial(3, 0.2), fiducial(4, 0.3)), 2_000_000L);

        PoseEstimate estimate = limelight.getMegaTag1_PoseEstimate();
        assertEquals(limelight.getRawFiducial().length, (int) limelight.getTagCountInView());
        assertEquals(pose3d.toPose2d(), estimate.pose);
        assertEquals(1, estimate.tagCount);
    }

    @Test
    @DisplayName("MT2 cache is independent of the MT1 cache")
    void mt2CacheIndependent() {
        DoubleArrayPublisher mt1 = arrayPublisher("botpose_wpiblue");
        DoubleArrayPublisher mt2 = arrayPublisher("botpose_orb_wpiblue");
        Limelight limelight = new Limelight(tableName);

        mt1.set(botpose(1.0, 2.0, 30.0, 20.0, fiducial(7, 0.1)), 1_000_000L);
        mt2.set(botpose(1.1, 2.1, 30.0, 15.0, fiducial(7, 0.1)), 1_000_000L);

        assertEquals(1.1, limelight.getMegaTag2_Pose2d().getX(), EPS);
        assertEquals(1.0 - 0.015, limelight.getMegaTag2PoseTimestamp(), EPS);

        mt2.set(botpose(9.0, 9.0, 0.0, 5.0, fiducial(1, 0.1)), 3_000_000L);
        assertEquals(1.1, limelight.getMegaTag2_Pose2d().getX(), EPS);

        limelight.invalidate();
        assertEquals(9.0, limelight.getMegaTag2_Pose2d().getX(), EPS);
        assertEquals(3.0 - 0.005, limelight.getMegaTag2PoseTimestamp(), EPS);
        // MT1 was untouched by the MT2 republish.
        assertEquals(1.0, limelight.getMegaTag1_Pose3d().getX(), EPS);
    }

    @Test
    @DisplayName("Empty botpose arrays yield the documented defaults")
    void emptyArrayDefaults() {
        arrayPublisher("botpose_wpiblue").set(new double[0]);
        arrayPublisher("botpose_orb_wpiblue").set(new double[0]);
        Limelight limelight = new Limelight(tableName);

        assertEquals(Pose3d.kZero, limelight.getMegaTag1_Pose3d());
        assertEquals(Pose2d.kZero, limelight.getMegaTag2_Pose2d());
        assertEquals(0, limelight.getTagCountInView(), EPS);
        assertEquals(0, limelight.getRawFiducial().length);
        assertEquals(0, limelight.getMegaTag1PoseTimestamp(), EPS);
        assertEquals(0, limelight.getMegaTag2PoseTimestamp(), EPS);
        assertFalse(limelight.targetInView());
        assertEquals(0, limelight.getTargetSize(), EPS);
        assertFalse(limelight.isCameraConnected());
        assertEquals(new PoseEstimate(), limelight.getMegaTag1_PoseEstimate());
    }

    @Test
    @DisplayName("Detached camera returns defaults regardless of published data")
    void detachedCameraIgnoresNetworkTables() {
        arrayPublisher("botpose_wpiblue")
                .set(botpose(1.0, 2.0, 30.0, 20.0, fiducial(7, 0.1)), 1_000_000L);
        doublePublisher("tv").set(1.0);
        doublePublisher("ta").set(5.0);
        Limelight limelight = new Limelight(tableName, false);

        limelight.invalidate(); // must not throw
        assertEquals(Pose3d.kZero, limelight.getMegaTag1_Pose3d());
        assertEquals(0, limelight.getTagCountInView(), EPS);
        assertEquals(0, limelight.getRawFiducial().length);
        assertEquals(0, limelight.getMegaTag1PoseTimestamp(), EPS);
        assertFalse(limelight.targetInView());
        assertEquals(0, limelight.getTargetSize(), EPS);
        assertFalse(limelight.isCameraConnected());
    }

    @Test
    @DisplayName("parsePoseEstimate is a pure function matching the NT-backed helper")
    void parsePoseEstimateMatchesHelper() {
        assertEquals(new PoseEstimate(), LimelightHelpers.parsePoseEstimate(null, 0, false));
        assertEquals(
                new PoseEstimate(), LimelightHelpers.parsePoseEstimate(new double[0], 0, false));

        // Tag count says 2 but no fiducial values follow: fiducials are dropped, count kept.
        double[] mismatched = botpose(1.0, 2.0, 30.0, 20.0);
        mismatched[7] = 2;
        PoseEstimate parsed = LimelightHelpers.parsePoseEstimate(mismatched, 4_000_000L, true);
        assertEquals(2, parsed.tagCount);
        assertEquals(0, parsed.rawFiducials.length);
        assertTrue(parsed.isMegaTag2);
        assertEquals(4.0 - 0.020, parsed.timestampSeconds, EPS);

        double[] one = botpose(1.0, 2.0, 30.0, 20.0, fiducial(7, 0.1));
        PoseEstimate single = LimelightHelpers.parsePoseEstimate(one, 4_000_000L, false);
        assertEquals(1, single.rawFiducials.length);
        assertEquals(7, single.rawFiducials[0].id);
        assertEquals(0.1, single.rawFiducials[0].ambiguity, EPS);
        assertEquals(1.0, single.pose.getX(), EPS);

        arrayPublisher("botpose_wpiblue").set(one, 4_000_000L);
        assertEquals(single, LimelightHelpers.getBotPoseEstimate_wpiBlue(tableName));
    }

    @Test
    @DisplayName("setRobotOrientation still writes without flushing")
    void setRobotOrientationWrites() {
        Limelight limelight = new Limelight(tableName);
        try (DoubleArraySubscriber sub =
                table.getDoubleArrayTopic("robot_orientation_set").subscribe(new double[0])) {
            limelight.setRobotOrientation(90.0);
            assertArrayEquals(new double[] {90, 0, 0, 0, 0, 0}, sub.get(), EPS);
        }
    }
}
