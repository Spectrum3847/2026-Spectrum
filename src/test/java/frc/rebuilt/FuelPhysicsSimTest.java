package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class FuelPhysicsSimTest {

    @Test
    @DisplayName("Test PhysicsConfig default values and copy")
    void testPhysicsConfig() {
        FuelPhysicsSim.PhysicsConfig config = new FuelPhysicsSim.PhysicsConfig();
        assertTrue(config.dragEnabled);
        assertTrue(config.magnusEnabled);
        assertTrue(config.frictionEnabled);
        assertEquals(4, config.solverIterations);

        FuelPhysicsSim.PhysicsConfig copy = config.copy();
        assertEquals(config.dragEnabled, copy.dragEnabled);
        assertEquals(config.solverIterations, copy.solverIterations);
    }

    @Test
    @DisplayName("Test SimBall creation and spin RPM calculation")
    void testSimBall() {
        Translation3d pos = new Translation3d(1.0, 2.0, 0.1);
        Translation3d vel = new Translation3d(0.5, 0.0, 0.0);
        // Spin 100 rad/s about Y axis
        Translation3d omega = new Translation3d(0.0, 100.0, 0.0);

        FuelPhysicsSim.SimBall ball = new FuelPhysicsSim.SimBall(pos, vel, omega);

        // RPM = rad/s * 60 / (2 * PI) = 100 * 60 / (2 * PI) approx 954.9296
        double expectedRPM = 100.0 * 60.0 / (2.0 * Math.PI);
        assertEquals(expectedRPM, ball.getSpinRPM(), 1e-4);
    }

    @Test
    @DisplayName("Test ScoringTarget scoring detection")
    void testScoringTarget() {
        Translation2d hubCenter = new Translation2d(4.5, 4.0);
        Translation3d exit = new Translation3d(1.0, 0.0, 0.0);

        FuelPhysicsSim.ScoringTarget target = new FuelPhysicsSim.ScoringTarget(hubCenter, exit, 1);
        assertEquals(0, target.getScore());

        // Ball falling vertically into hub opening (prevZ = 2.0, currZ = 1.7, opening height =
        // 1.829)
        FuelPhysicsSim.SimBall scoringBall =
                new FuelPhysicsSim.SimBall(new Translation3d(4.5, 4.0, 1.7));
        scoringBall.prevPos = new Translation3d(4.5, 4.0, 2.0);

        assertTrue(target.didScore(scoringBall));

        // Ball rising through opening (prevZ = 1.7, currZ = 2.0) should NOT score
        FuelPhysicsSim.SimBall risingBall =
                new FuelPhysicsSim.SimBall(new Translation3d(4.5, 4.0, 2.0));
        risingBall.prevPos = new Translation3d(4.5, 4.0, 1.7);

        assertFalse(target.didScore(risingBall));
    }

    @Test
    @DisplayName("Test FuelPhysicsSim instantiation")
    void testFuelPhysicsSimInstantiation() {
        FuelPhysicsSim sim = new FuelPhysicsSim("TestSim/Fuel");
        assertNotNull(sim);
    }
}
