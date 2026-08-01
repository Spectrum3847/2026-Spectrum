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

    /** Verifies PhysicsConfig defaults and that copy() preserves every field. */
    @Test
    @DisplayName("Test PhysicsConfig default values and copy")
    void testPhysicsConfig() {
        FuelPhysicsSim.PhysicsConfig config = new FuelPhysicsSim.PhysicsConfig();
        assertTrue(config.dragEnabled);
        assertTrue(config.magnusEnabled);
        assertTrue(config.frictionEnabled);
        assertEquals(4, config.solverIterations);

        // Assign distinct non-default values so copy() must carry every field to pass.
        config.dragEnabled = false;
        config.magnusEnabled = false;
        config.frictionEnabled = false;
        config.spinTransferEnabled = false;
        config.sleepingEnabled = false;
        config.ccdEnabled = false;
        config.velocityDependentCOR = false;
        config.spinDecayEnabled = false;
        config.solverIterations = 8;
        config.subticks = 3;
        config.spinDecayTau = 1.5;
        config.sleepVelocityThreshold = 0.05;
        config.sleepFrameThreshold = 4;
        config.ccdSpeedThreshold = 7.5;
        config.baumgarteBeta = 0.8;
        config.baumgarteSlop = 0.01;
        config.deterministic = true;
        config.deterministicSeed = 1234L;
        config.conservationMonitor = true;

        FuelPhysicsSim.PhysicsConfig copy = config.copy();
        assertEquals(config.dragEnabled, copy.dragEnabled);
        assertEquals(config.magnusEnabled, copy.magnusEnabled);
        assertEquals(config.frictionEnabled, copy.frictionEnabled);
        assertEquals(config.spinTransferEnabled, copy.spinTransferEnabled);
        assertEquals(config.sleepingEnabled, copy.sleepingEnabled);
        assertEquals(config.ccdEnabled, copy.ccdEnabled);
        assertEquals(config.velocityDependentCOR, copy.velocityDependentCOR);
        assertEquals(config.spinDecayEnabled, copy.spinDecayEnabled);
        assertEquals(config.solverIterations, copy.solverIterations);
        assertEquals(config.subticks, copy.subticks);
        assertEquals(config.spinDecayTau, copy.spinDecayTau, 1e-9);
        assertEquals(config.sleepVelocityThreshold, copy.sleepVelocityThreshold, 1e-9);
        assertEquals(config.sleepFrameThreshold, copy.sleepFrameThreshold);
        assertEquals(config.ccdSpeedThreshold, copy.ccdSpeedThreshold, 1e-9);
        assertEquals(config.baumgarteBeta, copy.baumgarteBeta, 1e-9);
        assertEquals(config.baumgarteSlop, copy.baumgarteSlop, 1e-9);
        assertEquals(config.deterministic, copy.deterministic);
        assertEquals(config.deterministicSeed, copy.deterministicSeed);
        assertEquals(config.conservationMonitor, copy.conservationMonitor);
    }

    /** Verifies SimBall spin RPM conversion from angular velocity. */
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

    /** Verifies ScoringTarget detects top-down entry and ignores rising balls. */
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

    /** Verifies a FuelPhysicsSim can be constructed with a table key. */
    @Test
    @DisplayName("Test FuelPhysicsSim instantiation")
    void testFuelPhysicsSimInstantiation() {
        FuelPhysicsSim sim = new FuelPhysicsSim("TestSim/Fuel");
        assertNotNull(sim);
    }
}
