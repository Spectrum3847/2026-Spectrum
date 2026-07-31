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

    @Test
    @DisplayName("Test PhysicsConfig copy produces an independent instance")
    void testPhysicsConfigCopyIsIndependent() {
        FuelPhysicsSim.PhysicsConfig config = new FuelPhysicsSim.PhysicsConfig();
        FuelPhysicsSim.PhysicsConfig copy = config.copy();

        // Mutating the copy must not affect the original
        copy.dragEnabled = false;
        copy.solverIterations = 99;
        copy.spinDecayTau = 123.0;

        assertTrue(config.dragEnabled);
        assertEquals(4, config.solverIterations);
        assertEquals(3.0, config.spinDecayTau, 1e-9);

        // And the copy itself reflects the mutated values
        assertFalse(copy.dragEnabled);
        assertEquals(99, copy.solverIterations);
        assertEquals(123.0, copy.spinDecayTau, 1e-9);
    }

    @Test
    @DisplayName("Test SimBall default spin is zero when omega is not provided")
    void testSimBallDefaultOmegaIsZero() {
        Translation3d pos = new Translation3d(0.0, 0.0, 0.0);
        Translation3d vel = new Translation3d(1.0, 0.0, 0.0);

        FuelPhysicsSim.SimBall ballWithVel = new FuelPhysicsSim.SimBall(pos, vel);
        assertEquals(0.0, ballWithVel.getSpinRPM(), 1e-9);

        FuelPhysicsSim.SimBall ballPosOnly = new FuelPhysicsSim.SimBall(pos);
        assertEquals(0.0, ballPosOnly.getSpinRPM(), 1e-9);
    }

    @Test
    @DisplayName("Test ScoringTarget does not score when ball is outside the entry radius")
    void testScoringTargetOutsideRadiusDoesNotScore() {
        Translation2d hubCenter = new Translation2d(4.5, 4.0);
        Translation3d exit = new Translation3d(1.0, 0.0, 0.0);
        FuelPhysicsSim.ScoringTarget target = new FuelPhysicsSim.ScoringTarget(hubCenter, exit, 1);

        // Falls through the correct height band but is 1m away from the hub center,
        // well outside the ~0.53m entry radius.
        FuelPhysicsSim.SimBall farBall =
                new FuelPhysicsSim.SimBall(new Translation3d(5.5, 4.0, 1.7));
        farBall.prevPos = new Translation3d(5.5, 4.0, 2.0);

        assertFalse(target.didScore(farBall));
    }

    @Test
    @DisplayName("Test ScoringTarget does not score when ball never crosses the entry height")
    void testScoringTargetNoHeightTransitionDoesNotScore() {
        Translation2d hubCenter = new Translation2d(4.5, 4.0);
        Translation3d exit = new Translation3d(1.0, 0.0, 0.0);
        FuelPhysicsSim.ScoringTarget target = new FuelPhysicsSim.ScoringTarget(hubCenter, exit, 1);

        // Ball stays entirely below the entry height on both steps (already inside/below).
        FuelPhysicsSim.SimBall lowBall =
                new FuelPhysicsSim.SimBall(new Translation3d(4.5, 4.0, 0.9));
        lowBall.prevPos = new Translation3d(4.5, 4.0, 1.0);
        assertFalse(target.didScore(lowBall));

        // Ball stays entirely above the entry height on both steps (never enters).
        FuelPhysicsSim.SimBall highBall =
                new FuelPhysicsSim.SimBall(new Translation3d(4.5, 4.0, 3.0));
        highBall.prevPos = new Translation3d(4.5, 4.0, 2.5);
        assertFalse(target.didScore(highBall));
    }

    @Test
    @DisplayName("Test ScoringTarget resetScore clears accumulated score")
    void testScoringTargetResetScore() {
        Translation2d hubCenter = new Translation2d(4.5, 4.0);
        Translation3d exit = new Translation3d(1.0, 0.0, 0.0);
        FuelPhysicsSim.ScoringTarget target = new FuelPhysicsSim.ScoringTarget(hubCenter, exit, 1);

        target.score = 5;
        assertEquals(5, target.getScore());

        target.resetScore();
        assertEquals(0, target.getScore());
    }
}
