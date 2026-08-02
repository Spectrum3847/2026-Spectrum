package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotSim;
import frc.robot.subsystems.swerve.Swerve;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Automated simulation tests for the RobotSim visualizer and fuel physics simulation. */
public class RobotSimTest extends SimTestBase {

    @Test
    @DisplayName("RobotSim initializes visualizer and fuel physics and updates cleanly")
    void testRobotSimCreationAndPeriodic() {
        assertDoesNotThrow(
                () -> {
                    RobotStack stack = createRobotStack();
                    try (Swerve swerve = stack.swerve) {
                        RobotSim robotSim = new RobotSim(stack.superStructure);
                        CommandScheduler.getInstance().registerSubsystem(stack.superStructure);

                        assertNotNull(robotSim.getBallSim());
                        assertNotNull(RobotSim.leftView);
                        assertNotNull(RobotSim.topView);

                        enableTeleopSim();
                        stepSim(0.020, 0.50);

                        robotSim.updateArticulatedMechanisms();
                        robotSim.getBallSim().tick();
                    }
                });
    }
}
