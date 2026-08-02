package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SuperStructure;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Automated simulation tests for the SuperStructure coordination subsystem. */
public class SuperStructureSimTest extends SimTestBase {

    @Test
    @DisplayName(
            "SuperStructure initializes all subsystems and steps simulation without exceptions")
    void testSuperStructureSimPeriodic() {
        assertDoesNotThrow(
                () -> {
                    RobotStack stack = createRobotStack();
                    try (stack.swerve) {
                        CommandScheduler.getInstance().registerSubsystem(stack.superStructure);

                        enableTeleopSim();

                        CommandScheduler.getInstance()
                                .schedule(
                                        stack.superStructure.setStateCommand(
                                                SuperStructure.WantedSuperState.INTAKE_FUEL));

                        stepSim(0.020, 0.50);

                        assertEquals(
                                SuperStructure.CurrentSuperState.INTAKE_FUEL,
                                stack.superStructure.getCurrentSuperState());
                    }
                });
    }
}
