package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.Auton;
import frc.robot.subsystems.swerve.Swerve;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Automated simulation tests for the autonomous routine chooser and execution. */
public class AutonSimTest extends SimTestBase {

    @Test
    @DisplayName("Auton initializes the SendableChooser and default option without exceptions")
    void testAutonChooserInitialization() {
        assertDoesNotThrow(
                () -> {
                    RobotStack stack = createRobotStack();
                    try (Swerve swerve = stack.swerve) {
                        Auton auton = new Auton(stack.superStructure);
                        assertNotNull(auton.getPathChooser());
                        assertNotNull(auton.getPathChooser().getSelected());
                    }
                });
    }

    @Test
    @DisplayName("Auton init schedules the selected auto and steps simulation time cleanly")
    void testAutonInitAndStep() {
        assertDoesNotThrow(
                () -> {
                    RobotStack stack = createRobotStack();
                    try (Swerve swerve = stack.swerve) {
                        Auton auton = new Auton(stack.superStructure);
                        CommandScheduler.getInstance().registerSubsystem(stack.superStructure);

                        enableAutonomousSim();
                        auton.init();

                        stepSim(0.020, 0.50);
                    }
                });
    }
}
