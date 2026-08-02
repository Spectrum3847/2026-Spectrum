package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.turret.Turret;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Automated simulation tests for individual mechanism subsystems. */
public class MechanismSimTest extends SimTestBase {

    @Test
    @DisplayName("Hood subsystem initializes and runs simulation steps without exceptions")
    void testHoodSimPeriodic() {
        assertDoesNotThrow(
                () -> {
                    Hood hood = new Hood(new Hood.HoodConfig());
                    CommandScheduler.getInstance().registerSubsystem(hood);

                    enableTeleopSim();
                    stepSim(0.020, 0.50);

                    assertNotNull(hood.getSim());
                });
    }

    @Test
    @DisplayName("Turret subsystem initializes and runs simulation steps without exceptions")
    void testTurretSimPeriodic() {
        assertDoesNotThrow(
                () -> {
                    Turret turret = new Turret(new Turret.TurretConfig());
                    CommandScheduler.getInstance().registerSubsystem(turret);

                    enableTeleopSim();
                    stepSim(0.020, 0.50);

                    assertNotNull(turret.getSim());
                });
    }

    @Test
    @DisplayName(
            "IntakeExtension subsystem initializes and runs simulation steps without exceptions")
    void testIntakeExtensionSimPeriodic() {
        assertDoesNotThrow(
                () -> {
                    IntakeExtension.Left.LeftConfig leftConfig =
                            new IntakeExtension.Left.LeftConfig();
                    IntakeExtension.Right.RightConfig rightConfig =
                            new IntakeExtension.Right.RightConfig(leftConfig);
                    IntakeExtension intakeExtension =
                            new IntakeExtension(
                                    new IntakeExtension.IntakeExtensionConfig(
                                            leftConfig, rightConfig));
                    CommandScheduler.getInstance().registerSubsystem(intakeExtension);

                    enableTeleopSim();
                    stepSim(0.020, 0.50);

                    assertNotNull(intakeExtension.getSim());
                });
    }
}
