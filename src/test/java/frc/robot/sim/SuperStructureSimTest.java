package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.configs.OM2026;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.dyeRotor.DyeRotor;
import frc.robot.subsystems.fuelIntake.FuelIntake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeExtension.IntakeExtension;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherTower;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
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
                    Robot.Config config = new OM2026();

                    try (Swerve swerve = new Swerve(config.swerve)) {
                        FuelIntake fuelIntake = new FuelIntake(config.fuelIntake);
                        IntakeExtension intakeExtension =
                                new IntakeExtension(config.intakeExtension);
                        DyeRotor dyeRotor = new DyeRotor(config.dyeRotor);
                        Launcher launcher = new Launcher(config.launcher);
                        LauncherTower launcherTower = new LauncherTower(config.launcherTower);
                        Turret turret = new Turret(config.turret);
                        Hood hood = new Hood(config.hood);

                        SuperStructure superStructure =
                                new SuperStructure(
                                        swerve,
                                        fuelIntake,
                                        intakeExtension,
                                        dyeRotor,
                                        launcher,
                                        launcherTower,
                                        turret,
                                        hood);

                        CommandScheduler.getInstance().registerSubsystem(superStructure);

                        enableTeleopSim();

                        superStructure
                                .setStateCommand(SuperStructure.WantedSuperState.INTAKE_FUEL)
                                .schedule();

                        stepSim(0.020, 0.50);

                        assertEquals(
                                SuperStructure.CurrentSuperState.INTAKE_FUEL,
                                superStructure.getCurrentSuperState());
                    }
                });
    }
}
