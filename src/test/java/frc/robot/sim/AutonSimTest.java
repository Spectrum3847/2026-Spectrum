package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.auton.Auton;
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

/** Automated simulation tests for the autonomous routine chooser and execution. */
public class AutonSimTest extends SimTestBase {

    @Test
    @DisplayName("Auton initializes the SendableChooser and default option without exceptions")
    void testAutonChooserInitialization() {
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

                        Auton auton = new Auton(superStructure);
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

                        Auton auton = new Auton(superStructure);
                        CommandScheduler.getInstance().registerSubsystem(superStructure);

                        enableAutonomousSim();
                        auton.init();

                        stepSim(0.020, 0.50);
                    }
                });
    }
}
