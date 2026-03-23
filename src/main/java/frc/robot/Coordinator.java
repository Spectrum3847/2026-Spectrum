package frc.robot;

import frc.robot.fuelIntake.FuelIntakeStates;
import frc.robot.indexerBed.IndexerBedStates;
import frc.robot.indexerTower.IndexerTowerStates;
import frc.robot.intakeExtension.IntakeExtensionStates;
import frc.robot.launcher.LauncherStates;

public class Coordinator {

    public void update() {}

    public void applyRobotState(State state) {
        switch (state) {
            case IDLE -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.neutral();
                LauncherStates.idlePrep();
            }
            case INTAKE_FUEL -> {
                FuelIntakeStates.intakeFuel();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.idlePrep();
            }
            case LAUNCHER_TRACK -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtendConditional();
                LauncherStates.aimAtTarget();
            }
            case LAUNCHER_TRACK_WITH_LAUNCH -> {
                FuelIntakeStates.slowIntakeFuel();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.slowIntakeClose();
                LauncherStates.aimAtTarget();
            }
            case UNJAM -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.unjam();
                IndexerBedStates.unjam();
                IntakeExtensionStates.fullExtendConditional();
                LauncherStates.neutral();
            }
            case FORCE_HOME -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullRetract();
                LauncherStates.neutral();
            }
            case CUSTOM_SPEED_TURRET_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.fullExtendConditional();
                LauncherStates.customLaunchSpeed();
            }
            case TEST_INFINITE_LAUNCH -> {
                FuelIntakeStates.slowIntakeFuel();
                IndexerTowerStates.slowIndex();
                IndexerBedStates.slowIndex();
                LauncherStates.slowLaunch();
            }
            case TEST_IDLE -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                LauncherStates.neutral();
            }
            case COAST -> {
                IntakeExtensionStates.coastMode();
            }
            case BRAKE -> {
                IntakeExtensionStates.brakeMode();
            }
            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
