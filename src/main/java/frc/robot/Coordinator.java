package frc.robot;

import frc.robot.fuelIntake.FuelIntakeStates;
import frc.robot.hood.HoodStates;
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
                HoodStates.home();
            }
            case INTAKE_FUEL -> {
                FuelIntakeStates.intakeFuel();
                IndexerTowerStates.neutral();
                IndexerBedStates.slowIndex();
                IntakeExtensionStates.fullExtend();
                LauncherStates.idlePrep();
                HoodStates.neutral();
            }
            case LAUNCHER_TRACK -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtendConditional();
                LauncherStates.aimAtTarget();
                HoodStates.aimAtTarget();
            }
            case LAUNCER_TRACK_WITH_LAUNCH -> {
                FuelIntakeStates.slowIntakeFuel();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.slowIntakeClose();
                LauncherStates.aimAtTarget();
                HoodStates.aimAtTarget();
            }
            case AUTON_LAUNCHER_TRACK -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtendConditional();
                LauncherStates.autonAimAtTarget();
                HoodStates.aimAtTarget();
            }
            case AUTON_LAUNCHER_TRACK_WITH_LAUNCH -> {
                FuelIntakeStates.slowIntakeFuel();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.slowIntakeClose();
                LauncherStates.autonAimAtTarget();
                HoodStates.aimAtTarget();
            }
            case UNJAM -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.unjam();
                IndexerBedStates.unjam();
                IntakeExtensionStates.fullExtendConditional();
                LauncherStates.neutral();
                HoodStates.neutral();
            }
            case FORCE_HOME -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullRetract();
                LauncherStates.neutral();
                HoodStates.neutral();
            }
            case CUSTOM_SPEED_TURRET_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.fullExtendConditional();
                LauncherStates.customLaunchSpeed();
                HoodStates.aimAtTarget();
            }
            case TEST_INFINITE_LAUNCH -> {
                FuelIntakeStates.slowIntakeFuel();
                IndexerTowerStates.slowIndex();
                IndexerBedStates.slowIndex();
                LauncherStates.slowLaunch();
                HoodStates.neutral();
            }
            case TEST_IDLE -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                LauncherStates.neutral();
                HoodStates.neutral();
            }
            case COAST -> {
                IntakeExtensionStates.coastMode();
                HoodStates.coastMode();
            }
            case BRAKE -> {
                IntakeExtensionStates.brakeMode();
                HoodStates.ensureBrakeMode();
            }
            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
