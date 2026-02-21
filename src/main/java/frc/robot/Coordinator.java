package frc.robot;

import frc.robot.fuelIntake.FuelIntakeStates;
import frc.robot.indexerBed.IndexerBedStates;
import frc.robot.indexerTower.IndexerTowerStates;
import frc.robot.intakeExtension.IntakeExtensionStates;
import frc.robot.launcher.LauncherStates;
import frc.robot.turretRotationalPivot.RotationalPivotStates;

public class Coordinator {

    public void update() {
    }

    public void applyRobotState(State state) {
        switch (state) {
            case IDLE -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.neutral();
                LauncherStates.neutral();
                RotationalPivotStates.neutral();
            }
            case INTAKE_FUEL -> {
                FuelIntakeStates.intakeFuel();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.neutral();
                RotationalPivotStates.neutral();
            }
            case TURRET_TRACK -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtHub();
            }
            case TURRET_TRACK_WITH_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtHub();
            }
            case TURRET_WITHOUT_TRACK -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtPresetPosition();
            }
            case TURRET_WITHOUT_TRACK_WITH_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtPresetPosition();
            }
            case TURRET_FEED_WITH_AIMING -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtHub();
            }
            case TURRET_FEED_WITH_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtHub();
            }
            case TURRET_FEED_WITHOUT_AIMING -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.neutral();
                IndexerBedStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtPresetPosition();
            }
            case TURRET_FEED_WITHOUT_AIMING_WITH_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerTowerStates.indexMax();
                IndexerBedStates.indexMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtTarget();
                RotationalPivotStates.aimAtPresetPosition();
            }
            case L1_CLIMB_PREP -> {

            }
            case L1_CLIMB_EXECUTE -> {

            }
            case L3_CLIMB_PREP -> {

            }
            case L3_CLIMB_EXECUTE -> {

            }
            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
