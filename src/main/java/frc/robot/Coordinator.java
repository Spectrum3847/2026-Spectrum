package frc.robot;

import frc.robot.fuelIntake.FuelIntakeStates;
import frc.robot.indexerBackward.IndexerBackwardStates;
import frc.robot.indexerForward.IndexerForwardStates;
import frc.robot.intakeExtension.IntakeExtensionStates;
import frc.robot.launcher.LauncherStates;
import frc.robot.towerIndexer.TowerIndexerStates;
import frc.robot.turret.TurretStates;

public class Coordinator {

    public void update() {}

    public void applyRobotState(State state) {
        switch (state) {
            case IDLE -> {
                FuelIntakeStates.stopMotor();
                IndexerBackwardStates.neutral();
                IndexerForwardStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.neutral();
                TowerIndexerStates.neutral();
                TurretStates.neutral();
            }
            case INTAKING_WITH_INDEXER -> {
                FuelIntakeStates.intakeFuel();
                IndexerBackwardStates.spinBack();
                IndexerForwardStates.spinMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.neutral();
                TowerIndexerStates.spinMax();
                TurretStates.neutral();
            }
            case LAUNCHING_WITH_INDEXER -> {
                FuelIntakeStates.intakeFuel();
                IndexerBackwardStates.spinBack();
                IndexerForwardStates.spinMax();
                // IntakeExtensionStates.agitateFuel();
                // LauncherStates.prepVelocity();
                TowerIndexerStates.spinMax();
                TurretStates.neutral();
            }
            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
