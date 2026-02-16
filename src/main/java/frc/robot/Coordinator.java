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
                FuelIntakeStates.stop();
                IndexerBackwardStates.neutral();
                IndexerForwardStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.neutral();
                TowerIndexerStates.neutral();
                TurretStates.neutral();
            }
            case CLIMB_FORWARD -> {
                FuelIntakeStates.climbForward();
            }
            case CLIMB_BACKWARD -> {
                FuelIntakeStates.climbBackward();
            }
            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
