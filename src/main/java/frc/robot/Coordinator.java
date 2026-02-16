package frc.robot;

import frc.robot.climb.ClimbStates;
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
                ClimbStates.stop();
                IndexerBackwardStates.neutral();
                IndexerForwardStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.neutral();
                TowerIndexerStates.neutral();
                TurretStates.neutral();
            }
            case CLIMB_FORWARD -> {
                ClimbStates.climbForward();
            }
            case CLIMB_BACKWARD -> {
                ClimbStates.climbBackward();
            }
            default -> {
                // Handle other states or throw an error
            }
        }
    }
}
