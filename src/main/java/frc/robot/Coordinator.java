package frc.robot;

import frc.robot.fuelIntake.FuelIntakeStates;
import frc.robot.indexer.IndexerStates;
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
                IndexerStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.neutral();
                RotationalPivotStates.neutral();
            }
            case INTAKE_FUEL -> {
                FuelIntakeStates.intakeFuel();
                IndexerStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.neutral();
                RotationalPivotStates.neutral();
            }
            case TURRET_TRACK_WITH_SPINUP -> {
                FuelIntakeStates.stop();
                IndexerStates.neutral();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtHub();
                RotationalPivotStates.aimAtHub();
            }
            case TURRET_TRACK_WITH_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerStates.indexMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtHub();
                RotationalPivotStates.aimAtHub();
            }
            case TURRET_FEED_WITH_SPINUP -> {
                FuelIntakeStates.stop();
                IndexerStates.indexMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtHub();
                RotationalPivotStates.aimAtHub();
            }
            case TURRET_FEED_WITH_LAUNCH -> {
                FuelIntakeStates.stop();
                IndexerStates.indexMax();
                IntakeExtensionStates.fullExtend();
                LauncherStates.aimAtHub();
                RotationalPivotStates.aimAtHub();
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
