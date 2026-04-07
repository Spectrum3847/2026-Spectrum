package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.indexerBed.IndexerBedStates;
import frc.robot.indexerTower.IndexerTowerStates;
import frc.robot.intakeExtension.IntakeExtensionStates;
import frc.spectrumLib.Telemetry;

/** This class should have any command calls that directly call the Operator */
public class OperatorStates {
    private static Operator operator = Robot.getOperator();

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        operator.setDefaultCommand(
                log(
                        rumble(0, 1)
                                .withName(
                                        "Operator.noRumble"))); // .repeatedly().withName("Operator.default"));
    }

    /** Set the states for the operator controller */
    public static void setStates() {
        operator.resetIntakeExtensionPos.onTrue(
                IntakeExtensionStates.operatorResetIntakeExtension(), rumble(1, 0.5));

        operator.BButton.whileTrue(IndexerTowerStates.unjamCommand());
        operator.XButton.whileTrue(IndexerBedStates.unjamCommand());

        operator.rightBumperOnly.whileTrue(IntakeExtensionStates.fullRetractCommand());
        operator.rightTriggerOnly.whileTrue(IntakeExtensionStates.slowIntakeCloseCommand());

        operator.testA.whileTrue(IntakeExtensionStates.fullExtendCommand());
        operator.testB.whileTrue(IntakeExtensionStates.fullRetractCommand());

        operator.coastA.onTrue(IntakeExtensionStates.coastMode());
        operator.brakeB.onTrue(IntakeExtensionStates.brakeMode());

        operator.dpadDown.onTrue(log(ShotCalculator.decreaseHoodAngleOffset()));
        operator.dpadUp.onTrue(log(ShotCalculator.increaseHoodAngleOffset()));
        operator.dpadRight.onTrue(log(ShotCalculator.decreaseDriveAngleOffset()));
        operator.dpadLeft.onTrue(log(ShotCalculator.increaseDriveAngleOffset()));
    }

    /** Command that can be used to rumble the operator controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return operator.rumbleCommand(intensity, durationSeconds).withName("Operator.rumble");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
