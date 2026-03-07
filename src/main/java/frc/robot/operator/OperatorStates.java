package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.intakeExtension.IntakeExtensionStates;
import frc.robot.turretRotationalPivot.RotationalPivotStates;
import frc.spectrumLib.Telemetry;

/** This class should have any command calls that directly call the Operator */
public class OperatorStates {
    private static Operator operator = Robot.getOperator();

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        operator.setDefaultCommand(
                log(
                        rumble(0, 1000000000)
                                .withName(
                                        "Operator.noRumble"))); // .repeatedly().withName("Operator.default"));
    }

    /** Set the states for the operator controller */
    public static void setStates() {
        operator.rightTriggerOnly.whileTrue(
                Robot.getTurret().joystickMove(() -> operator.getClimberTriggerAxis(), () -> 1));
        operator.leftTriggerOnly.whileTrue(
                Robot.getTurret().joystickMove(() -> -operator.getClimberTriggerAxis(), () -> -1));

        operator.YButton.onTrue(
                new InstantCommand(() -> IntakeExtensionStates.operatorResetIntakeExtension()));
        operator.XButton.onTrue(
                new InstantCommand(() -> RotationalPivotStates.operatorResetTurretPosition()));

        operator.dpadDown.onTrue(
                log(new InstantCommand(ShotCalculator::decreaseFlywheelSpeedOffset)));
        operator.dpadUp.onTrue(
                log(new InstantCommand(ShotCalculator::increaseFlywheelSpeedOffset)));
        operator.dpadRight.onTrue(
                log(new InstantCommand(ShotCalculator::decreaseTurretAngleOffsetDegrees)));
        operator.dpadLeft.onTrue(
                log(new InstantCommand(ShotCalculator::increaseTurretAngleOffsetDegrees)));
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
