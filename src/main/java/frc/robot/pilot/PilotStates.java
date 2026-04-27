package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.robot.State;
import frc.robot.fuelIntake.FuelIntakeStates;
import frc.robot.intakeExtension.IntakeExtensionStates;
import frc.robot.vision.VisionStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.util.Util;

/** This class should have any command calls that directly call the Pilot */
public class PilotStates {
    private static Pilot pilot = Robot.getPilot();

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        pilot.setDefaultCommand(log(rumble(0, 1).withName("Pilot.noRumble")));
    }

    private static Trigger reorientButton =
            pilot.upReorient.or(pilot.downReorient, pilot.leftReorient, pilot.rightReorient);

    private static final Trigger launching =
            new Trigger(() -> RobotStates.getAppliedState() == State.LAUNCH_WITH_SQUEEZE);

    /** Set the states for the pilot controller */
    public static void setStates() {
        // Reset vision pose with Left Bumper and Select
        pilot.visionPoseReset_LB_Select.onTrue(VisionStates.resetVisionPose());

        pilot.BButton.whileTrue(IntakeExtensionStates.slowIntakeCloseCommand());
        pilot.YButton.whileTrue(Robot.getAuton().launch());

        pilot.rightTriggerOnly.and(pilot.fn).whileTrue(FuelIntakeStates.ejectCommand());

        pilot.coastA.onTrue(IntakeExtensionStates.coastMode());
        pilot.brakeB.onTrue(IntakeExtensionStates.brakeMode());

        pilot.dpadDown.onTrue(log(ShotCalculator.decreaseHoodAngleOffset()));
        pilot.dpadUp.onTrue(log(ShotCalculator.increaseHoodAngleOffset()));
        pilot.dpadRight.onTrue(log(ShotCalculator.decreaseDriveAngleOffset()));
        pilot.dpadLeft.onTrue(log(ShotCalculator.increaseDriveAngleOffset()));

        // Slow mode when driver is launching fuel
        launching.whileTrue(slowMode());

        // Rumble whenever we reorient
        reorientButton.onTrue(log(rumble(1, 0.5).withName("Pilot.reorientRumble")));
    }

    public static final Trigger buttonAPress = pilot.AButton;

    /** Command that can be used to rumble the pilot controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return pilot.rumbleCommand(intensity, durationSeconds)
                .withName("Pilot.rumble")
                .onlyIf(Util.autoMode.not());
    }

    /**
     * Command that can be used to turn on the slow mode. Slow mode modifies the fwd, left, and CCW
     * methods, we don't want these to require the pilot subsystem arm
     */
    public static Command slowMode() {
        return Commands.startEnd(
                        () -> pilot.getSlowMode().setState(true),
                        () -> pilot.getSlowMode().setState(false))
                .withName("Pilot.setSlowMode");
    }

    /**
     * Command that can be used to turn on the turbo mode. Turbo mode modifies CCW methods, we don't
     * want these to require the pilot subsystem
     */
    public static Command turboMode() {
        return Commands.startEnd(
                        () -> pilot.getTurboMode().setState(true),
                        () -> pilot.getTurboMode().setState(false))
                .withName("Pilot.setTurboMode");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
