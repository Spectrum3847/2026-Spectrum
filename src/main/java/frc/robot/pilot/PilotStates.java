package frc.robot.pilot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShotCalculator;
import frc.robot.Robot;
import frc.robot.RobotSim;
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

    /** Set the states for the pilot controller */
    public static void setStates() {
        // Reset vision pose with Left Bumper and Select
        pilot.visionPoseReset_LB_Select.onTrue(VisionStates.resetVisionPose());

        // Simulation Only: Map RT and LT to intake and launch fuel for testing
        pilot.RT.and(Utils::isSimulation).whileTrue(RobotSim.mapleSimIntakeFuel());
        pilot.LT.and(Utils::isSimulation).whileTrue(RobotSim.mapleSimLaunchFuel());

        pilot.dpadDown.onTrue(log(new InstantCommand(ShotCalculator::decreaseFlywheelSpeedOffset)));
        pilot.dpadUp.onTrue(log(new InstantCommand(ShotCalculator::increaseFlywheelSpeedOffset)));
        pilot.dpadRight.onTrue(log(new InstantCommand(ShotCalculator::decreaseTurretAngleOffsetDegrees)));
        pilot.dpadLeft.onTrue(log(new InstantCommand(ShotCalculator::increaseTurretAngleOffsetDegrees)));

        // Slow mode when driver is intaking or launching fuel
        pilot.RT.whileTrue(slowMode());
        pilot.LT.whileTrue(slowMode());

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
