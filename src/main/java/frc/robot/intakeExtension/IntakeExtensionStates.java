package frc.robot.intakeExtension;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.intakeExtension.IntakeExtension.IntakeExtensionConfig;
import frc.spectrumLib.Telemetry;

public class IntakeExtensionStates {
    private static IntakeExtension intakeExtension = Robot.getIntakeExtension();
    private static IntakeExtensionConfig config = Robot.getConfig().intakeExtension;

    public static void setupDefaultCommand() {
        intakeExtension.setDefaultCommand(
                log(intakeExtension.runHoldIntakeExtension().withName("IntakeExtension.default")));
    }

    // -------------------- State Commands --------------------

    public static void fullExtend() {
        scheduleIfNotRunning(intakeExtension.move(() -> config.getMaxRotations()));
    }

    public static void fullRetract() {
        scheduleIfNotRunning(intakeExtension.move(() -> config.getMinRotations()));
    }

    public static void neutral() {
        scheduleIfNotRunning(intakeExtension.runVoltage(() -> 0));
    }

    // --------------------------------------------------------

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the intake extension subsystem only if it's not already the running
     * command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(intakeExtension);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
