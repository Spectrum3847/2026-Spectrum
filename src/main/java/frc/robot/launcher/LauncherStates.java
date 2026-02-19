package frc.robot.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class LauncherStates {
    private static Launcher launcher = Robot.getLauncher();

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(
                launcher.stopMotor().ignoringDisable(true).withName("Launcher.default"));
        launcher.setDefaultCommand(
                launcher.stopMotor().ignoringDisable(true).withName("Launcher.default"));
    }

    public static Trigger aimingAtTarget() {
        return launcher.aimingAtTarget();
    }

    // -------------------- State Commands --------------------

    public static void neutral() {
        scheduleIfNotRunning(launcher.runVoltage(() -> 0).withName("Launcher.neutral"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(launcher.coastMode());
        scheduleIfNotRunning(launcher.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(launcher.ensureBrakeMode());
        scheduleIfNotRunning(launcher.ensureBrakeMode());
    }

    public static void aimAtHub() {
        scheduleIfNotRunning(launcher.trackTargetCommand().withName("Launcher.aimAtHub"));
        scheduleIfNotRunning(launcher.trackTargetCommand().withName("Launcher.aimAtHub"));
    }

    // --------------------------------------------------------

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the launcher subsystem only if it's not already the running command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(launcher);
        Command current = commandScheduler.requiring(launcher);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
