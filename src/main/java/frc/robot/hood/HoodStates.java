package frc.robot.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class HoodStates {
    private static Hood hood = Robot.getHood();
    private static Hood.HoodConfig config = Robot.getConfig().hood;

    public static void setupDefaultCommand() {
        hood.setDefaultCommand(hood.runHoldHood().ignoringDisable(true).withName("Hood.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(hood.runVoltage(() -> 0).withName("Hood.neutral"));
    }

    public static void home() {
        scheduleIfNotRunning(hood.moveToDegrees(config::getInitPosition).withName("Hood.home"));
    }

    public static void aimAtTarget() {
        scheduleIfNotRunning(hood.trackTargetCommand().withName("Hood.aimAtHub"));
    }

    public static void autonAimAtTarget() {
        scheduleIfNotRunning(
                hood.moveToDegrees(config::getAutoTrenchShot).withName("Hood.autonAimAtTarget"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(hood.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(hood.ensureBrakeMode());
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the hood subsystem only if it's not already the running command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(hood);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
