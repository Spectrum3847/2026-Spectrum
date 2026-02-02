package frc.robot.launcher;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class LauncherStates {
    private static Launcher launcher = Robot.getLauncher();
    private static Launcher.LauncherConfig config = Robot.getConfig().launcher;
    public static final InterpolatingDoubleTreeMap DISTANCE_MAP = new InterpolatingDoubleTreeMap();

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(
                launcher.stopMotor().ignoringDisable(true).withName("IndexerBackward.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(launcher.runVoltage(() -> 0).withName("IndexerBackward.neutral"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(launcher.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(launcher.ensureBrakeMode());
    }

    public static void launch() {
        scheduleIfNotRunning(launcher.runVelocityTcFocRpm(config::getAMshooterRPM)
                .withName("Launcher.launch"));
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for a subsystem only if it's not already the running
     * command
     *
     * @param subsystem the subsystem the command requires
     * @param command   the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(launcher);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}