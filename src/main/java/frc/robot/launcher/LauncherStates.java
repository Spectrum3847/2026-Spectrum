package frc.robot.launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class LauncherStates {
    private static Launcher intake = Robot.getLauncher();
    private static Launcher.LauncherConfig config = Robot.getConfig().launcher;

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(
                intake.stopMotor().ignoringDisable(true).withName("Launcher.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(intake.runVoltage(() -> 0).withName("Launcher.neutral"));
    }

    public static Command launchFuel() {
        return intake.runTorqueFOC(config::getLauncherTorqueCurrent)
            .withName("Launcher.launchFuelCommand");
    }

    public static void coastMode() {
        scheduleIfNotRunning(intake.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(intake.ensureBrakeMode());
    }

    private static Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return intake.runVoltageCurrentLimits(voltage, supplyCurrent, torqueCurrent);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for a subsystem only if it's not already the running command
     *
     * @param subsystem the subsystem the command requires
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(intake);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
