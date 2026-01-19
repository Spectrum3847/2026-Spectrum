package frc.robot.indexerForward;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class IndexerForwardStates {
    private static IndexerForward intake = Robot.getIndexerForward();
    private static IndexerForward.IndexerForwardConfig config = Robot.getConfig().indexerForward;

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(
                intake.stopMotor().ignoringDisable(true).withName("IndexerForward.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(intake.runVoltage(() -> 0).withName("IndexerForward.neutral"));
    }

    public static void spinMax() {
        scheduleIfNotRunning(intake.runTorqueFOC(config::getIndexerForwardTorqueCurrent)
                .withName("IndexerForward.spinMax"));
    }

    public static Command spinMaxComm() {
        return intake.cycleOut(() -> 6)
                .withName("IndexerForward.spinMaxComm");
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
     * Schedules a command for a subsystem only if it's not already the running
     * command
     *
     * @param subsystem the subsystem the command requires
     * @param command   the command to schedule
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