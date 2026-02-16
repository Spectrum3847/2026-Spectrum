package frc.robot.indexerBed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class IndexerBedStates {
    private static IndexerBed IndexerBed = Robot.getIndexerBed();
    private static IndexerBed.IndexerBedConfig config = Robot.getConfig().indexerBed;

    public static void setupDefaultCommand() {
        IndexerBed.setDefaultCommand(
                IndexerBed.stopMotor().ignoringDisable(true).withName("IndexerBed.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(IndexerBed.runVoltage(() -> 0).withName("IndexerBed.neutral"));
    }

    public static void indexMax() {
        scheduleIfNotRunning(IndexerBed.runTorqueFOC(config::getIndexerTorqueCurrent)
                .withName("IndexerBed.feedMax"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(IndexerBed.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(IndexerBed.ensureBrakeMode());
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the indexer subsystem only if it's not already the
     * running command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(IndexerBed);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
