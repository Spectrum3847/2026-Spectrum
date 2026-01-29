package frc.robot.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class IndexerStates {
    private static Indexer indexer = Robot.getIndexer();
    private static Indexer.IndexerConfig config = Robot.getConfig().indexer;

    public static void setupDefaultCommand() {
        indexer.setDefaultCommand(
                indexer.stopMotor().ignoringDisable(true).withName("Indexer.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(indexer.runVoltage(() -> 0).withName("Indexer.neutral"));
    }

    public static void indexMax() {
        scheduleIfNotRunning(indexer.runTorqueFOC(config::getIndexerTorqueCurrent)
            .withName("Indexer.feedMax"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(indexer.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(indexer.ensureBrakeMode());
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the indexer subsystem only if it's not already the running command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(indexer);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
