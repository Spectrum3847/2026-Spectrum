package frc.robot.indexerTower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class IndexerTowerStates {
    private static IndexerTower indexerTower = Robot.getIndexerTower();
    private static IndexerTower.IndexerTowerConfig config = Robot.getConfig().indexerTower;

    public static void setupDefaultCommand() {
        indexerTower.setDefaultCommand(
                indexerTower.stopMotor().ignoringDisable(true).withName("IndexerTower.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(indexerTower.runVoltage(() -> 0).withName("IndexerTower.neutral"));
    }

    public static void indexMax() {
        scheduleIfNotRunning(
                indexerTower
                        .runVelocityTcFocRpm(config::getIndexerVelocityRPM)
                        .withName("IndexerTower.feedMax"));
    }

    public static void slowIndex() {
        scheduleIfNotRunning(
                indexerTower
                        .runVelocityTcFocRpm(config::getIndexerSlowVelocityRPM)
                        .withName("IndexerTower.slowFeed"));
    }

    public static void quickReverseThenIndex() {
        scheduleIfNotRunning(
                Commands.sequence(
                        indexerTower.runVelocityTcFocRpm(config::getIndexerUnjamRPM).withTimeout(1),
                        indexerTower.runVelocityTcFocRpm(config::getIndexerVelocityRPM)));
    }

    public static void unjam() {
        scheduleIfNotRunning(indexerTower.runVelocityTcFocRpm(config::getIndexerUnjamRPM));
    }

    public static void coastMode() {
        scheduleIfNotRunning(indexerTower.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(indexerTower.ensureBrakeMode());
    }

    public static Command unjamCommand() {
        return indexerTower.runVelocityTcFocRpm(config::getIndexerUnjamRPM);
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
        Command current = commandScheduler.requiring(indexerTower);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
