package frc.robot.indexerTower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class IndexerTowerStates {
    private static IndexerTower indexerTowerFront = Robot.getIndexerTower();
    private static IndexerTower.IndexerTowerConfig frontConfig = Robot.getConfig().indexerTower;
    private static IndexerTowerBack indexerTowerBack = Robot.getIndexerTowerBack();
    private static IndexerTowerBack.IndexerTowerBackConfig backConfig =
            Robot.getConfig().indexerTowerBack;

    public static void setupDefaultCommand() {
        indexerTowerFront.setDefaultCommand(
                indexerTowerFront
                        .stopMotor()
                        .ignoringDisable(true)
                        .alongWith(indexerTowerBack.stopMotor().ignoringDisable(true))
                        .withName("IndexerTower.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(
                indexerTowerFront
                        .runVoltage(() -> 0)
                        .alongWith(indexerTowerBack.runVoltage(() -> 0))
                        .withName("IndexerTower.neutral"));
    }

    public static void indexMax() {
        scheduleIfNotRunning(
                indexerTowerFront
                        .runVoltage(frontConfig::getIndexVoltageOut)
                        .alongWith(
                                indexerTowerBack
                                        .runVoltage(backConfig::getIndexVoltageOut)
                                        .withName("IndexerTower.feedMax")));
    }

    public static void slowIndex() {
        scheduleIfNotRunning(
                indexerTowerFront
                        .runVelocity(frontConfig::getIndexerSlowVelocityRPM)
                        .alongWith(
                                indexerTowerBack
                                        .runVelocity(backConfig::getIndexerSlowVelocityRPM)
                                        .withName("IndexerTower.slowFeed")));
    }

    public static void quickReverseThenIndex() {
        scheduleIfNotRunning(
                Commands.sequence(
                        indexerTowerFront
                                .runVoltage(frontConfig::getUnjamVoltageOut)
                                .withTimeout(1),
                        indexerTowerBack.runVoltage(backConfig::getUnjamVoltageOut).withTimeout(1),
                        indexerTowerFront.runVelocity(frontConfig::getIndexerVelocityRPM),
                        indexerTowerBack.runVelocity(backConfig::getIndexerVelocityRPM)));
    }

    public static void unjam() {
        scheduleIfNotRunning(
                indexerTowerFront
                        .runVoltage(frontConfig::getUnjamVoltageOut)
                        .alongWith(indexerTowerBack.runVoltage(backConfig::getUnjamVoltageOut)));
    }

    public static void coastMode() {
        scheduleIfNotRunning(indexerTowerFront.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(indexerTowerFront.ensureBrakeMode());
    }

    public static Command unjamCommand() {
        return indexerTowerFront.runVelocity(frontConfig::getUnjamVoltageOut);
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
        Command current = commandScheduler.requiring(indexerTowerFront);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
