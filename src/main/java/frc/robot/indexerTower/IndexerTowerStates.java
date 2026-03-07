package frc.robot.indexerTower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.spectrumLib.Telemetry;

public class IndexerTowerStates {
    private static IndexerTower indexerTowerFront = Robot.getIndexerTower();
    private static IndexerTower.IndexerTowerConfig frontConfig = Robot.getConfig().indexerTower;

    public static void setupDefaultCommand() {
        indexerTowerFront.setDefaultCommand(
                indexerTowerFront
                        .stopMotor()
                        .ignoringDisable(true)
                        .withName("IndexerTower.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(
                indexerTowerFront
                        .runVoltage(() -> 0)
                        .withName("IndexerTower.neutral"));
    }

    public static void indexMax() {
        scheduleIfNotRunning(
                indexerTowerFront
                        .runVoltage(frontConfig::getIndexerVoltageOut)
                        .withName("IndexerTower.feedMax"));
    }

    public static void indexIfReady() {
        scheduleIfNotRunning(
                indexerTowerFront
                        .runTorqueCurrentFoc(
                                () ->
                                        RobotStates.turretOnTarget.getAsBoolean()
                                                ? frontConfig.getIndexerTorqueCurrent()
                                                : 0)
                        .withName("IndexerTower.feedIfReady"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(indexerTowerFront.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(indexerTowerFront.ensureBrakeMode());
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
