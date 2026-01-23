package frc.robot.turretRotationalPivot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.turretRotationalPivot.RotationalPivot.RotationalPivotConfig;
import frc.spectrumLib.Telemetry;

public class RotationalPivotStates {
    private static RotationalPivot turretRotation = Robot.getTurret();
    private static RotationalPivotConfig config = Robot.getConfig().turret;

    public static void setupDefaultCommand() {
        turretRotation.setDefaultCommand(log(turretRotation.runHoldTurret().withName("Turret.default")));
        // turret.runStop());
    }

    // -------------------- State Commands --------------------
   
    public static void moveTo270() {
        scheduleIfNotRunning(
                log(
                        turretRotation
                                .moveToDegrees(() -> 270)
                                .withName("Turret.moveTo270")));
    }
    
    public static void moveTo180() {
        scheduleIfNotRunning(
                log(
                        turretRotation
                                .moveToDegrees(() -> 180)
                                .withName("Turret.moveTo180")));
    }

    public static void moveTo90() {
        scheduleIfNotRunning(
                log(
                        turretRotation
                                .moveToDegrees(() -> 90)
                                .withName("Turret.moveTo90")));
    }

    public static void moveTo0() {
        scheduleIfNotRunning(
                log(
                        turretRotation
                                .moveToDegrees(() -> 0)
                                .withName("Turret.moveTo0")));
    }

    public static void holdRotation() {
        scheduleIfNotRunning(
                log(
                        turretRotation
                                .moveToDegrees(() -> Robot.getSwerve().getRobotPose().getRotation().getDegrees())
                                .withName("Turret.holdRotation")));
    }

    public static void neutral() {
        scheduleIfNotRunning(turretRotation.runVoltage(() -> 0).withName("Turret.neutral"));
    }

    // --------------------------------------------------------
    
    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the rotational pivot subsystem only if it's not already the running
     * command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(turretRotation);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
