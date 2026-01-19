package frc.robot.turret;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.turret.Turret.TurretConfig;
import frc.spectrumLib.Telemetry;

public class TurretStates {
    private static Turret turret = Robot.getTurret();
    private static TurretConfig config = Robot.getConfig().turret;

    public static void setupDefaultCommand() {
        turret.setDefaultCommand(log(turret.runHoldTurret().withName("Turret.default")));
        // turret.runStop());
    }

    // -------------------- State Commands --------------------
   
    public static void moveTo270() {
        scheduleIfNotRunning(
                log(
                        turret
                                .moveToDegrees(() -> 270)
                                .withName("Turret.moveTo270")));
    }
    
    public static void moveTo180() {
        scheduleIfNotRunning(
                log(
                        turret
                                .moveToDegrees(() -> 180)
                                .withName("Turret.moveTo180")));
    }

    public static void moveTo90() {
        scheduleIfNotRunning(
                log(
                        turret
                                .moveToDegrees(() -> 90)
                                .withName("Turret.moveTo90")));
    }

    public static void moveTo0() {
        scheduleIfNotRunning(
                log(
                        turret
                                .moveToDegrees(() -> 0)
                                .withName("Turret.moveTo0")));
    }

    public static void holdRotation() {
        scheduleIfNotRunning(
                log(
                        turret
                                .moveToDegrees(() -> Robot.getSwerve().getRobotPose().getRotation().getDegrees())
                                .withName("Turret.holdRotation")));
    }

    public static void neutral() {
        scheduleIfNotRunning(log(turret.runVoltage(() -> 0)));
    }

    // --------------------------------------------------------
    
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
        Command current = commandScheduler.requiring(turret);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
