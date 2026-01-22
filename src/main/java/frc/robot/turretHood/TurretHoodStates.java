package frc.robot.turretHood;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.turretHood.TurretHood.TurretHoodConfig;
import frc.spectrumLib.Telemetry;

public class TurretHoodStates {
    private static TurretHood turretHood = Robot.getHood();
    private static TurretHoodConfig config = Robot.getConfig().turretHood;

    public static void setupDefaultCommand() {
        turretHood.setDefaultCommand(log(turretHood.runHoldHood().withName("TurretHood.default")));
    }

    // -------------------- State Commands --------------------
   
    public static void hoodUp() {
        log(turretHood.moveToDegrees(config::getMinRotations)).withName("TurretHood.hoodUp");
    }

    public static Command hoodDownComm() {
        return turretHood.moveToDegrees(config::getMinRotations).withName("TurretHood.hoodUPs");
    }

    public static Command hoodUpComm() {
        return turretHood.moveToDegrees(config::getMaxRotations).withName("TurretHood.hoodDOWNs");
    }

    public static void hoodDown() {
        log(turretHood.moveToDegrees(config::getMaxRotations)).withName("TurretHood.hoodDown");
    }

    public static void neutral() {
        scheduleIfNotRunning(turretHood.runVoltage(() -> 0).withName("TurretHood.neutral"));
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
        Command current = commandScheduler.requiring(turretHood);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
