package frc.robot.turret;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class TurretStates {
    private static Turret turret = Robot.getTurret();

    public static void setupDefaultCommand() {
        turret.setDefaultCommand(log(turret.runHoldTurret().withName("Turret.default")));
        // turret.runStop());
    }

    // -------------------- State Commands --------------------

    public static void aimAtHub() {
        scheduleIfNotRunning(
                log(turret.trackTargetCommand()).withName("Turret.aimAtHub"));
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
