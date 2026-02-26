package frc.robot.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class ClimbStates {
    private static Climb climb = Robot.getClimb();

    public static void climbForward() {
        scheduleIfNotRunning(climb.runDutyCycleOut(() -> 0.9).withName("climb.climbForward"));
    }

    public static void climbBackward() {
        scheduleIfNotRunning(climb.runDutyCycleOut(() -> -0.25).withName("climb.climbBackward"));
    }

    public static void stop() {
        scheduleIfNotRunning(climb.stopMotor());
    }

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
        Command current = commandScheduler.requiring(climb);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
