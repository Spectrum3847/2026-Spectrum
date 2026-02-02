package frc.robot.fuelIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class FuelIntakeStates {
    private static FuelIntake intake = Robot.getFuelIntake();

    public static Command intakeFuelComm() {
        return
            intake.runIntakeOut(() -> -1)
            .withName("Intake.intakeFuel");
    }

    public static void intakeFuel() {
        scheduleIfNotRunning(intake.runDutyCycleOut(() -> -0.55)
        .withName("Intake.intakeFuel"));
    }

    public static void stop() {
        scheduleIfNotRunning(intake.stopMotor());
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the fuel intake subsystem only if it's not already the running command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(intake);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
