package frc.robot.fuelIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class FuelIntakeStates {
    private static FuelIntake intake = Robot.getFuelIntake();
    private static FuelIntake.FuelIntakeConfig config = Robot.getConfig().fuelIntake;

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(
                intake.stopMotor().ignoringDisable(true).withName("Intake.default"));
    }

    public static void neutral() {
        scheduleIfNotRunning(intake.runVoltage(() -> 0).withName("Intake.neutral"));
    }

    public static void intakeFuel() {
        scheduleIfNotRunning(
                intake.runTorqueFOC(config.getIntakeTorqueCurrent()).withName("Intake.intakeFuel"));
    }

    public static void slowIntakeFuel() {
        scheduleIfNotRunning(
                intake.runTorqueFOC(config::getFuelSlowIntakeTorqueCurrent)
                        .withName("Intake.slowIntakeFuel"));
    }

    public static void agitateFuel() {
        scheduleIfNotRunning(
                Commands.repeatingSequence(
                                intake.runTorqueFOC(config::getFuelAgitationTorqueCurrent),
                                Commands.waitSeconds(0.5))
                        .withName("Intake.agitate"));
    }

    public static Command ejectCommand() {
        return intake.runTorqueCurrentFoc(config::getEjectTorqueCurrent).withName("Intake.eject");
    }

    public static void stop() {
        scheduleIfNotRunning(intake.stopMotor().withName("Intake.stop"));
    }

    public static void coastMode() {
        scheduleIfNotRunning(intake.coastMode());
    }

    public static void ensureBrakeMode() {
        scheduleIfNotRunning(intake.ensureBrakeMode());
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the fuel intake subsystem only if it's not already the running
     * command
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
