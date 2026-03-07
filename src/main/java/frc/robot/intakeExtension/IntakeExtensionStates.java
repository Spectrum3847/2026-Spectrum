package frc.robot.intakeExtension;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class IntakeExtensionStates {
    private static IntakeExtension intakeExtension = Robot.getIntakeExtension();

    private static boolean sentOutByIntakeState = false;

    public static void setupDefaultCommand() {
        intakeExtension.setDefaultCommand(
                log(intakeExtension.runHoldIntakeExtension().withName("IntakeExtension.default")));
    }

    public static void operatorResetIntakeExtension() {
        intakeExtension.resetCurrentPositionToMax();
    }

    // -------------------- State Commands --------------------

    public static void fullExtend() {
        scheduleIfNotRunning(
                intakeExtension
                        .voltageOutPositive()
                        .until(intakeExtension.atPercentage(() -> 100, () -> 5).debounce(0.5)));
        sentOutByIntakeState = true;
    }

    public static void fullRetract() {
        scheduleIfNotRunning(
                intakeExtension
                        .voltageOutNegative()
                        .until(intakeExtension.atPercentage(() -> 0, () -> 5).debounce(0.5)));
    }

    public static void fullExtendConditional() {
        if (sentOutByIntakeState) {
            scheduleIfNotRunning(
                    intakeExtension
                            .voltageOutPositive()
                            .until(intakeExtension.atPercentage(() -> 100, () -> 5).debounce(0.5)));
        } else {
            scheduleIfNotRunning(intakeExtension.runVoltage(() -> 0));
        }
    }

    public static void coastMode() {
        scheduleIfNotRunning(intakeExtension.coastMode());
    }

    public static void brakeMode() {
        scheduleIfNotRunning(intakeExtension.ensureBrakeMode());
    }

    public static void neutral() {
        scheduleIfNotRunning(intakeExtension.runVoltage(() -> 0));
    }

    // --------------------------------------------------------

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    /**
     * Schedules a command for the intake extension subsystem only if it's not already the running
     * command
     *
     * @param command the command to schedule
     */
    public static void scheduleIfNotRunning(Command command) {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Check what command is currently requiring this subsystem
        Command current = commandScheduler.requiring(intakeExtension);

        // Only schedule if it's not already the same same command
        if (current != command) {
            commandScheduler.schedule(command);
        }
    }
}
