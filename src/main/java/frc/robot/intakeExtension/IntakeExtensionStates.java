package frc.robot.intakeExtension;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;

public class IntakeExtensionStates {
    private static IntakeExtension intakeExtension = Robot.getIntakeExtension();
    private static IntakeExtension.IntakeExtensionConfig config = Robot.getConfig().intakeExtension;

    private static boolean sentOutByIntakeState = false;

    public static void setupDefaultCommand() {
        intakeExtension.setDefaultCommand(
                log(intakeExtension.runHoldIntakeExtension().withName("IntakeExtension.default")));
    }

    public static Command operatorResetIntakeExtension() {
        return new InstantCommand(() -> intakeExtension.resetCurrentPositionToMax());
    }

    // -------------------- State Commands --------------------

    public static void fullExtend() {
        scheduleIfNotRunning(
                intakeExtension
                        .motionMagicPercentMove(config::getFullOut)
                        .withName("IntakeExtension.fullExtend"));
        sentOutByIntakeState = true;
    }

    public static void fullRetract() {
        scheduleIfNotRunning(
                intakeExtension
                        .motionMagicPercentMove(config::getHome)
                        .withName("IntakeExtension.fullRetract"));
    }

    public static void fullExtendConditional() {
        if (sentOutByIntakeState) {
            scheduleIfNotRunning(
                    intakeExtension
                            .motionMagicPercentMove(config::getFullOut)
                            .withName("IntakeExtension.fullExtendConditional"));
        } else {
            neutral();
        }
    }

    public static Command fullExtendCommand() {
        return log(
                intakeExtension
                        .motionMagicPercentMove(config::getFullOut)
                        .withName("IntakeExtension.fullExtendCommand"));
    }

    public static Command fullRetractCommand() {
        return log(
                intakeExtension
                        .motionMagicPercentMove(config::getHome)
                        .withName("IntakeExtension.fullRetractCommand"));
    }

    public static Command slowIntakeCloseCommand() {
        return log(intakeExtension.slowMoveToPercent(config::getSqueeze))
                .withName("IntakeExtension.slowIntakeClose");
    }

    public static Command coastMode() {
        return log(intakeExtension.coastMode().withName("IntakeExtension.coastMode"));
    }

    public static Command brakeMode() {
        return log(intakeExtension.ensureBrakeMode().withName("IntakeExtension.brakeMode"));
    }

    public static void neutral() {
        scheduleIfNotRunning(
                intakeExtension.runVoltage(() -> 0).withName("IntakeExtension.neutral"));
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
