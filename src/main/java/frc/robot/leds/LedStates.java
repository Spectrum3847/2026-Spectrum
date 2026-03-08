package frc.robot.leds;

import frc.robot.Robot;

public class LedStates {
    private static CANdleLeds leds = Robot.getLeds();

    public static void setDefaultCommand() {
        leds.setDefaultCommand(leds.disabledPattern());
    }

    public static void setupStates() {}
}
