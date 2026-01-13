package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.util.Util;

public class LedStates {
    private static LedFull leds = Robot.getLeds();
    private static LedRight right = leds.getRight();
    private static LedLeft left = leds.getLeft();

    static void bindTriggers() {
        disabledPattern(Util.disabled.and(Util.dsAttached));
        teleopPattern(Util.teleop.and(Util.dsAttached));
        autoPattern(Util.autoMode.and(Util.dsAttached));
        testModePattern(Util.testMode.and(Util.dsAttached));

        timeLeftInTeleopLED(Util.teleop.and(Util.dsAttached), 10);
        timeLeftInShiftLED(Util.teleop.and(Util.dsAttached), 11);
    }

    /** Default LED commands for each mode */
    private static Trigger ledDefaultCommand(
            String name, SpectrumLEDs sLeds, LEDPattern pattern, Trigger trigger) {
        int priority = -1;
        return trigger.and(sLeds.checkPriority(priority), sLeds.defaultTrigger)
                // .onTrue(sLeds.setPattern(pattern, priority).withName(name));
                .onTrue(sLeds.setPattern(pattern, priority));
    }

    static void disabledPattern(Trigger trigger) {
        ledDefaultCommand(
                "right.disabledPattern", right, right.ombre(right.purple, right.white), trigger);
        ledDefaultCommand(
                "left.disabledPattern", left, left.ombre(left.purple, left.white), trigger);
    }

    static void teleopPattern(Trigger trigger) {
        ledDefaultCommand("right.teleopPattern", right, right.bounce(right.purple, 3), trigger);
        ledDefaultCommand("left.teleopPattern", left, left.bounce(left.purple, 3), trigger);
    }

    static void autoPattern(Trigger trigger) {
        ledDefaultCommand(
                "right.autoPattern", right, right.countdown(Timer::getFPGATimestamp, 20), trigger);

        ledDefaultCommand(
                "left.autoPattern", left, left.countdown(Timer::getFPGATimestamp, 20), trigger);
    }

    static void testModePattern(Trigger trigger) {
        ledDefaultCommand("right.testModePattern", right, right.chase(Color.kRed, 0.2, 1), trigger);
        ledDefaultCommand("left.testModePattern", left, left.chase(Color.kRed, 0.2, 1), trigger);
    }

    /** LED non-default Commands, set the priority value to see which command takes precedence */
    private static Trigger ledCommand(
            String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
        return trigger.and(sLed.checkPriority(priority))
                .whileTrue(log(sLed.setPattern(pattern, priority).withName(name)));
    }

    static void timeLeftInTeleopLED(Trigger trigger, int priority) {
        ledCommand("right.teleopTimeLeft", right, right.countdown(Timer::getFPGATimestamp, 140), priority, trigger);
        ledCommand("left.teleopTimeLeft", left, left.countdown(Timer::getFPGATimestamp, 140), priority, trigger);
    }

    static void timeLeftInShiftLED(Trigger trigger, int priority){
        Color color;

        if (Timer.getMatchTime() > 130 || Timer.getMatchTime() < 30 || Util.autoMode.getAsBoolean()){
            return;
        }


        if (Timer.getMatchTime() > 105){
            color = Color.kRed; // fix to show the current active team instead of assuming red goes first
        }
        else if (Timer.getMatchTime() > 80){
            color = Color.kBlue;
        }
        else if (Timer.getMatchTime() > 55){
            color = Color.kRed;
        }
        else { // if (Timer.getMatchTime() > 30){
            color = Color.kBlue;
        }

        ledCommand("right.shiftTimeLeft", right, right.colorCountdown(color, Timer::getFPGATimestamp, 25), priority, trigger);
        ledCommand("left.shiftTimeLeft", left, left.colorCountdown(color, Timer::getFPGATimestamp, 25), priority, trigger);
    }

    static void homeFinishLED(Trigger trigger, int priority) {
        ledCommand(
                "right.HomeFinish",
                right,
                right.bounce(right.purple, 3)
                        .blend(right.solid(right.purple).atBrightness(Percent.of(75))),
                priority,
                trigger);
        ledCommand(
                "left.HomeFinish",
                left,
                right.bounce(right.purple, 3)
                        .blend(right.solid(right.purple).atBrightness(Percent.of(75))),
                priority,
                trigger);
    }

    static void climbReadyLED(Trigger trigger, int priority) {
        ledCommand("right.ClimbReady", right, right.scrollingRainbow(), priority, trigger);
        ledCommand("left.ClimbReady", left, left.scrollingRainbow(), priority, trigger);
    }
    
    static void seesTagDefaultLED(Trigger trigger, int priority) {
        ledCommand("right.SeesTag", right, right.bounce(Color.kYellow, 3), priority, trigger);
        ledCommand("left.SeesTag", left, left.bounce(Color.kYellow, 3), priority, trigger);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
