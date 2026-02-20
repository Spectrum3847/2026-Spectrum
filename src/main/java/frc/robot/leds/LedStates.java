package frc.robot.leds;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotStates;
import frc.rebuilt.ShiftHelpers;
import frc.robot.leds.*;
import frc.robot.pilot.PilotStates;
import frc.spectrumLib.leds.SpectrumLEDs;
import frc.spectrumLib.util.Util;

public class LedStates extends LedCANdle{
    private static LedFull leds = Robot.getLeds();
    private static LedRight right = leds.getRight();
    private static LedLeft left = leds.getLeft();

    static double currentMatchTime = DriverStation.getMatchTime();

    static void bindTriggers() {
        // disabledPattern(Util.disabled.and(Util.dsAttached));
        betterDisabledPattern(Util.disabled);
        teleopPattern(Util.teleop.and(Util.dsAttached));
        autoPattern(Util.autoMode.and(Util.dsAttached));
        testModePattern(Util.testMode.and(Util.dsAttached));

        // Match time related patterns
        redAlliance(Util.teleop.and(() -> !ShiftHelpers.isCurrentShiftBlue(currentMatchTime)), 10);
        blueAlliance(Util.teleop.and(() -> ShiftHelpers.isCurrentShiftBlue(currentMatchTime)), 10);
        // shift();
        endgame(Util.teleop.and(() -> DriverStation.getMatchTime() <= 30), 20);

        // Robot things
        shootingFuel(RobotStates.launchingForLEDS, 12);

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

    static void betterDisabledPattern(Trigger trigger) {
        FireAnimation disabledFire = new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0);
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(disabledFire)));
    }

    static void teleopPattern(Trigger trigger) {
        ledDefaultCommand("right.teleopPattern", right, right.bounce(right.purple, 3), trigger);
        ledDefaultCommand("left.teleopPattern", left, left.bounce(left.purple, 3), trigger);
    }

    static void autoPattern(Trigger trigger) {
        ledDefaultCommand(
                "right.autoPattern", right, right.countdown(Timer::getFPGATimestamp, 15), trigger);

        ledDefaultCommand(
                "left.autoPattern", left, left.countdown(Timer::getFPGATimestamp, 15), trigger);
    }

    static void testModePattern(Trigger trigger) {
        ledDefaultCommand("right.testModePattern", right, right.chase(Color.kRed, 0.2, 1), trigger);
        ledDefaultCommand("left.testModePattern", left, left.chase(Color.kRed, 0.2, 1), trigger);
    }

    /** LED non-default Commands, set the priority value to see which command takes precedence */
    private static Trigger ledCommand(
            String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
        return trigger.and(sLed.checkPriority(priority))
                .whileTrue((sLed.setPattern(pattern, priority).withName(name)));
    }

    static void shift(Trigger trigger, int priority) {
        SingleFadeAnimation shiftAnimation = new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(ShiftHelpers.blueWonAuto() ? new RGBWColor(0, 0, 255) : new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(shiftAnimation)));
    }

    static void redAlliance(Trigger trigger, int priority) {
        SingleFadeAnimation redAllianceShift = new SingleFadeAnimation(kSlot0StartIdx, kSlot0StartIdx).withSlot(0).withColor(new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(redAllianceShift)));
    }

    static void blueAlliance(Trigger trigger, int priority) {
        SingleFadeAnimation blueAllianceShift = new SingleFadeAnimation(kSlot0StartIdx, kSlot0StartIdx).withSlot(0).withColor(new RGBWColor(0, 0, 255));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(blueAllianceShift)));
    }

    static void endgame(Trigger trigger, int priority) {
        SolidColor endgameAnimation = new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(new RGBWColor(207, 255, 4));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(endgameAnimation)));
    }
    
    static void shootingFuel(Trigger trigger, int priority) {
        RainbowAnimation shootingFuelAnimation = new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0);
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(shootingFuelAnimation)));
    }
}

/*  
 * if (robotOnFire) {
 *   LEDS = fire;
 * }
 */