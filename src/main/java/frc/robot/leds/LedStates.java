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

public class LedStates extends LedCANdle {
    private static LedFull leds = Robot.getLeds();
    private static LedRight right = leds.getRight();
    private static LedLeft left = leds.getLeft();

    static double currentMatchTime = DriverStation.getMatchTime();

    static void bindTriggers() {
        // disabledPattern(Util.disabled.and(Util.dsAttached));
        teleopPattern(Util.teleop.and(Util.dsAttached));
        autoPattern(Util.autoMode.and(Util.dsAttached));
        testModePattern(Util.testMode.and(Util.dsAttached));

        // Match time related patterns
        redAlliance(Util.teleop.and(() -> !ShiftHelpers.isCurrentShiftBlue(currentMatchTime)), 10);
        blueAlliance(Util.teleop.and(() -> ShiftHelpers.isCurrentShiftBlue(currentMatchTime)), 10);
        blinkyBlonk(
                Util.teleop.and(() -> {
                    double t = DriverStation.getMatchTime();
                    return (t <= 108 && t >= 102) || (t <= 83 && t >= 77) || (t <= 58 && t >= 52);
                }),
                25);
        shift(Util.teleop.and(() -> DriverStation.getMatchTime() <= 140 && DriverStation.getMatchTime() >= 130), 15);
        endgame(Util.teleop.and(() -> DriverStation.getMatchTime() <= 30), 20);
    }

    static void shift(Trigger trigger, int priority) {
        SingleFadeAnimation shiftAnimationRight = new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
            .withSlot(0)
            .withColor(ShiftHelpers.blueWonAuto() ? new RGBWColor(0, 0, 255) : new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(shiftAnimationRight)));
        SingleFadeAnimation shiftAnimationLeft = new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
            .withSlot(1)
            .withColor(ShiftHelpers.blueWonAuto() ? new RGBWColor(0, 0, 255) : new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(shiftAnimationLeft)));
    }

    static void redAlliance(Trigger trigger, int priority) {
        SingleFadeAnimation redAllianceShiftRight = new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
            .withSlot(0)
            .withColor(new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(redAllianceShiftRight)));
        SingleFadeAnimation redAllianceShiftLeft = new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
            .withSlot(1)
            .withColor(new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(redAllianceShiftLeft)));
    }

    static void blueAlliance(Trigger trigger, int priority) {
        SingleFadeAnimation blueAllianceShiftRight = new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
            .withSlot(0)
            .withColor(new RGBWColor(0, 0, 255));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(blueAllianceShiftRight)));
        SingleFadeAnimation blueAllianceShiftLeft = new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
            .withSlot(1)
            .withColor(new RGBWColor(0, 0, 255));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(blueAllianceShiftLeft)));
    }

    // shows upcoming alliance shift 3 seconds before and 3 seconds after
    static void blinkyBlonk(Trigger trigger, int priority) {
        StrobeAnimation aboutToShiftRight = new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx)
            .withSlot(0)
            .withColor(!ShiftHelpers.isCurrentShiftBlue(currentMatchTime) ? new RGBWColor(0, 0, 255) : new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(aboutToShiftRight)));
        StrobeAnimation aboutToShiftLeft = new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
            .withSlot(1)
            .withColor(!ShiftHelpers.isCurrentShiftBlue(currentMatchTime) ? new RGBWColor(0, 0, 255) : new RGBWColor(255, 0, 0));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(aboutToShiftLeft)));
    }

    static void endgame(Trigger trigger, int priority) {
        SingleFadeAnimation endgameAnimationRight = new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
                .withSlot(0)
                .withColor(new RGBWColor(207, 255, 4));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(endgameAnimationRight)));
        SingleFadeAnimation endgameAnimationLeft = new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                .withSlot(1)
                .withColor(new RGBWColor(207, 255, 4));
        trigger.onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> candle.setControl(endgameAnimationLeft)));
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

    static void betterDisabledPattern(Trigger trigger, int priority) {
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
    // private static Trigger ledCommand(
    //         String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
    //     return trigger.and(sLed.checkPriority(priority))
    //             .whileTrue((sLed.setPattern(pattern, priority).withName(name)));
    // }
}