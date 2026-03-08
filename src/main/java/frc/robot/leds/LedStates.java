package frc.robot.leds;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.rebuilt.ShiftHelpers;
import frc.spectrumLib.util.Util;

public class LedStates extends LedCANdle {

    static double currentMatchTime = DriverStation.getMatchTime();

    static void bindTriggers() {

        disabledPattern(Util.disabled.and(Util.dsAttached), 10);

        // Match time related patterns
        redAlliance(Util.teleop.and(() -> !ShiftHelpers.isCurrentShiftBlue(currentMatchTime)), 10);
        blueAlliance(Util.teleop.and(() -> ShiftHelpers.isCurrentShiftBlue(currentMatchTime)), 10);
        shiftSwitchBlink(
                Util.teleop.and(
                        () -> {
                            double t = DriverStation.getMatchTime();
                            return (t <= 108 && t >= 102)
                                    || (t <= 83 && t >= 77)
                                    || (t <= 58 && t >= 52);
                        }),
                25);
        shift(
                Util.teleop.and(
                        () ->
                                DriverStation.getMatchTime() <= 140
                                        && DriverStation.getMatchTime() >= 130),
                15);
        endgame(Util.teleop.and(() -> DriverStation.getMatchTime() <= 30), 20);
    }

    static void shift(Trigger trigger, int priority) {
        SingleFadeAnimation shiftAnimationRight =
                new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
                        .withSlot(0)
                        .withColor(
                                ShiftHelpers.blueWonAuto()
                                        ? new RGBWColor(0, 0, 255)
                                        : new RGBWColor(255, 0, 0));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(shiftAnimationRight)));
        SingleFadeAnimation shiftAnimationLeft =
                new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                        .withSlot(1)
                        .withColor(
                                ShiftHelpers.blueWonAuto()
                                        ? new RGBWColor(0, 0, 255)
                                        : new RGBWColor(255, 0, 0));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(shiftAnimationLeft)));
    }

    static void redAlliance(Trigger trigger, int priority) {
        SingleFadeAnimation redAllianceShiftRight =
                new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
                        .withSlot(0)
                        .withColor(new RGBWColor(255, 0, 0));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(redAllianceShiftRight)));
        SingleFadeAnimation redAllianceShiftLeft =
                new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                        .withSlot(1)
                        .withColor(new RGBWColor(255, 0, 0));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(redAllianceShiftLeft)));
    }

    static void blueAlliance(Trigger trigger, int priority) {
        SingleFadeAnimation blueAllianceShiftRight =
                new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
                        .withSlot(0)
                        .withColor(new RGBWColor(0, 0, 255));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(blueAllianceShiftRight)));
        SingleFadeAnimation blueAllianceShiftLeft =
                new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                        .withSlot(1)
                        .withColor(new RGBWColor(0, 0, 255));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(blueAllianceShiftLeft)));
    }

    // shows upcoming alliance shift 3 seconds before and 3 seconds after
    static void shiftSwitchBlink(Trigger trigger, int priority) {
        StrobeAnimation aboutToShiftRight =
                new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx)
                        .withSlot(0)
                        .withColor(
                                !ShiftHelpers.isCurrentShiftBlue(currentMatchTime)
                                        ? new RGBWColor(0, 0, 255)
                                        : new RGBWColor(255, 0, 0));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(aboutToShiftRight)));
        StrobeAnimation aboutToShiftLeft =
                new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                        .withSlot(1)
                        .withColor(
                                !ShiftHelpers.isCurrentShiftBlue(currentMatchTime)
                                        ? new RGBWColor(0, 0, 255)
                                        : new RGBWColor(255, 0, 0));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(aboutToShiftLeft)));
    }

    static void endgame(Trigger trigger, int priority) {
        SingleFadeAnimation endgameAnimationRight =
                new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx)
                        .withSlot(0)
                        .withColor(new RGBWColor(207, 255, 4));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(endgameAnimationRight)));
        SingleFadeAnimation endgameAnimationLeft =
                new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx)
                        .withSlot(1)
                        .withColor(new RGBWColor(207, 255, 4));
        trigger.onTrue(new InstantCommand(() -> candle.setControl(endgameAnimationLeft)));
    }

    static void disabledPattern(Trigger trigger, int priority) {
        FireAnimation disabledFire = new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0);
        trigger.onTrue(new InstantCommand(() -> candle.setControl(disabledFire)));
    }

    /** LED non-default Commands, set the priority value to see which command takes precedence */
    // private static Trigger ledCommand(
    //         String name, SpectrumLEDs sLed, LEDPattern pattern, int priority, Trigger trigger) {
    //     return trigger.and(sLed.checkPriority(priority))
    //             .whileTrue((sLed.setPattern(pattern, priority).withName(name)));
    // }
}
