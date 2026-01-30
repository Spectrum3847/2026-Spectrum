package frc.robot.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.LarsonAnimation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.pilot.PilotStates;
import frc.spectrumLib.leds.SpectrumLEDs2;
import frc.spectrumLib.util.Util;

public class LedStates {
        private static LedFull leds = Robot.getLeds();
        private static LedRight right = leds.getRight();
        private static LedLeft left = leds.getLeft();

        public static void bindTriggers() {
                disabledPattern(Util.disabled.and(Util.dsAttached));
                teleopPattern(Util.teleop.and(Util.dsAttached));
                autoPattern(Util.autoMode.and(Util.dsAttached));
                testModePattern(Util.testMode.and(Util.dsAttached));

                // General Led Commands
                testPattern(PilotStates.buttonAPress.and(Util.teleop), 5);
        }

        /** Default LED commands for each mode */
        private static Trigger ledDefaultCommand(
                        String name, SpectrumLEDs2 sLeds, ControlRequest pattern, Trigger trigger) {
                int priority = -1;
                return trigger.and(sLeds.checkPriority(priority), sLeds.defaultTrigger)
                                .onTrue(sLeds.setPattern(pattern, priority));
        }

        static void disabledPattern(Trigger trigger) {
                // ombre(purple, white) -> SingleFade(purple, 0.5)
                ledDefaultCommand(
                                "right.disabledPattern", right, right.singleFade(new Color(130, 103, 185), 0.5),
                                trigger);
                ledDefaultCommand(
                                "left.disabledPattern", left, left.singleFade(new Color(130, 103, 185), 0.5), trigger);
        }

        static void teleopPattern(Trigger trigger) {
                // bounce(purple, 3) -> Strobe(purple, 0.5) [Fallback due to Larson compilation
                // issue]
                ledDefaultCommand("right.teleopPattern", right,
                                right.strobe(new Color(130, 103, 185), 0.5),
                                trigger);
                ledDefaultCommand("left.teleopPattern", left,
                                left.strobe(new Color(130, 103, 185), 0.5), trigger);
        }

        static void autoPattern(Trigger trigger) {
                // countdown -> Strobe(Green, 1.0)
                ledDefaultCommand(
                                "right.autoPattern", right, right.strobe(Color.kGreen, 1.0), trigger);

                ledDefaultCommand(
                                "left.autoPattern", left, left.strobe(Color.kGreen, 1.0), trigger);
        }

        static void testModePattern(Trigger trigger) {
                // chase(Red, 0.2, 1) -> Strobe(Red, 0.5) [Fallback]
                ledDefaultCommand("right.testModePattern", right,
                                right.strobe(Color.kRed, 0.5), trigger);
                ledDefaultCommand("left.testModePattern", left,
                                left.strobe(Color.kRed, 0.5),
                                trigger);
        }

        /**
         * LED non-default Commands, set the priority value to see which command takes
         * precedence
         */
        private static Trigger ledCommand(
                        String name, SpectrumLEDs2 sLed, ControlRequest pattern, int priority, Trigger trigger) {
                return trigger.and(sLed.checkPriority(priority))
                                .whileTrue((sLed.setPattern(pattern, priority).withName(name)));
        }

        static void testPattern(Trigger trigger, int priority) {
                // switchCountdown(Blue) -> Strobe(Blue, 0.2)
                ledCommand(
                                "right.testPattern", right, right.strobe(Color.kBlue, 0.2), priority, trigger);
                ledCommand(
                                "left.testPattern", left, left.strobe(Color.kBlue, 0.2), priority, trigger);
        }
}