// package frc.robot.leds;

// import com.ctre.phoenix6.controls.SingleFadeAnimation;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.RGBWColor;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.rebuilt.Field;
// import frc.rebuilt.ShiftHelpers;
// import frc.robot.Robot;
// import frc.spectrumLib.util.Util;

// public class LedStates {
//     private static CANdleLeds leds = Robot.getLeds();
//     private static CANdle candle = leds.getCANdle();

//     public static final Trigger auto = Util.autoMode;

//     // period between end of auto and first alliance shift
//     public static final Trigger transitionShift =
//             new Trigger(
//                             () -> {
//                                 double t = DriverStation.getMatchTime();
//                                 return (t <= 140 && t > 133);
//                             })
//                     .and(Util.teleop);

//     public static final Trigger endgame =
//             new Trigger(() -> DriverStation.getMatchTime() <= 30).and(Util.teleop);

//     // 3 seconds before the end of each shift
//     public static final Trigger aboutToChangeShift =
//             new Trigger(
//                             () -> {
//                                 double t = DriverStation.getMatchTime();
//                                 return (t <= 108 && t >= 105)
//                                         || (t <= 83 && t >= 80)
//                                         || (t <= 58 && t >= 55)
//                                         || (t <= 33 && t >= 30);
//                             })
//                     .and(Util.teleop);

//     public static final Trigger transitionAboutToEnd =
//             new Trigger(
//                             () -> {
//                                 double t = DriverStation.getMatchTime();
//                                 return (t <= 133 && t > 130);
//                             })
//                     .and(Util.teleop);

//     public static final Trigger blueShift =
//             new Trigger(() -> ShiftHelpers.isCurrentShiftBlue(DriverStation.getMatchTime()))
//                     .and(Util.teleop);
//     public static final Trigger redShift =
//             new Trigger(() -> ShiftHelpers.isCurrentShiftRed(DriverStation.getMatchTime()))
//                     .and(Util.teleop);

//     public static Trigger bothInShift = auto.or(transitionShift, endgame);

//     public static void setDefaultCommand() {}

//     static void bindTriggers() {

//         // Match time related patterns
//         autoShift(auto, 15);
//         afterAutoTransition(transitionShift, 15);
//         transitionAboutToEnd(transitionAboutToEnd, 25);
//         redAlliance(redShift.and(bothInShift.not()), 10);
//         shiftAboutToEnd(aboutToChangeShift, 25);
//         blueAlliance(blueShift.and(bothInShift.not()), 10);
//         endgame(endgame, 20);
//     }

//     static void autoShift(Trigger trigger, int priority) {
//         SingleFadeAnimation shiftAnimation =
//                 new SingleFadeAnimation(0, 20)
//                         .withSlot(0)
//                         .withColor(
//                                 Field.isBlue()
//                                         ? new RGBWColor(0, 0, 255)
//                                         : new RGBWColor(255, 0, 0));
//         trigger.onTrue(Commands.runOnce() -> candle.setControl(shiftAnimation)));
//     }

//     static void afterAutoTransition(Trigger trigger, int priority) {
//         SingleFadeAnimation shiftAnimation =
//                 new SingleFadeAnimation(0, 20)
//                         .withSlot(0)
//                         .withColor(
//                                 Field.isBlue()
//                                         ? new RGBWColor(0, 0, 255)
//                                         : new RGBWColor(255, 0, 0));
//         trigger.onTrue(Commands.runOnce() -> candle.setControl(shiftAnimation)));
//     }

//     static void redAlliance(Trigger trigger, int priority) {
//         SingleFadeAnimation redAllianceShift =
//                 new SingleFadeAnimation(0, 20).withSlot(0).withColor(new RGBWColor(255, 0, 0));
//         trigger.onTrue(Commands.runOnce() -> candle.setControl(redAllianceShift)));
//     }

//     static void blueAlliance(Trigger trigger, int priority) {
//         SingleFadeAnimation blueAllianceShift =
//                 new SingleFadeAnimation(0, 20).withSlot(0).withColor(new RGBWColor(0, 0, 255));
//         trigger.onTrue(Commands.runOnce() -> candle.setControl(blueAllianceShift)));
//     }

//     static void shiftAboutToEnd(Trigger trigger, int priority) {
//         StrobeAnimation aboutToShift =
//                 new StrobeAnimation(0, 20)
//                         .withSlot(0)
//                         .withColor(
//                                 ShiftHelpers.isCurrentShiftBlue(DriverStation.getMatchTime())
//                                         ? new RGBWColor(0, 0, 255)
//                                         : new RGBWColor(255, 0, 0));
//         trigger.onTrue(Commands.runOnce() -> candle.setControl(aboutToShift)));
//     }

//     static void transitionAboutToEnd(Trigger trigger, int priority) {
//         StrobeAnimation aboutToShift =
//                 new StrobeAnimation(0, 20)
//                         .withSlot(0)
//                         .withColor(
//                                 Field.isBlue()
//                                         ? new RGBWColor(0, 0, 255)
//                                         : new RGBWColor(255, 0, 0));
//         trigger.onTrue(Commands.runOnce() -> candle.setControl(aboutToShift)));
//     }

//     static void endgame(Trigger trigger, int priority) {
//         SingleFadeAnimation endgameAnimation =
//                 new SingleFadeAnimation(0, 20).withSlot(0).withColor(new RGBWColor(207, 255, 4));
//         trigger.onTrue(Commands.runOnce() -> candle.setControl(endgameAnimation)));
//     }
// }
