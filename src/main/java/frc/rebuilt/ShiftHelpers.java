package frc.rebuilt;

import edu.wpi.first.wpilibj.DriverStation;

public class ShiftHelpers {

    public static boolean blueWonAuto() {
        String matchInfo = DriverStation.getGameSpecificMessage();
        if (matchInfo != null && matchInfo.length() > 0) {
            return matchInfo.charAt(0) == 'B';
        }
        // Safe default if data isn't ready yet
        return false;
    }

    public static int timeLeftInShiftSeconds(double currentMatchTime) {
        if (currentMatchTime >= 130) {
            return (int) (currentMatchTime - 130);
        } else if (currentMatchTime >= 105 && currentMatchTime <= 130) {
            return (int) (currentMatchTime - 105);
        } else if (currentMatchTime >= 80 && currentMatchTime <= 105) {
            return (int) (currentMatchTime - 80);
        } else if (currentMatchTime >= 55 && currentMatchTime <= 80) {
            return (int) (currentMatchTime - 55);
        } else if (currentMatchTime >= 30 && currentMatchTime <= 55) {
            return (int) (currentMatchTime - 30);
        } else {
            return (int) currentMatchTime;
        }
    }

    public static boolean isCurrentShiftBlue(double currentMatchTime) {
        if (currentMatchTime >= 105 && currentMatchTime <= 130) {
            return blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 80 && currentMatchTime <= 105) {
            return blueWonAuto() ? true : false;
        } else if (currentMatchTime >= 55 && currentMatchTime <= 80) {
            return blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 30 && currentMatchTime <= 55) {
            return blueWonAuto() ? true : false;
        } else {
            return true;
        }
    }

    public static boolean currentShiftIsYours() {
        double currentMatchTime = DriverStation.getMatchTime();
        boolean isBlueShift = isCurrentShiftBlue(currentMatchTime);
        if (Field.isBlue()) {
            return isBlueShift;
        } else {
            return !isBlueShift;
        }
    }
}
