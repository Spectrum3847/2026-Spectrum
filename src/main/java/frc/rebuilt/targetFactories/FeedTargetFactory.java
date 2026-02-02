package frc.rebuilt.targetFactories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.rebuilt.Field;
import frc.robot.Robot;
import frc.robot.swerve.Swerve;

public class FeedTargetFactory {

    private static final Swerve swerve = Robot.getSwerve();

    static InterpolatingTreeMap<Double, Double> distanceOffsetMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static {
        distanceOffsetMap.put(0.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(0);

    public static Translation2d generate() {
        boolean inFieldLeft = swerve.inFieldLeft().getAsBoolean();
        Translation2d feedTarget;

        if (inFieldLeft) {
            feedTarget = Field.isBlue() ? Field.LeftBlueBump.centerPose
                    : Field.BlueToRed(Field.LeftBlueBump.centerPose);
        } else {
            feedTarget = Field.isBlue() ? Field.RightBlueBump.centerPose
                    : Field.BlueToRed(Field.RightBlueBump.centerPose);
        }

        double distance = new Translation2d(feedTarget.getX(), feedTarget.getY()).getDistance(
                Robot.getSwerve().getRobotPose().getTranslation());

        double distanceOffset = distanceOffsetMap.get(distance);
        // Do math in blue alliance, we flip for red.
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (Field.isRed()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        feedTarget = new Translation2d(
                feedTarget.getX() + offSet.getX(), 
                feedTarget.getY() + offSet.getY());
        return feedTarget;
    }
}
