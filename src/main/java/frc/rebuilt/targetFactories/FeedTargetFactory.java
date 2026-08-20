package frc.rebuilt.targetFactories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.rebuilt.Field;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

public class FeedTargetFactory {

    private static final Swerve swerve = Robot.getSwerve();
    private static boolean inFieldLeft = true;

    static InterpolatingTreeMap<Double, Double> distanceOffsetMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        distanceOffsetMap.put(0.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(0);
    /** Generate. */
    public static Translation2d generate() {
        inFieldLeft = swerve.inFieldLeft().getAsBoolean();
        boolean inOpposingAllianceZone = swerve.inEnemyAllianceZone().getAsBoolean();
        Translation2d feedTarget;

        if (inOpposingAllianceZone) {
            if (inFieldLeft) {
                feedTarget = Field.isBlue() ? Field.deepFeedBlueLeft : Field.deepFeedRedRight;
            } else {
                feedTarget = Field.isBlue() ? Field.deepFeedBlueRight : Field.deepFeedRedLeft;
            }
        } else {
            if (inFieldLeft) {
                feedTarget = Field.isBlue() ? Field.normalFeedBlueLeft : Field.normalFeedRedRight;
            } else {
                feedTarget = Field.isBlue() ? Field.normalFeedBlueRight : Field.normalFeedRedLeft;
            }
        }

        double distance =
                new Translation2d(feedTarget.getX(), feedTarget.getY())
                        .getDistance(Robot.getSwerve().getRobotPose().getTranslation());

        double distanceOffset = distanceOffsetMap.get(distance);
        // Do math in blue alliance, we flip for red.
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (Field.isRed()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        feedTarget =
                new Translation2d(
                        feedTarget.getX() + offSet.getX(), feedTarget.getY() + offSet.getY());
        return feedTarget;
    }

    public static Command feedLeft() {
        return Commands.runOnce(() -> inFieldLeft = true);
    }

    public static Command feedRight() {
        return Commands.runOnce(() -> inFieldLeft = false);
    }
}
