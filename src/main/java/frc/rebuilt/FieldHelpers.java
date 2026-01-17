package frc.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.rebuilt.offsets.HomeOffsets;

public class FieldHelpers {

    private static Zones zones = new Zones();
    private static final HomeOffsets offsets = new HomeOffsets();

    // -----------------------------------------------------------------------
    // Field Helper Methods
    // -----------------------------------------------------------------------

    /* Methods to flip robot pose */

    public static double flipAngle(double angle) {
        return (angle + 180) % 360;
    }

    public static Rotation2d flipAngle(Rotation2d angle) {
        return angle.rotateBy(Rotation2d.fromDegrees(180));
    }

    public static double flipAngleIfRed(double blue) {
        if (Field.isRed()) {
            return (blue + 180) % 360;
        }
        return blue;
    }

    public static Rotation2d flipAngleIfRed(Rotation2d blue) {
        if (Field.isRed()) {
            return blue.rotateBy(Rotation2d.fromDegrees(180));
        }
        return blue;
    }

    public static Translation2d flipIfRed(Translation2d blue) {
        return new Translation2d(flipXifRed(blue.getX()), flipYifRed(blue.getY()));
    }

    public static Translation3d flipIfRed(Translation3d blue) {
        return new Translation3d(flipXifRed(blue.getX()), flipYifRed(blue.getY()), blue.getZ());
    }

    public static Pose2d flipIfRed(Pose2d red) {
        return new Pose2d(flipIfRed(red.getTranslation()), flipAngleIfRed(red.getRotation()));
    }

    public static Translation2d flipIfRedSide(Translation2d red) {
        if (Zones.blueFieldSide.getAsBoolean()) {
            return red;
        }
        return new Translation2d(flipX(red.getX()), flipY(red.getY()));
    }

    public static Pose2d flipIfRedSide(Pose2d red) {
        if (Zones.blueFieldSide.getAsBoolean()) {
            return red;
        }
        return new Pose2d(
                flipIfRedSide(new Translation2d(red.getX(), red.getY())),
                flipAngle(red.getRotation()));
    }

    public static double flipX(double xCoordinate) {
        return Field.fieldLength - xCoordinate;
    }

    public static double flipY(double yCoordinate) {
        return Field.fieldWidth - yCoordinate;
    }

    // If we are red flip the x pose to the other side of the field
    public static double flipXifRed(double xCoordinate) {
        if (Field.isRed()) {
            return Field.fieldLength - xCoordinate;
        }
        return xCoordinate;
    }

    // If we are red flip the y pose to the other side of the field
    public static double flipYifRed(double yCoordinate) {
        if (Field.isRed()) {
            return Field.fieldWidth - yCoordinate;
        }
        return yCoordinate;
    }

    /**
     * Normalizes an angle to the range [-π, π).
     *
     * @param angle The angle in radians.
     * @return The normalized angle.
     */
    public static double normalizeAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle >= Math.PI) angle -= 2 * Math.PI;
        if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static boolean poseOutOfField(Pose2d pose2D) {
        double x = pose2D.getX();
        double y = pose2D.getY();
        return (x <= 0 || x >= Field.fieldLength) || (y <= 0 || y >= Field.fieldWidth);
    }

    public static boolean poseOutOfField(Pose3d pose3D) {
        return poseOutOfField(pose3D.toPose2d());
    }
}
